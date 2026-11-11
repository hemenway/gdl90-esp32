#!/usr/bin/env python3
"""Control the GDL 90 ESP32 from a web browser over its USB data cable."""
import argparse
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time
import webbrowser
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

INDEX = Path(__file__).resolve().parents[1] / "ui" / "index.html"
MAX_BODY = 12 * 1024
GET_PATHS = {"/api/catalog", "/api/config", "/api/state", "/api/net"}
POST_PATHS = {"/api/config", "/api/action", "/api/net/action"}


def load_serial():
    try:
        import serial
        from serial.tools import list_ports
        return serial, list_ports
    except ImportError:
        pass
    # Keep first-time installation out of the system Python.
    if os.name == "nt":
        cache = Path(os.environ.get("LOCALAPPDATA", Path.home()))
    elif sys.platform == "darwin":
        cache = Path.home() / "Library" / "Caches"
    else:
        cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    env = cache / "gdl90-emulator" / "usb-panel-v1"
    python = env / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if Path(sys.prefix).resolve() == env.resolve():
        raise RuntimeError(f"USB helper is incomplete. Remove {env} and launch again.")
    if not python.exists():
        print("Preparing the USB helper (first launch requires internet)…", flush=True)
        subprocess.run([sys.executable, "-m", "venv", str(env)], check=True)
    if subprocess.run([str(python), "-c", "import serial.tools.list_ports"], stderr=subprocess.DEVNULL).returncode:
        subprocess.run([str(python), "-m", "pip", "install", "--disable-pip-version-check", "pyserial==3.5"], check=True)
    raise SystemExit(subprocess.call([str(python), str(Path(__file__).resolve()), *sys.argv[1:]]))


def choose_port(list_ports, requested):
    if requested:
        return requested
    devices = sorted((p for p in list_ports.comports() if p.vid is not None), key=lambda p: p.device)
    bridges = [p for p in devices if p.vid in (0x10C4, 0x1A86, 0x0403, 0x303A)]
    ports = bridges or devices
    if not ports:
        raise RuntimeError("No USB serial device found. Plug the ESP32 into a data cable and rerun.")
    if len(ports) == 1:
        return ports[0].device
    choices = "\n".join(f"{i}. {p.device} ({p.description})" for i, p in enumerate(ports, 1))
    if not sys.stdin.isatty():
        raise RuntimeError("Multiple devices found. Pass --port DEVICE.\n" + choices)
    print("Choose the ESP32 USB port:\n" + choices)
    index = input("Port number: ").strip()
    if not index.isdigit() or not 1 <= int(index) <= len(ports):
        raise RuntimeError("Invalid port number.")
    return ports[int(index) - 1].device


class Device:
    def __init__(self, serial, name):
        self.serial = serial
        self.name = name
        self.port = None
        self.lock = threading.Lock()

    def open(self):
        port = self.serial.Serial(baudrate=115200, timeout=0.25, write_timeout=3)
        # Set reset and boot lines before opening the USB-UART bridge.
        port.dtr = False
        port.rts = False
        port.port = self.name
        port.open()
        self.port = port

    def close(self):
        if self.port is not None:
            self.port.close()
            self.port = None

    def call(self, method, path, body=None, timeout=12):
        with self.lock:
            if self.port is None:
                self.open()
            request_id = secrets.randbits(32)
            payload = json.dumps({"id": request_id, "method": method, "path": path, "body": body},
                                 separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            if len(payload) > MAX_BODY + 480:
                return 413, b"Request exceeds the USB limit"
            prefix = f"G90! {request_id} ".encode("ascii")
            status = None
            pending, response = bytearray(), bytearray()
            try:
                self.port.write(b"G90? " + payload + b"\n")
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    pending.extend(self.port.read(max(1, min(self.port.in_waiting, 8192))))
                    while b"\n" in pending:
                        line, _, pending = pending.partition(b"\n")
                        line = line.rstrip(b"\r")
                        if not line.startswith(prefix):
                            continue  # ordinary diagnostics or a previous request
                        part = line[len(prefix):]
                        if part == b"END" and status is not None:
                            data = bytes(response)
                            decoded = json.loads(data)
                            if status != 200:
                                data = str(decoded).encode()
                            return status, data
                        code, delim, chunk = part.partition(b" ")
                        if delim and code in (b"200", b"400"):
                            status = int(code)
                            response.extend(chunk)
                    if len(pending) > 8192:
                        pending.clear()
                    if len(response) > 512 * 1024:
                        raise RuntimeError("Board response is too long.")
            except OSError:
                self.close()
                raise
            raise TimeoutError("No response from the ESP32. Check the cable and close any serial monitor.")


class Panel(BaseHTTPRequestHandler):
    def reply(self, status, data, content_type="text/plain; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def dispatch(self):
        suffix = "" if self.server.server_port == 80 else f":{self.server.server_port}"
        authority = self.headers.get("Host", "").lower()
        if authority not in ("127.0.0.1" + suffix, "localhost" + suffix):
            return self.reply(403, b"Use the panel's loopback address.")
        path = urlsplit(self.path).path
        if self.command == "GET" and path == "/usb/status":
            info = {"app": "gdl90-usb-panel", "port": self.server.device.name}
            return self.reply(200, json.dumps(info).encode(), "application/json")
        if self.command == "GET" and path in ("/", "/index.html"):
            return self.reply(200, INDEX.read_bytes(), "text/html; charset=utf-8")
        if path not in (GET_PATHS if self.command == "GET" else POST_PATHS):
            return self.reply(404, b"Unknown panel route")
        if len(self.path) > 256:
            return self.reply(400, b"Request path too long")
        body = None
        if self.command == "POST":
            origin = self.headers.get("Origin")
            if origin is not None and origin != "http://" + authority:
                return self.reply(403, b"Open the control panel to make changes.")
            if self.headers.get_content_type() != "application/json":
                return self.reply(415, b"Expected application/json")
            try:
                n = int(self.headers.get("Content-Length", "-1"))
                if not 0 < n <= MAX_BODY:
                    return self.reply(413, b"Invalid request length")
                body = json.loads(self.rfile.read(n))
            except (ValueError, UnicodeError):
                return self.reply(400, b"Invalid JSON")
        try:
            status, data = self.server.device.call(self.command, self.path, body)
        except (OSError, RuntimeError, ValueError) as err:
            return self.reply(503, str(err).encode())
        return self.reply(status, data, "application/json" if status == 200 else "text/plain; charset=utf-8")

    do_GET = do_POST = dispatch

    def log_message(self, fmt, *args):
        pass  # continuous state polling


def reuse_panel(port, device, no_browser):
    if port == 0:
        return False
    client = HTTPConnection("127.0.0.1", port, timeout=1)
    try:
        client.request("GET", "/usb/status")
        res = client.getresponse()
        if res.status != 200:
            return False
        status = json.loads(res.read(4096))
        if not isinstance(status, dict) or status.get("app") != "gdl90-usb-panel":
            return False
        if device and device != status.get("port"):
            raise RuntimeError("Another board is using this panel port. Choose another --http-port.")
    except (ValueError, OSError, HTTPException):
        return False
    finally:
        client.close()
    url = f"http://127.0.0.1:{port}/"
    print(f"USB panel is already open on {url}")
    if not no_browser:
        webbrowser.open(url)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", metavar="DEVICE", help="USB serial port, e.g. COM4 or /dev/cu.usbserial-0001")
    parser.add_argument("--http-port", default=8090, type=int, help="local browser port (default 8090)")
    parser.add_argument("--no-browser", action="store_true", help="print the URL without opening a browser")
    args = parser.parse_args()
    if reuse_panel(args.http_port, args.port, args.no_browser):
        return
    serial, list_ports = load_serial()
    device = Device(serial, choose_port(list_ports, args.port))
    print(f"Connecting to the ESP32 on {device.name}…", flush=True)
    try:
        # Some USB adapters briefly reset the board when opened. Retry only this read-only check.
        try:
            status, data = device.call("GET", "/api/catalog", timeout=3)
        except TimeoutError:
            status, data = device.call("GET", "/api/catalog", timeout=8)
        if status != 200 or json.loads(data).get("platform") != "esp32":
            raise RuntimeError("This port did not identify as the GDL 90 ESP32 firmware.")
        with ThreadingHTTPServer(("127.0.0.1", args.http_port), Panel) as server:
            server.device = device
            url = f"http://127.0.0.1:{server.server_port}/"
            print(f"USB control panel: {url}\nKeep this window open. Press Ctrl+C to close.", flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            server.serve_forever()
    finally:
        device.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as err:
        sys.exit(str(err))
