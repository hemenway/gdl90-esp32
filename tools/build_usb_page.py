"""Bundle the shared panel and its Web Serial transport as a standalone HTML file."""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path, help="destination HTML file")
args = parser.parse_args()
page = (ROOT / "ui/index.html").read_text(encoding="utf-8")
transport = (ROOT / "ui/web-serial.js").read_text(encoding="utf-8")
if page.count("<body>") != 1 or "</script" in transport.lower():
    raise SystemExit("Unable to locate the panel body or inline the USB transport")
dialog = """<dialog id="usbConnectDialog" aria-labelledby="usbTitle"
style="max-width:min(420px,92vw);background:var(--panel);color:var(--text);border:1px solid var(--line);border-radius:10px;padding:24px">
<style>#usbConnectDialog::backdrop { background: rgb(0 10 15 / 84%); }</style>
<h2 id="usbTitle" style="margin:0">Connect the ESP32</h2>
<p id="usbMessage" role="status">Plug in the board with a USB data cable. Select its USB serial port to open the simulator controls.</p>
<button type="button" id="usbConnectButton" class="primary">Connect via USB</button>
</dialog>"""
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(page.replace("<body>", f"<body>\n{dialog}\n<script>\n{transport}\n</script>"), encoding="utf-8")
