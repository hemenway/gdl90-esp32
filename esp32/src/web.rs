//! Browser control panel + JSON API on the ESP32 (same API as the PC app, polled instead of a
//! websocket), and captive-portal redirects.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{anyhow, bail, Result};
use esp_idf_svc::http::server::{Configuration as HttpConfig, EspHttpConnection, EspHttpServer, Request, Response};
use esp_idf_svc::http::Method;
use esp_idf_svc::io::Write;
use serde_json::{json, Value};

use crate::{actions, config, geo, scenarios, Shared};

const INDEX: &str = include_str!("../../ui/index.html");

type Req<'a, 'b> = Request<&'a mut EspHttpConnection<'b>>;

fn send(req: Req, status: u16, ctype: &str, body: &[u8]) -> Result<()> {
    let mut resp = req
        .into_response(status, None, &[("Content-Type", ctype), ("Cache-Control", "no-store")])
        .map_err(|e| anyhow!("{e:?}"))?;
    resp.write_all(body).map_err(|e| anyhow!("{e:?}"))?;
    Ok(())
}

struct HttpBody<'a, 'b, 'c>(&'a mut Response<&'b mut EspHttpConnection<'c>>);

impl std::io::Write for HttpBody<'_, '_, '_> {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        self.0.write(bytes).map_err(|e| std::io::Error::other(format!("{e:?}")))
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn send_json(req: Req, reply: &Payload) -> Result<()> {
    let mut response = req.into_response(
        200, None, &[("Content-Type", "application/json"), ("Cache-Control", "no-store")],
    ).map_err(|e| anyhow!("{e:?}"))?;
    let mut out = std::io::BufWriter::with_capacity(1024, HttpBody(&mut response));
    reply.write_to(&mut out)?;
    std::io::Write::flush(&mut out)?;
    Ok(())
}

fn read_json(req: &mut Req) -> Result<Value> {
    let len: usize = req.header("Content-Length").unwrap_or("0").parse()?;
    if len > MAX_BODY {
        bail!("request too large");
    }
    let mut buf = vec![0u8; len];
    let mut off = 0;
    while off < len {
        let n = req.read(&mut buf[off..]).map_err(|e| anyhow!("{e:?}"))?;
        if n == 0 {
            bail!("incomplete request");
        }
        off += n;
    }
    buf.truncate(off);
    Ok(serde_json::from_slice(&buf)?)
}

fn query(uri: &str, key: &str) -> Option<String> {
    let q = uri.split_once('?')?.1;
    q.split('&').find_map(|kv| {
        let (k, v) = kv.split_once('=')?;
        (k == key).then(|| v.to_string())
    })
}

fn catalog() -> Value {
    json!({
        "profiles": config::PROFILES.iter().map(|p| json!({"id": p.id, "name": p.name, "description": p.description})).collect::<Vec<_>>(),
        "scenarios": scenarios::SCENARIOS.iter().map(|s| json!({"id": s.id, "name": s.name, "description": s.description})).collect::<Vec<_>>(),
        "anchor": scenarios::Anchor::default(),
        "platform": "esp32",
        "features": {"record": false, "replay": false, "hotspot": false, "config_files": false},
    })
}

/// Network/device summary in the same shape the PC app reports (`net`), plus ESP specifics.
fn net_summary(sh: &Shared) -> Value {
    let n = sh.net.lock().unwrap();
    let ap = sh.ap_running.lock().unwrap();
    let has_saved = sh.store.lock().unwrap().has_saved();
    json!({
        "net": {
            "hotspot_iface": {"name": "esp32-ap", "ip": "192.168.4.1", "prefix": 24, "broadcast": "192.168.4.255"},
            "clients": n.clients.values().collect::<Vec<_>>(),
            "targets": n.targets,
            "sent_datagrams": n.sent,
            "send_errors": n.errors,
            "last_error": n.last_error,
            "listeners": n.listeners,
        },
        "ap": {
            "ssid": ap.ssid,
            "open": ap.passphrase.len() < 8,
            "channel": ap.channel,
            "captive_portal": sh.captive.load(Ordering::Relaxed),
            "uptime_s": sh.boot.elapsed().as_secs(),
            "free_heap": unsafe { esp_idf_svc::sys::esp_get_free_heap_size() },
            "min_free_heap": unsafe { esp_idf_svc::sys::esp_get_minimum_free_heap_size() },
            "time_synced": sh.time_synced.load(Ordering::Relaxed),
            "saved_settings": has_saved,
        }
    })
}

fn set_clock(unix: f64) {
    let tv = esp_idf_svc::sys::timeval { tv_sec: unix.floor() as _, tv_usec: ((unix.fract()) * 1e6) as _ };
    unsafe {
        esp_idf_svc::sys::settimeofday(&tv, core::ptr::null());
    }
}

fn platform_action(sh: &Arc<Shared>, v: &Value) -> Result<Value> {
    let t = v.get("type").and_then(|x| x.as_str()).unwrap_or("");
    match t {
        "set_time" => {
            let unix = v.get("unix").and_then(|x| x.as_f64()).ok_or_else(|| anyhow!("unix required"))?;
            let drift = (geo::unix_now_f64() - unix).abs();
            if drift > 1.0 {
                set_clock(unix);
                log::info!("Clock set from browser (was {drift:.0}s off)");
            }
            sh.time_synced.store(true, Ordering::Relaxed);
            Ok(json!({"drift_s": drift}))
        }
        "save_default" => {
            let cfg = sh.engine.lock().unwrap().cfg.clone();
            let n = sh.store.lock().unwrap().save_config(&cfg)?;
            sh.engine.lock().unwrap().notice(format!("Saved as power-on settings ({n} bytes)"));
            Ok(json!({"bytes": n}))
        }
        "clear_default" => {
            sh.store.lock().unwrap().clear()?;
            sh.engine.lock().unwrap().notice("Saved settings cleared; next boot uses defaults");
            Ok(json!({}))
        }
        "reboot" => {
            std::thread::spawn(|| {
                std::thread::sleep(Duration::from_millis(600));
                unsafe { esp_idf_svc::sys::esp_restart() };
            });
            Ok(json!({}))
        }
        other => bail!("unknown action '{other}'"),
    }
}

fn apply_wifi_later(sh: Arc<Shared>, next: config::NetworkConfig) -> Result<()> {
    static APPLYING: AtomicBool = AtomicBool::new(false);
    if APPLYING.swap(true, Ordering::AcqRel) {
        bail!("Wi-Fi is already restarting");
    }
    // Reply before stopping the AP. Keep the previous settings available for recovery.
    let worker = std::thread::Builder::new().stack_size(8192).spawn(move || {
        std::thread::sleep(Duration::from_millis(700));
        let previous = sh.ap_running.lock().unwrap().clone();
        let mut wifi = sh.wifi.lock().unwrap();
        let message = match crate::apply_ap(&mut wifi, &next) {
            Ok(()) => {
                let message = format!("Wi-Fi active: {}", next.ssid);
                *sh.ap_running.lock().unwrap() = next;
                message
            }
            Err(err) => match crate::apply_ap(&mut wifi, &previous) {
                Ok(()) => {
                    let mut engine = sh.engine.lock().unwrap();
                    if engine.cfg.network == next {
                        engine.cfg.network = previous.clone();
                    }
                    format!("Wi-Fi change failed ({err}); restored {}", previous.ssid)
                }
                Err(restore) => format!("Wi-Fi change failed ({err}); recovery failed ({restore}). Use USB to reconfigure."),
            },
        };
        drop(wifi);
        sh.net.lock().unwrap().clients.clear();
        sh.engine.lock().unwrap().notice(message);
        APPLYING.store(false, Ordering::Release);
    });
    if let Err(err) = worker {
        APPLYING.store(false, Ordering::Release);
        return Err(err.into());
    }
    Ok(())
}

pub const MAX_BODY: usize = 12 * 1024;

// ESP32 internal RAM is shared by both control paths. Finish serializing one API result
// before building another large traffic snapshot, even when USB and Wi-Fi clients overlap.
static CONTROL: Mutex<()> = Mutex::new(());

fn api(sh: &Arc<Shared>, method: &str, uri: &str, body: &Value) -> Result<Value> {
    let path = uri.split('?').next().unwrap_or(uri);
    match (method, path) {
        ("GET", "/api/catalog") => Ok(catalog()),
        ("GET", "/api/config") => Ok(serde_json::to_value(&sh.engine.lock().unwrap().cfg)?),
        ("POST", "/api/config") => {
            let mut e = sh.engine.lock().unwrap();
            e.patch_config(body)?;
            Ok(serde_json::to_value(&e.cfg)?)
        }
        ("GET", "/api/net") => Ok(net_summary(sh)),
        ("POST", "/api/action") => {
            let result = actions::handle(&mut sh.engine.lock().unwrap(), body)?;
            let result = match result {
                Some(v) => v,
                None => platform_action(sh, body)?,
            };
            if body["type"] == "reboot" {
                return Ok(json!({"ok": true, "result": result}));
            }
            let mut reply = json!({"ok": true});
            reply["config"] = serde_json::to_value(&sh.engine.lock().unwrap().cfg)?;
            reply["result"] = result;
            Ok(reply)
        }
        ("POST", "/api/net/action") if body["type"] == "apply_wifi" => {
            let n = sh.engine.lock().unwrap().cfg.network.clone();
            let message = format!("Restarting Wi-Fi as '{}'", n.ssid);
            apply_wifi_later(sh.clone(), n)?;
            Ok(json!({"ok": true, "message": message}))
        }
        _ => bail!("unknown control request"),
    }
}

// Fixed-size segments avoid doubling a large contiguous buffer on this heap while encoding
// the traffic list. Only encoded bytes survive the engine lock and cross the slower USB link.
#[derive(Default)]
pub struct Parts(Vec<Vec<u8>>);

impl std::io::Write for Parts {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        const SIZE: usize = 1024;
        if self.0.last().is_none_or(|last| last.len() == SIZE) {
            self.0.push(Vec::with_capacity(SIZE));
        }
        let part = self.0.last_mut().unwrap();
        let take = bytes.len().min(SIZE - part.len());
        part.extend_from_slice(&bytes[..take]);
        Ok(take)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

pub enum Payload {
    Json(Value),
    State(Parts),
}

impl Payload {
    pub fn write_to(&self, mut out: impl std::io::Write) -> Result<()> {
        match self {
            Self::Json(value) => serde_json::to_writer(out, value)?,
            Self::State(parts) => {
                for part in &parts.0 {
                    out.write_all(part)?;
                }
            }
        }
        Ok(())
    }
}

fn state(sh: &Shared, uri: &str) -> Result<Payload> {
    let since = query(uri, "since").and_then(|v| v.parse().ok()).unwrap_or(0);
    let extra = net_summary(sh);
    let mut parts = Parts::default();
    sh.engine.lock().unwrap().write_snapshot(&mut parts, extra, since)?;
    Ok(Payload::State(parts))
}

/// Both transports use the same dispatcher and serialization boundary.
pub fn with_reply<T>(
    sh: &Arc<Shared>,
    method: &str,
    uri: &str,
    body: &Value,
    respond: impl FnOnce(Result<Payload>) -> T,
) -> T {
    let _guard = CONTROL.lock().unwrap();
    let reply = if method == "GET" && uri.split('?').next() == Some("/api/state") {
        state(sh, uri)
    } else {
        api(sh, method, uri, body).map(Payload::Json)
    };
    respond(reply)
}

pub fn start(sh: Arc<Shared>) -> Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&HttpConfig {
        stack_size: 20480,
        max_uri_handlers: 12,
        uri_match_wildcard: true,
        max_open_sockets: 5,
        lru_purge_enable: true,
        ..Default::default()
    })?;

    for route in ["/", "/index.html"] {
        server.fn_handler::<anyhow::Error, _>(route, Method::Get, |req| {
            send(req, 200, "text/html; charset=utf-8", INDEX.as_bytes())
        })?;
    }

    for (route, method, verb) in [
        ("/api/catalog", Method::Get, "GET"),
        ("/api/config", Method::Get, "GET"),
        ("/api/config", Method::Post, "POST"),
        ("/api/state", Method::Get, "GET"),
        ("/api/net", Method::Get, "GET"),
        ("/api/action", Method::Post, "POST"),
        ("/api/net/action", Method::Post, "POST"),
    ] {
        let s = sh.clone();
        server.fn_handler::<anyhow::Error, _>(route, method, move |mut req| {
            let uri = req.uri().to_owned();
            let body = if verb == "POST" {
                match read_json(&mut req) {
                    Ok(value) => value,
                    Err(e) => return send(req, 400, "text/plain", e.to_string().as_bytes()),
                }
            } else {
                Value::Null
            };
            with_reply(&s, verb, &uri, &body, |result| match result {
                Ok(reply) => send_json(req, &reply),
                Err(err) => send(req, 400, "text/plain", err.to_string().as_bytes()),
            })
        })?;
    }

    // Captive portal: OS connectivity probes and any foreign host go to the control panel.
    let s = sh.clone();
    server.fn_handler::<anyhow::Error, _>("/*", Method::Get, move |req| {
        let host = req.header("Host").unwrap_or("").to_string();
        let ours = host.is_empty() || host.starts_with("192.168.4.1");
        if s.captive.load(Ordering::Relaxed) && !ours {
            let mut resp = req
                .into_response(302, Some("Found"), &[("Location", "http://192.168.4.1/"), ("Cache-Control", "no-store")])
                .map_err(|e| anyhow!("{e:?}"))?;
            resp.write_all(b"GDL 90 emulator control panel: http://192.168.4.1/").map_err(|e| anyhow!("{e:?}"))?;
            Ok(())
        } else {
            send(req, 404, "text/plain", b"not found")
        }
    })?;

    Ok(server)
}
