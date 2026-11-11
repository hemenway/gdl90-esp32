//! Local control UI + JSON API + telemetry websocket.

use std::sync::Arc;
use std::time::Duration;

use axum::body::Bytes;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{DefaultBodyLimit, Query, State};
use axum::http::StatusCode;
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::actions;
use crate::app::{self, App};
use crate::config::{self, Config};
use crate::recorder;
use crate::scenarios::{self, Anchor};

const INDEX: &str = include_str!("../ui/index.html");

type ApiResult = Result<Json<Value>, (StatusCode, String)>;

fn bad(e: impl std::fmt::Display) -> (StatusCode, String) {
    (StatusCode::BAD_REQUEST, e.to_string())
}

pub fn router(app: Arc<App>) -> Router {
    Router::new()
        .route("/", get(|| async { Html(INDEX) }))
        .route("/api/config", get(get_config).post(post_config))
        .route("/api/state", get(get_state))
        .route("/api/catalog", get(catalog))
        .route("/api/action", post(action))
        .route("/api/log", get(get_log))
        .route("/api/net", get(get_net))
        .route("/api/net/action", post(net_action))
        .route("/api/replay", post(replay_upload))
        .route("/ws", get(ws))
        .layer(DefaultBodyLimit::max(64 * 1024 * 1024))
        .with_state(app)
}

async fn get_config(State(app): State<Arc<App>>) -> Json<Value> {
    Json(serde_json::to_value(&app.engine.lock().unwrap().cfg).unwrap())
}

async fn post_config(State(app): State<Arc<App>>, Json(patch): Json<Value>) -> ApiResult {
    let mut e = app.engine.lock().unwrap();
    e.patch_config(&patch).map_err(bad)?;
    Ok(Json(serde_json::to_value(&e.cfg).unwrap()))
}

async fn get_state(State(app): State<Arc<App>>) -> Json<Value> {
    Json(app.engine.lock().unwrap().snapshot())
}

async fn catalog() -> Json<Value> {
    Json(json!({
        "profiles": config::PROFILES.iter().map(|p| json!({"id": p.id, "name": p.name, "description": p.description})).collect::<Vec<_>>(),
        "scenarios": scenarios::SCENARIOS.iter().map(|s| json!({"id": s.id, "name": s.name, "description": s.description})).collect::<Vec<_>>(),
        "anchor": Anchor::default(),
        "platform": "pc",
        "features": {"record": true, "replay": true, "hotspot": true, "config_files": true},
    }))
}

#[derive(Deserialize)]
struct LogQuery {
    since: Option<u64>,
    max: Option<usize>,
}

async fn get_log(State(app): State<Arc<App>>, Query(q): Query<LogQuery>) -> Json<Value> {
    Json(json!(app.engine.lock().unwrap().log_since(q.since.unwrap_or(0), q.max.unwrap_or(500))))
}

async fn get_net(State(app): State<Arc<App>>) -> Json<Value> {
    Json(json!({"net": app.net_summary(), "wifi": app.wifi_summary()}))
}

// Browser requests choose a filename under the application's storage directory.
fn data_path(name: &str, folder: &str, extension: &str) -> Result<String, (StatusCode, String)> {
    let name = name.strip_prefix(&format!("{folder}/")).unwrap_or(name);
    if name.len() > 120 || !name.ends_with(extension) || name.starts_with('.') ||
        !name.bytes().all(|b| b.is_ascii_alphanumeric() || b" ._-".contains(&b)) {
        return Err(bad(format!("Choose a filename ending in {extension}")));
    }
    std::fs::create_dir_all(folder).map_err(bad)?;
    let path = std::path::Path::new(folder).join(name);
    if std::fs::symlink_metadata(&path).is_ok_and(|m| m.is_symlink()) {
        return Err(bad("Choose a regular file"));
    }
    Ok(path.to_string_lossy().into_owned())
}

async fn action(State(app): State<Arc<App>>, Json(v): Json<Value>) -> ApiResult {
    let t = v.get("type").and_then(|x| x.as_str()).unwrap_or("").to_string();
    let s = |k: &str| v.get(k).and_then(|x| x.as_str()).map(str::to_string);
    let mut e = app.engine.lock().unwrap();
    if let Some(extra) = actions::handle(&mut e, &v).map_err(bad)? {
        let cfg = serde_json::to_value(&e.cfg).unwrap();
        return Ok(Json(json!({"ok": true, "config": cfg, "result": extra})));
    }
    let mut extra = json!({});
    match t.as_str() {
        "record_start" => extra = json!({"path": e.start_recording(s("path").filter(|p| !p.is_empty()).map(|p| data_path(&p, "recordings", ".bin")).transpose()?).map_err(bad)?}),
        "record_stop" => extra = json!({"path": e.stop_recording().map_err(bad)?}),
        "replay_file" => {
            let path = data_path(&s("path").ok_or_else(|| bad("path required"))?, "recordings", ".bin")?;
            let rec = recorder::load(std::path::Path::new(&path)).map_err(bad)?;
            let speed = v.get("speed").and_then(|x| x.as_f64()).unwrap_or(1.0);
            let looping = v.get("loop").and_then(|x| x.as_bool()).unwrap_or(false);
            e.start_replay(path, rec, speed, looping);
        }
        "set_time" => {} // the PC clock is already right
        "save_config" => {
            let path = data_path(&s("path").unwrap_or_else(|| "gdl90-emu.json".into()), "setups", ".json")?;
            std::fs::write(&path, serde_json::to_string_pretty(&e.cfg).unwrap()).map_err(bad)?;
            e.notice(format!("Config saved to {path}"));
            extra = json!({"path": path});
        }
        "load_config" => {
            let path = data_path(&s("path").unwrap_or_else(|| "gdl90-emu.json".into()), "setups", ".json")?;
            let text = std::fs::read_to_string(&path).map_err(bad)?;
            let cfg: Config = serde_json::from_str(&text).map_err(bad)?;
            cfg.validate().map_err(bad)?;
            e.cfg = cfg;
            e.notice(format!("Config loaded from {path}"));
        }
        other => return Err(bad(format!("unknown action '{other}'"))),
    }
    let cfg = serde_json::to_value(&e.cfg).unwrap();
    Ok(Json(json!({"ok": true, "config": cfg, "result": extra})))
}

async fn net_action(State(app): State<Arc<App>>, Json(v): Json<Value>) -> ApiResult {
    let t = v.get("type").and_then(|x| x.as_str()).unwrap_or("");
    let msg = match t {
        "hotspot_start" => {
            app.wifi.lock().unwrap().auto_tried = None;
            app::hotspot_start(&app).await.map_err(bad)?
        }
        "hotspot_stop" => {
            // don't let auto-start immediately undo a manual stop
            let g = app.wifi.lock().unwrap().chosen.as_ref().map(|a| a.guid.clone());
            app.wifi.lock().unwrap().auto_tried = g;
            app::hotspot_stop(&app).await.map_err(bad)?
        }
        "rescan" => {
            app.wifi_kick.notify_one();
            "rescanning".into()
        }
        "enable_adapter" => {
            let id = v
                .get("pnp_id")
                .and_then(|x| x.as_str())
                .map(str::to_string)
                .or_else(|| app.wifi.lock().unwrap().chosen.as_ref().map(|a| a.pnp_id.clone()))
                .ok_or_else(|| bad("no adapter"))?;
            app.wifi.lock().unwrap().log(format!("Requesting admin rights to enable {id}"));
            let r = tokio::task::spawn_blocking(move || crate::wifi::enable_device(&id)).await.map_err(bad)?;
            {
                let mut w = app.wifi.lock().unwrap();
                match &r {
                    Ok(_) => w.log("Enable command finished"),
                    Err(e) => w.log(format!("Enable failed: {e}")),
                }
                w.auto_tried = None;
            }
            r.map_err(bad)?;
            tokio::time::sleep(Duration::from_secs(2)).await;
            app.wifi_kick.notify_one();
            "enabled".into()
        }
        other => return Err(bad(format!("unknown net action '{other}'"))),
    };
    Ok(Json(json!({"ok": true, "message": msg})))
}

#[derive(Deserialize)]
struct ReplayQuery {
    name: Option<String>,
    speed: Option<f64>,
    #[serde(rename = "loop")]
    looping: Option<bool>,
}

async fn replay_upload(State(app): State<Arc<App>>, Query(q): Query<ReplayQuery>, body: Bytes) -> ApiResult {
    let rec = recorder::parse(&body).map_err(bad)?;
    let n = rec.records.len();
    app.engine
        .lock()
        .unwrap()
        .start_replay(q.name.unwrap_or_else(|| "upload".into()), rec, q.speed.unwrap_or(1.0), q.looping.unwrap_or(false));
    Ok(Json(json!({"ok": true, "records": n})))
}

async fn ws(ws: WebSocketUpgrade, State(app): State<Arc<App>>) -> impl IntoResponse {
    ws.on_upgrade(move |s| ws_loop(s, app))
}

async fn ws_loop(mut socket: WebSocket, app: Arc<App>) {
    let mut seq = 0u64;
    let mut iv = tokio::time::interval(Duration::from_millis(200));
    let mut n = 0u64;
    loop {
        tokio::select! {
            _ = iv.tick() => {
                n += 1;
                let mut snap = {
                    let e = app.engine.lock().unwrap();
                    let mut s = e.snapshot();
                    let log = e.log_since(seq, 400);
                    if let Some(l) = log.last() { seq = l.seq; }
                    s["log"] = json!(log);
                    s
                };
                if n % 5 == 1 {
                    snap["net"] = app.net_summary();
                    snap["wifi"] = app.wifi_summary();
                }
                if socket.send(Message::Text(snap.to_string())).await.is_err() { break; }
            }
            m = socket.recv() => {
                match m {
                    Some(Ok(Message::Close(_))) | None | Some(Err(_)) => break,
                    _ => {}
                }
            }
        }
    }
}
