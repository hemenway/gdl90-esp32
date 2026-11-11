//! Control actions shared by the PC app and the ESP32 firmware (`POST /api/action`).
//! Platform-specific actions (recording, replay from disk, config files, clock) are handled by
//! the caller when this returns `Ok(None)`.

use anyhow::{anyhow, Result};
use serde_json::{json, Value};

use crate::config::{Config, Waypoint};
use crate::engine::Engine;
use crate::scenarios::Anchor;

pub fn handle(e: &mut Engine, v: &Value) -> Result<Option<Value>> {
    let t = v.get("type").and_then(|x| x.as_str()).unwrap_or("");
    let s = |k: &str| v.get(k).and_then(|x| x.as_str()).map(str::to_string);
    let mut extra = json!({});
    match t {
        "start" => {
            e.running = true;
            e.paused = false;
        }
        "stop" => e.running = false,
        "pause" => e.paused = true,
        "resume" => e.paused = false,
        "load_scenario" => {
            let anchor: Anchor = v.get("anchor").cloned().map(serde_json::from_value).transpose()?.unwrap_or_default();
            e.load_scenario(&s("id").unwrap_or_default(), &anchor)?;
            e.running = true;
            e.paused = false;
        }
        "apply_profile" => e.apply_profile(&s("id").unwrap_or_default())?,
        "teleport" => e.teleport(v),
        "add_traffic" => extra = json!({"ids": e.add_traffic(v)?}),
        "remove_traffic" => e.remove_traffic(v.get("id").and_then(|x| x.as_u64()).unwrap_or(0) as u32),
        "clear_traffic" => e.clear_traffic(),
        "replay_stop" => {
            e.replay = None;
            e.notice("Replay stopped");
        }
        "reset_stats" => e.stats = Default::default(),
        "reset_config" => {
            let keep_net = e.cfg.network.clone();
            let keep_out = e.cfg.output.clone();
            e.cfg = Config { network: keep_net, output: keep_out, ..Default::default() };
            e.load_scenario("orbit", &Anchor::default())?;
        }
        "route_add" => {
            let f = &e.cfg.flight;
            anyhow::ensure!(f.route.len() < 64, "Route already contains 64 waypoints");
            let w = Waypoint {
                name: s("name").unwrap_or_else(|| format!("WP{}", f.route.len() + 1)),
                lat: v.get("lat").and_then(|x| x.as_f64()).ok_or_else(|| anyhow!("lat required"))?,
                lon: v.get("lon").and_then(|x| x.as_f64()).ok_or_else(|| anyhow!("lon required"))?,
                alt_ft: v.get("alt_ft").and_then(|x| x.as_f64()).unwrap_or(f.target_alt_ft),
                speed_kt: v.get("speed_kt").and_then(|x| x.as_f64()).unwrap_or(f.target_speed_kt),
            };
            e.cfg.flight.route.push(w);
        }
        "route_clear" => {
            e.cfg.flight.route.clear();
            e.nav = Default::default();
        }
        _ => return Ok(None),
    }
    Ok(Some(extra))
}
