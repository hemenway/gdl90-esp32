//! Shared application state and the long-running tasks (engine clock, Wi-Fi manager).

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Serialize;
use serde_json::{json, Value};
use tokio::sync::{mpsc::UnboundedSender, Notify};

use crate::engine::{Datagram, Engine};
use crate::geo;
use crate::net::NetState;
use crate::wifi::{self, AdapterInfo, HotspotStatus};

#[derive(Default, Serialize)]
pub struct WifiState {
    pub adapters: Vec<AdapterInfo>,
    pub chosen: Option<AdapterInfo>,
    pub hotspot: HotspotStatus,
    /// What the manager is doing right now ("" = idle).
    pub busy: String,
    pub last_scan: f64,
    pub log: VecDeque<(f64, String)>,
    /// GUID we already tried to auto-start on (no retry loop on failure).
    #[serde(skip)]
    pub auto_tried: Option<String>,
    /// We started the hotspot, so we stop it on exit.
    pub started_by_us: bool,
    pub scan_error: String,
}

impl WifiState {
    pub fn log(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        eprintln!("[wifi] {msg}");
        self.log.push_back((geo::unix_now_f64(), msg));
        while self.log.len() > 40 {
            self.log.pop_front();
        }
    }
}

pub struct App {
    pub engine: Mutex<Engine>,
    pub net: Mutex<NetState>,
    pub wifi: Mutex<WifiState>,
    pub tx: UnboundedSender<Datagram>,
    pub wifi_kick: Notify,
}

impl App {
    pub fn net_summary(&self) -> Value {
        let n = self.net.lock().unwrap();
        json!({
            "hotspot_iface": n.hotspot_iface,
            "interfaces": n.interfaces,
            "clients": n.clients.values().collect::<Vec<_>>(),
            "targets": n.targets,
            "sent_datagrams": n.sent_datagrams,
            "send_errors": n.send_errors,
            "last_error": n.last_error,
            "listeners": n.listeners,
        })
    }

    pub fn wifi_summary(&self) -> Value {
        serde_json::to_value(&*self.wifi.lock().unwrap()).unwrap_or(Value::Null)
    }
}

/// 50 Hz engine clock.
pub async fn engine_loop(app: Arc<App>) {
    let mut iv = tokio::time::interval(Duration::from_millis(20));
    iv.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut last = Instant::now();
    loop {
        iv.tick().await;
        let now = Instant::now();
        let dt = (now - last).as_secs_f64().min(0.5);
        last = now;
        let out = app.engine.lock().unwrap().step(dt, geo::unix_now_f64());
        for d in out {
            let _ = app.tx.send(d);
        }
    }
}

pub async fn hotspot_start(app: &Arc<App>) -> anyhow::Result<String> {
    let (guid, name, mac, ssid, pass, band) = {
        let w = app.wifi.lock().unwrap();
        let e = app.engine.lock().unwrap();
        let a = w.chosen.clone().ok_or_else(|| anyhow::anyhow!("no Wi-Fi adapter selected"))?;
        if !a.usable() {
            anyhow::bail!("{} is not usable ({})", a.name, a.note);
        }
        if a.mobile_hotspot_ok == Some(false) {
            anyhow::bail!(
                "{} has a legacy (non-WDI) driver, so Windows 11 can't run a hotspot on it: Windows would \
                 silently use another radio instead. Options: plug in Ethernet and pick the built-in Wi-Fi, \
                 use an adapter with a WDI driver, or join the stick and the tablet to the same network.",
                a.name
            );
        }
        let n = &e.cfg.network;
        (a.guid.clone(), a.name.clone(), a.mac.clone(), n.ssid.clone(), n.passphrase.clone(), n.band.clone())
    };
    {
        let mut w = app.wifi.lock().unwrap();
        w.busy = format!("starting hotspot on {name}");
        w.log(format!("Starting hotspot '{ssid}' on {name}"));
    }
    let g = guid.clone();
    let mut r = tokio::task::spawn_blocking(move || wifi::hotspot::start(&g, &ssid, &pass, &band)).await?;
    // Windows treats the target adapter as a hint: if the driver lacks Wi-Fi Direct it silently
    // hosts on another radio (seen 2026-11-09: RTL8188CU requested, built-in RTL8822CE used, and
    // the PC's own link dropped from 5 GHz to 2.4 GHz). Check which radio got the ICS address.
    if r.is_ok() && !mac.is_empty() {
        let mut found = None;
        for _ in 0..12 {
            found = tokio::task::spawn_blocking(wifi::ics_adapter).await.ok().flatten();
            if found.is_some() {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
        }
        if let Some((vname, vdesc, vmac)) = found {
            if wifi::mac_tail(&vmac) != wifi::mac_tail(&mac) {
                let g = guid.clone();
                let _ = tokio::task::spawn_blocking(move || wifi::hotspot::stop(Some(&g))).await;
                r = Err(anyhow::anyhow!(
                    "Windows put the Mobile Hotspot on {vname} ({vdesc}, {vmac}), not {name}; stopped it. \
                     This adapter's driver can't host Mobile Hotspot; use hosted-network mode instead."
                ));
            }
        }
    }
    let mut w = app.wifi.lock().unwrap();
    w.busy.clear();
    match &r {
        Ok(m) => {
            w.started_by_us = true;
            w.log(m.clone());
        }
        Err(e) => w.log(format!("Hotspot start failed: {e}")),
    }
    drop(w);
    app.wifi_kick.notify_one();
    r
}

pub async fn hotspot_stop(app: &Arc<App>) -> anyhow::Result<String> {
    let guid = app.wifi.lock().unwrap().chosen.as_ref().map(|a| a.guid.clone()).filter(|g| !g.is_empty());
    app.wifi.lock().unwrap().busy = "stopping hotspot".into();
    let r = tokio::task::spawn_blocking(move || wifi::hotspot::stop(guid.as_deref())).await?;
    let mut w = app.wifi.lock().unwrap();
    w.busy.clear();
    match &r {
        Ok(m) => {
            w.started_by_us = false;
            w.log(m.clone());
        }
        Err(e) => w.log(format!("Hotspot stop failed: {e}")),
    }
    drop(w);
    app.wifi_kick.notify_one();
    r
}

/// Finds the preferred adapter, keeps hotspot status/clients fresh, auto-starts the hotspot.
pub async fn wifi_manager(app: Arc<App>) {
    let mut last_scan = Instant::now() - Duration::from_secs(3600);
    loop {
        let (adapter_match, force_guid, auto) = {
            let e = app.engine.lock().unwrap();
            (e.cfg.network.adapter_match.clone(), e.cfg.network.adapter_guid.clone(), e.cfg.network.auto_hotspot)
        };
        let hotspot_on = app.wifi.lock().unwrap().hotspot.state == "on";
        let scan_every = if hotspot_on { 30 } else { 10 };
        if last_scan.elapsed() >= Duration::from_secs(scan_every) {
            last_scan = Instant::now();
            let (m, g) = (adapter_match.clone(), force_guid.clone());
            match tokio::task::spawn_blocking(move || wifi::scan(&m, &g)).await {
                Ok(Ok(list)) => {
                    let mut w = app.wifi.lock().unwrap();
                    let chosen = wifi::pick(&list, &force_guid).cloned();
                    let prev = w.chosen.as_ref().map(|a| (a.pnp_id.clone(), a.usable()));
                    let now = chosen.as_ref().map(|a| (a.pnp_id.clone(), a.usable()));
                    if prev != now {
                        match &chosen {
                            Some(a) if a.usable() && a.mobile_hotspot_ok == Some(false) => w.log(format!(
                                "Found {} ({}) but its driver can't host a hotspot on Windows 11: {}",
                                a.name, a.description, a.note
                            )),
                            Some(a) if a.usable() => w.log(format!("Using {} ({}) {}", a.name, a.description, a.note)),
                            Some(a) => w.log(format!("Found {} ({}) but it is not usable: {}", a.name, a.description, a.note)),
                            None => w.log("No spare Wi-Fi adapter found (the one carrying this PC's connection is never auto-used)"),
                        }
                    }
                    w.adapters = list;
                    w.chosen = chosen;
                    w.last_scan = geo::unix_now_f64();
                    w.scan_error.clear();
                }
                Ok(Err(e)) => app.wifi.lock().unwrap().scan_error = e.to_string(),
                Err(e) => app.wifi.lock().unwrap().scan_error = e.to_string(),
            }
        }

        let guid = app.wifi.lock().unwrap().chosen.as_ref().filter(|a| a.usable()).map(|a| a.guid.clone());
        let g2 = guid.clone();
        let status = tokio::task::spawn_blocking(move || wifi::hotspot::status(g2.as_deref())).await;
        let mut start_now = false;
        {
            let mut w = app.wifi.lock().unwrap();
            match status {
                Ok(Ok(mut s)) => {
                    s.adapter = w.chosen.as_ref().map(|a| a.name.clone()).unwrap_or_default();
                    let clients = s.clients.clone();
                    w.hotspot = s;
                    drop(w);
                    let mut net = app.net.lock().unwrap();
                    for c in clients {
                        for ip in &c.ips {
                            if let Ok(ip) = ip.parse() {
                                net.seen(ip, "hotspot", Some(&c.mac), Some(&c.name), None);
                            }
                        }
                    }
                    w = app.wifi.lock().unwrap();
                }
                Ok(Err(e)) => {
                    w.hotspot.state = "error".into();
                    w.hotspot.message = e.to_string();
                }
                Err(e) => w.hotspot.message = e.to_string(),
            }
            let can_host = w.chosen.as_ref().map(|a| a.mobile_hotspot_ok != Some(false)).unwrap_or(false);
            if auto && can_host && w.busy.is_empty() && w.hotspot.state == "off" {
                if let Some(g) = &guid {
                    if w.auto_tried.as_deref() != Some(g.as_str()) {
                        w.auto_tried = Some(g.clone());
                        start_now = true;
                    }
                }
            }
        }
        if start_now {
            let _ = hotspot_start(&app).await;
            continue;
        }
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_secs(4)) => {}
            _ = app.wifi_kick.notified() => { last_scan = Instant::now() - Duration::from_secs(3600); }
        }
    }
}
