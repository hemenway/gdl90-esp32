//! UDP delivery and device discovery on the ESP32's own access point (192.168.4.0/24).
//!
//! Devices are learned from the AP's DHCP leases, iFly's "Stratus: leave ForeFlight mode" probe
//! (sent to <subnet>.1:41500 on every connect, which is us) and ForeFlight JSON broadcasts on
//! 63093. "auto" delivery unicasts to known devices and broadcasts while there are none.

use std::collections::{BTreeMap, BTreeSet};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use serde::Serialize;

use crate::config::OutputConfig;
use crate::engine::Datagram;
use crate::geo;
use crate::Shared;

const SUBNET_BROADCAST: Ipv4Addr = Ipv4Addr::new(192, 168, 4, 255);

#[derive(Clone, Debug, Serialize)]
pub struct Client {
    pub ip: Ipv4Addr,
    pub mac: String,
    pub name: String,
    pub sources: BTreeSet<String>,
    pub last_seen: f64,
    pub port: Option<u16>,
}

#[derive(Default)]
pub struct NetState {
    pub clients: BTreeMap<Ipv4Addr, Client>,
    pub targets: Vec<String>,
    pub sent: u64,
    pub errors: u64,
    pub last_error: String,
    pub listeners: Vec<String>,
}

fn ap_client(ip: Ipv4Addr) -> bool {
    matches!(ip.octets(), [192, 168, 4, 2..=254])
}

impl NetState {
    pub fn seen(&mut self, ip: Ipv4Addr, source: &str, mac: Option<&str>, port: Option<u16>) {
        if !ap_client(ip) {
            return;
        }
        let now = geo::unix_now_f64();
        let c = self.clients.entry(ip).or_insert_with(|| Client {
            ip,
            mac: String::new(),
            name: String::new(),
            sources: BTreeSet::new(),
            last_seen: now,
            port: None,
        });
        c.last_seen = now;
        c.sources.insert(source.to_string());
        if let Some(m) = mac.filter(|m| !m.is_empty()) {
            c.mac = m.to_string();
        }
        if port.is_some() {
            c.port = port;
        }
    }

    /// `geo::unix_now_f64` can jump when the browser sets the clock, so age by "seen recently"
    /// and refresh from DHCP every couple of seconds.
    pub fn prune(&mut self, max_age_s: f64) {
        let now = geo::unix_now_f64();
        self.clients.retain(|_, c| (now - c.last_seen).abs() < max_age_s);
    }
}

/// Associated stations and their DHCP addresses.
pub fn ap_stations(wifi: &BlockingWifi<EspWifi<'static>>) -> Vec<(String, Option<Ipv4Addr>)> {
    use esp_idf_svc::sys::*;
    let mut out = Vec::new();
    unsafe {
        let mut list: wifi_sta_list_t = core::mem::zeroed();
        if esp_wifi_ap_get_sta_list(&mut list) != ESP_OK {
            return out;
        }
        let n = (list.num.max(0) as usize).min(list.sta.len());
        if n == 0 {
            return out;
        }
        let mut pairs: Vec<esp_netif_pair_mac_ip_t> = (0..n)
            .map(|i| {
                let mut p: esp_netif_pair_mac_ip_t = core::mem::zeroed();
                p.mac = list.sta[i].mac;
                p
            })
            .collect();
        let netif = wifi.wifi().ap_netif().handle();
        let ok = esp_netif_dhcps_get_clients_by_mac(netif as _, n as i32, pairs.as_mut_ptr()) == ESP_OK;
        for p in &pairs {
            let mac = p.mac.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":");
            let ip = Ipv4Addr::from(p.ip.addr.to_le_bytes());
            out.push((mac, if ok && !ip.is_unspecified() { Some(ip) } else { None }));
        }
    }
    out
}

fn parse_target(s: &str) -> Option<(Ipv4Addr, Option<u16>)> {
    let s = s.trim();
    if let Ok(sa) = s.parse::<SocketAddrV4>() {
        return Some((*sa.ip(), Some(sa.port())));
    }
    s.parse::<Ipv4Addr>().ok().map(|ip| (ip, None))
}

pub fn resolve(out: &OutputConfig, net: &NetState, port_override: Option<u16>) -> Vec<SocketAddr> {
    let ports: Vec<u16> = match port_override {
        Some(p) => vec![p],
        None => out.ports.clone(),
    };
    let mut v = Vec::new();
    let clients: Vec<&Client> = net.clients.values().collect();
    let mode = out.delivery.as_str();
    let unicast = matches!(mode, "unicast" | "both") || (mode == "auto" && !clients.is_empty());
    let broadcast = matches!(mode, "broadcast" | "both") || (mode == "auto" && clients.is_empty());
    if broadcast {
        for p in &ports {
            v.push(SocketAddr::new(IpAddr::V4(SUBNET_BROADCAST), *p));
        }
    }
    if unicast {
        for c in clients {
            match (port_override, c.port) {
                (None, Some(cp)) => v.push(SocketAddr::new(IpAddr::V4(c.ip), cp)),
                _ => {
                    for p in &ports {
                        v.push(SocketAddr::new(IpAddr::V4(c.ip), *p));
                    }
                }
            }
        }
    }
    for t in &out.targets {
        if let Some((ip, port)) = parse_target(t).filter(|(ip, _)| ap_client(*ip)) {
            match (port, port_override) {
                (Some(p), None) => v.push(SocketAddr::new(IpAddr::V4(ip), p)),
                _ => {
                    for p in &ports {
                        v.push(SocketAddr::new(IpAddr::V4(ip), *p));
                    }
                }
            }
        }
    }
    v.sort();
    v.dedup();
    v
}

pub fn send(sock: &UdpSocket, d: &Datagram, out: &OutputConfig, net: &Mutex<NetState>) {
    let mut n = net.lock().unwrap();
    let targets = resolve(out, &n, d.port);
    if d.port.is_none() {
        n.targets = targets.iter().map(|a| a.to_string()).collect();
    }
    for t in &targets {
        match sock.send_to(&d.bytes, t) {
            Ok(_) => n.sent += 1,
            Err(e) => {
                // ENOMEM/EAGAIN when the Wi-Fi TX queue is full: count it, don't stall the clock
                n.errors += 1;
                n.last_error = format!("{t}: {e}");
            }
        }
    }
}

/// iFly's probe on 41500 and ForeFlight discovery on 63093.
pub fn spawn_listeners(sh: Arc<Shared>) -> anyhow::Result<()> {
    std::thread::Builder::new().name("discovery".into()).stack_size(6144).spawn(move || {
        let bind = |port: u16| -> Option<UdpSocket> {
            match UdpSocket::bind(("0.0.0.0", port)) {
                Ok(s) => {
                    let _ = s.set_read_timeout(Some(Duration::from_millis(150)));
                    sh.net.lock().unwrap().listeners.push(format!("udp/{port}: listening"));
                    Some(s)
                }
                Err(e) => {
                    sh.net.lock().unwrap().listeners.push(format!("udp/{port}: {e}"));
                    None
                }
            }
        };
        let probe = bind(41500);
        let ff = bind(63093);
        let mut buf = [0u8; 512];
        loop {
            if let Some(s) = &probe {
                if let Ok((n, from)) = s.recv_from(&mut buf) {
                    if let IpAddr::V4(ip) = from.ip() {
                        sh.net.lock().unwrap().seen(ip, "probe", None, None);
                        let hex = crate::gdl90::hex(&buf[..n.min(16)]);
                        sh.engine.lock().unwrap().notice(format!("iFly probe from {ip}: {hex}"));
                    }
                }
            }
            if let Some(s) = &ff {
                if let Ok((n, from)) = s.recv_from(&mut buf) {
                    if let IpAddr::V4(ip) = from.ip() {
                        let port = serde_json::from_slice::<serde_json::Value>(&buf[..n])
                            .ok()
                            .and_then(|v| v.get("GDL90").and_then(|g| g.get("port")).and_then(|p| p.as_u64()))
                            .and_then(|p| u16::try_from(p).ok()).filter(|p| *p != 0);
                        sh.net.lock().unwrap().seen(ip, "foreflight", None, port);
                    }
                }
            }
            if probe.is_none() && ff.is_none() {
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    })?;
    Ok(())
}
