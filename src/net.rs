//! UDP output, target resolution and client discovery.
//!
//! Discovery sources: the hotspot's own client list (WinRT), the ARP table on the hotspot
//! subnet, iFly's "Stratus: leave ForeFlight mode" probe (raw `C2 53 FF 56 01 01 6E 37` sent to
//! `<subnet>.1:41500` on every connect) and ForeFlight-style JSON broadcasts on UDP 63093.

use std::collections::{BTreeMap, BTreeSet};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::Serialize;
use tokio::net::UdpSocket;
use tokio::sync::mpsc::UnboundedReceiver;

use crate::app::App;
use crate::config::OutputConfig;
use crate::engine::Datagram;
use crate::geo;

#[derive(Clone, Debug, Serialize)]
pub struct Client {
    pub ip: Ipv4Addr,
    pub mac: String,
    pub name: String,
    pub sources: BTreeSet<String>,
    pub last_seen: f64,
    /// GDL 90 port the client asked for (ForeFlight discovery), if any.
    pub port: Option<u16>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Iface {
    pub name: String,
    pub ip: Ipv4Addr,
    pub prefix: u8,
    pub broadcast: Ipv4Addr,
}

#[derive(Default, Serialize)]
pub struct NetState {
    pub interfaces: Vec<Iface>,
    /// Interface carrying the hotspot (192.168.137.x under Windows ICS).
    pub hotspot_iface: Option<Iface>,
    pub clients: BTreeMap<Ipv4Addr, Client>,
    pub listeners: Vec<String>,
    pub targets: Vec<String>,
    pub sent_datagrams: u64,
    pub send_errors: u64,
    pub last_error: String,
}

impl NetState {
    pub fn seen(&mut self, ip: Ipv4Addr, source: &str, mac: Option<&str>, name: Option<&str>, port: Option<u16>) {
        if ip.is_loopback() || ip.is_unspecified() || ip.is_broadcast() || ip.is_multicast() {
            return;
        }
        if let Some(h) = &self.hotspot_iface {
            if ip == h.ip {
                return;
            }
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
        if let Some(n) = name.filter(|n| !n.is_empty()) {
            c.name = n.to_string();
        }
        if port.is_some() {
            c.port = port;
        }
    }

    pub fn prune(&mut self, max_age_s: f64) {
        let now = geo::unix_now_f64();
        self.clients.retain(|_, c| now - c.last_seen < max_age_s);
    }

    fn on_hotspot(&self, ip: Ipv4Addr) -> bool {
        match &self.hotspot_iface {
            Some(h) => same_subnet(ip, h.ip, h.prefix),
            None => false,
        }
    }
}

pub fn same_subnet(a: Ipv4Addr, b: Ipv4Addr, prefix: u8) -> bool {
    let mask = if prefix == 0 { 0 } else { u32::MAX << (32 - prefix.min(32)) };
    (u32::from(a) & mask) == (u32::from(b) & mask)
}

pub fn refresh_interfaces(net: &mut NetState) {
    let mut v = Vec::new();
    if let Ok(list) = if_addrs::get_if_addrs() {
        for i in list {
            if let if_addrs::IfAddr::V4(a) = &i.addr {
                if a.ip.is_loopback() || a.ip.is_link_local() {
                    continue;
                }
                let prefix = u32::from(a.netmask).count_ones() as u8;
                let bc = a.broadcast.unwrap_or_else(|| Ipv4Addr::from(u32::from(a.ip) | !u32::from(a.netmask)));
                v.push(Iface { name: i.name.clone(), ip: a.ip, prefix, broadcast: bc });
            }
        }
    }
    // Windows ICS (Mobile Hotspot) uses 192.168.137.1/24 by default.
    net.hotspot_iface = v.iter().find(|i| i.ip.octets()[..3] == [192, 168, 137]).cloned();
    net.interfaces = v;
}

/// Parse `arp -a` output: dynamic entries on the hotspot interface.
pub fn arp_clients(hotspot_ip: Ipv4Addr) -> Vec<(Ipv4Addr, String)> {
    let mut cmd = std::process::Command::new("arp");
    cmd.args(["-a", "-N", &hotspot_ip.to_string()]);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        cmd.creation_flags(0x0800_0000);
    }
    let Ok(out) = cmd.output() else { return vec![] };
    let text = String::from_utf8_lossy(&out.stdout);
    let mut v = Vec::new();
    for line in text.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 3 && parts[2].eq_ignore_ascii_case("dynamic") {
            if let Ok(ip) = parts[0].parse::<Ipv4Addr>() {
                if !ip.is_multicast() && ip.octets()[3] != 255 {
                    v.push((ip, parts[1].to_string()));
                }
            }
        }
    }
    v
}

fn parse_target(s: &str) -> Option<(Ipv4Addr, Option<u16>)> {
    let s = s.trim();
    if let Ok(sa) = s.parse::<SocketAddrV4>() {
        return Some((*sa.ip(), Some(sa.port())));
    }
    s.parse::<Ipv4Addr>().ok().map(|ip| (ip, None))
}

/// Resolve the destinations for one datagram.
pub fn resolve_targets(out: &OutputConfig, net: &NetState, port_override: Option<u16>) -> Vec<SocketAddr> {
    let ports: Vec<u16> = match port_override {
        Some(p) => vec![p],
        None => out.ports.clone(),
    };
    let mut v: Vec<SocketAddr> = Vec::new();
    let now = geo::unix_now_f64();
    let clients: Vec<&Client> = net
        .clients
        .values()
        .filter(|c| now - c.last_seen < 120.0 && (net.on_hotspot(c.ip) || c.sources.contains("probe") || c.sources.contains("foreflight")))
        .collect();
    let mode = out.delivery.as_str();
    let want_unicast = matches!(mode, "unicast" | "both") || (mode == "auto" && !clients.is_empty());
    let want_broadcast = matches!(mode, "broadcast" | "both") || (mode == "auto" && clients.is_empty());
    if want_broadcast {
        let bc = if out.broadcast_addr.trim().eq_ignore_ascii_case("auto") || out.broadcast_addr.trim().is_empty() {
            net.hotspot_iface.as_ref().map(|h| h.broadcast)
        } else {
            out.broadcast_addr.trim().parse().ok()
        };
        if let Some(bc) = bc {
            for p in &ports {
                v.push(SocketAddr::new(IpAddr::V4(bc), *p));
            }
        }
    }
    if want_unicast {
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
        if let Some((ip, port)) = parse_target(t) {
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
    if out.localhost {
        for p in &ports {
            v.push(SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), *p));
        }
    }
    v.sort();
    v.dedup();
    v
}

/// Sends datagrams to the resolved targets.
pub async fn sender_task(app: Arc<App>, mut rx: UnboundedReceiver<Datagram>) {
    let std_sock = match std::net::UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[net] cannot bind UDP sender: {e}");
            return;
        }
    };
    let _ = std_sock.set_broadcast(true);
    let _ = std_sock.set_nonblocking(true);
    let sock = UdpSocket::from_std(std_sock).expect("tokio socket");
    let mut cache: (Instant, OutputConfig) = (Instant::now() - Duration::from_secs(10), OutputConfig::default());
    let mut target_cache: BTreeMap<Option<u16>, Vec<SocketAddr>> = BTreeMap::new();

    while let Some(d) = rx.recv().await {
        let now = Instant::now();
        if now.duration_since(cache.0) > Duration::from_millis(500) {
            cache = (now, app.engine.lock().unwrap().cfg.output.clone());
            target_cache.clear();
        }
        let mut errors = Vec::new();
        let mut sent = 0u64;
        if !target_cache.contains_key(&d.port) {
            let mut net = app.net.lock().unwrap();
            let t = resolve_targets(&cache.1, &net, d.port);
            net.targets = t.iter().map(|a| a.to_string()).collect();
            target_cache.insert(d.port, t);
        }
        for addr in &target_cache[&d.port] {
            match sock.send_to(&d.bytes, addr).await {
                Ok(_) => sent += 1,
                Err(e) => errors.push(format!("{addr}: {e}")),
            }
        }
        let mut net = app.net.lock().unwrap();
        net.sent_datagrams += sent;
        if !errors.is_empty() {
            net.send_errors += errors.len() as u64;
            net.last_error = errors.pop().unwrap();
        }
    }
}

/// Listen for iFly's Stratus probe (41500) and ForeFlight discovery broadcasts (63093).
pub async fn discovery_listeners(app: Arc<App>) {
    for port in [41500u16, 63093] {
        let app = app.clone();
        tokio::spawn(async move {
            let sock = match UdpSocket::bind(("0.0.0.0", port)).await {
                Ok(s) => s,
                Err(e) => {
                    app.net.lock().unwrap().listeners.push(format!("udp/{port}: {e}"));
                    return;
                }
            };
            app.net.lock().unwrap().listeners.push(format!("udp/{port}: listening"));
            let mut buf = vec![0u8; 2048];
            loop {
                let Ok((n, from)) = sock.recv_from(&mut buf).await else { continue };
                let IpAddr::V4(ip) = from.ip() else { continue };
                let data = &buf[..n];
                if port == 41500 {
                    app.net.lock().unwrap().seen(ip, "probe", None, None, None);
                    let hex = crate::gdl90::hex(data);
                    app.engine.lock().unwrap().notice(format!("iFly probe from {ip}: {hex}"));
                } else {
                    // {"App":"ForeFlight","GDL90":{"port":4000}}
                    let port = serde_json::from_slice::<serde_json::Value>(data)
                        .ok()
                        .and_then(|v| v.get("GDL90").and_then(|g| g.get("port")).and_then(|p| p.as_u64()))
                        .map(|p| p as u16);
                    app.net.lock().unwrap().seen(ip, "foreflight", None, None, port);
                }
            }
        });
    }
}

/// Periodic interface refresh + ARP discovery on the hotspot subnet.
pub async fn discovery_poller(app: Arc<App>) {
    let mut tick = tokio::time::interval(Duration::from_secs(3));
    loop {
        tick.tick().await;
        let hotspot_ip = {
            let mut net = app.net.lock().unwrap();
            refresh_interfaces(&mut net);
            net.prune(180.0);
            net.hotspot_iface.as_ref().map(|h| h.ip)
        };
        if let Some(ip) = hotspot_ip {
            let entries = tokio::task::spawn_blocking(move || arp_clients(ip)).await.unwrap_or_default();
            let mut net = app.net.lock().unwrap();
            for (cip, mac) in entries {
                if net.on_hotspot(cip) {
                    net.seen(cip, "arp", Some(&mac), None, None);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn net_with_hotspot() -> NetState {
        let mut n = NetState::default();
        n.hotspot_iface = Some(Iface {
            name: "hs".into(),
            ip: "192.168.137.1".parse().unwrap(),
            prefix: 24,
            broadcast: "192.168.137.255".parse().unwrap(),
        });
        n
    }

    #[test]
    fn auto_delivery_broadcasts_until_a_client_appears() {
        let mut out = OutputConfig::default();
        out.ports = vec![4000, 43211];
        let mut n = net_with_hotspot();
        let t = resolve_targets(&out, &n, None);
        assert_eq!(t.len(), 2);
        assert!(t.iter().all(|a| a.ip().to_string() == "192.168.137.255"));
        n.seen("192.168.137.42".parse().unwrap(), "arp", Some("aa-bb"), None, None);
        let t = resolve_targets(&out, &n, None);
        assert_eq!(t.len(), 2);
        assert!(t.iter().all(|a| a.ip().to_string() == "192.168.137.42"));
        // xplane/nmea override port
        let t = resolve_targets(&out, &n, Some(49002));
        assert_eq!(t, vec!["192.168.137.42:49002".parse::<SocketAddr>().unwrap()]);
    }

    #[test]
    fn no_hotspot_means_no_lan_broadcast() {
        let out = OutputConfig::default();
        let n = NetState::default();
        assert!(resolve_targets(&out, &n, None).is_empty());
    }
}
