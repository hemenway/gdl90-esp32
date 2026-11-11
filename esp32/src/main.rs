//! GDL 90 emulator firmware for the ESP32.
//!
//! The ESP32 is the whole receiver: it runs its own Wi-Fi access point ("GDL90-SIM"), streams
//! GDL 90 over UDP to every device that joins, and serves the browser control panel at
//! http://192.168.4.1 (opened automatically by the captive portal on phones and PCs).
//! The simulation engine is the exact same source as the PC app (`../src`).
#![allow(dead_code)]

#[path = "../../src/actions.rs"]
mod actions;
#[path = "../../src/captive.rs"]
mod captive;
#[path = "../../src/config.rs"]
mod config;
#[path = "../../src/engine.rs"]
mod engine;
#[path = "../../src/fisb.rs"]
mod fisb;
#[path = "../../src/gdl90.rs"]
mod gdl90;
#[path = "../../src/geo.rs"]
mod geo;
#[path = "../../src/recorder.rs"]
mod recorder;
#[path = "../../src/scenarios.rs"]
mod scenarios;
#[path = "../../src/sim.rs"]
mod sim;

mod dns;
mod net;
mod store;
mod usb;
mod web;

use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration, NetifStack};
use esp_idf_svc::ipv4;
use esp_idf_svc::wifi::WifiDriver;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, BlockingWifi, Configuration as WifiConfiguration, EspWifi};

use crate::config::{Config, NetworkConfig};
use crate::engine::Engine;

pub const AP_IP: [u8; 4] = [192, 168, 4, 1];

pub struct Shared {
    pub engine: Mutex<Engine>,
    pub net: Mutex<net::NetState>,
    pub wifi: Mutex<BlockingWifi<EspWifi<'static>>>,
    pub store: Mutex<store::Store>,
    pub boot: Instant,
    pub time_synced: AtomicBool,
    pub captive: AtomicBool,
    /// SSID/password/channel the access point is actually running with.
    pub ap_running: Mutex<NetworkConfig>,
}

/// First-boot settings: open network (no password) so tablets join with one tap, like Stratux.
fn default_config() -> Config {
    let mut c = Config::default();
    c.network.passphrase.clear();
    c.network.auto_hotspot = true;
    c
}

pub fn apply_ap(wifi: &mut BlockingWifi<EspWifi<'static>>, n: &NetworkConfig) -> Result<()> {
    let open = n.passphrase.is_empty();
    let ap = AccessPointConfiguration {
        ssid: n.ssid.as_str().try_into().map_err(|_| anyhow!("SSID longer than 32 characters"))?,
        ssid_hidden: false,
        channel: n.channel.clamp(1, 13),
        auth_method: if open { AuthMethod::None } else { AuthMethod::WPA2Personal },
        password: if open {
            "".try_into().map_err(|_| anyhow!("empty password"))?
        } else {
            n.passphrase.as_str().try_into().map_err(|_| anyhow!("password longer than 64 characters"))?
        },
        max_connections: 8,
        ..Default::default()
    };
    if wifi.is_started()? {
        wifi.stop()?;
    }
    wifi.set_configuration(&WifiConfiguration::AccessPoint(ap))?;
    wifi.start()?;
    wifi.wait_netif_up()?;
    log::info!(
        "Access point '{}' up on channel {} ({}) at 192.168.4.1",
        n.ssid,
        n.channel,
        if open { "open" } else { "WPA2" }
    );
    Ok(())
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("GDL 90 emulator (ESP32) starting");

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let store = store::Store::new(nvs.clone())?;
    let saved = store.load_config();
    let mut cfg = match &saved {
        Some(c) => {
            log::info!("Loaded saved settings from flash");
            c.clone()
        }
        None => default_config(),
    };

    let ip = ipv4::Ipv4Addr::from(AP_IP);
    let ap = NetifConfiguration {
        ip_configuration: Some(ipv4::Configuration::Router(ipv4::RouterConfiguration {
            subnet: ipv4::Subnet { gateway: ip, mask: ipv4::Mask(24) },
            dhcp_enabled: true,
            dns: Some(ip),
            secondary_dns: None,
        })),
        ..NetifConfiguration::wifi_default_router()
    };
    let driver = WifiDriver::new(peripherals.modem, sysloop.clone(), Some(nvs))?;
    let radio = EspWifi::wrap_all(driver, EspNetif::new(NetifStack::Sta)?, EspNetif::new_with_conf(&ap)?)?;
    let mut wifi = BlockingWifi::wrap(radio, sysloop)?;
    if let Err(err) = apply_ap(&mut wifi, &cfg.network) {
        log::warn!("Could not start saved Wi-Fi settings ({err}). Starting GDL90-SIM.");
        cfg.network = default_config().network;
        apply_ap(&mut wifi, &cfg.network)?;
    }

    let mut engine = Engine::new(cfg.clone());
    if saved.is_some() {
        // Engine::new loads the default scenario. Restore exactly
        // what was saved.
        engine.cfg = cfg.clone();
    }
    engine.notice("ESP32 emulator ready: join the Wi-Fi, then open http://192.168.4.1");

    let shared = Arc::new(Shared {
        engine: Mutex::new(engine),
        net: Mutex::new(Default::default()),
        wifi: Mutex::new(wifi),
        store: Mutex::new(store),
        boot: Instant::now(),
        time_synced: AtomicBool::new(false),
        captive: AtomicBool::new(cfg.network.captive_portal),
        ap_running: Mutex::new(cfg.network.clone()),
    });

    dns::spawn(shared.clone())?;
    net::spawn_listeners(shared.clone())?;
    let _server = web::start(shared.clone())?;
    log::info!("Control panel on http://192.168.4.1/");

    usb::start(shared.clone())?;
    run(shared)
}

/// 20 Hz engine clock + UDP output (the engine itself schedules each stream's rate).
fn run(sh: Arc<Shared>) -> ! {
    let sock = UdpSocket::bind("0.0.0.0:0").expect("UDP socket");
    let _ = sock.set_broadcast(true);
    let _ = sock.set_nonblocking(true);
    let tick = Duration::from_millis(50);
    let mut last = Instant::now();
    let mut last_clients = Instant::now();
    let mut last_log = Instant::now();

    loop {
        let start = Instant::now();
        let dt = (start - last).as_secs_f64().min(0.5);
        last = start;

        let (out, ocfg, captive) = {
            let mut e = sh.engine.lock().unwrap();
            let out = e.step(dt, geo::unix_now_f64());
            (out, e.cfg.output.clone(), e.cfg.network.captive_portal)
        };
        sh.captive.store(captive, Ordering::Relaxed);

        if last_clients.elapsed() >= Duration::from_secs(2) {
            last_clients = Instant::now();
            let stations = net::ap_stations(&sh.wifi.lock().unwrap());
            let mut n = sh.net.lock().unwrap();
            for (mac, ip) in stations {
                if let Some(ip) = ip {
                    n.seen(ip, "dhcp", Some(&mac), None);
                }
            }
            n.prune(120.0);
        }

        for d in out {
            net::send(&sock, &d, &ocfg, &sh.net);
        }

        if last_log.elapsed() >= Duration::from_secs(30) {
            last_log = Instant::now();
            let n = sh.net.lock().unwrap();
            log::info!(
                "uptime {}s, {} device(s), {} datagrams sent, {} errors, heap {} B",
                sh.boot.elapsed().as_secs(),
                n.clients.len(),
                n.sent,
                n.errors,
                unsafe { esp_idf_svc::sys::esp_get_free_heap_size() }
            );
        }

        let el = start.elapsed();
        if el < tick {
            std::thread::sleep(tick - el);
        }
    }
}
