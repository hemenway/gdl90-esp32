//! gdl90-emu: GDL 90 ADS-B receiver emulator for driving an EFB over Wi-Fi as if it were flying.
#![allow(dead_code)]

mod actions;
mod app;
mod captive;
mod config;
mod engine;
mod fisb;
mod gdl90;
mod geo;
mod net;
mod recorder;
mod scenarios;
mod sim;
mod web;
mod wifi;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use serde_json::json;

use crate::config::Config;

#[derive(Parser)]
#[command(name = "gdl90-emu", version, about = "GDL 90 ADS-B receiver emulator (Wi-Fi/UDP) for EFB testing")]
struct Cli {
    #[command(subcommand)]
    cmd: Option<Cmd>,
    #[command(flatten)]
    serve: ServeArgs,
}

#[derive(Args, Clone, Default)]
struct ServeArgs {
    /// Control UI port
    #[arg(long, default_value_t = 8090)]
    port: u16,
    /// Serve the control UI on all interfaces (default: this PC only)
    #[arg(long)]
    lan: bool,
    /// Don't open the browser
    #[arg(long)]
    no_browser: bool,
    /// Don't auto-start the hotspot
    #[arg(long)]
    no_hotspot: bool,
    /// Device profile to start with (see `profiles`)
    #[arg(long)]
    profile: Option<String>,
    /// Scenario to start with (see `scenarios`)
    #[arg(long)]
    scenario: Option<String>,
    /// Load a saved config JSON
    #[arg(long)]
    config: Option<PathBuf>,
    /// Start with output stopped
    #[arg(long)]
    stopped: bool,
    /// Extra unicast target ip[:port] (repeatable)
    #[arg(long)]
    target: Vec<String>,
    /// Also send to 127.0.0.1 (iFly for Windows on this PC)
    #[arg(long)]
    localhost: bool,
}

#[derive(Subcommand)]
enum Cmd {
    /// Run the emulator with the web control UI (default)
    Serve(ServeArgs),
    /// List Wi-Fi adapters and which one would host the hotspot
    Adapters {
        /// Preferred description substring
        #[arg(long, default_value = "802.11n")]
        r#match: String,
    },
    /// Enable a disabled Wi-Fi adapter (UAC prompt). Defaults to the preferred adapter.
    EnableAdapter { pnp_id: Option<String> },
    /// Control the Mobile Hotspot on the preferred adapter
    Hotspot {
        /// start | stop | status | configure (write SSID/pass/band without starting)
        action: String,
        #[arg(long, default_value = "GDL90-SIM")]
        ssid: String,
        #[arg(long, default_value = "gdl90sim")]
        pass: String,
        /// auto | 2.4 | 5
        #[arg(long, default_value = "auto")]
        band: String,
        /// Adapter GUID (default: auto-pick)
        #[arg(long)]
        adapter: Option<String>,
    },
    /// Act as an EFB: receive GDL 90 on a UDP port and print decoded messages
    Listen {
        #[arg(long, default_value_t = 4000)]
        port: u16,
        /// Print every message (default: per-type summary every 5 s plus non-routine messages)
        #[arg(long)]
        all: bool,
        /// Stop after N seconds
        #[arg(long)]
        seconds: Option<u64>,
    },
    /// Decode an iFly .bin recording or a raw GDL 90 byte capture
    Decode { file: PathBuf },
    /// List device profiles
    Profiles,
    /// List prebuilt scenarios
    Scenarios,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        None => serve(cli.serve),
        Some(Cmd::Serve(a)) => serve(a),
        Some(Cmd::Adapters { r#match }) => {
            let list = wifi::scan(&r#match, "")?;
            let pick = wifi::pick(&list, "").map(|a| a.pnp_id.clone());
            for a in &list {
                let mark = if Some(&a.pnp_id) == pick.as_ref() { "=>" } else { "  " };
                println!(
                    "{mark} [{:>4}] {:<14} {:<64} {:<12} {}",
                    a.score,
                    a.name,
                    a.description,
                    if a.disabled() { "DISABLED" } else { &a.status },
                    a.note
                );
                println!("          guid={} pnp={}", a.guid, a.pnp_id);
            }
            if list.is_empty() {
                println!("No Wi-Fi adapters found.");
            }
            Ok(())
        }
        Some(Cmd::EnableAdapter { pnp_id }) => {
            let id = match pnp_id {
                Some(id) => id,
                None => {
                    let list = wifi::scan("802.11n", "")?;
                    wifi::pick(&list, "").context("no adapter found")?.pnp_id.clone()
                }
            };
            println!("Enabling {id} (accept the UAC prompt)...");
            wifi::enable_device(&id)?;
            println!("Done. Run `gdl90-emu adapters` to check.");
            Ok(())
        }
        Some(Cmd::Hotspot { action, ssid, pass, band, adapter }) => {
            // (guid, usable)
            let chosen: Option<(String, bool)> = match adapter {
                Some(g) => Some((g, true)),
                None => {
                    let list = wifi::scan("802.11n", "")?;
                    wifi::pick(&list, "").map(|a| {
                        println!("Adapter: {} ({}) {}", a.name, a.description, a.note);
                        (a.guid.clone(), a.usable())
                    })
                }
            };
            let usable_guid = chosen.as_ref().filter(|c| c.1).map(|c| c.0.clone());
            match action.as_str() {
                "start" => {
                    let g = usable_guid.context("no usable spare Wi-Fi adapter (disabled? try `gdl90-emu enable-adapter`, or pass --adapter <GUID>)")?;
                    println!("{}", wifi::hotspot::start(&g, &ssid, &pass, &band)?)
                }
                "configure" => {
                    wifi::hotspot::configure(usable_guid.as_deref(), &ssid, &pass, &band)?;
                    println!("Windows Mobile Hotspot settings: SSID '{ssid}', band {band} (not started)");
                }
                "stop" => println!("{}", wifi::hotspot::stop(usable_guid.as_deref())?),
                _ => println!("{}", serde_json::to_string_pretty(&wifi::hotspot::status(usable_guid.as_deref())?)?),
            }
            Ok(())
        }
        Some(Cmd::Listen { port, all, seconds }) => listen(port, all, seconds),
        Some(Cmd::Decode { file }) => decode(&file),
        Some(Cmd::Profiles) => {
            for p in config::PROFILES {
                println!("{:<12} {}\n             {}", p.id, p.name, p.description);
            }
            Ok(())
        }
        Some(Cmd::Scenarios) => {
            for s in scenarios::SCENARIOS {
                println!("{:<16} {}\n                 {}", s.id, s.name, s.description);
            }
            Ok(())
        }
    }
}

#[tokio::main]
async fn serve(a: ServeArgs) -> Result<()> {
    let mut cfg = match &a.config {
        Some(p) => serde_json::from_str::<Config>(&std::fs::read_to_string(p)?).with_context(|| format!("parse {}", p.display()))?,
        None => Config::default(),
    };
    if a.no_hotspot {
        cfg.network.auto_hotspot = false;
    }
    cfg.output.targets.extend(a.target.iter().cloned());
    if a.localhost {
        cfg.output.localhost = true;
    }
    let mut engine = engine::Engine::new(cfg);
    if let Some(s) = &a.scenario {
        engine.load_scenario(s, &scenarios::Anchor::default())?;
    }
    if let Some(p) = &a.profile {
        engine.apply_profile(p)?;
    }
    if a.stopped {
        engine.running = false;
    }

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let app = Arc::new(app::App {
        engine: Mutex::new(engine),
        net: Mutex::new(Default::default()),
        wifi: Mutex::new(Default::default()),
        tx,
        wifi_kick: Default::default(),
    });
    net::refresh_interfaces(&mut app.net.lock().unwrap());

    tokio::spawn(net::sender_task(app.clone(), rx));
    tokio::spawn(app::engine_loop(app.clone()));
    tokio::spawn(net::discovery_listeners(app.clone()));
    tokio::spawn(net::discovery_poller(app.clone()));
    if cfg!(windows) {
        tokio::spawn(app::wifi_manager(app.clone()));
    }

    let host = if a.lan { [0, 0, 0, 0] } else { [127, 0, 0, 1] };
    let mut listener = None;
    for p in a.port..a.port + 20 {
        if let Ok(l) = tokio::net::TcpListener::bind(SocketAddr::from((host, p))).await {
            listener = Some((l, p));
            break;
        }
    }
    let (listener, port) = listener.context("no free port for the control UI")?;
    let url = format!("http://127.0.0.1:{port}/");
    println!("GDL 90 emulator running. Control UI: {url}");
    println!("Ctrl+C to quit (stops a hotspot this program started).");
    if !a.no_browser {
        let _ = std::process::Command::new("cmd").args(["/C", "start", "", &url]).spawn();
    }

    let server = axum::serve(listener, web::router(app.clone()));
    tokio::select! {
        r = server => r?,
        _ = tokio::signal::ctrl_c() => {}
    }
    println!("Shutting down...");
    if let Ok(Some(p)) = app.engine.lock().unwrap().stop_recording() {
        println!("Recording saved: {p}");
    }
    let started = app.wifi.lock().unwrap().started_by_us;
    if started {
        match app::hotspot_stop(&app).await {
            Ok(m) => println!("{m}"),
            Err(e) => println!("Hotspot stop failed: {e}"),
        }
    }
    Ok(())
}

fn listen(port: u16, all: bool, seconds: Option<u64>) -> Result<()> {
    use std::collections::BTreeMap;
    use std::time::{Duration, Instant};
    let sock = std::net::UdpSocket::bind(("0.0.0.0", port)).with_context(|| format!("bind udp/{port}"))?;
    sock.set_read_timeout(Some(Duration::from_millis(250)))?;
    println!("Listening for GDL 90 on udp/{port} ...");
    let mut d = gdl90::Deframer::default();
    let mut counts: BTreeMap<&'static str, u64> = BTreeMap::new();
    let mut bad = 0u64;
    let mut datagrams = 0u64;
    let mut buf = vec![0u8; 65536];
    let start = Instant::now();
    let mut last = Instant::now();
    loop {
        if let Some(s) = seconds {
            if start.elapsed() >= Duration::from_secs(s) {
                break;
            }
        }
        if let Ok((n, from)) = sock.recv_from(&mut buf) {
            datagrams += 1;
            for f in d.push(&buf[..n]) {
                match f {
                    gdl90::Frame::Ok(m) => {
                        let name = gdl90::message_name(&m);
                        let first = !counts.contains_key(name);
                        *counts.entry(name).or_default() += 1;
                        if all || first || matches!(m[0], 0x0A) && counts[name] % 10 == 1 {
                            println!("{:>8.3} {from} {}", start.elapsed().as_secs_f64(), gdl90::describe(&m));
                        }
                    }
                    other => {
                        bad += 1;
                        if all || bad <= 5 {
                            println!("{:>8.3} {from} BAD {:?}", start.elapsed().as_secs_f64(), other);
                        }
                    }
                }
            }
        }
        if last.elapsed() >= Duration::from_secs(5) {
            last = Instant::now();
            println!("-- {datagrams} datagrams, bad frames {bad}, {:?}", counts);
        }
    }
    println!("== {datagrams} datagrams, bad frames {bad}");
    println!("{}", serde_json::to_string_pretty(&json!(counts))?);
    Ok(())
}

fn decode(path: &std::path::Path) -> Result<()> {
    let bytes = std::fs::read(path)?;
    let chunks: Vec<(u32, Vec<u8>)> = match recorder::parse(&bytes) {
        Ok(r) => {
            println!("iFly recording: brand {} , {} records, {} comments", r.brand, r.records.len(), r.comments.len());
            for c in &r.comments {
                println!("# {c}");
            }
            r.records
        }
        Err(_) => vec![(0, bytes)],
    };
    let mut d = gdl90::Deframer::default();
    let mut counts: std::collections::BTreeMap<&'static str, u64> = Default::default();
    for (ms, c) in chunks {
        for f in d.push(&c) {
            match f {
                gdl90::Frame::Ok(m) => {
                    *counts.entry(gdl90::message_name(&m)).or_default() += 1;
                    println!("{:>9.3}s {}", ms as f64 / 1000.0, gdl90::describe(&m));
                }
                other => println!("{:>9.3}s BAD {other:?}", ms as f64 / 1000.0),
            }
        }
    }
    println!("{}", serde_json::to_string_pretty(&json!(counts))?);
    Ok(())
}
