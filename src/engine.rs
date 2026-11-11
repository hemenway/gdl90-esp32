//! Flight model, protocol scheduler and decoded packet log.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::path::PathBuf;

use anyhow::{anyhow, bail, Result};
use rand::{rngs::SmallRng, Rng, SeedableRng};
use serde::Serialize;
use serde_json::{json, Value};

use crate::config::{self, Config, Stream};
use crate::fisb;
use crate::gdl90;
use crate::geo;
use crate::recorder::{self, Recorder};
use crate::scenarios::{self, Anchor, Event};
use crate::sim::{self, Behavior, History, Kin, NavState, Target};

/// One UDP payload to send. `port: None` = the configured GDL 90 ports.
#[derive(Debug, Clone)]
pub struct Datagram {
    pub bytes: Vec<u8>,
    pub port: Option<u16>,
}

#[derive(Serialize, Clone)]
pub struct LogEntry {
    pub seq: u64,
    pub t: f64,
    pub name: String,
    pub id: u8,
    pub len: usize,
    pub text: String,
    pub hex: String,
    pub flags: String,
}

#[derive(Serialize, Default, Clone)]
pub struct Stats {
    pub msgs: u64,
    pub bytes: u64,
    pub datagrams: u64,
    pub per_type: BTreeMap<String, u64>,
    pub msgs_per_s: f64,
    pub bytes_per_s: f64,
}

pub struct Replay {
    pub name: String,
    pub records: Vec<(u32, Vec<u8>)>,
    pub idx: usize,
    pub start: f64,
    pub speed: f64,
    pub looping: bool,
    deframer: gdl90::Deframer,
}

struct ScenarioRun {
    id: String,
    name: String,
    events: Vec<Event>,
    next: usize,
}

enum Out {
    /// Clear GDL 90 message (ID + data): framed before delivery.
    Msg(Vec<u8>),
    /// NMEA or simulator text datagram.
    Text { bytes: Vec<u8>, port: Option<u16>, label: &'static str },
}

pub struct Engine {
    pub cfg: Config,
    pub running: bool,
    pub paused: bool,
    pub own: Kin,
    pub nav: NavState,
    pub traffic: Vec<Target>,
    next_target_id: u32,
    history: History,
    pub sim_t: f64,
    scenario: Option<ScenarioRun>,
    scenario_t0: f64,
    sched: HashMap<&'static str, f64>,
    uplinks: Vec<[u8; fisb::PAYLOAD_LEN]>,
    uplink_idx: usize,
    uplink_built_at: f64,
    uplink_dirty: bool,
    uplinks_this_sec: u8,
    uplinks_last_sec: u8,
    cur_sec: i64,
    pub station: Option<(f64, f64)>,
    log: VecDeque<LogEntry>,
    log_seq: u64,
    pub stats: Stats,
    win_start: f64,
    win_msgs: u64,
    win_bytes: u64,
    rng: SmallRng,
    notices: VecDeque<(f64, String)>,
    pub replay: Option<Replay>,
    pub recorder: Option<Recorder>,
}

// The ESP32 has ~300 KB of heap; the PC has gigabytes.
#[cfg(not(target_os = "espidf"))]
mod caps {
    pub const LOG: usize = 2000;
    /// Ownship history for TIS-B shadows: sample period and length.
    pub const HISTORY_EVERY_S: f64 = 0.1;
    pub const HISTORY_S: f64 = 180.0;
    pub const UPLINK_PAYLOADS: usize = 200;
    pub const TRAFFIC: usize = 500;
}
#[cfg(target_os = "espidf")]
mod caps {
    pub const LOG: usize = 32;
    pub const HISTORY_EVERY_S: f64 = 0.5;
    pub const HISTORY_S: f64 = 30.0;
    pub const UPLINK_PAYLOADS: usize = 16;
    pub const TRAFFIC: usize = 40;
}
// The firmware sends column names once. Holding a BTreeMap of owned field names
// for every target would exhaust internal RAM in the busy-airspace scenario.
#[cfg(target_os = "espidf")]
const TRAFFIC_COLUMNS: &[&str] = &[
    "id", "callsign", "address", "addr_type", "emitter", "lat", "lon", "alt_ft", "track",
    "gs_kt", "vs_fpm", "on_ground", "range_nm", "bearing", "rel_alt", "alerting", "silent",
    "emergency", "behavior", "alert", "nic", "nacp", "track_type", "alt_invalid", "extrapolated",
];
const LOG_CAP: usize = caps::LOG;

impl Engine {
    pub fn new(cfg: Config) -> Self {
        let mut e = Engine {
            cfg,
            running: true,
            paused: false,
            own: Kin::default(),
            nav: NavState::default(),
            traffic: Vec::new(),
            next_target_id: 1,
            history: VecDeque::new(),
            sim_t: 0.0,
            scenario: None,
            scenario_t0: 0.0,
            sched: HashMap::new(),
            uplinks: Vec::new(),
            uplink_idx: 0,
            uplink_built_at: 0.0,
            uplink_dirty: true,
            uplinks_this_sec: 0,
            uplinks_last_sec: 0,
            cur_sec: 0,
            station: None,
            log: VecDeque::new(),
            log_seq: 0,
            stats: Stats::default(),
            win_start: 0.0,
            win_msgs: 0,
            win_bytes: 0,
            rng: SmallRng::from_entropy(),
            notices: VecDeque::new(),
            replay: None,
            recorder: None,
        };
        let _ = e.load_scenario("orbit", &Anchor::default());
        e
    }

    pub fn notice(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        eprintln!("[engine] {msg}");
        self.notices.push_back((geo::unix_now_f64(), msg));
        while self.notices.len() > 60 {
            self.notices.pop_front();
        }
    }

    // -----------------------------------------------------------------------------------------
    // Control
    // -----------------------------------------------------------------------------------------

    pub fn patch_config(&mut self, patch: &Value) -> Result<()> {
        let mut patch = patch.clone();
        if let Some(obj) = patch.as_object_mut() {
            if let Some(tp) = obj.remove("_traffic") {
                self.patch_traffic(&tp)?;
            }
        }
        let new = self.cfg.patched(&patch)?;
        if new.weather != self.cfg.weather {
            self.uplink_dirty = true;
        }
        if new.flight.route != self.cfg.flight.route || new.flight.mode != self.cfg.flight.mode {
            self.nav = NavState::default();
        }
        if new.weather.station_auto && !self.cfg.weather.station_auto {
            self.station = None;
        }
        self.cfg = new;
        Ok(())
    }

    fn patch_traffic(&mut self, tp: &Value) -> Result<()> {
        let obj = tp.as_object().ok_or_else(|| anyhow!("_traffic must be an object"))?;
        for (id, p) in obj {
            let id: u32 = id.parse()?;
            if let Some(t) = self.traffic.iter_mut().find(|t| t.id == id) {
                let mut v = serde_json::to_value(&*t)?;
                config::merge_patch(&mut v, p);
                *t = serde_json::from_value(v)?;
            }
        }
        Ok(())
    }

    pub fn apply_profile(&mut self, id: &str) -> Result<()> {
        let p = config::profile_patch(id).ok_or_else(|| anyhow!("unknown profile {id}"))?;
        self.patch_config(&p)?;
        self.sched.clear();
        self.notice(format!("Profile: {id}"));
        Ok(())
    }

    pub fn load_scenario(&mut self, id: &str, anchor: &Anchor) -> Result<()> {
        let icao = self.own_icao();
        let cs = self.cfg.ownship.callsign.clone();
        let plan = scenarios::build(id, anchor, icao, &cs, &mut self.rng).ok_or_else(|| anyhow!("unknown scenario {id}"))?;
        if let Some(p) = plan.profile {
            if let Some(pp) = config::profile_patch(p) {
                self.patch_config(&pp)?;
            }
        }
        // routes/arrays are replaced wholesale by the merge patch; clear first so an empty plan
        // route doesn't inherit the previous scenario's
        self.cfg.flight.route.clear();
        self.patch_config(&plan.patch)?;
        self.own = plan.start;
        self.nav = NavState::default();
        self.history.clear();
        self.traffic = plan.traffic;
        self.next_target_id = self.traffic.iter().map(|t| t.id).max().unwrap_or(0) + 1;
        let name = scenarios::SCENARIOS.iter().find(|s| s.id == id).map(|s| s.name).unwrap_or(id);
        self.scenario = Some(ScenarioRun { id: id.into(), name: name.into(), events: plan.events, next: 0 });
        self.scenario_t0 = self.sim_t;
        self.station = None;
        self.uplink_dirty = true;
        self.sched.clear();
        self.notice(format!("Scenario: {name}"));
        Ok(())
    }

    pub fn teleport(&mut self, v: &Value) {
        let g = |k: &str| v.get(k).and_then(|x| x.as_f64());
        if let (Some(lat), Some(lon)) = (g("lat"), g("lon")) {
            self.own.lat = lat;
            self.own.lon = lon;
        }
        if let Some(a) = g("alt_ft") {
            self.own.alt_ft = a;
            self.cfg.flight.target_alt_ft = a;
        }
        if let Some(t) = g("track") {
            self.own.track = t;
            self.own.heading = t;
            self.cfg.flight.target_track = t;
        }
        if let Some(s) = g("speed_kt") {
            self.own.tas_kt = s;
            self.own.gs_kt = s;
            self.cfg.flight.target_speed_kt = s;
        }
        if let Some(og) = v.get("on_ground").and_then(|x| x.as_bool()) {
            self.own.on_ground = og;
            if og {
                self.own.alt_ft = self.cfg.flight.ground_elev_ft;
            }
        }
        self.history.clear();
    }

    pub fn add_traffic(&mut self, v: &Value) -> Result<Vec<u32>> {
        let preset = v.get("preset").and_then(|x| x.as_str()).unwrap_or("random");
        let room = caps::TRAFFIC.saturating_sub(self.traffic.len());
        if room == 0 {
            bail!("traffic limit reached ({} targets)", caps::TRAFFIC);
        }
        let count = (v.get("count").and_then(|x| x.as_u64()).unwrap_or(1) as usize).clamp(1, room);
        let own = self.own.clone();
        let mut ids = Vec::new();
        for _ in 0..count {
            let id = self.next_target_id;
            self.next_target_id += 1;
            let mut t = Target {
                id,
                address: scenarios::random_icao(&mut self.rng),
                callsign: scenarios::random_tail(&mut self.rng),
                ..Default::default()
            };
            match preset {
                "converging" | "head_on" | "crossing" | "overtake" => {
                    let rel = v.get("rel_bearing").and_then(|x| x.as_f64()).unwrap_or(match preset {
                        "crossing" => 90.0,
                        "overtake" => 180.0,
                        _ => 0.0,
                    });
                    let speed = v.get("speed_kt").and_then(|x| x.as_f64()).unwrap_or(if preset == "overtake" { own.gs_kt + 50.0 } else { 130.0 });
                    let cpa = v.get("cpa_s").and_then(|x| x.as_f64()).unwrap_or(60.0);
                    let dalt = v.get("alt_offset_ft").and_then(|x| x.as_f64()).unwrap_or(0.0);
                    let mut o = own.clone();
                    if o.gs_kt < 1.0 {
                        o.gs_kt = 1.0;
                    }
                    t.kin = sim::converging(&o, rel, speed, cpa, dalt);
                }
                "shadow" => {
                    t.address = self.own_icao();
                    t.addr_type = 2;
                    t.callsign = String::new();
                    t.kin = own.clone();
                    t.behavior = Behavior::Shadow {
                        delay_s: v.get("delay_s").and_then(|x| x.as_f64()).unwrap_or(1.5),
                        offset_nm: v.get("offset_nm").and_then(|x| x.as_f64()).unwrap_or(0.05),
                        offset_brg: 180.0,
                        alt_offset_ft: 0.0,
                    };
                }
                "at" => {
                    let lat = v.get("lat").and_then(|x| x.as_f64()).unwrap_or(own.lat);
                    let lon = v.get("lon").and_then(|x| x.as_f64()).unwrap_or(own.lon);
                    let alt = v.get("alt_ft").and_then(|x| x.as_f64()).unwrap_or(own.alt_ft);
                    let trk = v.get("track").and_then(|x| x.as_f64()).unwrap_or(self.rng.gen_range(0.0..360.0));
                    let spd = v.get("speed_kt").and_then(|x| x.as_f64()).unwrap_or(110.0);
                    let og = v.get("on_ground").and_then(|x| x.as_bool()).unwrap_or(false);
                    t.kin = Kin::at(lat, lon, alt, trk, spd, og);
                    if og || spd == 0.0 {
                        t.behavior = Behavior::Static;
                    }
                }
                "orbit" => {
                    let lat = v.get("lat").and_then(|x| x.as_f64()).unwrap_or(own.lat);
                    let lon = v.get("lon").and_then(|x| x.as_f64()).unwrap_or(own.lon);
                    let r = v.get("radius_nm").and_then(|x| x.as_f64()).unwrap_or(2.0);
                    let (slat, slon) = geo::destination(lat, lon, 0.0, r);
                    t.kin = Kin::at(slat, slon, own.alt_ft + 500.0, 90.0, 100.0, false);
                    t.behavior = Behavior::Orbit { lat, lon, radius_nm: r, clockwise: true };
                }
                _ => {
                    let radius = v.get("radius_nm").and_then(|x| x.as_f64()).unwrap_or(15.0);
                    let brg = self.rng.gen_range(0.0..360.0);
                    let dist = self.rng.gen_range(0.5..radius.max(1.0));
                    let (lat, lon) = geo::destination(own.lat, own.lon, brg, dist);
                    let alt = (own.alt_ft + self.rng.gen_range(-3000.0..3000.0)).max(self.cfg.flight.ground_elev_ft + 500.0);
                    t.kin = Kin::at(lat, lon, (alt / 100.0).round() * 100.0, self.rng.gen_range(0.0..360.0), self.rng.gen_range(70.0..250.0), false);
                    t.addr_type = if self.rng.gen_bool(0.3) { 2 } else { 0 };
                }
            }
            if let Some(p) = v.get("patch") {
                let mut tv = serde_json::to_value(&t)?;
                config::merge_patch(&mut tv, p);
                t = serde_json::from_value(tv)?;
                t.id = id;
            }
            ids.push(id);
            self.traffic.push(t);
        }
        Ok(ids)
    }

    pub fn remove_traffic(&mut self, id: u32) {
        self.traffic.retain(|t| t.id != id);
    }

    pub fn clear_traffic(&mut self) {
        self.traffic.clear();
    }

    pub fn start_recording(&mut self, path: Option<String>) -> Result<String> {
        if self.recorder.is_some() {
            bail!("already recording");
        }
        let now = geo::unix_now_f64();
        let (y, mo, d, h, mi, _) = geo::utc_fields(now);
        let path = match path {
            Some(p) if !p.trim().is_empty() => PathBuf::from(p),
            _ => PathBuf::from(format!("recordings/Adsb_{y:04}_{mo:02}_{d:02}-{h:02}.{mi:02}.bin")),
        };
        let r = Recorder::create(&path, recorder::brand_for_profile(&self.cfg.profile), now)?;
        let p = r.path.clone();
        self.recorder = Some(r);
        self.notice(format!("Recording to {p}"));
        Ok(p)
    }

    pub fn stop_recording(&mut self) -> Result<Option<String>> {
        match self.recorder.take() {
            Some(r) => {
                let n = r.records;
                let p = r.finish()?;
                self.notice(format!("Recording saved: {p} ({n} records)"));
                Ok(Some(p))
            }
            None => Ok(None),
        }
    }

    pub fn start_replay(&mut self, name: String, rec: recorder::Recording, speed: f64, looping: bool) {
        let n = rec.records.len();
        self.replay = Some(Replay {
            name: name.clone(),
            records: rec.records,
            idx: 0,
            start: geo::unix_now_f64(),
            speed: speed.clamp(0.05, 50.0),
            looping,
            deframer: Default::default(),
        });
        self.notice(format!("Replaying {name} ({n} records, brand {}); simulation output paused", rec.brand));
    }

    // -----------------------------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------------------------

    pub fn own_icao(&self) -> u32 {
        u32::from_str_radix(self.cfg.ownship.icao.trim().trim_start_matches("0x"), 16).unwrap_or(0) & 0xFF_FFFF
    }

    fn gps_ok(&self) -> bool {
        self.cfg.ownship.gps_valid
    }

    fn pressure_alt(&self, msl: f64) -> f64 {
        msl + (29.92 - self.cfg.flight.altimeter_inhg) * 1000.0
    }

    fn own_report(&self) -> gdl90::Report {
        let o = &self.cfg.ownship;
        let k = &self.own;
        let gps = self.gps_ok();
        let heading_mag = (k.heading - self.cfg.flight.mag_var_deg).rem_euclid(360.0);
        gdl90::Report {
            alert: o.alert,
            addr_type: o.addr_type,
            address: self.own_icao(),
            lat: if gps { k.lat } else { 0.0 },
            lon: if gps { k.lon } else { 0.0 },
            pressure_alt_ft: if o.alt_invalid { None } else { Some(self.pressure_alt(k.alt_ft)) },
            airborne: match o.air_ground.as_str() {
                "air" => true,
                "ground" => false,
                _ => !k.on_ground,
            },
            extrapolated: o.extrapolated,
            track_type: o.track_type,
            nic: if gps { o.nic } else { 0 },
            nacp: if gps { o.nacp } else { 0 },
            h_vel_kt: if o.hvel_invalid { None } else { Some(k.gs_kt) },
            v_vel_fpm: if o.vvel_invalid { None } else { Some(k.vs_fpm) },
            track_deg: match o.track_type {
                2 => heading_mag,
                3 => k.heading,
                _ => k.track,
            },
            emitter: o.emitter,
            callsign: o.callsign.clone(),
            emergency: o.emergency,
            spare: o.spare,
        }
    }

    fn target_alerting(&self, t: &Target) -> bool {
        match t.alert.as_str() {
            "on" => true,
            "off" => false,
            _ => {
                let own = &self.own;
                geo::distance_nm(own.lat, own.lon, t.kin.lat, t.kin.lon) < 1.0 && (own.alt_ft - t.kin.alt_ft).abs() < 600.0
            }
        }
    }

    fn target_report(&self, t: &Target) -> gdl90::Report {
        gdl90::Report {
            alert: self.target_alerting(t) as u8,
            addr_type: t.addr_type,
            address: t.address,
            lat: t.kin.lat,
            lon: t.kin.lon,
            pressure_alt_ft: if t.alt_invalid { None } else { Some(self.pressure_alt(t.kin.alt_ft)) },
            airborne: !t.kin.on_ground,
            extrapolated: t.extrapolated,
            track_type: t.track_type,
            nic: t.nic,
            nacp: t.nacp,
            h_vel_kt: Some(t.kin.gs_kt),
            v_vel_fpm: Some(t.kin.vs_fpm),
            track_deg: t.kin.track,
            emitter: t.emitter,
            callsign: t.callsign.clone(),
            emergency: t.emergency,
            spare: 0,
        }
    }

    /// (roll, pitch, heading) as the AHRS reports them.
    fn attitude(&self) -> (f64, f64, f64) {
        let a = &self.cfg.ahrs;
        let k = &self.own;
        let roll = k.roll * a.roll_sign + a.roll_offset;
        let pitch = k.pitch + a.pitch_offset;
        let mut hdg = k.heading + a.heading_offset;
        if a.heading_magnetic {
            hdg -= self.cfg.flight.mag_var_deg;
        }
        (roll, pitch, hdg.rem_euclid(360.0))
    }

    fn ahrs_valid(&self) -> bool {
        self.cfg.ahrs.valid
    }

    fn station_pos(&mut self) -> (f64, f64) {
        let w = &self.cfg.weather;
        if !w.station_auto {
            return (w.station_lat, w.station_lon);
        }
        if self.station.is_none() {
            let off = w.station_offset_nm;
            self.station = Some(geo::destination(self.own.lat, self.own.lon, 45.0, off));
        }
        self.station.unwrap()
    }

    fn expand_tokens(s: &str, utc: f64) -> String {
        let now = geo::ddhhmmz(utc);
        let (_, _, d, h, _, _) = geo::utc_fields(utc);
        let (_, _, d2, h2, _, _) = geo::utc_fields(utc + 24.0 * 3600.0);
        let (_, _, d3, h3, _, _) = geo::utc_fields(utc + 6.0 * 3600.0);
        s.replace("{TIME}", &now)
            .replace("{VALID}", &format!("{d:02}{h:02}/{d2:02}{h2:02}"))
            .replace("{FM}", &format!("{d3:02}{h3:02}00"))
    }

    fn build_uplinks(&mut self, utc: f64) {
        let (lat, lon) = self.station_pos();
        let w = self.cfg.weather.clone();
        let (_, _, _, hh, mm, _) = geo::utc_fields(utc);
        let header = fisb::UplinkHeader {
            lat,
            lon,
            position_valid: true,
            utc_coupled: true,
            app_data_valid: true,
            slot_id: 5,
            tisb_site_id: w.tisb_site_id,
        };
        let mut frames = Vec::new();
        for line in w.text.iter().filter(|l| !l.trim().is_empty()) {
            let text = Self::expand_tokens(line.trim(), utc);
            let rec = fisb::text_record(&text);
            if rec.len() <= 418 {
                frames.push(fisb::info_frame(0, &fisb::apdu(413, fisb::ApduTime::hm(hh, mm), &rec)));
            }
        }
        if !w.cells.is_empty() {
            let t = fisb::ApduTime::hm(hh, mm - mm % 2);
            frames.extend(fisb::nexrad_frames(w.nexrad_product, t, &w.cells));
        }
        self.uplinks = fisb::pack_payloads(&header, &frames);
        if self.uplinks.len() > caps::UPLINK_PAYLOADS {
            let n = self.uplinks.len();
            self.uplinks.truncate(caps::UPLINK_PAYLOADS);
            self.notice(format!("FIS-B: {n} uplink payloads needed, sending the first {} (memory cap)", caps::UPLINK_PAYLOADS));
        }
        self.uplink_idx = 0;
        self.uplink_dirty = false;
    }

    fn due(sched: &mut HashMap<&'static str, f64>, key: &'static str, s: &Stream, now: f64) -> bool {
        if !s.enabled || s.rate_hz <= 0.0 {
            sched.remove(key);
            return false;
        }
        let period = 1.0 / s.rate_hz.min(200.0);
        let next = sched.entry(key).or_insert_with(|| if period >= 1.0 { now.ceil() } else { now });
        if now + 1e-6 >= *next {
            *next += period;
            if *next < now {
                *next = if period >= 1.0 { now.floor() + period } else { now + period };
            }
            true
        } else {
            false
        }
    }

    fn nmea_sentences(&self, utc: f64) -> Vec<u8> {
        let k = &self.own;
        let gps = self.gps_ok();
        let (y, mo, d, h, mi, s) = geo::utc_fields(utc);
        let frac = ((utc.fract()) * 100.0) as u32;
        let fmt_lat = |v: f64| {
            let a = v.abs();
            format!("{:02}{:07.4},{}", a.trunc() as u32, a.fract() * 60.0, if v >= 0.0 { 'N' } else { 'S' })
        };
        let fmt_lon = |v: f64| {
            let a = v.abs();
            format!("{:03}{:07.4},{}", a.trunc() as u32, a.fract() * 60.0, if v >= 0.0 { 'E' } else { 'W' })
        };
        let cs = |body: &str| body.bytes().fold(0u8, |a, b| a ^ b);
        let rmc = format!(
            "GPRMC,{h:02}{mi:02}{s:02}.{frac:02},{},{},{},{:.1},{:.1},{d:02}{mo:02}{:02},{:.1},{},{}",
            if gps { 'A' } else { 'V' },
            fmt_lat(k.lat),
            fmt_lon(k.lon),
            k.gs_kt,
            k.track,
            y % 100,
            self.cfg.flight.mag_var_deg.abs(),
            if self.cfg.flight.mag_var_deg >= 0.0 { 'E' } else { 'W' },
            if self.cfg.vendor.waas { 'D' } else { 'A' }
        );
        let gga = format!(
            "GPGGA,{h:02}{mi:02}{s:02}.{frac:02},{},{},{},{:02},0.8,{:.1},M,{:.1},M,,",
            fmt_lat(k.lat),
            fmt_lon(k.lon),
            if !gps { 0 } else if self.cfg.vendor.waas { 2 } else { 1 },
            self.cfg.vendor.sats_locked,
            k.alt_ft * 0.3048,
            self.cfg.flight.geoid_sep_ft * 0.3048
        );
        format!("${rmc}*{:02X}\r\n${gga}*{:02X}\r\n", cs(&rmc), cs(&gga)).into_bytes()
    }

    // -----------------------------------------------------------------------------------------
    // Tick
    // -----------------------------------------------------------------------------------------

    pub fn step(&mut self, dt_real: f64, now: f64) -> Vec<Datagram> {
        self.roll_rates(now);
        let mut out = Vec::new();
        if !self.running {
            return out;
        }
        if self.replay.is_some() {
            self.step_replay(now, &mut out);
            return out;
        }

        if !self.paused {
            let dt = dt_real * self.cfg.flight.time_scale.clamp(0.0, 50.0);
            self.sim_t += dt;
            self.run_events();
            sim::step_ownship(&mut self.own, &self.cfg.flight, &mut self.nav, dt);
            let last = self.history.back().map(|(t, _)| *t).unwrap_or(f64::MIN);
            if self.sim_t - last >= caps::HISTORY_EVERY_S || self.sim_t < last {
                self.history.push_back((self.sim_t, self.own.clone()));
            }
            while self.history.front().map(|(t, _)| *t < self.sim_t - caps::HISTORY_S).unwrap_or(false) {
                self.history.pop_front();
            }
            let sim_t = self.sim_t;
            let hist = std::mem::take(&mut self.history);
            for t in &mut self.traffic {
                sim::step_target(t, dt, sim_t, &hist);
            }
            self.history = hist;
        }

        let utc = now;
        let sec = utc.floor() as i64;
        if sec != self.cur_sec {
            self.cur_sec = sec;
            self.uplinks_last_sec = self.uplinks_this_sec;
            self.uplinks_this_sec = 0;
        }

        let mut outs: Vec<Out> = Vec::new();
        self.emit_streams(now, utc, &mut outs);
        self.finish_tick(now, outs, &mut out);
        out
    }

    fn run_events(&mut self) {
        let elapsed = self.sim_t - self.scenario_t0;
        loop {
            let ev = match &mut self.scenario {
                Some(s) if s.next < s.events.len() && s.events[s.next].at_s <= elapsed => {
                    s.next += 1;
                    s.events[s.next - 1].clone()
                }
                _ => break,
            };
            if let Err(e) = self.patch_config(&ev.patch) {
                self.notice(format!("Event '{}' failed: {e}", ev.label));
            } else {
                self.notice(format!("T+{:.0}s {}", ev.at_s, ev.label));
            }
        }
    }

    fn emit_streams(&mut self, now: f64, utc: f64, outs: &mut Vec<Out>) {
        let st = self.cfg.streams.clone();
        let gps = self.gps_ok();
        let sched = &mut self.sched;
        let mut due = |key: &'static str, s: &Stream| Self::due(sched, key, s, now);
        let d_hb = due("heartbeat", &st.heartbeat);
        let d_shb = due("stratux_heartbeat", &st.stratux_heartbeat);
        let d_sx = due("stratux_status", &st.stratux_status);
        let d_ffid = due("ff_id", &st.ff_id);
        let d_sky = due("skyradar_status", &st.skyradar_status);
        let d_cls = due("clarity_status", &st.clarity_status);
        let d_pfs = due("pathfinder_status", &st.pathfinder_status);
        let d_own = due("ownship", &st.ownship);
        let d_geo = due("geo_alt", &st.geo_alt);
        let d_hat = due("hat", &st.hat);
        let d_trf = due("traffic", &st.traffic);
        let d_ffa = due("ff_ahrs", &st.ff_ahrs);
        let d_le = due("levil_ahrs", &st.levil_ahrs);
        let d_cla = due("clarity_ahrs", &st.clarity_ahrs);
        let d_xg = due("xgps_ahrs", &st.xgps_ahrs);
        let d_pfa = due("pathfinder_ahrs", &st.pathfinder_ahrs);
        let d_up = due("uplink", &st.uplink);
        let d_xpg = due("xplane_gps", &st.xplane_gps);
        let d_xpa = due("xplane_att", &st.xplane_att);
        let d_xpt = due("xplane_traffic", &st.xplane_traffic);
        let d_nmea = due("nmea", &st.nmea);

        let live_traffic = self.traffic.iter().filter(|t| !t.silent).count();
        let secs = utc.rem_euclid(86_400.0) as u32;
        let (roll, pitch, hdg) = self.attitude();
        let ahrs_ok = self.ahrs_valid();
        let v = self.cfg.vendor.clone();

        if d_hb {
            outs.push(Out::Msg(gdl90::heartbeat(&self.cfg.heartbeat, gps, secs, self.uplinks_last_sec, live_traffic.min(1023) as u16)));
        }
        if d_shb {
            outs.push(Out::Msg(gdl90::stratux_heartbeat(gps, ahrs_ok, v.stratux_protocol)));
        }
        if d_sx {
            let (slat, slon) = self.station_pos();
            let mut flags = 1 << 2 | 1 << 4 | 1 << 5 | 1 << 6;
            if ahrs_ok {
                flags |= 0b11;
            }
            let s = gdl90::StratuxStatus {
                version: v.stratux_version,
                gps_fix: if gps { if v.waas { 2 } else { 1 } } else { 0 },
                flags,
                sats_locked: v.sats_locked,
                sats_tracked: v.sats_tracked,
                uat_targets: live_traffic as u16,
                es_targets: 0,
                uat_msgs_min: (live_traffic as u16).saturating_mul(60),
                es_msgs_min: 0,
                cpu_temp_c: v.cpu_temp_c,
                towers: if st.uplink.enabled { vec![(slat, slon)] } else { vec![] },
            };
            outs.push(Out::Msg(gdl90::stratux_status(&s)));
        }
        if d_ffid {
            let f = &self.cfg.foreflight;
            let serial = u64::from_str_radix(f.serial_hex.trim(), 16).ok();
            let caps = (f.geo_alt_msl as u32) | ((f.internet_policy as u32 & 3) << 1);
            outs.push(Out::Msg(gdl90::foreflight_id(serial, &f.name, &f.long_name, caps)));
        }
        if d_sky {
            let fix = if !gps { b'0' } else if v.waas { b'2' } else { b'1' };
            outs.push(Out::Msg(vec![0x65, v.skyradar_fw[0], v.skyradar_fw[1], fix, 0, 0, 0, 0]));
        }
        if d_cls {
            let b4 = if gps { (3 << 4) | if v.waas { 1 << 2 } else { 0 } } else { 0 };
            outs.push(Out::Msg(vec![0x3C, 0x01, 0x00, 0x00, b4, 0x00, 0x00, 0x00]));
        }
        if d_pfs {
            outs.push(Out::Msg(vec![
                0x46, 0x00, 0x01, 0x00, 0x00,
                (v.pathfinder_fw >> 8) as u8, v.pathfinder_fw as u8,
                0x00, v.pathfinder_battery, v.pathfinder_charge, 0x00, 0x00,
            ]));
        }
        if d_own {
            outs.push(Out::Msg(gdl90::report(gdl90::id::OWNSHIP, &self.own_report())));
        }
        if d_geo && gps {
            let o = &self.cfg.ownship;
            let k = &self.own;
            let alt = k.alt_ft + if o.geo_alt_datum == "msl" { 0.0 } else { self.cfg.flight.geoid_sep_ft };
            let vfom = if o.vfom_m < 0 { None } else { Some(o.vfom_m as u32) };
            outs.push(Out::Msg(gdl90::ownship_geo_alt(alt, o.vertical_warning, vfom)));
        }
        if d_hat {
            let hat = self.cfg.ownship.hat_ft.unwrap_or((self.own.alt_ft - self.cfg.flight.ground_elev_ft).round() as i32);
            outs.push(Out::Msg(gdl90::height_above_terrain(Some(hat.clamp(-32767, 32767) as i16))));
        }
        if d_trf {
            for t in self.traffic.iter().filter(|t| !t.silent) {
                outs.push(Out::Msg(gdl90::report(gdl90::id::TRAFFIC, &self.target_report(t))));
            }
        }
        if d_ffa {
            let a = gdl90::ForeFlightAhrs {
                roll: ahrs_ok.then_some(roll),
                pitch: ahrs_ok.then_some(pitch),
                heading: ahrs_ok.then_some((hdg, self.cfg.ahrs.heading_magnetic)),
                ias: (ahrs_ok && self.cfg.ahrs.send_airspeed).then_some(self.own.ias_kt),
                tas: (ahrs_ok && self.cfg.ahrs.send_airspeed).then_some(self.own.tas_kt),
            };
            outs.push(Out::Msg(gdl90::foreflight_ahrs(&a)));
        }
        if d_le {
            let a = if ahrs_ok {
                gdl90::LevilAhrs {
                    roll: Some(roll),
                    pitch: Some(pitch),
                    heading: Some(hdg),
                    slip_skid: Some(self.own.slip),
                    yaw_rate: Some(self.own.turn_rate),
                    g_load: Some(self.own.g_load),
                    ias: self.cfg.ahrs.send_airspeed.then_some(self.own.ias_kt),
                    palt: Some(self.pressure_alt(self.own.alt_ft)),
                    vs: Some(self.own.vs_fpm),
                }
            } else {
                gdl90::LevilAhrs::default()
            };
            outs.push(Out::Msg(gdl90::levil_ahrs(&a)));
        }
        if d_cla {
            let (r, p) = if ahrs_ok { (roll as f32, pitch as f32) } else { (f32::NAN, f32::NAN) };
            let mut m = vec![0x3D, 0x00, 0x00, 0x00];
            m.extend_from_slice(&r.to_be_bytes());
            m.extend_from_slice(&p.to_be_bytes());
            m.extend_from_slice(&(hdg as f32).to_be_bytes());
            m.resize(32, 0);
            outs.push(Out::Msg(m));
        }
        if d_xg {
            let enc = |v: f64| if ahrs_ok { ((v * 10.0).round() as i16).to_be_bytes() } else { [0x7F, 0xFF] };
            let mut m = vec![0x44, 0x00, 0x00];
            m.extend_from_slice(&enc(roll));
            m.extend_from_slice(&enc(pitch));
            m.extend_from_slice(&enc(hdg));
            m.resize(16, 0);
            outs.push(Out::Msg(m));
        }
        if d_pfa {
            let mut m = vec![0x46, 0x00, 0x02, 0x00];
            m.extend_from_slice(&((pitch * 364.08).round() as i16).to_le_bytes());
            m.extend_from_slice(&((roll * 182.04).round() as i16).to_le_bytes());
            m.resize(12, 0);
            outs.push(Out::Msg(m));
        }
        if d_up {
            let rebuild = self.uplink_dirty || now - self.uplink_built_at > self.cfg.weather.rebuild_s.max(5.0);
            if rebuild {
                self.build_uplinks(utc);
                self.uplink_built_at = now;
            }
            if !self.uplinks.is_empty() {
                let p = self.uplinks[self.uplink_idx % self.uplinks.len()];
                self.uplink_idx = (self.uplink_idx + 1) % self.uplinks.len();
                let tor = ((utc.fract() * 12_500_000.0) as u32).min(12_499_999);
                outs.push(Out::Msg(gdl90::uplink(tor, &p)));
                self.uplinks_this_sec = self.uplinks_this_sec.saturating_add(1);
            }
        }
        let xp = self.cfg.output.xplane_port;
        let name = v.xplane_sim_name.clone();
        if d_xpg {
            let k = &self.own;
            if gps {
                let s = format!("XGPS{name},{:.6},{:.6},{:.1},{:.2},{:.2}", k.lon, k.lat, k.alt_ft * 0.3048, k.track, k.gs_kt * geo::KT_TO_MPS);
                outs.push(Out::Text { bytes: s.into_bytes(), port: Some(xp), label: "XGPS" });
            }
        }
        if d_xpa && ahrs_ok {
            let s = format!("XATT{name},{:.1},{:.1},{:.1}", (self.own.heading + self.cfg.ahrs.heading_offset).rem_euclid(360.0), pitch, roll);
            outs.push(Out::Text { bytes: s.into_bytes(), port: Some(xp), label: "XATT" });
        }
        if d_xpt {
            for t in self.traffic.iter().filter(|t| !t.silent) {
                let s = format!(
                    "XTRAFFIC{name},{},{:.5},{:.5},{:.0},{:.0},{},{:.1},{:.1},{}",
                    t.address, t.kin.lat, t.kin.lon, t.kin.alt_ft, t.kin.vs_fpm, !t.kin.on_ground as u8, t.kin.track, t.kin.gs_kt, t.callsign
                );
                outs.push(Out::Text { bytes: s.into_bytes(), port: Some(xp), label: "XTRAFFIC" });
            }
        }
        if d_nmea {
            let np = self.cfg.output.nmea_port;
            outs.push(Out::Text { bytes: self.nmea_sentences(utc), port: Some(np), label: "NMEA" });
        }
    }

    fn push_log(&mut self, t: f64, name: &str, msg: &[u8], text: String, flags: &str) {
        self.log_seq += 1;
        let shown = &msg[..msg.len().min(64)];
        let mut hex = gdl90::hex(shown);
        if msg.len() > 64 {
            hex.push_str(" …");
        }
        self.log.push_back(LogEntry {
            seq: self.log_seq,
            t,
            name: name.to_string(),
            id: msg.first().copied().unwrap_or(0),
            len: msg.len(),
            text,
            hex,
            flags: flags.to_string(),
        });
        while self.log.len() > LOG_CAP {
            self.log.pop_front();
        }
    }

    fn finish_tick(&mut self, now: f64, outs: Vec<Out>, out: &mut Vec<Datagram>) {
        if outs.is_empty() {
            return;
        }
        let mut bundled = Vec::new();
        for item in outs {
            match item {
                Out::Text { bytes, port, label } => {
                    let text = String::from_utf8_lossy(&bytes[..bytes.len().min(120)]).trim().to_owned();
                    self.count(label, bytes.len());
                    self.push_log(now, label, &bytes, text, "");
                    out.push(Datagram { bytes, port });
                }
                Out::Msg(msg) => {
                    let name = gdl90::message_name(&msg);
                    let frame = gdl90::frame(&msg);
                    self.count(name, frame.len());
                    self.push_log(now, name, &msg, gdl90::describe(&msg), "");
                    if self.cfg.output.per_message_datagrams {
                        out.push(Datagram { bytes: frame, port: None });
                    } else {
                        if bundled.len() + frame.len() > 1400 && !bundled.is_empty() {
                            out.push(Datagram { bytes: std::mem::take(&mut bundled), port: None });
                        }
                        bundled.extend(frame);
                    }
                }
            }
        }
        if !bundled.is_empty() {
            out.push(Datagram { bytes: bundled, port: None });
        }
        // recording: GDL 90 datagrams only (what iFly's recorder would have seen on the port)
        if let Some(r) = &mut self.recorder {
            for d in out.iter().filter(|d| d.port.is_none()) {
                if let Err(e) = r.write(now, &d.bytes) {
                    eprintln!("[recorder] {e}");
                }
            }
        }
    }

    fn count(&mut self, name: &str, bytes: usize) {
        self.stats.msgs += 1;
        self.stats.bytes += bytes as u64;
        self.win_msgs += 1;
        self.win_bytes += bytes as u64;
        *self.stats.per_type.entry(name.to_string()).or_default() += 1;
    }

    fn roll_rates(&mut self, now: f64) {
        let el = now - self.win_start;
        if el >= 1.0 {
            self.stats.msgs_per_s = self.win_msgs as f64 / el;
            self.stats.bytes_per_s = self.win_bytes as f64 / el;
            self.win_msgs = 0;
            self.win_bytes = 0;
            self.win_start = now;
        }
    }

    fn step_replay(&mut self, now: f64, out: &mut Vec<Datagram>) {
        let mut logs = Vec::new();
        let mut finished = false;
        if let Some(rp) = &mut self.replay {
            let elapsed_ms = ((now - rp.start) * 1000.0 * rp.speed) as u32;
            while rp.idx < rp.records.len() && rp.records[rp.idx].0 <= elapsed_ms {
                for frame in rp.deframer.push(&rp.records[rp.idx].1) {
                    if let gdl90::Frame::Ok(msg) = &frame {
                        if gdl90::standard_replay_message(msg) {
                            out.push(Datagram { bytes: gdl90::frame(msg), port: None });
                        }
                    }
                    logs.push(frame);
                }
                rp.idx += 1;
            }
            if rp.idx >= rp.records.len() {
                if rp.looping {
                    rp.deframer = Default::default();
                    rp.idx = 0;
                    rp.start = now;
                } else {
                    finished = true;
                }
            }
        }
        for d in out.iter() {
            self.count("replay", d.bytes.len());
        }
        for fr in logs {
            match fr {
                gdl90::Frame::Ok(m) => {
                    let name = gdl90::message_name(&m);
                    self.push_log(now, name, &m, gdl90::describe(&m), "REPLAY");
                }
                gdl90::Frame::BadCrc { msg, .. } => self.push_log(now, "BadCRC", &msg, "frame with bad FCS".into(), "REPLAY,BADCRC"),
                gdl90::Frame::TooShort(m) => self.push_log(now, "Short", &m, "short frame".into(), "REPLAY"),
            }
        }
        if finished {
            self.replay = None;
            self.notice("Replay finished; simulation output resumed");
        }
    }

    // -----------------------------------------------------------------------------------------
    // Views
    // -----------------------------------------------------------------------------------------

    pub fn log_since(&self, seq: u64, max: usize) -> Vec<LogEntry> {
        let mut entries: Vec<_> = self.log.iter().rev().take_while(|e| e.seq > seq).take(max).cloned().collect();
        entries.reverse();
        entries
    }

    fn traffic_row(&self, t: &Target) -> Value {
        let own = &self.own;
        let r = geo::distance_nm(own.lat, own.lon, t.kin.lat, t.kin.lon);
        let b = geo::bearing_deg(own.lat, own.lon, t.kin.lat, t.kin.lon);
        let row = json!({
            "id": t.id, "callsign": t.callsign, "address": format!("{:06X}", t.address),
            "addr_type": t.addr_type, "emitter": t.emitter, "lat": t.kin.lat, "lon": t.kin.lon,
            "alt_ft": t.kin.alt_ft, "track": t.kin.track, "gs_kt": t.kin.gs_kt, "vs_fpm": t.kin.vs_fpm,
            "on_ground": t.kin.on_ground, "range_nm": r, "bearing": b, "rel_alt": t.kin.alt_ft - own.alt_ft,
            "alerting": self.target_alerting(t), "silent": t.silent, "emergency": t.emergency,
            "behavior": serde_json::to_value(&t.behavior).ok().and_then(|v| v.get("kind").cloned()),
            "alert": t.alert, "nic": t.nic, "nacp": t.nacp, "track_type": t.track_type,
            "alt_invalid": t.alt_invalid, "extrapolated": t.extrapolated,
        });
        #[cfg(target_os = "espidf")]
        {
            let mut row = row;
            Value::Array(TRAFFIC_COLUMNS.iter().map(|&k| row[k].take()).collect())
        }
        #[cfg(not(target_os = "espidf"))]
        row
    }

    fn snapshot_fields(&self) -> Value {
        let own = &self.own;
        let scenario = self.scenario.as_ref().map(|s| {
            let elapsed = self.sim_t - self.scenario_t0;
            let next = s.events.get(s.next).map(|e| json!({"at_s": e.at_s, "label": e.label}));
            json!({"id": s.id, "name": s.name, "elapsed_s": elapsed, "next_event": next,
                   "events": s.events.iter().map(|e| json!({"at_s": e.at_s, "label": e.label})).collect::<Vec<_>>()})
        });
        let route_wp = if self.cfg.flight.mode == "route" { self.cfg.flight.route.get(self.nav.wp_index).map(|w| w.name.clone()) } else { None };
        let state = json!({
            "running": self.running,
            "paused": self.paused,
            "sim_t": self.sim_t,
            "utc": geo::unix_now_f64(),
            "own": self.own,
            "reported": own,
            "gps_ok": self.gps_ok(),
            "pressure_alt": self.pressure_alt(self.own.alt_ft),
            "nav": {"wp_index": self.nav.wp_index, "wp_name": route_wp, "finished": self.nav.finished},
            "stats": self.stats,
            "scenario": scenario,
            "station": self.station,
            "uplink_payloads": self.uplinks.len(),
            "notices": self.notices.iter().rev().take(12).map(|(t, m)| json!({"t": t, "msg": m})).collect::<Vec<_>>(),
            "recording": self.recorder.as_ref().map(|r| json!({"path": r.path, "records": r.records, "bytes": r.bytes})),
            "replay": self.replay.as_ref().map(|r| json!({"name": r.name, "idx": r.idx, "total": r.records.len(), "speed": r.speed, "looping": r.looping})),
            "profile": self.cfg.profile,
        });
        state
    }

    #[cfg(not(target_os = "espidf"))]
    pub fn snapshot(&self) -> Value {
        let mut state = self.snapshot_fields();
        state["traffic"] = Value::Array(self.traffic.iter().map(|t| self.traffic_row(t)).collect());
        state
    }

    /// Serialize one ESP32 traffic row at a time. No collection holds all per-target Values.
    #[cfg(target_os = "espidf")]
    pub fn write_snapshot(&self, out: impl std::io::Write, extra: Value, since: u64) -> Result<()> {
        use serde::ser::SerializeMap;

        struct TrafficRows<'a>(&'a Engine);
        impl Serialize for TrafficRows<'_> {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error> {
                serializer.collect_seq(self.0.traffic.iter().map(|t| self.0.traffic_row(t)))
            }
        }

        let mut serializer = serde_json::Serializer::new(out);
        let mut map = serde::Serializer::serialize_map(&mut serializer, None)?;
        for (key, value) in self.snapshot_fields().as_object().unwrap() {
            map.serialize_entry(key, value)?;
        }
        for (key, value) in extra.as_object().unwrap() {
            map.serialize_entry(key, value)?;
        }
        drop(extra);
        let recent: Vec<_> = self.log.iter().skip(self.log.len().saturating_sub(8))
            .filter(|e| e.seq > since).collect();
        map.serialize_entry("log", &recent)?;
        map.serialize_entry("traffic_columns", TRAFFIC_COLUMNS)?;
        map.serialize_entry("traffic", &TrafficRows(self))?;
        map.end()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(e: &mut Engine, secs: f64) -> Vec<Datagram> {
        let mut out = Vec::new();
        let mut t = 1_800_000_000.0;
        let steps = (secs / 0.02) as usize;
        for _ in 0..steps {
            out.extend(e.step(0.02, t));
            t += 0.02;
        }
        out
    }

    #[test]
    fn generic_profile_emits_valid_frames() {
        let mut e = Engine::new(Config::default());
        e.add_traffic(&json!({"preset": "random", "count": 3})).unwrap();
        let out = run(&mut e, 3.0);
        let mut d = gdl90::Deframer::default();
        let mut ids = std::collections::BTreeSet::new();
        for g in &out {
            for f in d.push(&g.bytes) {
                match f {
                    gdl90::Frame::Ok(m) => {
                        ids.insert(m[0]);
                    }
                    other => panic!("bad frame {other:?}"),
                }
            }
        }
        for id in [0u8, 7, 10, 11, 20] {
            assert!(ids.contains(&id), "missing id {id}: {ids:?}");
        }
    }

    #[test]
    fn every_profile_and_scenario_runs() {
        for p in config::PROFILES {
            for s in scenarios::SCENARIOS {
                let mut e = Engine::new(Config::default());
                e.load_scenario(s.id, &Anchor::default()).unwrap();
                e.apply_profile(p.id).unwrap();
                let out = run(&mut e, 1.5);
                assert!(!out.is_empty(), "{} / {} produced nothing", p.id, s.id);
            }
        }
    }

}
