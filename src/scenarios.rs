//! Prebuilt test scenarios, generated relative to an anchor (airport reference point, runway
//! heading, field elevation) so they can be dropped anywhere on the map.

use rand::{rngs::SmallRng, Rng};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::config::Waypoint;
use crate::geo;
use crate::sim::{self, Behavior, Kin, Target};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Anchor {
    pub lat: f64,
    pub lon: f64,
    /// True heading of the runway in use.
    pub runway_hdg: f64,
    pub elev_ft: f64,
}

impl Default for Anchor {
    fn default() -> Self {
        // Salem / McNary Field (KSLE), the area the GDL 90 ICD's own examples use.
        Self { lat: 44.9095, lon: -123.0026, runway_hdg: 325.0, elev_ft: 214.0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Event {
    pub at_s: f64,
    pub label: String,
    /// Config merge patch; may carry "_traffic": {"<id>": <Target merge patch>}.
    pub patch: Value,
}

pub struct Plan {
    pub profile: Option<&'static str>,
    pub patch: Value,
    pub start: Kin,
    pub traffic: Vec<Target>,
    pub events: Vec<Event>,
}

pub struct ScenarioInfo {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
}

pub const SCENARIOS: &[ScenarioInfo] = &[
    ScenarioInfo { id: "orbit", name: "Orbit the field", description: "Airborne 5 nm orbit at 3,300 ft AGL. Steady, predictable baseline." },
    ScenarioInfo { id: "parked", name: "Parked on the ramp", description: "On the ground, stationary, GPS valid. Checks ground state and the no-movement display." },
    ScenarioInfo { id: "taxi", name: "Taxi loop", description: "15 kt taxi around the field on the ground (air/ground bit = ground)." },
    ScenarioInfo { id: "pattern", name: "Traffic pattern, touch-and-goes", description: "Takeoff roll, left traffic at 1,000 ft AGL, touch-and-go, repeat. Exercises air/ground transitions and approach/altitude callouts." },
    ScenarioInfo { id: "approach", name: "Straight-in approach + go-around", description: "10 nm final on a 3-degree path, go-around at 200 ft AGL, re-enter on downwind." },
    ScenarioInfo { id: "xc", name: "Cross-country triangle", description: "Takeoff, climb to cruise, 30 nm legs at 120 kt, loops." },
    ScenarioInfo { id: "steep_turns", name: "Steep turns (45 degree bank)", description: "Tight orbit at 45 degrees of bank, reversing direction at 90 s. Exercises AHRS / EFIS / G." },
    ScenarioInfo { id: "head_on", name: "Traffic conflicts", description: "Head-on intruder at co-altitude (CPA 75 s), crossing from the right 300 ft below (CPA 120 s), overtaker from behind (CPA 150 s). Alert bit set inside 1 nm / 600 ft." },
    ScenarioInfo { id: "busy", name: "Busy airspace", description: "35 random ADS-B / TIS-B targets within 20 nm, surface vehicles on the field, one emergency." },
    ScenarioInfo { id: "emergency", name: "Emergency codes", description: "Targets squawking general emergency, no-comm and downed aircraft." },
    ScenarioInfo { id: "shadow", name: "TIS-B ownship shadow", description: "TIS-B target using the ownship ICAO trailing 0.05 nm; at 60 s it slides to 0.8 nm, where iFly stops treating it as a ghost." },
    ScenarioInfo { id: "gps_outage", name: "Receiver GPS outage", description: "At 30 s the ownship goes invalid (lat/lon/NIC zero, no 0x0B, heartbeat GPS bit off); recovers at 90 s." },
    ScenarioInfo { id: "weather", name: "FIS-B weather", description: "Storm cells ahead of the route as regional NEXRAD, METAR/TAF text, 4 uplinks/s from a ground station." },
    ScenarioInfo { id: "high_altitude", name: "Climb to FL510 and back", description: "4,000 fpm climb to 51,000 ft at 420 kt, then descent. Exercises altitude/vertical-rate encoding ranges." },
];

fn pt(a: &Anchor, along_nm: f64, left_nm: f64) -> (f64, f64) {
    let (lat, lon) = geo::destination(a.lat, a.lon, a.runway_hdg, along_nm);
    if left_nm.abs() < 1e-9 {
        return (lat, lon);
    }
    geo::destination(lat, lon, (a.runway_hdg - 90.0).rem_euclid(360.0), left_nm)
}

fn wp(name: &str, p: (f64, f64), alt_ft: f64, speed_kt: f64) -> Waypoint {
    Waypoint { name: name.into(), lat: p.0, lon: p.1, alt_ft, speed_kt }
}

fn round500(ft: f64) -> f64 {
    (ft / 500.0).round() * 500.0
}

pub fn random_tail(rng: &mut SmallRng) -> String {
    if rng.gen_bool(0.25) {
        let al = ["UAL", "SWA", "ASA", "DAL", "AAL", "SKW", "QXE", "FDX"];
        format!("{}{}", al[rng.gen_range(0..al.len())], rng.gen_range(100..3000))
    } else {
        let mut s = format!("N{}", rng.gen_range(1..10));
        for _ in 0..rng.gen_range(1..4) {
            s.push(char::from(b'0' + rng.gen_range(0..10)));
        }
        if rng.gen_bool(0.6) {
            for _ in 0..rng.gen_range(1..3) {
                s.push(char::from(b'A' + rng.gen_range(0..26)));
            }
        }
        s
    }
}

pub fn random_icao(rng: &mut SmallRng) -> u32 {
    rng.gen_range(0xA00001..0xADF7C7)
}

fn target(id: u32, rng: &mut SmallRng, kin: Kin, behavior: Behavior) -> Target {
    Target {
        id,
        address: random_icao(rng),
        callsign: random_tail(rng),
        kin,
        behavior,
        ..Default::default()
    }
}

fn base_patch(a: &Anchor) -> Value {
    json!({
        "ownship": {"gps_valid": true},
        "weather": { "cells": [] },
        "output": { "per_message_datagrams": true },
        "flight": {
            "ground_elev_ft": a.elev_ft,
            "time_scale": 1.0,
            "turn_rate_dps": 3.0,
            "max_bank_deg": 30.0,
            "climb_fpm": 700.0,
            "descent_fpm": 700.0,
            "accel_kt_s": 3.0,
            "orbit": { "lat": a.lat, "lon": a.lon, "radius_nm": 5.0, "clockwise": true },
            "route_loop": true,
            "route_loop_to": 0
        }
    })
}

fn with(mut base: Value, extra: Value) -> Value {
    crate::config::merge_patch(&mut base, &extra);
    base
}

fn cruise_start(a: &Anchor, agl: f64, speed: f64) -> Kin {
    let (lat, lon) = pt(a, -6.0, 0.0);
    Kin::at(lat, lon, round500(a.elev_ft + agl), a.runway_hdg, speed, false)
}

fn route_value(r: &[Waypoint]) -> Value {
    serde_json::to_value(r).unwrap()
}

pub fn build(id: &str, a: &Anchor, own_icao: u32, own_callsign: &str, rng: &mut SmallRng) -> Option<Plan> {
    let e = a.elev_ft;
    let h = a.runway_hdg;
    let base = base_patch(a);
    let on_runway = Kin::at(a.lat, a.lon, e, h, 0.0, true);
    let mut events = Vec::new();
    let mut traffic = Vec::new();
    let profile = Some("generic");

    let (patch, start) = match id {
        "orbit" => {
            let (lat, lon) = geo::destination(a.lat, a.lon, 180.0, 5.0);
            (
                with(base, json!({"flight": {"mode": "orbit", "target_alt_ft": round500(e + 3300.0), "target_speed_kt": 110.0}})),
                Kin::at(lat, lon, round500(e + 3300.0), 270.0, 110.0, false),
            )
        }
        "parked" => (with(base, json!({"flight": {"mode": "parked"}})), on_runway.clone()),
        "taxi" => {
            let r = vec![
                wp("Twy A", pt(a, 0.0, 0.08), e, 15.0),
                wp("Twy A end", pt(a, 0.9, 0.08), e, 15.0),
                wp("Ramp", pt(a, 0.9, 0.25), e, 10.0),
                wp("Ramp W", pt(a, 0.1, 0.25), e, 12.0),
            ];
            let (lat, lon) = pt(a, 0.1, 0.25);
            (
                with(base, json!({"flight": {"mode": "route", "route": route_value(&r)}})),
                Kin::at(lat, lon, e, h + 180.0, 0.0, true),
            )
        }
        "pattern" => {
            let tpa = round500(e + 1000.0).max(e + 800.0);
            let r = vec![
                wp("Upwind", pt(a, 1.6, 0.0), e + 500.0, 80.0),
                wp("Crosswind", pt(a, 2.0, 0.8), tpa, 90.0),
                wp("Downwind", pt(a, 0.0, 0.8), tpa, 90.0),
                wp("Base", pt(a, -1.0, 0.8), e + 700.0, 75.0),
                wp("Final", pt(a, -1.2, 0.0), e + 450.0, 65.0),
                wp("Threshold", pt(a, 0.0, 0.0), e, 60.0),
                wp("Touch-and-go", pt(a, 0.6, 0.0), e, 60.0),
            ];
            (
                with(base, json!({"flight": {"mode": "route", "route": route_value(&r), "wind_dir": h, "wind_kt": 6.0}})),
                on_runway.clone(),
            )
        }
        "approach" => {
            let r = vec![
                wp("FAF", pt(a, -5.0, 0.0), e + 1600.0, 90.0),
                wp("Threshold+200", pt(a, -0.6, 0.0), e + 200.0, 80.0),
                wp("Go-around", pt(a, 1.5, 0.0), e + 1000.0, 85.0),
                wp("Crosswind", pt(a, 2.2, 1.0), round500(e + 1500.0), 100.0),
                wp("Downwind", pt(a, -2.0, 1.0), round500(e + 1500.0), 100.0),
                wp("Base", pt(a, -6.0, 1.0), round500(e + 2500.0), 100.0),
                wp("Intercept", pt(a, -10.0, 0.0), round500(e + 3000.0), 100.0),
            ];
            let (lat, lon) = pt(a, -10.0, 0.0);
            (
                with(base, json!({"flight": {"mode": "route", "route": route_value(&r), "descent_fpm": 900.0, "route_loop_to": 0}})),
                Kin::at(lat, lon, e + 3000.0, h, 100.0, false),
            )
        }
        "xc" => {
            let cruise = round500(e + 4500.0);
            let (la, loa) = geo::destination(a.lat, a.lon, h, 30.0);
            let (lb, lob) = geo::destination(la, loa, (h + 120.0).rem_euclid(360.0), 30.0);
            let r = vec![
                wp("Climbout", pt(a, 3.0, 0.0), e + 1500.0, 110.0),
                wp("Leg A", (la, loa), cruise, 120.0),
                wp("Leg B", (lb, lob), cruise, 120.0),
                wp("Home", pt(a, 4.0, 0.0), round500(e + 3000.0), 120.0),
            ];
            (
                with(base, json!({"flight": {"mode": "route", "route": route_value(&r), "route_loop_to": 1, "climb_fpm": 800.0}})),
                on_runway.clone(),
            )
        }
        "steep_turns" => {
            let (lat, lon) = pt(a, 5.0, 0.0);
            let alt = round500(e + 3000.0);
            events.push(Event { at_s: 90.0, label: "Reverse turn".into(), patch: json!({"flight": {"orbit": {"clockwise": false}}}) });
            (
                with(base, json!({"flight": {"mode": "orbit", "orbit": {"lat": lat, "lon": lon, "radius_nm": 0.16, "clockwise": true},
                    "turn_rate_dps": 12.0, "max_bank_deg": 45.0, "target_alt_ft": alt, "target_speed_kt": 100.0, "wind_kt": 0.0}})),
                Kin::at(geo::destination(lat, lon, 0.0, 0.16).0, lon, alt, 90.0, 100.0, false),
            )
        }
        "head_on" | "emergency" | "busy" | "shadow" | "gps_outage" | "weather" => {
            let start = cruise_start(a, 3300.0, 120.0);
            let mut p = with(base, json!({"flight": {"mode": "manual", "target_track": h, "target_alt_ft": start.alt_ft, "target_speed_kt": 120.0, "wind_kt": 8.0}}));
            match id {
                "head_on" => {
                    let mut t1 = target(1, rng, sim::converging(&start, 0.0, 140.0, 75.0, 0.0), Behavior::Straight { level_ft: None });
                    t1.callsign = "N4417K".into();
                    let mut t2 = target(2, rng, sim::converging(&start, 90.0, 110.0, 120.0, -300.0), Behavior::Straight { level_ft: None });
                    t2.callsign = "N72CX".into();
                    let mut t3 = target(3, rng, sim::converging(&start, 180.0, 170.0, 150.0, 200.0), Behavior::Straight { level_ft: None });
                    t3.callsign = "N500TB".into();
                    t3.emitter = 2;
                    traffic.extend([t1, t2, t3]);
                }
                "emergency" => {
                    let mut t1 = target(1, rng, sim::converging(&start, 30.0, 130.0, 90.0, 500.0), Behavior::Straight { level_ft: None });
                    t1.emergency = 1;
                    let (lat, lon) = pt(a, 1.0, -1.5);
                    let mut t2 = target(2, rng, Kin::at(lat, lon, e, 45.0, 0.0, true), Behavior::Static);
                    t2.emergency = 6;
                    t2.callsign = "N9DOWN".into();
                    let (olat, olon) = pt(a, 2.0, 3.0);
                    let mut t3 = target(3, rng, Kin::at(olat, olon, e + 2500.0, 0.0, 100.0, false),
                        Behavior::Orbit { lat: olat, lon: olon, radius_nm: 2.0, clockwise: false });
                    t3.emergency = 4;
                    traffic.extend([t1, t2, t3]);
                }
                "busy" => {
                    p = with(p, json!({"flight": {"mode": "orbit", "orbit": {"lat": a.lat, "lon": a.lon, "radius_nm": 6.0}}}));
                    for i in 0..35u32 {
                        let brg = rng.gen_range(0.0..360.0);
                        let dist = rng.gen_range(1.0..20.0);
                        let (lat, lon) = geo::destination(a.lat, a.lon, brg, dist);
                        let alt = round500(e + rng.gen_range(500.0..9000.0)) + if rng.gen_bool(0.5) { 0.0 } else { 0.0 };
                        let spd = rng.gen_range(80.0..260.0);
                        let mut k = Kin::at(lat, lon, alt, rng.gen_range(0.0..360.0), spd, false);
                        k.vs_fpm = if rng.gen_bool(0.3) { rng.gen_range(-800.0..800.0) } else { 0.0 };
                        let beh = if rng.gen_bool(0.3) {
                            Behavior::Orbit { lat, lon, radius_nm: rng.gen_range(1.0..4.0), clockwise: rng.gen_bool(0.5) }
                        } else {
                            Behavior::Straight { level_ft: Some(alt + k.vs_fpm.signum() * 1500.0) }
                        };
                        let mut t = target(i + 1, rng, k, beh);
                        t.addr_type = match rng.gen_range(0..10) { 0..=5 => 0, 6 | 7 => 2, 8 => 3, _ => 1 };
                        t.emitter = match rng.gen_range(0..10) { 0..=6 => 1, 7 => 2, 8 => 3, _ => 7 };
                        if t.addr_type >= 2 {
                            t.nic = 7;
                            t.nacp = 8;
                        }
                        traffic.push(t);
                    }
                    for i in 0..4u32 {
                        let (lat, lon) = pt(a, rng.gen_range(0.0..0.9), rng.gen_range(0.1..0.3));
                        let trk = rng.gen_range(0.0..360.0);
                        let mut t = target(100 + i, rng, Kin::at(lat, lon, e, trk, 12.0, true),
                            Behavior::Orbit { lat, lon, radius_nm: 0.1, clockwise: true });
                        t.addr_type = 4;
                        t.emitter = 18;
                        t.callsign = format!("TUG{}", i + 1);
                        traffic.push(t);
                    }
                    if let Some(t) = traffic.get_mut(5) {
                        t.emergency = 1;
                    }
                }
                "shadow" => {
                    let mut s1 = target(1, rng, start.clone(), Behavior::Shadow { delay_s: 1.5, offset_nm: 0.05, offset_brg: 180.0, alt_offset_ft: 0.0 });
                    s1.address = own_icao;
                    s1.addr_type = 2;
                    s1.callsign = String::new();
                    s1.nic = 6;
                    s1.nacp = 7;
                    let mut s2 = target(2, rng, start.clone(), Behavior::Shadow { delay_s: 2.0, offset_nm: 0.1, offset_brg: 150.0, alt_offset_ft: 100.0 });
                    s2.addr_type = 3;
                    s2.callsign = own_callsign.to_string();
                    traffic.extend([s1, s2]);
                    events.push(Event {
                        at_s: 60.0,
                        label: "Shadow 1 drifts to 0.8 nm".into(),
                        patch: json!({"_traffic": {"1": {"behavior": {"kind": "shadow", "delay_s": 1.5, "offset_nm": 0.8, "offset_brg": 90.0, "alt_offset_ft": 0.0}}}}),
                    });
                }
                "gps_outage" => {
                    events.push(Event { at_s: 30.0, label: "Receiver loses GPS".into(), patch: json!({"ownship": {"gps_valid": false}}) });
                    events.push(Event { at_s: 90.0, label: "GPS recovers".into(), patch: json!({"ownship": {"gps_valid": true}}) });
                }
                "weather" => {
                    let mut cells = Vec::new();
                    for (dist, off, r, i) in [(14.0, 2.0, 8.0, 6u8), (24.0, -5.0, 6.0, 4), (32.0, 4.0, 10.0, 7), (8.0, -9.0, 5.0, 3)] {
                        let (lat, lon) = geo::destination(start.lat, start.lon, h, dist);
                        let (lat, lon) = geo::destination(lat, lon, (h + 90.0).rem_euclid(360.0), off);
                        cells.push(json!({"lat": lat, "lon": lon, "radius_nm": r, "intensity": i}));
                    }
                    p = with(p, json!({"weather": {"cells": cells}, "streams": {"uplink": {"enabled": true, "rate_hz": 4.0}}}));
                }
                _ => {}
            }
            (p, start)
        }
        "high_altitude" => {
            let start = cruise_start(a, 3000.0, 250.0);
            events.push(Event { at_s: 780.0, label: "Begin descent".into(), patch: json!({"flight": {"target_alt_ft": round500(e + 3000.0), "target_speed_kt": 250.0}}) });
            (
                with(base, json!({"flight": {"mode": "manual", "target_track": h, "target_alt_ft": 51000.0, "target_speed_kt": 420.0,
                    "climb_fpm": 4000.0, "descent_fpm": 4000.0, "accel_kt_s": 4.0}})),
                start,
            )
        }
        _ => return None,
    };
    Some(Plan { profile, patch, start, traffic, events })
}
