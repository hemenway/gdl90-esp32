//! Kinematic flight model for the ownship and traffic targets.

use serde::{Deserialize, Serialize};

use crate::config::{FlightConfig, Waypoint};
use crate::geo::{self, angle_diff, KT_TO_MPS};

#[derive(Clone, Debug, Serialize, Deserialize, Default, PartialEq)]
#[serde(default)]
pub struct Kin {
    pub lat: f64,
    pub lon: f64,
    /// MSL altitude, ft.
    pub alt_ft: f64,
    pub tas_kt: f64,
    pub ias_kt: f64,
    pub gs_kt: f64,
    /// True ground track.
    pub track: f64,
    /// True heading.
    pub heading: f64,
    pub vs_fpm: f64,
    pub pitch: f64,
    /// + = right wing down.
    pub roll: f64,
    /// Heading rate, deg/s.
    pub turn_rate: f64,
    pub g_load: f64,
    pub slip: f64,
    pub on_ground: bool,
}

impl Kin {
    pub fn at(lat: f64, lon: f64, alt_ft: f64, track: f64, speed_kt: f64, on_ground: bool) -> Self {
        Kin {
            lat,
            lon,
            alt_ft,
            tas_kt: speed_kt,
            ias_kt: speed_kt,
            gs_kt: speed_kt,
            track,
            heading: track,
            g_load: 1.0,
            on_ground,
            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct NavState {
    pub wp_index: usize,
    pub finished: bool,
}

fn approach(cur: f64, target: f64, max_step: f64) -> f64 {
    if (target - cur).abs() <= max_step {
        target
    } else {
        cur + max_step.copysign(target - cur)
    }
}

/// Advance the ownship by `dt` seconds under the flight config.
pub fn step_ownship(k: &mut Kin, f: &FlightConfig, nav: &mut NavState, dt: f64) {
    if dt <= 0.0 {
        return;
    }
    let ground = f.ground_elev_ft;
    if f.mode == "parked" {
        k.tas_kt = 0.0;
        k.ias_kt = 0.0;
        k.gs_kt = 0.0;
        k.vs_fpm = 0.0;
        k.roll = 0.0;
        k.pitch = 0.0;
        k.turn_rate = 0.0;
        k.g_load = 1.0;
        k.on_ground = true;
        k.alt_ft = ground;
        return;
    }

    // ---- targets ----------------------------------------------------------------------------
    let mut tgt_track = f.target_track;
    let mut tgt_alt = f.target_alt_ft;
    let mut tgt_speed = f.target_speed_kt;
    let mut path_vs: Option<f64> = None;

    match f.mode.as_str() {
        "route" if !f.route.is_empty() && !nav.finished => {
            if nav.wp_index >= f.route.len() {
                nav.wp_index = 0;
            }
            let wp = &f.route[nav.wp_index];
            let dist = geo::distance_nm(k.lat, k.lon, wp.lat, wp.lon);
            let brg = geo::bearing_deg(k.lat, k.lon, wp.lat, wp.lon);
            // turn anticipation toward the next leg
            let next = if nav.wp_index + 1 < f.route.len() {
                Some(&f.route[nav.wp_index + 1])
            } else if f.route_loop {
                f.route.get(f.route_loop_to.min(f.route.len() - 1))
            } else {
                None
            };
            let lead = match next {
                Some(n) => {
                    let next_brg = geo::bearing_deg(wp.lat, wp.lon, n.lat, n.lon);
                    let delta = angle_diff(next_brg, brg).abs().min(150.0);
                    let omega = f.turn_rate_dps.max(0.5).to_radians();
                    let r_nm = (k.gs_kt.max(30.0) / 3600.0) / omega;
                    (r_nm * (delta.to_radians() / 2.0).tan()).clamp(0.05, 1.5)
                }
                None => 0.05,
            };
            if dist <= lead || (dist < 0.3 && angle_diff(brg, k.track).abs() > 100.0) {
                nav.wp_index += 1;
                if nav.wp_index >= f.route.len() {
                    if f.route_loop {
                        nav.wp_index = f.route_loop_to.min(f.route.len() - 1);
                    } else {
                        nav.finished = true;
                        nav.wp_index = f.route.len() - 1;
                    }
                }
            }
            let wp = &f.route[nav.wp_index];
            tgt_track = geo::bearing_deg(k.lat, k.lon, wp.lat, wp.lon);
            tgt_alt = wp.alt_ft;
            tgt_speed = wp.speed_kt;
            if nav.finished {
                tgt_track = k.track;
            }
            // descend on a path that reaches the waypoint altitude at the waypoint
            let d = geo::distance_nm(k.lat, k.lon, wp.lat, wp.lon);
            let eta_min = d / k.gs_kt.max(40.0) * 60.0;
            if wp.alt_ft < k.alt_ft && !k.on_ground {
                let need = (wp.alt_ft - k.alt_ft) / eta_min.max(0.05);
                let mut vs = need.max(-f.descent_fpm.max(100.0) * 1.6);
                // landing: a waypoint on the runway means "touch down now", not "float the
                // whole leg"; flare onto the surface at >= 250 fpm so there is a real ground roll
                if wp.alt_ft <= ground + 5.0 && k.alt_ft - ground < 60.0 {
                    vs = vs.min(-250.0);
                }
                path_vs = Some(vs);
            }
        }
        "orbit" => {
            let o = &f.orbit;
            let d = geo::distance_nm(o.lat, o.lon, k.lat, k.lon);
            let brg_from_center = geo::bearing_deg(o.lat, o.lon, k.lat, k.lon);
            let err = d - o.radius_nm;
            let corr = (err * 60.0).clamp(-60.0, 60.0);
            tgt_track = if o.clockwise {
                brg_from_center + 90.0 + corr
            } else {
                brg_from_center - 90.0 - corr
            }
            .rem_euclid(360.0);
            if d > o.radius_nm * 2.0 + 1.0 {
                tgt_track = geo::bearing_deg(k.lat, k.lon, o.lat, o.lon);
            }
        }
        _ => {}
    }

    // ---- speed ------------------------------------------------------------------------------
    let accel = if k.on_ground && tgt_speed < k.tas_kt { 6.0 } else { f.accel_kt_s.max(0.1) };
    k.tas_kt = approach(k.tas_kt, tgt_speed.max(0.0), accel * dt);

    // ---- vertical ---------------------------------------------------------------------------
    let want_air = tgt_alt > ground + 50.0;
    if k.on_ground {
        k.alt_ft = ground;
        k.vs_fpm = 0.0;
        if want_air && k.tas_kt >= f.rotate_kt {
            k.on_ground = false;
            k.vs_fpm = 200.0;
            k.alt_ft = ground + 1.0;
        }
    } else {
        let diff = tgt_alt - k.alt_ft;
        let cmd = match path_vs {
            Some(v) => v,
            None => (diff * 4.0).clamp(-f.descent_fpm.max(50.0), f.climb_fpm.max(50.0)),
        };
        k.vs_fpm = approach(k.vs_fpm, cmd, 1200.0 * dt);
        k.alt_ft += k.vs_fpm * dt / 60.0;
        if k.alt_ft <= ground {
            k.alt_ft = ground;
            k.vs_fpm = 0.0;
            k.on_ground = true;
        }
    }

    // ---- lateral ----------------------------------------------------------------------------
    let tas = k.tas_kt.max(0.0);
    if k.on_ground {
        let max_rate = if tas < 30.0 { 20.0 } else { 3.0 };
        let dh = angle_diff(tgt_track, k.heading);
        let rate = (dh * 0.8).clamp(-max_rate, max_rate);
        k.heading = (k.heading + rate * dt).rem_euclid(360.0);
        k.turn_rate = rate;
        k.track = k.heading;
        k.gs_kt = tas;
        k.roll = approach(k.roll, 0.0, 15.0 * dt);
    } else {
        let w = f.wind_kt.max(0.0);
        let wca = if tas > 1.0 {
            ((w * (f.wind_dir - tgt_track).to_radians().sin()) / tas).clamp(-1.0, 1.0).asin().to_degrees()
        } else {
            0.0
        };
        let desired_hdg = (tgt_track + wca).rem_euclid(360.0);
        let v = tas.max(30.0) * KT_TO_MPS;
        let bank_limited = (geo::G * f.max_bank_deg.clamp(1.0, 85.0).to_radians().tan() / v).to_degrees();
        let max_rate = f.turn_rate_dps.max(0.1).min(bank_limited);
        let dh = angle_diff(desired_hdg, k.heading);
        let rate = (dh * 0.6).clamp(-max_rate, max_rate);
        k.heading = (k.heading + rate * dt).rem_euclid(360.0);
        k.turn_rate = rate;
        let bank = (v * rate.to_radians() / geo::G).atan().to_degrees();
        k.roll = approach(k.roll, bank, 15.0 * dt);
        // air vector + wind vector (wind blows toward wind_dir + 180)
        let h = k.heading.to_radians();
        let wt = (f.wind_dir + 180.0).to_radians();
        let n = tas * h.cos() + w * wt.cos();
        let e = tas * h.sin() + w * wt.sin();
        k.gs_kt = n.hypot(e);
        if k.gs_kt > 0.1 {
            k.track = e.atan2(n).to_degrees().rem_euclid(360.0);
        }
    }

    // ---- attitude & derived -----------------------------------------------------------------
    let gamma = if k.gs_kt > 1.0 {
        (k.vs_fpm / (k.gs_kt.max(tas) * 101.27)).atan().to_degrees()
    } else {
        0.0
    };
    let aoa = if k.on_ground {
        if want_air && tas > f.rotate_kt * 0.9 { 6.0 } else { 0.0 }
    } else {
        (2.0 + (100.0 - tas) / 20.0).clamp(0.5, 9.0)
    };
    let pitch_target = if k.on_ground { aoa } else { gamma + aoa };
    k.pitch = approach(k.pitch, pitch_target, 4.0 * dt);
    k.g_load = 1.0 / k.roll.to_radians().cos().max(0.2);
    k.ias_kt = tas / (1.0 + 0.02 * (k.alt_ft.max(0.0) / 1000.0));
    k.slip = 0.0;

    let dist = k.gs_kt * dt / 3600.0;
    if dist > 0.0 {
        let (lat, lon) = geo::destination(k.lat, k.lon, k.track, dist);
        k.lat = lat;
        k.lon = lon;
    }
}

// ---------------------------------------------------------------------------------------------
// Traffic
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Behavior {
    /// Keep track/speed; vertical speed until `level_ft` (if any).
    Straight { level_ft: Option<f64> },
    Orbit { lat: f64, lon: f64, radius_nm: f64, clockwise: bool },
    Route { waypoints: Vec<Waypoint>, index: usize, looped: bool },
    /// Follows the ownship's track history (TIS-B ownship shadow / formation).
    Shadow { delay_s: f64, offset_nm: f64, offset_brg: f64, alt_offset_ft: f64 },
    Static,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Target {
    pub id: u32,
    pub address: u32,
    pub addr_type: u8,
    pub callsign: String,
    pub emitter: u8,
    pub kin: Kin,
    pub behavior: Behavior,
    /// "auto" (CSA-like: < 1 nm and < 600 ft), "on" or "off".
    pub alert: String,
    pub emergency: u8,
    pub nic: u8,
    pub nacp: u8,
    /// Stop transmitting (target goes stale in the EFB).
    pub silent: bool,
    pub alt_invalid: bool,
    pub track_type: u8,
    pub extrapolated: bool,
}

impl Default for Target {
    fn default() -> Self {
        Self {
            id: 0,
            address: 0xABCDEF,
            addr_type: 0,
            callsign: String::new(),
            emitter: 1,
            kin: Kin::default(),
            behavior: Behavior::Straight { level_ft: None },
            alert: "auto".into(),
            emergency: 0,
            nic: 8,
            nacp: 9,
            silent: false,
            alt_invalid: false,
            track_type: 1,
            extrapolated: false,
        }
    }
}

/// Ownship history sample for shadows: (sim time, kinematics).
pub type History = std::collections::VecDeque<(f64, Kin)>;

pub fn step_target(t: &mut Target, dt: f64, sim_t: f64, history: &History) {
    let k = &mut t.kin;
    match &mut t.behavior {
        Behavior::Static => {
            k.gs_kt = 0.0;
            k.vs_fpm = 0.0;
        }
        Behavior::Straight { level_ft } => {
            if let Some(level) = level_ft {
                if (k.vs_fpm > 0.0 && k.alt_ft >= *level) || (k.vs_fpm < 0.0 && k.alt_ft <= *level) {
                    k.alt_ft = *level;
                    k.vs_fpm = 0.0;
                }
            }
            k.alt_ft += k.vs_fpm * dt / 60.0;
            move_along(k, dt);
        }
        Behavior::Orbit { lat, lon, radius_nm, clockwise } => {
            let d = geo::distance_nm(*lat, *lon, k.lat, k.lon);
            let b = geo::bearing_deg(*lat, *lon, k.lat, k.lon);
            let corr = ((d - *radius_nm) * 60.0).clamp(-60.0, 60.0);
            let want = if *clockwise { b + 90.0 + corr } else { b - 90.0 - corr };
            turn_toward(k, want, 3.0, dt);
            k.alt_ft += k.vs_fpm * dt / 60.0;
            move_along(k, dt);
        }
        Behavior::Route { waypoints, index, looped } => {
            if waypoints.is_empty() {
                move_along(k, dt);
            } else {
                if *index >= waypoints.len() {
                    *index = 0;
                }
                let wp = waypoints[*index].clone();
                let d = geo::distance_nm(k.lat, k.lon, wp.lat, wp.lon);
                if d < 0.2 {
                    *index += 1;
                    if *index >= waypoints.len() {
                        if *looped {
                            *index = 0;
                        } else {
                            *index = waypoints.len() - 1;
                        }
                    }
                }
                let wp = &waypoints[*index];
                let brg = geo::bearing_deg(k.lat, k.lon, wp.lat, wp.lon);
                turn_toward(k, brg, 3.0, dt);
                k.gs_kt = approach(k.gs_kt, wp.speed_kt, 3.0 * dt);
                let diff = wp.alt_ft - k.alt_ft;
                k.vs_fpm = (diff * 4.0).clamp(-1000.0, 1000.0);
                k.alt_ft += k.vs_fpm * dt / 60.0;
                move_along(k, dt);
            }
        }
        Behavior::Shadow { delay_s, offset_nm, offset_brg, alt_offset_ft } => {
            let want_t = sim_t - delay_s.max(0.0);
            let sample = history
                .iter()
                .rev()
                .find(|(ts, _)| *ts <= want_t)
                .or_else(|| history.front())
                .map(|(_, s)| s.clone());
            if let Some(s) = sample {
                let (lat, lon) = geo::destination(s.lat, s.lon, (s.track + *offset_brg).rem_euclid(360.0), *offset_nm);
                *k = Kin { lat, lon, alt_ft: s.alt_ft + *alt_offset_ft, ..s };
            }
        }
    }
    k.on_ground = k.on_ground && k.gs_kt < 40.0;
}

fn turn_toward(k: &mut Kin, want: f64, rate: f64, dt: f64) {
    let dh = angle_diff(want.rem_euclid(360.0), k.track);
    let r = (dh * 0.6).clamp(-rate, rate);
    k.track = (k.track + r * dt).rem_euclid(360.0);
    k.heading = k.track;
    k.turn_rate = r;
}

fn move_along(k: &mut Kin, dt: f64) {
    let d = k.gs_kt * dt / 3600.0;
    if d > 0.0 {
        let (lat, lon) = geo::destination(k.lat, k.lon, k.track, d);
        k.lat = lat;
        k.lon = lon;
    }
}

/// Target placed so it reaches the ownship's predicted position after `cpa_s` seconds,
/// approaching from `rel_bearing` (degrees relative to the ownship's track: 0 = head-on).
pub fn converging(own: &Kin, rel_bearing: f64, speed_kt: f64, cpa_s: f64, alt_offset_ft: f64) -> Kin {
    let own_dist = own.gs_kt * cpa_s / 3600.0;
    let (clat, clon) = geo::destination(own.lat, own.lon, own.track, own_dist);
    let from = (own.track + rel_bearing).rem_euclid(360.0);
    let tgt_dist = speed_kt * cpa_s / 3600.0;
    let (lat, lon) = geo::destination(clat, clon, from, tgt_dist);
    let track = (from + 180.0).rem_euclid(360.0);
    Kin::at(lat, lon, own.alt_ft + alt_offset_ft, track, speed_kt, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn takeoff_climb_and_turn() {
        let mut f = FlightConfig::default();
        f.ground_elev_ft = 200.0;
        f.target_alt_ft = 3000.0;
        f.target_speed_kt = 100.0;
        f.target_track = 90.0;
        f.wind_kt = 0.0;
        let mut k = Kin::at(45.0, -123.0, 200.0, 0.0, 0.0, true);
        let mut nav = NavState::default();
        for _ in 0..(300 * 10) {
            step_ownship(&mut k, &f, &mut nav, 0.1);
        }
        assert!(!k.on_ground);
        assert!((k.alt_ft - 3000.0).abs() < 50.0, "alt {}", k.alt_ft);
        assert!(angle_diff(k.track, 90.0).abs() < 2.0, "trk {}", k.track);
        assert!((k.gs_kt - 100.0).abs() < 1.0);
    }

    #[test]
    fn wind_crab() {
        let mut f = FlightConfig::default();
        f.wind_dir = 0.0;
        f.wind_kt = 20.0;
        f.target_track = 90.0;
        f.target_speed_kt = 100.0;
        let mut k = Kin::at(45.0, -123.0, 3500.0, 90.0, 100.0, false);
        let mut nav = NavState::default();
        for _ in 0..600 {
            step_ownship(&mut k, &f, &mut nav, 0.1);
        }
        // wind from the north: heading left of track (into the wind)
        assert!(angle_diff(k.track, 90.0).abs() < 1.0);
        assert!(k.heading < 90.0 && k.heading > 75.0, "hdg {}", k.heading);
    }

    #[test]
    fn converging_meets_ownship() {
        let own = Kin::at(45.0, -123.0, 4500.0, 0.0, 120.0, false);
        let t = converging(&own, 0.0, 140.0, 60.0, 0.0);
        let mut o = own.clone();
        let mut k = t.clone();
        for _ in 0..600 {
            move_along(&mut o, 0.1);
            move_along(&mut k, 0.1);
        }
        assert!(geo::distance_nm(o.lat, o.lon, k.lat, k.lon) < 0.05);
    }
}
