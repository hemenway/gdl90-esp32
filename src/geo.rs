//! Spherical-earth navigation helpers and UTC time helpers (no external time crate).

use std::time::{SystemTime, UNIX_EPOCH};

pub const EARTH_RADIUS_NM: f64 = 3440.065;
pub const FT_PER_M: f64 = 3.280_84;
pub const KT_TO_MPS: f64 = 0.514_444;
pub const G: f64 = 9.806_65;

pub fn distance_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_NM * a.sqrt().min(1.0).asin()
}

/// Initial true bearing from 1 to 2, degrees 0..360.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Point at `dist_nm` along true bearing `brg_deg`.
pub fn destination(lat: f64, lon: f64, brg_deg: f64, dist_nm: f64) -> (f64, f64) {
    let d = dist_nm / EARTH_RADIUS_NM;
    let b = brg_deg.to_radians();
    let p1 = lat.to_radians();
    let l1 = lon.to_radians();
    let p2 = (p1.sin() * d.cos() + p1.cos() * d.sin() * b.cos()).asin();
    let l2 = l1 + (b.sin() * d.sin() * p1.cos()).atan2(d.cos() - p1.sin() * p2.sin());
    let mut lon2 = l2.to_degrees();
    lon2 = (lon2 + 540.0).rem_euclid(360.0) - 180.0;
    (p2.to_degrees(), lon2)
}

/// Signed smallest difference a - b in degrees, -180..180.
pub fn angle_diff(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

pub fn unix_now_f64() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

/// Howard Hinnant's civil_from_days.
pub fn civil_from_days(z: i64) -> (i64, u8, u8) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u8;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u8;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// (year, month, day, hour, minute, second) UTC for a unix time.
pub fn utc_fields(unix: f64) -> (i64, u8, u8, u8, u8, u8) {
    let secs = unix.floor() as i64;
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    let (y, m, d) = civil_from_days(days);
    (y, m, d, (sod / 3600) as u8, (sod / 60 % 60) as u8, (sod % 60) as u8)
}

pub fn utc_now_fields() -> (i64, u8, u8, u8, u8, u8) {
    utc_fields(unix_now_f64())
}

/// "DDHHMMZ" for a unix time.
pub fn ddhhmmz(unix: f64) -> String {
    let (_, _, d, h, m, _) = utc_fields(unix);
    format!("{d:02}{h:02}{m:02}Z")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dest_and_back() {
        let (lat, lon) = destination(44.9, -123.0, 45.0, 10.0);
        assert!((distance_nm(44.9, -123.0, lat, lon) - 10.0).abs() < 1e-6);
        assert!((bearing_deg(44.9, -123.0, lat, lon) - 45.0).abs() < 0.1);
    }

    #[test]
    fn civil() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20401), (2025, 11, 9));
        assert_eq!(civil_from_days(20766), (2026, 11, 9));
    }
}
