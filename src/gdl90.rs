//! GDL 90 data interface: framing, FCS (CRC-CCITT), and message encoders/decoders.
//!
//! Reference: Garmin 560-1058-00 Rev A, "GDL 90 Data Interface Specification" (June 2007),
//! plus the de-facto vendor extensions EFBs understand (ForeFlight 0x65, Stratux/Levil 0x4C,
//! Stratux 0xCC heartbeat and 'SX' status).

use serde::{Deserialize, Serialize};

pub const FLAG: u8 = 0x7E;
pub const CONTROL_ESCAPE: u8 = 0x7D;

/// Message IDs (decimal per the ICD).
pub mod id {
    pub const HEARTBEAT: u8 = 0;
    pub const INITIALIZATION: u8 = 2;
    pub const UPLINK: u8 = 7;
    pub const HEIGHT_ABOVE_TERRAIN: u8 = 9;
    pub const OWNSHIP: u8 = 10;
    pub const OWNSHIP_GEO_ALT: u8 = 11;
    pub const TRAFFIC: u8 = 20;
    pub const BASIC_REPORT: u8 = 30;
    pub const LONG_REPORT: u8 = 31;
    pub const FOREFLIGHT: u8 = 0x65;
    pub const LEVIL_AHRS: u8 = 0x4C;
    pub const STRATUX_HEARTBEAT: u8 = 0xCC;
    pub const STRATUX_STATUS: u8 = 0x53; // 'S', followed by 'X'
}

// ---------------------------------------------------------------------------------------------
// FCS
// ---------------------------------------------------------------------------------------------

static CRC_TABLE: [u16; 256] = build_crc_table();

/// ICD §2.2.3 crcInit().
const fn build_crc_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc: u16 = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x1021 } else { 0 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// ICD §2.2.3 crcCompute(): CRC over the clear message (ID + data), before byte-stuffing.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc = CRC_TABLE[(crc >> 8) as usize] ^ (crc << 8) ^ b as u16;
    }
    crc
}

// ---------------------------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------------------------

/// Frame a clear message (ID + data): append FCS (LSB first), byte-stuff, wrap in flags.
pub fn frame(msg: &[u8]) -> Vec<u8> {
    let crc = crc16(msg);
    let mut out = Vec::with_capacity(msg.len() + 8);
    out.push(FLAG);
    for b in msg.iter().copied().chain([(crc & 0xFF) as u8, (crc >> 8) as u8]) {
        if b == FLAG || b == CONTROL_ESCAPE {
            out.push(CONTROL_ESCAPE);
            out.push(b ^ 0x20);
        } else {
            out.push(b);
        }
    }
    out.push(FLAG);
    out
}

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    /// Clear message (ID + data), FCS verified and removed.
    Ok(Vec<u8>),
    BadCrc { msg: Vec<u8>, got: u16, want: u16 },
    TooShort(Vec<u8>),
}

/// Streaming receiver (ICD §2.2.1 reception steps). Handles back-to-back and shared flags.
#[derive(Default)]
pub struct Deframer {
    buf: Vec<u8>,
    in_frame: bool,
    escape: bool,
}

impl Deframer {
    pub fn push(&mut self, data: &[u8]) -> Vec<Frame> {
        let mut out = Vec::new();
        for &b in data {
            if b == FLAG {
                if self.in_frame && !self.buf.is_empty() {
                    out.push(Self::finish(std::mem::take(&mut self.buf)));
                }
                self.in_frame = true;
                self.escape = false;
                continue;
            }
            if !self.in_frame {
                continue;
            }
            if b == CONTROL_ESCAPE {
                self.escape = true;
                continue;
            }
            let v = if self.escape {
                self.escape = false;
                b ^ 0x20
            } else {
                b
            };
            self.buf.push(v);
        }
        out
    }

    fn finish(raw: Vec<u8>) -> Frame {
        if raw.len() < 3 {
            return Frame::TooShort(raw);
        }
        let n = raw.len() - 2;
        let got = raw[n] as u16 | (raw[n + 1] as u16) << 8;
        let msg = raw[..n].to_vec();
        let want = crc16(&msg);
        if got == want {
            Frame::Ok(msg)
        } else {
            Frame::BadCrc { msg, got, want }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Field helpers
// ---------------------------------------------------------------------------------------------

fn be24(v: u32) -> [u8; 3] {
    [(v >> 16) as u8, (v >> 8) as u8, v as u8]
}

fn be16(v: u16) -> [u8; 2] {
    [(v >> 8) as u8, v as u8]
}

/// 24-bit two's-complement semicircle (ICD §3.5.1.3), resolution 180/2^23 deg.
/// Truncates toward zero, which is what the ICD's worked example (§3.5.2) does.
pub fn encode_semicircle(deg: f64) -> u32 {
    let v = (deg * (1u32 << 23) as f64 / 180.0).trunc() as i64;
    let v = v.clamp(-(1 << 23), (1 << 23) - 1);
    (v as u32) & 0xFF_FFFF
}

pub fn decode_semicircle(raw: u32) -> f64 {
    let mut v = (raw & 0xFF_FFFF) as i32;
    if v & 0x80_0000 != 0 {
        v -= 1 << 24;
    }
    v as f64 * 180.0 / (1u32 << 23) as f64
}

/// 8-bit angular weighted binary, 360/256 deg.
pub fn encode_track(deg: f64) -> u8 {
    ((deg.rem_euclid(360.0) * 256.0 / 360.0).round() as u32 % 256) as u8
}

/// 8 ASCII characters, space padded. Characters are passed through as-is (uppercased) so
/// invalid call signs can be injected deliberately.
pub fn callsign_bytes(cs: &str) -> [u8; 8] {
    let mut out = [b' '; 8];
    for (i, c) in cs.bytes().take(8).enumerate() {
        out[i] = c.to_ascii_uppercase();
    }
    out
}

fn put_name<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    for (i, c) in s.bytes().take(N).enumerate() {
        out[i] = c;
    }
    out
}

// ---------------------------------------------------------------------------------------------
// Heartbeat (ID 0) — ICD §3.1
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct HeartbeatFields {
    /// Status byte 1 bit 7. `None` = follow the simulated GPS state.
    pub gps_pos_valid: Option<bool>,
    pub maint_required: bool,
    pub ident: bool,
    pub addr_type: bool,
    pub gps_batt_low: bool,
    pub ratcs: bool,
    /// Status byte 1 bit 1 (reserved, should be 0).
    pub reserved_s1b1: bool,
    pub uat_initialized: bool,
    pub csa_requested: bool,
    pub csa_not_available: bool,
    /// Status byte 2 bits 4..1 (reserved, should be 0).
    pub reserved_s2: u8,
    pub utc_ok: bool,
    /// Uplink receptions last second (5 bits). `None` = derive from the uplink stream.
    pub uplink_count: Option<u8>,
    /// Basic + Long receptions last second (10 bits). `None` = derive from traffic count.
    pub basic_long_count: Option<u16>,
}

impl Default for HeartbeatFields {
    fn default() -> Self {
        Self {
            gps_pos_valid: None,
            maint_required: false,
            ident: false,
            addr_type: false,
            gps_batt_low: false,
            ratcs: false,
            reserved_s1b1: false,
            uat_initialized: true,
            csa_requested: false,
            csa_not_available: false,
            reserved_s2: 0,
            utc_ok: true,
            uplink_count: None,
            basic_long_count: None,
        }
    }
}

pub fn heartbeat(
    f: &HeartbeatFields,
    gps_valid: bool,
    secs_since_midnight: u32,
    uplinks: u8,
    basic_long: u16,
) -> Vec<u8> {
    let ts = secs_since_midnight & 0x1FFFF;
    let mut s1 = 0u8;
    if f.gps_pos_valid.unwrap_or(gps_valid) {
        s1 |= 0x80;
    }
    if f.maint_required {
        s1 |= 0x40;
    }
    if f.ident {
        s1 |= 0x20;
    }
    if f.addr_type {
        s1 |= 0x10;
    }
    if f.gps_batt_low {
        s1 |= 0x08;
    }
    if f.ratcs {
        s1 |= 0x04;
    }
    if f.reserved_s1b1 {
        s1 |= 0x02;
    }
    if f.uat_initialized {
        s1 |= 0x01;
    }
    let mut s2 = 0u8;
    if ts & 0x10000 != 0 {
        s2 |= 0x80;
    }
    if f.csa_requested {
        s2 |= 0x40;
    }
    if f.csa_not_available {
        s2 |= 0x20;
    }
    s2 |= (f.reserved_s2 & 0x0F) << 1;
    if f.utc_ok {
        s2 |= 0x01;
    }
    let up = f.uplink_count.unwrap_or(uplinks).min(31) as u16;
    let bl = f.basic_long_count.unwrap_or(basic_long).min(1023);
    let counts = (up << 11) | bl;
    vec![
        id::HEARTBEAT,
        s1,
        s2,
        (ts & 0xFF) as u8,
        ((ts >> 8) & 0xFF) as u8,
        (counts >> 8) as u8,
        (counts & 0xFF) as u8,
    ]
}

// ---------------------------------------------------------------------------------------------
// Ownship (ID 10) / Traffic (ID 20) report — ICD §3.4, §3.5
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Report {
    /// "s": 0 = no alert, 1 = traffic alert.
    pub alert: u8,
    /// "t": 0 ADS-B ICAO, 1 ADS-B self-assigned, 2 TIS-B ICAO, 3 TIS-B track ID, 4 surface, 5 beacon.
    pub addr_type: u8,
    pub address: u32,
    pub lat: f64,
    pub lon: f64,
    /// Pressure altitude (29.92). `None` encodes 0xFFF.
    pub pressure_alt_ft: Option<f64>,
    pub airborne: bool,
    pub extrapolated: bool,
    /// 0 = invalid, 1 = true track, 2 = magnetic heading, 3 = true heading.
    pub track_type: u8,
    pub nic: u8,
    pub nacp: u8,
    /// `None` encodes 0xFFF.
    pub h_vel_kt: Option<f64>,
    /// `None` encodes 0x800.
    pub v_vel_fpm: Option<f64>,
    pub track_deg: f64,
    pub emitter: u8,
    pub callsign: String,
    pub emergency: u8,
    pub spare: u8,
}

pub fn encode_altitude(alt: Option<f64>) -> u16 {
    match alt {
        Some(a) => ((a + 1000.0) / 25.0).round().clamp(0.0, 4094.0) as u16,
        None => 0xFFF,
    }
}

pub fn encode_vvel(fpm: Option<f64>) -> u16 {
    match fpm {
        Some(v) => ((v / 64.0).round().clamp(-510.0, 510.0) as i16 as u16) & 0xFFF,
        None => 0x800,
    }
}

pub fn report(msg_id: u8, r: &Report) -> Vec<u8> {
    let mut m = Vec::with_capacity(28);
    m.push(msg_id);
    m.push(((r.alert & 0xF) << 4) | (r.addr_type & 0xF));
    m.extend_from_slice(&be24(r.address));
    m.extend_from_slice(&be24(encode_semicircle(r.lat)));
    m.extend_from_slice(&be24(encode_semicircle(r.lon)));
    let ddd = encode_altitude(r.pressure_alt_ft);
    let misc = ((r.airborne as u8) << 3) | ((r.extrapolated as u8) << 2) | (r.track_type & 3);
    m.push((ddd >> 4) as u8);
    m.push((((ddd & 0xF) as u8) << 4) | misc);
    m.push(((r.nic & 0xF) << 4) | (r.nacp & 0xF));
    let hhh: u16 = match r.h_vel_kt {
        Some(v) => v.round().clamp(0.0, 4094.0) as u16,
        None => 0xFFF,
    };
    let vvv = encode_vvel(r.v_vel_fpm);
    m.push((hhh >> 4) as u8);
    m.push((((hhh & 0xF) as u8) << 4) | (vvv >> 8) as u8);
    m.push((vvv & 0xFF) as u8);
    m.push(encode_track(r.track_deg));
    m.push(r.emitter);
    m.extend_from_slice(&callsign_bytes(&r.callsign));
    m.push(((r.emergency & 0xF) << 4) | (r.spare & 0xF));
    m
}

pub fn decode_report(m: &[u8]) -> Option<Report> {
    if m.len() < 28 {
        return None;
    }
    let u24 = |i: usize| (m[i] as u32) << 16 | (m[i + 1] as u32) << 8 | m[i + 2] as u32;
    let ddd = (m[11] as u16) << 4 | (m[12] >> 4) as u16;
    let misc = m[12] & 0xF;
    let hhh = (m[14] as u16) << 4 | (m[15] >> 4) as u16;
    let vvv = ((m[15] & 0xF) as u16) << 8 | m[16] as u16;
    let vfpm = if vvv == 0x800 {
        None
    } else {
        let s = if vvv & 0x800 != 0 { vvv as i32 - 0x1000 } else { vvv as i32 };
        Some(s as f64 * 64.0)
    };
    Some(Report {
        alert: m[1] >> 4,
        addr_type: m[1] & 0xF,
        address: u24(2),
        lat: decode_semicircle(u24(5)),
        lon: decode_semicircle(u24(8)),
        pressure_alt_ft: if ddd == 0xFFF { None } else { Some(ddd as f64 * 25.0 - 1000.0) },
        airborne: misc & 8 != 0,
        extrapolated: misc & 4 != 0,
        track_type: misc & 3,
        nic: m[13] >> 4,
        nacp: m[13] & 0xF,
        h_vel_kt: if hhh == 0xFFF { None } else { Some(hhh as f64) },
        v_vel_fpm: vfpm,
        track_deg: m[17] as f64 * 360.0 / 256.0,
        emitter: m[18],
        callsign: String::from_utf8_lossy(&m[19..27]).trim_end().to_string(),
        emergency: m[27] >> 4,
        spare: m[27] & 0xF,
    })
}

// ---------------------------------------------------------------------------------------------
// Ownship geometric altitude (ID 11) — ICD §3.8
// ---------------------------------------------------------------------------------------------

/// `vfom_m`: `None` = 0x7FFF (not available); values >= 32766 encode 0x7FFE.
pub fn ownship_geo_alt(alt_ft: f64, vertical_warning: bool, vfom_m: Option<u32>) -> Vec<u8> {
    let a = (alt_ft / 5.0).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16 as u16;
    let vfom: u16 = match vfom_m {
        Some(v) => v.min(0x7FFE) as u16,
        None => 0x7FFF,
    };
    let vm = ((vertical_warning as u16) << 15) | vfom;
    let mut m = vec![id::OWNSHIP_GEO_ALT];
    m.extend_from_slice(&be16(a));
    m.extend_from_slice(&be16(vm));
    m
}

// ---------------------------------------------------------------------------------------------
// Other ICD messages
// ---------------------------------------------------------------------------------------------

/// Uplink data (ID 7): TOR (80 ns units, LSB first; 0xFFFFFF = invalid) + 432-byte payload.
pub fn uplink(tor: u32, payload: &[u8; 432]) -> Vec<u8> {
    let mut m = Vec::with_capacity(436);
    m.push(id::UPLINK);
    m.extend_from_slice(&[(tor & 0xFF) as u8, (tor >> 8) as u8, (tor >> 16) as u8]);
    m.extend_from_slice(payload);
    m
}

/// Height above terrain (ID 9, normally display -> GDL 90). `None` = 0x8000 invalid.
pub fn height_above_terrain(ft: Option<i16>) -> Vec<u8> {
    let v = ft.map(|v| v as u16).unwrap_or(0x8000);
    vec![id::HEIGHT_ABOVE_TERRAIN, (v >> 8) as u8, v as u8]
}

/// Pass-through Basic (ID 30, 18-byte payload) or Long (ID 31, 34-byte payload) UAT report.
pub fn pass_through(long: bool, tor: u32, payload: &[u8]) -> Vec<u8> {
    let n = if long { 34 } else { 18 };
    let mut m = vec![if long { id::LONG_REPORT } else { id::BASIC_REPORT }];
    m.extend_from_slice(&[(tor & 0xFF) as u8, (tor >> 8) as u8, (tor >> 16) as u8]);
    let mut p = payload.to_vec();
    p.resize(n, 0);
    m.extend_from_slice(&p);
    m
}

// ---------------------------------------------------------------------------------------------
// ForeFlight extension (0x65) — "GDL 90 Extended Specification"
// ---------------------------------------------------------------------------------------------

/// ID message (sub-ID 0), 39 bytes.
/// capabilities bit 0: geo altitude datum (0 = WGS-84 ellipsoid, 1 = MSL);
/// bits 1-2: internet policy (0 unrestricted, 1 expensive, 2 disallowed).
pub fn foreflight_id(serial: Option<u64>, name: &str, long_name: &str, capabilities: u32) -> Vec<u8> {
    let mut m = vec![id::FOREFLIGHT, 0x00, 0x01];
    m.extend_from_slice(&serial.unwrap_or(u64::MAX).to_be_bytes());
    m.extend_from_slice(&put_name::<8>(name));
    m.extend_from_slice(&put_name::<16>(long_name));
    m.extend_from_slice(&capabilities.to_be_bytes());
    m
}

/// AHRS message (sub-ID 1), 12 bytes. Roll + = right wing down, pitch + = nose up,
/// 0.1 deg units; invalid = 0x7FFF. Heading bit 15: 1 = magnetic; invalid = 0xFFFF.
#[derive(Clone, Copy, Debug, Default)]
pub struct ForeFlightAhrs {
    pub roll: Option<f64>,
    pub pitch: Option<f64>,
    pub heading: Option<(f64, bool)>,
    pub ias: Option<f64>,
    pub tas: Option<f64>,
}

pub fn foreflight_ahrs(a: &ForeFlightAhrs) -> Vec<u8> {
    let ang = |v: Option<f64>| -> u16 {
        match v {
            Some(d) => ((d * 10.0).round().clamp(-1800.0, 1800.0) as i16) as u16,
            None => 0x7FFF,
        }
    };
    let hdg: u16 = match a.heading {
        Some((h, mag)) => {
            let t = (h.rem_euclid(360.0) * 10.0).round() as u16 % 3600;
            t | if mag { 0x8000 } else { 0 }
        }
        None => 0xFFFF,
    };
    let spd = |v: Option<f64>| -> u16 {
        match v {
            Some(k) => k.round().clamp(0.0, 65534.0) as u16,
            None => 0xFFFF,
        }
    };
    let mut m = vec![id::FOREFLIGHT, 0x01];
    m.extend_from_slice(&be16(ang(a.roll)));
    m.extend_from_slice(&be16(ang(a.pitch)));
    m.extend_from_slice(&be16(hdg));
    m.extend_from_slice(&be16(spd(a.ias)));
    m.extend_from_slice(&be16(spd(a.tas)));
    m
}

// ---------------------------------------------------------------------------------------------
// Levil / Stratux AHRS (0x4C 'LE'), Stratux heartbeat (0xCC) and status ('SX')
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct LevilAhrs {
    pub roll: Option<f64>,
    pub pitch: Option<f64>,
    pub heading: Option<f64>,
    pub slip_skid: Option<f64>,
    pub yaw_rate: Option<f64>,
    pub g_load: Option<f64>,
    pub ias: Option<f64>,
    /// Pressure altitude, ft.
    pub palt: Option<f64>,
    /// Baro vertical speed, fpm.
    pub vs: Option<f64>,
}

/// 24-byte 'LE' AHRS report as Stratux emits it (sub-type 0x45 0x01, version 0x01).
pub fn levil_ahrs(a: &LevilAhrs) -> Vec<u8> {
    let x10 = |v: Option<f64>| -> u16 {
        match v {
            Some(d) => (d * 10.0).round().clamp(-32767.0, 32766.0) as i16 as u16,
            None => 0x7FFF,
        }
    };
    let x1 = |v: Option<f64>| -> u16 {
        match v {
            Some(d) => d.round().clamp(-32767.0, 32766.0) as i16 as u16,
            None => 0x7FFF,
        }
    };
    let palt: u16 = match a.palt {
        Some(p) => (p + 5000.5).clamp(0.0, 65534.0) as u16,
        None => 0xFFFF,
    };
    let mut m = vec![id::LEVIL_AHRS, 0x45, 0x01, 0x01];
    for v in [
        x10(a.roll),
        x10(a.pitch),
        x10(a.heading),
        x10(a.slip_skid),
        x10(a.yaw_rate),
        x10(a.g_load),
        x10(a.ias),
        palt,
        x1(a.vs),
    ] {
        m.extend_from_slice(&be16(v));
    }
    m.extend_from_slice(&[0x7F, 0xFF]);
    m
}

/// Stratux heartbeat 0xCC: bit 1 GPS valid, bit 0 AHRS valid, bits 7..2 protocol version.
pub fn stratux_heartbeat(gps_valid: bool, ahrs_valid: bool, protocol: u8) -> Vec<u8> {
    vec![
        id::STRATUX_HEARTBEAT,
        ((protocol & 0x3F) << 2) | ((gps_valid as u8) << 1) | ahrs_valid as u8,
    ]
}

#[derive(Clone, Debug, Default)]
pub struct StratuxStatus {
    pub version: [u8; 4],
    pub gps_fix: u8,
    pub flags: u8,
    pub sats_locked: u8,
    pub sats_tracked: u8,
    pub uat_targets: u16,
    pub es_targets: u16,
    pub uat_msgs_min: u16,
    pub es_msgs_min: u16,
    pub cpu_temp_c: f64,
    pub towers: Vec<(f64, f64)>,
}

/// 'SX' status message (Stratux), version 1.
pub fn stratux_status(s: &StratuxStatus) -> Vec<u8> {
    let mut m = vec![0u8; 29];
    m[0] = b'S';
    m[1] = b'X';
    m[2] = 1;
    m[3..7].copy_from_slice(&s.version);
    m[7..11].copy_from_slice(&[0xFF; 4]);
    m[12] = s.flags;
    m[13] = s.gps_fix;
    m[15] = 1; // one radio
    m[16] = s.sats_locked;
    m[17] = s.sats_tracked;
    m[18..20].copy_from_slice(&be16(s.uat_targets));
    m[20..22].copy_from_slice(&be16(s.es_targets));
    m[22..24].copy_from_slice(&be16(s.uat_msgs_min));
    m[24..26].copy_from_slice(&be16(s.es_msgs_min));
    m[26..28].copy_from_slice(&be16((s.cpu_temp_c * 10.0).round().clamp(0.0, 65535.0) as u16));
    m[28] = s.towers.len().min(255) as u8;
    for (lat, lon) in s.towers.iter().take(255) {
        m.extend_from_slice(&be24(encode_semicircle(*lat)));
        m.extend_from_slice(&be24(encode_semicircle(*lon)));
    }
    m
}

// ---------------------------------------------------------------------------------------------
// Human-readable decode (packet log, `listen` and `decode` subcommands)
// ---------------------------------------------------------------------------------------------

pub fn message_name(m: &[u8]) -> &'static str {
    match m.first().copied() {
        Some(0) => "Heartbeat",
        Some(2) => "Initialization",
        Some(7) => "Uplink",
        Some(9) => "HeightAboveTerrain",
        Some(10) => "Ownship",
        Some(11) => "OwnshipGeoAlt",
        Some(20) => "Traffic",
        Some(30) => "BasicReport",
        Some(31) => "LongReport",
        Some(0x65) => match m.get(1) {
            Some(0) => "FF-ID",
            Some(1) => "FF-AHRS",
            _ => "FF-?",
        },
        Some(0x4C) => "LE-AHRS",
        Some(0xCC) => "StratuxHB",
        Some(0x53) if m.get(1) == Some(&b'X') => "SX-Status",
        _ => "Unknown",
    }
}

/// Message types with fixed sizes in the GDL 90 and ForeFlight specifications.
pub fn standard_replay_message(msg: &[u8]) -> bool {
    matches!((msg.first(), msg.get(1), msg.len()),
        (Some(0), _, 7) | (Some(7), _, 436) | (Some(9), _, 3) |
        (Some(10 | 20), _, 28) | (Some(11), _, 5) |
        (Some(30), _, 22) | (Some(31), _, 38) |
        (Some(0x65), Some(0), 39) | (Some(0x65), Some(1), 12))
}

pub fn describe(m: &[u8]) -> String {
    let name = message_name(m);
    match m.first().copied() {
        Some(0) if m.len() >= 7 => {
            let ts = m[3] as u32 | (m[4] as u32) << 8 | ((m[2] as u32 & 0x80) << 9);
            let counts = (m[5] as u16) << 8 | m[6] as u16;
            format!(
                "{name} gps={} init={} utc={} t={:02}:{:02}:{:02}Z up={} bl={}",
                (m[1] & 0x80 != 0) as u8,
                m[1] & 1,
                m[2] & 1,
                ts / 3600,
                ts / 60 % 60,
                ts % 60,
                counts >> 11,
                counts & 0x3FF
            )
        }
        Some(10) | Some(20) => match decode_report(m) {
            Some(r) => format!(
                "{name} {:06X}/t{} {:<8} {:.5},{:.5} {} {} {}kt {} trk{:.0} nic{} nacp{} {}{}",
                r.address,
                r.addr_type,
                r.callsign,
                r.lat,
                r.lon,
                r.pressure_alt_ft.map(|a| format!("{a:.0}ft")).unwrap_or("alt?".into()),
                if r.airborne { "air" } else { "gnd" },
                r.h_vel_kt.map(|v| format!("{v:.0}")).unwrap_or("?".into()),
                r.v_vel_fpm.map(|v| format!("{v:+.0}fpm")).unwrap_or("vs?".into()),
                r.track_deg,
                r.nic,
                r.nacp,
                if r.alert != 0 { "ALERT " } else { "" },
                if r.emergency != 0 { format!("emerg{}", r.emergency) } else { String::new() }
            ),
            None => format!("{name} (short {}B)", m.len()),
        },
        Some(11) if m.len() >= 5 => {
            let a = i16::from_be_bytes([m[1], m[2]]) as i32 * 5;
            let vm = u16::from_be_bytes([m[3], m[4]]);
            format!("{name} {a}ft warn={} vfom={}", vm >> 15, vm & 0x7FFF)
        }
        Some(7) if m.len() >= 436 => {
            let tor = m[1] as u32 | (m[2] as u32) << 8 | (m[3] as u32) << 16;
            let products = crate::fisb::summarize_payload(&m[4..436]);
            format!("{name} tor={tor} {products}")
        }
        Some(0x65) if m.get(1) == Some(&1) && m.len() >= 12 => {
            let g = |i: usize| u16::from_be_bytes([m[i], m[i + 1]]);
            let ang = |v: u16| if v == 0x7FFF { "inv".to_string() } else { format!("{:.1}", v as i16 as f64 / 10.0) };
            let h = g(6);
            let hdg = if h == 0xFFFF {
                "inv".into()
            } else {
                format!("{:.1}{}", (h & 0x7FFF) as f64 / 10.0, if h & 0x8000 != 0 { "M" } else { "T" })
            };
            format!("{name} roll={} pitch={} hdg={} ias={} tas={}", ang(g(2)), ang(g(4)), hdg, g(8), g(10))
        }
        Some(0x65) if m.get(1) == Some(&0) && m.len() >= 39 => {
            let nm = String::from_utf8_lossy(&m[11..19]).trim_end_matches('\0').to_string();
            let ln = String::from_utf8_lossy(&m[19..35]).trim_end_matches('\0').to_string();
            let caps = u32::from_be_bytes([m[35], m[36], m[37], m[38]]);
            format!("{name} '{nm}' '{ln}' caps={caps:#x}")
        }
        Some(0x4C) if m.len() >= 24 => {
            let g = |i: usize| i16::from_be_bytes([m[i], m[i + 1]]);
            let f = |v: i16| if v == 0x7FFF { "inv".to_string() } else { format!("{:.1}", v as f64 / 10.0) };
            format!(
                "{name} roll={} pitch={} hdg={} g={} palt={}",
                f(g(4)),
                f(g(6)),
                f(g(8)),
                f(g(14)),
                u16::from_be_bytes([m[18], m[19]]) as i32 - 5000
            )
        }
        Some(0x53) if m.get(1) == Some(&b'X') && m.len() >= 29 => format!(
            "{name} v{}.{}.{} gps_fix={} sats={}/{} uat_targets={} towers={}",
            m[3],
            m[4],
            m[5],
            m[13],
            m[16],
            m[17],
            u16::from_be_bytes([m[18], m[19]]),
            m[28]
        ),
        Some(0xCC) if m.len() >= 2 => {
            format!("{name} gps={} ahrs={} ver={}", (m[1] >> 1) & 1, m[1] & 1, m[1] >> 2)
        }
        _ => format!("{name} id=0x{:02X} len={}", m.first().copied().unwrap_or(0), m.len()),
    }
}

pub fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        s.push_str(&format!("{b:02X}"));
    }
    s
}

pub fn parse_hex(s: &str) -> anyhow::Result<Vec<u8>> {
    let clean: String = s
        .chars()
        .filter(|c| c.is_ascii_hexdigit())
        .collect();
    if clean.len() % 2 != 0 {
        anyhow::bail!("odd number of hex digits");
    }
    (0..clean.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).map_err(Into::into))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ICD §2.2.4: [7E 00 81 41 DB D0 08 02 B3 8B 7E]
    #[test]
    fn icd_heartbeat_example_crc_and_frame() {
        let msg = [0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02];
        assert_eq!(crc16(&msg), 0x8BB3);
        assert_eq!(
            frame(&msg),
            vec![0x7E, 0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02, 0xB3, 0x8B, 0x7E]
        );
    }

    #[test]
    fn icd_heartbeat_fields() {
        let f = HeartbeatFields { csa_requested: true, ..Default::default() };
        // ts 0xD0DB, 1 uplink, 2 basic/long
        let m = heartbeat(&f, true, 0xD0DB, 1, 2);
        assert_eq!(m, vec![0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02]);
        // §3.1.4 example: 4 uplinks + 567 basic/long -> 0x22 0x37
        let m = heartbeat(&f, true, 0, 4, 567);
        assert_eq!(&m[5..7], &[0x22, 0x37]);
        // bit 16 of the timestamp lands in status byte 2 bit 7
        let m = heartbeat(&f, true, 86399, 0, 0);
        assert_eq!(m[2] & 0x80, 0x80);
    }

    #[test]
    fn byte_stuffing() {
        let f = frame(&[0x02, 0x7E]);
        assert_eq!(&f[..4], &[0x7E, 0x02, 0x7D, 0x5E]);
        let f = frame(&[0x03, 0x7D]);
        assert_eq!(&f[..4], &[0x7E, 0x03, 0x7D, 0x5D]);
    }

    /// ICD §3.5.2 Table 12 — traffic report over Salem OR.
    #[test]
    fn icd_traffic_example() {
        let r = Report {
            alert: 0,
            addr_type: 0,
            address: 0o52642511,
            lat: 44.90708,
            lon: -122.99488,
            pressure_alt_ft: Some(5000.0),
            airborne: true,
            extrapolated: false,
            track_type: 1,
            nic: 10,
            nacp: 9,
            h_vel_kt: Some(123.0),
            v_vel_fpm: Some(64.0),
            track_deg: 45.0,
            emitter: 1,
            callsign: "N825V".into(),
            emergency: 0,
            spare: 0,
        };
        let m = report(id::TRAFFIC, &r);
        let want = [
            0x14, 0x00, 0xAB, 0x45, 0x49, 0x1F, 0xEF, 0x15, 0xA8, 0x89, 0x78, 0x0F, 0x09, 0xA9, 0x07,
            0xB0, 0x01, 0x20, 0x01, 0x4E, 0x38, 0x32, 0x35, 0x56, 0x20, 0x20, 0x20, 0x00,
        ];
        assert_eq!(m, want);
        let back = decode_report(&m).unwrap();
        assert!((back.lat - r.lat).abs() < 3e-5 && (back.lon - r.lon).abs() < 3e-5);
        assert_eq!(back.pressure_alt_ft, Some(5000.0));
        assert_eq!(back.v_vel_fpm, Some(64.0));
    }

    #[test]
    fn semicircle_examples() {
        assert_eq!(encode_semicircle(0.0), 0x000000);
        assert_eq!(encode_semicircle(45.0), 0x200000);
        assert_eq!(encode_semicircle(-45.0), 0xE00000);
        assert_eq!(encode_semicircle(90.0), 0x400000);
        assert_eq!(encode_semicircle(-180.0), 0x800000);
        assert_eq!(encode_semicircle(180.0), 0x7FFFFF);
    }

    #[test]
    fn altitude_and_vvel_examples() {
        assert_eq!(encode_altitude(Some(-1000.0)), 0x000);
        assert_eq!(encode_altitude(Some(0.0)), 0x028);
        assert_eq!(encode_altitude(Some(1000.0)), 0x050);
        assert_eq!(encode_altitude(Some(101350.0)), 0xFFE);
        assert_eq!(encode_altitude(None), 0xFFF);
        assert_eq!(encode_vvel(Some(0.0)), 0x000);
        assert_eq!(encode_vvel(Some(64.0)), 0x001);
        assert_eq!(encode_vvel(Some(-64.0)), 0xFFF);
        assert_eq!(encode_vvel(Some(32576.0)), 0x1FD);
        assert_eq!(encode_vvel(Some(40000.0)), 0x1FE);
        assert_eq!(encode_vvel(Some(-32576.0)), 0xE03);
        assert_eq!(encode_vvel(Some(-40000.0)), 0xE02);
        assert_eq!(encode_vvel(None), 0x800);
    }

    #[test]
    fn geo_alt_examples() {
        assert_eq!(&ownship_geo_alt(-1000.0, false, Some(10))[1..3], &[0xFF, 0x38]);
        assert_eq!(&ownship_geo_alt(1000.0, false, Some(10))[1..3], &[0x00, 0xC8]);
        assert_eq!(&ownship_geo_alt(0.0, true, None)[3..5], &[0xFF, 0xFF]);
        assert_eq!(&ownship_geo_alt(0.0, false, Some(40000))[3..5], &[0x7F, 0xFE]);
        assert_eq!(&ownship_geo_alt(0.0, true, Some(50))[3..5], &[0x80, 0x32]);
    }

    #[test]
    fn deframer_roundtrip_and_bad_crc() {
        let a = frame(&[0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02]);
        let b = frame(&[0x02, 0x7E, 0x7D, 0x00]);
        let mut c = frame(&[0x0B, 0x00, 0x10, 0x00, 0x0A]);
        c[2] ^= 1; // Damaged input must be rejected by the decoder.
        let mut stream = a.clone();
        stream.extend(&b);
        stream.extend(&c);
        let mut d = Deframer::default();
        // feed in odd-sized chunks
        let mut frames = Vec::new();
        for chunk in stream.chunks(5) {
            frames.extend(d.push(chunk));
        }
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0], Frame::Ok(vec![0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02]));
        assert_eq!(frames[1], Frame::Ok(vec![0x02, 0x7E, 0x7D, 0x00]));
        assert!(matches!(frames[2], Frame::BadCrc { .. }));
    }

    #[test]
    fn foreflight_lengths() {
        assert_eq!(foreflight_id(None, "Stratux", "Stratux", 0).len(), 39);
        assert_eq!(foreflight_ahrs(&ForeFlightAhrs::default()).len(), 12);
        assert_eq!(levil_ahrs(&LevilAhrs::default()).len(), 24);
        let m = foreflight_ahrs(&ForeFlightAhrs {
            roll: Some(-12.3),
            pitch: Some(4.5),
            heading: Some((270.0, true)),
            ias: Some(100.0),
            tas: None,
        });
        assert_eq!(&m[2..4], &(-123i16).to_be_bytes());
        assert_eq!(&m[6..8], &(2700u16 | 0x8000).to_be_bytes());
        assert_eq!(&m[10..12], &[0xFF, 0xFF]);
    }
}
