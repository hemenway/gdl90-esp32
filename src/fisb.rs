//! UAT ground uplink payload (ICD §4–5): UAT-specific header, information frames, FIS-B APDUs,
//! DLAC text (product 413) and NEXRAD global-block run-length products (63/64).

use crate::geo;

pub const PAYLOAD_LEN: usize = 432;
pub const APP_DATA_LEN: usize = 424;

// ---------------------------------------------------------------------------------------------
// Bit writer
// ---------------------------------------------------------------------------------------------

#[derive(Default)]
struct BitWriter {
    bytes: Vec<u8>,
    nbits: usize,
}

impl BitWriter {
    fn put(&mut self, value: u32, bits: usize) {
        for i in (0..bits).rev() {
            let bit = (value >> i) & 1;
            if self.nbits % 8 == 0 {
                self.bytes.push(0);
            }
            if bit != 0 {
                let last = self.bytes.len() - 1;
                self.bytes[last] |= 0x80 >> (self.nbits % 8);
            }
            self.nbits += 1;
        }
    }
    fn finish(self) -> Vec<u8> {
        self.bytes // trailing bits are already zero-filled
    }
}

// ---------------------------------------------------------------------------------------------
// DLAC 6-bit text (RTCA DO-267A Appendix K)
// ---------------------------------------------------------------------------------------------

/// Index -> character. 0 = ETX, 28 = TAB (next char is a space count), 29 = RS, 30 = LF.
pub const DLAC_ALPHABET: &[u8; 64] =
    b"\x03ABCDEFGHIJKLMNOPQRSTUVWXYZ\x1A\t\x1E\n| !\"#$%&'()*+,-./0123456789:;<=>?";
pub const DLAC_RS: u8 = 29;

pub fn dlac_index(c: char) -> u8 {
    let c = c.to_ascii_uppercase();
    match c {
        'A'..='Z' => c as u8 - b'A' + 1,
        ' '..='?' => c as u8, // 32..63 map 1:1 onto ASCII 0x20..0x3F
        '\x03' => 0,
        '\x1E' => DLAC_RS,
        '\n' => 30,
        '|' => 31,
        '\t' => 32, // send tabs as plain spaces
        _ => b'?',
    }
}

/// Pack 6-bit DLAC codes MSB-first, zero-filled to a byte boundary.
pub fn dlac_pack(codes: &[u8]) -> Vec<u8> {
    let mut w = BitWriter::default();
    for &c in codes {
        w.put((c & 0x3F) as u32, 6);
    }
    w.finish()
}

/// Unpack 6-bit codes (no tab expansion).
pub fn dlac_unpack(bytes: &[u8]) -> Vec<u8> {
    let total = bytes.len() * 8 / 6;
    (0..total)
        .map(|i| {
            let bit = i * 6;
            let mut v = 0u8;
            for k in 0..6 {
                let b = bit + k;
                v = (v << 1) | ((bytes[b / 8] >> (7 - b % 8)) & 1);
            }
            v
        })
        .collect()
}

/// Decode to text, expanding TAB runs; stops at ETX.
pub fn dlac_decode(bytes: &[u8]) -> String {
    let mut out = String::new();
    let mut tab = false;
    for c in dlac_unpack(bytes) {
        if tab {
            out.extend(std::iter::repeat(' ').take(c as usize));
            tab = false;
        } else if c == 0 {
            break;
        } else if c == 28 {
            tab = true;
        } else {
            out.push(DLAC_ALPHABET[c as usize] as char);
        }
    }
    out
}

/// Encode one text record ("METAR KSLE 091753Z ...") terminated with RS and byte-aligned.
pub fn text_record(text: &str) -> Vec<u8> {
    let mut codes: Vec<u8> = text.trim_end_matches('\x1E').chars().map(dlac_index).collect();
    codes.push(DLAC_RS);
    dlac_pack(&codes)
}

// ---------------------------------------------------------------------------------------------
// APDU header and information frames
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct ApduTime {
    /// 0 = hh:mm, 1 = hh:mm:ss, 2 = mon/day hh:mm, 3 = mon/day hh:mm:ss
    pub t_opt: u8,
    pub month: u8,
    pub day: u8,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
}

impl ApduTime {
    pub fn hm(hours: u8, minutes: u8) -> Self {
        Self { t_opt: 0, month: 0, day: 0, hours, minutes, seconds: 0 }
    }
    pub fn now(t_opt: u8) -> Self {
        let (_, mo, d, h, mi, s) = geo::utc_now_fields();
        Self { t_opt, month: mo, day: d, hours: h, minutes: mi, seconds: s }
    }
}

/// APDU header without the 16-bit APDU ID (UAT variant, ICD §4.3.1), no segmentation.
pub fn apdu(product_id: u16, time: ApduTime, payload: &[u8]) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.put(0, 3); // A, G, P flags
    w.put(product_id as u32 & 0x7FF, 11);
    w.put(0, 1); // S flag
    w.put(time.t_opt as u32 & 3, 2);
    if time.t_opt >= 2 {
        w.put(time.month as u32, 4);
        w.put(time.day as u32, 5);
    }
    w.put(time.hours as u32, 5);
    w.put(time.minutes as u32, 6);
    if time.t_opt == 1 || time.t_opt == 3 {
        w.put(time.seconds as u32, 6);
    }
    let mut out = w.finish();
    out.extend_from_slice(payload);
    out
}

/// Information frame: 9-bit length, 3 reserved bits, 4-bit frame type, then data.
pub fn info_frame(frame_type: u8, data: &[u8]) -> Vec<u8> {
    let len = data.len().min(422) as u16;
    let mut out = vec![(len >> 1) as u8, (((len & 1) as u8) << 7) | (frame_type & 0x0F)];
    out.extend_from_slice(&data[..len as usize]);
    out
}

// ---------------------------------------------------------------------------------------------
// UAT-specific header (DO-282 §2.2.3.2.2) and payload packing
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct UplinkHeader {
    pub lat: f64,
    pub lon: f64,
    pub position_valid: bool,
    pub utc_coupled: bool,
    pub app_data_valid: bool,
    pub slot_id: u8,
    pub tisb_site_id: u8,
}

pub fn encode_uplink_header(h: &UplinkHeader) -> [u8; 8] {
    let scale = (1u32 << 24) as f64 / 360.0;
    let lat = if h.lat < 0.0 { h.lat + 180.0 } else { h.lat };
    let lon = if h.lon < 0.0 { h.lon + 360.0 } else { h.lon };
    let raw_lat = ((lat * scale).round() as u32) & 0x7F_FFFF;
    let raw_lon = ((lon * scale).round() as u32) & 0xFF_FFFF;
    let mut b = [0u8; 8];
    b[0] = (raw_lat >> 15) as u8;
    b[1] = (raw_lat >> 7) as u8;
    b[2] = ((raw_lat & 0x7F) << 1) as u8 | (raw_lon >> 23) as u8;
    b[3] = (raw_lon >> 15) as u8;
    b[4] = (raw_lon >> 7) as u8;
    b[5] = ((raw_lon & 0x7F) << 1) as u8 | h.position_valid as u8;
    b[6] = ((h.utc_coupled as u8) << 7) | ((h.app_data_valid as u8) << 5) | (h.slot_id & 0x1F);
    b[7] = (h.tisb_site_id & 0x0F) << 4;
    b
}

pub fn decode_uplink_header(b: &[u8]) -> UplinkHeader {
    let raw_lat = (b[0] as u32) << 15 | (b[1] as u32) << 7 | (b[2] as u32) >> 1;
    let raw_lon = ((b[2] & 1) as u32) << 23 | (b[3] as u32) << 15 | (b[4] as u32) << 7 | (b[5] as u32) >> 1;
    let mut lat = raw_lat as f64 * 360.0 / 16_777_216.0;
    let mut lon = raw_lon as f64 * 360.0 / 16_777_216.0;
    if lat > 90.0 {
        lat -= 180.0;
    }
    if lon > 180.0 {
        lon -= 360.0;
    }
    UplinkHeader {
        lat,
        lon,
        position_valid: b[5] & 1 != 0,
        utc_coupled: b[6] & 0x80 != 0,
        app_data_valid: b[6] & 0x20 != 0,
        slot_id: b[6] & 0x1F,
        tisb_site_id: b[7] >> 4,
    }
}

/// Greedily pack information frames into as few 432-byte payloads as possible.
pub fn pack_payloads(header: &UplinkHeader, frames: &[Vec<u8>]) -> Vec<[u8; PAYLOAD_LEN]> {
    let mut out = Vec::new();
    let mut cur: Vec<u8> = Vec::new();
    let flush = |cur: &mut Vec<u8>, out: &mut Vec<[u8; PAYLOAD_LEN]>| {
        if cur.is_empty() {
            return;
        }
        let mut p = [0u8; PAYLOAD_LEN];
        p[..8].copy_from_slice(&encode_uplink_header(header));
        p[8..8 + cur.len()].copy_from_slice(cur);
        out.push(p);
        cur.clear();
    };
    for f in frames {
        if f.len() > APP_DATA_LEN {
            continue;
        }
        if cur.len() + f.len() > APP_DATA_LEN {
            flush(&mut cur, &mut out);
        }
        cur.extend_from_slice(f);
    }
    flush(&mut cur, &mut out);
    out
}

// ---------------------------------------------------------------------------------------------
// NEXRAD (products 63 regional / 64 CONUS), global block representation, northern hemisphere
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Cell {
    pub lat: f64,
    pub lon: f64,
    pub radius_nm: f64,
    /// Peak intensity 2..7 (VIP 1..6).
    pub intensity: u8,
}

/// Block number for the 4' x 48' block containing (lat, lon), lat 0..60 N.
pub fn block_number(lat: f64, lon: f64) -> Option<u32> {
    if !(0.0..60.0).contains(&lat) {
        return None;
    }
    let row = (lat * 15.0).floor() as u32; // 4 arc-minutes per row
    let col = (lon.rem_euclid(360.0) / 0.8).floor() as u32 % 450;
    Some(row * 450 + col)
}

/// South-west corner of a block (deg).
pub fn block_origin(bn: u32) -> (f64, f64) {
    let row = bn / 450;
    let col = bn % 450;
    let lat = row as f64 / 15.0;
    let mut lon = col as f64 * 0.8;
    if lon >= 180.0 {
        lon -= 360.0;
    }
    (lat, lon)
}

fn cell_intensity(cells: &[Cell], lat: f64, lon: f64) -> u8 {
    let mut best = 0u8;
    for c in cells {
        let d = geo::distance_nm(c.lat, c.lon, lat, lon);
        if d < c.radius_nm && c.radius_nm > 0.0 {
            let f = 1.0 - d / c.radius_nm;
            let peak = c.intensity.clamp(1, 7) as f64;
            let lvl = (1.0 + (peak - 1.0) * f.powf(0.8)).round() as u8;
            best = best.max(lvl.clamp(1, 7));
        }
    }
    best
}

/// 128 bins (4 rows x 32), row 0 = northern row, west to east.
pub fn block_bins(bn: u32, cells: &[Cell]) -> [u8; 128] {
    let (lat0, lon0) = block_origin(bn);
    let mut bins = [0u8; 128];
    for r in 0..4 {
        for c in 0..32 {
            let lat = lat0 + (4.0 - r as f64 - 0.5) / 60.0;
            let lon = lon0 + (c as f64 + 0.5) * 1.5 / 60.0;
            bins[r * 32 + c] = cell_intensity(cells, lat, lon);
        }
    }
    bins
}

pub fn rle_encode(bins: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < bins.len() {
        let v = bins[i] & 7;
        let mut run = 1;
        while i + run < bins.len() && bins[i + run] & 7 == v && run < 32 {
            run += 1;
        }
        out.push((((run - 1) as u8) << 3) | v);
        i += run;
    }
    out
}

pub fn nexrad_block_apdu(product_id: u16, time: ApduTime, bn: u32, bins: &[u8; 128]) -> Vec<u8> {
    let mut payload = vec![0x80 | ((bn >> 16) & 0x0F) as u8, (bn >> 8) as u8, bn as u8];
    payload.extend(rle_encode(bins));
    apdu(product_id, time, &payload)
}

/// All blocks touched by the cells (non-empty ones only), as FIS-B information frames.
pub fn nexrad_frames(product_id: u16, time: ApduTime, cells: &[Cell]) -> Vec<Vec<u8>> {
    let mut blocks = std::collections::BTreeSet::new();
    for c in cells {
        let dlat = c.radius_nm / 60.0;
        let dlon = c.radius_nm / (60.0 * c.lat.to_radians().cos().max(0.1));
        let mut lat = c.lat - dlat;
        while lat <= c.lat + dlat + 1.0 / 15.0 {
            let mut lon = c.lon - dlon;
            while lon <= c.lon + dlon + 0.8 {
                if let Some(bn) = block_number(lat.min(c.lat + dlat), lon.min(c.lon + dlon)) {
                    blocks.insert(bn);
                }
                lon += 0.8;
            }
            lat += 1.0 / 15.0;
        }
    }
    blocks
        .into_iter()
        .filter_map(|bn| {
            let bins = block_bins(bn, cells);
            if bins.iter().all(|&b| b == 0) {
                None
            } else {
                Some(info_frame(0, &nexrad_block_apdu(product_id, time, bn, &bins)))
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------
// Decoding (packet log / self-test)
// ---------------------------------------------------------------------------------------------

pub struct DecodedFrame {
    pub frame_type: u8,
    pub product_id: Option<u16>,
    pub t_opt: u8,
    pub data: Vec<u8>,
    pub apdu_payload: Vec<u8>,
}

pub fn parse_app_data(app: &[u8]) -> Vec<DecodedFrame> {
    let mut out = Vec::new();
    let mut p = 0;
    while p + 2 <= app.len() {
        let len = ((app[p] as usize) << 1) | (app[p + 1] as usize >> 7);
        let ftype = app[p + 1] & 0x0F;
        if len == 0 || p + 2 + len > app.len() {
            break;
        }
        let data = app[p + 2..p + 2 + len].to_vec();
        let (pid, t_opt, payload) = if ftype == 0 && data.len() >= 4 {
            let pid = (((data[0] & 0x1F) as u16) << 6) | (data[1] >> 2) as u16;
            let t_opt = ((data[1] & 1) << 1) | (data[2] >> 7);
            let hdr = match t_opt {
                0 => 4,
                1 | 2 => 5,
                _ => 6,
            };
            (Some(pid), t_opt, data.get(hdr..).unwrap_or(&[]).to_vec())
        } else {
            (None, 0, Vec::new())
        };
        out.push(DecodedFrame { frame_type: ftype, product_id: pid, t_opt, data, apdu_payload: payload });
        p += 2 + len;
    }
    out
}

pub fn summarize_payload(payload: &[u8]) -> String {
    if payload.len() < PAYLOAD_LEN {
        return "short".into();
    }
    let h = decode_uplink_header(&payload[..8]);
    let frames = parse_app_data(&payload[8..]);
    let mut counts: std::collections::BTreeMap<u16, usize> = Default::default();
    let mut first_text = None;
    for f in &frames {
        if let Some(pid) = f.product_id {
            *counts.entry(pid).or_default() += 1;
            if pid == 413 && first_text.is_none() {
                first_text = Some(dlac_decode(&f.apdu_payload));
            }
        }
    }
    let prods: Vec<String> = counts.iter().map(|(p, n)| format!("p{p}x{n}")).collect();
    let mut s = format!("gs {:.3},{:.3} slot{} [{}]", h.lat, h.lon, h.slot_id, prods.join(" "));
    if let Some(t) = first_text {
        let t: String = t.trim_end_matches('\x1E').chars().take(48).collect();
        s.push_str(&format!(" \"{t}\""));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gdl90::parse_hex;

    const ICD_TAF_SAMPLE: &str = "2180067441905011a02d3305832db0e70c1a04d832d71cf1d60c38c30d8b5204364cd806157c36c\
2008b3b1cb079c146370d30c205920b0ccb5204364cd8130d4cb5c3d79d2180067441905011a02d\
0118832db0e70c1a04d832d71cf1d60c38c30d8b5204364cd806157c36c2008b3b1cb079c146370\
d30c205920b0ccb5204364cd8130d4cb5c3d79d2180067441905011a02c5547832db0e70c1a04d8\
32d71cf1d60c38c30d8b5204364cd806157c36c2008b3b1cb079c146370d30c205920b0ccb52043\
64cd8130d4cb5c3d79d2180067441905011a02c14d4832db0e70c1a04d832d71cf1d60c38c30d8b\
5204364cd806157c36c2008b3b1cb079c146370d30c205920b0ccb5204364cd8130d4cb5c3d79d2\
180067441905011a02c824f832db0e70c1a04d832d71cf1d60c38c30d8b5204364cd806157c36c2\
008b3b1cb079c146370d30c205920b0ccb5204364cd8130d4cb5c3d79d";

    /// ICD §5.2.4: five TAF APDUs, each I-frame length 67, product 413, 16:25.
    #[test]
    fn icd_taf_sample_decodes_and_reencodes() {
        let app = parse_hex(ICD_TAF_SAMPLE).unwrap();
        let frames = parse_app_data(&app);
        assert_eq!(frames.len(), 5);
        let stations = ["KSLE", "KPDX", "KEUG", "KAST", "KHIO"];
        for (f, st) in frames.iter().zip(stations) {
            assert_eq!(f.product_id, Some(413));
            assert_eq!(f.data.len(), 67);
            let text = dlac_decode(&f.apdu_payload);
            assert!(text.starts_with(&format!("TAF {st} 260900Z")), "{text:?}");
            // Re-encode from the raw 6-bit codes and compare the whole frame byte-for-byte.
            let codes = dlac_unpack(&f.apdu_payload);
            let n = codes.iter().position(|&c| c == DLAC_RS).unwrap() + 1;
            let rebuilt = info_frame(0, &apdu(413, ApduTime::hm(16, 25), &dlac_pack(&codes[..n])));
            let mut orig = vec![0x21, 0x80];
            orig.extend_from_slice(&f.data);
            assert_eq!(rebuilt, orig);
        }
    }

    #[test]
    fn text_record_roundtrip() {
        let rec = "METAR KSLE 091753Z 31008KT 10SM FEW035 14/06 A3008 RMK AO2=";
        let enc = text_record(rec);
        let dec = dlac_decode(&enc);
        assert_eq!(dec.trim_end_matches('\x1E'), rec);
        assert!(dec.ends_with('\x1E'));
    }

    /// ICD §5.1.4: block reference 0x84 0xA5 0x70 = run-length block 0x4A570, 45°04'-45°08' N,
    /// 123°12'-122°24' W.
    #[test]
    fn icd_nexrad_block_location() {
        let bn = 0x4A570;
        let (lat, lon) = block_origin(bn);
        assert!((lat - (45.0 + 4.0 / 60.0)).abs() < 1e-9);
        assert!((lon - (-(123.0 + 12.0 / 60.0))).abs() < 1e-9);
        assert_eq!(block_number(45.1, -123.0), Some(bn));
        let app = parse_hex("130000FC000084A570308950111A53120930110A23451B0A0918090A1B0C1D0607061D041B0A010820").unwrap();
        let frames = parse_app_data(&app);
        assert_eq!(frames[0].product_id, Some(63));
        assert_eq!(&frames[0].apdu_payload[..3], &[0x84, 0xA5, 0x70]);
        // RLE in the sample covers exactly 128 bins
        let total: usize = frames[0].apdu_payload[3..].iter().map(|b| (b >> 3) as usize + 1).sum();
        assert_eq!(total, 128);
    }

    #[test]
    fn uplink_header_roundtrip() {
        let h = UplinkHeader {
            lat: 44.9,
            lon: -123.0,
            position_valid: true,
            utc_coupled: true,
            app_data_valid: true,
            slot_id: 7,
            tisb_site_id: 3,
        };
        let b = encode_uplink_header(&h);
        let d = decode_uplink_header(&b);
        assert!((d.lat - 44.9).abs() < 1e-4 && (d.lon + 123.0).abs() < 1e-4);
        assert!(d.position_valid && d.utc_coupled && d.app_data_valid);
        assert_eq!((d.slot_id, d.tisb_site_id), (7, 3));
    }

    #[test]
    fn nexrad_cells_produce_valid_rle() {
        let cells = [Cell { lat: 45.0, lon: -122.8, radius_nm: 12.0, intensity: 6 }];
        let frames = nexrad_frames(63, ApduTime::hm(12, 0), &cells);
        assert!(!frames.is_empty());
        let mut app = Vec::new();
        for f in &frames {
            app.extend_from_slice(f);
        }
        for f in parse_app_data(&app) {
            assert_eq!(f.product_id, Some(63));
            let total: usize = f.apdu_payload[3..].iter().map(|b| (b >> 3) as usize + 1).sum();
            assert_eq!(total, 128);
        }
    }

    #[test]
    fn apdu_header_t_opt2_is_five_bytes() {
        let t = ApduTime { t_opt: 2, month: 11, day: 9, hours: 17, minutes: 53, seconds: 0 };
        let a = apdu(8, t, &[]);
        assert_eq!(a.len(), 5);
        let pid = (((a[0] & 0x1F) as u16) << 6) | (a[1] >> 2) as u16;
        assert_eq!(pid, 8);
        assert_eq!(((a[1] & 1) << 1) | (a[2] >> 7), 2);
    }
}
