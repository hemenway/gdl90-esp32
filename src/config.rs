//! Every user-controllable setting. The web UI edits this tree with JSON merge patches
//! (RFC 7386), so anything here is "full control" without bespoke endpoints.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::fisb::Cell;
use crate::gdl90::HeartbeatFields;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Stream {
    pub enabled: bool,
    pub rate_hz: f64,
}

impl Default for Stream {
    fn default() -> Self {
        Self { enabled: false, rate_hz: 1.0 }
    }
}

fn on(rate_hz: f64) -> Stream {
    Stream { enabled: true, rate_hz }
}
fn off(rate_hz: f64) -> Stream {
    Stream { enabled: false, rate_hz }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Streams {
    /// 0x00 Heartbeat
    pub heartbeat: Stream,
    /// 0x0A Ownship report
    pub ownship: Stream,
    /// 0x0B Ownship geometric altitude (iFly only accepts ADS-B GPS once it sees this)
    pub geo_alt: Stream,
    /// 0x14 Traffic reports (one per target per period)
    pub traffic: Stream,
    /// 0x07 FIS-B uplinks (messages per second)
    pub uplink: Stream,
    /// 0x09 Height above terrain (normally an input to the GDL 90)
    pub hat: Stream,
    /// 0x65/0 ForeFlight ID
    pub ff_id: Stream,
    /// 0x65/1 ForeFlight AHRS
    pub ff_ahrs: Stream,
    /// 0x4C 'LE' AHRS (Levil / Stratux)
    pub levil_ahrs: Stream,
    /// 0xCC Stratux heartbeat
    pub stratux_heartbeat: Stream,
    /// 'SX' Stratux status (iFly identifies Stratux by this)
    pub stratux_status: Stream,
    /// 0x65 SkyRadar status (any 0x65 that is not ID/AHRS)
    pub skyradar_status: Stream,
    /// 0x3C Clarity status / 0x3D Clarity AHRS
    pub clarity_status: Stream,
    pub clarity_ahrs: Stream,
    /// 0x44 XGPS190 AHRS
    pub xgps_ahrs: Stream,
    /// 0x46 Pathfinder status (sub 1) / AHRS (sub 2)
    pub pathfinder_status: Stream,
    pub pathfinder_ahrs: Stream,
    /// ForeFlight/X-Plane simulator text (XGPS / XATT / XTRAFFIC) to `output.xplane_port`
    pub xplane_gps: Stream,
    pub xplane_att: Stream,
    pub xplane_traffic: Stream,
    /// NMEA 0183 ($GPRMC + $GPGGA) to `output.nmea_port`
    pub nmea: Stream,
}

impl Default for Streams {
    fn default() -> Self {
        profile_streams("generic")
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct OwnshipConfig {
    /// 24-bit ICAO address, hex.
    pub icao: String,
    pub callsign: String,
    pub emitter: u8,
    pub addr_type: u8,
    pub nic: u8,
    pub nacp: u8,
    /// Master GPS switch: false zeroes lat/lon/NIC (ICD §3.4) and drops 0x0B.
    pub gps_valid: bool,
    pub alert: u8,
    pub emergency: u8,
    /// 0 invalid, 1 true track, 2 magnetic heading, 3 true heading.
    pub track_type: u8,
    /// "auto" (from the flight model), "air" or "ground".
    pub air_ground: String,
    pub extrapolated: bool,
    pub alt_invalid: bool,
    pub hvel_invalid: bool,
    pub vvel_invalid: bool,
    pub spare: u8,
    /// Datum for 0x0B: "hae" (ICD: height above WGS-84 ellipsoid) or "msl".
    pub geo_alt_datum: String,
    /// Vertical figure of merit in metres; -1 = not available (0x7FFF).
    pub vfom_m: i32,
    pub vertical_warning: bool,
    /// Height-above-terrain override for 0x09; null = altitude minus ground elevation.
    pub hat_ft: Option<i32>,
}

impl Default for OwnshipConfig {
    fn default() -> Self {
        Self {
            icao: "A5B6C7".into(),
            callsign: "SIM001".into(),
            emitter: 1,
            addr_type: 0,
            nic: 9,
            nacp: 10,
            gps_valid: true,
            alert: 0,
            emergency: 0,
            track_type: 1,
            air_ground: "auto".into(),
            extrapolated: false,
            alt_invalid: false,
            hvel_invalid: false,
            vvel_invalid: false,
            spare: 0,
            geo_alt_datum: "hae".into(),
            vfom_m: 10,
            vertical_warning: false,
            hat_ft: None,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Waypoint {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_ft: f64,
    pub speed_kt: f64,
}

impl Default for Waypoint {
    fn default() -> Self {
        Self { name: String::new(), lat: 0.0, lon: 0.0, alt_ft: 3000.0, speed_kt: 110.0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Orbit {
    pub lat: f64,
    pub lon: f64,
    pub radius_nm: f64,
    pub clockwise: bool,
}

impl Default for Orbit {
    fn default() -> Self {
        Self { lat: 44.9095, lon: -123.0026, radius_nm: 3.0, clockwise: true }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct FlightConfig {
    /// "manual" | "route" | "orbit" | "parked"
    pub mode: String,
    pub target_track: f64,
    pub target_alt_ft: f64,
    /// True airspeed target.
    pub target_speed_kt: f64,
    pub climb_fpm: f64,
    pub descent_fpm: f64,
    pub turn_rate_dps: f64,
    pub max_bank_deg: f64,
    pub accel_kt_s: f64,
    pub rotate_kt: f64,
    pub route: Vec<Waypoint>,
    pub route_loop: bool,
    /// Index to restart from when looping.
    pub route_loop_to: usize,
    pub orbit: Orbit,
    /// Wind FROM direction (true) and speed.
    pub wind_dir: f64,
    pub wind_kt: f64,
    pub altimeter_inhg: f64,
    pub ground_elev_ft: f64,
    /// Geoid separation (HAE - MSL) in feet; about -60 to -120 ft across CONUS.
    pub geoid_sep_ft: f64,
    /// Magnetic variation, east positive.
    pub mag_var_deg: f64,
    /// Physics time multiplier.
    pub time_scale: f64,
}

impl Default for FlightConfig {
    fn default() -> Self {
        Self {
            mode: "manual".into(),
            target_track: 0.0,
            target_alt_ft: 3500.0,
            target_speed_kt: 110.0,
            climb_fpm: 700.0,
            descent_fpm: 700.0,
            turn_rate_dps: 3.0,
            max_bank_deg: 30.0,
            accel_kt_s: 3.0,
            rotate_kt: 55.0,
            route: Vec::new(),
            route_loop: true,
            route_loop_to: 0,
            orbit: Orbit::default(),
            wind_dir: 270.0,
            wind_kt: 10.0,
            altimeter_inhg: 29.92,
            ground_elev_ft: 214.0,
            geoid_sep_ft: -75.0,
            mag_var_deg: 15.0,
            time_scale: 1.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct AhrsConfig {
    pub valid: bool,
    pub heading_magnetic: bool,
    pub roll_offset: f64,
    pub pitch_offset: f64,
    pub heading_offset: f64,
    /// -1 mirrors roll (receiver mounted backwards).
    pub roll_sign: f64,
    pub send_airspeed: bool,
}

impl Default for AhrsConfig {
    fn default() -> Self {
        Self {
            valid: true,
            heading_magnetic: true,
            roll_offset: 0.0,
            pitch_offset: 0.0,
            heading_offset: 0.0,
            roll_sign: 1.0,
            send_airspeed: true,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct ForeFlightIdConfig {
    /// Hex serial; empty = 0xFFFFFFFFFFFFFFFF (invalid).
    pub serial_hex: String,
    pub name: String,
    pub long_name: String,
    /// Capability bit 0: geometric altitude datum is MSL (else WGS-84 ellipsoid).
    pub geo_alt_msl: bool,
    /// Capability bits 1-2: 0 unrestricted, 1 expensive, 2 disallowed.
    pub internet_policy: u8,
}

impl Default for ForeFlightIdConfig {
    fn default() -> Self {
        Self {
            serial_hex: "0000000000ABCDEF".into(),
            name: "GDL90SIM".into(),
            long_name: "GDL90 Emulator".into(),
            geo_alt_msl: false,
            internet_policy: 0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct VendorConfig {
    /// Stratux 'SX' version bytes (major, minor, patch, type).
    pub stratux_version: [u8; 4],
    pub stratux_protocol: u8,
    pub sats_locked: u8,
    pub sats_tracked: u8,
    /// WAAS/SBAS fix (Stratux SX gps byte 2, SkyRadar '2', Clarity WAAS bit).
    pub waas: bool,
    pub cpu_temp_c: f64,
    pub skyradar_fw: [u8; 2],
    pub pathfinder_fw: u16,
    pub pathfinder_battery: u8,
    /// 0 not charging, 2/3 charging.
    pub pathfinder_charge: u8,
    /// Name the X-Plane / ForeFlight sim protocol announces ("XGPS<name>,...").
    pub xplane_sim_name: String,
}

impl Default for VendorConfig {
    fn default() -> Self {
        Self {
            stratux_version: [1, 6, 1, 0],
            stratux_protocol: 1,
            sats_locked: 11,
            sats_tracked: 14,
            waas: true,
            cpu_temp_c: 48.5,
            skyradar_fw: [3, 1],
            pathfinder_fw: 0x0203,
            pathfinder_battery: 87,
            pathfinder_charge: 0,
            xplane_sim_name: "GDL90EMU".into(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct WeatherConfig {
    /// One DLAC text record per line: "METAR KSLE {TIME} ...". Tokens: {TIME} = DDHHMMZ now,
    /// {VALID} = TAF validity DDHH/DDHH, {FM} = DDHHMM six hours ahead.
    pub text: Vec<String>,
    pub cells: Vec<Cell>,
    /// 63 = regional NEXRAD, 64 = CONUS.
    pub nexrad_product: u16,
    /// Put the ground station `station_offset_nm` from the ownship start point.
    pub station_auto: bool,
    pub station_lat: f64,
    pub station_lon: f64,
    pub station_offset_nm: f64,
    pub tisb_site_id: u8,
    /// Rebuild payloads (fresh times) every N seconds.
    pub rebuild_s: f64,
}

impl Default for WeatherConfig {
    fn default() -> Self {
        Self {
            text: vec![
                "METAR KSLE {TIME} 31008KT 10SM FEW035 BKN250 14/06 A3008 RMK AO2=".into(),
                "METAR KPDX {TIME} 33010KT 10SM SCT040 15/07 A3007 RMK AO2=".into(),
                "METAR KEUG {TIME} 35006KT 7SM -RA BKN020 OVC045 12/09 A3010 RMK AO2=".into(),
                "TAF KSLE {TIME} {VALID} 32008KT P6SM SCT040 BKN250 FM{FM} 34012KT P6SM FEW050=".into(),
            ],
            cells: Vec::new(),
            nexrad_product: 63,
            station_auto: true,
            station_lat: 45.0,
            station_lon: -122.8,
            station_offset_nm: 18.0,
            tisb_site_id: 1,
            rebuild_s: 60.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct OutputConfig {
    /// GDL 90 UDP ports (iFly: 4000 default; 43211 SkyRadar/iLevil/Pathfinder; 41502 Stratus 1).
    pub ports: Vec<u16>,
    /// "auto": unicast to devices seen on the hotspot (hotspot client list, ARP, iFly's
    /// 41500 probe, ForeFlight discovery), broadcast on the hotspot subnet while none are known.
    /// "broadcast" | "unicast" | "both" force one behaviour.
    pub delivery: String,
    /// "auto" = hotspot subnet broadcast only (never the office LAN); or an explicit IP.
    pub broadcast_addr: String,
    /// Extra targets: "ip" or "ip:port".
    pub targets: Vec<String>,
    /// Also send to 127.0.0.1 (iFly for Windows on this PC).
    pub localhost: bool,
    /// One datagram per message (true) or one per scheduler burst (false).
    pub per_message_datagrams: bool,
    pub xplane_port: u16,
    pub nmea_port: u16,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            ports: vec![4000],
            delivery: "auto".into(),
            broadcast_addr: "auto".into(),
            targets: Vec::new(),
            localhost: false,
            per_message_datagrams: true,
            xplane_port: 49002,
            nmea_port: 4000,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct NetworkConfig {
    /// Start the hotspot on the preferred adapter at launch.
    pub auto_hotspot: bool,
    pub ssid: String,
    /// WPA2 passphrase (Windows Mobile Hotspot requires 8+ characters).
    pub passphrase: String,
    /// Prefer adapters whose description contains this (case-insensitive).
    pub adapter_match: String,
    /// Force a specific adapter by interface GUID; empty = auto.
    pub adapter_guid: String,
    /// Hotspot band: "auto" (lets a radio that is also this PC's uplink share its current
    /// channel), "2.4" or "5". Forcing 2.4 on a shared radio moves the PC's own link to 2.4 GHz.
    pub band: String,
    /// ESP32 access point channel (1-13).
    pub channel: u8,
    /// ESP32: answer every DNS name with the emulator's address so phones/PCs that join pop the
    /// control panel up as a "sign in to network" page.
    pub captive_portal: bool,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            auto_hotspot: true,
            ssid: "GDL90-SIM".into(),
            passphrase: "gdl90sim".into(),
            adapter_match: "802.11n".into(),
            adapter_guid: String::new(),
            band: "auto".into(),
            channel: 6,
            captive_portal: true,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Config {
    pub profile: String,
    pub ownship: OwnshipConfig,
    pub flight: FlightConfig,
    pub streams: Streams,
    pub heartbeat: HeartbeatFields,
    pub ahrs: AhrsConfig,
    pub foreflight: ForeFlightIdConfig,
    pub vendor: VendorConfig,
    pub weather: WeatherConfig,
    pub output: OutputConfig,
    pub network: NetworkConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            profile: "generic".into(),
            ownship: Default::default(),
            flight: Default::default(),
            streams: Default::default(),
            heartbeat: Default::default(),
            ahrs: Default::default(),
            foreflight: Default::default(),
            vendor: Default::default(),
            weather: Default::default(),
            output: Default::default(),
            network: Default::default(),
        }
    }
}

/// RFC 7386 JSON merge patch (arrays are replaced wholesale).
pub fn merge_patch(target: &mut Value, patch: &Value) {
    match patch {
        Value::Object(p) => {
            if !target.is_object() {
                *target = Value::Object(Default::default());
            }
            let t = target.as_object_mut().unwrap();
            for (k, v) in p {
                if v.is_null() {
                    t.remove(k);
                } else {
                    merge_patch(t.entry(k.clone()).or_insert(Value::Null), v);
                }
            }
        }
        other => *target = other.clone(),
    }
}

impl Config {
    pub fn validate(&self) -> anyhow::Result<()> {
        use anyhow::ensure;
        ensure!(!self.network.ssid.is_empty() && self.network.ssid.len() <= 32, "SSID must be 1–32 bytes");
        let pass = &self.network.passphrase;
        ensure!(pass.is_empty() || (8..=63).contains(&pass.len()), "Use an empty Wi-Fi password or 8–63 bytes");
        ensure!((1..=13).contains(&self.network.channel), "Wi-Fi channel must be 1–13");
        ensure!(!self.output.ports.is_empty() && self.output.ports.len() <= 8 && !self.output.ports.contains(&0), "Choose 1–8 output ports from 1–65535");
        ensure!(self.weather.cells.len() <= 12, "Use up to 12 weather cells");
        for cell in &self.weather.cells {
            ensure!((-90.0..=90.0).contains(&cell.lat) && (-180.0..=180.0).contains(&cell.lon) &&
                (0.1..=30.0).contains(&cell.radius_nm) && cell.intensity <= 7,
                "Use valid coordinates, 0.1–30 NM radius and 0–7 intensity for each weather cell");
        }
        ensure!([63, 64].contains(&self.weather.nexrad_product), "Choose NEXRAD product 63 or 64");
        ensure!(self.weather.text.len() <= 32 && self.weather.text.iter().all(|t| t.len() <= 500), "Use up to 32 weather reports of 500 bytes each");
        ensure!(self.flight.route.len() <= 64, "Use up to 64 waypoints");
        ensure!(self.flight.climb_fpm >= 0.0 && self.flight.descent_fpm >= 0.0 && self.flight.turn_rate_dps >= 0.0 &&
            self.flight.max_bank_deg >= 0.0 && self.flight.accel_kt_s >= 0.0, "Flight rates and bank limit must be positive or zero");
        #[cfg(target_os = "espidf")]
        {
            use std::net::{Ipv4Addr, SocketAddrV4};
            ensure!(self.output.targets.len() <= 8, "Use up to 8 device addresses");
            for target in &self.output.targets {
                let ip = target.trim().parse::<Ipv4Addr>().ok().or_else(|| target.trim().parse::<SocketAddrV4>().ok().filter(|a| a.port() > 0).map(|a| *a.ip()));
                ensure!(ip.is_some_and(|ip| matches!(ip.octets(), [192, 168, 4, 2..=254])), "Targets must be devices on the simulator Wi-Fi (192.168.4.2–254)");
            }
            ensure!(matches!(self.output.broadcast_addr.trim(), "" | "auto" | "192.168.4.255"), "The simulator Wi-Fi broadcast address is 192.168.4.255");
        }
        Ok(())
    }

    pub fn patched(&self, patch: &Value) -> anyhow::Result<Config> {
        let mut v = serde_json::to_value(self)?;
        merge_patch(&mut v, patch);
        let config: Config = serde_json::from_value(v)?;
        config.validate()?;
        Ok(config)
    }
}

// ---------------------------------------------------------------------------------------------
// Device profiles ("prebuilt protocols"), matched to how iFly identifies each receiver
// ---------------------------------------------------------------------------------------------

pub struct Profile {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
}

pub const PROFILES: &[Profile] = &[
    Profile { id: "generic", name: "Generic GDL 90 (ICD)", description: "Heartbeat, Ownship, Geo Alt, Traffic and FIS-B uplinks per the Garmin ICD on UDP 4000. No vendor extensions." },
    Profile { id: "stratux", name: "Stratux", description: "ICD set plus 0xCC heartbeat, 'SX' status (iFly identifies Stratux by it), 0x4C 'LE' AHRS at 5 Hz and the ForeFlight ID message. UDP 4000." },
    Profile { id: "foreflight", name: "ForeFlight-extension box (Stratus-class)", description: "ICD set plus ForeFlight 0x65 ID and 12-byte 0x65 AHRS at 5 Hz. UDP 4000." },
    Profile { id: "ilevil", name: "iLevil", description: "ICD set plus 0x4C 'LE' AHRS (24 bytes) at 10 Hz. iFly tags the device iLevil after 5 s. UDP 43211." },
    Profile { id: "skyradar", name: "SkyRadar", description: "ICD set plus the 0x65 SkyRadar status (firmware + GPS fix character). UDP 43211." },
    Profile { id: "clarity", name: "Clarity", description: "ICD set plus 0x3C status (identifies Clarity) and 0x3D float AHRS. UDP 4000." },
    Profile { id: "pathfinder", name: "Pathfinder", description: "ICD set plus 0x46 status (sub 1, identifies Pathfinder) and 0x46 AHRS (sub 2, little-endian). UDP 43211." },
    Profile { id: "xgps190", name: "Dual XGPS190", description: "ICD set plus 0x44 AHRS. UDP 4000." },
    Profile { id: "xplane", name: "X-Plane / ForeFlight sim text", description: "XGPS / XATT / XTRAFFIC text datagrams on UDP 49002 (iFly auto-detect listens there). No GDL 90." },
    Profile { id: "nmea", name: "NMEA 0183 over UDP", description: "$GPRMC + $GPGGA at 1 Hz on UDP 4000. iFly falls back to NMEA after 2000 bytes without a valid GDL 90 frame." },
];

pub fn profile_streams(id: &str) -> Streams {
    let base = Streams {
        heartbeat: on(1.0),
        ownship: on(1.0),
        geo_alt: on(1.0),
        traffic: on(1.0),
        uplink: on(2.0),
        hat: off(1.0),
        ff_id: off(1.0),
        ff_ahrs: off(5.0),
        levil_ahrs: off(5.0),
        stratux_heartbeat: off(1.0),
        stratux_status: off(1.0),
        skyradar_status: off(1.0),
        clarity_status: off(1.0),
        clarity_ahrs: off(5.0),
        xgps_ahrs: off(5.0),
        pathfinder_status: off(1.0),
        pathfinder_ahrs: off(5.0),
        xplane_gps: off(1.0),
        xplane_att: off(5.0),
        xplane_traffic: off(1.0),
        nmea: off(1.0),
    };
    let none = Streams {
        heartbeat: off(1.0),
        ownship: off(1.0),
        geo_alt: off(1.0),
        traffic: off(1.0),
        uplink: off(2.0),
        ..base.clone()
    };
    match id {
        "stratux" => Streams {
            stratux_heartbeat: on(1.0),
            stratux_status: on(1.0),
            levil_ahrs: on(5.0),
            ff_id: on(1.0),
            ..base
        },
        "foreflight" => Streams { ff_id: on(1.0), ff_ahrs: on(5.0), ..base },
        "ilevil" => Streams { levil_ahrs: on(10.0), ..base },
        "skyradar" => Streams { skyradar_status: on(1.0), ..base },
        "clarity" => Streams { clarity_status: on(1.0), clarity_ahrs: on(5.0), ..base },
        "pathfinder" => Streams { pathfinder_status: on(1.0), pathfinder_ahrs: on(5.0), ..base },
        "xgps190" => Streams { xgps_ahrs: on(5.0), ..base },
        "xplane" => Streams { xplane_gps: on(1.0), xplane_att: on(5.0), xplane_traffic: on(1.0), ..none },
        "nmea" => Streams { nmea: on(1.0), ..none },
        _ => base,
    }
}

/// Merge patch that switches the whole config to a profile (streams, ports, identity).
pub fn profile_patch(id: &str) -> Option<Value> {
    PROFILES.iter().find(|p| p.id == id)?;
    let streams = serde_json::to_value(profile_streams(id)).ok()?;
    let ports: Vec<u16> = match id {
        "ilevil" | "skyradar" | "pathfinder" => vec![43211],
        _ => vec![4000],
    };
    let (name, long_name) = match id {
        "stratux" => ("Stratux", "Stratux"),
        "foreflight" => ("Stratus", "Stratus-class GDL90"),
        _ => ("GDL90SIM", "GDL90 Emulator"),
    };
    Some(json!({
        "profile": id,
        "streams": streams,
        "output": { "ports": ports },
        "ahrs": { "valid": true },
        "foreflight": { "name": name, "long_name": long_name },
    }))
}
