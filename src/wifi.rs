//! Wi-Fi adapter discovery and the Windows Mobile Hotspot (WinRT tethering) on a chosen adapter.
//!
//! Detection uses PowerShell (Get-NetAdapter / Get-PnpDevice) because it also sees adapters
//! that are disabled or unplugged; the hotspot uses NetworkOperatorTetheringManager with the
//! USB adapter as the private (access point) side, which needs no admin rights.

use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct AdapterInfo {
    pub name: String,
    pub description: String,
    pub guid: String,
    pub if_index: i64,
    pub status: String,
    pub mac: String,
    pub pnp_id: String,
    pub wireless: bool,
    pub usb: bool,
    pub present: bool,
    /// Config Manager problem code (22 = disabled in Device Manager).
    pub problem: Option<i64>,
    pub ipv4: Vec<String>,
    /// SSID this adapter is connected to as a client (if any).
    pub connected_ssid: Option<String>,
    /// `netsh wlan show drivers` "Hosted network supported" (legacy SoftAP).
    pub hosted_supported: Option<bool>,
    /// Whether Windows Mobile Hotspot can run on this radio. Windows 11 needs a WDI driver;
    /// legacy (non-WDI) drivers claim Wi-Fi Direct GO but get no virtual port, and Windows then
    /// silently hosts on another radio (RTL8188CU / rtwlanu_oldic, 2026-11-09). Hosted network
    /// fails on them too ("not in the correct state").
    pub mobile_hotspot_ok: Option<bool>,
    pub score: i32,
    pub note: String,
}

impl AdapterInfo {
    pub fn disabled(&self) -> bool {
        self.problem == Some(22)
    }
    pub fn usable(&self) -> bool {
        self.present && self.problem.unwrap_or(0) == 0
    }
}

#[derive(Clone, Debug, Serialize, Default)]
pub struct HotspotClient {
    pub mac: String,
    pub ips: Vec<String>,
    pub name: String,
}

#[derive(Clone, Debug, Serialize, Default)]
pub struct HotspotStatus {
    /// "off" | "starting" | "on" | "stopping" | "error" | "unsupported"
    pub state: String,
    pub ssid: String,
    pub adapter: String,
    pub clients: Vec<HotspotClient>,
    pub message: String,
}

const SCAN_PS: &str = r#"
$ErrorActionPreference = 'SilentlyContinue'
$na  = @(Get-NetAdapter -IncludeHidden | Select-Object Name, InterfaceDescription, InterfaceGuid, ifIndex, Status, MacAddress, PnPDeviceID, NdisPhysicalMedium, Virtual)
$pnp = @(Get-PnpDevice -Class Net | Select-Object FriendlyName, InstanceId, Status, Present, ConfigManagerErrorCode)
$ip  = @(Get-NetIPAddress -AddressFamily IPv4 | Select-Object ifIndex, IPAddress, PrefixLength)
$wl  = (netsh wlan show interfaces) -join "`n"
$drv = (netsh wlan show drivers) -join "`n"
$cap = (netsh wlan show wirelesscapabilities) -join "`n"
[pscustomobject]@{ adapters = $na; pnp = $pnp; ip = $ip; wlan = $wl; drivers = $drv; caps = $cap } | ConvertTo-Json -Depth 4 -Compress
"#;

/// `netsh wlan show wirelesscapabilities` -> (interface name, has a WDI driver).
fn wdi_support(text: &str) -> Vec<(String, bool)> {
    let mut out = Vec::new();
    let mut name: Option<String> = None;
    for line in text.lines() {
        let t = line.trim();
        if let Some(n) = t.strip_prefix("Interface name:") {
            name = Some(n.trim().to_string());
        } else if let Some((k, v)) = t.split_once(':') {
            if k.trim() == "WDI Version (IHV)" {
                if let Some(n) = name.take() {
                    out.push((n, !v.to_ascii_lowercase().contains("not supported")));
                }
            }
        }
    }
    out
}

/// `netsh wlan show drivers` -> (interface name, hosted network supported).
fn hosted_support(text: &str) -> Vec<(String, bool)> {
    let mut out = Vec::new();
    let mut name: Option<String> = None;
    for line in text.lines() {
        let t = line.trim();
        if let Some(n) = t.strip_prefix("Interface name:") {
            name = Some(n.trim().to_string());
        } else if let Some((k, v)) = t.split_once(':') {
            if k.trim() == "Hosted network supported" {
                if let Some(n) = name.take() {
                    out.push((n, v.trim().eq_ignore_ascii_case("yes")));
                }
            }
        }
    }
    out
}

/// Last five octets of a MAC, normalised. Windows derives a hosted/Wi-Fi Direct virtual
/// adapter's MAC from its physical radio by changing only the first octet, so this tells which
/// radio a virtual AP really lives on.
pub fn mac_tail(mac: &str) -> String {
    let hex: String = mac.chars().filter(|c| c.is_ascii_hexdigit()).collect::<String>().to_ascii_lowercase();
    if hex.len() == 12 {
        hex[2..].to_string()
    } else {
        hex
    }
}

/// The adapter currently holding the Mobile Hotspot (ICS) address 192.168.137.x:
/// (name, description, MAC).
pub fn ics_adapter() -> Option<(String, String, String)> {
    let script = r#"
$ip = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.IPAddress -like '192.168.137.*' } | Select-Object -First 1
if ($ip) { $a = Get-NetAdapter -IncludeHidden -InterfaceIndex $ip.ifIndex; "$($a.Name)|$($a.InterfaceDescription)|$($a.MacAddress)" }
"#;
    let out = run_ps(script).ok()?;
    let line = out.lines().map(str::trim).find(|l| l.contains('|'))?;
    let mut p = line.splitn(3, '|');
    Some((p.next()?.to_string(), p.next()?.to_string(), p.next()?.to_string()))
}

fn run_ps(script: &str) -> anyhow::Result<String> {
    let mut cmd = std::process::Command::new("powershell.exe");
    cmd.args(["-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-Command", script]);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        cmd.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
    }
    let out = cmd.output()?;
    if !out.status.success() && out.stdout.is_empty() {
        anyhow::bail!("powershell failed: {}", String::from_utf8_lossy(&out.stderr));
    }
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}

fn as_vec(v: &Value) -> Vec<Value> {
    match v {
        Value::Array(a) => a.clone(),
        Value::Null => vec![],
        other => vec![other.clone()],
    }
}

fn s(v: &Value, k: &str) -> String {
    match v.get(k) {
        Some(Value::String(s)) => s.clone(),
        Some(Value::Number(n)) => n.to_string(),
        Some(Value::Bool(b)) => b.to_string(),
        _ => String::new(),
    }
}

fn is_wireless_desc(d: &str) -> bool {
    let d = d.to_ascii_lowercase();
    ["wireless", "wi-fi", "wifi", "802.11", "wlan"].iter().any(|k| d.contains(k))
}

/// Parse `netsh wlan show interfaces` into (interface name -> connected SSID).
fn wlan_ssids(text: &str) -> Vec<(String, Option<String>)> {
    let mut out = Vec::new();
    let mut name: Option<String> = None;
    let mut state = String::new();
    let mut ssid: Option<String> = None;
    let flush = |name: &mut Option<String>, state: &str, ssid: &mut Option<String>, out: &mut Vec<(String, Option<String>)>| {
        if let Some(n) = name.take() {
            out.push((n, if state.starts_with("connected") { ssid.take() } else { None }));
        }
        *ssid = None;
    };
    for line in text.lines() {
        let Some((k, v)) = line.split_once(':') else { continue };
        let k = k.trim();
        let v = v.trim().to_string();
        match k {
            "Name" => {
                flush(&mut name, &state, &mut ssid, &mut out);
                name = Some(v);
                state.clear();
            }
            "State" => state = v.to_ascii_lowercase(),
            "SSID" => ssid = Some(v),
            _ => {}
        }
    }
    flush(&mut name, &state, &mut ssid, &mut out);
    out
}

pub fn scan(adapter_match: &str, force_guid: &str) -> anyhow::Result<Vec<AdapterInfo>> {
    let raw = run_ps(SCAN_PS)?;
    let v: Value = serde_json::from_str(raw.trim()).map_err(|e| anyhow::anyhow!("adapter scan JSON: {e}"))?;
    let pnp = as_vec(&v["pnp"]);
    let ips = as_vec(&v["ip"]);
    let ssids = wlan_ssids(v["wlan"].as_str().unwrap_or(""));
    let hosted = hosted_support(v["drivers"].as_str().unwrap_or(""));
    let wdi = wdi_support(v["caps"].as_str().unwrap_or(""));
    let mut out: Vec<AdapterInfo> = Vec::new();

    for a in as_vec(&v["adapters"]) {
        let desc = s(&a, "InterfaceDescription");
        let medium = a.get("NdisPhysicalMedium").and_then(|x| x.as_i64()).unwrap_or(-1);
        let virt = a.get("Virtual").and_then(|x| x.as_bool()).unwrap_or(false);
        let pnp_id = s(&a, "PnPDeviceID");
        let wireless = medium == 9 || is_wireless_desc(&desc);
        if !wireless || virt || desc.to_ascii_lowercase().contains("wi-fi direct") {
            continue;
        }
        let if_index = a.get("ifIndex").and_then(|x| x.as_i64()).unwrap_or(-1);
        let p = pnp.iter().find(|p| s(p, "InstanceId").eq_ignore_ascii_case(&pnp_id));
        let problem = p.and_then(|p| p.get("ConfigManagerErrorCode").and_then(|x| x.as_i64())).filter(|c| *c != 0);
        // A disabled adapter shows "Not Present" in Get-NetAdapter; PnP's Present flag is the truth.
        let present = match p.and_then(|p| p.get("Present").and_then(|x| x.as_bool())) {
            Some(pr) => pr,
            None => s(&a, "Status") != "Not Present",
        };
        let name = s(&a, "Name");
        out.push(AdapterInfo {
            name: name.clone(),
            description: desc,
            guid: s(&a, "InterfaceGuid"),
            if_index,
            status: s(&a, "Status"),
            mac: s(&a, "MacAddress"),
            usb: pnp_id.to_ascii_uppercase().starts_with("USB\\"),
            pnp_id,
            wireless: true,
            present,
            problem,
            ipv4: ips
                .iter()
                .filter(|i| i.get("ifIndex").and_then(|x| x.as_i64()) == Some(if_index))
                .map(|i| format!("{}/{}", s(i, "IPAddress"), s(i, "PrefixLength")))
                .collect(),
            connected_ssid: ssids.iter().find(|(n, _)| *n == name).and_then(|(_, s)| s.clone()),
            hosted_supported: hosted.iter().find(|(n, _)| *n == name).map(|(_, h)| *h),
            mobile_hotspot_ok: wdi.iter().find(|(n, _)| *n == name).map(|(_, w)| *w),
            score: 0,
            note: String::new(),
        });
    }
    // Disabled/hidden wireless devices that Get-NetAdapter didn't list.
    for p in &pnp {
        let id = s(p, "InstanceId");
        let fname = s(p, "FriendlyName");
        if !is_wireless_desc(&fname) || fname.to_ascii_lowercase().contains("direct") {
            continue;
        }
        if out.iter().any(|a| a.pnp_id.eq_ignore_ascii_case(&id)) {
            continue;
        }
        let present = p.get("Present").and_then(|x| x.as_bool()).unwrap_or(false);
        if !present {
            continue; // stale entry for a device that isn't plugged in
        }
        out.push(AdapterInfo {
            name: fname.clone(),
            description: fname,
            usb: id.to_ascii_uppercase().starts_with("USB\\"),
            pnp_id: id,
            wireless: true,
            present,
            problem: p.get("ConfigManagerErrorCode").and_then(|x| x.as_i64()).filter(|c| *c != 0),
            status: s(p, "Status"),
            ..Default::default()
        });
    }
    // De-duplicate on PnP id: keep the present one.
    out.sort_by_key(|a| !a.present);
    let mut seen = std::collections::HashSet::new();
    out.retain(|a| seen.insert(a.pnp_id.to_ascii_uppercase()));

    let m = adapter_match.to_ascii_lowercase();
    for a in &mut out {
        let mut score = 0;
        let mut notes = Vec::new();
        if !m.is_empty() && a.description.to_ascii_lowercase().contains(&m) {
            score += 50;
            notes.push(format!("matches '{adapter_match}'"));
        }
        if a.usb {
            score += 30;
            notes.push("USB".into());
        }
        if a.connected_ssid.is_some() {
            score -= 60;
            notes.push(format!("carrying this PC's connection to '{}'", a.connected_ssid.clone().unwrap_or_default()));
        }
        if !a.present {
            score -= 100;
            notes.push("not plugged in".into());
        } else if a.disabled() {
            notes.push("disabled in Device Manager".into());
        } else if let Some(p) = a.problem {
            notes.push(format!("device problem code {p}"));
            score -= 20;
        }
        if a.mobile_hotspot_ok == Some(false) {
            score -= 40;
            notes.push("legacy (non-WDI) driver: Windows 11 can't host a hotspot on it".into());
        }
        if !force_guid.is_empty() && norm_guid(&a.guid) == norm_guid(force_guid) {
            score += 1000;
            notes.push("selected".into());
        }
        a.score = score;
        a.note = notes.join(", ");
    }
    out.sort_by(|a, b| b.score.cmp(&a.score));
    Ok(out)
}

pub fn norm_guid(g: &str) -> String {
    g.trim().trim_matches(|c| c == '{' || c == '}').to_ascii_lowercase()
}

/// Adapter to host the access point. An explicitly selected GUID always wins; otherwise the
/// best-scoring plugged-in adapter that is NOT carrying this PC's own Wi-Fi connection (never
/// auto-pick the uplink: a hotspot on it could drop the PC off its network).
pub fn pick<'a>(adapters: &'a [AdapterInfo], force_guid: &str) -> Option<&'a AdapterInfo> {
    if !force_guid.trim().is_empty() {
        return adapters.iter().find(|a| norm_guid(&a.guid) == norm_guid(force_guid));
    }
    let spare = |a: &&AdapterInfo| a.present && a.connected_ssid.is_none();
    // Prefer a spare that can actually host; otherwise still return the best spare so the UI
    // can say why it can't be used.
    adapters
        .iter()
        .filter(spare)
        .find(|a| a.mobile_hotspot_ok != Some(false))
        .or_else(|| adapters.iter().find(spare))
}

/// Enable a disabled device. Raises a UAC prompt (pnputil needs admin).
pub fn enable_device(pnp_id: &str) -> anyhow::Result<()> {
    if pnp_id.contains('\'') || pnp_id.contains('"') {
        anyhow::bail!("bad device id");
    }
    let script = format!(
        "$p = Start-Process -FilePath pnputil.exe -ArgumentList '/enable-device \"{pnp_id}\"' -Verb RunAs -Wait -PassThru -WindowStyle Hidden; exit $p.ExitCode"
    );
    let out = run_ps(&script);
    match out {
        Ok(_) => Ok(()),
        Err(e) => Err(e),
    }
}

// ---------------------------------------------------------------------------------------------
// Mobile Hotspot (WinRT)
// ---------------------------------------------------------------------------------------------

#[cfg(windows)]
pub mod hotspot {
    use super::{norm_guid, HotspotClient, HotspotStatus};
    use anyhow::{anyhow, bail, Result};
    use windows::core::HSTRING;
    use windows::Devices::WiFi::WiFiAdapter;
    use windows::Networking::Connectivity::{ConnectionProfile, NetworkAdapter, NetworkInformation};
    use windows::Networking::NetworkOperators::{
        NetworkOperatorTetheringManager, TetheringCapability, TetheringOperationStatus, TetheringOperationalState,
        TetheringWiFiBand,
    };
    use windows::Win32::System::Com::{CoInitializeEx, COINIT_MULTITHREADED};

    fn com() {
        unsafe {
            let _ = CoInitializeEx(None, COINIT_MULTITHREADED);
        }
    }

    fn op_status(s: TetheringOperationStatus) -> &'static str {
        match s.0 {
            0 => "Success",
            1 => "Unknown",
            2 => "MobileBroadbandDeviceOff",
            3 => "WiFiDeviceOff",
            4 => "EntitlementCheckTimeout",
            5 => "EntitlementCheckFailure",
            6 => "OperationInProgress",
            7 => "BluetoothDeviceOff",
            8 => "NetworkLimitedConnectivity",
            _ => "?",
        }
    }

    fn capability(c: TetheringCapability) -> &'static str {
        match c.0 {
            0 => "Enabled",
            1 => "DisabledByGroupPolicy",
            2 => "DisabledByHardwareLimitation",
            3 => "DisabledByOperator",
            4 => "DisabledBySku",
            5 => "DisabledByRequiredAppNotInstalled",
            6 => "DisabledDueToUnknownCause",
            7 => "DisabledBySystemCapability",
            _ => "?",
        }
    }

    /// The connection being shared (public side). Prefer the internet profile.
    fn public_profile() -> Result<ConnectionProfile> {
        if let Ok(p) = NetworkInformation::GetInternetConnectionProfile() {
            return Ok(p);
        }
        for p in NetworkInformation::GetConnectionProfiles()? {
            if p.NetworkAdapter().is_ok() {
                return Ok(p);
            }
        }
        bail!("no network connection to share; Mobile Hotspot needs one (Ethernet or another Wi-Fi)")
    }

    fn wifi_adapter(guid: &str) -> Result<NetworkAdapter> {
        let want = norm_guid(guid);
        let mut seen = Vec::new();
        for a in WiFiAdapter::FindAllAdaptersAsync()?.get()? {
            let na = a.NetworkAdapter()?;
            let id = norm_guid(&format!("{:?}", na.NetworkAdapterId()?));
            if id == want {
                return Ok(na);
            }
            seen.push(id);
        }
        bail!("WinRT does not list Wi-Fi adapter {guid} (saw {})", seen.join(", "))
    }

    fn manager(guid: Option<&str>) -> Result<NetworkOperatorTetheringManager> {
        com();
        let profile = public_profile()?;
        let cap = NetworkOperatorTetheringManager::GetTetheringCapabilityFromConnectionProfile(&profile)?;
        if cap != TetheringCapability::Enabled {
            bail!("tethering not available: {}", capability(cap));
        }
        Ok(match guid {
            Some(g) => NetworkOperatorTetheringManager::CreateFromConnectionProfileWithTargetAdapter(&profile, &wifi_adapter(g)?)?,
            None => NetworkOperatorTetheringManager::CreateFromConnectionProfile(&profile)?,
        })
    }

    /// Write SSID / passphrase / band into Windows' Mobile Hotspot settings without starting it.
    pub fn configure(guid: Option<&str>, ssid: &str, passphrase: &str, band: &str) -> Result<NetworkOperatorTetheringManager> {
        if passphrase.len() < 8 {
            bail!("hotspot passphrase must be at least 8 characters");
        }
        let mgr = manager(guid)?;
        let cfg = mgr.GetCurrentAccessPointConfiguration()?;
        cfg.SetSsid(&HSTRING::from(ssid))?;
        cfg.SetPassphrase(&HSTRING::from(passphrase))?;
        let b = match band {
            "2.4" => TetheringWiFiBand::TwoPointFourGigahertz,
            "5" => TetheringWiFiBand::FiveGigahertz,
            _ => TetheringWiFiBand::Auto,
        };
        if b == TetheringWiFiBand::Auto || cfg.IsBandSupported(b).unwrap_or(false) {
            let _ = cfg.SetBand(b);
        }
        mgr.ConfigureAccessPointAsync(&cfg)?.get()?;
        Ok(mgr)
    }

    pub fn start(guid: &str, ssid: &str, passphrase: &str, band: &str) -> Result<String> {
        let mgr = configure(Some(guid), ssid, passphrase, band)?;
        if mgr.TetheringOperationalState()? == TetheringOperationalState::On {
            return Ok("hotspot already on".into());
        }
        let r = mgr.StartTetheringAsync()?.get()?;
        let st = r.Status()?;
        if st != TetheringOperationStatus::Success {
            let extra = r.AdditionalErrorMessage().map(|h| h.to_string()).unwrap_or_default();
            return Err(anyhow!("StartTethering: {} {}", op_status(st), extra));
        }
        Ok(format!("hotspot '{ssid}' started"))
    }

    pub fn stop(guid: Option<&str>) -> Result<String> {
        let mgr = manager(guid)?;
        if mgr.TetheringOperationalState()? == TetheringOperationalState::Off {
            return Ok("hotspot already off".into());
        }
        let r = mgr.StopTetheringAsync()?.get()?;
        let st = r.Status()?;
        if st != TetheringOperationStatus::Success {
            bail!("StopTethering: {}", op_status(st));
        }
        Ok("hotspot stopped".into())
    }

    pub fn status(guid: Option<&str>) -> Result<HotspotStatus> {
        let mgr = manager(guid)?;
        let state = match mgr.TetheringOperationalState()? {
            TetheringOperationalState::On => "on",
            TetheringOperationalState::Off => "off",
            TetheringOperationalState::InTransition => "starting",
            _ => "unknown",
        };
        let ssid = mgr.GetCurrentAccessPointConfiguration().and_then(|c| c.Ssid()).map(|s| s.to_string()).unwrap_or_default();
        let mut clients = Vec::new();
        if state == "on" {
            if let Ok(list) = mgr.GetTetheringClients() {
                for c in list {
                    let mac = c.MacAddress().map(|s| s.to_string()).unwrap_or_default();
                    let mut ips = Vec::new();
                    let mut name = String::new();
                    if let Ok(hosts) = c.HostNames() {
                        for h in hosts {
                            let n = h.CanonicalName().map(|s| s.to_string()).unwrap_or_default();
                            if n.parse::<std::net::Ipv4Addr>().is_ok() {
                                ips.push(n);
                            } else if !n.contains(':') && name.is_empty() {
                                name = n;
                            }
                        }
                    }
                    clients.push(HotspotClient { mac, ips, name });
                }
            }
        }
        Ok(HotspotStatus { state: state.into(), ssid, adapter: String::new(), clients, message: String::new() })
    }
}

#[cfg(not(windows))]
pub mod hotspot {
    use super::HotspotStatus;
    pub fn configure(_: Option<&str>, _: &str, _: &str, _: &str) -> anyhow::Result<()> {
        anyhow::bail!("Desktop hotspot management requires Windows")
    }
    pub fn start(_: &str, _: &str, _: &str, _: &str) -> anyhow::Result<String> {
        anyhow::bail!("hotspot control is Windows-only; bring up an AP yourself (e.g. hostapd)")
    }
    pub fn stop(_: Option<&str>) -> anyhow::Result<String> {
        anyhow::bail!("Windows-only")
    }
    pub fn status(_: Option<&str>) -> anyhow::Result<HotspotStatus> {
        Ok(HotspotStatus { state: "unsupported".into(), ..Default::default() })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drivers_parse_and_mac_tail() {
        let t = "Interface name: Wi-Fi \n    Driver  : X\n    Hosted network supported  : No\n\nInterface name: Wi-Fi 2 \n    Hosted network supported  : Yes\n";
        assert_eq!(hosted_support(t), vec![("Wi-Fi".into(), false), ("Wi-Fi 2".into(), true)]);
        // Wi-Fi Direct virtual adapter of the built-in card seen on devdesktop2, 2026-11-09
        assert_eq!(mac_tail("FA-6F-38-B0-69-A5"), mac_tail("e8:6f:38:b0:69:a5"));
        assert_ne!(mac_tail("FA-6F-38-B0-69-A5"), mac_tail("00:e0:4d:0d:00:4c"));
        let caps = "Interface name: Wi-Fi \n    WDI Version (IHV)    : 0.1.1.9\n    Wi-Fi Direct GO : Supported\nInterface name: Wi-Fi 2 \n    WDI Version (IHV)    : Wdi Interface Not Supported\n";
        assert_eq!(wdi_support(caps), vec![("Wi-Fi".into(), true), ("Wi-Fi 2".into(), false)]);
    }

    #[test]
    fn netsh_parse() {
        let t = "    Name                   : Wi-Fi\n    State                  : connected\n    SSID                   : Home-5G\n\n    Name                   : Wi-Fi 2\n    State                  : disconnected\n";
        let v = wlan_ssids(t);
        assert_eq!(v, vec![("Wi-Fi".into(), Some("Home-5G".into())), ("Wi-Fi 2".into(), None)]);
    }
}
