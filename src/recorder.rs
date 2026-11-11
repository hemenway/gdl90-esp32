//! iFly ADS-B stream recording format (`Adsb_YYYY_MM_DD-HH.MM.bin`, playback `Adsb.play.bin`):
//! little-endian; header int64 .NET UTC ticks + int32 Wifi.Brand; records int32 ms-since-first
//! + int32 count + raw bytes (framed + escaped, as read); comment = int32 -1 + .NET string;
//! end = int32 0x7FFFFFFF. Source: iFly.Core Adsb/AdsbStreamRecorder.cs, AdsbStreamPlayback.cs.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;

use anyhow::{bail, Context, Result};

/// iFly `Settings.Wifi.Brand` values.
pub fn brand_for_profile(profile: &str) -> i32 {
    match profile {
        "skyradar" => 1,
        "clarity" => 2,
        "ilevil" => 5,
        "xgps190" => 8,
        "pathfinder" => 10,
        "lynx" => 13,
        "stratux" => 14,
        "xplane" => 17,
        "sentry" => 19,
        _ => 99,
    }
}

pub fn dotnet_ticks(unix: f64) -> i64 {
    ((unix + 62_135_596_800.0) * 10_000_000.0) as i64
}

pub struct Recorder {
    w: BufWriter<File>,
    first: Option<f64>,
    pub path: String,
    pub records: u64,
    pub bytes: u64,
}

impl Recorder {
    pub fn create(path: &Path, brand: i32, unix_now: f64) -> Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).ok();
        }
        let f = File::create(path).with_context(|| format!("create {}", path.display()))?;
        let mut w = BufWriter::new(f);
        w.write_all(&dotnet_ticks(unix_now).to_le_bytes())?;
        w.write_all(&brand.to_le_bytes())?;
        Ok(Self { w, first: None, path: path.display().to_string(), records: 0, bytes: 0 })
    }

    pub fn write(&mut self, unix: f64, data: &[u8]) -> Result<()> {
        let first = *self.first.get_or_insert(unix);
        let ms = ((unix - first) * 1000.0).max(0.0) as i32;
        self.w.write_all(&ms.to_le_bytes())?;
        self.w.write_all(&(data.len() as i32).to_le_bytes())?;
        self.w.write_all(data)?;
        self.records += 1;
        self.bytes += data.len() as u64;
        Ok(())
    }

    pub fn finish(mut self) -> Result<String> {
        self.w.write_all(&i32::MAX.to_le_bytes())?;
        self.w.flush()?;
        Ok(self.path)
    }
}

pub struct Recording {
    pub start_ticks: i64,
    pub brand: i32,
    /// (ms since first record, raw bytes)
    pub records: Vec<(u32, Vec<u8>)>,
    pub comments: Vec<String>,
}

fn read_dotnet_string(buf: &[u8], pos: &mut usize) -> Option<String> {
    let mut len: usize = 0;
    let mut shift = 0;
    loop {
        let b = *buf.get(*pos)?;
        *pos += 1;
        len |= ((b & 0x7F) as usize) << shift;
        if b & 0x80 == 0 {
            break;
        }
        shift += 7;
        if shift > 28 {
            return None;
        }
    }
    let s = buf.get(*pos..*pos + len)?;
    *pos += len;
    Some(String::from_utf8_lossy(s).into_owned())
}

pub fn parse(buf: &[u8]) -> Result<Recording> {
    if buf.len() < 12 {
        bail!("file too short for an iFly ADS-B recording");
    }
    let start_ticks = i64::from_le_bytes(buf[0..8].try_into()?);
    let brand = i32::from_le_bytes(buf[8..12].try_into()?);
    let mut pos = 12;
    let mut records = Vec::new();
    let mut comments = Vec::new();
    while pos + 4 <= buf.len() {
        let ms = i32::from_le_bytes(buf[pos..pos + 4].try_into()?);
        pos += 4;
        if ms == i32::MAX {
            break;
        }
        if ms == -1 {
            match read_dotnet_string(buf, &mut pos) {
                Some(s) => comments.push(s),
                None => break,
            }
            continue;
        }
        if pos + 4 > buf.len() {
            break;
        }
        let count = i32::from_le_bytes(buf[pos..pos + 4].try_into()?);
        pos += 4;
        if !(1..=120_000).contains(&count) || pos + count as usize > buf.len() {
            break;
        }
        records.push((ms.max(0) as u32, buf[pos..pos + count as usize].to_vec()));
        pos += count as usize;
    }
    if records.is_empty() {
        bail!("no data records found (not an iFly ADS-B .bin?)");
    }
    Ok(Recording { start_ticks, brand, records, comments })
}

pub fn load(path: &Path) -> Result<Recording> {
    let mut buf = Vec::new();
    File::open(path)
        .with_context(|| format!("open {}", path.display()))?
        .read_to_end(&mut buf)?;
    parse(&buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let dir = std::env::temp_dir().join("gdl90emu-rec-test.bin");
        let mut r = Recorder::create(&dir, 14, 1_700_000_000.0).unwrap();
        r.write(1_700_000_000.0, &[0x7E, 0x00, 0x7E]).unwrap();
        r.write(1_700_000_001.5, &[0x7E, 0x0A, 0x7E]).unwrap();
        r.finish().unwrap();
        let rec = load(&dir).unwrap();
        assert_eq!(rec.brand, 14);
        assert_eq!(rec.records.len(), 2);
        assert_eq!(rec.records[1].0, 1500);
        std::fs::remove_file(dir).ok();
    }
}
