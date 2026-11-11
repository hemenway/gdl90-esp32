//! Power-on settings in NVS flash (the whole config tree as one JSON blob).

use anyhow::Result;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use crate::config::Config;

pub struct Store {
    nvs: EspNvs<NvsDefault>,
}

const KEY: &str = "cfg";

impl Store {
    pub fn new(p: EspDefaultNvsPartition) -> Result<Self> {
        Ok(Self { nvs: EspNvs::new(p, "gdl90", true)? })
    }

    pub fn load_config(&self) -> Option<Config> {
        let len = self.nvs.blob_len(KEY).ok()??;
        if len > 20 * 1024 { return None; }
        let mut buf = vec![0u8; len];
        let data = self.nvs.get_blob(KEY, &mut buf).ok()??;
        match serde_json::from_slice::<Config>(data).map_err(anyhow::Error::from).and_then(|c| { c.validate()?; Ok(c) }) {
            Ok(c) => Some(c),
            Err(e) => {
                log::warn!("saved settings unreadable, using defaults: {e}");
                None
            }
        }
    }

    pub fn save_config(&mut self, c: &Config) -> Result<usize> {
        let v = serde_json::to_vec(c)?;
        anyhow::ensure!(v.len() <= 20 * 1024, "Saved settings exceed 20 KiB");
        self.nvs.set_blob(KEY, &v)?;
        Ok(v.len())
    }

    pub fn has_saved(&self) -> bool {
        self.nvs.blob_len(KEY).ok().flatten().is_some()
    }

    pub fn clear(&mut self) -> Result<()> {
        self.nvs.remove(KEY)?;
        Ok(())
    }
}
