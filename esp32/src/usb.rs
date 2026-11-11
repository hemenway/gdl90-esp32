//! Console UART transport for the loopback browser launcher. Requests are newline-delimited
//! JSON. Each reply chunk is one console write, so ordinary boot and diagnostic log lines
//! can share the port. The host joins only chunks tagged with its current request ID.
use std::io::{self, Write};
use std::sync::Arc;

use anyhow::Result;
use esp_idf_svc::sys;
use serde::Deserialize;
use serde_json::Value;

use crate::{web, Shared};

const PREFIX: &[u8] = b"G90? ";
const CHUNK: usize = 960;

#[derive(Deserialize)]
struct Request<'a> {
    id: u32,
    method: &'a str,
    path: &'a str,
    #[serde(default)]
    body: Value,
}

struct Reply {
    packet: Vec<u8>,
    prefix_len: usize,
}

impl Reply {
    fn new(id: u32, status: u16) -> Self {
        let mut packet = Vec::with_capacity(CHUNK + 1);
        write!(packet, "G90! {id} {status} ").unwrap();
        Self { prefix_len: packet.len(), packet }
    }
}

impl Write for Reply {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let take = bytes.len().min(CHUNK - self.packet.len());
        self.packet.extend_from_slice(&bytes[..take]);
        if self.packet.len() == CHUNK {
            self.flush()?;
        }
        Ok(take)
    }

    fn flush(&mut self) -> io::Result<()> {
        if self.packet.len() > self.prefix_len {
            self.packet.push(b'\n');
            io::stdout().write_all(&self.packet)?;
            self.packet.truncate(self.prefix_len);
        }
        Ok(())
    }
}

fn handle(sh: &Arc<Shared>, line: &[u8]) {
    let Some(data) = line.strip_prefix(PREFIX) else { return };
    let Ok(req) = serde_json::from_slice::<Request>(data) else { return };
    web::with_reply(sh, req.method, req.path, &req.body, |result| {
        let (status, payload) = match result {
            Ok(reply) => (200, reply),
            Err(err) => (400, web::Payload::Json(Value::String(err.to_string()))),
        };
        let mut out = Reply::new(req.id, status);
        if payload.write_to(&mut out).is_ok() && out.flush().is_ok() {
            println!("G90! {} END", req.id);
        }
    });
}

// The IDF 5.3 UART VFS symbol is available even when the Rust bindings omit uart_vfs.h.
extern "C" {
    fn uart_vfs_dev_use_driver(uart_num: i32);
}

pub fn start(sh: Arc<Shared>) -> Result<()> {
    // UART0 is already routed to the USB bridge by the board's console configuration.
    // The interrupt-driven RX ring preserves full requests while Wi-Fi tasks are active.
    unsafe {
        sys::esp!(sys::uart_driver_install(0, 2048, 1024, 0, core::ptr::null_mut(), 0))?;
        uart_vfs_dev_use_driver(0);
    }
    std::thread::Builder::new()
        .name("usb-control".into())
        .stack_size(18 * 1024)
        .spawn(move || {
            let mut buf = [0u8; 256];
            let mut line = Vec::new();
            let mut overflow = false;
            loop {
                let n = unsafe {
                    sys::uart_read_bytes(0, buf.as_mut_ptr().cast(), buf.len() as _, 20)
                };
                if n <= 0 {
                    std::thread::sleep(std::time::Duration::from_millis(10));
                    continue;
                }
                for &b in &buf[..n as usize] {
                    match b {
                        b'\n' => {
                            if !overflow {
                                handle(&sh, &line);
                            }
                            line.clear();
                            if line.capacity() > 1024 {
                                line.shrink_to(512);
                            }
                            overflow = false;
                        }
                        _ if line.len() < web::MAX_BODY + 512 => line.push(b),
                        _ => overflow = true,
                    }
                }
            }
        })?;
    Ok(())
}
