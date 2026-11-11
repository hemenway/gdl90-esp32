//! Captive-portal DNS on udp/53 (answers every name with 192.168.4.1 while enabled).

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use crate::Shared;

pub fn spawn(sh: Arc<Shared>) -> anyhow::Result<()> {
    std::thread::Builder::new().name("dns".into()).stack_size(4096).spawn(move || {
        let sock = match UdpSocket::bind("0.0.0.0:53") {
            Ok(s) => s,
            Err(e) => {
                log::warn!("captive DNS: cannot bind udp/53: {e}");
                return;
            }
        };
        let mut buf = [0u8; 512];
        loop {
            let Ok((n, from)) = sock.recv_from(&mut buf) else { continue };
            if !sh.captive.load(Ordering::Relaxed) {
                continue; // like a Stratux: no DNS at all, devices just see "no internet"
            }
            if let Some(r) = crate::captive::dns_answer(&buf[..n], crate::AP_IP) {
                let _ = sock.send_to(&r, from);
            }
        }
    })?;
    Ok(())
}
