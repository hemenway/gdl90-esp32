//! Captive-portal DNS: answer every A query with the emulator's own address, so a phone or PC
//! that joins the emulator's Wi-Fi runs its "sign in to network" check against us and opens the
//! control panel by itself. AAAA and other types get an empty (NOERROR) answer.

/// Build the response for one DNS query packet, or `None` if it isn't a query we answer.
pub fn dns_answer(q: &[u8], ip: [u8; 4]) -> Option<Vec<u8>> {
    if q.len() < 12 {
        return None;
    }
    let flags = u16::from_be_bytes([q[2], q[3]]);
    if flags & 0x8000 != 0 || (flags >> 11) & 0xF != 0 {
        return None; // a response, or not a standard query
    }
    if u16::from_be_bytes([q[4], q[5]]) == 0 {
        return None;
    }
    // first question: QNAME labels, then QTYPE + QCLASS
    let mut i = 12;
    while i < q.len() && q[i] != 0 {
        if q[i] & 0xC0 != 0 {
            return None;
        }
        i += 1 + q[i] as usize;
    }
    if i + 5 > q.len() {
        return None;
    }
    let qtype = u16::from_be_bytes([q[i + 1], q[i + 2]]);
    let qend = i + 5;
    let answer = qtype == 1 || qtype == 255;
    let mut r = Vec::with_capacity(qend + 16);
    r.extend_from_slice(&q[0..2]); // ID
    r.push(0x80 | (q[2] & 0x01)); // QR=1, opcode 0, copy RD
    r.push(0x80); // RA=1, RCODE=0
    r.extend_from_slice(&[0, 1, 0, answer as u8, 0, 0, 0, 0]);
    r.extend_from_slice(&q[12..qend]);
    if answer {
        // name = pointer to the question, type A, class IN, TTL 30 s, 4 bytes
        r.extend_from_slice(&[0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, 30, 0, 4]);
        r.extend_from_slice(&ip);
    }
    Some(r)
}

/// HTTP paths operating systems probe to detect a captive portal.
pub const PROBE_PATHS: &[&str] = &[
    "/generate_204",
    "/gen_204",
    "/hotspot-detect.html",
    "/library/test/success.html",
    "/connecttest.txt",
    "/ncsi.txt",
    "/redirect",
    "/success.txt",
    "/canonical.html",
];

#[cfg(test)]
mod tests {
    use super::*;

    fn query(name: &str, qtype: u16) -> Vec<u8> {
        let mut q = vec![0x12, 0x34, 0x01, 0x00, 0, 1, 0, 0, 0, 0, 0, 0];
        for label in name.split('.') {
            q.push(label.len() as u8);
            q.extend_from_slice(label.as_bytes());
        }
        q.push(0);
        q.extend_from_slice(&qtype.to_be_bytes());
        q.extend_from_slice(&1u16.to_be_bytes());
        q
    }

    #[test]
    fn a_query_gets_our_address() {
        let q = query("www.msftconnecttest.com", 1);
        let r = dns_answer(&q, [192, 168, 4, 1]).unwrap();
        assert_eq!(&r[0..2], &[0x12, 0x34]);
        assert_eq!(r[2] & 0x80, 0x80);
        assert_eq!(u16::from_be_bytes([r[6], r[7]]), 1); // one answer
        assert_eq!(&r[r.len() - 4..], &[192, 168, 4, 1]);
        assert_eq!(&r[12..q.len()], &q[12..]);
    }

    #[test]
    fn aaaa_gets_empty_answer_and_responses_are_ignored() {
        let r = dns_answer(&query("connectivitycheck.gstatic.com", 28), [192, 168, 4, 1]).unwrap();
        assert_eq!(u16::from_be_bytes([r[6], r[7]]), 0);
        let mut resp = query("x.com", 1);
        resp[2] |= 0x80;
        assert!(dns_answer(&resp, [192, 168, 4, 1]).is_none());
        assert!(dns_answer(&[1, 2, 3], [192, 168, 4, 1]).is_none());
    }
}
