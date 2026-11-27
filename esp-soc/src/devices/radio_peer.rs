//! Native two-process packet medium for the SX1262 and CC1101 board models.
//! Both guests start at zero after rendezvous. At each 1 ms boundary they exchange
//! completed transmissions before granting each other the next slice. A frame's
//! fixed 2 ms delivery delay gives the receiver time to schedule its exact guest
//! deadline even when its host process reaches the barrier first.
use std::collections::VecDeque;
use std::io::{self, BufReader, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

const SLICE_NS: u64 = 1_000_000;
const DELAY_NS: u64 = 2 * SLICE_NS;
const TIMEOUT: Duration = Duration::from_secs(60);
const HEADER: usize = 11;
const MAX_PACKET: usize = HEADER + 255;
const FINISH: u64 = u64::MAX;

pub const SX1262: u8 = 1;
pub const CC1101: u8 = 2;

pub struct Frame {
    due: u64,
    packet: Vec<u8>,
}

impl Frame {
    pub fn matching(&self, kind: u8, channel: [u8; 10]) -> Option<&[u8]> {
        (self.packet[0] == kind && self.packet[1..HEADER] == channel).then_some(&self.packet[HEADER..])
    }
}

pub struct RadioPeer {
    wire: Option<BufReader<TcpStream>>,
    reservation: Option<TcpListener>,
    cycles_per_us: u64,
    boundary_ns: u64,
    outgoing: Vec<u8>,
    incoming: VecDeque<Frame>,
    error: Option<io::Error>,
}

impl RadioPeer {
    pub fn connect((local, remote): (u16, u16), cycles_per_us: u64) -> io::Result<Self> {
        // Reserve both advertised ports. The higher side dials the lower side once;
        // the claimed port pair must match before either guest leaves reset.
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, local))?;
        listener.set_nonblocking(true)?;
        let until = Instant::now() + TIMEOUT;
        let mut stream = loop {
            if local < remote {
                match listener.accept() {
                    Ok((stream, _)) => break stream,
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => {},
                    Err(e) => return Err(e),
                }
            } else {
                let address = SocketAddr::from((Ipv4Addr::LOCALHOST, remote));
                match TcpStream::connect_timeout(&address, Duration::from_millis(100)) {
                    Ok(stream) => break stream,
                    Err(e) if matches!(e.kind(), io::ErrorKind::ConnectionRefused | io::ErrorKind::TimedOut) => {},
                    Err(e) => return Err(e),
                }
            }
            if Instant::now() >= until {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "no radio peer joined"));
            }
            std::thread::sleep(Duration::from_millis(10));
        };
        stream.set_nonblocking(false)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(TIMEOUT))?;
        stream.set_write_timeout(Some(TIMEOUT))?;
        stream.write_all(&hello(local, remote))?;
        let mut reply = [0; 8];
        stream.read_exact(&mut reply)?;
        if reply != hello(remote, local) { return Err(invalid("radio peer protocol or ports differ")); }
        let mut peer = Self::new(stream, cycles_per_us);
        peer.reservation = Some(listener);
        Ok(peer)
    }

    fn new(stream: TcpStream, cycles_per_us: u64) -> Self {
        Self { wire: Some(BufReader::new(stream)), reservation: None, cycles_per_us, boundary_ns: SLICE_NS,
               outgoing: Vec::new(), incoming: VecDeque::new(), error: None }
    }

    fn cycle(&self, ns: u64) -> u64 { (ns * self.cycles_per_us).div_ceil(1000) }

    pub fn deadline(&self) -> Option<u64> {
        self.wire.as_ref().map(|_| self.cycle(self.boundary_ns))
            .into_iter().chain(self.incoming.front().map(|f| f.due)).min()
    }

    /// Queue one completed transmission. Channel and payload are captured before
    /// the next guest instruction can change the chip's configuration.
    pub fn transmit(&mut self, now: u64, kind: u8, channel: [u8; 10], payload: &[u8]) -> bool {
        if self.wire.is_none() { return false; }
        put_header(&mut self.outgoing, now * 1000 / self.cycles_per_us, HEADER + payload.len());
        self.outgoing.push(kind);
        self.outgoing.extend(channel);
        self.outgoing.extend(payload);
        true
    }

    pub fn advance(&mut self, now: u64) -> impl Iterator<Item = Frame> + '_ {
        if self.wire.is_some() && now >= self.cycle(self.boundary_ns) {
            if let Err(e) = self.exchange(now) {
                self.error = Some(e);
                self.wire = None;
                self.incoming.clear();
            }
        }
        std::iter::from_fn(move || {
            if self.incoming.front().is_some_and(|f| f.due <= now) {
                self.incoming.pop_front()
            } else { None }
        })
    }

    fn exchange(&mut self, now: u64) -> io::Result<()> {
        let wire = self.wire.as_mut().unwrap();
        put_header(&mut self.outgoing, self.boundary_ns, 0);
        wire.get_mut().write_all(&self.outgoing)?;
        self.outgoing.clear();

        loop {
            let (timestamp, len) = read_header(self.wire.as_mut().unwrap())?;
            if len == 0 {
                if timestamp == FINISH {
                    // This acknowledges the peer's final frames. We can run ahead
                    // after this point, keeping any already scheduled receptions.
                    self.wire.take().unwrap().get_mut().write_all(&finish_header())?;
                } else if timestamp == self.boundary_ns {
                    self.boundary_ns += SLICE_NS;
                } else {
                    return Err(invalid("unexpected radio synchronization boundary"));
                }
                return Ok(());
            }
            if timestamp < self.boundary_ns - SLICE_NS || timestamp > self.boundary_ns {
                return Err(invalid("radio frame timestamp outside its slice"));
            }
            self.queue_frame(now, timestamp, len)?;
        }
    }

    fn queue_frame(&mut self, now: u64, timestamp: u64, len: usize) -> io::Result<()> {
        let due = self.cycle(timestamp + DELAY_NS);
        if due <= now || self.incoming.back().is_some_and(|frame| frame.due > due) {
            return Err(invalid("late or out-of-order radio frame"));
        }
        let mut packet = vec![0; len];
        self.wire.as_mut().unwrap().read_exact(&mut packet)?;
        self.incoming.push_back(Frame { due, packet });
        Ok(())
    }

    pub fn finish(&mut self) -> io::Result<()> {
        self.reservation = None;
        if let Some(error) = self.error.take() { return Err(error); }
        let Some(mut wire) = self.wire.take() else { return Ok(()); };
        self.outgoing.extend(finish_header());
        wire.get_mut().write_all(&self.outgoing)?;
        // The peer either finished at the same boundary or acknowledges on its
        // next exchange. Read through its last frames before closing the stream.
        let mut discarded = [0; MAX_PACKET];
        loop {
            let (timestamp, len) = read_header(&mut wire)?;
            if (timestamp, len) == (FINISH, 0) { break; }
            wire.read_exact(&mut discarded[..len])?;
        }
        Ok(())
    }
}


fn hello(local: u16, remote: u16) -> [u8; 8] {
    let [a, b] = local.to_be_bytes();
    let [c, d] = remote.to_be_bytes();
    [b'R', b'A', b'D', 1, a, b, c, d]
}

fn finish_header() -> [u8; 10] {
    let mut header = [0; 10];
    header[..8].copy_from_slice(&FINISH.to_be_bytes());
    header
}

fn put_header(out: &mut Vec<u8>, time: u64, len: usize) {
    out.extend(time.to_be_bytes());
    out.extend((len as u16).to_be_bytes());
}

fn read_header(input: &mut impl Read) -> io::Result<(u64, usize)> {
    let mut time = [0; 8];
    let mut len = [0; 2];
    input.read_exact(&mut time)?;
    input.read_exact(&mut len)?;
    let len = u16::from_be_bytes(len) as usize;
    if len != 0 && !(HEADER..=MAX_PACKET).contains(&len) { return Err(invalid("invalid radio frame size")); }
    Ok((u64::from_be_bytes(time), len))
}

fn invalid(reason: &str) -> io::Error { io::Error::new(io::ErrorKind::InvalidData, reason) }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_time_and_final_frames_survive_different_guest_rates() {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let tx_stream = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (rx_stream, _) = listener.accept().unwrap();
        for stream in [&tx_stream, &rx_stream] {
            stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        }
        let sender = std::thread::spawn(move || {
            let mut tx = RadioPeer::new(tx_stream, 160);
            assert!(tx.transmit(80_000, SX1262, [1; 10], b"first"));
            assert_eq!(tx.advance(160_000).count(), 0);
            assert!(tx.transmit(240_000, CC1101, [2; 10], b"final"));
            tx.finish().unwrap();
        });
        let mut rx = RadioPeer::new(rx_stream, 240);
        assert_eq!(rx.deadline(), Some(240_000));
        assert_eq!(rx.advance(240_000).count(), 0);
        assert_eq!(rx.advance(480_000).count(), 0);
        assert_eq!(rx.deadline(), Some(600_000));
        assert_eq!(rx.advance(599_999).count(), 0);
        let first = rx.advance(600_000).next().unwrap();
        assert_eq!(first.matching(SX1262, [1; 10]), Some(&b"first"[..]));
        assert_eq!(first.matching(SX1262, [0; 10]), None);
        assert_eq!(rx.deadline(), Some(840_000));
        let final_frame = rx.advance(840_000).next().unwrap();
        assert_eq!(final_frame.matching(CC1101, [2; 10]), Some(&b"final"[..]));
        assert_eq!(rx.deadline(), None);
        assert!(!rx.transmit(850_000, CC1101, [2; 10], b"no peer"));
        rx.finish().unwrap();
        sender.join().unwrap();
    }
}
