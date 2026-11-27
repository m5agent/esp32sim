//! Shared backing store for both interfaces to Tab5’s microSD socket.

/// Two protocol front ends of the same physical socket use one backing store.
#[derive(Clone)]
pub struct SocketStore(pub std::rc::Rc<std::cell::RefCell<Box<dyn esp_periph::sdmmc::SdStore>>>);
impl esp_periph::sdmmc::SdStore for SocketStore {
    fn len(&self) -> u64 { self.0.borrow().len() }
    fn read(&mut self, lba: u64, buf: &mut [u8; 512]) -> std::io::Result<()> { self.0.borrow_mut().read(lba, buf) }
    fn write(&mut self, lba: u64, buf: &[u8; 512]) -> std::io::Result<()> { self.0.borrow_mut().write(lba, buf) }
    fn erase(&mut self, lo: u64, hi: u64) -> std::io::Result<()> { self.0.borrow_mut().erase(lo, hi) }
    fn describe(&self) -> String { self.0.borrow().describe() }
}
