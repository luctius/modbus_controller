use heapless::consts::U64;
use crate::units::MiliSeconds;

pub(crate) const TIMEOUT: MiliSeconds = MiliSeconds(1700);

pub type ModBusBufferSz = U64;
