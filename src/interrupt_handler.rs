use embedded_hal::{
    serial::{
        Read, Write,
    },
    digital::v2::OutputPin,
};

use modbus_core::rtu::SlaveId;

use heapless::Vec;
use crate::units::{Instant, CharacterDelay};
use crate::constants::ModBusBufferSz;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum IrqStates {
    SendingStart,
    Sending,
    SendingComplete,
    ReceiveWait,
    Receiving(Instant),
    Idle,
}

pub struct IrqContext<W, R, P, UNIT, ESW, ESR, EP>
            where W: Write<UNIT, Error=ESW>,
                R: Read<UNIT, Error=ESR>,
                P: OutputPin<Error=EP>,
                UNIT: Copy +PartialEq + From<u8> + Into<u8> + core::fmt::Debug,
                ESW: core::fmt::Debug,
                ESR: core::fmt::Debug,
                EP: core::fmt::Debug, {
    pub(crate) buffer: Vec<u8, ModBusBufferSz>,
    pub(crate) state: IrqStates,
    pub(crate) chardelay: CharacterDelay,
    pub(crate) slave_id: SlaveId,
    tx: W,
    rx: R,
    pin: Option<P>,
    _esw: core::marker::PhantomData<ESW>,
    _esr: core::marker::PhantomData<ESR>,
    _ep: core::marker::PhantomData<EP>,
    _unit: core::marker::PhantomData<UNIT>,
}
impl<W, R, P, UNIT, ESW, ESR, EP> IrqContext<W, R, P, UNIT, ESW, ESR, EP>
            where W: Write<UNIT, Error=ESW>,
                R: Read<UNIT, Error=ESR>,
                P: OutputPin<Error=EP>,
                UNIT: Copy +PartialEq + From<u8> + Into<u8> + core::fmt::Debug,
                ESW: core::fmt::Debug,
                ESR: core::fmt::Debug,
                EP: core::fmt::Debug {
    #[must_use]
    pub fn new( (tx, rx): (W, R), pin: Option<P>, slave_id: SlaveId, baudrate: u32) -> Self {
        Self {
            state: IrqStates::Idle,
            buffer: Vec::new(),
            tx,
            rx,
            pin,
            chardelay: CharacterDelay::from_baudrate(baudrate).to_t3_5(),
            slave_id,
            _esw: core::marker::PhantomData,
            _esr: core::marker::PhantomData,
            _ep: core::marker::PhantomData,
            _unit: core::marker::PhantomData,
        }
    }
    pub fn irq_handler(&mut self, now: Instant) {
        match self.state {
            IrqStates::Idle => {},
            IrqStates::SendingStart => {
                if !self.buffer.is_empty() {
                    self.state = IrqStates::Sending;

                    if let Some(pin) = &mut self.pin {
                        pin.set_high().unwrap();
                    }

                    if self.tx.flush().is_ok() {
                        if let Some(byte) = self.buffer.pop() {
                            self.tx.write(byte.into() ).unwrap();
                        }
                    }
                }
            },
            IrqStates::Sending => {
                if self.tx.flush().is_ok() {
                    if let Some(byte) = self.buffer.pop() {
                        self.tx.write(byte.into() ).unwrap();
                    }
                    else {
                        self.state = IrqStates::SendingComplete;
                    }
                }
            },
            IrqStates::SendingComplete => {
                if self.tx.flush().is_ok() {
                    if let Some(pin) = &mut self.pin {
                        pin.set_low().unwrap();
                    }
                    self.buffer.clear();
                    self.state = IrqStates::ReceiveWait;
                }
            },
            IrqStates::ReceiveWait => {
                if let Ok(byte) = self.rx.read() {
                    if self.slave_id == byte.into() && self.buffer.len() < self.buffer.capacity() {
                        self.buffer.push(byte.into() ).unwrap();
                        self.state = IrqStates::Receiving(now);
                    }
                }
            }
            IrqStates::Receiving(_) => {
                if let Ok(byte) = self.rx.read() {
                    if self.buffer.len() < self.buffer.capacity() {
                        self.buffer.push(byte.into() ).unwrap();
                        self.state = IrqStates::Receiving(now);
                    }
                }
            },
        }
    }
}

