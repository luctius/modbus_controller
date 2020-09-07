use modbus_core::{
    rtu::{
        master::{
            encode_request,
            decode_response,
        },
        RequestAdu,
        SlaveId,
        Header,
    },
    Request,
    RequestPdu,
    Response,
    Address,
    Quantity,
    ObjectId,
    Data,
    Coils,
};

use embedded_hal::{
    serial::{
        Read, Write,
    },
    digital::v2::OutputPin,
};

use heapless::{
    consts::U8,
    Vec,
};

use crate::units::Instant;
use crate::constants::{TIMEOUT, ModBusBufferSz, };
use crate::interrupt_handler::{
    IrqStates,
    IrqContext,
};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CallbackError {
    DataTooShort,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum HandleResponseError {
    WrongReponse,
    InvalidDataLength,
}

type DeviceIdx = usize;
type ReadDataCallbackFn  <C> = fn(context: &mut C, data: Data<'_>) -> Result<(), CallbackError>;
type ReadCoilsCallbackFn <C> = fn(context: &mut C, data: Coils<'_>) -> Result<(), CallbackError>;
type WriteDataCallbackFn <C> = fn(context: &C, data: &mut Vec<u16, ModBusBufferSz>);
type WriteCoilsCallbackFn<C> = fn(context: &C, data: &mut Vec<bool, ModBusBufferSz>);

pub enum Command<C> {
    ReadMultipleCoils(Address, Quantity, ReadCoilsCallbackFn<C>),
    WriteMultipleCoils(Address, WriteCoilsCallbackFn<C>),
    ReadMultipleHoldingRegisters(Address, Quantity, ReadDataCallbackFn<C>),
    WriteMultipleHoldingRegisters(Address, WriteDataCallbackFn<C>),
    ReadInputRegisters(Address, Quantity, ReadDataCallbackFn<C>),
    ReadDiscreteInputs(Address, Quantity, ReadCoilsCallbackFn<C>),
}
impl<C> Command<C> {
    #[must_use]
    fn to_request<'a>(&self, context: &C, data: &'a mut[u8]) -> Option<Request<'a> > {
        match self {
            Self::ReadMultipleCoils(addr, qnt, _) => {
                Some(Request::ReadCoils(*addr, *qnt) )
            },
            Self::WriteMultipleCoils(addr, func) => {
                let mut buf = Vec::new();
                func(context, &mut buf);
                if let Ok(data) = Coils::from_bools(&buf, data) {
                    Some(Request::WriteMultipleCoils(*addr, data) )
                }
                else {
                    None
                }
            },
            Self::ReadMultipleHoldingRegisters(addr, qnt, _) => {
                Some(Request::ReadHoldingRegisters(*addr, *qnt) )
            },
            Self::WriteMultipleHoldingRegisters(addr, func) => {
                let mut buf = Vec::new();
                func(context, &mut buf);
                if let Ok(data) = Data::from_words(&buf, data) {
                    Some(Request::WriteMultipleRegisters(*addr, data) )
                }
                else {
                    None
                }
            },
            Self::ReadInputRegisters(addr, qnt, _) => {
                Some(Request::ReadInputRegisters(*addr, *qnt) )
            },
            Self::ReadDiscreteInputs(addr, qnt, _) => {
                Some(Request::ReadDiscreteInputs(*addr, *qnt) )
            },
        }
    }
    fn handle_response<'a>(&self, response: Response<'a>, context: &'a mut C) -> Result<(),HandleResponseError> {
        match (self, response) {
            (Self::ReadMultipleCoils(_, qnt, func), Response::ReadCoils(coils) ) |
                (Self::ReadDiscreteInputs(_, qnt, func), Response::ReadDiscreteInputs(coils) ) => {
                if coils.len() == usize::from(*qnt) {
                    func(context, coils).unwrap();
                    Ok( () )
                }
                else { Err(HandleResponseError::InvalidDataLength) }
            },
            (Self::ReadMultipleHoldingRegisters(_, qnt, func), Response::ReadHoldingRegisters(data) ) |
                (Self::ReadInputRegisters(_, qnt, func), Response::ReadInputRegisters(data) ) => {
                if data.len() == usize::from(*qnt) {
                    func(context, data).unwrap();
                    Ok( () )
                }
                else { Err(HandleResponseError::InvalidDataLength) }
            },
            (Self::WriteMultipleCoils(_, _), Response::WriteMultipleCoils(_, _qnt) ) |
                (Self::WriteMultipleHoldingRegisters(_, _), Response::WriteMultipleRegisters(_, _qnt) )
                => { Ok( () ) },
            _ => { Err(HandleResponseError::WrongReponse) },
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
struct DeviceAtom {
    address: SlaveId,
    device_id: DeviceIdx,
}

pub struct Device<'a, C> {
    pub addr_list: &'a [SlaveId],
    pub vendor_id: &'a str,
    pub product_code: &'a str,
    pub cmd: Command<C>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum IdentificationState {
    Vendor  = 0,
    Product = 1,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum ProbeState {
    Send(DeviceAtom, IdentificationState),
    Receive(DeviceAtom, IdentificationState, Instant),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum State {
    Start(Instant),
    Probing(ProbeState),
    Sending(usize),
    Receiving(usize, Instant),
    Idle,
}

pub struct Master<'a, C, IrqPend> where IrqPend: Fn() {
    static_list: &'a [Device<'a, C>],
    state: State,
    device_list: Vec<DeviceAtom,U8>,
    irq_pend: IrqPend,
}
impl<'a, C, IrqPend> Master<'a, C, IrqPend>  where IrqPend: Fn() {
    #[must_use]
    pub fn new(devlist: &'a [Device<'a, C>], irq_pend: IrqPend) -> Self {
        Self {
            state: State::Start(Instant(0) ),
            device_list: Vec::new(),
            static_list: devlist,
            irq_pend,
        }
    }

    fn find_next_dev(&self, idx: usize) -> Option<usize> {
        if self.device_list.is_empty() {
            // This path *should* not happen...
            // It can only happen if we found no devices active
            // Which means the master should be in idle, not seeking out devs.
            None
        }
        else if self.device_list.get(idx+1).is_some() {
            Some(idx+1)
        }
        else {
            // There was no next device on the list
            // We therefor to the first device.
            Some(0)
        }
    }

    fn find_next_probe(&self, atom: DeviceAtom) -> Option<DeviceAtom> {
        if let Some( (i,_) ) = self.static_list[atom.device_id].addr_list.iter().enumerate().find(|(_,addr)| **addr == atom.address) {

            // Use next address if it exists, or move on to the next device
            self.static_list[atom.device_id].addr_list.get(i)
                .map(|addr| DeviceAtom { device_id: atom.device_id, address: *addr, })
                .or_else(|| self.static_list.get(atom.device_id+1)
                    .map(|_| DeviceAtom { device_id: atom.device_id+1, address: 0, })
            )
        }
        else {
            // Could not find address in the list
            // We therefor move to the next device.
            // This path *should* not happen anyway...
            let device_id = atom.device_id +1;
            let address = 0;

            self.static_list.get(device_id)
                .map(|_| DeviceAtom{ device_id, address} )
        }
    }

    pub fn is_probing(&self) -> bool {
        if let State::Probing(_) = self.state { true }
        else { false }
    }

    pub fn process<W, R, P, UNIT, ESW, ESR, EP>(&mut self, irq_ctx: &mut IrqContext<W, R, P, UNIT, ESW, ESR, EP>, context: &mut C, now: Instant)
            where W: Write<UNIT, Error=ESW>,
                R: Read<UNIT, Error=ESR>,
                P: OutputPin<Error=EP>,
                UNIT: Copy +PartialEq + From<u8> + Into<u8> + core::fmt::Debug,
                ESW: core::fmt::Debug,
                ESR: core::fmt::Debug,
                EP: core::fmt::Debug {
        match (self.state, irq_ctx.state) {
            (State::Start(timestamp), _) => {
                let duration = now.duration_since(timestamp);

                if duration > irq_ctx.chardelay.to_t3_5().into() {
                    self.state = State::Probing(
                        ProbeState::Send(
                            DeviceAtom{
                                address: self.static_list[0].addr_list[0],
                                device_id: 0,
                            }, IdentificationState::Vendor) );
                    irq_ctx.state = IrqStates::SendingStart;
                }
            },
            (State::Sending(dev), IrqStates::SendingStart) => {
                self.state = State::Idle;

                if let Some(dev_atom) = self.device_list.get(dev) {
                    let address = dev_atom.address;
                    let id = dev_atom.device_id;

                    irq_ctx.buffer.clear();

                    let mut buffer: Vec<u8, ModBusBufferSz> = Vec::new();
                    if let Some(request) = self.static_list[id].cmd.to_request(context, &mut buffer) {
                        let reqadu = RequestAdu {
                            hdr: Header { slave: address },
                            pdu: RequestPdu(request),
                        };

                        if encode_request(reqadu, &mut irq_ctx.buffer).is_ok() {
                            self.state = State::Receiving(dev, now);
                            irq_ctx.state = IrqStates::SendingStart;
                            (self.irq_pend)();
                        }
                    }
                }

                // Move on to the next device if there was an problem encoding the current device's
                // request
                if self.state == State::Idle {
                    if let Some(dev) = self.find_next_dev(dev) {
                        self.state = State::Sending(dev);
                    }
                }
            },
            (State::Receiving(dev, timestamp), IrqStates::Receiving(last_rx) ) => {
                let duration = now.duration_since(timestamp);
                let frame_delay = now.duration_since(last_rx);

                if frame_delay > irq_ctx.chardelay.to_t3_5().into() {
                    //This should mark the end of the receiving frame
                    self.state = State::Idle;

                    match decode_response(&irq_ctx.buffer) {
                        Ok(Some(response_adu)) => {
                            if response_adu.hdr.slave == irq_ctx.slave_id {
                                if let Some(dev_atom) = self.device_list.get(dev) {
                                    if let Some(dev) = self.static_list.get(dev_atom.device_id) {
                                        match response_adu.pdu.0 {
                                            Ok(pdu) => {
                                                dev.cmd.handle_response(pdu, context);
                                            },
                                            Err(_exception) => {},
                                        }
                                    }
                                }
                            }
                        },
                        Ok(None) | Err(_) => {},
                    }
                }
                // move to next request because the slave device has not responded in time
                else if duration > TIMEOUT.into() { self.state = State::Idle; }

                if self.state == State::Idle {
                    if let Some(dev) = self.find_next_dev(dev) {
                        self.state = State::Sending(dev);
                        irq_ctx.state = IrqStates::SendingStart;
                    }
                }
            },
            (State::Receiving(dev, timestamp), IrqStates::ReceiveWait) => {
                // Test for Receive TimeOut

                let duration = now.duration_since(timestamp);
                if duration > TIMEOUT.into() {
                    self.state = State::Idle;
                    irq_ctx.state = IrqStates::SendingStart;

                    if let Some(dev) = self.find_next_dev(dev) {
                        self.state = State::Sending(dev);
                    }
                }
            }

            // Probing the listed devices in the modbus network
            (State::Probing(_), _) => {
                self.probe(irq_ctx, now);
            },
            _ => {},
        }
    }

    fn probe<W, R, P, UNIT, ESW, ESR, EP>(&mut self, irq_ctx: &mut IrqContext<W, R, P, UNIT, ESW, ESR, EP>, now: Instant)
            where W: Write<UNIT, Error=ESW>,
                R: Read<UNIT, Error=ESR>,
                P: OutputPin<Error=EP>,
                UNIT: Copy +PartialEq + From<u8> + Into<u8> + core::fmt::Debug,
                ESW: core::fmt::Debug,
                ESR: core::fmt::Debug,
                EP: core::fmt::Debug {
        match (self.state, irq_ctx.state) {
            // Probing the listed devices in the modbus network
            (State::Probing(ProbeState::Send(dev_atom, state)), IrqStates::SendingStart) => {
                let request = Request::ReadDeviceIdentification(state as ObjectId);

                let reqadu = RequestAdu {
                    hdr: Header { slave: dev_atom.address },
                    pdu: RequestPdu(request),
                };

                if encode_request(reqadu, &mut irq_ctx.buffer).is_ok() {
                    self.state = State::Probing( ProbeState::Receive(dev_atom, state, now) );
                    irq_ctx.state = IrqStates::SendingStart;
                    (self.irq_pend)();
                }
                else if let Some(dev_atom) = self.find_next_probe(dev_atom) {
                    // There are more devices to probe
                    self.state = State::Probing( ProbeState::Send(dev_atom, IdentificationState::Vendor) );
                }
                else if self.device_list.first().is_some() {
                    //Last device has been probed, and we have devices to work with
                    self.state = State::Sending(0);
                }
                else {
                    //Last device has been probed, but no devices were found
                    self.state = State::Idle;
                }
            },
            (State::Probing(ProbeState::Receive(dev_atom, state, timestamp) ), IrqStates::Receiving(last_rx) ) => {
                let duration = now.duration_since(timestamp);
                let frame_delay = now.duration_since(last_rx);

                if frame_delay > irq_ctx.chardelay.to_t3_5().into() {
                    //This should mark the end of the receiving frame
                    self.state = State::Idle;

                    match decode_response(&irq_ctx.buffer) {
                        Ok(Some(response_adu)) => {
                            if response_adu.hdr.slave == 0x0 {
                                if let Some(dev) = self.static_list.get(dev_atom.device_id) {
                                    if let Ok(Response::ReadDeviceIdentification(objid, data) ) = response_adu.pdu.0 {
                                        if objid == state as u8 {
                                            match state {
                                                IdentificationState::Vendor  => {
                                                    if dev.vendor_id.as_bytes() == data {
                                                        self.state = State::Probing( ProbeState::Send(dev_atom, IdentificationState::Product) );
                                                        irq_ctx.state = IrqStates::SendingStart;
                                                    }
                                                },
                                                IdentificationState::Product => {
                                                    if dev.product_code.as_bytes() == data {
                                                        self.device_list.push(dev_atom).unwrap();
                                                    }
                                                },
                                            }
                                        }
                                    }
                                }
                            }
                        },
                        Ok(None) | Err(_) => {},
                    }
                }
                // move to next request because the slave device has not responded in time
                else if duration > TIMEOUT.into() { self.state = State::Idle; }

                if self.state == State::Idle {
                    if let Some(dev_atom) = self.find_next_probe(dev_atom) {
                        self.state = State::Probing( ProbeState::Send(dev_atom, IdentificationState::Vendor) );
                        irq_ctx.state = IrqStates::SendingStart;
                    }
                }
            },
            (State::Probing(ProbeState::Receive(dev_atom, _, timestamp) ), IrqStates::ReceiveWait) => {
                // Test for Receive TimeOut while Probing

                let duration = now.duration_since(timestamp);
                if duration > TIMEOUT.into() {
                    self.state = State::Idle;
                    irq_ctx.state = IrqStates::SendingStart;

                    if let Some(dev_atom) = self.find_next_probe(dev_atom) {
                        self.state = State::Probing(ProbeState::Send(dev_atom, IdentificationState::Vendor) );
                    }
                }
            },
            _ => {},
        }
    }
}
