use modbus_core::{
    rtu::{
        server::{
            decode_request,
            encode_response,
        },
        Header,
        ResponseAdu,
    },
    Request,
    Response,
    ResponsePdu,
    FnCode,
    Address,
    Data,
    Coils,
    Exception,
    ExceptionResponse,
};

use embedded_hal::{
    serial::{
        Read, Write,
    },
    digital::v2::OutputPin,
};

use heapless::Vec;

use core::convert::TryFrom;
use crate::units::Instant;
use crate::constants::ModBusBufferSz;
use crate::interrupt_handler::{
    IrqStates,
    IrqContext,
};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum RegisterSetError {
    AddressOutOfBounds,
    DataInvalid,
}

pub struct RegisterList<'a, T> {
    pub address: Address,
    pub list: &'a mut [T],
    pub within_bounds: Option<&'a dyn Fn(usize, T) -> bool>
}
impl<'a, T> RegisterList<'a, T> {
    #[must_use]
    fn contains_addr(&self, address: core::ops::Range<usize>) -> bool {
        address.start >= usize::from(self.address) && address.end < usize::from(self.address) + self.len()
    }
    #[must_use]
    const fn len(&self) -> usize {
        self.list.len()
    }
    #[must_use]
    const fn is_empty(&self) -> bool {
        self.list.is_empty()
    }
    #[must_use]
    fn iter(&self) -> core::slice::Iter<'_, T> {
        self.list.iter()
    }
    #[must_use]
    fn iter_mut(&mut self) -> core::slice::IterMut<'_, T> {
        self.list.iter_mut()
    }

    fn set(&mut self, address: usize, val: T) -> Result<(), RegisterSetError> where T: Copy {
        if self.contains_addr(address..address) {
            let mut valid = true;
            if let Some(func) = self.within_bounds {
                if valid && !func(address, val) {
                    valid = false;
                }
            }

            if valid {
                self.list[address - usize::from(self.address)] = val;
                Ok(())
            }
            else { Err(RegisterSetError::DataInvalid) }
        }
        else { Err(RegisterSetError::AddressOutOfBounds) }
    }
}

pub struct Registers<'a> {
    pub coils:              RegisterList<'a, bool>,
    pub discrete_inputs:    RegisterList<'a, bool>,
    pub input_registers:    RegisterList<'a, u16>,
    pub holding_registers:  RegisterList<'a, u16>,
    pub vendor_id:          Option<&'a str>,
    pub product_id:         Option<&'a str>,
    pub major_minor_rev:    Option<&'a str>,
}
impl<'a> Registers<'a> {
    fn handle_request(&mut self, req: Request<'_>) -> Result<(), Exception> {
        match req {
            Request::WriteSingleCoil(a, coil) => {
                let start_address = usize::from(a) - usize::from(self.coils.address);
                if self.coils.contains_addr(start_address..start_address) {
                    let address = start_address - usize::from(self.coils.address);
                    self.coils.list[address] = coil;
                    Ok(())
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::WriteMultipleCoils(a, coils) => {
                let start_address = usize::from(a) - usize::from(self.coils.address);
                if self.coils.contains_addr(start_address..start_address + coils.len() ) {
                    let address = start_address - usize::from(self.coils.address);

                    for (i,coil) in coils.into_iter().enumerate() {
                        self.coils.list[address +i] = coil;
                    }
                    Ok(())
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::WriteSingleRegister(a, word) => {
                let start_address = usize::from(a) - usize::from(self.holding_registers.address);
                if self.holding_registers.contains_addr(start_address..start_address) {
                    let address = start_address - usize::from(self.holding_registers.address);

                    self.holding_registers.set(address, word).map(|_| () ).map_err(|_| Exception::IllegalDataValue)
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::WriteMultipleRegisters(a, data) | Request::ReadWriteMultipleRegisters(_, _, a, data) => {
                let start_address = usize::from(a) - usize::from(self.holding_registers.address);
                if self.holding_registers.contains_addr(start_address..start_address + data.len() ) {
                    for (i,d) in data.into_iter().enumerate() {
                        if self.holding_registers.set(start_address +i, d).is_err() {
                            return Err(Exception::IllegalDataValue);
                        }
                    }

                    Ok(())
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadHoldingRegisters(_, _)  | Request::ReadCoils(_, _) |
                Request::ReadDiscreteInputs(_, _) | Request::ReadInputRegisters(_, _) |
                Request::ReadExceptionStatus | Request::Diagnostics(_, _) |
                Request::GetCommEventCounter | Request::GetCommEventLog |
                Request::ReportServerId | Request::ReadDeviceIdentification(_) |
                Request::Custom(_, _)
                => { Ok(()) },
        }
    }

    fn create_response<'b>(&self, buf: &'b mut Vec<u8, ModBusBufferSz>, req: Request<'_>) -> Result<Response<'b>, Exception> {
        match req {
            Request::WriteSingleCoil(a, _) => {
                Ok(Response::WriteSingleCoil(a) )
            },
            Request::WriteMultipleCoils(a, coils) => {
                Ok(Response::WriteMultipleCoils(a, u16::try_from(coils.len() ).unwrap() ) )
            },
            Request::WriteSingleRegister(a, _) => {
                let start_address = usize::from(a) - usize::from(self.holding_registers.address);
                let val = self.holding_registers.list[start_address];
                Ok(Response::WriteSingleRegister(a, val) )
            },
            Request::WriteMultipleRegisters(a, data) => {
                let start_address = usize::from(a) - usize::from(self.holding_registers.address);
                let end_address = start_address + data.len();
                if self.holding_registers.contains_addr(start_address..end_address) {
                    if let Ok(data) = Data::from_words(&self.holding_registers.list[start_address..end_address], buf) {
                        Ok(Response::WriteMultipleRegisters(a, u16::try_from(data.len() ).unwrap() ) )
                    }
                    else { Err(Exception::IllegalDataValue) }
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadCoils(a, quantity) => {
                let start_address = usize::from(a) - usize::from(self.coils.address);
                let end_address = start_address + usize::from(quantity);
                if self.coils.contains_addr(start_address..end_address) {
                    if let Ok(coils) = Coils::from_bools(&self.coils.list[start_address..end_address], buf) {
                        Ok(Response::ReadCoils(coils) )
                    }
                    else { Err(Exception::IllegalDataValue) }
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadDiscreteInputs(a, quantity) => {
                let start_address = usize::from(a) - usize::from(self.discrete_inputs.address);
                let end_address = start_address + usize::from(quantity);
                if self.discrete_inputs.contains_addr(start_address..end_address) {
                    if let Ok(coils) = Coils::from_bools(&self.discrete_inputs.list[start_address..end_address], buf) {
                        Ok(Response::ReadDiscreteInputs(coils) )
                    }
                    else { Err(Exception::IllegalDataValue) }
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadInputRegisters(a, quantity) => {
                let start_address = usize::from(a) - usize::from(self.input_registers.address);
                let end_address = start_address + usize::from(quantity);
                if self.input_registers.contains_addr(start_address..end_address) {
                    if let Ok(data) = Data::from_words(&self.input_registers.list[start_address..end_address], buf) {
                        Ok(Response::ReadInputRegisters(data) )
                    }
                    else { Err(Exception::IllegalDataValue) }
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadHoldingRegisters(a, quantity) | Request::ReadWriteMultipleRegisters(a, quantity, _, _) => {
                let start_address = usize::from(a) - usize::from(self.holding_registers.address);
                let end_address = start_address + usize::from(quantity);
                if self.holding_registers.contains_addr(start_address..end_address) {
                    if let Ok(data) = Data::from_words(&self.holding_registers.list[start_address..end_address], buf) {
                        Ok(Response::ReadHoldingRegisters(data) )
                    }
                    else { Err(Exception::IllegalDataValue) }
                }
                else { Err(Exception::IllegalDataAddress) }
            },
            Request::ReadDeviceIdentification(oid) => {
                match oid {
                    0 => {
                        if let Some(data) = self.vendor_id {
                            buf.extend_from_slice(data.as_bytes() ).unwrap();
                            Ok(buf)
                        }
                        else { Err(Exception::IllegalDataAddress) }
                    },
                    1 => {
                        if let Some(data) = self.product_id {
                            buf.extend_from_slice(data.as_bytes() ).unwrap();
                            Ok(buf)
                        }
                        else { Err(Exception::IllegalDataAddress) }
                    },
                    2 => {
                        if let Some(data) = self.major_minor_rev {
                            buf.extend_from_slice(data.as_bytes() ).unwrap();
                            Ok(buf)
                        }
                        else { Err(Exception::IllegalDataAddress) }
                    },
                    _ => Err(Exception::IllegalDataAddress),
                }.map(|buf| {
                    Response::ReadDeviceIdentification(oid, buf)
                })
            },
            Request::Diagnostics(_fncode, _data)  => { todo!() },

            /* Unsupported (For Now) */
            Request::Custom(_, _) | Request::ReadExceptionStatus |
                Request::GetCommEventCounter | Request::GetCommEventLog |
                Request::ReportServerId => { Err(Exception::IllegalDataAddress) },
        }
    }

    pub fn process_coils<F>(&self, mut f: F) where F: FnMut(usize, bool) {
        let start_address = usize::from(self.coils.address);
        for i in 0..self.coils.len() {
            let addr = start_address +i;
            let val = self.coils.list[i];
            f(addr, val);
        }
    }
    pub fn process_discrete_inputs<F>(&mut self, f: F) where F: Fn(usize) -> bool {
        let start_address = usize::from(self.discrete_inputs.address);
        for i in 0..self.discrete_inputs.len() {
            let addr = start_address +i;
            self.discrete_inputs.list[i] = f(addr);
        }
    }
    pub fn process_input_registers<F>(&mut self, f: F) where F: Fn(usize) -> u16 {
        let start_address = usize::from(self.input_registers.address);
        for i in 0..self.input_registers.len() {
            let addr = start_address +i;
            self.input_registers.set(addr, f(addr) ).unwrap();
        }
    }
    pub fn process_holding_registers<F>(&self, mut f: F) where F: FnMut(usize, u16) {
        let start_address = usize::from(self.holding_registers.address);
        for i in 0..self.holding_registers.len() {
            let addr = start_address +i;
            let val = self.holding_registers.list[i];
            f(addr, val);
        }
    }
}

pub struct Slave<'a, IrqPend> where IrqPend: Fn() {
    registers: Registers<'a>,
    irq_pend:  IrqPend,
}
impl<'a, IrqPend> Slave<'a, IrqPend> where IrqPend: Fn() {
    #[must_use]
    pub fn new(registers: Registers<'a>, irq_pend: IrqPend) -> Self {
        Self {
            registers,
            irq_pend,
        }
    }

    pub fn process<W, R, P, UNIT, ESW, ESR, EP>(&mut self, irq_ctx: &mut IrqContext<W, R, P, UNIT, ESW, ESR, EP>, now: Instant)
            where W: Write<UNIT, Error=ESW>,
                R: Read<UNIT, Error=ESR>,
                P: OutputPin<Error=EP>,
                UNIT: Copy +PartialEq + From<u8> + Into<u8> + core::fmt::Debug,
                ESW: core::fmt::Debug,
                ESR: core::fmt::Debug,
                EP: core::fmt::Debug {
        if let IrqStates::Receiving(ts_last) = irq_ctx.state {
            let frame_delay = now.duration_since(ts_last);

            //A delay of 3.5 characters should mean the end of a transmission
            if frame_delay >= irq_ctx.chardelay.to_t3_5().into() {
                irq_ctx.state = IrqStates::ReceiveWait;

                if irq_ctx.buffer.len() > 3 {
                    let mut tbuf = Vec::new();

                    match decode_request(&irq_ctx.buffer) {
                        Ok(Some(adu) ) => {
                            if adu.hdr.slave == irq_ctx.slave_id {
                                let fncode = FnCode::from(adu.pdu.0);
                                irq_ctx.state = IrqStates::Sending;

                                let response = ResponseAdu {
                                    hdr: Header { slave: 0x0 },
                                    pdu: ResponsePdu(self.registers.handle_request(adu.pdu.0).
                                        and_then(|_| self.registers.create_response(&mut tbuf, adu.pdu.0) ).map_err(|e|
                                            ExceptionResponse {
                                                function: fncode,
                                                exception: e,
                                            } ) )
                                };

                                let _ = encode_response(response, &mut irq_ctx.buffer).unwrap();

                                // Reverse the transmit buffer to simplify the irq handler
                                let slice: &mut [u8] = &mut irq_ctx.buffer;
                                slice.reverse();
                                (self.irq_pend)();
                            }
                        }
                        /* No valid Frame Found*/
                        Ok(None) | Err(_) => { },
                    }
                }
            }

            if irq_ctx.state == IrqStates::ReceiveWait {
                irq_ctx.buffer.clear();
            }
        }
        else if irq_ctx.state == IrqStates::Idle {
            let duration = now.duration_since(Instant(0));
            if duration > irq_ctx.chardelay.to_t3_5().into() {
                irq_ctx.state = IrqStates::ReceiveWait;
            }
        }
    }

    pub fn process_coils<F>(&self, f: F) where F: FnMut(usize, bool) {
        self.registers.process_coils(f);
    }
    pub fn process_discrete_inputs<F>(&mut self, f: F) where F: Fn(usize) -> bool {
        self.registers.process_discrete_inputs(f);
    }
    pub fn process_input_registers<F>(&mut self, f: F) where F: Fn(usize) -> u16 {
        self.registers.process_input_registers(f);
    }
    pub fn process_holding_registers<F>(&self, f: F) where F: FnMut(usize, u16) {
        self.registers.process_holding_registers(f);
    }
}
