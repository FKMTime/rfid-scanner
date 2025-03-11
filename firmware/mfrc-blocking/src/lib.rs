#![no_std]

use consts::{PCDErrorCode, Uid, UidSize};
use embedded_hal::spi::Operation;

pub mod consts;
pub mod debug;
pub mod mifare;
pub mod pcd;
pub mod picc;

pub struct MFRC522<S>
where
    S: embedded_hal::spi::SpiDevice,
{
    spi: S,
    get_current_time: fn() -> u64,
}

impl<S> MFRC522<S>
where
    S: embedded_hal::spi::SpiDevice,
{
    pub fn new(spi: S, get_current_time: fn() -> u64) -> Self {
        Self {
            spi,
            get_current_time,
        }
    }

    pub fn sleep(&self, time_ms: u64) {
        let start_time = (self.get_current_time)(); // microseconds
        while (self.get_current_time)() - start_time < time_ms * 1_000 {}
    }

    pub fn get_card(&mut self, size: UidSize) -> Result<Uid, PCDErrorCode> {
        let mut uid = Uid {
            size: size.to_byte(),
            sak: 0,
            uid_bytes: [0; 10],
        };

        self.picc_select(&mut uid, 0)?;
        Ok(uid)
    }

    pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        self.spi
            .transaction(&mut [Operation::Write(&[reg << 1, val])])?;

        Ok(())
    }

    pub fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.spi.transaction(&mut [
            Operation::Write(&[reg << 1]),
            Operation::Write(&values[..count]),
        ])?;

        Ok(())
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        let mut read = [0; 1];
        self.spi.transaction(&mut [
            Operation::Write(&[(reg << 1) | 0x80]),
            Operation::Transfer(&mut read, &[0]),
        ])?;

        Ok(read[0])
    }

    pub fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> Result<(), PCDErrorCode> {
        if count == 0 {
            return Ok(());
        }

        let addr = 0x80 | (reg << 1);
        let first_out_byte = output_buff[0];
        output_buff[..count - 1].fill(addr);
        output_buff[count - 1] = 0;

        self.spi.transaction(&mut [
            Operation::Write(&[addr]),
            Operation::TransferInPlace(&mut output_buff[..count]),
        ])?;

        if rx_align > 0 {
            let mask = (0xFF << rx_align) & 0xFF;
            output_buff[0] = (first_out_byte & !mask) | (output_buff[0] & mask);
        }

        Ok(())
    }
}

#[inline(always)]
pub fn tif<T>(expr: bool, true_val: T, false_val: T) -> T {
    if expr {
        true_val
    } else {
        false_val
    }
}
