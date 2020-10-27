use embedded_hal::blocking::spi::{self};
use embedded_hal::digital::v2::OutputPin;

use super::registers::Register;

pub trait Bus {
    type Error;
    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error>;
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error>;
    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Debug)]
pub enum SpiError<WE, TE, OE> {
    WriteError(WE),
    TransferError(TE),
    OutputPinError(OE),
}

pub trait DelayNs<T> {
    fn delay_ns(&mut self, ns: T);
}

pub struct SpiBus<SPI, CS, D> {
    spi: SPI,
    cs: CS,
    delay: D,
}

impl<'a, SPI, CS, D> SpiBus<SPI, CS, D>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    CS: OutputPin,
    D: DelayNs<u16>,
{
    pub fn new(spi: SPI, cs: CS, delay: D) -> Self {
        Self { spi, cs, delay }
    }

    pub fn free(self) -> (SPI, CS, D) {
        (self.spi, self.cs, self.delay)
    }
}

impl<WE, TE, OE, SPI, CS, D> SpiBus<SPI, CS, D>
where
    SPI: spi::Transfer<u8, Error = TE> + spi::Write<u8, Error = WE>,
    CS: OutputPin<Error = OE>,
{
    fn chip_select(&mut self, select: bool) -> Result<(), SpiError<WE, TE, OE>> {
        if select { self.cs.set_low() } else { self.cs.set_high() }
            .map_err(|e| SpiError::OutputPinError(e))
    }
}

impl<WE, TE, OE, SPI, CS, D> Bus for SpiBus<SPI, CS, D>
where
    SPI: spi::Transfer<u8, Error = TE> + spi::Write<u8, Error = WE>,
    CS: OutputPin<Error = OE>,
    D: DelayNs<u16>,
{
    type Error = SpiError<WE, TE, OE>;

    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.chip_select(true)?;
        self.delay.delay_ns(100);
        let result = self.spi.write(&[reg as u8, value]);
        self.chip_select(false)?;
        self.delay.delay_ns(500);
        result.map_err(|e| SpiError::WriteError(e))
    }

    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut value = [0u8];
        self.chip_select(true)?;
        self.delay.delay_ns(100);
        self.spi.write(&[reg as u8 | 0x80]).map_err(|e| SpiError::WriteError(e))?;
        self.spi.transfer(&mut value).map_err(|e| SpiError::TransferError(e))?;
        self.chip_select(false)?;
        self.delay.delay_ns(500);
        Ok(value[0])
    }

    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), Self::Error> {
        self.chip_select(true)?;
        self.delay.delay_ns(100);
        self.spi.write(&[reg as u8 | 0x80]).map_err(|e| SpiError::WriteError(e))?;
        self.spi.transfer(output).map_err(|e| SpiError::TransferError(e))?;
        self.chip_select(false)?;
        self.delay.delay_ns(500);
        Ok(())
    }
}
