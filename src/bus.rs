use core::slice::from_mut;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::{i2c, spi};
use embedded_hal::digital::v2::OutputPin;

use super::registers::Register;

pub enum SpiError<WE, TE, OE> {
    WriteError(WE),
    TransferError(TE),
    OutputPinError(OE),
}

pub struct SpiBus<BUS, CS, DELAY> {
    bus: BUS,
    cs: CS,
    delay: DELAY,
}

pub trait RegAccess {
    type Error;
    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error>;
    fn read(&mut self, reg: Register) -> Result<u8, Self::Error>;
    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), Self::Error>;
}

impl<WE, TE, OE, SPI, CS, DELAY> SpiBus<SPI, CS, DELAY>
where
    SPI: spi::Write<u8, Error = WE> + spi::Transfer<u8, Error = TE>,
    CS: OutputPin<Error = OE>,
    DELAY: DelayUs<u8>,
{
    pub fn new(spi: SPI, cs: CS, delay: DELAY) -> Self {
        Self { bus: spi, cs, delay }
    }

    fn chip_select(&mut self, select: bool) -> Result<(), SpiError<WE, TE, OE>> {
        if select { self.cs.set_low() } else { self.cs.set_high() }
            .map_err(|e| SpiError::OutputPinError(e))
    }
}

impl<SPI, CS, DELAY> SpiBus<SPI, CS, DELAY> {
    pub fn free(self) -> (SPI, CS, DELAY) {
        (self.bus, self.cs, self.delay)
    }
}

impl<WE, TE, OE, SPI, CS, DELAY> RegAccess for SpiBus<SPI, CS, DELAY>
where
    SPI: spi::Write<u8, Error = WE> + spi::Transfer<u8, Error = TE>,
    CS: OutputPin<Error = OE>,
    DELAY: DelayUs<u8>,
{
    type Error = SpiError<WE, TE, OE>;

    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.chip_select(true)?;
        self.delay.delay_us(1);
        let result = self.bus.write(&[reg as u8, value]);
        self.chip_select(false)?;
        self.delay.delay_us(1);
        result.map_err(|e| Self::Error::WriteError(e))
    }

    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut value = 0u8;
        self.chip_select(true)?;
        self.delay.delay_us(1);
        self.bus.write(&[reg as u8 | 0x80]).map_err(|e| Self::Error::WriteError(e))?;
        self.bus.transfer(from_mut(&mut value)).map_err(|e| Self::Error::TransferError(e))?;
        self.chip_select(false)?;
        self.delay.delay_us(1);
        Ok(value)
    }

    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), Self::Error> {
        self.chip_select(true)?;
        self.delay.delay_us(1);
        self.bus.write(&[reg as u8 | 0x80]).map_err(|e| Self::Error::WriteError(e))?;
        self.bus.transfer(output).map_err(|e| Self::Error::TransferError(e))?;
        self.chip_select(false)?;
        self.delay.delay_us(1);
        Ok(())
    }
}

pub struct I2cBus<BUS, DELAY> {
    bus: BUS,
    address: u8,
    delay: DELAY,
}

impl<WE, RE, I2C, DELAY> I2cBus<I2C, DELAY>
where
    I2C: i2c::Write<Error = WE> + i2c::Read<Error = RE>,
{
    pub fn i2c(i2c: I2C, address: u8, delay: DELAY) -> Self {
        Self { bus: i2c, address, delay }
    }
}

impl<I2C, DELAY> I2cBus<I2C, DELAY> {
    pub fn free(self) -> (I2C, DELAY) {
        (self.bus, self.delay)
    }
}

pub enum I2CError<RE, WE> {
    ReadError(RE),
    WriteError(WE),
}

impl<RE, WE, I2C, DELAY> RegAccess for I2cBus<I2C, DELAY>
where
    I2C: i2c::Read<Error = RE> + i2c::Write<Error = WE>,
    DELAY: DelayUs<u8>,
{
    type Error = I2CError<RE, WE>;

    fn write(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address, &[reg as u8, value]).map_err(|e| Self::Error::WriteError(e))
    }

    fn read(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut value = 0u8;
        self.bus
            .write(self.address, &[reg as u8 | 0x80])
            .map_err(|e| Self::Error::WriteError(e))?;

        self.bus.read(self.address, from_mut(&mut value)).map_err(|e| Self::Error::ReadError(e))?;
        Ok(value)
    }

    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), Self::Error> {
        self.bus
            .write(self.address, &[reg as u8 | 0x80])
            .map_err(|e| Self::Error::WriteError(e))?;
        self.bus.read(self.address, output).map_err(|e| Self::Error::ReadError(e))?;
        Ok(())
    }
}
