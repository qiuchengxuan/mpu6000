#![no_std]
#[macro_use]
pub mod registers;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{self};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity};
use nalgebra::Vector3;

use registers::{
    AccelerometerSensitive, GyroSensitive, IntPinConfig, PowerManagement1, ProductId, Register,
    SignalPathReset,
};

pub const SPI_MODE: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

#[derive(Default)]
pub struct FifoEnable {
    pub temperature: bool,
    pub x_g_force: bool,
    pub y_g_force: bool,
    pub z_g_force: bool,
    pub acceleration: bool,
    pub slave2: bool,
    pub slave1: bool,
    pub slave0: bool,
}

impl Into<u8> for FifoEnable {
    fn into(self) -> u8 {
        (self.temperature as u8) << 7
            | (self.x_g_force as u8) << 6
            | (self.y_g_force as u8) << 5
            | (self.z_g_force as u8) << 4
            | (self.acceleration as u8) << 3
            | (self.slave2 as u8) << 2
            | (self.slave1 as u8) << 1
            | (self.slave0 as u8) << 0
    }
}

pub trait Bus<E> {
    fn write(&mut self, reg: Register, value: u8) -> Result<(), E>;
    fn read(&mut self, reg: Register) -> Result<u8, E>;
    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), E>;
}

pub enum SpiError<WE, TE, OE> {
    WriteError(WE),
    TransferError(TE),
    OutputPinError(OE),
}

pub struct SpiBus<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI: spi::Transfer<u8> + spi::Write<u8>, CS: OutputPin> SpiBus<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }
}

impl<WE, TE, OE, SPI, CS> SpiBus<SPI, CS>
where
    SPI: spi::Transfer<u8, Error = TE> + spi::Write<u8, Error = WE>,
    CS: OutputPin<Error = OE>,
{
    fn chip_select(&mut self, select: bool) -> Result<(), SpiError<WE, TE, OE>> {
        if select {
            self.cs.set_low()
        } else {
            self.cs.set_high()
        }
        .map_err(|e| SpiError::OutputPinError(e))
    }
}

impl<WE, TE, OE, SPI, CS> Bus<SpiError<WE, TE, OE>> for SpiBus<SPI, CS>
where
    SPI: spi::Transfer<u8, Error = TE> + spi::Write<u8, Error = WE>,
    CS: OutputPin<Error = OE>,
{
    fn write(&mut self, reg: Register, value: u8) -> Result<(), SpiError<WE, TE, OE>> {
        self.chip_select(true)?;
        let result = self.spi.write(&[reg as u8, value]);
        self.chip_select(false)?;
        result.map_err(|e| SpiError::WriteError(e))
    }

    fn read(&mut self, reg: Register) -> Result<u8, SpiError<WE, TE, OE>> {
        let mut value = [0u8];
        self.chip_select(true)?;
        self.spi
            .write(&[reg as u8 | 0x80])
            .map_err(|e| SpiError::WriteError(e))?;
        self.spi
            .transfer(&mut value)
            .map_err(|e| SpiError::TransferError(e))?;
        self.chip_select(false)?;
        Ok(value[0])
    }

    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), SpiError<WE, TE, OE>> {
        self.chip_select(true)?;
        self.spi
            .write(&[reg as u8 | 0x80])
            .map_err(|e| SpiError::WriteError(e))?;
        self.spi
            .transfer(output)
            .map_err(|e| SpiError::TransferError(e))?;
        self.chip_select(false)?;
        Ok(())
    }
}

pub struct MPU6000<'a, E> {
    bus: &'a mut dyn Bus<E>,
    accelerometer_sensitive: AccelerometerSensitive,
    gyro_sensitive: GyroSensitive,
    whoami: u8,
}

impl<'a, E> MPU6000<'a, E> {
    pub fn set_register_bit(&mut self, reg: Register, offset: usize, bit: bool) -> Result<(), E> {
        let mut value = self.bus.read(reg)?;
        if bit {
            value &= !(1u8 << offset);
        } else {
            value |= 1u8 << offset;
        }
        self.bus.write(reg, value)
    }

    pub fn read_into(&mut self, reg: Register, buf: &mut [u8]) -> Result<(), E> {
        self.bus.reads(reg, buf)
    }

    pub fn set_slave_address(&mut self, address: u8) {
        self.whoami = address
    }

    fn read_vector3(&mut self, reg: Register) -> Result<Vector3<u32>, E> {
        let mut buf = [0u8; 6];
        self.read_into(reg.into(), &mut buf)?;
        Ok(Vector3::<u32>::new(
            u16::from_be_bytes([buf[0], buf[1]]) as u32,
            u16::from_be_bytes([buf[2], buf[3]]) as u32,
            u16::from_be_bytes([buf[4], buf[5]]) as u32,
        ))
    }

    pub fn enable_fifo(&mut self, fifo_enable: FifoEnable) -> Result<(), E> {
        let value: u8 = fifo_enable.into();
        self.bus.write(Register::FifoEnable, value)
    }

    pub fn enable_fifo_buffer(&mut self) -> Result<(), E> {
        let value = self.bus.read(Register::UserControl)?;
        self.bus.write(Register::UserControl, value | 1 << 6)
    }

    pub fn whoami(&mut self) -> Result<u8, E> {
        self.bus.read(Register::WhoAmI)
    }

    pub fn product_id(&mut self) -> Result<u8, E> {
        self.bus.read(Register::ProductId)
    }

    pub fn verify(&mut self) -> Result<bool, E> {
        Ok(self.whoami()? == self.whoami && self.product_id()? != ProductId::Unknown as u8)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn new(bus: &'a mut dyn Bus<E>) -> Self {
        MPU6000 {
            bus,
            accelerometer_sensitive: AccelerometerSensitive::Sensitive16384,
            gyro_sensitive: GyroSensitive::Sensitive131,
            whoami: 0x68,
        }
    }

    /// Required when connected via BUS
    pub fn reset<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        let reset_bit = PowerManagement1::DeviceReset as u8;
        self.bus.write(Register::PowerManagement1, reset_bit)?;
        delay.delay_ms(150u8.into());

        let value = SignalPathReset::TemperatureReset as u8
            | SignalPathReset::AccelerometerReset as u8
            | SignalPathReset::GyroReset as u8;
        self.bus.write(Register::SignalPathReset, value)?;
        delay.delay_ms(150u8.into());
        Ok(())
    }

    pub fn wake<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        self.set_register_bit(Register::PowerManagement1, 6, false)?;
        delay.delay_ms(150u8.into());
        Ok(())
    }

    pub fn set_i2c_disable(&mut self, disable: bool) -> Result<(), E> {
        self.set_register_bit(Register::UserControl, 2, disable)
    }

    /// Sample Rate = Gyroscope Output Rate / (1 + divider)
    pub fn set_sample_rate_divider(&mut self, divider: u8) -> Result<(), E> {
        self.bus.write(Register::SampleRateDivider, divider)
    }

    pub fn set_int_pin_config(&mut self, pin_config: IntPinConfig, enable: bool) -> Result<(), E> {
        self.set_register_bit(Register::IntPinConfig, pin_config as usize, enable)
    }
}

impl<'a, E> MPU6000<'a, E> {
    /// Temperature in centi degrees celcius
    pub fn get_temperature(&mut self) -> Result<u16, E> {
        let mut buf = [0u8; 2];
        self.read_into(Register::TemperatureHigh, &mut buf)?;
        Ok((u16::from_be_bytes(buf) as u32 * 100 / 340 + 3653) as u16)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn set_gyro_sensitive(&mut self, sensitive: GyroSensitive) -> Result<(), E> {
        self.bus
            .write(Register::GyroConfig, (sensitive as u8) << 3)?;
        self.gyro_sensitive = sensitive;
        Ok(())
    }

    /// Gyro readings in centi degree/s
    pub fn get_gyro(&mut self) -> Result<Vector3<u32>, E> {
        let vector = self.read_vector3(Register::GyroXHigh)?;
        let sensitive = self.gyro_sensitive as u32;
        Ok(vector * 100 / sensitive)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn set_accelerometer_sensitive(
        &mut self,
        sensitive: AccelerometerSensitive,
    ) -> Result<(), E> {
        self.bus
            .write(Register::AccelerometerConfig, (sensitive as u8) << 3)?;
        self.accelerometer_sensitive = sensitive;
        Ok(())
    }

    /// Accelerometer readings in cm/s^2
    pub fn get_accelerator(&mut self) -> Result<Vector3<u32>, E> {
        let vector = self.read_vector3(Register::AccelerometerXHigh)?;
        let sensitive = self.accelerometer_sensitive as u32;
        Ok(vector * 100 / sensitive)
    }
}

mod test {
    use embedded_hal::blocking::spi::{Transfer, Write};
    use embedded_hal::digital::v2::OutputPin;

    use crate::DelayMs;

    struct StubSPI {}

    impl Write<u8> for StubSPI {
        type Error = &'static str;
        fn write(&mut self, _bytes: &[u8]) -> Result<(), &'static str> {
            Ok(())
        }
    }

    impl Transfer<u8> for StubSPI {
        type Error = &'static str;
        fn transfer<'w>(&mut self, bytes: &'w mut [u8]) -> Result<&'w [u8], &'static str> {
            Ok(bytes)
        }
    }

    struct StubOutputPin {}
    impl OutputPin for StubOutputPin {
        type Error = &'static str;
        fn set_high(&mut self) -> Result<(), &'static str> {
            Ok(())
        }

        fn set_low(&mut self) -> Result<(), &'static str> {
            Ok(())
        }
    }

    struct Nodelay {}
    impl DelayMs<u8> for Nodelay {
        fn delay_ms(&mut self, _ms: u8) {}
    }

    #[test]
    fn test_functional() {
        extern crate std;

        use crate::registers::{AccelerometerSensitive, GyroSensitive};
        use crate::{SpiBus, MPU6000};

        let spi = StubSPI {};
        let output_pin = StubOutputPin {};
        let mut delay = Nodelay {};
        let mut spi_bus: SpiBus<StubSPI, StubOutputPin> = SpiBus::new(spi, output_pin);
        let mut mpu6000 = MPU6000::new(&mut spi_bus);
        let _ = mpu6000.reset(&mut delay);
        let _ = mpu6000.wake(&mut delay);
        let _ = mpu6000.set_accelerometer_sensitive(accelerometer_sensitive!(+/-16g, 2048/LSB));
        let _ = mpu6000.set_gyro_sensitive(gyro_sensitive!(+/-2000dps, 16.4LSB/dps));
    }
}
