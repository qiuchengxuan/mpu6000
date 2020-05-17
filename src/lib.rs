#![no_std]
#[macro_use]
pub mod registers;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{self};
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

pub trait Bus<E> {
    fn write(&mut self, reg: Register, value: u8) -> Result<(), E>;
    fn read(&mut self, reg: Register) -> Result<u8, E>;
    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), E>;
}

pub struct SpiBus<SPI: spi::Transfer<u8> + spi::Write<u8>>(SPI);

impl<SPI: spi::Transfer<u8> + spi::Write<u8>> From<SPI> for SpiBus<SPI> {
    fn from(spi: SPI) -> SpiBus<SPI> {
        SpiBus(spi)
    }
}

impl<E, SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>> Bus<E> for SpiBus<SPI> {
    fn write(&mut self, reg: Register, value: u8) -> Result<(), E> {
        self.0.write(&[reg as u8, value])
    }

    fn read(&mut self, reg: Register) -> Result<u8, E> {
        let mut value = [0u8];
        self.0.write(&[reg as u8 | 0x80])?;
        self.0.transfer(&mut value)?;
        Ok(value[0])
    }

    fn reads(&mut self, reg: Register, output: &mut [u8]) -> Result<(), E> {
        self.0.write(&[reg as u8 | 0x80])?;
        self.0.transfer(output)?;
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

    /// Required when connected via SPI
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

    struct Nodelay {}
    impl DelayMs<u8> for Nodelay {
        fn delay_ms(&mut self, _ms: u8) {}
    }

    #[test]
    fn test_functional() -> Result<(), &'static str> {
        extern crate std;

        use crate::registers::{AccelerometerSensitive, GyroSensitive};
        use crate::{SpiBus, MPU6000};

        let bus = StubSPI {};
        let mut delay = Nodelay {};
        let mut spi_bus: SpiBus<StubSPI> = bus.into();
        let mut mpu6000 = MPU6000::new(&mut spi_bus);
        mpu6000.reset(&mut delay)?;
        mpu6000.wake(&mut delay)?;
        mpu6000.set_accelerometer_sensitive(accelerometer_sensitive!(+/-16g, 2048/LSB))?;
        mpu6000.set_gyro_sensitive(gyro_sensitive!(+/-2000dps, 16.4LSB/dps))?;
        Ok(())
    }
}
