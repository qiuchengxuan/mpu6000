#[macro_use]
pub mod registers;

use core::cell::Cell;
use nalgebra::Vector3;

use registers::{
    AccelerometerSensitive, GyroSensitive, IntPinConfig, PowerManagement1, ProductId, Register,
    SignalPathReset,
};

pub trait Bus<E> {
    fn write(&self, bytes: &[u8]) -> Result<(), E>;
    fn write_read(&self, input: &[u8], output: &mut [u8]) -> Result<(), E>;
}

pub trait DelayMs {
    fn delay_ms(&self, ms: usize);
}

pub struct MPU6000<'a, E> {
    bus: Cell<&'a dyn Bus<E>>,
    delay: Cell<&'a dyn DelayMs>,
    accelerometer_sensitive: Cell<AccelerometerSensitive>,
    gyro_sensitive: Cell<GyroSensitive>,
    whoami: u8,
}

impl<'a, E> MPU6000<'a, E> {
    pub fn write_byte(&self, reg: Register, byte: u8) -> Result<(), E> {
        let bus = self.bus.get();
        bus.write(&[reg as u8, byte])?;
        Ok(())
    }

    pub fn read_byte(&self, reg: Register) -> Result<u8, E> {
        let mut buf: [u8; 1] = [0; 1];
        let bus = self.bus.get();
        bus.write_read(&[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    pub fn set_register_bit(&self, reg: Register, offset: usize, bit: bool) -> Result<(), E> {
        let mut value = self.read_byte(reg)?;
        if bit {
            value &= !(1u8 << offset);
        } else {
            value |= 1u8 << offset;
        }
        self.write_byte(reg, value)
    }

    pub fn read_into(&self, reg: Register, buf: &mut [u8]) -> Result<(), E> {
        let bus = self.bus.get();
        bus.write_read(&[reg as u8], buf)
    }

    pub fn set_slave_address(&mut self, address: u8) {
        self.whoami = address
    }

    pub(crate) fn read_vector3(&self, reg: Register) -> Result<Vector3<u32>, E> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_into(reg.into(), &mut buf)?;
        Ok(Vector3::<u32>::new(
            u16::from_be_bytes([buf[0], buf[1]]) as u32,
            u16::from_be_bytes([buf[2], buf[3]]) as u32,
            u16::from_be_bytes([buf[4], buf[5]]) as u32,
        ))
    }

    pub fn verify(&self) -> Result<bool, E> {
        let whoami = self.read_byte(Register::WhoAmI)?;
        let product_id = self.read_byte(Register::ProductId)?;
        Ok(whoami == self.whoami && ProductId::from(product_id) != ProductId::Unknown)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn new(bus: &'a dyn Bus<E>, delay: &'a dyn DelayMs) -> Self {
        MPU6000 {
            bus: Cell::new(bus),
            delay: Cell::new(delay),
            accelerometer_sensitive: Cell::new(AccelerometerSensitive::Sensitive16384),
            gyro_sensitive: Cell::new(GyroSensitive::Sensitive131),
            whoami: 0x68,
        }
    }

    /// Required when connected via SPI
    pub fn reset(&self) -> Result<(), E> {
        let reset_bit = PowerManagement1::DeviceReset as u8;
        self.write_byte(Register::PowerManagement1, reset_bit)?;
        self.delay.get().delay_ms(100);

        let value = SignalPathReset::TemperatureReset as u8
            | SignalPathReset::AccelerometerReset as u8
            | SignalPathReset::GyroReset as u8;
        self.write_byte(Register::SignalPathReset, value)?;
        self.delay.get().delay_ms(100);
        Ok(())
    }

    pub fn wake(&self) -> Result<(), E> {
        self.set_register_bit(Register::PowerManagement1, 6, false)?;
        self.delay.get().delay_ms(100);
        Ok(())
    }

    pub fn set_i2c_disable(&mut self, disable: bool) -> Result<(), E> {
        self.set_register_bit(Register::UserControl, 2, disable)
    }

    /// Sample Rate = Gyroscope Output Rate / (1 + divider)
    pub fn set_sample_rate_divider(&self, divider: u8) -> Result<(), E> {
        self.write_byte(Register::SampleRateDivider, divider)
    }

    pub fn set_int_pin_config(&self, pin_config: IntPinConfig, enable: bool) -> Result<(), E> {
        self.set_register_bit(Register::IntPinConfig, pin_config as usize, enable)
    }
}

impl<'a, E> MPU6000<'a, E> {
    /// Temperature in centi degrees celcius
    pub fn get_temperature(&self) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_into(Register::TemperatureHigh, &mut buf)?;
        Ok((u16::from_be_bytes(buf) as u32 * 100 / 340 + 3653) as u16)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn set_gyro_sensitive(&self, sensitive: GyroSensitive) -> Result<(), E> {
        self.write_byte(Register::GyroConfig, (sensitive as u8) << 3)?;
        self.gyro_sensitive.set(sensitive);
        Ok(())
    }

    /// Gyro readings in centi degree/s
    pub fn get_gyro(&self) -> Result<Vector3<u32>, E> {
        let vector = self.read_vector3(Register::GyroXHigh)?;
        let sensitive = self.gyro_sensitive.get() as u32;
        Ok(vector * 100 / sensitive)
    }
}

impl<'a, E> MPU6000<'a, E> {
    pub fn set_accelerometer_sensitive(&self, sensitive: AccelerometerSensitive) -> Result<(), E> {
        self.write_byte(Register::AccelerometerConfig, (sensitive as u8) << 3)?;
        self.accelerometer_sensitive.set(sensitive);
        Ok(())
    }

    /// Accelerometer readings in cm/s^2
    pub fn get_accelerator(&self) -> Result<Vector3<u32>, E> {
        let vector = self.read_vector3(Register::AccelerometerXHigh)?;
        let sensitive = self.accelerometer_sensitive.get() as u32;
        Ok(vector * 100 / sensitive)
    }
}

mod test {
    use crate::{Bus, DelayMs};

    #[derive(Debug)]
    enum Error {}

    struct StubBus {}
    impl Bus<Error> for StubBus {
        fn write(&self, _bytes: &[u8]) -> Result<(), Error> {
            Ok(())
        }

        fn write_read(&self, _input: &[u8], _output: &mut [u8]) -> Result<(), Error> {
            Ok(())
        }
    }

    struct Nodelay {}
    impl DelayMs for Nodelay {
        fn delay_ms(&self, _ms: usize) {}
    }

    #[test]
    fn test_functional() -> Result<(), Error> {
        extern crate std;

        use crate::registers::{AccelerometerSensitive, GyroSensitive};
        use crate::MPU6000;

        let bus = StubBus {};
        let delay = Nodelay {};
        let mpu6000 = MPU6000::new(&bus, &delay);
        mpu6000.reset()?;
        mpu6000.wake()?;
        mpu6000.set_accelerometer_sensitive(accelerometer_sensitive!(+/-16g, 2048/LSB))?;
        mpu6000.set_gyro_sensitive(gyro_sensitive!(+/-2000dps, 16.4LSB/dps))?;
        Ok(())
    }
}
