use crate::registers::{AccelerometerSensitive, GyroSensitive};

pub struct Acceleration(pub i16, pub i16, pub i16);

impl Acceleration {
    pub fn into_f32(self, sensitive: AccelerometerSensitive) -> (f32, f32, f32) {
        let div: f32 = sensitive.into();
        (self.0 as f32 / div, self.1 as f32 / div, self.2 as f32 / div)
    }
}

impl From<&[i16]> for Acceleration {
    fn from(array: &[i16]) -> Self {
        Self(i16::from_be(array[0]), i16::from_be(array[1]), i16::from_be(array[2]))
    }
}

impl From<&[u8]> for Acceleration {
    fn from(bytes: &[u8]) -> Self {
        let array: &[i16] = unsafe { core::mem::transmute(bytes) };
        array.into()
    }
}

pub struct Gyro(pub i16, pub i16, pub i16);

impl Gyro {
    pub fn into_f32(self, sensitive: GyroSensitive) -> (f32, f32, f32) {
        let div: f32 = sensitive.into();
        (self.0 as f32 / div, self.1 as f32 / div, self.2 as f32 / div)
    }
}

impl From<&[i16]> for Gyro {
    fn from(array: &[i16]) -> Self {
        Self(i16::from_be(array[0]), i16::from_be(array[1]), i16::from_be(array[2]))
    }
}

impl From<&[u8]> for Gyro {
    fn from(bytes: &[u8]) -> Self {
        let array: &[i16] = unsafe { core::mem::transmute(bytes) };
        array.into()
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Temperature(pub i16);

impl Temperature {
    pub fn new(high: u8, low: u8) -> Self {
        Self((high as i16) << 8 | low as i16)
    }

    pub fn centi_celcius(self) -> i16 {
        (self.0 as i32 * 100 / 340 + 3653) as i16
    }
}

impl Into<f32> for Temperature {
    fn into(self) -> f32 {
        self.0 as f32 / 340.0 + 365.3
    }
}
