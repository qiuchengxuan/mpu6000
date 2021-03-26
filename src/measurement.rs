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
        Self(array[0], array[1], array[2])
    }
}

impl From<&[u8]> for Acceleration {
    fn from(bytes: &[u8]) -> Self {
        let mut array: [i16; 3] = [0i16; 3];
        for (i, bytes) in bytes[..6].chunks(2).enumerate() {
            array[i] = i16::from_be_bytes([bytes[0], bytes[1]]);
        }
        Self(array[0], array[1], array[2])
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
        Self(array[0], array[1], array[2])
    }
}

impl From<&[u8]> for Gyro {
    fn from(bytes: &[u8]) -> Self {
        let mut array: [i16; 3] = [0i16; 3];
        for (i, bytes) in bytes[..6].chunks(2).enumerate() {
            array[i] = i16::from_be_bytes([bytes[0], bytes[1]]);
        }
        Self(array[0], array[1], array[2])
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Temperature(pub i16);

impl From<&[u8]> for Temperature {
    fn from(bytes: &[u8]) -> Self {
        Self(i16::from_be_bytes([bytes[0], bytes[1]]))
    }
}

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
