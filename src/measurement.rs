use core::convert::{TryFrom, TryInto};

use fixed_point::FixedPoint;

use crate::registers::{AccelerometerSensitive, GyroSensitive};

/// By dividing sensitive(`LSB/g`) will get acceleration in g(9.8m²/s)
pub struct Acceleration(pub [i16; 3]);

impl core::ops::Div<AccelerometerSensitive> for Acceleration {
    type Output = [f32; 3];

    fn div(self, sensitive: AccelerometerSensitive) -> Self::Output {
        let div: f32 = sensitive.into();
        [self.0[0] as f32 / div, self.0[1] as f32 / div, self.0[2] as f32 / div]
    }
}

impl From<&[u8; 6]> for Acceleration {
    fn from(bytes: &[u8; 6]) -> Self {
        let mut array: [i16; 3] = [0i16; 3];
        for i in 0..3 {
            array[i] = i16::from_be_bytes([bytes[i * 2], bytes[i * 2 + 1]]);
        }
        Self(array)
    }
}

impl TryFrom<&[i16]> for Acceleration {
    type Error = ();
    fn try_from(slice: &[i16]) -> Result<Self, Self::Error> {
        slice.try_into().map(|array| Self(array)).map_err(|_| ())
    }
}

impl TryFrom<&[u8]> for Acceleration {
    type Error = ();
    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let array: &[u8; 6] = bytes.try_into().map_err(|_| ())?;
        Ok(array.into())
    }
}

/// By dividing sensitive(`LSB/(°/s)`) will get angular velocity °/s
pub struct Gyro(pub [i16; 3]);

impl core::ops::Div<GyroSensitive> for Gyro {
    type Output = [f32; 3];

    fn div(self, sensitive: GyroSensitive) -> Self::Output {
        let div: f32 = sensitive.into();
        [self.0[0] as f32 / div, self.0[1] as f32 / div, self.0[2] as f32 / div]
    }
}

impl From<&[u8; 6]> for Gyro {
    fn from(bytes: &[u8; 6]) -> Self {
        let mut array: [i16; 3] = [0i16; 3];
        for i in 0..3 {
            array[i] = i16::from_be_bytes([bytes[i * 2], bytes[i * 2 + 1]]);
        }
        Self(array)
    }
}

impl TryFrom<&[i16]> for Gyro {
    type Error = ();
    fn try_from(slice: &[i16]) -> Result<Self, ()> {
        slice.try_into().map(|array| Self(array)).map_err(|_| ())
    }
}

impl TryFrom<&[u8]> for Gyro {
    type Error = ();
    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let array: &[u8; 6] = bytes.try_into().map_err(|_| ())?;
        Ok(array.into())
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Temperature(pub i16);
impl From<&[u8; 2]> for Temperature {
    fn from(bytes: &[u8; 2]) -> Self {
        Self(i16::from_be_bytes([bytes[0], bytes[1]]))
    }
}

impl TryFrom<&[u8]> for Temperature {
    type Error = ();
    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let array: &[u8; 2] = bytes.try_into().map_err(|_| ())?;
        Ok(array.into())
    }
}

impl Temperature {
    pub fn new(high: u8, low: u8) -> Self {
        Self((high as i16) << 8 | low as i16)
    }

    pub fn celcius(self) -> FixedPoint<i16, 2> {
        FixedPoint((self.0 as i32 * 100 / 340 + 3653) as i16)
    }
}

impl Into<f32> for Temperature {
    fn into(self) -> f32 {
        self.0 as f32 / 340.0 + 365.3
    }
}
