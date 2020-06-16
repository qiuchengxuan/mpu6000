use nalgebra::Vector3;

pub type Tuple3<T> = (T, T, T);

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Measurement<S> {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub sensitive: S,
}

impl<S: Copy> Measurement<S> {
    pub fn from_array(array: &[i16], sensitive: S) -> Self {
        Self {
            x: i16::from_be(array[0]),
            y: i16::from_be(array[1]),
            z: i16::from_be(array[2]),
            sensitive,
        }
    }

    pub fn from_bytes(bytes: &[u8], sensitive: S) -> Self {
        Self {
            x: i16::from_be_bytes([bytes[0], bytes[1]]),
            y: i16::from_be_bytes([bytes[2], bytes[3]]),
            z: i16::from_be_bytes([bytes[4], bytes[5]]),
            sensitive,
        }
    }

    pub fn average(a: &Self, b: &Self) -> Self {
        Self {
            x: ((a.x + b.x) as i32 / 2) as i16,
            y: ((a.y + b.y) as i32 / 2) as i16,
            z: ((a.z + b.z) as i32 / 2) as i16,
            sensitive: b.sensitive,
        }
    }

    pub fn calibrated(&mut self, calibration: &Self) {
        self.x -= calibration.x;
        self.y -= calibration.y;
        self.z -= calibration.z;
    }
}

impl<S: Into<f32>> Into<Tuple3<f32>> for Measurement<S> {
    fn into(self) -> Tuple3<f32> {
        let sensitive: f32 = self.sensitive.into();
        (self.x as f32 / sensitive, self.y as f32 / sensitive, self.z as f32 / sensitive)
    }
}

impl<S: Into<f32>> Into<Vector3<f32>> for Measurement<S> {
    fn into(self) -> Vector3<f32> {
        let sensitive: f32 = self.sensitive.into();
        Vector3::new(
            self.x as f32 / sensitive,
            self.y as f32 / sensitive,
            self.z as f32 / sensitive,
        )
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
