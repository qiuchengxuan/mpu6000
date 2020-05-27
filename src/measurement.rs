use nalgebra::Vector3;

pub type Tuple3<T> = (T, T, T);

pub struct RawMeasurement {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub sensitive: f32,
}

impl Into<Tuple3<f32>> for RawMeasurement {
    fn into(self) -> Tuple3<f32> {
        (
            self.x as f32 / self.sensitive,
            self.y as f32 / self.sensitive,
            self.z as f32 / self.sensitive,
        )
    }
}

impl Into<Vector3<f32>> for RawMeasurement {
    fn into(self) -> Vector3<f32> {
        Vector3::new(
            self.x as f32 / self.sensitive,
            self.y as f32 / self.sensitive,
            self.z as f32 / self.sensitive,
        )
    }
}
