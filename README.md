MPU6000
=======

Not tested yet

[DATASHEET][DATASHEET] | [REGISTER MAP][REGISTER_MAP]

[DATASHEET]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
[REGISTER_MAP]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

Example
=======

```rust
use mpu6000::{Bus, DelayMs, MPU6000};
use mpu6000::registers::{AccelerometerSensitive, GyroSensitive};
...
// impl Bus...
// impl DelayMs...
...
let mpu6000 = MPU6000::new(&bus, delay);
mpu6000.reset()?;
mpu6000.wake()?;
mpu6000.set_accelerometer_sensitive(accelerometer_sensitive!(+/-16g, 2048/LSB))?;
mpu6000.set_gyro_sensitive(gyro_sensitive!(+/-2000dps, 16.4LSB/dps))?;
```
