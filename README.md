MPU6000
=======

[DATASHEET][DATASHEET] | [REGISTER MAP][REGISTER_MAP]

[DATASHEET]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
[REGISTER_MAP]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

Example
=======

```rust
use embedded_hal::blocking::delay::DelayMs;
use mpu6000::{Bus, DelayMs, MPU6000};
use mpu6000::registers::{AccelerometerSensitive, GyroSensitive};
...
// create SPI that implemented embedded_hal::blocking::spi::Write + Transfer
// create OutputPin as Chip Select that impelmented embedded_hal::digital::v2::OutputPin
// create Delay that implemented embedded_hal::blocking::delay::DelayMs
...
let mut spi_bus: SpiBus<SPI, OutputPin> = SpiBus::new(spi, output_pin);
let mpu6000 = MPU6000::new(&mut spi_bus);
mpu6000.reset(&delay)?;
mpu6000.wake(&delay)?;
mpu6000.set_accelerometer_sensitive(accelerometer_sensitive!(+/-16g, 2048/LSB))?;
mpu6000.set_gyro_sensitive(gyro_sensitive!(+/-2000dps, 16.4LSB/dps))?;
```
