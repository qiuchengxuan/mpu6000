//! All constants used in the driver, mostly register addresses

pub enum ClockSelection {
    Internal8Mhz = 0,
    PLLWithXAxisGyroRef = 1,
    PLLWithYAxisGyroRef = 2,
    PLLWithZAxisGyroRef = 3,
    PLLWithExternal32_768KHZ = 4,
    PLLWithExternal19_2MHZ = 5,
    StopClock = 7,
}

pub enum PowerManagement1 {
    DeviceReset = 1 << 7,
}

pub enum SignalPathReset {
    GyroReset = 1 << 2,
    AccelerometerReset = 1 << 1,
    TemperatureReset = 1,
}

#[derive(PartialEq, Debug, Copy, Clone)]
pub enum AccelerometerSensitive {
    Sensitive16384 = 0,
    Sensitive8192 = 1,
    Sensitive4096 = 2,
    Sensitive2048 = 3,
}

impl Into<f32> for AccelerometerSensitive {
    fn into(self) -> f32 {
        match self {
            AccelerometerSensitive::Sensitive16384 => 16384.0,
            AccelerometerSensitive::Sensitive8192 => 8192.0,
            AccelerometerSensitive::Sensitive4096 => 4096.0,
            AccelerometerSensitive::Sensitive2048 => 2048.0,
        }
    }
}

#[macro_export]
macro_rules! accelerometer_sensitive {
    (+/-2g, 16384LSB/g) => {
        AccelerometerSensitive::Sensitive16384
    };
    (+/-4g, 8192LSB/g) => {
        AccelerometerSensitive::Sensitive8192
    };
    (+/-8g, 4096LSB/g) => {
        AccelerometerSensitive::Sensitive4096
    };
    (+/-16g, 2048LSB/g) => {
        AccelerometerSensitive::Sensitive2048
    };
}

#[derive(PartialEq, Debug, Copy, Clone)]
pub enum GyroSensitive {
    Sensitive131 = 0,
    Sensitive65_5 = 1,
    Sensitive32_8 = 2,
    Sensitive16_4 = 3,
}

impl Into<f32> for GyroSensitive {
    fn into(self) -> f32 {
        match self {
            GyroSensitive::Sensitive131 => 131.0,
            GyroSensitive::Sensitive65_5 => 65.5,
            GyroSensitive::Sensitive32_8 => 32.8,
            GyroSensitive::Sensitive16_4 => 16.4,
        }
    }
}

#[macro_export]
macro_rules! gyro_sensitive {
    (+/-250dps, 131LSB/dps) => {
        GyroSensitive::Sensitive131
    };
    (+/-500dps, 65.5LSB/dps) => {
        GyroSensitive::Sensitive65_5
    };
    (+/-1000dps, 32.8LSB/dps) => {
        GyroSensitive::Sensitive32_8
    };
    (+/-2000dps, 16.4LSB/dps) => {
        GyroSensitive::Sensitive16_4
    };
}

#[allow(non_camel_case_types)]
#[derive(PartialEq)]
pub enum ProductId {
    Unknown,
    MPU6000ES_REV_C4,
    MPU6000ES_REV_C5,
    MPU6000ES_REV_D6,
    MPU6000ES_REV_D7,
    MPU6000ES_REV_D8,
    MPU6000_REV_C4,
    MPU6000_REV_C5,
    MPU6000_REV_D6,
    MPU6000_REV_D7,
    MPU6000_REV_D8,
    MPU6000_REV_D9,
    MPU6000_REV_D10,
}

impl From<u8> for ProductId {
    fn from(value: u8) -> Self {
        match value {
            0x14 => ProductId::MPU6000_REV_C4,
            0x15 => ProductId::MPU6000ES_REV_C5,
            0x16 => ProductId::MPU6000ES_REV_D6,
            0x17 => ProductId::MPU6000ES_REV_D7,
            0x18 => ProductId::MPU6000ES_REV_D8,
            0x54 => ProductId::MPU6000_REV_C4,
            0x55 => ProductId::MPU6000_REV_C5,
            0x56 => ProductId::MPU6000_REV_D6,
            0x57 => ProductId::MPU6000_REV_D7,
            0x58 => ProductId::MPU6000_REV_D8,
            0x59 => ProductId::MPU6000_REV_D9,
            0x5A => ProductId::MPU6000_REV_D10,
            _ => ProductId::Unknown,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Register {
    ProductId = 0xc,
    SampleRateDivider = 0x19,
    Configuration = 0x1a,
    GyroConfig = 0x1b,
    AccelerometerConfig = 0x1c,
    FifoEnable = 0x23,
    IntPinConfig = 0x37,
    InterruptEnable = 0x38,
    AccelerometerXHigh = 0x3b,
    AccelerometerXLow = 0x3c,
    AccelerometerYHigh = 0x3d,
    AccelerometerYLow = 0x3e,
    AccelerometerZHigh = 0x3f,
    AccelerometerZLow = 0x40,
    TemperatureHigh = 0x41,
    TemperatureLow = 0x42,
    GyroXHigh = 0x43,
    GyroXLow = 0x44,
    GyroYHigh = 0x45,
    GyroYLow = 0x46,
    GyroZHigh = 0x47,
    GyroZLow = 0x48,
    SignalPathReset = 0x68,
    UserControl = 0x6a,
    PowerManagement1 = 0x6b, // Register to control chip waking from sleep, enabling sensors, default: sleep
    PowerManagement2 = 0x6c, // Internal register to check slave addr
    FifoCountHigh = 0x72,
    FifoCountLow = 0x73,
    FifoReadWrite = 0x74,
    WhoAmI = 0x75,
}
