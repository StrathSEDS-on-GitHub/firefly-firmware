use bitvec::view::{BitView, BitViewSized};
use derive_more::{From, Into};
use gortsz::{stats::whitepaper::WhitepaperOptions, *};
use heapless::String;
use serde::{Deserialize, Serialize};

use crate::{MissionStage, PyroPin, Role};

mod private {
    pub trait MessageContext {}
}
pub trait MessageContext: private::MessageContext {}
impl<T: private::MessageContext> MessageContext for T {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RadioCtxt {
    pub source: Role,
    pub counter: u16,
    pub stage: MissionStage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LocalCtxt {
    pub timestamp: u32,
}

impl private::MessageContext for LocalCtxt {}
impl private::MessageContext for RadioCtxt {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message<CTX: MessageContext> {
    pub context: CTX,
    pub message: MessageType,
}

#[derive(Debug, Clone)]
pub struct BitBuffer<T, A = Lsb0>
where
    T: BitViewSized,
    A: gortsz::BitOrder,
{
    bits: BitArray<T, A>,
    used: usize,
}

impl<T, A> BitBuffer<T, A>
where
    T: BitViewSized,
    A: gortsz::BitOrder,
{
    pub fn from_array(bits: BitArray<T, A>, used: usize) -> Self {
        BitBuffer { bits, used }
    }

    pub fn from_slice(bits: &BitSlice<T::Store, A>) -> Self {
        let mut array = BitArray::new(T::ZERO);
        assert!(
            bits.len() <= array.len(),
            "BitSlice length exceeds capacity of BitBuffer"
        );
        array[..bits.len()].copy_from_bitslice(bits);
        BitBuffer {
            bits: array,
            used: bits.len(),
        }
    }

    pub fn as_bitslice(&self) -> &BitSlice<T::Store, A> {
        &self.bits[..self.used]
    }

    pub fn used(&self) -> usize {
        self.used
    }
}

impl<'ser, T, A> Serialize for BitBuffer<T, A>
where
    T: BitViewSized,
    <T as BitView>::Store: Serialize,
    <<T as BitView>::Store as BitStore>::Mem: Serialize,
    A: BitOrder,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.as_bitslice().serialize(serializer)
    }
}

impl<'de, const N: usize, A> Deserialize<'de> for BitBuffer<[u8; N], A>
where
    A: BitOrder,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let bitslice = <&BitSlice<u8, A>>::deserialize(deserializer)?;
        Ok(BitBuffer::from_slice(bitslice))
    }
}

macro_rules! decompress_impl {
    ($compressed:ident, $sample:ident, $width:literal) => {
        impl $compressed {
            pub fn decompress(
                &self,
            ) -> impl Iterator<Item = Result<$sample, DecompressError>> + '_ {
                gortsz::Decompressor::<$width, 3, WhitepaperOptions>::new(self.0.as_bitslice())
                    .map(|it| it.map(From::from))
            }
        }
    };
}

#[derive(Debug, Clone, Serialize, Deserialize, From, Into)]
pub struct GpsCompressed(BitBuffer<[u8; TS_BUF_SIZE]>);
#[derive(Debug, Clone, Serialize, Deserialize, From, Into)]
pub struct PressureTempCompressed(BitBuffer<[u8; TS_BUF_SIZE]>);
#[derive(Debug, Clone, Serialize, Deserialize, From, Into)]
pub struct ImuCompressed(BitBuffer<[u8; TS_BUF_SIZE]>);

#[derive(Debug, Clone, Serialize, Deserialize, From, Into)]
pub struct AccelerometerCompressed(BitBuffer<[u8; TS_BUF_SIZE]>);

decompress_impl!(GpsCompressed, GPSSample, 3);
decompress_impl!(PressureTempCompressed, PressureTempSample, 2);
decompress_impl!(ImuCompressed, IMUSample, 6);
decompress_impl!(AccelerometerCompressed, AccelerometerSample, 3);

pub const TS_BUF_SIZE: usize = 222;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CommandType {
    Info,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CommandResponseType {
    Info {
        hardware: String<32>,
        firmware: String<32>,
        role: Role,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MessageType {
    Log {
        timestamp: u32,
        message: String<200>,
    },
    Gps(GpsCompressed),
    PressureTemp(PressureTempCompressed),
    Imu(ImuCompressed),
    Accelerometer(AccelerometerCompressed),
    Arm(Role),
    Disarm(Role),
    TestPyro(Role, PyroPin, u32),
    SetStage(Role, MissionStage),
    Pyro(u16),
    MissionSumary {
        max_altitude: f32,
        max_altitude_time: u32,
        max_acceleration: f32,
        max_velocity: f32,
        time_of_flight: u32,
    },
    Command {
        role: Role,
        id: u16,
        command: CommandType,
    },
    CommandResponse {
        id: u16,
        response: CommandResponseType,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GPSSample {
    pub timestamp: u32,
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
}

impl Into<(u32, [f32; 3])> for GPSSample {
    fn into(self) -> (u32, [f32; 3]) {
        (
            self.timestamp,
            [self.latitude, self.longitude, self.altitude],
        )
    }
}

impl From<(u32, [f32; 3])> for GPSSample {
    fn from(value: (u32, [f32; 3])) -> Self {
        GPSSample {
            timestamp: value.0,
            latitude: value.1[0],
            longitude: value.1[1],
            altitude: value.1[2],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PressureTempSample {
    pub timestamp: u32,
    pub pressure: f32,
    pub temperature: f32,
}

impl From<(u32, [f32; 2])> for PressureTempSample {
    fn from(value: (u32, [f32; 2])) -> Self {
        PressureTempSample {
            timestamp: value.0,
            pressure: value.1[0],
            temperature: value.1[1],
        }
    }
}

impl Into<(u32, [f32; 2])> for PressureTempSample {
    fn into(self) -> (u32, [f32; 2]) {
        (self.timestamp, [self.pressure, self.temperature])
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct IMUSample {
    pub timestamp: u32,
    pub acceleration: [f32; 3],
    pub angular_velocity: [f32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccelerometerSample {
    pub timestamp: u32,
    pub acceleration: [f32; 3],
}

impl From<(u32, [f32; 6])> for IMUSample {
    fn from(value: (u32, [f32; 6])) -> Self {
        IMUSample {
            timestamp: value.0,
            acceleration: [value.1[0], value.1[1], value.1[2]],
            angular_velocity: [value.1[3], value.1[4], value.1[5]],
        }
    }
}

impl From<IMUSample> for (u32, [f32; 6]) {
    fn from(other: IMUSample) -> Self {
        (
            other.timestamp,
            [
                other.acceleration[0],
                other.acceleration[1],
                other.acceleration[2],
                other.angular_velocity[0],
                other.angular_velocity[1],
                other.angular_velocity[2],
            ],
        )
    }
}

impl From<(u32, [f32; 3])> for AccelerometerSample {
    fn from(value: (u32, [f32; 3])) -> Self {
        AccelerometerSample {
            timestamp: value.0,
            acceleration: value.1,
        }
    }
}

impl From<AccelerometerSample> for (u32, [f32; 3]) {
    fn from(other: AccelerometerSample) -> Self {
        (other.timestamp, other.acceleration)
    }
}

macro_rules! impl_message_type {
    ($name:ident, $sample:ident, $cardinality:literal, $message_kind: ident) => {
        pub fn $name(data: impl IntoIterator<Item = $sample>) -> Self {
            let mut arr = BitArray::new([0u8; TS_BUF_SIZE]);
            let bit_buf = arr.as_mut_bitslice();
            let compressed = match gortsz::compress::<$cardinality, 3, WhitepaperOptions>(
                data.into_iter().map(Into::into),
                bit_buf,
            ) {
                Ok(bits) => bits,
                Err(CompressError { valid_bits, .. }) => {
                    // FIXME: don't silently ignore skipped rows
                    valid_bits
                }
            };
            let used = compressed.len();
            MessageType::$message_kind(BitBuffer::from_array(arr, used).into())
        }
    };
}

impl MessageType {
    impl_message_type!(new_gps, GPSSample, 3, Gps);
    impl_message_type!(new_pressure_temp, PressureTempSample, 2, PressureTemp);
    impl_message_type!(new_imu, IMUSample, 6, Imu);
    impl_message_type!(new_accel, AccelerometerSample, 3, Accelerometer);

    pub fn new_log(timestamp: u32, message: impl AsRef<str>) -> Option<Self> {
        message
            .as_ref()
            .try_into()
            .map(|message| MessageType::Log { timestamp, message })
            .ok()
    }

    pub fn new_arm(role: Role) -> Self {
        MessageType::Arm(role)
    }

    pub fn new_disarm(role: Role) -> Self {
        MessageType::Disarm(role)
    }

    pub fn new_pyro(mv: u16) -> Self {
        MessageType::Pyro(mv)
    }

    pub fn new_test_pyro(role: Role, pin: PyroPin, duration: u32) -> Self {
        MessageType::TestPyro(role, pin, duration)
    }

    pub fn new_set_stage(role: Role, stage: MissionStage) -> Self {
        MessageType::SetStage(role, stage)
    }

    pub fn into_message<CTX: MessageContext>(self, context: CTX) -> Message<CTX> {
        Message {
            context: context,
            message: self,
        }
    }
}
