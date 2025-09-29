use defmt::*;

use core::f32::consts::PI;
use libm;

pub const CARGO_HEADER_SIZE: usize = 4;
pub const CARGO_LEN_FIELD_SIZE: usize = 2;
pub const CARGO_LENGTH_MASK: u16 = 0x7FFF;
pub const CARGO_CHANNEL_INDEX: usize = 2;
pub const CARGO_SEQ_NUM_INDEX: usize = 3;


#[derive(Format, Default)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl EulerAngles {
    pub fn to_deg(&mut self) {
        self.roll = self.roll * 180.0 / PI;
        self.pitch = self.pitch * 180.0 / PI;
        self.yaw = self.yaw * 180.0 / PI;
    }
}

#[derive(Format, Clone, Copy, Default)]
pub struct Quaternion {
    pub i: i16,
    pub j: i16,
    pub k: i16,
    pub r: i16,
}

pub fn q_to_f32(q_val: i16, fractional_bits: i16) -> f32 {
    f32::from(q_val) / ((1 << fractional_bits) as f32)
}

impl Quaternion {
    pub fn from_byte_array(byte_arr: &[u8]) -> Self {
        Self {
            i: i16::from_le_bytes(byte_arr[0..2].try_into().unwrap()),
            j: i16::from_le_bytes(byte_arr[2..4].try_into().unwrap()),
            k: i16::from_le_bytes(byte_arr[4..6].try_into().unwrap()),
            r: i16::from_le_bytes(byte_arr[6..8].try_into().unwrap()),
        }
    }

    pub fn to_f32(&self) -> (f32, f32, f32, f32) {
        (
            q_to_f32(self.i, 14),
            q_to_f32(self.j, 14),
            q_to_f32(self.k, 14),
            q_to_f32(self.r, 14),
        )
    }

    // source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
    pub fn to_euler_rad(&self) -> EulerAngles {
        let mut euler: EulerAngles = EulerAngles::default();

        let (x, y, z, w) = self.to_f32();

        let sinr_cosp: f32 = 2.0 * (w * x + y * z);
        let cosr_cosp: f32 = 1.0 - 2.0 * (x * x + y * y);
        euler.roll = libm::atan2f(sinr_cosp, cosr_cosp);

        let sinp: f32 = libm::sqrtf(1.0 + 2.0 * (w * y - x * z));
        let cosp: f32 = libm::sqrtf(1.0 - 2.0 * (w * y - x * z));
        euler.pitch = 2.0 * libm::atan2f(sinp, cosp) - PI / 2.0;


        let siny_cosp: f32 = 2.0 * (w * z + x * y);
        let cosy_cosp: f32 = 1.0 - 2.0 * (y * y + z * z);
        euler.yaw = libm::atan2f(siny_cosp, cosy_cosp);

        euler
    }
}

pub trait ImuInterface {
    fn reset(&mut self) -> impl Future<Output =  ()> + Send;

    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output =  ()> + Send;

    fn write(&mut self, buf: &[u8]) -> impl Future<Output =  ()> + Send;
}

#[derive(Format, Clone, Copy, Default)]
pub enum Channel {
    SHTPCommand = 0,
    Device = 1,
    Control = 2,
    InputNormal = 3,
    InputWake = 4,
    InputGyroRv = 5,
    #[default]
    Undefined = 255,
}

impl Channel {
    pub fn from_u8(channel_id: u8) -> Self {
        match channel_id {
            0 => Self::SHTPCommand,
            1 => Self::Device,
            2 => Self::Control,
            3 => Self::InputNormal,
            4 => Self::InputWake,
            5 => Self::InputGyroRv,
            _ => Self::Undefined,
        }
    }
}

#[derive(Format, Default)]
pub struct SHTPHeader {
    pub cargo_len: usize,
    pub channel: Channel,
    pub seq_num: usize,
}

impl SHTPHeader {
    pub fn from_byte_array(buf: &[u8]) -> Self {
        let temp_len = u16::from_le_bytes(
            buf[0..CARGO_LEN_FIELD_SIZE].try_into().unwrap()
        );

        Self {
            cargo_len: (temp_len & CARGO_LENGTH_MASK) as usize,
            channel: Channel::from_u8(buf[CARGO_CHANNEL_INDEX]),
            seq_num: buf[CARGO_SEQ_NUM_INDEX] as usize,
        }
    }

    pub fn write_to_byte_array(&self, buf: &mut [u8]) {
        buf[0..=1].copy_from_slice(&(self.cargo_len as u16).to_le_bytes());
        buf[2] = self.channel as u8;
        buf[3] = self.seq_num as u8;
    }
}

#[derive(Format, Clone, Copy)]
pub enum ReportID {
    RotationVector = 0x05,
    CommandResponse = 0xF1,
    BaseTimestamp = 0xFB,
    GetFeature = 0xFC,
    SetFeature = 0xFD,
    Undefined = 0xFF,
}

impl ReportID {
    pub fn from_u8(id: u8) -> Self {
        match id {
            0x05    => Self::RotationVector,
            0xF1    => Self::CommandResponse,
            0xFB    => Self::BaseTimestamp,
            0xFC    => Self::GetFeature,
            0xFD    => Self::SetFeature,
            _       => Self::Undefined,
        }
    }
}


// const RELATIVE_CHANGE_SENSITIVITY_FLAG: u8 = 0x01;
// const ENABLE_CHANGE_SENSITIVITY_FLAG: u8 = 0x02;
// const ENABLE_WAKE_UP_FLAG: u8 = 0x04;
// const ENABLE_ALWAYS_ON_FLAG: u8 = 0x08;
pub struct SetFeatureReport {
    pub report_id: ReportID,
    pub feature_report_id: ReportID,
    pub feature_flags: u8,
    pub change_sensitivy: u16,
    pub report_interval: u32, /* Enable sensor by setting positive value */
    pub batch_interval: u32,
    pub misc_config: u32,
}

impl SetFeatureReport {
    pub fn write_to_byte_array(&self, buf: &mut [u8]) {
        buf[0] = self.report_id as u8;
        buf[1] = self.feature_report_id as u8;
        buf[2] = self.feature_flags;
        buf[3..=4].copy_from_slice(&self.change_sensitivy.to_le_bytes());
        buf[5..=8].copy_from_slice(&self.report_interval.to_le_bytes());
        buf[9..=12].copy_from_slice(&self.batch_interval.to_be_bytes());
        buf[13..=16].copy_from_slice(&self.misc_config.to_le_bytes());
    }
}
