use defmt::*;

pub const CARGO_HEADER_SIZE: usize = 4;
pub const CARGO_LEN_FIELD_SIZE: usize = 2;
pub const CARGO_LENGTH_MASK: u16 = 0x7FFF;
pub const CARGO_CHANNEL_INDEX: usize = 2;
pub const CARGO_SEQ_NUM_INDEX: usize = 3;

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
