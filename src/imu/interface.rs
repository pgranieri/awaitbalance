use defmt::*;

pub const CARGO_HEADER_SIZE: usize = 4;
pub const CARGO_LEN_FIELD_SIZE: usize = 2;
pub const CARGO_LENGTH_MASK: u16 = 0x7FFF;
pub const CARGO_CHANNEL_INDEX: usize = 2;
pub const CARGO_SEQ_NUM_INDEX: usize = 3;

pub trait IMUInterface {
    fn setup(&mut self) -> impl Future<Output =  ()> + Send;

    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output =  usize> + Send;

    fn write(&mut self, buf: &[u8]) -> impl Future<Output =  ()> + Send;
}

#[derive(Format, Clone, Copy)]
pub enum Channel {
    SHTPCommand = 0,
    Device = 1,
    Control = 2,
    InputNormal = 3,
    InputWake = 4,
    InputGyroRv = 5,
    Undefined = 255,
}

impl Channel {
    fn from_u8(channel_id: u8) -> Self {
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

#[derive(Format)]
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