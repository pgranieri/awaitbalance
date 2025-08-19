use embassy_stm32::{exti::ExtiInput, gpio::Output, mode::Async, spi::Spi};
use embassy_time::Timer;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[allow(dead_code)]
pub const MAX_CARGO_SIZE: usize = 0x7FFE;
pub const CARGO_BUFFER_SIZE: usize = 2048;
pub const CARGO_HEADER_SIZE: usize = 4;
pub const CARGO_LENGTH_MASK: u16 = 0x7FFF;

const CARGO_LEN_FIELD_SIZE: usize = 2;
const CARGO_CHANNEL_INDEX: usize = 2;
const CARGO_SEQ_NUM_INDEX: usize = 3;

const COMMAND_RESPONSE_REPORT_SIZE: usize = 16;
const BASE_TIMESTAMP_REPORT_SIZE: usize = 5;
const GET_FEATURE_REPORT_SIZE: usize = 17;
const SET_FEATURE_REPORT_SIZE: usize = 17;
const ROTATION_VECTOR_REPORT_SIZE: usize = 14;

const Q12_SCALE: f32 = (1 << 12) as f32;
const Q14_SCALE: f32 = (1 << 14) as f32;

static CARGO_BUFFER: Mutex<ThreadModeRawMutex, [u8; CARGO_BUFFER_SIZE]> = Mutex::new([0; CARGO_BUFFER_SIZE]);

#[derive(Format)]
struct SHTPHeader {
    cargo_len: usize,
    channel: Channel,
    seq_num: usize,
}

impl SHTPHeader {
    fn from_byte_array(buf: &[u8]) -> Self {
        let temp_len = u16::from_le_bytes(
            buf[0..CARGO_LEN_FIELD_SIZE].try_into().unwrap()
        );

        Self {
            cargo_len: (temp_len & CARGO_LENGTH_MASK) as usize,
            channel: Channel::from_u8(buf[CARGO_CHANNEL_INDEX]),
            seq_num: buf[CARGO_SEQ_NUM_INDEX] as usize,
        }
    }

    fn write_to_byte_array(&self, buf: &mut [u8]) {
        buf[0..=1].copy_from_slice(&(self.cargo_len as u16).to_le_bytes());
        buf[2] = self.channel as u8;
        buf[3] = self.seq_num as u8;
    }
}

#[derive(Format, Clone, Copy)]
enum Channel {
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
enum SHTPCommandID {
    Advertisement,
    ErrorList,
    Undefined,
}

impl SHTPCommandID {
    fn from_u8(command_id: u8) -> Self {
        match command_id {
            0 => Self::Advertisement,
            1 => Self::ErrorList,
            _ => Self::Undefined,
        }
    }
}

#[derive(Format)]
enum SHTPTag {
    Reserved,
    GUID,
    MaxCargoPlusHeaderWrite,
    MaxCargoPlusHeaderRead,
    MaxTransferWrite,
    MaxTransferRead,
    NormalChannel,
    WakeChannel,
    AppName,
    ChannelName,
    Version,
    ReportLengths,
    Undefined,
}

impl SHTPTag {
    fn from_u8(i: u8) -> Self {
        match i {
            0       => Self::Reserved,
            1       => Self::GUID,
            2       => Self::MaxCargoPlusHeaderWrite,
            3       => Self::MaxCargoPlusHeaderRead,
            4       => Self::MaxTransferWrite,
            5       => Self::MaxTransferRead,
            6       => Self::NormalChannel,
            7       => Self::WakeChannel,
            8       => Self::AppName,
            9       => Self::ChannelName,
            0x80    => Self::Version,
            0x81    => Self::ReportLengths,
            _       => Self::Undefined,
        }
    }
}

#[derive(Format)]
enum ControlCommandID {
    Initialization,
    InitializationUnsolicited,
    Undefined,
}

impl ControlCommandID {
    fn from_u8(id: u8) -> Self {
        match id {
            0x4     => Self::Initialization,
            0x84    => Self::InitializationUnsolicited,
            _       => Self::Undefined,
        }
    }
}

#[derive(Format, Clone, Copy)]
enum ReportID {
    RotationVector = 0x05,
    CommandResponse = 0xF1,
    BaseTimestamp = 0xFB,
    GetFeature = 0xFC,
    SetFeature = 0xFD,
    Undefined = 0xFF,
}

impl ReportID {
    fn from_u8(id: u8) -> Self {
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

#[derive(Format)]
struct Quaternion {
    i: u16,
    j: u16,
    k: u16,
    r: u16,
}

fn q14_to_f32(q_val: u16) -> f32 {
    f32::from(q_val) / Q14_SCALE
}

fn q12_to_f32(q_val: u16) -> f32 {
    f32::from(q_val) / Q12_SCALE
}

impl Quaternion {
    fn from_byte_array(byte_arr: &[u8]) -> Self {
        Self {
            i: u16::from_le_bytes(byte_arr[0..2].try_into().unwrap()),
            j: u16::from_le_bytes(byte_arr[2..4].try_into().unwrap()),
            k: u16::from_le_bytes(byte_arr[4..6].try_into().unwrap()),
            r: u16::from_le_bytes(byte_arr[6..8].try_into().unwrap()),
        }
    }

    fn to_f32(&self) -> [f32; 4] {
        [
            q14_to_f32(self.i),
            q14_to_f32(self.j),
            q14_to_f32(self.k),
            q14_to_f32(self.r),
        ]
    }
}

pub async fn reset_imu(rst: &mut Output<'_>) {
    /* drive the RST pin low to reset the BNO085 */
    rst.set_low();
    Timer::after_ticks(1).await; // 10ns minimum hold time, tick period is ~30.5us
    rst.set_high();
}

pub async fn get_shtp_response(spi: &mut Spi<'_, Async>, chip_select: &mut Output<'_>) {
    chip_select.set_low();
    Timer::after_ticks(1).await;

    let mut cargo_buf = *CARGO_BUFFER.lock().await;

    spi.read(
        &mut cargo_buf[0..CARGO_HEADER_SIZE]
    ).await.expect("spi read should return");

    let header = SHTPHeader::from_byte_array(
        &cargo_buf[0..CARGO_HEADER_SIZE]
    );

    if header.cargo_len < CARGO_HEADER_SIZE || header.cargo_len > CARGO_BUFFER_SIZE {
        error!("Bad cargo len: {}", header.cargo_len);
        chip_select.set_high();
        return
    } else {
        info!("{}", header);
    }

    let buf_index = CARGO_HEADER_SIZE;

    spi.read(
        &mut cargo_buf[buf_index..header.cargo_len]
    ).await.expect("spi read should return");

    match header.channel {
        Channel::SHTPCommand => {
            process_shtp_command_response(header, &cargo_buf).await;
        },
        Channel::Device => {
            process_device_response(&cargo_buf);
        },
        Channel::Control => {
            process_control_response(header, &cargo_buf);
        },
        Channel::InputNormal => {
            process_report_response(header, &cargo_buf);
        }
        _ => error!("Channel not implemented: {}", header.channel),
    };

    chip_select.set_high();
}

async fn process_shtp_command_response(header: SHTPHeader, cargo_buf: &[u8]) {
    let mut buf_index = CARGO_HEADER_SIZE;

    let command = SHTPCommandID::from_u8(cargo_buf[buf_index]);

    info!("SHTP Command: {}", command);

    match command {
        SHTPCommandID::Advertisement => {
            buf_index += 1;

            while buf_index < header.cargo_len {
                let tag = SHTPTag::from_u8(cargo_buf[buf_index]);
                let value_len = cargo_buf[buf_index+1] as usize;

                buf_index += 2;
                /* Give rtt time to buffer */
                Timer::after_millis(20).await;

                match tag {
                    SHTPTag::GUID => info!(
                        "{}: {}",
                        tag,
                        u32::from_le_bytes(
                            cargo_buf[
                                buf_index..buf_index + value_len
                            ].try_into().unwrap()
                        )
                    ),
                    SHTPTag::MaxCargoPlusHeaderWrite |
                    SHTPTag::MaxCargoPlusHeaderRead |
                    SHTPTag::MaxTransferWrite |
                    SHTPTag::MaxTransferRead => info!(
                        "{}: {}",
                        tag,
                        u16::from_le_bytes(
                            cargo_buf[
                                buf_index..buf_index + value_len
                            ].try_into().unwrap()
                        )
                    ),
                    SHTPTag::NormalChannel |
                    SHTPTag::WakeChannel => info!(
                        "{}: {}",
                        tag,
                        u8::from_le_bytes(
                            cargo_buf[
                                buf_index..buf_index + value_len
                            ].try_into().unwrap()
                        )
                    ),
                    SHTPTag::Version |
                    SHTPTag::AppName |
                    SHTPTag::ChannelName => info!(
                        "{}: {}",
                        tag,
                        str::from_utf8(
                            &cargo_buf[buf_index..buf_index + value_len]
                        ).expect("should be name or version str")
                    ),
                    SHTPTag::ReportLengths => {
                        info!("{}:", tag);
                        for pair_offset in (0..value_len).step_by(2) {
                            let report_id_offset = buf_index + pair_offset;
                            let report_size_offset = report_id_offset + 1;
                            info!(
                                "    ReportID: {:#02X}, ReportLen: {}",
                                cargo_buf[report_id_offset],
                                cargo_buf[report_size_offset]
                            );
                            Timer::after_millis(20).await;
                        }
                    },
                    SHTPTag::Reserved |
                    SHTPTag::Undefined => {
                        info!(
                            "{}: Len: {}", tag, value_len,
                        )
                    },
                }

                buf_index += value_len;
            }
        },
        SHTPCommandID::ErrorList => {
            buf_index += 1;

            if buf_index == header.cargo_len {
                info!("No errors found!");
                return
            }

            for err_index in buf_index..header.cargo_len {
                error!("Error: {}", cargo_buf[err_index]);
            }
        },
        SHTPCommandID::Undefined => error!("Undefined command: {}", cargo_buf[buf_index]),
    };
}

fn process_device_response(cargo_buf: &[u8]) {
    let buf_index = CARGO_HEADER_SIZE;

    let response = cargo_buf[buf_index];

    if response == 1 {
        info!("Device reset complete");
    } else {
        info!("Unexpected device response: {}", response);
    }
}

fn process_control_response(header: SHTPHeader, cargo_buf: &[u8]) {
    let mut buf_index = CARGO_HEADER_SIZE;

    while buf_index < header.cargo_len {
        let id = ReportID::from_u8(cargo_buf[buf_index]);

        info!("ReportID: {}", id);

        match id {
            ReportID::CommandResponse => {
                let seq_num = cargo_buf[buf_index + 1];
                let mut command = cargo_buf[buf_index + 2];
                let command_seq_num = cargo_buf[buf_index + 3];
                let response_seq_num = cargo_buf[buf_index + 4];
                let vals: &[u8] = &cargo_buf[buf_index + 5..COMMAND_RESPONSE_REPORT_SIZE];

                let autonomous = (command & 0x80) != 0;
                command = command & 0x7F;

                info!("    seq_num: {}, auto: {}, command: {}", seq_num, autonomous, ControlCommandID::from_u8(command));
                info!("    command_seq_num: {}, response_seq_num: {}", command_seq_num, response_seq_num);
                for (idx, r) in vals.iter().enumerate() {
                        info!("    Index: {}, Byte: {:#X}", idx, r);
                }

                buf_index += COMMAND_RESPONSE_REPORT_SIZE;
            },
            ReportID::GetFeature => {
                let feature_report_id = ReportID::from_u8(cargo_buf[buf_index + 1]);
                let _feature_flags = cargo_buf[buf_index + 2];
                let _change_sensitivity = u16::from_le_bytes(
                    cargo_buf[
                        buf_index + 3..=buf_index + 4
                    ].try_into().unwrap()
                );
                let report_interval = u32::from_le_bytes(
                    cargo_buf[
                        buf_index + 5..=buf_index + 8
                    ].try_into().unwrap()
                );
                let _batch_interval = u32::from_le_bytes(
                    cargo_buf[
                        buf_index + 9..=buf_index + 12
                    ].try_into().unwrap()
                );
                let _misc_config = u32::from_le_bytes(
                    cargo_buf[
                        buf_index + 13..=buf_index + 16
                    ].try_into().unwrap()
                );

                info!("Feature Report: {}, Report Interval: {}us", feature_report_id, report_interval);

                buf_index += GET_FEATURE_REPORT_SIZE;
            },
            _ => {
                error!("Control Channel: ReportID not implemented: {}", cargo_buf[buf_index]);
                buf_index = header.cargo_len;
            },
        };
    }
}

fn process_report_response(header: SHTPHeader, cargo_buf: &[u8]) {
    let mut buf_index = CARGO_HEADER_SIZE;

    while buf_index < header.cargo_len {
        let id = ReportID::from_u8(cargo_buf[buf_index]);

        info!("ReportID: {}", id);

        match id {
            ReportID::RotationVector => {
                let seq_num = cargo_buf[buf_index + 1];
                let status = cargo_buf[buf_index + 2];
                let delay = cargo_buf[buf_index + 3];
                let q = Quaternion::from_byte_array(
                    &cargo_buf[
                        buf_index + 4..=buf_index + 11
                    ]
                );
                let acc = u16::from_le_bytes(
                    cargo_buf[
                        buf_index + 12..=buf_index + 13
                    ].try_into().unwrap()
                );

                info!("seq_num: {}, status: {}, delay: {}", seq_num, status, delay);
                info!("quaternion: {}", q.to_f32());
                info!("accuracy: {}", q12_to_f32(acc));

                buf_index += ROTATION_VECTOR_REPORT_SIZE;
            },
            ReportID::BaseTimestamp => {
                let base_delta = i32::from_le_bytes(
                    cargo_buf[buf_index + 1..=buf_index + 4].try_into().unwrap()
                );

                info!("Batch base timestamp: {} * 100us", base_delta);

                buf_index += BASE_TIMESTAMP_REPORT_SIZE;
            },
            _ => {
                error!("ReportID not implemented: {:#02X}", cargo_buf[buf_index]);
                buf_index = header.cargo_len;
            }
        }
    }
}

struct SetFeatureReport {
    report_id: ReportID,
    feature_report_id: ReportID,
    feature_flags: u8,
    change_sensitivy: u16,
    report_interval: u32, /* Enable sensor by setting positive value */
    batch_interval: u32,
    misc_config: u32,
}

impl SetFeatureReport {
    fn write_to_byte_array(&self, buf: &mut [u8]) {
        buf[0] = self.report_id as u8;
        buf[1] = self.feature_report_id as u8;
        buf[2] = self.feature_flags;
        buf[3..=4].copy_from_slice(&self.change_sensitivy.to_le_bytes());
        buf[5..=8].copy_from_slice(&self.report_interval.to_le_bytes());
        buf[9..=12].copy_from_slice(&self.batch_interval.to_be_bytes());
        buf[13..=16].copy_from_slice(&self.misc_config.to_le_bytes());
    }
}

async fn set_feature_request(report: SetFeatureReport, wake: &mut Output<'_>, spi_int: &mut ExtiInput<'_>, spi: &mut Spi<'_, Async>, chip_select: &mut Output<'_>) {
    let header = SHTPHeader {
        cargo_len: CARGO_HEADER_SIZE + SET_FEATURE_REPORT_SIZE,
        channel: Channel::Control,
        seq_num: 0,
    };

    let mut cargo_buf = *CARGO_BUFFER.lock().await;

    header.write_to_byte_array(
        &mut cargo_buf[0..CARGO_HEADER_SIZE]
    );

    report.write_to_byte_array(
        &mut cargo_buf[CARGO_HEADER_SIZE..header.cargo_len]
    );

    wake.set_low();
    spi_int.wait_for_falling_edge().await;

    chip_select.set_low();
    Timer::after_ticks(1).await;
    wake.set_high();

    spi.write(&cargo_buf[0..header.cargo_len]).await.expect("spi write should return");

    chip_select.set_high();
}

// const RELATIVE_CHANGE_SENSITIVITY_FLAG: u8 = 0x01;
// const ENABLE_CHANGE_SENSITIVITY_FLAG: u8 = 0x02;
// const ENABLE_WAKE_UP_FLAG: u8 = 0x04;
// const ENABLE_ALWAYS_ON_FLAG: u8 = 0x08;

pub async fn enable_rotation_vector(wake: &mut Output<'_>, spi_int: &mut ExtiInput<'_>, spi: &mut Spi<'_, Async>, chip_select: &mut Output<'_>) {
    let report = SetFeatureReport {
        report_id: ReportID::SetFeature,
        feature_report_id: ReportID::RotationVector,
        feature_flags: 0,
        change_sensitivy: 0,
        report_interval: 1_000_000, /* 1,000,000us -> 1Hz */
        batch_interval: 0,
        misc_config: 0,
    };

    set_feature_request(report, wake, spi_int, spi, chip_select).await;
}
