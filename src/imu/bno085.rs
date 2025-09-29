use embassy_time::Timer;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use super::interface::*;

#[allow(dead_code)]
const MAX_CARGO_SIZE: usize = 0x7FFE;
const CARGO_BUFFER_SIZE: usize = 2048;

const NUM_SHTP_CHANNELS: usize = 6;
const SHTP_INPUT: usize = 0;
const SHTP_OUTPUT: usize = 1;
const SHTP_IO_SIZE: usize = 2;
const COMMAND_RESPONSE_REPORT_SIZE: usize = 16;
const BASE_TIMESTAMP_REPORT_SIZE: usize = 5;
const GET_FEATURE_REPORT_SIZE: usize = 17;
const SET_FEATURE_REPORT_SIZE: usize = 17;
const ROTATION_VECTOR_REPORT_SIZE: usize = 14;

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


pub struct BNO085<S: ImuInterface> {
    interface: S,
    seq_nums: [[u8; NUM_SHTP_CHANNELS]; SHTP_IO_SIZE],
    cargo_buffer: [u8; CARGO_BUFFER_SIZE],
    rotation_vector: Quaternion,
    rotation_vector_cb: Option<fn(Quaternion)>
}

impl<S: ImuInterface> BNO085<S> {
    pub fn new(interface: S) -> Self {
        Self {
            interface,
            seq_nums: [[0; NUM_SHTP_CHANNELS]; SHTP_IO_SIZE],
            cargo_buffer: [0; CARGO_BUFFER_SIZE],
            rotation_vector: Quaternion::default(),
            rotation_vector_cb: None,
        }
    }

    pub async fn init(&mut self) {
        self.interface.reset().await;

        /* consume advertisement packet */
        self.get_shtp_response().await;

        /* consume initialization command response packet */
        self.get_shtp_response().await;

        /* consume device reset complete packet */
        self.get_shtp_response().await;
    }

    pub async fn get_shtp_response(&mut self) {
        self.interface.read(&mut self.cargo_buffer).await;

        let header = SHTPHeader::from_byte_array(&self.cargo_buffer[0..CARGO_HEADER_SIZE]);

        match header.channel {
            Channel::SHTPCommand => {
                self.process_shtp_command_response(header).await;
            },
            Channel::Device => {
                self.process_device_response(header);
            },
            Channel::Control => {
                self.process_control_response(header);
            },
            Channel::InputNormal => {
                self.process_report_response(header);
            }
            _ => error!("Channel not implemented: {}", header.channel),
        };
    }

    pub async fn set_feature_request(&mut self, report: SetFeatureReport, callback: fn(Quaternion)) {
        debug!("Set Feature Request: {}", report.feature_report_id);

        self.rotation_vector_cb = Some(callback);

        let channel = Channel::Control;

        let header = SHTPHeader {
            cargo_len: CARGO_HEADER_SIZE + SET_FEATURE_REPORT_SIZE,
            channel: channel,
            seq_num: self.seq_nums[SHTP_OUTPUT][channel as usize] as usize,
        };

        self.seq_nums[SHTP_OUTPUT][channel as usize] =
            self.seq_nums[SHTP_OUTPUT][channel as usize].wrapping_add(1);

        header.write_to_byte_array(
            &mut self.cargo_buffer[0..CARGO_HEADER_SIZE]
        );

        report.write_to_byte_array(
            &mut self.cargo_buffer[CARGO_HEADER_SIZE..header.cargo_len]
        );

        self.interface.write(&self.cargo_buffer[0..header.cargo_len]).await;
}

    async fn process_shtp_command_response(&mut self, header: SHTPHeader) {
        self.seq_nums[SHTP_INPUT][Channel::SHTPCommand as usize] =
            header.seq_num as u8;

        let mut buf_index = CARGO_HEADER_SIZE;
        let command = SHTPCommandID::from_u8(
            self.cargo_buffer[buf_index]
        );

        debug!("SHTP Command: {}", command);

        match command {
            SHTPCommandID::Advertisement => {
                buf_index += 1;

                while buf_index < header.cargo_len {
                    let tag = SHTPTag::from_u8(self.cargo_buffer[buf_index]);
                    let value_len = self.cargo_buffer[buf_index+1] as usize;

                    buf_index += 2;
                    /* Give rtt time to buffer */
                    Timer::after_millis(20).await;

                    match tag {
                        SHTPTag::GUID => debug!(
                            "{}: {}",
                            tag,
                            u32::from_le_bytes(
                                self.cargo_buffer[
                                    buf_index..buf_index + value_len
                                ].try_into().unwrap()
                            )
                        ),
                        SHTPTag::MaxCargoPlusHeaderWrite |
                        SHTPTag::MaxCargoPlusHeaderRead |
                        SHTPTag::MaxTransferWrite |
                        SHTPTag::MaxTransferRead => debug!(
                            "{}: {}",
                            tag,
                            u16::from_le_bytes(
                                self.cargo_buffer[
                                    buf_index..buf_index + value_len
                                ].try_into().unwrap()
                            )
                        ),
                        SHTPTag::NormalChannel |
                        SHTPTag::WakeChannel => debug!(
                            "{}: {}",
                            tag,
                            u8::from_le_bytes(
                                self.cargo_buffer[
                                    buf_index..buf_index + value_len
                                ].try_into().unwrap()
                            )
                        ),
                        SHTPTag::Version |
                        SHTPTag::AppName |
                        SHTPTag::ChannelName => debug!(
                            "{}: {}",
                            tag,
                            core::str::from_utf8(
                                &self.cargo_buffer[buf_index..buf_index + value_len]
                            ).expect("should be name or version str")
                        ),
                        SHTPTag::ReportLengths => {
                            debug!("{}:", tag);
                            for pair_offset in (0..value_len).step_by(2) {
                                let report_id_offset = buf_index + pair_offset;
                                let report_size_offset = report_id_offset + 1;
                                debug!(
                                    "    ReportID: {:#02X}, ReportLen: {}",
                                    self.cargo_buffer[report_id_offset],
                                    self.cargo_buffer[report_size_offset]
                                );
                                Timer::after_millis(20).await;
                            }
                        },
                        SHTPTag::Reserved |
                        SHTPTag::Undefined => {
                            debug!(
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
                    debug!("No errors found!");
                    return
                }

                for err_index in buf_index..header.cargo_len {
                    error!("Error: {}", self.cargo_buffer[err_index]);
                }
            },
            SHTPCommandID::Undefined => error!("Undefined command: {}", self.cargo_buffer[buf_index]),
        };
    }

    fn process_device_response(&mut self, header: SHTPHeader) {
        self.seq_nums[SHTP_INPUT][Channel::Device as usize] =
            header.seq_num as u8;

        let buf_index = CARGO_HEADER_SIZE;

        let response = self.cargo_buffer[buf_index];

        if response == 1 {
            info!("Device reset complete");
        } else {
            error!("Unexpected device response: {}", response);
        }
    }

    fn process_control_response(&mut self, header: SHTPHeader) {
        self.seq_nums[SHTP_INPUT][Channel::Control as usize] =
            header.seq_num as u8;

        let mut buf_index = CARGO_HEADER_SIZE;

        while buf_index < header.cargo_len {
            let id = ReportID::from_u8(self.cargo_buffer[buf_index]);

            debug!("ReportID: {}", id);

            match id {
                ReportID::CommandResponse => {
                    let seq_num = self.cargo_buffer[buf_index + 1];
                    let mut command = self.cargo_buffer[buf_index + 2];
                    let command_seq_num = self.cargo_buffer[buf_index + 3];
                    let response_seq_num = self.cargo_buffer[buf_index + 4];
                    let vals: &[u8] = &self.cargo_buffer[buf_index + 5..COMMAND_RESPONSE_REPORT_SIZE];

                    let autonomous = (command & 0x80) != 0;
                    command = command & 0x7F;

                    debug!("    seq_num: {}, auto: {}, command: {}", seq_num, autonomous, ControlCommandID::from_u8(command));
                    debug!("    command_seq_num: {}, response_seq_num: {}", command_seq_num, response_seq_num);
                    for (idx, r) in vals.iter().enumerate() {
                            debug!("    Index: {}, Byte: {:#X}", idx, r);
                    }

                    buf_index += COMMAND_RESPONSE_REPORT_SIZE;
                },
                ReportID::GetFeature => {
                    let feature_report_id = ReportID::from_u8(self.cargo_buffer[buf_index + 1]);
                    let _feature_flags = self.cargo_buffer[buf_index + 2];
                    let _change_sensitivity = u16::from_le_bytes(
                        self.cargo_buffer[
                            buf_index + 3..=buf_index + 4
                        ].try_into().unwrap()
                    );
                    let report_interval = u32::from_le_bytes(
                        self.cargo_buffer[
                            buf_index + 5..=buf_index + 8
                        ].try_into().unwrap()
                    );
                    let _batch_interval = u32::from_le_bytes(
                        self.cargo_buffer[
                            buf_index + 9..=buf_index + 12
                        ].try_into().unwrap()
                    );
                    let _misc_config = u32::from_le_bytes(
                        self.cargo_buffer[
                            buf_index + 13..=buf_index + 16
                        ].try_into().unwrap()
                    );

                    info!("Feature Report: {}, Report Interval: {}us", feature_report_id, report_interval);

                    buf_index += GET_FEATURE_REPORT_SIZE;
                },
                _ => {
                    error!("Control Channel: ReportID not implemented: {}", self.cargo_buffer[buf_index]);
                    buf_index = header.cargo_len;
                },
            };
        }
    }

    fn process_report_response(&mut self, header: SHTPHeader) {
        self.seq_nums[SHTP_INPUT][Channel::InputNormal as usize] =
            header.seq_num as u8;

        let mut buf_index = CARGO_HEADER_SIZE;

        while buf_index < header.cargo_len {
            let id = ReportID::from_u8(self.cargo_buffer[buf_index]);

            debug!("ReportID: {}", id);

            match id {
                ReportID::RotationVector => {
                    let seq_num = self.cargo_buffer[buf_index + 1];
                    let status = self.cargo_buffer[buf_index + 2];
                    let delay = self.cargo_buffer[buf_index + 3];
                    self.rotation_vector = Quaternion::from_byte_array(
                        &self.cargo_buffer[
                            buf_index + 4..=buf_index + 11
                        ]
                    );
                    let acc = i16::from_le_bytes(
                        self.cargo_buffer[
                            buf_index + 12..=buf_index + 13
                        ].try_into().unwrap()
                    );

                    debug!("seq_num: {}, status: {}, delay: {}, acc: {}", seq_num, status, delay, q_to_f32(acc, 12));

                    if let Some(callback) = self.rotation_vector_cb {
                        callback(self.rotation_vector);
                    }

                    buf_index += ROTATION_VECTOR_REPORT_SIZE;
                },
                ReportID::BaseTimestamp => {
                    let base_delta = i32::from_le_bytes(
                        self.cargo_buffer[buf_index + 1..=buf_index + 4].try_into().unwrap()
                    );

                    debug!("Batch base timestamp: {} * 100us", base_delta);

                    buf_index += BASE_TIMESTAMP_REPORT_SIZE;
                },
                _ => {
                    error!("ReportID not implemented: {:#02X}", self.cargo_buffer[buf_index]);
                    buf_index = header.cargo_len;
                }
            }
        }
    }
}
