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

static CARGO_BUFFER: Mutex<ThreadModeRawMutex, [u8; CARGO_BUFFER_SIZE]> = Mutex::new([0; CARGO_BUFFER_SIZE]);

pub async fn reset_imu(rst: &mut Output<'_>) {
    /* drive the RST pin low to reset the BNO085 */
    info!("[SPI] drive pin low to reset IMU");
    rst.set_low();
    Timer::after_ticks(1).await; // 10ns minimum hold time, tick period is ~30.5us
    rst.set_high();
}

pub async fn get_advertisement_packet(spi: &mut Spi<'_, Async>, host_interrupt: &mut ExtiInput<'_>, chip_select: &mut Output<'_>) {
    host_interrupt.wait_for_falling_edge().await;

    info!("[SPI] Beginning SPI read, setting cs low...");
    chip_select.set_low();
    Timer::after_ticks(1).await;

    let mut cargo_buf = *CARGO_BUFFER.lock().await;
    spi.read(
        &mut cargo_buf[0..CARGO_HEADER_SIZE]
    ).await.expect("spi read should return");

    let header = SHTPHeader::from_byte_array(
        cargo_buf[0..CARGO_HEADER_SIZE].try_into().unwrap()
    );

    if header.cargo_len as usize > CARGO_BUFFER_SIZE {
        error!("Cargo larger than buffer, aborting: {}", header);
        chip_select.set_high();
        return
    }

    info!("Reading {} bytes over SPI...", header.cargo_len);
    spi.read(
        &mut cargo_buf[CARGO_HEADER_SIZE..header.cargo_len as usize]
    ).await.expect("spi read should return");

    let mut i = CARGO_HEADER_SIZE as usize;
    while i < header.cargo_len as usize{
        let tag = SHTPTag::from_u8(cargo_buf[i]);
        let value_len = cargo_buf[i+1] as usize;

        i += 2;

        match tag {
            SHTPTag::GUID => info!(
                "{}: {}",
                tag,
                u32::from_le_bytes(
                    cargo_buf[i..i + value_len].try_into().unwrap()
                )
            ),
            SHTPTag::MaxCargoPlusHeaderWrite |
            SHTPTag::MaxCargoPlusHeaderRead |
            SHTPTag::MaxTransferWrite |
            SHTPTag::MaxTransferRead => info!(
                "{}: {}",
                tag,
                u16::from_le_bytes(
                    cargo_buf[i..i + value_len].try_into().unwrap()
                )
            ),
            SHTPTag::NormalChannel |
            SHTPTag::WakeChannel => info!(
                "{}: {}",
                tag,
                u8::from_le_bytes(
                    cargo_buf[i..i + value_len].try_into().unwrap()
                )
            ),
            SHTPTag::SHTPVersion |
            SHTPTag::AppName |
            SHTPTag::ChannelName => info!(
                "{}: {}",
                tag,
                str::from_utf8(
                    &cargo_buf[i..i + value_len]
                ).expect("should be name or version str")
            ),
            SHTPTag::Reserved |
            SHTPTag::Undefined => info!(
                "{}: Len: {}",
                tag,
                value_len,
            ),
        }

        i += value_len;
    }

    chip_select.set_high();
}

#[derive(Format)]
struct SHTPHeader {
    cargo_len: u16,
    channel: u8,
    seq_num: u8,
}

const CARGO_LEN_FIELD_SIZE: usize = 2;
const CARGO_CHANNEL_INDEX: usize = 2;
const CARGO_SEQ_NUM_INDEX: usize = 3;

impl SHTPHeader {
    fn from_byte_array(byte_arr: [u8; CARGO_HEADER_SIZE]) -> SHTPHeader {
        let temp_len = u16::from_le_bytes(
            byte_arr[0..CARGO_LEN_FIELD_SIZE].try_into().unwrap()
        );

        SHTPHeader {
            cargo_len: temp_len & CARGO_LENGTH_MASK,
            channel: byte_arr[CARGO_CHANNEL_INDEX],
            seq_num: byte_arr[CARGO_SEQ_NUM_INDEX],
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
    SHTPVersion,
    Undefined,
}

impl SHTPTag {
    fn from_u8(i: u8) -> SHTPTag {
        match i {
            0       => SHTPTag::Reserved,
            1       => SHTPTag::GUID,
            2       => SHTPTag::MaxCargoPlusHeaderWrite,
            3       => SHTPTag::MaxCargoPlusHeaderRead,
            4       => SHTPTag::MaxTransferWrite,
            5       => SHTPTag::MaxTransferRead,
            6       => SHTPTag::NormalChannel,
            7       => SHTPTag::WakeChannel,
            8       => SHTPTag::AppName,
            9       => SHTPTag::ChannelName,
            0x80    => SHTPTag::SHTPVersion,
            _       => SHTPTag::Undefined,
        }
    }
}