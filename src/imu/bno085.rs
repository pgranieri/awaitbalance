use embassy_stm32::{exti::ExtiInput, gpio::Output, mode::Async, spi::Spi};
use embassy_time::Timer;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[allow(dead_code)]
pub const MAX_CARGO_SIZE: usize = 0x7FFE;
pub const CARGO_BUFFER_SIZE: usize = 2000;
pub const CARGO_HEADER_SIZE: usize = 4;
pub const CARGO_LENGTH_MASK: u16 = 0x7FFF;

#[allow(dead_code)]
static CARGO_BUFFER: Mutex<ThreadModeRawMutex, [u8; CARGO_BUFFER_SIZE]> = Mutex::new([0; CARGO_BUFFER_SIZE]);

pub async fn reset_imu(rst: &mut Output<'_>) {
    /* drive the RST pin low to reset the BNO085 */
    info!("[SPI] drive pin low to reset IMU");
    rst.set_low();
    Timer::after_ticks(1).await; // 10ns minimum hold time, tick period is ~30.5us
    rst.set_high();
}

#[allow(dead_code)]
pub async fn get_initial_packet(spi: &mut Spi<'_, Async>, host_interrupt: &mut ExtiInput<'_>, chip_select: &mut Output<'_>) {
    host_interrupt.wait_for_falling_edge().await;

    info!("[SPI] IMU restarted, taking buffer");

    let mut cargo_buf = *CARGO_BUFFER.lock().await;

    info!("[SPI] took cargo buf: {}", cargo_buf.len());

    info!("[SPI] Trying SPI read, setting cs low...");
    chip_select.set_low();
    Timer::after_ticks(1).await;
    spi.read(&mut cargo_buf[0..CARGO_HEADER_SIZE]).await.expect("spi read should return");
    chip_select.set_high();

    info!("[SPI] read finished");

    let cargo_len = u16::from_le_bytes(cargo_buf[0..2].try_into().unwrap()) & CARGO_LENGTH_MASK;

    if cargo_len > MAX_CARGO_SIZE.try_into().unwrap() {
        error!("[SPI] bad cargo len: {}", cargo_len);
    } else {
        info!("[SPI] packet size: {}", cargo_len);
    }

    /* Wait for next reset pin */
    info!("[SPI] waiting for next interrupt");
    host_interrupt.wait_for_falling_edge().await;

    info!("[SPI] reading 128 bytes to see if there's any data");
    chip_select.set_low();
    Timer::after_ticks(1).await;
    spi.read(&mut cargo_buf[0..128]).await.expect("spi read should return");
    chip_select.set_high();

    for (i, b) in cargo_buf[0..128].iter().enumerate() {
        info!("[SPI] Index: {}, Byte: {:#X}", i, b);
    }
}
