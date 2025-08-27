#![no_std]
#![no_main]

use awaitbalance::imu::interface::IMUInterface;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH3, EXTI6, PA5, PA6, PB15, PB5, PB8, PB9, PC6, SPI1};
use embassy_stm32::spi::Spi;
use embassy_stm32::spi;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiBus;
use {defmt_rtt as _, panic_probe as _};

use awaitbalance::*;
use awaitbalance::imu::*;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    /*
        Pin Map
        CS:     PB_8
        SCK:    PA_5
        MISO:   PA_6 (SDA)
        MOSI:   PB_5 (DI)
        INT:    PC_6
        RST:    PB_15
     */

    let p = embassy_stm32::init(
       mcu::stm32f207zg::set_clock_config()
    );

    info!("Hello, World!");

    let spi_config = mcu::stm32f207zg::set_spi_config();

    let spi_bus = Spi::new(p.SPI1, p.PA5, p.PB5, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi_config);
    let host_int = ExtiInput::new(p.PC6, p.EXTI6, Pull::None);
    let chip_select = Output::new(p.PB8, Level::High, Speed::High);
    let reset = Output::new(p.PB15, Level::High, Speed::High);
    let wake = Output::new(p.PB9, Level::High, Speed::High);

    let spi = mcu::spi::SPI::new(
        spi_bus,
        host_int,
        chip_select,
        reset,
        wake,
    );

    spawner.spawn(spi1_task(spi)).unwrap();
    // spawner.spawn(led_task(p.PB0.degrade(), 1000, "Green")).unwrap();
    // spawner.spawn(led_task(p.PB7.degrade(), 1010, "Blue")).unwrap();
    // spawner.spawn(led_task(p.PB14.degrade(), 1020, "Red")).unwrap();
}

// #[embassy_executor::task(pool_size = 3)]
// async fn led_task(pin: AnyPin, delay: u64, _name: &'static str) {
//     let mut led = Output::new(pin, Level::High, Speed::Low);

//     loop {
//         led.set_high();
//         // info!("{}: LED High", name);
//         Timer::after_millis(delay).await;

//         led.set_low();
//         // info!("{}: LED Low", name);
//         Timer::after_millis(delay).await;
//     }
// }

#[embassy_executor::task]
async fn spi1_task(
    mut spi: mcu::spi::SPI<'static>
) {
    info!("[SPI] task begin");

    spi.setup().await;

    // bno085::reset_imu(&mut rst).await;

    // /* Get advertisement packet */
    // spi_int.wait_for_falling_edge().await;
    // bno085::get_shtp_response(&mut spi, &mut cs).await;

    // /* Get initialization command response packet */
    // spi_int.wait_for_falling_edge().await;
    // bno085::get_shtp_response(&mut spi, &mut cs).await;

    // /* Get device reset complete packet */
    // spi_int.wait_for_falling_edge().await;
    // bno085::get_shtp_response(&mut spi, &mut cs).await;

    // bno085::enable_rotation_vector(&mut wake, &mut spi_int, &mut spi, &mut cs).await;

    // loop {
    //     info!("Waiting for next packet");
    //     spi_int.wait_for_falling_edge().await;

    //     bno085::get_shtp_response(&mut spi, &mut cs).await;
    // }
}
