#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};
use defmt::*;
use embassy_executor::Spawner;
use embassy_executor::main as embassy_main;
use embassy_executor::task as embassy_task;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::spi::Spi as EmbassySPI;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;

use awaitbalance::mcu::stm32f207zg;
use awaitbalance::mcu::spi::SPI as McuSpi;
use awaitbalance::imu::bno085::*;
use awaitbalance::imu::interface::*;

static ROTATION_VECTOR_SINGAL: Signal<ThreadModeRawMutex, Quaternion> = Signal::new();

#[embassy_main]
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
       stm32f207zg::set_clock_config()
    );

    info!("Hello, World!");

    let spi_config = stm32f207zg::set_spi_config();

    let spi_bus = EmbassySPI::new(p.SPI1, p.PA5, p.PB5, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi_config);
    let host_int = ExtiInput::new(p.PC6, p.EXTI6, Pull::None);
    let chip_select = Output::new(p.PB8, Level::High, Speed::High);
    let reset = Output::new(p.PB15, Level::High, Speed::High);
    let wake = Output::new(p.PB9, Level::High, Speed::High);

    let spi = McuSpi::new(
        spi_bus,
        host_int,
        chip_select,
        reset,
        wake,
    );

    let green_led = Output::new(p.PB0, Level::High, Speed::Low);
    let blue_led = Output::new(p.PB7, Level::High, Speed::Low);
    let red_led = Output::new(p.PB14, Level::High, Speed::Low);

    spawner.spawn(imu_task(spi)).unwrap();
    spawner.spawn(pid_task()).unwrap();
    spawner.spawn(led_task(green_led, 1000)).unwrap();
    spawner.spawn(led_task(blue_led, 1010)).unwrap();
    spawner.spawn(led_task(red_led, 1020)).unwrap();
}

#[embassy_task(pool_size = 3)]
async fn led_task(mut led: Output<'static>, delay: u64) {
    loop {
        Timer::after_millis(delay).await;
        led.toggle();
    }
}

#[embassy_task]
async fn pid_task() {
    loop {
        let q = ROTATION_VECTOR_SINGAL.wait().await;

        let mut euler = q.to_euler_rad();
        euler.to_deg();

        info!("[PID] Roll: {}, Pitch: {}, Yaw: {}", euler.roll, euler.pitch, euler.yaw);
    }
}

fn signal_quaternion(q: Quaternion) {
    ROTATION_VECTOR_SINGAL.signal(q);
}

#[embassy_task]
async fn imu_task(
    spi: McuSpi<'static>
) {
    info!("[IMU] task begin");

    let mut imu = BNO085::new(spi);

    imu.init().await;

    let report = SetFeatureReport {
        report_id: ReportID::SetFeature,
        feature_report_id: ReportID::RotationVector,
        feature_flags: 0,
        change_sensitivy: 0,
        report_interval: 2_500, /* 2,500us -> 400Hz */
        batch_interval: 0,
        misc_config: 0,
    };

    imu.set_feature_request(report, signal_quaternion).await;

    loop {
        imu.get_shtp_response().await;
    }
}
