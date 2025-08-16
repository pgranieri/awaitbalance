#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH3, EXTI6, PA5, PA6, PB15, PB5, PB8, PC6, SPI1};
use embassy_stm32::spi::{Spi, MODE_3};
use embassy_stm32::{spi, Config};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod imu;
use imu::bno085::*;

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

    let p = embassy_stm32::init(set_clock_config());

    info!("Hello, World!");

    // Clock source for SPI1 is APB2, which is set to 60MHz
    // BNO085 has a maximum SPI clock speed of 3MHz
    // Set SPI clock divider to 32, giving a baud rate of 1.875MHz
    // trying divider of 128, giving 468,750Hz
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(468_750);
    spi_config.mode = MODE_3; //CPOL = 1, CPHA = 1

    spawner.spawn(
        spi1_task(
            p.SPI1,
            p.PA5,
            p.PB5,
            p.PA6,
            p.DMA2_CH3,
            p.DMA2_CH2,
            p.PB8,
            p.PC6,
            p.EXTI6,
            p.PB15,
            spi_config
        )
    ).unwrap();
    spawner.spawn(led_task(p.PB0.degrade(), 1000, "Green")).unwrap();
    spawner.spawn(led_task(p.PB7.degrade(), 1010, "Blue")).unwrap();
    spawner.spawn(led_task(p.PB14.degrade(), 1020, "Red")).unwrap();
}

fn set_clock_config() -> Config {
    let mut config = Config::default();

    {
        use embassy_stm32::rcc::*;

        // By default, HSE on the board comes from an 8 MHz clock signal (not a crystal) from STLink
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });

        // PLL uses HSE as the clock source
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            // 8 MHz clock source / 8 = 1 MHz PLL input
            prediv: unwrap!(PllPreDiv::try_from(8)),
            // 1 MHz PLL input * 240 = 240 MHz PLL VCO
            mul: unwrap!(PllMul::try_from(240)),
            // 240 MHz PLL VCO / 2 = 120 MHz main PLL output
            divp: Some(PllPDiv::DIV2),
            // 240 MHz PLL VCO / 5 = 48 MHz PLL48 output
            divq: Some(PllQDiv::DIV5),
            divr: None,
        });

        // System clock comes from PLL (= the 120 MHz main PLL output)
        config.rcc.sys = Sysclk::PLL1_P;
        // 120 MHz / 4 = 30 MHz APB1 frequency
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        // 120 MHz / 2 = 60 MHz APB2 frequency
        config.rcc.apb2_pre = APBPrescaler::DIV2;
    }

    config
}

#[embassy_executor::task(pool_size = 3)]
async fn led_task(pin: AnyPin, delay: u64, name: &'static str) {
    let mut led = Output::new(pin, Level::High, Speed::Low);

    loop {
        led.set_high();
        info!("{}: LED High", name);
        Timer::after_millis(delay).await;

        led.set_low();
        info!("{}: LED Low", name);
        Timer::after_millis(delay).await;
    }
}

#[embassy_executor::task]
async fn spi1_task(
    spi1: SPI1,
    sck_pin: PA5,
    mosi_pin: PB5,
    miso_pin: PA6,
    tx_dma: DMA2_CH3,
    rx_dma: DMA2_CH2,
    cs_pin: PB8,
    int_pin: PC6,
    exti6: EXTI6,
    rst_pin: PB15,
    spi_config: spi::Config,
) {
    info!("[SPI] task begin");

    let mut spi = Spi::new(spi1, sck_pin, mosi_pin, miso_pin, tx_dma, rx_dma, spi_config);
    let mut spi_int = ExtiInput::new(int_pin, exti6, Pull::None);
    let mut cs = Output::new(cs_pin, Level::High, Speed::High);
    let mut rst = Output::new(rst_pin, Level::High, Speed::High);

    reset_imu(&mut rst).await;

    get_advertisement_packet(&mut spi, &mut spi_int, &mut cs).await;

    loop {
        Timer::after_secs(5).await;
        info!("[SPI] It's lonely here...");
    }
}
