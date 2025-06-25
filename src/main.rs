#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Speed};
use embassy_stm32::Config;
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    let p = embassy_stm32::init(config);

    info!("Hello, World!");

    spawner.spawn(led_task(p.PB0.degrade(), 1000, "Green")).unwrap();
    spawner.spawn(led_task(p.PB7.degrade(), 1010, "Blue")).unwrap();
    spawner.spawn(led_task(p.PB14.degrade(), 1020, "Red")).unwrap();
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
