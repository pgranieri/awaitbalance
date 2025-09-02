use defmt::*;
use embassy_stm32::{spi::MODE_3, time::Hertz, Config};

/// Set PLL source to 8MHz external signal from SWD
/// Set main PLL output to 120MHz
/// Set system clock to PLL output (120MHz)
/// Set APB1 clock to 30MHz
/// Set APB2 clock to 60MHz
pub fn set_clock_config() -> Config {
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

/// Clock source for SPI1 is APB2, which is set to 60MHz
/// BNO085 has a maximum SPI clock speed of 3MHz
/// Set SPI clock divider to 32, giving a baud rate of 1.875MHz
pub fn set_spi_config() -> embassy_stm32::spi::Config {
    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(1_875_000);
    spi_config.mode = MODE_3; // CPOL = 1, CPHA = 1
    spi_config
}
