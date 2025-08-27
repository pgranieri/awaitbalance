use crate::imu::interface::*;
use embassy_stm32::mode::Async;
use embassy_time::Timer;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;

pub struct SPI<'d> {
    spi_bus: Spi<'d, Async>,
    host_int: ExtiInput<'d>,
    chip_select: Output<'d>,
    reset: Output<'d>,
    wake: Output<'d>,
}

impl<'d> SPI<'d> {
    pub fn new(
        spi_bus: Spi<'d, Async>,
        host_int: ExtiInput<'d>,
        chip_select: Output<'d>,
        reset: Output<'d>,
        wake: Output<'d>,
    ) -> Self {
        let spi = Self {
            spi_bus: spi_bus,
            host_int: host_int,
            chip_select: chip_select,
            reset: reset,
            wake: wake,
        };

        spi
    }
}

impl<'d> SPI<'d> {
    async fn get_shtp_header(&mut self, buf: &mut [u8]) {
        self.spi_bus.read(
            &mut buf[0..CARGO_HEADER_SIZE]
        ).await.expect("spi header read should succeed");
    }

    async fn get_cargo_body(&mut self, buf: &mut [u8], len: usize) {
        self.spi_bus.read(
            &mut buf[CARGO_HEADER_SIZE..len]
        ).await.expect("spi body read should succeed");
    }
}

impl<'d> ImuInterface for SPI<'d> {
    fn reset(&mut self) -> impl Future<Output =  ()> + Send {
        async {
            /* drive the RST pin low to reset the BNO085 */
            self.reset.set_low();
            Timer::after_ticks(1).await; // 10ns minimum hold time, tick period is ~30.5us
            self.reset.set_high();
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output =  ()> + Send {
        async {
            self.host_int.wait_for_falling_edge().await;

            self.chip_select.set_low();
            Timer::after_ticks(1).await;

            self.get_shtp_header(buf).await;
            let header = SHTPHeader::from_byte_array(&buf[0..CARGO_HEADER_SIZE]);

            self.get_cargo_body(buf, header.cargo_len).await;
            self.chip_select.set_high();
        }
    }

    fn write(&mut self, buf: &[u8]) -> impl Future<Output =  ()> + Send {
        async {
            self.wake.set_low();
            self.host_int.wait_for_falling_edge().await;

            self.chip_select.set_low();
            Timer::after_ticks(1).await;
            self.wake.set_high();

            self.spi_bus.write(buf).await.expect("spi cargo write should succeed");
        }
    }
}