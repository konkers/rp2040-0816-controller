use embassy_rp::gpio::{self, Level, Pin};
use pnpfeeder::Input;

pub struct GpioInput<'d, T: Pin> {
    input: gpio::Input<'d, T>,
}

impl<'d, T: Pin> GpioInput<'d, T> {
    pub fn new(mut input: gpio::Input<'d, T>) -> Self {
        input.set_schmitt(true);
        Self { input }
    }
}

impl<'d, T: Pin> Input for GpioInput<'d, T> {
    async fn wait_for_high(&mut self) {
        self.input.wait_for_high().await
    }

    async fn wait_for_low(&mut self) {
        self.input.wait_for_low().await
    }

    async fn wait_for_state_change(&mut self) {
        self.input.wait_for_any_edge().await
    }

    async fn get_state(&mut self) -> bool {
        self.input.get_level() == Level::High
    }
}
