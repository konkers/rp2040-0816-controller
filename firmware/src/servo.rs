use embassy_rp::{
    pwm::{self, Config, Pwm},
    Peripheral,
};
use fixed::traits::ToFixed;

pub enum Error {
    AngleOutOfRange,
}

pub type Result<T> = core::result::Result<T, Error>;

pub trait Servo {
    fn set_angle(&mut self, angle: f32) -> Result<()>;
}

pub struct PwmServo<'d, CH: pwm::Channel> {
    pwm: Pwm<'d, CH>,
    config: Config,
}

impl<'d, CH: pwm::Channel> PwmServo<'d, CH> {
    const CYCLES_PER_PERIOD: u16 = 9804;
    const MS_PER_DEGREE: f32 = 1.0f32 / 180.0;
    const CYCLES_PER_MS: f32 = (Self::CYCLES_PER_PERIOD as f32) / 20.0;
    const CYCLES_PER_DEGREE: f32 = Self::CYCLES_PER_MS * Self::MS_PER_DEGREE;
    const CYCLES_OFFSET: f32 = 1.0 * Self::CYCLES_PER_MS;

    pub fn new_a(
        peripheral: impl Peripheral<P = CH> + 'd,
        pin: impl Peripheral<P = impl pwm::PwmPinA<CH>> + 'd,
    ) -> Self {
        let mut config: pwm::Config = Default::default();
        config.divider = 255.to_fixed();
        config.top = Self::CYCLES_PER_PERIOD;

        let pwm = Pwm::new_output_a(peripheral, pin, config.clone());

        Self { pwm, config }
    }
}

impl<'d, CH: pwm::Channel> Servo for PwmServo<'d, CH> {
    fn set_angle(&mut self, angle: f32) -> Result<()> {
        if !(0.0..=180.0).contains(&angle) {
            return Err(Error::AngleOutOfRange);
        }
        self.config.compare_a = (angle * Self::CYCLES_PER_DEGREE + Self::CYCLES_OFFSET) as u16;
        self.pwm.set_config(&self.config);
        Ok(())
    }
}
