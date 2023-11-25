use embassy_rp::{
    pwm::{self, Config, Pwm},
    Peripheral,
};
use fixed::traits::ToFixed;

use crate::{Error, Result};

#[derive(Clone)]
pub struct PwmLimits {
    pub zero: u16,
    pub one_eighty: u16,
}

impl PwmLimits {
    fn scale_angle(&self, angle: f32) -> Result<u16> {
        if !(0.0..=180.0).contains(&angle) {
            return Err(Error::AngleOutOfRange);
        }
        let range = self.one_eighty - self.zero;

        Ok(self.zero + (range as f32 * angle / 180.0) as u16)
    }
}

pub trait Servo {
    fn set_angle(&mut self, angle: f32) -> Result<()>;
    fn set_pwm_limits(&mut self, limits: PwmLimits) -> Result<()>;
    fn get_pwm_limits(&self) -> PwmLimits;
}

pub struct PwmServo<'d, CH: pwm::Channel> {
    pwm: Pwm<'d, CH>,
    config: Config,
    limits: PwmLimits,
}

impl<'d, CH: pwm::Channel> PwmServo<'d, CH> {
    const COUNTS_PER_PERIOD: u16 = 9804;
    const COUNTS_PER_MS: f32 = (Self::COUNTS_PER_PERIOD as f32) / 20.0;
    const DEFAULT_COUNTS_AT_0: u16 = (1.0 * Self::COUNTS_PER_MS) as u16;
    const DEFAULT_COUNTS_AT_180: u16 = (2.0 * Self::COUNTS_PER_MS) as u16;

    pub fn new_a(
        peripheral: impl Peripheral<P = CH> + 'd,
        pin: impl Peripheral<P = impl pwm::PwmPinA<CH>> + 'd,
    ) -> Self {
        let mut config: pwm::Config = Default::default();
        config.divider = 255.to_fixed();
        config.top = Self::COUNTS_PER_PERIOD;

        let pwm = Pwm::new_output_a(peripheral, pin, config.clone());

        Self {
            pwm,
            config,
            limits: PwmLimits {
                zero: Self::DEFAULT_COUNTS_AT_0,
                one_eighty: Self::DEFAULT_COUNTS_AT_180,
            },
        }
    }
}

impl<'d, CH: pwm::Channel> Servo for PwmServo<'d, CH> {
    fn set_angle(&mut self, angle: f32) -> Result<()> {
        self.config.compare_a = self.limits.scale_angle(angle)?;
        self.pwm.set_config(&self.config);
        Ok(())
    }

    fn set_pwm_limits(&mut self, limits: PwmLimits) -> Result<()> {
        if limits.zero > Self::COUNTS_PER_PERIOD || limits.one_eighty > Self::COUNTS_PER_PERIOD {
            return Err(Error::PwmValueOutOfRange);
        }
        self.limits = limits;
        Ok(())
    }

    fn get_pwm_limits(&self) -> PwmLimits {
        self.limits.clone()
    }
}
