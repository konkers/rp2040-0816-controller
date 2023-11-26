use az::Cast;
use embassy_rp::{
    pwm::{self, Config, Pwm},
    Peripheral,
};
use fixed::traits::ToFixed;

use crate::{Error, Result, Value};

#[derive(Clone)]
pub struct PwmLimits {
    pub zero: Value,
    pub one_eighty: Value,
}

impl PwmLimits {
    fn scale_angle(&self, angle: Value) -> Result<Value> {
        if !(0.0..=180.0).contains(&angle) {
            return Err(Error::AngleOutOfRange);
        }
        let range: Value = self.one_eighty - self.zero;

        Ok(self.zero + (range * angle / Value::from_num(180.0)))
    }
}

pub trait Servo {
    fn set_angle(&mut self, angle: Value) -> Result<()>;
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

    pub fn new_a(
        peripheral: impl Peripheral<P = CH> + 'd,
        pin: impl Peripheral<P = impl pwm::PwmPinA<CH>> + 'd,
    ) -> Self {
        let mut config: pwm::Config = Default::default();
        config.divider = 255.to_fixed();
        config.top = Self::COUNTS_PER_PERIOD;

        let pwm = Pwm::new_output_a(peripheral, pin, config.clone());

        let counts_per_ms = Value::from_num(Self::COUNTS_PER_PERIOD) / Value::from_num(20.0);
        let zero = Value::from_num(1.0) * counts_per_ms;
        let one_eighty = Value::from_num(2.0) * counts_per_ms;
        Self {
            pwm,
            config,
            limits: PwmLimits { zero, one_eighty },
        }
    }
}

impl<'d, CH: pwm::Channel> Servo for PwmServo<'d, CH> {
    fn set_angle(&mut self, angle: Value) -> Result<()> {
        self.config.compare_a = self.limits.scale_angle(angle)?.cast();
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
