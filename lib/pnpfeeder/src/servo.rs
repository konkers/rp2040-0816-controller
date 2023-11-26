use crate::{Error, Result, Value, Value64};
use fixed::traits::LosslessTryFrom;

#[derive(Clone)]
pub struct PwmLimits {
    pub zero: Value,
    pub one_eighty: Value,
}

impl PwmLimits {
    pub fn scale_angle(&self, angle: Value) -> Result<Value> {
        if !(0.0..=180.0).contains(&angle) {
            return Err(Error::AngleOutOfRange);
        }
        let angle = Value64::from(angle);
        let range = Value64::from(self.one_eighty - self.zero);
        let counts = Value64::from(self.zero) + (range * angle / Value64::from_num(180.0));
        Value::lossless_try_from(counts).ok_or(Error::FixedPointError)
    }
}

pub trait Servo {
    fn set_angle(&mut self, angle: Value) -> Result<()>;
    fn set_pwm_limits(&mut self, limits: PwmLimits) -> Result<()>;
    fn get_pwm_limits(&self) -> PwmLimits;
}
