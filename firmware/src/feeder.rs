use embassy_time::Timer;

use crate::{
    servo::{PwmLimits, Servo},
    Error, Result, Value,
};

#[derive(Clone)]
pub struct FeederConfig {
    pub advanced_angle: Value,
    pub half_advanced_angle: Value,
    pub retract_angle: Value,
    pub feed_length: Value,
    pub settle_time: u32,
    pub pwm_0: Value,
    pub pwm_180: Value,
    pub ignore_feeback_pin: bool,
}

pub struct Feeder<'a> {
    servo: &'a mut dyn Servo,
    config: FeederConfig,
    enabled: bool,
}

impl<'a> Feeder<'a> {
    pub fn new(servo: &'a mut dyn Servo) -> Self {
        let limits = servo.get_pwm_limits();
        let config = FeederConfig {
            advanced_angle: Value::from_num(135.0),
            half_advanced_angle: Value::from_num(107.5),
            retract_angle: Value::from_num(80),
            feed_length: Value::from_num(2.0),
            settle_time: 300,
            pwm_0: limits.zero,
            pwm_180: limits.one_eighty,
            ignore_feeback_pin: false,
        };

        Self {
            servo,
            config,
            enabled: false,
        }
    }

    pub fn set_config(&mut self, config: FeederConfig) -> Result<()> {
        self.servo.set_pwm_limits(PwmLimits {
            zero: config.pwm_0,
            one_eighty: config.pwm_180,
        })?;
        self.config = config;
        Ok(())
    }

    pub fn get_config(&self) -> FeederConfig {
        self.config.clone()
    }

    pub fn set_servo_angle(&mut self, angle: Value) -> Result<()> {
        if self.enabled {
            self.servo.set_angle(angle)
        } else {
            Err(Error::FeederDisabled)
        }
    }

    async fn settle(&self) {
        Timer::after_micros(self.config.settle_time as u64 * 1000).await;
    }

    pub async fn advance(&mut self, _length: Option<f32>, _override_error: bool) -> Result<()> {
        self.set_servo_angle(self.config.advanced_angle)?;
        self.settle().await;
        self.set_servo_angle(self.config.retract_angle)?;
        self.settle().await;
        Ok(())
    }

    pub fn enable(&mut self, enabled: bool) {
        self.enabled = enabled
    }
}
