use crate::{
    servo::{PwmLimits, Servo},
    Result,
};

#[derive(Clone)]
pub struct FeederConfig {
    pub advanced_angle: f32,
    pub half_advanced_angle: f32,
    pub retract_angle: f32,
    pub feed_length: f32,
    pub settle_time: u32,
    pub pwm_0: u16,
    pub pwm_180: u16,
    pub ignore_feeback_pin: bool,
}

pub struct Feeder<'a> {
    servo: &'a mut dyn Servo,
    config: FeederConfig,
}

impl<'a> Feeder<'a> {
    const DEFAULT_ADVANCED_ANGLE: f32 = 130.0;
    const DEFAULT_HALF_ADVANCED_ANGLE: f32 = 107.5;
    const DEFAULT_RETRACT_ANGLE: f32 = 85.0;
    const DEFAULT_FEED_LENGTH: f32 = 2.0;
    const DEFAULT_SETTLE_TIME: u32 = 500; // ms
    const DEFAULT_IGNORE_FEEDBACK_PIN: bool = false;

    pub fn new(servo: &'a mut dyn Servo) -> Self {
        let limits = servo.get_pwm_limits();
        let config = FeederConfig {
            advanced_angle: Self::DEFAULT_ADVANCED_ANGLE,
            half_advanced_angle: Self::DEFAULT_HALF_ADVANCED_ANGLE,
            retract_angle: Self::DEFAULT_RETRACT_ANGLE,
            feed_length: Self::DEFAULT_FEED_LENGTH,
            settle_time: Self::DEFAULT_SETTLE_TIME,
            pwm_0: limits.zero,
            pwm_180: limits.one_eighty,
            ignore_feeback_pin: Self::DEFAULT_IGNORE_FEEDBACK_PIN,
        };

        Self { servo, config }
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

    pub fn set_servo_angle(&mut self, angle: f32) -> Result<()> {
        self.servo.set_angle(angle)
    }
}
