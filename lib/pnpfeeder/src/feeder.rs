use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{self, Channel},
};
use embassy_time::Timer;

use crate::{
    servo::{PwmLimits, Servo},
    Error, Input, Result, Value,
};

#[derive(Clone, Debug)]
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

impl Default for FeederConfig {
    fn default() -> Self {
        Self {
            advanced_angle: Value::from_num(135.0),
            half_advanced_angle: Value::from_num(107.5),
            retract_angle: Value::from_num(80),
            feed_length: Value::from_num(2.0),
            settle_time: 300,
            pwm_0: Value::from_num(0),
            pwm_180: Value::from_num(0),
            ignore_feeback_pin: false,
        }
    }
}

enum FeederCommand {
    SetConfig(FeederConfig),
    GetConfig(),
    SetServoAngle(Value),
    Advance {
        length: Option<Value>,
        override_error: bool,
    },
    Enable(bool),
    #[cfg(test)]
    Shutdown,
}

pub struct FeederChannel {
    command_channel: channel::Channel<NoopRawMutex, FeederCommand, 2>,
    response_channel: channel::Channel<NoopRawMutex, Result<Option<FeederConfig>>, 2>,
}

impl FeederChannel {
    pub fn new() -> Self {
        Self {
            command_channel: Channel::new(),
            response_channel: Channel::new(),
        }
    }
}

impl Default for FeederChannel {
    fn default() -> Self {
        Self::new()
    }
}

pub struct FeederClient<'a> {
    channel: &'a FeederChannel,
}

impl<'a> FeederClient<'a> {
    pub fn new(channel: &'a FeederChannel) -> Self {
        Self { channel }
    }

    pub async fn set_config(&mut self, config: FeederConfig) -> Result<()> {
        self.channel
            .command_channel
            .send(FeederCommand::SetConfig(config))
            .await;
        let response = self.channel.response_channel.receive().await?;
        match response {
            Some(_) => Err(Error::InvalidFeederCommandResponse),
            None => Ok(()),
        }
    }

    pub async fn get_config(&mut self) -> Result<FeederConfig> {
        self.channel
            .command_channel
            .send(FeederCommand::GetConfig())
            .await;
        let response = self.channel.response_channel.receive().await?;
        match response {
            Some(config) => Ok(config),
            None => Err(Error::InvalidFeederCommandResponse),
        }
    }

    pub async fn set_servo_angle(&mut self, angle: Value) -> Result<()> {
        self.channel
            .command_channel
            .send(FeederCommand::SetServoAngle(angle))
            .await;
        let response = self.channel.response_channel.receive().await?;
        match response {
            Some(_) => Err(Error::InvalidFeederCommandResponse),
            None => Ok(()),
        }
    }

    pub async fn advance(&mut self, length: Option<Value>, override_error: bool) -> Result<()> {
        self.channel
            .command_channel
            .send(FeederCommand::Advance {
                length,
                override_error,
            })
            .await;
        let response = self.channel.response_channel.receive().await?;
        match response {
            Some(_) => Err(Error::InvalidFeederCommandResponse),
            None => Ok(()),
        }
    }

    pub async fn enable(&mut self, state: bool) -> Result<()> {
        self.channel
            .command_channel
            .send(FeederCommand::Enable(state))
            .await;
        let response = self.channel.response_channel.receive().await?;
        match response {
            Some(_) => Err(Error::InvalidFeederCommandResponse),
            None => Ok(()),
        }
    }

    #[cfg(test)]
    pub async fn shutdown(&mut self) {
        self.channel
            .command_channel
            .send(FeederCommand::Shutdown)
            .await;
    }
}

pub struct Feeder<S: Servo, I: Input> {
    servo: S,
    feedback: I,
    config: FeederConfig,
    enabled: bool,
}

impl<S: Servo, I: Input> Feeder<S, I> {
    pub fn new(servo: S, feedback: I) -> Self {
        let limits = servo.get_pwm_limits();
        let config = FeederConfig {
            pwm_0: limits.zero,
            pwm_180: limits.one_eighty,
            ..Default::default()
        };

        Self {
            servo,
            feedback,
            config,
            enabled: false,
        }
    }

    pub async fn run(&mut self, channel: &FeederChannel) {
        loop {
            let response = match channel.command_channel.receive().await {
                FeederCommand::SetConfig(config) => self.set_config(config).map(|()| None),
                FeederCommand::GetConfig() => Ok(Some(self.get_config())),
                FeederCommand::SetServoAngle(angle) => self.set_servo_angle(angle).map(|()| None),
                FeederCommand::Advance {
                    length,
                    override_error,
                } => self.advance(length, override_error).await.map(|()| None),
                FeederCommand::Enable(state) => {
                    self.enable(state);
                    Ok(None)
                }
                #[cfg(test)]
                FeederCommand::Shutdown => return,
            };
            channel.response_channel.send(response).await;
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

    pub async fn advance(&mut self, _length: Option<Value>, override_error: bool) -> Result<()> {
        let override_error = override_error || self.config.ignore_feeback_pin;
        if !override_error && self.feedback.get_state().await {
            return Err(Error::FeederNotReady);
        }

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
