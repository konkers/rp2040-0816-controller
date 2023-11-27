use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{self, Channel},
};
use embassy_time::{Duration, Instant, Timer};

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

struct FeedbackInputRecognizer {
    last_event: Option<(bool, Instant)>,
}

impl FeedbackInputRecognizer {
    const MIN_PULSE: Duration = Duration::from_millis(50);
    const MAX_PULSE: Duration = Duration::from_millis(500);
    fn new() -> Self {
        Self { last_event: None }
    }

    fn update(&mut self, state: bool) -> bool {
        let now = Instant::now();
        let mut should_feed = false;
        if let Some((last_state, last_time)) = self.last_event {
            let pulse_duration = now.saturating_duration_since(last_time);
            if !last_state && (Self::MIN_PULSE..=Self::MAX_PULSE).contains(&pulse_duration) {
                should_feed = true;
            }
        }
        self.last_event = Some((state, now));
        should_feed
    }

    fn reset(&mut self) {
        self.last_event = None;
    }
}

pub struct Feeder<S: Servo, I: Input> {
    servo: S,
    feedback: I,
    config: FeederConfig,
    enabled: bool,
    feedback_recognizer: FeedbackInputRecognizer,
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
            feedback_recognizer: FeedbackInputRecognizer::new(),
        }
    }

    pub async fn run(&mut self, channel: &FeederChannel) {
        loop {
            match select(
                self.feedback.wait_for_state_change(),
                channel.command_channel.receive(),
            )
            .await
            {
                Either::First(()) => self.handle_feedback_state_change().await,
                Either::Second(command) => {
                    if self.handle_command(channel, command).await {
                        return;
                    }
                }
            }
        }
    }
    async fn handle_feedback_state_change(&mut self) {
        if self
            .feedback_recognizer
            .update(self.feedback.get_state().await)
        {
            let _ = self.advance(None, true).await;
        }
    }

    async fn handle_command(&mut self, channel: &FeederChannel, command: FeederCommand) -> bool {
        let response = match command {
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
            FeederCommand::Shutdown => return true,
        };
        channel.response_channel.send(response).await;

        false
    }

    fn set_config(&mut self, config: FeederConfig) -> Result<()> {
        self.servo.set_pwm_limits(PwmLimits {
            zero: config.pwm_0,
            one_eighty: config.pwm_180,
        })?;
        self.config = config;
        Ok(())
    }

    fn get_config(&self) -> FeederConfig {
        self.config.clone()
    }

    fn set_servo_angle(&mut self, angle: Value) -> Result<()> {
        if self.enabled {
            self.servo.set_angle(angle)
        } else {
            Err(Error::FeederDisabled)
        }
    }

    async fn settle(&self) {
        Timer::after_micros(self.config.settle_time as u64 * 1000).await;
    }

    async fn advance(&mut self, _length: Option<Value>, override_error: bool) -> Result<()> {
        let override_error = override_error || self.config.ignore_feeback_pin;
        if !override_error && self.feedback.get_state().await {
            return Err(Error::FeederNotReady);
        }

        self.set_servo_angle(self.config.advanced_angle)?;
        self.settle().await;
        self.set_servo_angle(self.config.retract_angle)?;
        self.settle().await;

        // Reset the feedback as a button recognizer since we just fed.
        self.feedback_recognizer.reset();

        Ok(())
    }

    fn enable(&mut self, enabled: bool) {
        self.enabled = enabled
    }
}
