use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{self, Channel},
};
use embassy_time::{Duration, Instant, Timer};
use serde::{Deserialize, Serialize};

use crate::{
    servo::{PwmLimits, Servo},
    Error, Input, Result, Value,
};

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FeederConfig {
    pub advanced_angle: Value,
    pub half_advanced_angle: Value,
    pub retract_angle: Value,
    pub feed_length: Value,
    pub settle_time: u32,
    pub pwm_0: Value,
    pub pwm_180: Value,
    pub ignore_feeback_pin: bool,
    pub always_retract: bool,
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
            always_retract: false,
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
    advance_offset: Value,
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
            advance_offset: Value::from_num(0),
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

    async fn advance(&mut self, length: Option<Value>, override_error: bool) -> Result<()> {
        let override_error = override_error || self.config.ignore_feeback_pin;
        if !override_error && self.feedback.get_state().await {
            return Err(Error::FeederNotReady);
        }

        let mut length = length.unwrap_or(self.config.feed_length);

        // Ensure the the feed length is an even multiple of 2mm.
        if length % Value::from_num(2) != 0 {
            return Err(Error::InvalidFeedLength(length));
        }

        while length > Value::from_num(0) {
            // The feeder can advance in maximum of 4mm increments (the distance between feed
            // holes.  A feed longer than that needs to be broken up into a series of
            // advance/retract cycles.
            //
            // Additionally, in order to support tap with 2mm part spacing, we have a
            // `half_advanced_angle`.  Some feeders can't be retracted from the half advanced
            // state so we need to track the a feed offset and only retract the servo when
            // it reaches 4mm.  This means that sometimes we can only advance 2mm before a retract.

            // Caclulate the maximum amount we can advance this cycle, taking into account the
            // current offset.
            let advance_length = core::cmp::min(Value::from_num(4) - self.advance_offset, length);

            // Caclulate the absolute advance position that the advance length equates to, taking
            // into account the current offset.
            let advance_to = self.advance_offset + advance_length;

            // Depending on the final advace position, advance to either the full or half angle.
            if advance_to == Value::from_num(2) {
                self.set_servo_angle(self.config.half_advanced_angle)?;
            } else {
                self.set_servo_angle(self.config.advanced_angle)?;
            }

            self.settle().await;

            if self.config.always_retract || advance_to == Value::from_num(4) {
                // If either the feeder should retract on every advance of we have reach a 4mm
                // offset, retract the servro and reset the offset.
                self.set_servo_angle(self.config.retract_angle)?;
                self.settle().await;
                self.advance_offset = Value::from_num(0);
            } else {
                // ... otherwise set the offset to our current advance state.
                self.advance_offset = advance_to;
            }

            // Update the length remaining to advance by the amount advanced this cycle.
            length -= advance_length;
        }

        // Reset the feedback as a button recognizer since we just fed.
        self.feedback_recognizer.reset();

        Ok(())
    }

    fn enable(&mut self, enabled: bool) {
        self.enabled = enabled
    }
}
