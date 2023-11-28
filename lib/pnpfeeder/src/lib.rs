#![feature(type_alias_impl_trait)]
#![cfg_attr(not(feature = "std"), no_std)]

use az::Cast;
use core::fmt::{Display, Write as _};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{self, Channel},
};
use embedded_io_async::Write;
use fixed::FixedI32;
use fixed::{types::extra::U16, FixedI64};
use fixed_gcode::BufferTypes;
use heapless::{String, Vec};

mod feeder;
mod input;
mod servo;

pub use feeder::{Feeder, FeederChannel, FeederClient, FeederConfig};
pub use input::Input;
pub use servo::{PwmLimits, Servo};

pub type Value = FixedI32<U16>;
pub type Value64 = FixedI64<U16>;

pub struct Types;
impl BufferTypes<Value> for Types {
    type Words = Vec<Word, 10>;
}

pub type Word = fixed_gcode::Word<Value>;
pub type Line = fixed_gcode::Line<Value, Types>;

#[derive(Debug)]
pub enum Error {
    Disconnected,
    InputBufferOverflow,
    Io,
    AngleOutOfRange,
    PwmValueOutOfRange,
    UnsupportedCommand(Word),
    NoIndex,
    InvalidIndex(usize),
    InvalidArgument(char),
    FeederDisabled,
    FixedPointError,
    InvalidFeederCommandResponse,
    FeederNotReady,
}

pub type Result<T> = core::result::Result<T, Error>;

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Disconnected => write!(f, "disconnected"),
            Self::InputBufferOverflow => write!(f, "input buffer overflow"),
            Self::Io => write!(f, "IO error"),
            Self::AngleOutOfRange => write!(f, "angle out of range"),
            Self::PwmValueOutOfRange => write!(f, "pwm value out of range"),
            Self::UnsupportedCommand(word) => write!(f, "unsupported command {word}"),
            Self::NoIndex => write!(f, "no index specified"),
            Self::InvalidIndex(index) => write!(f, "no feeder {}", index),
            Self::InvalidArgument(char) => write!(f, "invalid argument type {}", char),
            Self::FeederDisabled => write!(f, "feeder disabled"),
            Self::FixedPointError => write!(f, "fixed point error"),
            Self::InvalidFeederCommandResponse => write!(f, "invalid feeder command respons"),
            Self::FeederNotReady => write!(f, "feeder not ready"),
        }
    }
}

pub enum GCodeEvent {
    Connect,
    Disconnect,
    Line(Line),
}

pub type GCodeEventChannel<const N: usize> = Channel<NoopRawMutex, GCodeEvent, N>;
pub type GCodeEventReceiver<'a, const N: usize> =
    channel::Receiver<'a, NoopRawMutex, GCodeEvent, N>;
pub type GCodeEventSender<'a, const N: usize> = channel::Sender<'a, NoopRawMutex, GCodeEvent, N>;

pub struct GCodeHandler<'a, W: Write, const N: usize> {
    feeders: [FeederClient<'a>; N],
    output: W,
}

macro_rules! word {
    ($letter:literal, $value:literal) => {
        Word::new($letter, $value)
    };
}

impl<'a, W: Write, const N: usize> GCodeHandler<'a, W, N> {
    pub fn new(feeders: [FeederClient<'a>; N], output: W) -> Self {
        Self { feeders, output }
    }

    pub async fn run(&mut self, receiver: GCodeEventReceiver<'_, 2>) {
        loop {
            let exit = match receiver.receive().await {
                GCodeEvent::Connect => self.handle_connect().await,
                GCodeEvent::Disconnect => self.handle_disconnect().await,
                GCodeEvent::Line(line) => self.handle_line(line).await,
            };
            if exit {
                break;
            }
        }
    }

    pub async fn handle_connect(&mut self) -> bool {
        let _ = self.output.write_all(b"saved settings:\n").await;
        for index in 0..self.feeders.len() {
            let _ = self.output_feeder_config(Some(index)).await; // Ignore errors on connect.
        }
        let _ = self.output.write_all(b"ready\n").await;
        false
    }

    pub async fn handle_disconnect(&mut self) -> bool {
        // Disable feeders on disconnect
        for feeder in self.feeders.iter_mut() {
            feeder.enable(false).await.ok(); // Ignore disable errors on disconnect.
        }
        false
    }

    pub async fn handle_line(&mut self, line: Line) -> bool {
        let Some(command) = line.command() else {
            return false;
        };

        // Use M999 to allow tests to exit the loop.
        #[cfg(test)]
        if *command == word!('M', 999) {
            for feeder in self.feeders.iter_mut() {
                feeder.shutdown().await;
            }
            return true;
        }

        let ret = if *command == word!('G', 0) {
            self.handle_g0(line).await
        } else if *command == word!('M', 600) {
            self.handle_m600(line).await
        } else if *command == word!('M', 610) {
            self.handle_m610(line).await
        } else if *command == word!('M', 620) {
            self.handle_m620(line).await
        } else if *command == word!('M', 621) {
            self.handle_m621(line).await
        } else {
            Err(Error::UnsupportedCommand(command.clone()))
        };

        match ret {
            Ok(_) => {
                let _ = self.output.write_all(b"ok\n").await;
            }
            Err(e) => {
                let mut s = String::<64>::new();
                writeln!(s, "error: {}", e).ok();
                let _ = self.output.write_all(s.as_bytes()).await;
            }
        }
        false
    }

    fn resolve_feeder<'b>(
        &'b mut self,
        index: Option<usize>,
    ) -> Result<(usize, &'b mut FeederClient<'a>)>
    where
        'a: 'b,
    {
        let index = index.ok_or(Error::NoIndex)?;

        if index >= self.feeders.len() {
            return Err(Error::InvalidIndex(index));
        }

        Ok((index, &mut self.feeders[index]))
    }

    async fn handle_g0(&mut self, command: Line) -> Result<()> {
        let mut index = None;
        let mut angle = None;
        for arg in command.arguments() {
            match arg.letter {
                'N' => index = Some(arg.value.cast()),
                'A' => angle = Some(arg.value.cast()),
                _ => return Err(Error::InvalidArgument(arg.letter)),
            }
        }

        let (_, feeder) = self.resolve_feeder(index)?;
        if let Some(angle) = angle {
            feeder.set_servo_angle(angle).await?;
        }

        Ok(())
    }

    async fn handle_m600(&mut self, command: Line) -> Result<()> {
        let mut index = None;
        let mut feed_length = None;
        let mut override_error = false;

        for arg in command.arguments() {
            match arg.letter {
                'N' => index = Some(arg.value.cast()),
                'F' => feed_length = Some(arg.value.cast()),
                'X' => override_error = arg.value != 0,
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        let (_, feeder) = self.resolve_feeder(index)?;

        feeder.advance(feed_length, override_error).await?;

        Ok(())
    }

    async fn handle_m610(&mut self, command: Line) -> Result<()> {
        let mut status = None;

        for arg in command.arguments() {
            match arg.letter {
                'S' => status = Some(arg.value != 0.0),
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        if let Some(status) = status {
            for feeder in self.feeders.iter_mut() {
                feeder.enable(status).await?;
            }
        }

        Ok(())
    }

    async fn handle_m620(&mut self, command: Line) -> Result<()> {
        let mut index = None;
        let mut advanced_angle = None;
        let mut half_advanced_angle = None;
        let mut retract_angle = None;
        let mut feed_length = None;
        let mut settle_time = None;
        let mut pwm_0 = None;
        let mut pwm_180 = None;
        let mut ignore_feeback_pin = None;

        for arg in command.arguments() {
            match arg.letter {
                'N' => index = Some(arg.value.cast()),
                'A' => advanced_angle = Some(arg.value.cast()),
                'B' => half_advanced_angle = Some(arg.value.cast()),
                'C' => retract_angle = Some(arg.value.cast()),
                'F' => feed_length = Some(arg.value.cast()),
                'U' => settle_time = Some(arg.value.cast()),
                'V' => pwm_0 = Some(arg.value.cast()),
                'W' => pwm_180 = Some(arg.value.cast()),
                'X' => ignore_feeback_pin = Some(arg.value != 0),
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        let (_, feeder) = self.resolve_feeder(index)?;
        let mut config = feeder.get_config().await?;

        macro_rules! handle_parameter {
            ($parameter:ident) => {
                if let Some(val) = $parameter {
                    config.$parameter = val;
                }
            };
        }

        handle_parameter!(advanced_angle);
        handle_parameter!(half_advanced_angle);
        handle_parameter!(retract_angle);
        handle_parameter!(feed_length);
        handle_parameter!(settle_time);
        handle_parameter!(pwm_0);
        handle_parameter!(pwm_180);
        handle_parameter!(ignore_feeback_pin);

        feeder.set_config(config).await?;

        Ok(())
    }

    async fn handle_m621(&mut self, command: Line) -> Result<()> {
        let mut index = None;
        for arg in command.arguments() {
            match arg.letter {
                'N' => index = Some(arg.value.cast()),
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        self.output_feeder_config(index).await?;

        Ok(())
    }

    async fn output_feeder_config(&mut self, index: Option<usize>) -> Result<()> {
        let (index, feeder) = self.resolve_feeder(index)?;
        let config = feeder.get_config().await?;

        let mut s: String<64> = String::new();
        writeln!(
            s,
            "M620 N{} A{} B{} C{} F{} U{} V{} W{} X{}",
            index,
            config.advanced_angle,
            config.half_advanced_angle,
            config.retract_angle,
            config.feed_length,
            config.settle_time,
            config.pwm_0,
            config.pwm_180,
            if config.ignore_feeback_pin { 1 } else { 0 }
        )
        .ok();
        let _ = self.output.write_all(s.as_bytes()).await;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use alloc::sync::Arc;
    use embassy_futures::join::{join, join_array};
    use embassy_time::Timer;
    use std::{string::String, sync::Mutex, vec::Vec};

    use super::*;

    struct FakeServo {
        limits: servo::PwmLimits,
        positions: Arc<Mutex<Vec<Value>>>,
    }

    impl FakeServo {
        const COUNTS_PER_PERIOD: u16 = 9804;
        fn new() -> (Arc<Mutex<Vec<Value>>>, Self) {
            let counts_per_ms = Value::from_num(Self::COUNTS_PER_PERIOD) / Value::from_num(20.0);
            let zero = Value::from_num(1.0) * counts_per_ms;
            let one_eighty = Value::from_num(2.0) * counts_per_ms;

            let positions = Arc::new(Mutex::new(Vec::new()));
            (
                positions.clone(),
                Self {
                    limits: PwmLimits { zero, one_eighty },
                    positions,
                },
            )
        }
    }

    impl Servo for FakeServo {
        fn set_angle(&mut self, angle: Value) -> Result<()> {
            println!("fake servo: set angle {angle}");

            self.positions.lock().unwrap().push(angle);
            Ok(())
        }

        fn set_pwm_limits(&mut self, limits: PwmLimits) -> Result<()> {
            if limits.zero > Self::COUNTS_PER_PERIOD || limits.one_eighty > Self::COUNTS_PER_PERIOD
            {
                return Err(Error::PwmValueOutOfRange);
            }
            self.limits = limits;
            Ok(())
        }

        fn get_pwm_limits(&self) -> PwmLimits {
            self.limits.clone()
        }
    }

    type FakeInputChannel = Channel<NoopRawMutex, bool, 4>;

    struct FakeInput<'a> {
        channel: &'a FakeInputChannel,
        state: bool,
    }

    impl<'a> FakeInput<'a> {
        pub fn new(state: bool, channel: &'a FakeInputChannel) -> Self {
            Self { channel, state }
        }

        async fn poll_state(&mut self) -> bool {
            loop {
                // Consume all update events in queue and return state when empty.
                let Ok(new_state) = self.channel.try_receive() else {
                    return self.state;
                };
                self.state = new_state;
            }
        }

        async fn wait_for_state(&mut self, state: bool) {
            loop {
                let new_state = self.poll_state().await;
                if new_state == state {
                    return;
                }
            }
        }
    }

    impl<'a> Input for FakeInput<'a> {
        async fn wait_for_high(&mut self) {
            self.wait_for_state(true).await
        }

        async fn wait_for_low(&mut self) {
            self.wait_for_state(false).await
        }

        async fn wait_for_state_change(&mut self) {
            self.state = self.channel.receive().await;
        }

        async fn get_state(&mut self) -> bool {
            self.poll_state().await
        }
    }

    async fn run_handler<W: Write>(
        feeders: [FeederClient<'_>; 2],
        output: W,
        line_reciever: GCodeEventReceiver<'_, 2>,
    ) {
        let mut gcode_handler = GCodeHandler::new(feeders, output);
        gcode_handler.run(line_reciever).await;
    }

    async fn run_test_harness(
        line_reciever: GCodeEventReceiver<'_, 2>,
        fake_inputs: &[FakeInputChannel; 2],
    ) -> ([Vec<Value>; 2], Vec<u8>) {
        let (positions_0, servo_0) = FakeServo::new();
        let (positions_1, servo_1) = FakeServo::new();
        let mut feeder_0 = Feeder::new(servo_0, FakeInput::new(false, &fake_inputs[0]));
        let mut feeder_1 = Feeder::new(servo_1, FakeInput::new(false, &fake_inputs[1]));
        let channels = [&FeederChannel::new(), &FeederChannel::new()];
        let feeder_future = join_array([feeder_0.run(channels[0]), feeder_1.run(channels[1])]);
        let mut output = Vec::<u8>::new();
        join(
            feeder_future,
            run_handler(
                [
                    FeederClient::new(channels[0]),
                    FeederClient::new(channels[1]),
                ],
                &mut output,
                line_reciever,
            ),
        )
        .await;
        let positions_0 = positions_0.lock().unwrap().clone();
        let positions_1 = positions_1.lock().unwrap().clone();
        ([positions_0, positions_1], output)
    }

    fn line_event(s: &str) -> GCodeEvent {
        GCodeEvent::Line(s.parse().unwrap())
    }

    #[futures_test::test]
    async fn test_harnes_exits_on_m999() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move { line_sender.send(line_event("M999")).await };
        let (_, _) = join(test_harness_future, test_future).await;
    }

    #[futures_test::test]
    async fn feeder_doesnt_move_before_enabled() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(line_event("G0 N1 A120.0")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;
        assert_eq!("error: feeder disabled\n", String::from_utf8_lossy(&output));
        assert!(servos[0].is_empty());
        assert!(servos[1].is_empty());
    }

    #[futures_test::test]
    async fn g0_moves_correct_servo() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("G0 N1 A120.0")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;
        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].is_empty());
        assert_eq!(servos[1], vec![Value::from_num(120.0)]);
    }

    #[futures_test::test]
    async fn m600_advances_feeder() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("M600 N1")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;

        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].is_empty());

        // Servo 1 should go to the advanced angle then the retract angle.
        let feeder_config_defaults: FeederConfig = Default::default();
        assert_eq!(
            servos[1],
            vec![
                feeder_config_defaults.advanced_angle,
                feeder_config_defaults.retract_angle
            ]
        );
    }

    #[futures_test::test]
    async fn m620_changes_feeder_angles() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("M620 N1 A122 C22")).await;
            line_sender.send(line_event("M600 N1")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;

        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].is_empty());
        assert_eq!(servos[1], vec![Value::from_num(122), Value::from_num(22)]);
    }

    #[futures_test::test]
    async fn m621_reflects_m620_changes() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender
                .send(line_event("M620 N1 A1 B2 C3 F4 U5 V6 W7 X1"))
                .await;
            line_sender.send(line_event("M621 N1")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;

        let output = String::from_utf8_lossy(&output);
        assert_eq!(output, "ok\nM620 N1 A1 B2 C3 F4 U5 V6 W7 X1\nok\n");
    }

    #[futures_test::test]
    async fn feeders_disable_on_disconnect() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(GCodeEvent::Connect).await;
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("G0 N1 A120.0")).await;
            line_sender.send(GCodeEvent::Disconnect).await;
            line_sender.send(GCodeEvent::Connect).await;
            line_sender.send(line_event("G0 N1 A90.0")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;
        println!("{}", String::from_utf8_lossy(&output));
        // Ensure that servo 1 didn't move to 90 after a disconnect.
        assert!(servos[0].is_empty());
        assert_eq!(servos[1], vec![Value::from_num(120.0)]);
    }

    #[futures_test::test]
    async fn settings_output_on_connect() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send(GCodeEvent::Connect).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;
        assert_eq!(String::from_utf8_lossy(&output), "saved settings:\nM620 N0 A135 B107.5 C80 F2 U300 V490.2 W980.4 X0\nM620 N1 A135 B107.5 C80 F2 U300 V490.2 W980.4 X0\nready\n");
    }

    #[futures_test::test]
    async fn advance_returns_error_on_high_feedback() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();

        // drive feedback hgih.
        fake_inputs[0].send(true).await;

        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("M600 N0")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;

        let output = String::from_utf8_lossy(&output);
        assert_eq!(output, "ok\nerror: feeder not ready\n");
    }

    #[futures_test::test]
    async fn advance_respects_override_error_arg() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();

        // drive feedback hgih.
        fake_inputs[0].send(true).await;

        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("M600 N0 X1")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;

        let output = String::from_utf8_lossy(&output);
        assert_eq!(output, "ok\nok\n");
    }

    #[futures_test::test]
    async fn advance_respects_ignore_feedback_pin_config() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();

        // drive feedback high.
        fake_inputs[0].send(true).await;

        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            line_sender.send(line_event("M620 N0 X1")).await;
            line_sender.send(line_event("M600 N0")).await;
            line_sender.send(line_event("M999")).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;

        let output = String::from_utf8_lossy(&output);
        assert_eq!(output, "ok\nok\nok\n");
    }

    #[futures_test::test]
    async fn feedback_pulse_advances_feeder() {
        let gcode_channel = GCodeEventChannel::<2>::new();
        let fake_inputs = [FakeInputChannel::new(), FakeInputChannel::new()];
        let test_harness_future = run_test_harness(gcode_channel.receiver(), &fake_inputs);
        let line_sender = gcode_channel.sender();
        let feedback0 = &fake_inputs[0];

        // Start with switch unpressed.
        feedback0.send(true).await;

        let test_future = async move {
            line_sender.send(line_event("M610 S1")).await;
            // Set to known angles
            line_sender.send(line_event("M620 N0 A122 C22")).await;
            // Press switch

            feedback0.send(false).await;
            Timer::after_micros(250_000).await;
            // Release switch.
            feedback0.send(true).await;

            line_sender.send(line_event("M999")).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;

        println!("{}", String::from_utf8_lossy(&output));
        assert_eq!(servos[0], vec![Value::from_num(122), Value::from_num(22)]);
        assert!(servos[1].is_empty());
    }
}
