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
mod servo;
pub use feeder::{Feeder, FeederConfig};
pub use servo::{PwmLimits, Servo};

pub type Value = FixedI32<U16>;
pub type Value64 = FixedI64<U16>;

pub struct Types;
impl BufferTypes<Value> for Types {
    type Words = Vec<Word, 10>;
}

pub type Word = fixed_gcode::Word<Value>;
pub type Line = fixed_gcode::Line<Value, Types>;

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
        }
    }
}

pub type GCodeLineChannel<const N: usize> = Channel<NoopRawMutex, Line, N>;
pub type GCodeLineReceiver<'a, const N: usize> = channel::Receiver<'a, NoopRawMutex, Line, N>;
pub type GCodeLineSender<'a, const N: usize> = channel::Sender<'a, NoopRawMutex, Line, N>;

pub struct GCodeHandler<'a, 'b: 'a, W: Write> {
    feeders: &'a mut [Feeder<'b>],
    output: W,
}

macro_rules! word {
    ($letter:literal, $value:literal) => {
        Word::new($letter, $value)
    };
}

impl<'a, 'b: 'a, W: Write> GCodeHandler<'a, 'b, W> {
    pub fn new(feeders: &'a mut [Feeder<'b>], output: W) -> Self {
        Self { feeders, output }
    }

    pub async fn run(&mut self, receiver: GCodeLineReceiver<'_, 2>) {
        loop {
            let line = receiver.receive().await;

            let Some(command) = line.command() else {
                continue;
            };

            // Use M999 to allow tests to exit the loop.
            #[cfg(test)]
            if *command == word!('M', 999) {
                return;
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
        }
    }

    fn resolve_feeder(&mut self, index: Option<usize>) -> Result<(usize, &mut Feeder<'b>)> {
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
            feeder.set_servo_angle(angle)?;
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
                feeder.enable(status);
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
        let mut config = feeder.get_config();

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

        feeder.set_config(config)?;

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

        let (index, feeder) = self.resolve_feeder(index)?;
        let config = feeder.get_config();

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
    use embassy_futures::join::join;
    use std::{string::String, vec::Vec};

    use super::*;

    struct FakeServo {
        limits: servo::PwmLimits,
        positions: Vec<Value>,
    }

    impl FakeServo {
        const COUNTS_PER_PERIOD: u16 = 9804;
        fn new() -> Self {
            let counts_per_ms = Value::from_num(Self::COUNTS_PER_PERIOD) / Value::from_num(20.0);
            let zero = Value::from_num(1.0) * counts_per_ms;
            let one_eighty = Value::from_num(2.0) * counts_per_ms;
            Self {
                limits: PwmLimits { zero, one_eighty },
                positions: Vec::new(),
            }
        }
    }

    impl Servo for FakeServo {
        fn set_angle(&mut self, angle: Value) -> Result<()> {
            self.positions.push(angle);
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

    async fn run_handler<W: Write>(
        feeders: &mut [Feeder<'_>],
        output: W,
        line_reciever: GCodeLineReceiver<'_, 2>,
    ) {
        let mut gcode_handler = GCodeHandler::new(feeders, output);
        gcode_handler.run(line_reciever).await;
    }
    async fn run_test_harness(
        line_reciever: GCodeLineReceiver<'_, 2>,
    ) -> ([FakeServo; 2], Vec<u8>) {
        let mut servo_0 = FakeServo::new();
        let mut servo_1 = FakeServo::new();
        let feeder_0 = Feeder::new(&mut servo_0);
        let feeder_1 = Feeder::new(&mut servo_1);
        let mut feeders = [feeder_0, feeder_1];
        let mut output = Vec::<u8>::new();
        run_handler(&mut feeders, &mut output, line_reciever).await;
        ([servo_0, servo_1], output)
    }

    #[futures_test::test]
    async fn test_harnes_exits_on_m999() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move { line_sender.send("M999".parse().unwrap()).await };
        let (_, _) = join(test_harness_future, test_future).await;
    }

    #[futures_test::test]
    async fn feeder_doesnt_move_before_enabled() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send("G0 N1 A120.0".parse().unwrap()).await;
            line_sender.send("M999".parse().unwrap()).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;
        assert_eq!("error: feeder disabled\n", String::from_utf8_lossy(&output));
        assert!(servos[0].positions.is_empty());
        assert!(servos[1].positions.is_empty());
    }

    #[futures_test::test]
    async fn g0_moves_correct_servo() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send("M610 S1".parse().unwrap()).await;
            line_sender.send("G0 N1 A120.0".parse().unwrap()).await;
            line_sender.send("M999".parse().unwrap()).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;
        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].positions.is_empty());
        assert_eq!(servos[1].positions, vec![Value::from_num(120.0)]);
    }

    #[futures_test::test]
    async fn m600_advances_feeder() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send("M610 S1".parse().unwrap()).await;
            line_sender.send("M600 N1".parse().unwrap()).await;
            line_sender.send("M999".parse().unwrap()).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;

        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].positions.is_empty());

        // Servo 1 should go to the advanced angle then the retract angle.
        let feeder_config_defaults: FeederConfig = Default::default();
        assert_eq!(
            servos[1].positions,
            vec![
                feeder_config_defaults.advanced_angle,
                feeder_config_defaults.retract_angle
            ]
        );
    }

    #[futures_test::test]
    async fn m620_changes_feeder_angles() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender.send("M610 S1".parse().unwrap()).await;
            line_sender.send("M620 N1 A122 C22".parse().unwrap()).await;
            line_sender.send("M600 N1".parse().unwrap()).await;
            line_sender.send("M999".parse().unwrap()).await;
        };
        let ((servos, output), _) = join(test_harness_future, test_future).await;

        println!("{}", String::from_utf8_lossy(&output));
        assert!(servos[0].positions.is_empty());
        assert_eq!(
            servos[1].positions,
            vec![Value::from_num(122), Value::from_num(22)]
        );
    }

    #[futures_test::test]
    async fn m621_reflects_m620_changes() {
        let gcode_channel = GCodeLineChannel::<2>::new();
        let test_harness_future = run_test_harness(gcode_channel.receiver());
        let line_sender = gcode_channel.sender();
        let test_future = async move {
            line_sender
                .send("M620 N1 A1 B2 C3 F4 U5 V6 W7 X1".parse().unwrap())
                .await;
            line_sender.send("M621 N1".parse().unwrap()).await;
            line_sender.send("M999".parse().unwrap()).await;
        };
        let ((_servos, output), _) = join(test_harness_future, test_future).await;

        let output = String::from_utf8_lossy(&output);
        assert_eq!(output, "ok\nM620 N1 A1 B2 C3 F4 U5 V6 W7 X1\nok\n");
    }
}
