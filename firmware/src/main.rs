#![no_std]
#![no_main]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

use core::fmt::{Display, Write as _};
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embassy_usb::driver::EndpointError;
use embedded_io_async::Write;
use gcode::Mnemonic;
use heapless::String;
use rp2040_flash::flash;
use {defmt_rtt as _, panic_probe as _};

mod feeder;
mod servo;
mod usb;

use feeder::Feeder;
use servo::PwmServo;
use usb::{GCodeCommand, GCodeCommandChannel, GCodeCommandReceiver};

enum Error {
    Disconnected,
    InputBufferOverflow,
    Io,
    AngleOutOfRange,
    PwmValueOutOfRange,
    UnsupportedCommand(Mnemonic, u32, u32),
    NoIndex,
    InvalidIndex(usize),
    InvalidArgument(char),
    FeederDisabled,
}

type Result<T> = core::result::Result<T, Error>;

impl From<EndpointError> for Error {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Error::Disconnected {},
        }
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Disconnected => write!(f, "disconnected"),
            Self::InputBufferOverflow => write!(f, "input buffer overflow"),
            Self::Io => write!(f, "IO error"),
            Self::AngleOutOfRange => write!(f, "angle out of range"),
            Self::PwmValueOutOfRange => write!(f, "pwm value out of range"),
            Self::UnsupportedCommand(mnemonic, major, minor) => {
                if *minor != 0 {
                    write!(f, "unsupported command {}{}.{}", mnemonic, major, minor)
                } else {
                    write!(f, "unsupported command {}{}", mnemonic, major)
                }
            }
            Self::NoIndex => write!(f, "no index specified"),
            Self::InvalidIndex(index) => write!(f, "no feeder {}", index),
            Self::InvalidArgument(char) => write!(f, "invalid argument type {}", char),
            Self::FeederDisabled => write!(f, "feeder disabled"),
        }
    }
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut servo = PwmServo::new_a(p.PWM_CH0, p.PIN_16);
    let feeder = Feeder::new(&mut servo);
    let mut feeders = [feeder];

    let _jedec_id: u32 = unsafe { cortex_m::interrupt::free(|_cs| flash::flash_jedec_id(true)) };
    let mut unique_id = [0u8; 8];
    unsafe { cortex_m::interrupt::free(|_cs| flash::flash_unique_id(&mut unique_id, true)) };

    let mut cdc_output_pipe = Pipe::<NoopRawMutex, 256>::new();
    let (gcode_output_reader, gcode_output_writer) = cdc_output_pipe.split();

    let gcode_command_channel = GCodeCommandChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_command_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let mut gcode_handler = GCodeHandler::new(&mut feeders, gcode_output_writer);
    let gcode_future = gcode_handler.run(gcode_command_channel.receiver());

    join(usb_future, gcode_future).await;
}

struct GCodeHandler<'a, 'b: 'a, W: Write> {
    feeders: &'a mut [Feeder<'b>],
    output: W,
}

impl<'a, 'b: 'a, W: Write> GCodeHandler<'a, 'b, W> {
    pub fn new(feeders: &'a mut [Feeder<'b>], output: W) -> Self {
        Self { feeders, output }
    }

    pub async fn run(&mut self, receiver: GCodeCommandReceiver<'_, 2>) {
        loop {
            let command = receiver.receive().await;

            let ret = match (
                command.mnemonic(),
                command.major_number(),
                command.minor_number(),
            ) {
                (Mnemonic::General, 0, 0) => self.handle_g0(command).await,
                (Mnemonic::Miscellaneous, 600, 0) => self.handle_m600(command).await,
                (Mnemonic::Miscellaneous, 610, 0) => self.handle_m610(command).await,
                (Mnemonic::Miscellaneous, 620, 0) => self.handle_m620(command).await,
                (Mnemonic::Miscellaneous, 621, 0) => self.handle_m621(command).await,
                (mnemonic, major, minor) => Err(Error::UnsupportedCommand(mnemonic, major, minor)),
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

    async fn handle_g0(&mut self, command: GCodeCommand) -> Result<()> {
        let mut index = None;
        let mut angle = None;
        for arg in command.arguments() {
            match arg.letter {
                'I' => index = Some(arg.value as usize),
                'A' => angle = Some(arg.value),
                _ => return Err(Error::InvalidArgument(arg.letter)),
            }
        }

        let (index, feeder) = self.resolve_feeder(index)?;
        if let Some(angle) = angle {
            info!("setting F{} to A{}", index, angle);
            feeder.set_servo_angle(angle)?;
        }

        Ok(())
    }

    async fn handle_m600(&mut self, command: GCodeCommand) -> Result<()> {
        let mut index = None;
        let mut feed_length = None;
        let mut override_error = false;

        for arg in command.arguments() {
            match arg.letter {
                'I' => index = Some(arg.value as usize),
                'F' => feed_length = Some(arg.value),
                'X' => override_error = !(arg.value == 0.0),
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        let (_, feeder) = self.resolve_feeder(index)?;

        feeder.advance(feed_length, override_error).await?;

        Ok(())
    }

    async fn handle_m610(&mut self, command: GCodeCommand) -> Result<()> {
        let mut status = None;

        for arg in command.arguments() {
            match arg.letter {
                'S' => status = Some(!(arg.value == 0.0)),
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

    async fn handle_m620(&mut self, command: GCodeCommand) -> Result<()> {
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
                'I' => index = Some(arg.value as usize),
                'A' => advanced_angle = Some(arg.value),
                'B' => half_advanced_angle = Some(arg.value),
                'C' => retract_angle = Some(arg.value),
                'F' => feed_length = Some(arg.value),
                'U' => settle_time = Some(arg.value as u32),
                'V' => pwm_0 = Some(arg.value as u16),
                'W' => pwm_180 = Some(arg.value as u16),
                'X' => ignore_feeback_pin = Some(!(arg.value == 0.0)),
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

    async fn handle_m621(&mut self, command: GCodeCommand) -> Result<()> {
        let mut index = None;
        for arg in command.arguments() {
            match arg.letter {
                'I' => index = Some(arg.value as usize),
                letter => return Err(Error::InvalidArgument(letter)),
            }
        }

        let (index, feeder) = self.resolve_feeder(index)?;
        let config = feeder.get_config();

        let mut s: String<64> = String::new();
        writeln!(
            s,
            "M620 I{} A{} B{} C{} F{} U{} V{} W{} X{}",
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
