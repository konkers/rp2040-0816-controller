#![no_std]
#![no_main]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_io_async::Write;
use gcode::Mnemonic;
use heapless::String;
use rp2040_flash::flash;
use servo::{PwmServo, Servo};
use usb::{GCodeCommand, GCodeCommandChannel, GCodeCommandReceiver};
use {defmt_rtt as _, panic_probe as _};

mod servo;
mod usb;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut servo = PwmServo::new_a(p.PWM_CH0, p.PIN_16);

    let _jedec_id: u32 = unsafe { cortex_m::interrupt::free(|_cs| flash::flash_jedec_id(true)) };
    let mut unique_id = [0u8; 8];
    unsafe { cortex_m::interrupt::free(|_cs| flash::flash_unique_id(&mut unique_id, true)) };

    let mut cdc_output_pipe = Pipe::<NoopRawMutex, 256>::new();
    let (gcode_output_reader, gcode_output_writer) = cdc_output_pipe.split();

    let gcode_command_channel = GCodeCommandChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_command_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let mut gcode_handler = GCodeHandler::new(&mut servo, gcode_output_writer);
    let gcode_future = gcode_handler.run(gcode_command_channel.receiver());

    join(usb_future, gcode_future).await;
}

struct GCodeHandler<'a, W: Write> {
    servo: &'a mut dyn Servo,
    output: W,
}

impl<'a, W: Write> GCodeHandler<'a, W> {
    pub fn new(servo: &'a mut dyn Servo, output: W) -> Self {
        Self { servo, output }
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
                (_, _, _) => self.handle_unsupported(command).await,
            };

            if ret.is_ok() {
                // ignore error
                let _ = self.output.write_all(b"ok\n").await;
            }
        }
    }

    async fn handle_unsupported(&mut self, command: GCodeCommand) -> Result<(), ()> {
        let mut s: String<32> = String::new();
        if command.minor_number() == 0 {
            writeln!(
                s,
                "unsupported gcode {}{}",
                command.mnemonic(),
                command.major_number()
            )
            .ok();
        } else {
            writeln!(
                s,
                "unsupported gcode {}{}.{}",
                command.mnemonic(),
                command.major_number(),
                command.minor_number()
            )
            .ok();
        }
        let _ = self.output.write_all(s.as_bytes()).await;

        Err(())
    }
    async fn handle_g0(&mut self, command: GCodeCommand) -> Result<(), ()> {
        for arg in command.arguments() {
            if arg.letter == 'A' {
                if self.servo.set_angle(arg.value).is_err() {
                    let _ = self.output.write_all(b"invalid angle\n").await;
                    return Err(());
                }
            } else {
                let _ = self.output.write_all(b"invalid axis\n").await;
                return Err(());
            }
        }

        Ok(())
    }
}
