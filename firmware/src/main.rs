#![no_std]
#![no_main]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PWM_CH0, USB};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_io_async::Write;
use fixed::traits::ToFixed;
use gcode::Mnemonic;
use rp2040_flash::flash;
use usb::{GCodeCommand, GCodeCommandChannel, GCodeCommandReceiver};
use {defmt_rtt as _, panic_probe as _};

mod usb;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut pwm_config: pwm::Config = Default::default();
    pwm_config.divider = 255.to_fixed();
    pwm_config.top = 9804;

    let pwm = Pwm::new_output_a(p.PWM_CH0, p.PIN_16, pwm_config);

    let _jedec_id: u32 = unsafe { cortex_m::interrupt::free(|_cs| flash::flash_jedec_id(true)) };
    let mut unique_id = [0u8; 8];
    unsafe { cortex_m::interrupt::free(|_cs| flash::flash_unique_id(&mut unique_id, true)) };

    let mut cdc_output_pipe = Pipe::<NoopRawMutex, 256>::new();
    let (gcode_output_reader, gcode_output_writer) = cdc_output_pipe.split();

    let gcode_command_channel = GCodeCommandChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_command_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let mut gcode_handler = GCodeHandler::new(pwm, gcode_output_writer);
    let gcode_future = gcode_handler.run(gcode_command_channel.receiver());

    join(usb_future, gcode_future).await;
}

struct GCodeHandler<'a, W: Write> {
    pwm: Pwm<'a, PWM_CH0>,
    output: W,
}

impl<'a, W: Write> GCodeHandler<'a, W> {
    pub fn new(pwm: Pwm<'a, PWM_CH0>, output: W) -> Self {
        Self { pwm, output }
    }

    pub async fn run(&mut self, receiver: GCodeCommandReceiver<'_, 2>) {
        loop {
            let command = receiver.receive().await;
            match (
                command.mnemonic(),
                command.major_number(),
                command.minor_number(),
            ) {
                (Mnemonic::General, 0, 0) => self.handle_g0(command).await,
                (Mnemonic::Miscellaneous, 600, 0) => info!("got m600"),
                (Mnemonic::Miscellaneous, 601, 0) => info!("got m601"),
                (Mnemonic::Miscellaneous, 620, 0) => info!("got m620"),
                (_mnemonic, _major, _minor) => info!("got unknown gcode"),
            }

            // ignore error
            let _ = self.output.write_all(b"ok\n").await;
        }
    }

    async fn handle_g0(&mut self, command: GCodeCommand) {
        for arg in command.arguments() {
            if arg.letter == 'A' {
                let ms_per_degree = 1.0f32 / 180.0;
                let cycles_per_ms = 9804.0f32 / 20.0;
                let value = arg.value.clamp(0.0, 180.0);
                let value_ms = 1.0 + (ms_per_degree * value);
                let cycles = cycles_per_ms * value_ms;

                info!("Setting pwm compare to {}", cycles as u16);
                let mut pwm_config: pwm::Config = Default::default();
                pwm_config.divider = 255.to_fixed();
                pwm_config.top = 9804;
                pwm_config.compare_a = cycles as u16;
                self.pwm.set_config(&pwm_config);
            }
        }
    }
}
