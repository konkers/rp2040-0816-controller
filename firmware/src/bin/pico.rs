#![no_std]
#![no_main]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use pnpfeeder::{Feeder, GCodeHandler, GCodeLineChannel};
use rp2040_0816::{pwm_servo::PwmServo, usb};
use rp2040_flash::flash;

use {defmt_rtt as _, panic_probe as _};

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

    let gcode_command_channel = GCodeLineChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_command_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let mut gcode_handler = GCodeHandler::new(&mut feeders, gcode_output_writer);
    let gcode_future = gcode_handler.run(gcode_command_channel.receiver());

    join(usb_future, gcode_future).await;
}