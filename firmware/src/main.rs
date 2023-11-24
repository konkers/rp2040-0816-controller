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
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_io_async::Write;
use gcode::Mnemonic;
use rp2040_flash::flash;
use usb::{GCodeCommandChannel, GCodeCommandReceiver};
use {defmt_rtt as _, panic_probe as _};

mod usb;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let _jedec_id: u32 = unsafe { cortex_m::interrupt::free(|_cs| flash::flash_jedec_id(true)) };
    let mut unique_id = [0u8; 8];
    unsafe { cortex_m::interrupt::free(|_cs| flash::flash_unique_id(&mut unique_id, true)) };

    let mut cdc_output_pipe = Pipe::<NoopRawMutex, 256>::new();
    let (gcode_output_reader, gcode_output_writer) = cdc_output_pipe.split();

    let gcode_command_channel = GCodeCommandChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_command_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let gcode_future = handle_gcode(gcode_command_channel.receiver(), gcode_output_writer);

    join(usb_future, gcode_future).await;
}

pub async fn handle_gcode(receiver: GCodeCommandReceiver<'_, 2>, mut output: impl Write) {
    loop {
        let command = receiver.receive().await;
        match (
            command.mnemonic(),
            command.major_number(),
            command.minor_number(),
        ) {
            (Mnemonic::General, 0, 0) => info!("got g0"),
            (Mnemonic::Miscellaneous, 600, 0) => info!("got m600"),
            (Mnemonic::Miscellaneous, 601, 0) => info!("got m601"),
            (Mnemonic::Miscellaneous, 620, 0) => info!("got m620"),
            (_mnemonic, _major, _minor) => info!("got unknown gcode"),
        }

        // ignore error
        let _ = output.write_all(b"ok\n").await;
    }
}
