#![no_std]
#![no_main]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

use embassy_executor::Spawner;
use embassy_futures::join::{join3, join4};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use pnpfeeder::{Feeder, FeederChannel, FeederClient, GCodeEventChannel, GCodeHandler};
use rp2040_0816::{gpio_input::GpioInput, pwm_servo::PwmServo, usb};
use rp2040_flash::flash;

use {defmt_rtt as _, panic_probe as _};

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

    let gcode_event_channel = GCodeEventChannel::<2>::new();

    let usb = usb::Usb::new(gcode_output_reader, gcode_event_channel.sender());
    let usb_future = usb.run(p.USB, Irqs, &unique_id);

    let mut feeder_0 = Feeder::new(
        PwmServo::new_a(p.PWM_CH0, p.PIN_16),
        GpioInput::new(gpio::Input::new(p.PIN_17, Pull::Up)),
    );
    let mut feeder_1 = Feeder::new(
        PwmServo::new_a(p.PWM_CH1, p.PIN_18),
        GpioInput::new(gpio::Input::new(p.PIN_19, Pull::Up)),
    );
    let mut feeder_2 = Feeder::new(
        PwmServo::new_a(p.PWM_CH2, p.PIN_20),
        GpioInput::new(gpio::Input::new(p.PIN_21, Pull::Up)),
    );
    let mut feeder_3 = Feeder::new(
        PwmServo::new_a(p.PWM_CH7, p.PIN_14),
        GpioInput::new(gpio::Input::new(p.PIN_15, Pull::Up)),
    );

    let channels = [
        &FeederChannel::new(),
        &FeederChannel::new(),
        &FeederChannel::new(),
        &FeederChannel::new(),
    ];

    let feeder_future = join4(
        feeder_0.run(channels[0]),
        feeder_1.run(channels[1]),
        feeder_2.run(channels[2]),
        feeder_3.run(channels[3]),
    );

    let mut gcode_handler = GCodeHandler::new(
        [
            FeederClient::new(channels[0]),
            FeederClient::new(channels[1]),
            FeederClient::new(channels[2]),
            FeederClient::new(channels[3]),
        ],
        gcode_output_writer,
    );
    let gcode_future = gcode_handler.run(gcode_event_channel.receiver());

    join3(usb_future, gcode_future, feeder_future).await;
}
