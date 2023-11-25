use base64::{engine::general_purpose, Engine as _};
use embassy_futures::join::join;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_rp::Peripheral;
use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
use embassy_usb::{Builder, Config};
use embedded_io_async::Read;
use heapless::{String, Vec};

mod gcode_interface;
mod picotool;

pub use gcode_interface::{
    GCodeCommand, GCodeCommandChannel, GCodeCommandReceiver, GCodeCommandSender,
};

pub struct Usb<'a, const GCODE_CHANNEL_LEN: usize, OutputReader: Read> {
    gcode_output_reader: OutputReader,
    gcode_command_sender: GCodeCommandSender<'a, GCODE_CHANNEL_LEN>,
}

impl<'a, const GCODE_CHANNEL_LEN: usize, OutputReader: Read>
    Usb<'a, GCODE_CHANNEL_LEN, OutputReader>
{
    pub fn new(
        cdc_output_reader: OutputReader,
        gcode_command_sender: GCodeCommandSender<'a, GCODE_CHANNEL_LEN>,
    ) -> Self {
        Self {
            gcode_output_reader: cdc_output_reader,
            gcode_command_sender,
        }
    }

    pub async fn run<'d, T: Instance>(
        self,
        usb_peripheral: impl Peripheral<P = T> + 'd,
        irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        unique_id: &[u8; 8],
    ) {
        let driver = Driver::new(usb_peripheral, irq);
        let serial = unique_id_string(unique_id);

        // Use RPI's VID/PID combo to allow interoperability with `picotool`.
        let mut config = Config::new(0x2e8a, 0x000a);
        config.manufacturer = Some("Konkers");
        config.product = Some("RP2040-0816");
        config.serial_number = Some(serial.as_str());
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xef;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;

        // Buffers in which to store USB descriptors
        let mut device_descriptor = [0; 256];
        let mut config_descriptor = [0; 256];
        let mut bos_descriptor = [0; 256];

        // USB control endpoint descriptor
        let mut control_buf = [0; 64];

        let mut cdc_acm_state = cdc_acm::State::new();
        let mut picotool_state = picotool::State::new();

        let mut builder = Builder::new(
            driver,
            config,
            &mut device_descriptor,
            &mut config_descriptor,
            &mut bos_descriptor,
            &mut [], // no msos descriptors
            &mut control_buf,
        );

        // Start building the USB device
        let cdc_acm_class = CdcAcmClass::new(&mut builder, &mut cdc_acm_state, 64);
        let mut _picotool_class = picotool::PicotoolClass::new(&mut builder, &mut picotool_state);

        // Finish building USB device.
        let mut usb = builder.build();

        let mut gcode = gcode_interface::GCodeInterface::new(
            cdc_acm_class,
            self.gcode_output_reader,
            self.gcode_command_sender,
        );

        let usb_future = usb.run();
        let gcode_future = gcode.run();
        join(usb_future, gcode_future).await;
    }
}

fn unique_id_string(id: &[u8; 8]) -> String<11> {
    const BUF_LEN: usize = base64::encoded_len(8, false).unwrap();
    let mut buf = Vec::<u8, BUF_LEN>::new();
    let _ = buf.extend_from_slice(&[0u8; BUF_LEN]);
    general_purpose::STANDARD_NO_PAD
        .encode_slice(id, &mut buf)
        .unwrap();

    // Safety: base64 encode will generate valid utf8
    unsafe { String::<BUF_LEN>::from_utf8_unchecked(buf) }
}
