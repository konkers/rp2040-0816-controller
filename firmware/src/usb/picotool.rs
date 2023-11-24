use core::mem::MaybeUninit;

use embassy_rp::rom_data;
use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::Driver;
use embassy_usb::types::{InterfaceNumber, StringIndex};
use embassy_usb::{Builder, Handler};

const USB_CLASS_VENDOR_SPECIFIC: u8 = 0xff;
const PICOTOOL_INTERFACE_SUBCLASS: u8 = 0x00;
const PICOTOOL_INTERFACE_PROTOCOL: u8 = 0x01;

const PICOTOOL_REQUEST_BOOTSEL: u8 = 0x01;

struct Control {
    comm_if: InterfaceNumber,
    reset_string_index: StringIndex,
}

pub struct State {
    control: MaybeUninit<Control>,
}

impl State {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
        }
    }
}

pub struct PicotoolClass<'d, D: Driver<'d>> {
    _comm_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> PicotoolClass<'d, D> {
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State,
        //max_packet_size: u16,
    ) -> Self {
        //assert!(builder.control_buf_len() >= 7);

        let mut func = builder.function(
            USB_CLASS_VENDOR_SPECIFIC,
            PICOTOOL_INTERFACE_SUBCLASS,
            PICOTOOL_INTERFACE_PROTOCOL,
        );

        let mut iface = func.interface();
        let reset_string_index = iface.string();
        let comm_if = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_CLASS_VENDOR_SPECIFIC,
            PICOTOOL_INTERFACE_SUBCLASS,
            PICOTOOL_INTERFACE_PROTOCOL,
            Some(reset_string_index),
        );
        let comm_ep = alt.endpoint_interrupt_in(8, 255);
        // Embassy's build in class drivers drop `func` here.  They do not document why.
        drop(func);
        let control = state.control.write(Control {
            comm_if,
            reset_string_index,
        });
        builder.handler(control);

        Self { _comm_ep: comm_ep }
    }
}

impl Handler for Control {
    fn reset(&mut self) {
        defmt::info!("picotool: reset");
    }

    fn control_out(&mut self, req: control::Request, _data: &[u8]) -> Option<OutResponse> {
        // Send no response if this is not ours.
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Class,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            return None;
        }

        defmt::info!("picotool: control_out {}", req.request);
        match req.request {
            PICOTOOL_REQUEST_BOOTSEL => {
                defmt::info!("received BOOTSEL request");

                // For now, hard code disabling the USB mass storage interface.
                rom_data::reset_to_usb_boot(0, 1);

                unreachable!()
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // Send no response if this is not ours.
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Class,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            return None;
        }

        defmt::info!("picotool: control_in");

        // Reject all other responses
        Some(InResponse::Rejected)
    }

    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.reset_string_index {
            return Some("Reset");
        }
        None
    }
}
