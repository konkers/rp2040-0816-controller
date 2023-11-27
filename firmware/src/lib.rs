#![no_std]
#![feature(const_option)]
#![feature(type_alias_impl_trait)]
// This is used for `utf8_char_width`.
#![feature(str_internals)]

pub mod gpio_input;
pub mod pwm_servo;
pub mod usb;
