use arrayvec::ArrayVec;
use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_rp::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{self, Channel};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embedded_io_async::Read;
use gcode::buffers::Buffers;
use gcode::{Comment, Nop, Parser, Word};
use heapless::Vec;

use crate::{Error, Result};

pub type GCodeCommand = gcode::GCode<ArrayVec<[Word; 10]>>;
pub type GCodeCommandChannel<const N: usize> = Channel<NoopRawMutex, GCodeCommand, N>;
pub type GCodeCommandReceiver<'a, const N: usize> =
    channel::Receiver<'a, NoopRawMutex, GCodeCommand, N>;
pub type GCodeCommandSender<'a, const N: usize> =
    channel::Sender<'a, NoopRawMutex, GCodeCommand, N>;

struct CharAssembler {
    buf: [u8; 4],
    len: usize,
}

impl CharAssembler {
    fn new() -> Self {
        Self {
            buf: [0u8; 4],
            len: 0,
        }
    }

    fn handle_byte(&mut self, b: u8) -> Option<char> {
        // We know that `self.buf` will not overflow because 4 bytes is large enough for any char.
        self.buf[self.len] = b;
        self.len += 1;

        // Only proceed if we have the correct number of bytes for a character.
        if self.len != core::str::utf8_char_width(self.buf[0]) {
            return None;
        }

        let ret = char::from_u32(u32::from_le_bytes(self.buf));

        // Reset the internal buffer regardless of the character's validity.
        self.len = 0;
        self.buf = [0u8; 4];

        ret
    }
}

struct LineReader<const N: usize> {
    char_assembler: CharAssembler,
    input_buffer: Vec<u8, N>,
    in_overflow: bool,
    new_line: bool,
}

impl<const N: usize> LineReader<N> {
    pub fn new() -> Self {
        Self {
            char_assembler: CharAssembler::new(),
            input_buffer: Vec::new(),
            in_overflow: false,
            new_line: false,
        }
    }

    pub fn handle_byte(&mut self, b: u8) -> Result<Option<&str>> {
        // Previous iteration resulted in a new line.  Clear our buffer now.
        if self.new_line {
            self.input_buffer.clear();
            self.new_line = false;
        }

        // wait for a valid unicode char.
        let Some(c) = self.char_assembler.handle_byte(b) else {
            return Ok(None);
        };

        if self.in_overflow {
            // Discard any non-newline characters while in overflow condition.
            if !Self::is_newline(c) {
                return Ok(None);
            }

            // Otherwise record the overflow and reset the buffer length and overflow state
            self.in_overflow = false;
            self.input_buffer.clear();
            Err(Error::InputBufferOverflow)
        } else if Self::is_newline(c) {
            // If we're not in overflow and have a newline, return the line.

            // Safety: We only write valid utf8 to self.buf.
            let s = unsafe { core::str::from_utf8_unchecked(self.input_buffer.as_slice()) };
            self.new_line = true;
            Ok(Some(s))
        } else {
            let mut encode_buf = [0u8; 4];
            let encoded = c.encode_utf8(&mut encode_buf).as_bytes();

            if self.input_buffer.extend_from_slice(encoded).is_err() {
                self.in_overflow = true;
                // Wait to return Error::InputBufferOverflow until we receive a newline.
                return Ok(None);
            }
            Ok(None)
        }
    }

    fn is_newline(c: char) -> bool {
        c == '\n' || c == '\r'
    }
}

struct GCodeBuffers;
impl<'input> Buffers<'input> for GCodeBuffers {
    type Arguments = ArrayVec<[Word; 10]>;
    type Commands = ArrayVec<[gcode::GCode<Self::Arguments>; 2]>;
    type Comments = ArrayVec<[Comment<'input>; 1]>;
}

pub struct GCodeInterface<
    'd,
    'g,
    const GCODE_CHANNEL_LEN: usize,
    OutputReader: Read,
    T: Instance + 'd,
> {
    class: CdcAcmClass<'d, Driver<'d, T>>,
    output_reader: OutputReader,
    command_sender: GCodeCommandSender<'g, GCODE_CHANNEL_LEN>,
}

impl<'d, 'g, const GCODE_CHANNEL_LEN: usize, OutputReader: Read, T: Instance + 'd>
    GCodeInterface<'d, 'g, GCODE_CHANNEL_LEN, OutputReader, T>
{
    pub fn new(
        class: CdcAcmClass<'d, Driver<'d, T>>,
        output_reader: OutputReader,
        command_sender: GCodeCommandSender<'g, GCODE_CHANNEL_LEN>,
    ) -> Self {
        Self {
            class,
            output_reader,
            command_sender,
        }
    }

    pub async fn run(&mut self) {
        loop {
            info!("Waiting for connection");
            self.class.wait_connection().await;
            info!("USB Connected");
            let _ = self.handle_connection().await;
            info!("USB Disconnected");
        }
    }

    async fn handle_connection(&mut self) -> Result<()> {
        let mut usb_buf = [0; 64];
        let mut output_buf = [0; 64];
        let mut line_reader = LineReader::<64>::new();
        loop {
            match select(
                self.output_reader.read(&mut output_buf),
                self.class.read_packet(&mut usb_buf),
            )
            .await
            {
                Either::First(read_len) => {
                    let read_len = read_len.map_err(|_| Error::Io)?;
                    self.class.write_packet(&output_buf[..read_len]).await?;
                }
                Either::Second(read_len) => {
                    let read_len = read_len?;
                    // Echo input back to the connection.
                    self.class.write_packet(&usb_buf[..read_len]).await?;

                    for b in &usb_buf[..read_len] {
                        if let Some(line) = line_reader.handle_byte(*b)? {
                            // Echo a new line incase were just send a '\r'.  Having a
                            // real line editor would make things nicer here.
                            self.class.write_packet(b"\n").await?;
                            self.handle_line(line).await?;
                        }
                    }
                }
            }
        }
    }

    async fn handle_line(&mut self, line: &str) -> Result<()> {
        let parser: Parser<Nop, GCodeBuffers> = Parser::new(line, Nop);
        for line in parser {
            for command in line.gcodes() {
                // Block on having room in the command buffer.
                self.command_sender.send(command.clone()).await;
            }
        }
        Ok(())
    }
}
