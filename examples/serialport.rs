use serialport::{DataBits, Parity, SerialPort, StopBits};
use std::ffi::{OsStr, OsString};
use std::fs::File;
use std::io::{Read, Write};
use std::thread;
use std::time;
use stm32_bootloader_client::{BootloaderIO, ProtocolVersion};

type Result<T> = std::result::Result<T, stm32_bootloader_client::Error>;

struct StmSerial {
    serial: Box<dyn SerialPort>,
}

impl StmSerial {
    fn new(serial: Box<dyn SerialPort>) -> Self {
        StmSerial { serial }
    }

    fn autobaud(&mut self) -> Result<()> {
        self.serial.write(&[0x7f]).unwrap();
        let mut buf = [0];
        let rr = self.serial.read(&mut buf);
        println!("{:?} {:x}", rr, buf[0]);
        if buf[0] == 0x79 {
            while self.serial.read(&mut buf).is_ok() {}
            Ok(())
        } else {
            Err(stm32_bootloader_client::Error::UnexpectedResponse)
        }
    }
}

impl BootloaderIO for StmSerial {
    /// Write given bytes to IO device
    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        println!("Writing {:x?}", bytes);
        match self.serial.write(bytes) {
            Ok(_) => Ok(()),
            Err(_) => Err(stm32_bootloader_client::Error::TransportError),
        }
    }

    /// Read to buffer from IO device
    fn read(&mut self, out: &mut [u8]) -> Result<()> {
        println!("Attempting to read {} bytes", out.len());
        match self.serial.read_exact(out) {
            Ok(_) => {
                println!("Read {:x?}", out);
                Ok(())
            }
            Err(_) => {
                println!("Read timeout {:x?}", out);
                Err(stm32_bootloader_client::Error::TransportError)
            }
        }
    }

    /// Read to buffer with a timeout / retry limit
    fn read_with_timeout(&mut self, out: &mut [u8]) -> Result<()> {
        println!("Attempting to read {} bytes with timeout", out.len());
        match self.serial.read_exact(out) {
            Ok(_) => {
                println!("Read {:x?}", out);
                Ok(())
            }
            Err(_) => {
                println!("Read timeout {:x?}", out);
                Err(stm32_bootloader_client::Error::TransportError)
            }
        }
    }

    /// Get the configured wait time for erase operation
    fn get_config_erase_wait_ms(&self) -> u32 {
        100
    }
}

fn get_file_as_byte_vec(filename: &OsStr) -> std::io::Result<Vec<u8>> {
    let mut f = File::open(&filename)?;
    let metadata = std::fs::metadata(&filename)?;
    let mut buffer = vec![0; metadata.len() as usize];
    f.read(&mut buffer)?;
    Ok(buffer)
}

fn print_progress(prog: stm32_bootloader_client::Progress) {
    println!("{} / {}", prog.bytes_complete, prog.bytes_total);
}

fn main() {
    let blob = get_file_as_byte_vec(&OsString::from("stm32g4")).unwrap();

    let port = serialport::new("/dev/tty.usbmodem11303", 115_200)
        .data_bits(DataBits::Eight)
        .parity(Parity::Even)
        .stop_bits(StopBits::One)
        .timeout(time::Duration::from_millis(250))
        .open()
        .unwrap();

    let mut stm_serial = StmSerial::new(port);
    println!("{:?}", stm_serial.autobaud());
    let mut stm32 = stm32_bootloader_client::Stm32::new(stm_serial, ProtocolVersion::Version3_1);
    println!("Init done");

    println!("version {:?}", stm32.get_bootloader_version());
    // println!("support {:?}", stm32.is_command_supported(0x01));

    if stm32.verify(0x0800_0000, &blob, &print_progress).is_ok() {
        println!("Image is OK, exiting");
        println!("Go: {:?}", stm32.go(0x0800_0000));
        return;
    }
    println!("Could not verify image, re-flashing");

    let r = stm32.erase_flash(&mut |ms| thread::sleep(time::Duration::from_millis(ms.into())));
    println!("Erase {:?}", r);

    let r = stm32.write_bulk(0x08000800, &blob[0x800..], &print_progress);
    println!("write1 {:?}", r);
    let r = stm32.write_bulk(0x08000000, &blob[..0x800], &print_progress);
    println!("write2 {:?}", r);

    if stm32.verify(0x08000000, &blob, &print_progress).is_ok() {
        println!("Image is OK, exiting");
        println!("Go: {:?}", stm32.go(0x08000000));
    } else {
        println!("FLASH ERROR");
    }

    // println!("Chip ID {:?}", stm32.get_chip_id());
}
