#![deny(missing_docs, unsafe_code)]
// Copyright 2022 The stm32-bootloader-client-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

//! Communicates with the STM32 factory bootloader over i2c. See AN4221 for
//! details of how the factory bootloader works.

#![cfg_attr(not(any(test, feature = "std")), no_std)]

use embedded_hal::i2c::I2c;

use embedded_io::Read;
use embedded_io::Write;

const BOOTLOADER_ACK: u8 = 0x79;
const BOOTLOADER_NACK: u8 = 0x1f;
const BOOTLOADER_BUSY: u8 = 0x76;

/// Configuration for communication with stm32 system bootloader.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct Config {
    /// See AN2606 for the i2c address for the specific chip you're talking to.
    /// The i2c address will also depend on which i2c bus on the chip you're
    /// connected to.
    i2c_address: u8,

    /// The maximum number of nanoseconds that a full flash erase can take.
    /// Search the datasheet for your specific STM32 for "mass erase time". If
    /// in doubt, round up.
    pub mass_erase_max_ns: u64,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Command {
    GetCommands = 0x00,
    GetVersion = 0x01,
    GetId = 0x02,
    ReadMemory = 0x11,
    Go = 0x21,
    WriteMemory = 0x31,
    Erase = 0x44,
    WriteProtect = 0x63,
    WriteUnprotect = 0x73,
    ReadoutProtect = 0x82,
    ReadoutUnprotect = 0x92,
    // No-Stretch command
    GetMemoryChecksumNS = 0xA1,
}

/// STM32 Bootloader version - to accomodate for variations in the protocol
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ProtocolVersion {
    /// Version 1.0
    Version1_0 = 0x10,
    /// Version 1.1
    Version1_1 = 0x11,
    /// Version 3.1
    Version3_1 = 0x31,
}

impl ProtocolVersion {
    /// Obtain protocol version from binary representation if possible.
    pub fn from_u8(n: u8) -> Option<Self> {
        match n {
            0x10 => Some(ProtocolVersion::Version1_0),
            0x11 => Some(ProtocolVersion::Version1_1),
            0x31 => Some(ProtocolVersion::Version3_1),
            _ => None,
        }
    }
}

const SPECIAL_ERASE_ALL: [u8; 2] = [0xff, 0xff];

/// Maximume size that can be read or written throgh read/write calls
pub const MAX_READ_WRITE_SIZE: usize = 128;

type Result<T, E = Error> = core::result::Result<T, E>;

/// Errors that can be encountered and reported
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Error in the transport layer
    TransportError,
    /// Nack was received
    Nack,
    /// Command produced an unexpected Nack
    NackFromCommand(u8),
    /// Device busy
    Busy,
    /// Checksum command failed
    ChecksumCommandError,
    /// NoStretch Command timed out
    CommandTimeout,
    /// Unexpected response was encountered
    UnexpectedResponse,
    /// Invalid arguments were provided
    InvalidArgument,
    /// Verification failure
    VerifyFailedAtAddress(u32),
    /// Erase failed
    EraseFailed,
    /// Wrong protocol version
    IncorrectProtocol(ProtocolVersion),
    /// Could not parse protocol version
    UnknownProtocol(u8),
}

/// Trait that encapsulates IO to the STM32 bootloader
pub trait BootloaderIO {
    /// Write given bytes to IO device
    fn write(&mut self, bytes: &[u8]) -> Result<()>;
    /// Read to buffer from IO device
    fn read(&mut self, out: &mut [u8]) -> Result<()>;
    /// Read to buffer with a timeout / retry limit
    fn read_with_timeout(&mut self, out: &mut [u8]) -> Result<()>;
    /// Get the configured wait time for erase operation
    fn get_config_erase_wait_ns(&self) -> u64;
}

/// A structure with a borrow to an I2C device that can be used for bootloader IO
pub struct Stm32i2c<I2C: I2c> {
    dev: I2C,
    config: Config,
}

impl<I2cImpl> Stm32i2c<I2cImpl>
    where
        I2cImpl: I2c
{
    /// Create a new instance of the I2C bootloader IO structure
    pub fn new(dev: I2cImpl, config: Config) -> Self {
        Stm32i2c { dev, config }
    }
}

impl<I2cImpl> BootloaderIO for Stm32i2c<I2cImpl>
where
    I2cImpl: I2c
{
    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        self.dev
            .write(self.config.i2c_address, bytes)
            .map_err(|error| {
                log_error(&error);
                Error::TransportError
            })
    }

    fn read(&mut self, out: &mut [u8]) -> Result<()> {
        self.dev
            .read(self.config.i2c_address, out)
            .map_err(|error| {
                log_error(&error);
                Error::TransportError
            })
    }

    fn read_with_timeout(&mut self, out: &mut [u8]) -> Result<()> {
        // TODO: Implement timeout mechanism
        const MAX_ATTEMPTS: u32 = 10000;
        let mut attempts = 0;
        loop {
            attempts += 1;
            let result = self.read(out);
            if result.is_ok() || attempts == MAX_ATTEMPTS {
                return result;
            }
        }
    }

    fn get_config_erase_wait_ns(&self) -> u64 {
        self.config.mass_erase_max_ns
    }
}

/// A structure with a borrow to an UART device that can be used for bootloader IO
pub struct Stm32Uart<'a, U: Write + Read> {
    dev: &'a mut U,
    config: Config,
}

impl<'a, E, U> Stm32Uart<'a, U>
where
    E: core::fmt::Debug,
    U: Write<Error = E> + Read<Error = E>,
{
    /// Create a new instance of the I2C bootloader IO structure
    pub fn new(dev: &'a mut U, config: Config) -> Self {
        Stm32Uart { dev, config }
    }
}

impl<'a, E, U> Stm32Uart<'a, U>
where
    E: core::fmt::Debug,
    U: Write<Error = E> + Read<Error = E>,
{
    /// Attempt to do the auto baud rate exchange
    pub fn auto_baud(&mut self) -> Result<()> {
        self.write(&[0x7f])?;
        let mut buf = [0];
        self.read(&mut buf)?;
        if buf[0] == 0x79 {
            Ok(())
        } else {
            Err(Error::TransportError)
        }
    }
}

impl<'a, E, U> BootloaderIO for Stm32Uart<'a, U>
where
    E: core::fmt::Debug,
    U: Write<Error = E> + Read<Error = E>,
{
    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        for index in 0..bytes.len() {
            while self.dev.write(&bytes[index..index + 1]).is_err() {}
        }
        
        Ok(())
    }

    fn read(&mut self, out: &mut [u8]) -> Result<()> {
        for index in 0..out.len() {
            let mut retries = 250000;
            loop {
                match self.dev.read(&mut out[index..index + 1]) {
                    Ok(_) => break,
                    Err(_) => {
                        if retries == 0 {
                            return Err(Error::TransportError);
                        }
                        retries -= 1;
                    },
                }
            }
        }
        Ok(())
    }

    fn read_with_timeout(&mut self, out: &mut [u8]) -> Result<()> {
        // TODO: Implement timeout mechanism
        const MAX_ATTEMPTS: u32 = 100;
        let mut attempts = 0;
        loop {
            attempts += 1;
            let result = self.read(out);
            if result.is_ok() || attempts == MAX_ATTEMPTS {
                return result;
            }
        }
    }

    fn get_config_erase_wait_ns(&self) -> u64 {
        self.config.mass_erase_max_ns
    }
}

/// A generic bootloader Interface that uses the BootloaderIO trait to communicate with the STM32 bootloader
pub struct Stm32<D: BootloaderIO> {
    dev: D,
    version: ProtocolVersion,
}

/// Progress information provided to the callback progress handler
#[derive(Debug, Clone)]
pub struct Progress {
    /// Number of bytes that have been completed
    pub bytes_complete: usize,
    /// The total number of bytes to transfer or verify
    pub bytes_total: usize,
}

impl<D> Stm32<D>
where
    D: BootloaderIO,
{
    /// Borrows both BootloaderIO implementation with a lifetime.
    pub fn new(dev: D, version: ProtocolVersion) -> Stm32<D> {
        Self { dev, version }
    }

    /// Obtain the chip ID of the device that the bootloader is running on
    pub fn get_chip_id(&mut self) -> Result<u16> {
        self.send_command(Command::GetId)?;
        // For STM32, the first byte will always be a 1 and the payload will
        // always be 3 bytes.
        let mut buffer = [0u8; 3];
        self.dev.read(&mut buffer)?;
        self.get_ack_for_command(Command::GetId)?;
        Ok(u16::from_be_bytes([buffer[1], buffer[2]]))
    }

    /// Reads memory starting from `address`, putting the result into `out`.
    pub fn read_memory(&mut self, address: u32, out: &mut [u8]) -> Result<()> {
        if out.len() > MAX_READ_WRITE_SIZE {
            return Err(Error::InvalidArgument);
        }
        self.send_command(Command::ReadMemory)?;
        self.send_address(address)?;
        let mut buffer = [0u8; 2];
        buffer[0] = (out.len() - 1) as u8;
        buffer[1] = checksum(&buffer[0..1]);
        self.dev.write(&buffer)?;
        self.get_ack_for_command(Command::ReadMemory)?;
        self.dev.read(out)?;

        Ok(())
    }

    /// Writes `data` at `address`. Maximum write size is 256 bytes.
    pub fn write_memory(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if data.len() > MAX_READ_WRITE_SIZE {
            return Err(Error::InvalidArgument);
        }
        self.send_command(Command::WriteMemory)?;
        self.send_address(address)?;

        let mut buffer = [0u8; MAX_READ_WRITE_SIZE + 2];
        buffer[0] = (data.len() - 1) as u8;
        buffer[1..1 + data.len()].copy_from_slice(data);
        buffer[1 + data.len()] = checksum(&buffer[0..1 + data.len()]);
        self.dev.write(&buffer[..data.len() + 2])?;

        self.get_ack_for_command(Command::WriteMemory)
    }

    /// Writes `bytes` to `address`, calling `progress_cb` after each block.
    pub fn write_bulk(
        &mut self,
        mut address: u32,
        bytes: &[u8],
        mut progress_cb: impl FnMut(Progress),
    ) -> Result<()> {
        let mut complete = 0;
        for chunk in bytes.chunks(MAX_READ_WRITE_SIZE) {
            // Write-memory sometimes gets a NACK, so allow a single retry.
            if self.write_memory(address, chunk).is_err() {
                self.write_memory(address, chunk)?;
            }
            complete += chunk.len();
            address += chunk.len() as u32;
            progress_cb(Progress {
                bytes_complete: complete,
                bytes_total: bytes.len(),
            });
        }
        Ok(())
    }

    /// Verifies that memory at `address` is equal to `bytes`, calling
    /// `progress_cb` to report progress.
    pub fn verify(
        &mut self,
        mut address: u32,
        bytes: &[u8],
        mut progress_cb: impl FnMut(Progress),
    ) -> Result<()> {
        let mut read_back_buffer = [0; MAX_READ_WRITE_SIZE];
        let mut complete = 0;
        for chunk in bytes.chunks(MAX_READ_WRITE_SIZE) {
            let read_back = &mut read_back_buffer[..chunk.len()];
            self.read_memory(address, read_back)?;
            for (offset, (expected, actual)) in chunk.iter().zip(read_back.iter()).enumerate() {
                if expected != actual {
                    return Err(Error::VerifyFailedAtAddress(address + offset as u32));
                }
            }
            complete += chunk.len();
            address += chunk.len() as u32;
            progress_cb(Progress {
                bytes_complete: complete,
                bytes_total: bytes.len(),
            });
        }
        Ok(())
    }

    /// Erase the whole flash of the STM32.
    pub fn erase_flash(&mut self, delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        self.send_command(Command::Erase)?;
        let mut buffer = [0u8; 3];
        buffer[0..2].copy_from_slice(&SPECIAL_ERASE_ALL);
        buffer[2] = checksum(&buffer[..2]);
        self.dev.write(&buffer)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack().map_err(|_| Error::EraseFailed)
    }

    /// Erase single pages of the STM32.
    pub fn erase_pages(&mut self, pages: &[u16], delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        assert!(pages.len() >= 1);
        self.send_command(Command::Erase)?;

        // submit number of pages to erase
        let page_count = (pages.len() - 1) as u16;
        let mut buffer = [0u8; 3];
        buffer[0..2].copy_from_slice(&page_count.to_be_bytes());
        buffer[2] = checksum(&buffer[..2]);
        self.dev.write(&buffer)?;
        self.get_ack().map_err(|_| Error::EraseFailed)?;

        // submit actual page numbers
        let mut pages_buffer = vec![0u8; (pages.len() * 2) + 1];
        for (buf_index, page)  in pages.iter().enumerate() {
            pages_buffer[buf_index * 2..(buf_index * 2) + 2].copy_from_slice(&page.to_be_bytes());
        }
        pages_buffer[pages.len() * 2] = checksum(&pages_buffer[..pages.len() * 2]);
        self.dev.write(&pages_buffer)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack().map_err(|_| Error::EraseFailed)
    }

    /// Write protect sectors
    pub fn write_protect(&mut self, sectors: &[u8], delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        self.send_command(Command::WriteProtect)?;
        
        let mut buffer = [0u8; 2];
        buffer[0] = sectors.len() as u8;
        buffer[1] = checksum(&buffer[0..1]);
        self.dev.write(&buffer)?;
        self.get_ack_for_command(Command::WriteProtect)?;

        let mut buffer = vec![0u8; sectors.len() + 1];
        buffer[0..sectors.len()].copy_from_slice(&sectors);
        buffer[sectors.len()] = checksum(&buffer[..sectors.len()]);
        self.dev.write(&buffer)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack_for_command(Command::WriteProtect)
    }

    /// Deactivate write protection globally
    pub fn write_unprotect(&mut self, delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        self.send_command(Command::WriteUnprotect)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack_for_command(Command::WriteUnprotect)
    }

    /// Activate readout protection
    pub fn readout_protect(&mut self, delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        self.send_command(Command::ReadoutProtect)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack_for_command(Command::ReadoutProtect)
    }

    /// Deactivate readout protection
    pub fn readout_unprotect(&mut self, delay_ns: &mut dyn Fn(u64)) -> Result<()> {
        self.send_command(Command::ReadoutUnprotect)?;
        delay_ns(self.dev.get_config_erase_wait_ns());
        self.get_ack_for_command(Command::ReadoutUnprotect)
    }

    /*
    /// Calculate checksum over specified memory area
    pub fn get_memory_checksum(&mut self, address: u32, length: u32) -> Result<u32> {
        self.send_command(Command::GetMemoryChecksumNS)?;
        // Address
        self.send_address(address).map_err(|_|Error::ChecksumCommandError)?;
        // Size
        self.send_address(length).map_err(|_|Error::ChecksumCommandError)?;

        let mut resp = [0u8; 5];
        resp[0] = BOOTLOADER_BUSY;
        while resp[0] == BOOTLOADER_BUSY {
            // TODO: Sleep
            self.dev.read(&mut resp[..1])?;
        }

        self.dev.read(&mut resp).map_err(|_|Error::CommandTimeout)?;
        assert_eq!(checksum(&resp[..4]), resp[4]);
        Ok(u32::from_be_bytes(resp[..4].try_into().unwrap()))
    }
    */

    // Returns the version number of the bootloader (v1.x)
    fn get_bootloader_version_v1(&mut self) -> Result<u8> {
        self.send_command(Command::GetVersion)?;
        let mut buffer = [0];
        self.dev.read(&mut buffer)?;
        self.get_ack_for_command(Command::GetVersion)?;
        Ok(buffer[0])
    }

    // Returns the version number of the bootloader (v3.1)
    fn get_bootloader_version_v3(&mut self) -> Result<u8> {
        self.send_command(Command::GetVersion)?;
        let mut buffer = [0; 3];
        self.dev.read(&mut buffer)?;
        self.get_ack_for_command(Command::GetVersion)?;
        Ok(buffer[0])
    }

    /// Returns the version number of the bootloader.
    pub fn get_bootloader_version(&mut self) -> Result<u8> {
        match self.version {
            ProtocolVersion::Version1_0 => self.get_bootloader_version_v1(),
            ProtocolVersion::Version1_1 => self.get_bootloader_version_v1(),
            ProtocolVersion::Version3_1 => self.get_bootloader_version_v3(),
        }
    }

    /// Get the command list to probe the protocol version in a safer way,
    /// Without requiring prior knowledge of the version on the remote end
    /// Will also optionally report if a specific command is supported,
    /// as well as mismatch to the selected protocol version
    pub fn is_command_supported(&mut self, command: u8) -> Result<(ProtocolVersion, bool)> {
        self.send_command(Command::GetCommands)?;
        let mut num_bytes = [0];
        self.dev.read(&mut num_bytes)?;
        let num_bytes = num_bytes[0] as usize + 1;
        let mut cmd_list_buffer = [0; MAX_READ_WRITE_SIZE];
        if num_bytes > 0 && num_bytes <= cmd_list_buffer.len() {
            self.dev.read(&mut cmd_list_buffer[..num_bytes])?;
        } else {
            return Err(Error::InvalidArgument);
        }
        self.get_ack()?;
        let version = ProtocolVersion::from_u8(cmd_list_buffer[0])
            .ok_or(Error::UnknownProtocol(cmd_list_buffer[0]))?;
        if version != self.version {
            // Report the version that was reported from boot loader
            return Err(Error::IncorrectProtocol(version));
        }

        Ok((
            version,
            (num_bytes > 1 && cmd_list_buffer[..num_bytes].iter().any(|c| *c == command)),
        ))
    }

    /// Exit system bootloader by jumping to the reset vector specified in the
    /// vector table at `address`.
    pub fn go(&mut self, address: u32) -> Result<()> {
        self.send_command(Command::Go)?;
        self.send_address(address)
    }

    fn send_command(&mut self, command: Command) -> Result<()> {
        let command_u8 = command as u8;
        self.dev.write(&[command_u8, !command_u8])?;
        self.get_ack_for_command(command)
    }

    fn get_ack(&mut self) -> Result<()> {
        let mut response = [0u8; 1];
        self.dev.read_with_timeout(&mut response)?;
        match response[0] {
            BOOTLOADER_ACK => Ok(()),
            BOOTLOADER_NACK => Err(Error::Nack),
            BOOTLOADER_BUSY => Err(Error::Busy),
            _ => Err(Error::UnexpectedResponse),
        }
    }

    fn get_ack_for_command(&mut self, command: Command) -> Result<()> {
        self.get_ack().map_err(|error| match error {
            Error::Nack => Error::NackFromCommand(command as u8),
            x => x,
        })
    }

    fn send_address(&mut self, address: u32) -> Result<()> {
        let mut buffer = [0u8; 5];
        buffer[0..4].copy_from_slice(&address.to_be_bytes());
        buffer[4] = checksum(&buffer[0..4]);
        self.dev.write(&buffer)?;
        self.get_ack()
    }
}

/// An implementation of the embedded-hal DelayMs trait that works when std is
/// available.
#[cfg(feature = "std")]
pub struct StdDelay;

#[cfg(feature = "std")]
impl embedded_hal::delay::DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(std::time::Duration::from_nanos(ns.into()));
    }
}

fn checksum(bytes: &[u8]) -> u8 {
    let initial = if bytes.len() == 1 { 0xff } else { 0 };
    bytes
        .iter()
        .fold(initial, |checksum, value| checksum ^ value)
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::TransportError => write!(f, "Transport error"),
            Error::Nack => write!(f, "NACK"),
            Error::NackFromCommand(command) => write!(f, "Nack from command {}", command),
            Error::Busy => write!(f, "Busy"),
            Error::ChecksumCommandError => write!(f, "Checksum command error"),
            Error::CommandTimeout => write!(f, "NoStretch command timed out"),
            Error::UnexpectedResponse => write!(f, "Unexpected response"),
            Error::InvalidArgument => write!(f, "Invalid argument"),
            Error::VerifyFailedAtAddress(address) => {
                write!(f, "Verify failed at address {:x}", address)
            }
            Error::EraseFailed => write!(f, "Erase failed"),
            Error::UnknownProtocol(pv) => write!(f, "Unknown protocol version {:02x}", pv),
            Error::IncorrectProtocol(pv) => write!(f, "Incorrect protocol version {:?}", pv),
        }
    }
}

impl Config {
    /// Create a default configuration for i2c IO
    pub const fn i2c_address(i2c_address: u8) -> Self {
        Self {
            i2c_address,
            // A moderately conservative default. stm32g071 has 40.1ms.
            // stm32l452 has 24.59ms
            mass_erase_max_ns: 200_000_000,
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

fn log_error<E: core::fmt::Debug>(_error: &E) {
    #[cfg(feature = "defmt")]
    {
        defmt::error!("I2C error: {:?}", defmt::Debug2Format(_error));
    }
    #[cfg(feature = "log")]
    {
        log::error!("I2C error: {:?}", _error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c;

    const I2C_ADDRESS: u8 = 0x51;
    const CONFIG: Config = Config::i2c_address(I2C_ADDRESS);

    fn mock_write_with_checksum(bytes: &[u8]) -> i2c::Transaction {
        let mut with_checksum = Vec::with_capacity(bytes.len() + 1);
        with_checksum.extend_from_slice(bytes);
        with_checksum.push(checksum(bytes));
        i2c::Transaction::write(I2C_ADDRESS, with_checksum)
    }

    fn mock_read(bytes: &[u8]) -> i2c::Transaction {
        i2c::Transaction::read(I2C_ADDRESS, bytes.to_owned())
    }

    struct Delay;

    impl embedded_hal::delay::DelayNs for Delay {
        fn delay_ns(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_get_chip_id() {
        let expectations = [
            mock_write_with_checksum(&[Command::GetId as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0, 0xab, 0xcd]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        assert_eq!(stm32.get_chip_id().unwrap(), 0xabcd);
        i2c.done();
    }

    #[test]
    fn test_get_bootloader_version() {
        let expectations = [
            mock_write_with_checksum(&[Command::GetVersion as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0xef]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        assert_eq!(stm32.get_bootloader_version().unwrap(), 0xef);
        i2c.done();
    }

    #[test]
    fn test_read_memory() {
        let expectations = [
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x02]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0xab, 0xcd, 0xef]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        let mut out = [0u8; 3];
        stm32.read_memory(0x12345678, &mut out).unwrap();
        assert_eq!(&out, &[0xab, 0xcd, 0xef]);
        i2c.done();
    }

    #[test]
    fn test_write_memory() {
        let expectations = [
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x03, 0xab, 0xcd, 0xef, 0x12]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .write_memory(0x12345678, &[0xab, 0xcd, 0xef, 0x12])
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_go() {
        let expectations = [
            mock_write_with_checksum(&[Command::Go as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32.go(0x12345678).unwrap();
        i2c.done();
    }

    #[test]
    fn test_write_bulk() {
        let to_write: Vec<u8> = (0..200).collect();
        let mut write1 = vec![127u8];
        write1.extend_from_slice(&to_write[..128]);
        let mut write2 = vec![(to_write.len() - 128 - 1) as u8];
        write2.extend_from_slice(&to_write[128..]);
        let expectations = [
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&write1),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0xf8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&write2),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        let mut callback_count = 0;
        stm32
            .write_bulk(0x12345678, &to_write, |_| {
                callback_count += 1;
            })
            .unwrap();
        assert_eq!(callback_count, 2);
        i2c.done();
    }

    #[test]
    fn test_erase_flash() {
        let expectations = [
            mock_write_with_checksum(&[Command::Erase as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0xFF, 0xFF]),
            mock_read(&[BOOTLOADER_ACK])
        ];

        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .erase_flash(&mut |_| {})
            .unwrap();
        i2c.done();
    }


    #[test]
    fn test_erase_page_one() {
        let expectations = [
            mock_write_with_checksum(&[Command::Erase as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x00, 0x00]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x00, 0x01]),
            mock_read(&[BOOTLOADER_ACK])
        ];

        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .erase_pages(&[1], &mut |_| {})
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_erase_page_one_and_two() {
        let expectations = [
            mock_write_with_checksum(&[Command::Erase as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x00, 0x01]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x00, 0x01, 0x00, 0x02]),
            mock_read(&[BOOTLOADER_ACK])
        ];

        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .erase_pages(&[1, 2], &mut |_| {})
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_write_protect() {
        let expectations = [
            mock_write_with_checksum(&[Command::WriteProtect as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x03]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[2, 5, 9]),
            mock_read(&[BOOTLOADER_ACK])
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .write_protect(&[2, 5, 9], &mut |_| {})
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_write_unprotect() {
        let expectations = [
            mock_write_with_checksum(&[Command::WriteUnprotect as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .write_unprotect(&mut |_| {})
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_readout_protect() {
        let expectations = [
            mock_write_with_checksum(&[Command::ReadoutProtect as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .readout_protect(&mut |_| {})
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_readout_unprotect() {
        let expectations = [
            mock_write_with_checksum(&[Command::ReadoutUnprotect as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        stm32
            .readout_unprotect(&mut |_| {})
            .unwrap();
        i2c.done();
    }

    /*
    #[test]
    fn test_get_memory_checksum() {
        let expectations = [
            mock_write_with_checksum(&[Command::GetMemoryChecksumNS as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x00, 0x00, 0x01, 0x23]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[BOOTLOADER_BUSY]),
            mock_read(&[BOOTLOADER_BUSY]),
            mock_read(&[BOOTLOADER_BUSY]),
            mock_read(&[0xF0, 0xF1, 0xF2, 0xF3, 0x00]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        let checksum = stm32
            .get_memory_checksum(0x12345678, 0x123)
            .unwrap();
        assert_eq!(checksum, 0x00000000);
        i2c.done();
    }
    */

    #[test]
    fn test_verify() {
        let to_verify: Vec<u8> = (0..200).collect();
        let expectations = [
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[127]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&to_verify[..128]),
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0xf8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[(to_verify.len() - 128 - 1) as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&to_verify[128..]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let boot_io = Stm32i2c::new(&mut i2c, CONFIG);
        let mut stm32 = Stm32::new(boot_io, ProtocolVersion::Version1_1);
        let mut callback_count = 0;
        stm32
            .verify(0x12345678, &to_verify, |_| {
                callback_count += 1;
            })
            .unwrap();
        assert_eq!(callback_count, 2);
        i2c.done();
    }
}
