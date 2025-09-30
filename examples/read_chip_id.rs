// Copyright 2022 The stm32-bootloader-client-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

use anyhow::Result;
use mcp2221_hal::gpio::GpioChanges;
use stm32_bootloader_client::{ProtocolVersion, Stm32, Stm32i2c};

fn main() {
    if let Err(error) = run() {
        println!("Error: {}", error);
    }
}

fn run() -> Result<()> {
    let mut dev = mcp2221_hal::MCP2221::connect()?;
    dev.i2c_set_bus_speed(mcp2221_hal::i2c::I2cSpeed::fast_400k())?;

    // Set GPIO pin 0 high. This is useful if your I2C bus goes through a level
    // shifter and you need to enable that level shifter in order to use the I2C
    // bus.
    let mut gpio_change = GpioChanges::new();
    gpio_change
        .with_gp0_direction(mcp2221_hal::gpio::GpioDirection::Output)
        .with_gp0_level(mcp2221_hal::gpio::LogicLevel::High);
    dev.gpio_write(&gpio_change)?;

    dev.status()?;

    // This is the address for I2C1 on the STM32G0 series. See AN2606 for
    // the list of addresses for other parts.
    let config = stm32_bootloader_client::Config::i2c_address(0x51);
    let mut stm32 = Stm32::new(Stm32i2c::new(&mut dev, config), ProtocolVersion::Version1_1);
    let chip_id = stm32.get_chip_id()?;
    println!("Found chip ID: 0x{chip_id:x}");

    Ok(())
}
