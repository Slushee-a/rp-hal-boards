#![no_std]

//! A Hardware Abstraction Layer for the Picoboy by Jan Schulz
//!
//! This crate serves as a HAL (Hardware Abstraction Layer) for the Picoboy. Since the Picoboy
//! is based on the RP2040 chip, it re-exports the [rp2040_hal] crate which contains the tooling to work with the
//! rp2040 chip.
//!
//! # Examples:
//!
//! The following example turns on the onboard red LED. Note that most of the logic works through the [rp2040_hal] crate.
//! ```ignore
//! #![no_main]
//! use picoboy::entry;
//! use panic_halt as _;
//! use embedded_hal::digital::v2::OutputPin;
//! use picoboy::hal::pac;
//! use picoboy::hal;

//! #[entry]
//! fn does_not_have_to_be_main() -> ! {
//!   let mut pac = pac::Peripherals::take().unwrap();
//!   let sio = hal::Sio::new(pac.SIO);
//!   let pins = rp_pico::Pins::new(
//!        pac.IO_BANK0,
//!        pac.PADS_BANK0,
//!        sio.gpio_bank0,
//!        &mut pac.RESETS,
//!   );
//!   let mut red_led = pins.red_led.into_push_pull_output();
//!   red_led.set_high().unwrap();
//!   loop {
//!   }
//! }
//! ```

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;

/// The `entry` macro declares the starting function to the linker.
/// This is similar to the `main` function in console applications.
///
/// It is based on the [cortex_m_rt](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/attr.entry.html) crate.
///
/// # Examples
/// ```ignore
/// #![no_std]
/// #![no_main]
/// use rp_pico::entry;
/// #[entry]
/// fn you_can_use_a_custom_main_name_here() -> ! {
///   loop {}
/// }
/// ```

#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080; // Also works for the
                                                                          // onboard W25Q16JV

pub use hal::pac;

hal::bsp_pins!(
    /// GPIO 0 is connected to _joystick enter_ and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp0Pio0]            |
    /// | `PIO1`       | [crate::Gp0Pio1]            |
    Gpio0 {
        name: j_enter,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio0].
            FunctionPio0, PullNone: Gp0Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio0].
            FunctionPio1, PullNone: Gp0Pio1
        }
    },

    /// GPIO 1 is connected to _joystick up_ and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp1Pio0]            |
    /// | `PIO1`       | [crate::Gp1Pio1]            |
    Gpio1 {
        name: j_up,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio1].
            FunctionPio0, PullNone: Gp1Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio1].
            FunctionPio1, PullNone: Gp1Pio1
        }
    },

    /// GPIO 2 is connected to _joystick right_ and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp2Pio0]            |
    /// | `PIO1`       | [crate::Gp2Pio1]            |
    Gpio2 {
        name: j_right,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio2].
            FunctionPio0, PullNone: Gp2Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio2].
            FunctionPio1, PullNone: Gp2Pio1
        }
    },

    /// GPIO 3 is connected to _joystick down_ and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// | `PIO0`       | [crate::Gp3Pio0]            |
    /// | `PIO1`       | [crate::Gp3Pio1]            |
    Gpio3 {
        name: j_down,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio3].
            FunctionPio0, PullNone: Gp3Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio3].
            FunctionPio1, PullNone: Gp3Pio1
        }
    },

    /// GPIO 4 is connected to _joystick left_ and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp4Pio0]            |
    /// | `PIO1`       | [crate::Gp4Pio1]            |
    Gpio4 {
        name: j_left,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio4].
            FunctionPio0, PullNone: Gp4Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio4].
            FunctionPio1, PullNone: Gp4Pio1
        }
    },

    /// GPIO 5 is connected to the red LED and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM2 B`     | [crate::Gp5Pwm2B]           |
    /// | `PIO0`       | [crate::Gp5Pio0]            |
    /// | `PIO1`       | [crate::Gp5Pio1]            |
    Gpio5 {
        name: red_led,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::gpio5].
            FunctionPwm, PullNone: Gp5Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::gpio5].
            FunctionPio0, PullNone: Gp5Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio5].
            FunctionPio1, PullNone: Gp5Pio1
        }
    },

    /// GPIO 6 is connected to the yellow LED and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// | `PWM3 A`     | [crate::Gp6Pwm3A]           |
    /// | `PIO0`       | [crate::Gp6Pio0]            |
    /// | `PIO1`       | [crate::Gp6Pio1]            |
    Gpio6 {
        name: yellow_led,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::gpio6].
            FunctionPwm, PullNone: Gp6Pwm3A,
            /// PIO0 Function alias for pin [crate::Pins::gpio6].
            FunctionPio0, PullNone: Gp6Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio6].
            FunctionPio1, PullNone: Gp6Pio1
        }
    },

    /// GPIO 7 is connected to the green LED and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM3 B`     | [crate::Gp7Pwm3B]           |
    /// | `PIO0`       | [crate::Gp7Pio0]            |
    /// | `PIO1`       | [crate::Gp7Pio1]            |
    Gpio7 {
        name: green_led,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::gpio7].
            FunctionPwm, PullNone: Gp7Pwm3B,
            /// PIO0 Function alias for pin [crate::Pins::gpio7].
            FunctionPio0, PullNone: Gp7Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio7].
            FunctionPio1, PullNone: Gp7Pio1
        }
    },

    /// GPIO 8 is connected to the OLED data/cmd select pin (`OLED_DC`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp8Pio0]            |
    /// | `PIO1`       | [crate::Gp8Pio1]            |
    Gpio8 {
        name: oled_dc,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio8].
            FunctionPio0, PullNone: Gp8Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio8].
            FunctionPio1, PullNone: Gp8Pio1
        }
    },

    /// GPIO 9 is connected to the OLED reset pin (`OLED_RST`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp9Pio0]            |
    /// | `PIO1`       | [crate::Gp9Pio1]            |
    Gpio9 {
        name: oled_rst,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio9].
            FunctionPio0, PullNone: Gp9Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio9].
            FunctionPio1, PullNone: Gp9Pio1
        }
    },

    /// GPIO 10 is connected to the OLED chip select pin (`OLED_CS`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PIO0`       | [crate::Gp10Pio0]           |
    /// | `PIO1`       | [crate::Gp10Pio1]           |
    Gpio10 {
        name: oled_cs,
        aliases: {
            /// PIO0 Function alias for pin [crate::Pins::gpio10].
            FunctionPio0, PullNone: Gp10Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio10].
            FunctionPio1, PullNone: Gp10Pio1
        }
    },

    /// GPIO 11 is not connected.
    Gpio11 {
        name: gpio11,
    },

    /// GPIO 12 is not connected.
    Gpio12 {
        name: gpio12,
    },

    /// GPIO 13 is not connected.
    Gpio13 {
        name: gpio13,
    },

    /// GPIO 14 is not connected.
    Gpio14 {
        name: gpio14,
    },

    /// GPIO 15 is connected to the passive buzzer and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM7 B`     | [crate::Gp15Pwm7B]          |
    /// | `PIO0`       | [crate::Gp15Pio0]           |
    /// | `PIO1`       | [crate::Gp15Pio1]           |
    Gpio15 {
        name: buzzer,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::gpio15].
            FunctionPwm, PullNone: Gp15Pwm7B,
            /// PIO0 Function alias for pin [crate::Pins::gpio15].
            FunctionPio0, PullNone: Gp15Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio15].
            FunctionPio1, PullNone: Gp15Pio1
        }
    },

    /// GPIO 16 is not connected and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp16Spi0Rx]         |
    /// | `PIO0`       | [crate::Gp16Pio0]           |
    /// | `PIO1`       | [crate::Gp16Pio1]           |
    Gpio16 {
        name: oled_rx,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio16].
            FunctionSpi, PullNone: Gp16Spi0Rx,
            /// PIO0 Function alias for pin [crate::Pins::gpio16].
            FunctionPio0, PullNone: Gp16Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio16].
            FunctionPio1, PullNone: Gp16Pio1
        }
    },

    /// GPIO 17 is not connected supports following functions:
    Gpio17 {
        name: gpio17,
    },

    /// GPIO 18 is connected to the OLED serial clock (`OLED_CLK`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp18Spi0Sck]        |
    /// | `PIO0`       | [crate::Gp18Pio0]           |
    /// | `PIO1`       | [crate::Gp18Pio1]           |
    Gpio18 {
        name: oled_clk,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio18].
            FunctionSpi, PullNone: Gp18Spi0Sck,
            /// PIO0 Function alias for pin [crate::Pins::gpio18].
            FunctionPio0, PullNone: Gp18Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio18].
            FunctionPio1, PullNone: Gp18Pio1
        }
    },

    /// GPIO 19 is connected to the OLED data in (`OLED_DIN`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp19Spi0Tx]         |
    /// | `PIO0`       | [crate::Gp19Pio0]           |
    /// | `PIO1`       | [crate::Gp19Pio1]           |
    Gpio19 {
        name: oled_din,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio19].
            FunctionSpi, PullNone: Gp19Spi0Tx,
            /// PIO0 Function alias for pin [crate::Pins::gpio19].
            FunctionPio0, PullNone: Gp19Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio19].
            FunctionPio1, PullNone: Gp19Pio1
        }
    },

    /// GPIO 20 is connected to the accelerometer serial data (`ACCS_SDA`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `I2C0 SDA`   | [crate::Gp20I2C0Sda]        |
    /// | `PIO0`       | [crate::Gp20Pio0]           |
    /// | `PIO1`       | [crate::Gp20Pio1]           |
    Gpio20 {
        name: accs_sda,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio20].
            FunctionI2C, PullUp: Gp20I2C0Sda,
            /// PIO0 Function alias for pin [crate::Pins::gpio20].
            FunctionPio0, PullNone: Gp20Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio20].
            FunctionPio1, PullNone: Gp20Pio1
        }
    },

    /// GPIO 21 is connected to the accelerometer serial clock (`ACCS_SCL`)
    /// and supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `I2C0 SCL`   | [crate::Gp21I2C0Scl]        |
    /// | `PIO0`       | [crate::Gp21Pio0]           |
    /// | `PIO1`       | [crate::Gp21Pio1]           |
    Gpio21 {
        name: accs_scl,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio21].
            FunctionI2C, PullUp: Gp21I2C0Scl,
            /// PIO0 Function alias for pin [crate::Pins::gpio21].
            FunctionPio0, PullNone: Gp21Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio21].
            FunctionPio1, PullNone: Gp21Pio1
        }
    },

    /// GPIO 22 is not connected.
    Gpio22 {
        name: gpio22,
    },

    /// GPIO 23 is not connected.
    Gpio23 {
        name: b_power_save,
    },

    /// GPIO 24 is not connected.
    Gpio24 {
        name: vbus_detect,
    },

    /// GPIO 25 is not connected.
    Gpio25 {
        name: led,
    },

    /// GPIO 26 is not connected.
    Gpio26 {
        name: gpio26,
    },

    /// GPIO 27 is not connected.
    Gpio27 {
        name: gpio27,
    },

    /// GPIO 28 is not connected.
    Gpio28 {
        name: gpio28,
    },

    /// GPIO 29 is not connected.
    Gpio29 {
        name: gpio29
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
