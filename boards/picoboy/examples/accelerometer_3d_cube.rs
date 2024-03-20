#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

use bsp::entry;
use bsp::{
    hal::{
        clocks::init_clocks_and_plls,
        clocks::Clock,
        fugit::RateExtU32,
        gpio::{FunctionSpi, PinState},
        i2c::I2C,
        pac,
        spi::Spi,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use embedded_hal::spi::MODE_3;
use picoboy as bsp;

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use sh1106::{prelude::*, Builder};

use accelerometer::Accelerometer;
use stk8ba58::Stk8ba58;

use core::fmt::Write;
use heapless::String;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Define all the OLED's pins
    let oled_tx = pins.oled_tx.into_function::<FunctionSpi>();
    let oled_rx = pins.oled_rx.into_function::<FunctionSpi>();
    let oled_clk = pins.oled_clk.into_function::<FunctionSpi>();
    let oled_dc = pins.oled_dc.into_push_pull_output();
    let oled_cs = pins.oled_cs.into_push_pull_output();
    let _oled_rst = pins.oled_rst.into_push_pull_output_in_state(PinState::High); // Reset pin must
                                                                                  // be pulled high
                                                                                  // Define and start the OLED's SPI interface
    let oled_spi = Spi::new(pac.SPI0, (oled_tx, oled_rx, oled_clk));

    let oled_spi: Spi<_, _, _, 8> = oled_spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1u32.MHz(),
        MODE_3,
    );

    // Define the OLED as an object we can draw to
    let mut oled: GraphicsMode<_> = Builder::new().connect_spi(oled_spi.into(), oled_dc).into();
    oled.init().unwrap();

    let accel_i2c = I2C::i2c0(
        pac.I2C0,
        pins.accs_sda.reconfigure(),
        pins.accs_scl.reconfigure(),
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let mut accelerometer = Stk8ba58::new(accel_i2c);

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);

    loop {
        let xyz = accelerometer.accel_norm().unwrap();
        let x = xyz.x;
        let y = xyz.y;
        let z = xyz.z;
        let mut temp_str: String<64> = String::new();
        write!(&mut temp_str, "X:{x}\nY:{y}\nZ:{z}").unwrap();

        oled.clear();
        Text::new(&temp_str, Point::new(0, 10), style)
            .draw(&mut oled.into())
            .unwrap();
        oled.flush().unwrap();
        delay.delay_ms(200);
    }
}
