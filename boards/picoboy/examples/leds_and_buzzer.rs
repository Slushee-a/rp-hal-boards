#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, pwm,
    sio::Sio,
    watchdog::Watchdog,
};
use bsp::{entry, XOSC_CRYSTAL_FREQ};
use picoboy as bsp;

use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};

use defmt_rtt as _;
use panic_halt as _;

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut red_led = pins.red_led.into_push_pull_output();
    let mut yellow_led = pins.yellow_led.into_push_pull_output();
    let mut green_led = pins.green_led.into_push_pull_output();

    // Init PWMs
    let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // The buzzer on the Picoboy uses the PWM slice 7 and the channel B
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_div_int(125);
    pwm.set_top(1135); // 440Hz
    pwm.enable();

    let buzzer = &mut pwm.channel_b;
    buzzer.output_to(pins.buzzer);
    buzzer.set_duty_cycle_percent(50).unwrap(); // 50% duty cycle
    buzzer.set_enabled(false); // Boot without making noise

    loop {
        red_led.set_high().unwrap();
        delay.delay_ms(500);

        yellow_led.set_high().unwrap();
        delay.delay_ms(500);

        green_led.set_high().unwrap();
        buzzer.set_enabled(true);
        delay.delay_ms(500);

        red_led.set_low().unwrap();
        buzzer.set_enabled(true);
        delay.delay_ms(500);

        yellow_led.set_low().unwrap();
        delay.delay_ms(500);

        green_led.set_low().unwrap();
        delay.delay_ms(500);
    }
}
