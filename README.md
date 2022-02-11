![build_workflow](https://github.com/robhany/lp5018/actions/workflows/rust.yml/badge.svg)
[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]
![No Std][no-std-badge]

# TDC1000

This crate is a no_std driver for the TDC1000 Ultrasonic Sensing Analog Front End

## Datasheet

https://www.ti.com/lit/gpn/tdc1000


## About this driver
This driver allows you to configure the tdc1000 analog frontend device via spi.
This driver works on an NUCLEO-L433RC when compiled in release mode.

## Usage
Add this to your Cargo.toml:

```toml
[dependencies]
tdc1000 = "0.1.2"
```

And this to your main.rs

```rust
//SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mut cs = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    cs.set_high().unwrap();

    //TDC 1000 enable
    let mut enable_pin = gpioc
        .pc8
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        enable_pin.set_low().unwrap();

    //TDC 1000 trigger
    let mut trigger_pin = gpioc
    .pc9
    .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    trigger_pin.set_low().unwrap();

    //TDC 1000 start pulse pin
    let start_pin = gpioc
    .pc7
    .into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);

    //TDC 1000 stop pulse pin
    let stop_pin = gpioc
    .pc6
    .into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);

    let mut tdc1000 = Tdc1000::default();
    tdc1000.set_tx_frequency_divider(TxFrequencyDivider::DivideBy8);
    tdc1000.set_number_of_tx_pulses(TxPulses::new(1)); 
    tdc1000.set_tx_pulse_shift_position(TxPulseShiftPosition::new(5));
    tdc1000.set_time_of_flight(TimeOfFlightValue::new(TimeOfFlightValue::HIGH));
    tdc1000.set_short_tof_blank_period(ShortTofBlankPeriod::T0Times32);
    tdc1000.write_settings(&mut cs, &mut spi).unwrap();
    

    loop {
        enable_pin.set_high().unwrap();
        delay_ms(10_u32);
        let mut timeout = 0_u32;
        while start_pin.is_low().unwrap() && timeout < SOME_TIMEOUT {
            timeout += 1;
        }
        let start_cnt = DWT::get_cycle_count(); // Use cycle count to measure time

        timeout = 0;
        while stop_pin.is_low().unwrap() && timeout < SOME_TIMEOUT {
            timeout += 1;
        }
        let stop_ctn = DWT::get_cycle_count();

        let measured_cycles = stop_cnt - start_cnt; 
        
        enable_pin.set_high().unwrap();
        delay_ms(5000_u32);
    }
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

<!-- Badges -->
[crates-io]: https://crates.io/crates/tdc1000
[crates-io-badge]: https://img.shields.io/crates/v/tdc1000.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/tdc1000
[crates-io-download-badge]: https://img.shields.io/crates/d/tdc1000.svg?maxAge=3600
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue
