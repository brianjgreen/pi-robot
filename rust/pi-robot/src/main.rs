use hc_sr04::{HcSr04, Unit};
use rppal::gpio::Gpio;
use rppal::i2c::I2c;
use std::{thread, time};

fn char_to_led(alpha_num: char) -> u8 {
    match alpha_num {
        ' ' => 0x00,
        '-' => 0x40,
        '0' => 0x3f,
        '1' => 0x06,
        '2' => 0x5b,
        '3' => 0x4F,
        '4' => 0x66,
        '5' => 0x6D,
        '6' => 0x7D,
        '7' => 0x07,
        '8' => 0x7F,
        '9' => 0x6F,
        'A' => 0x77,
        'B' => 0x7C,
        'C' => 0x39,
        'D' => 0x5E,
        'E' => 0x79,
        'F' => 0x71,
        'H' => 0x76,
        'L' => 0x38,
        _ => 0x00,
    }
}

fn main() {
    let mut smbus: I2c = I2c::new().expect("Unable to init i2c bus");
    let _ = smbus.set_slave_address(0x71);
    let mut display = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let led_digit = [0, 2, 6, 8];

    // Hold in reset
    let _ = smbus.smbus_send_byte(0x21);
    // Prescale 0x03 is about 1600 Hz
    let _ = smbus.smbus_send_byte(0x81);
    // Come out of reset, enable auto register address increment and run!
    let _ = smbus.smbus_send_byte(0xE8);

    let mut i = 0;
    let message = "2468";
    for m in message.chars() {
        display[led_digit[i]] = char_to_led(m);
        i += 1;
    }
    let _ = smbus.block_write(0x00, &display);

    let gpio17 = Gpio::new(); // IR mid left
    let gpio18 = Gpio::new(); // IR right
    let gpio22 = Gpio::new(); // IR mid right
    let gpio23 = Gpio::new(); // IR left
    let gpio27 = Gpio::new(); // IR mid
    let pin17 = gpio17
        .expect("GPIO17 no good")
        .get(17)
        .expect("GPIO17 in use");
    let pin18 = gpio18
        .expect("GPIO18 no good")
        .get(18)
        .expect("GPIO18 in use");
    let pin22 = gpio22
        .expect("GPIO22 no good")
        .get(22)
        .expect("GPIO22 in use");
    let pin23 = gpio23
        .expect("GPIO23 no good")
        .get(23)
        .expect("GPIO23 in use");
    let pin27 = gpio27
        .expect("GPIO27 no good")
        .get(27)
        .expect("GPIO27 in use");

    for _ in 1..10 {
        println!(
            "17={} 18={} 22={} 23={} 27={}",
            pin17.read(),
            pin18.read(),
            pin22.read(),
            pin23.read(),
            pin27.read()
        );
        thread::sleep(time::Duration::from_millis(500));
    }

    // Initialize driver.
    let mut ultrasonic = HcSr04::new(
        21,   // TRIGGER -> Gpio 21
        26,   // ECHO -> Gpio 26
        None, // Ambient temperature (if `None` defaults to 20.0C)
    )
    .unwrap();

    loop {
        // Perform distance measurement, specifying measuring unit of return value.
        let mut message = "ELLO".to_string();
        match ultrasonic.measure_distance(Unit::Centimeters).unwrap() {
            Some(dist) => {
                println!("Distance: {}cm", dist);
                message = dist.floor().to_string()
            }
            None => println!("Object out of range"),
        }
        let mut i = 0;
        for m in format!("{:0>4}", message).chars() {
            display[led_digit[i]] = char_to_led(m);
            i += 1;
        }
        let _ = smbus.block_write(0x00, &display);
        thread::sleep(time::Duration::from_millis(500));
    }
}
