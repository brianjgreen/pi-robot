use hc_sr04::{HcSr04, Unit};
use rppal::gpio::Gpio;
use std::{thread, time};
mod display;
use crate::display::{get_display, write_display};

fn main() {
    let smbus = get_display();
    write_display(&smbus, "2468");

    // IR mid left
    let gpio17 = Gpio::new()
        .expect("GPIO17 no good")
        .get(17)
        .expect("GPIO17 in use");

    // IR right
    let gpio18 = Gpio::new()
        .expect("GPIO18 no good")
        .get(18)
        .expect("GPIO18 in use");

    // IR mid right
    let gpio22 = Gpio::new()
        .expect("GPIO22 no good")
        .get(22)
        .expect("GPIO22 in use");

    // IR left
    let gpio23 = Gpio::new()
        .expect("GPIO23 no good")
        .get(23)
        .expect("GPIO23 in use");

    // IR mid
    let gpio27 = Gpio::new()
        .expect("GPIO27 no good")
        .get(27)
        .expect("GPIO27 in use");

    for _ in 1..10 {
        println!(
            "17={} 18={} 22={} 23={} 27={}",
            gpio17.read(),
            gpio18.read(),
            gpio22.read(),
            gpio23.read(),
            gpio27.read()
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

        write_display(&smbus, format!("{:0>4}", message).as_str());
        thread::sleep(time::Duration::from_millis(500));
    }
}
