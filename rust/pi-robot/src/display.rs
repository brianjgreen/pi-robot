use rppal::i2c::I2c;

pub fn get_display() -> I2c {
    let mut smbus: I2c = I2c::new().expect("Unable to init i2c bus");
    smbus
        .set_slave_address(0x71)
        .expect("Unable to address target 0x71");
    smbus
        .smbus_send_byte(0x21) // Hold in reset
        .expect("Failed to hold LED controller in reset");
    smbus
        .smbus_send_byte(0x81) // Prescale 0x03 is about 1600 Hz
        .expect("Failed to prescale LED controller");
    smbus
        .smbus_send_byte(0xE8) // Come out of reset, enable auto register address increment and run!
        .expect("Failed to come out of reset or enable auto reg addr inc");

    smbus
}

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

pub fn write_display(smbus: &I2c, message: &str) {
    let mut display: [u8; 16] = [0; 16];
    let led_digit = [0, 2, 6, 8];

    let mut i = 0;
    for m in message.chars() {
        display[led_digit[i]] = char_to_led(m);
        i += 1;
    }
    smbus
        .block_write(0x00, &display)
        .expect("Failed to send LED commands to controller");
}
