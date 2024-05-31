#![no_std]
#![no_main]

use defmt::{info, error, panic};
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_stm32::adc::{Adc, Resolution, SampleTime};
use embassy_stm32::gpio::{Level, Output, OutputType, Pin, Speed};
use embassy_stm32::peripherals::{DMA1_CH4, DMA1_CH5, I2C1, PD4, TIM1, TIM3, TIM4, USB_OTG_HS};
use embassy_stm32::time::{Hertz, khz};
use embassy_stm32::timer::{Channel, OutputPolarity};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Timer, /*Instant,*/ Delay};
// use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
// use embassy_stm32::mode::Async;
// use static_cell::StaticCell;
// use embassy_sync::blocking_mutex::NoopMutex;
use embassy_stm32::i2c::{self, Error, I2c};

use embassy_futures::join::*;
use embassy_stm32::usb_otg::{Driver, Instance, InterruptHandler};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embassy_usb::class::midi::MidiClass;

use pid::Pid;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use embassy_usb::class::midi;

use {defmt_rtt as _, panic_probe as _};

const ADDRESS: u8 = 0x1B;
const HARDWARE_ID: u8 = 0x00;
const FIRMWARE_VERSION: u8 = 0x01;
const DETECTION_STATUS: u8 = 0x02;
const KEY_STATUS: u8 = 0x03;
/*const KEY_0_NTHR: u8 = 32;
const NTHR: u8 = 20;
const KEY_5_NTHR: u8 = 37;*/
const KEY_0_AKS: u8 = 39;
const AKE_AKS: u8 = 0b001000_00; // 8 sample averaging, no adjacent key suppression
const MAX_ON_DURATION: u8 = 55;
const CALIBRATE: u8 = 56;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    OTG_HS => InterruptHandler<USB_OTG_HS>;
});


static SLIDER_1_MEAS: AtomicU16 = AtomicU16::new(0);
static SLIDER_1_DES: AtomicU16 = AtomicU16::new(0);

static SLIDER_1_TOUCH: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();

    let mut p = embassy_stm32::init(config);

    info!("Starting program...");

    // ADC Initialization
    let mut delay = Delay;
    let mut adc = Adc::new(p.ADC1, &mut delay);
    adc.set_sample_time(SampleTime::Cycles16_5);
    adc.set_resolution(Resolution::EightBit);

    // Capacitive Touch Sensor Initialization
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Hertz(100_000),
        Default::default(),
    );

    let mut data = [0u8; 1];

    match i2c.write_read(ADDRESS, &[HARDWARE_ID], &mut data).await {
        Ok(()) => info!("Whoami: {:#01x}", data[0]),
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }
    match i2c.write_read(ADDRESS, &[FIRMWARE_VERSION], &mut data).await {
        Ok(()) => info!("Firmware version: {:#01x}", data[0]),
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    for i in 0..5 {
        match i2c.write(ADDRESS, &[KEY_0_AKS + i, AKE_AKS]).await {
            Ok(()) => info!("Set Key {} AKE/AKS", i),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
    }

    match i2c.write(ADDRESS, &[MAX_ON_DURATION, 138]).await {
        Ok(()) => info!("Set Key Max on Duration to 22 seconds"),
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    match i2c.write(ADDRESS, &[CALIBRATE, 5]).await {
        Ok(()) => info!("Started calibration"),
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    loop {
        match i2c.write_read(ADDRESS, &[DETECTION_STATUS], &mut data).await {
            // Ok(()) => info!("Detection Status: {:#010b}", data[0]),
            Ok(()) => (),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        let calibration = ((1 << 7) & data[0]) > 0;
        if !calibration {
            info!("Calibration finished");
            break;
        } else {
            Timer::after_millis(50).await;
        }
    }

    // Motor initialization
    let mot1_sleep = p.PD4;
    let mot1_in1 = p.PD12;
    let mot1_in2 = p.PD13;

    let mut sleep = Output::new(mot1_sleep, Level::High, Speed::Low);
    sleep.set_high();

    let ch1 = PwmPin::new_ch1(mot1_in1, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(mot1_in2, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM4, Some(ch1), Some(ch2), None, None, khz(20), Default::default());
    pwm.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow); // inverted PWM
    pwm.set_polarity(Channel::Ch2, OutputPolarity::ActiveLow);

    enable_motor(&mut pwm, 'a');

    let duty = 80_i16;

    let desired_position = SLIDER_1_DES.load(Ordering::Relaxed);
    let mut pid = Pid::new(desired_position as f32, 100_f32);
    pid.p(0.75, 100_f32);
    pid.i(1.7, 25_f32);
    spawner.spawn(get_touch(i2c, sleep)).unwrap();


    // USB Initialization
    let mut ep_out_buffer = [0u8; 256];
    let mut config = usb_otg::Config::default();
    config.vbus_detection = false; // Necessary since STM32 is powered solely by USB
    let driver = Driver::new_fs(p.USB_OTG_HS, Irqs, p.PA12, p.PA11, &mut ep_out_buffer, config);
    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-MIDI example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;
    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );
    let mut class = MidiClass::new(&mut builder, 1, 1, 64);
    // The `MidiClass` can be split into `Sender` and `Receiver`, to be used in separate tasks.
    let (mut sender, mut receiver) = class.split();
    let mut usb = builder.build();
    let usb_fut = usb.run();

    // Use the Midi class!
    let midi_send_fut = async {
        loop {
            sender.wait_connection().await;
            info!("Connected");
            let _ = midi_send(&mut sender).await;
            info!("Disconnected");
        }
    };
    
    let midi_recv_fut = async {
        loop {
            receiver.wait_connection().await;
            // info!("Connected");
            let _ = midi_recv(&mut receiver).await;
            // info!("Disconnected");
        }
    };

    let slider_fut = async {
        loop {
            let measured = adc.read(&mut p.PC0) >> 1;
            // info!("measured: {}", measured);
            
            SLIDER_1_MEAS.store(measured, Ordering::Relaxed);
            if SLIDER_1_TOUCH.load(Ordering::Relaxed) {
                SLIDER_1_DES.store(measured, Ordering::Relaxed);
            }

            pid.setpoint(SLIDER_1_DES.load(Ordering::Relaxed) as f32);
            let output = pid.next_control_output(measured as f32);
            let mut correction = output.output as i16;
            if correction.abs() > 5_i16 {
                correction += 20_i16 * correction.signum();
            }
            
            set_motor_duty(&mut pwm, 'a', correction);
            
            Timer::after_millis(35).await;
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join4(usb_fut, midi_send_fut, midi_recv_fut, slider_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn midi_recv<'d, T: Instance + 'd>(class: &mut midi::Receiver<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        // info!("data: {:x}", data);
        SLIDER_1_DES.store(data[3] as u16, Ordering::Relaxed);
    }
}

async fn midi_send<'d, T: Instance + 'd>(class: &mut midi::Sender<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut previous_position: u8 = 0;
    loop {
        let slider_position = (SLIDER_1_MEAS.load(Ordering::Relaxed).clamp(0,127)) as u8;
        if (previous_position != slider_position) && (SLIDER_1_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 16, slider_position]).await?; // First byte is cable 0, B for control change
            previous_position = slider_position;
        }
        Timer::after_millis(20).await;
    }
}

#[embassy_executor::task]
async fn get_touch(i2c: I2c<'static, I2C1, DMA1_CH4, DMA1_CH5>, sleep: Output<'static, PD4>) {
    let mut data = [0u8; 1];
    let mut i2c_task = i2c;
    let mut sleep_task = sleep;
    loop {
        match i2c_task.write_read(ADDRESS, &[KEY_STATUS], &mut data).await {
            // Ok(()) => info!("Key Status: {:#010b}", data[0]),
            Ok(()) => (),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        let mot1_sense = ((1 << 4) & data[0]) > 0;
        SLIDER_1_TOUCH.store(mot1_sense, Ordering::Relaxed);
        // let mot2_sense = ((1 << 3) & data[0]) > 0;
        // let mot3_sense = ((1 << 2) & data[0]) > 0;
        // let mot4_sense = ((1 << 1) & data[0]) > 0;
        // let mot5_sense = ((1 << 0) & data[0]) > 0;
        if mot1_sense {
            // info!("Mot1 touched");
            sleep_task.set_low();
        } else {
            sleep_task.set_high();
        }
        Timer::after_millis(25).await;
    }
}

/// Enable a motor by enabling the PWM channels
fn enable_motor(pwm: &mut SimplePwm<TIM4>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.enable(channels[0]);
    pwm.enable(channels[1]);
}

/// Disable a motor by disabling the PWM channels
fn disable_motor(pwm: &mut SimplePwm<TIM4>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.disable(channels[0]);
    pwm.disable(channels[1]);
    set_motor_duty(pwm, motor, 0);
}

/// Set the duty cycle of a motor
fn set_motor_duty(pwm: &mut SimplePwm<TIM4>, motor: char, duty: i16) {
    let clamped_duty = duty.clamp(-100, 100) as i32;
    let max = pwm.get_max_duty() as u32;

    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    // info!("Motor: {}", motor);
    // info!("Clamped Duty: {}", clamped_duty);

    if duty == 0 {
        pwm.set_duty(channels[0], 0);
        pwm.set_duty(channels[1], 0);
    } else if duty <= 0 {
        pwm.set_duty(channels[0], 0);
        pwm.set_duty(channels[1], (clamped_duty.unsigned_abs()*max/100) as u16);
    } else {
        pwm.set_duty(channels[0], (clamped_duty.unsigned_abs()*max/100) as u16);
        pwm.set_duty(channels[1], 0);
    }
}