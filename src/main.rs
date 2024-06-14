#![no_std]
#![no_main]

use defmt::{info, error, panic};
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, usb_otg, Config};
use embassy_stm32::adc::{Adc, SampleTime, Resolution};
use embassy_stm32::gpio::{AnyPin, Level, Output, OutputType, Speed};
use embassy_stm32::peripherals::{DMA1_CH4, DMA1_CH5, I2C1, TIM3, TIM4, USB_OTG_HS};
use embassy_stm32::time::{Hertz, khz};
use embassy_stm32::timer::{CaptureCompare16bitInstance, Channel, OutputPolarity};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Timer, Delay};
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
const KEY_0_NTHR: u8 = 32;
const NTHR: u8 = 30;
// const KEY_5_NTHR: u8 = 37;
const KEY_0_AKS: u8 = 39;
const AKE_AKS: u8 = 0b100000_00; // 32 sample averaging, no adjacent key suppression
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
static SLIDER_2_MEAS: AtomicU16 = AtomicU16::new(0);
static SLIDER_2_DES: AtomicU16 = AtomicU16::new(0);
static SLIDER_2_TOUCH: AtomicBool = AtomicBool::new(false);
static SLIDER_3_MEAS: AtomicU16 = AtomicU16::new(0);
static SLIDER_3_DES: AtomicU16 = AtomicU16::new(0);
static SLIDER_3_TOUCH: AtomicBool = AtomicBool::new(false);
static SLIDER_4_MEAS: AtomicU16 = AtomicU16::new(0);
static SLIDER_4_DES: AtomicU16 = AtomicU16::new(0);
static SLIDER_4_TOUCH: AtomicBool = AtomicBool::new(false);
static SLIDER_5_MEAS: AtomicU16 = AtomicU16::new(0);
static SLIDER_5_DES: AtomicU16 = AtomicU16::new(0);
static SLIDER_5_TOUCH: AtomicBool = AtomicBool::new(false);


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();

    /*let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }*/
    
    let mut p = embassy_stm32::init(config);

    info!("Starting program...");

    // ADC Initialization
    let mut delay = Delay;
    // let mut adc1 = Adc::new(p.ADC1, &mut delay);
    let mut adc2 = Adc::new(p.ADC2, &mut delay);
    // adc1.set_sample_time(SampleTime::Cycles16_5);
    // adc1.set_resolution(Resolution::EightBit);
    adc2.set_sample_time(SampleTime::Cycles16_5);
    adc2.set_resolution(Resolution::EightBit);
    
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

    for i in 0..5 {
        match i2c.write(ADDRESS, &[KEY_0_NTHR+i, NTHR]).await {
            Ok(()) => info!("Set Key {} NTHR", i),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        /*let mut data = [0u8; 1];
        match i2c.write_read(ADDRESS, &[KEY_0_NTHR+i], &mut data).await {
            Ok(()) => info!("Key {} NTHR: {}", i, data[0]),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }*/
    }

    for _i in 0..1 {
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
                info!("Calibration not finished");
                Timer::after_millis(50).await;
            }
        }
    }

    // LCD initialization
    let lcd_backlight = p.PC12;
    let lcd_backlight_channel = PwmPin::new_ch1(lcd_backlight, OutputType::PushPull);
    let mut lcd_backlight_pwm = SimplePwm::new(p.TIM15, Some(lcd_backlight_channel), None, None, None, khz(50), Default::default());
    
    let backlight_duty = 50_u16;
    let lcd_backlight_pwm_max = lcd_backlight_pwm.get_max_duty();
    lcd_backlight_pwm.set_duty(Channel::Ch1, backlight_duty*lcd_backlight_pwm_max/100);
    
    // Motor initialization
    let mot1_sleep = p.PD4;
    let mot1_in1 = p.PD12;
    let mot1_in2 = p.PD13;
    let mot2_sleep = p.PD5;
    let mot2_in1 = p.PD14;
    let mot2_in2 = p.PD15;
    let mot3_sleep = p.PD6;
    let mot3_in1 = p.PE9;
    let mot3_in2 = p.PE11;
    let mot4_sleep = p.PD7;
    let mot4_in1 = p.PB4;
    let mot4_in2 = p.PB5;
    let mot5_sleep = p.PD8;
    let mot5_in1 = p.PB0;
    let mot5_in2 = p.PB1;

    let mut sleep1 = Output::new(mot1_sleep, Level::High, Speed::Low);
    let mut sleep2 = Output::new(mot2_sleep, Level::High, Speed::Low);
    let mut sleep3 = Output::new(mot3_sleep, Level::High, Speed::Low);
    let mut sleep4 = Output::new(mot4_sleep, Level::High, Speed::Low);
    let mut sleep5 = Output::new(mot5_sleep, Level::High, Speed::Low);
    sleep1.set_high();
    sleep2.set_high();
    sleep3.set_high();
    sleep4.set_high();
    sleep5.set_high();

    let mot1_ch1 = PwmPin::new_ch1(mot1_in1, OutputType::PushPull);
    let mot1_ch2 = PwmPin::new_ch2(mot1_in2, OutputType::PushPull);
    let mut mot1_pwm = SimplePwm::new(p.TIM4, Some(mot1_ch1), Some(mot1_ch2), None, None, khz(20), Default::default());
    let mot2_ch1 = PwmPin::new_ch3(mot2_in1, OutputType::PushPull);
    let mot2_ch2 = PwmPin::new_ch4(mot2_in2, OutputType::PushPull);
    let mut mot2_pwm = SimplePwm::new(unsafe {TIM4::steal()}, None, None, Some(mot2_ch1), Some(mot2_ch2), khz(20), Default::default());
    let mot3_ch1 = PwmPin::new_ch1(mot3_in1, OutputType::PushPull);
    let mot3_ch2 = PwmPin::new_ch2(mot3_in2, OutputType::PushPull);
    let mut mot3_pwm = SimplePwm::new(p.TIM1, Some(mot3_ch1), Some(mot3_ch2), None, None, khz(20), Default::default());
    let mot4_ch1 = PwmPin::new_ch1(mot4_in1, OutputType::PushPull);
    let mot4_ch2 = PwmPin::new_ch2(mot4_in2, OutputType::PushPull);
    let mut mot4_pwm = SimplePwm::new(p.TIM3, Some(mot4_ch1), Some(mot4_ch2), None, None, khz(20), Default::default());
    let mot5_ch1 = PwmPin::new_ch3(mot5_in1, OutputType::PushPull);
    let mot5_ch2 = PwmPin::new_ch4(mot5_in2, OutputType::PushPull);
    let mut mot5_pwm = SimplePwm::new(unsafe {TIM3::steal()}, None, None, Some(mot5_ch1), Some(mot5_ch2), khz(20), Default::default());
    
    mot1_pwm.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow); // inverted PWM
    mot1_pwm.set_polarity(Channel::Ch2, OutputPolarity::ActiveLow);
    mot2_pwm.set_polarity(Channel::Ch3, OutputPolarity::ActiveLow);
    mot2_pwm.set_polarity(Channel::Ch4, OutputPolarity::ActiveLow);
    mot3_pwm.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow);
    mot3_pwm.set_polarity(Channel::Ch2, OutputPolarity::ActiveLow);
    mot4_pwm.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow);
    mot4_pwm.set_polarity(Channel::Ch2, OutputPolarity::ActiveLow);
    mot5_pwm.set_polarity(Channel::Ch3, OutputPolarity::ActiveLow);
    mot5_pwm.set_polarity(Channel::Ch4, OutputPolarity::ActiveLow);

    enable_motor(&mut mot1_pwm, 'a');
    enable_motor(&mut mot2_pwm, 'b');
    enable_motor(&mut mot3_pwm, 'a');
    enable_motor(&mut mot4_pwm, 'a');
    enable_motor(&mut mot5_pwm, 'b');

    // let duty = 80_i16;

    let desired_position1 = SLIDER_1_DES.load(Ordering::Relaxed);
    let desired_position2 = SLIDER_2_DES.load(Ordering::Relaxed);
    let desired_position3 = SLIDER_3_DES.load(Ordering::Relaxed);
    let desired_position4 = SLIDER_4_DES.load(Ordering::Relaxed);
    let desired_position5 = SLIDER_5_DES.load(Ordering::Relaxed);
    
    let mut pid1 = Pid::new(desired_position1 as f32, 100_f32);
    let mut pid2 = Pid::new(desired_position2 as f32, 100_f32);
    let mut pid3 = Pid::new(desired_position3 as f32, 100_f32);
    let mut pid4 = Pid::new(desired_position4 as f32, 100_f32);
    let mut pid5 = Pid::new(desired_position5 as f32, 100_f32);

    let pid_p = 0.85; // 0.75
    let pid_p_lim = 100_f32;
    let pid_i = 1.7;
    let pid_i_lim = 25_f32;
    let pid_d = 0.0;
    let pid_d_lim = 0_f32;
    pid1.p(pid_p, pid_p_lim);
    pid1.i(pid_i, pid_i_lim);
    pid1.d(pid_d, pid_d_lim);
    pid2.p(pid_p, pid_p_lim);
    pid2.i(pid_i, pid_i_lim);
    pid2.d(pid_d, pid_d_lim);
    pid3.p(pid_p, pid_p_lim);
    pid3.i(pid_i, pid_i_lim);
    pid3.d(pid_d, pid_d_lim);
    pid4.p(pid_p, pid_p_lim);
    pid4.i(pid_i, pid_i_lim);
    pid4.d(pid_d, pid_d_lim);
    pid5.p(pid_p, pid_p_lim);
    pid5.i(pid_i, pid_i_lim);
    pid5.d(pid_d, pid_d_lim);
    
    spawner.spawn(get_touch(i2c, sleep1.degrade(), sleep2.degrade(), sleep3.degrade(), sleep4.degrade(), sleep5.degrade())).unwrap();


    // USB Initialization
    let mut ep_out_buffer = [0u8; 256];
    let mut config = usb_otg::Config::default();
    config.vbus_detection = false; // Necessary since STM32 is powered solely by USB
    let driver = Driver::new_fs(p.USB_OTG_HS, Irqs, p.PA12, p.PA11, &mut ep_out_buffer, config);
    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("CollapseOfSound");
    config.product = Some("Flying Fader");
    config.serial_number = Some("00000001");
    config.max_power = 200;
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
    let class = MidiClass::new(&mut builder, 1, 1, 64);
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
            let mot1_measured = adc2.read(&mut p.PC0) >> 1;
            let mot2_measured = adc2.read(&mut p.PC1) >> 1;
            let mot3_measured = adc2.read(&mut p.PC2_C) >> 1;
            let mot4_measured = adc2.read(&mut p.PC3_C) >> 1;
            let mot5_measured = adc2.read(&mut p.PC4) >> 1;
            // info!("measured: {}, {}, {}, {}, {}", mot1_measured, mot2_measured, mot3_measured, mot4_measured, mot5_measured);
            
            SLIDER_1_MEAS.store(mot1_measured, Ordering::Relaxed);
            if SLIDER_1_TOUCH.load(Ordering::Relaxed) { SLIDER_1_DES.store(mot1_measured, Ordering::Relaxed); }
            SLIDER_2_MEAS.store(mot2_measured, Ordering::Relaxed);
            if SLIDER_2_TOUCH.load(Ordering::Relaxed) { SLIDER_2_DES.store(mot2_measured, Ordering::Relaxed); }
            SLIDER_3_MEAS.store(mot3_measured, Ordering::Relaxed);
            if SLIDER_3_TOUCH.load(Ordering::Relaxed) { SLIDER_3_DES.store(mot3_measured, Ordering::Relaxed); }
            SLIDER_4_MEAS.store(mot4_measured, Ordering::Relaxed);
            if SLIDER_4_TOUCH.load(Ordering::Relaxed) { SLIDER_4_DES.store(mot4_measured, Ordering::Relaxed); }
            SLIDER_5_MEAS.store(mot5_measured, Ordering::Relaxed);
            if SLIDER_5_TOUCH.load(Ordering::Relaxed) { SLIDER_5_DES.store(mot5_measured, Ordering::Relaxed); }

            pid1.setpoint(SLIDER_1_DES.load(Ordering::Relaxed) as f32);
            let output1 = pid1.next_control_output(mot1_measured as f32);
            let mut correction1 = output1.output as i16;
            if correction1.abs() > 5_i16 { correction1 += 20_i16 * correction1.signum(); }
            set_motor_duty(&mut mot1_pwm, 'a', correction1);
            
            pid2.setpoint(SLIDER_2_DES.load(Ordering::Relaxed) as f32);
            let output2 = pid2.next_control_output(mot2_measured as f32);
            let mut correction2 = output2.output as i16;
            if correction2.abs() > 5_i16 { correction2 += 20_i16 * correction2.signum(); }
            set_motor_duty(&mut mot2_pwm, 'b', correction2);
            
            pid3.setpoint(SLIDER_3_DES.load(Ordering::Relaxed) as f32);
            let output3 = pid3.next_control_output(mot3_measured as f32);
            let mut correction3 = output3.output as i16;
            if correction3.abs() > 5_i16 { correction3 += 20_i16 * correction3.signum(); }
            set_motor_duty(&mut mot3_pwm, 'a', correction3);
            
            pid4.setpoint(SLIDER_4_DES.load(Ordering::Relaxed) as f32);
            let output4 = pid4.next_control_output(mot4_measured as f32);
            let mut correction4 = output4.output as i16;
            if correction4.abs() > 5_i16 { correction4 += 20_i16 * correction4.signum(); }
            set_motor_duty(&mut mot4_pwm, 'a', correction4);
            
            pid5.setpoint(SLIDER_5_DES.load(Ordering::Relaxed) as f32);
            let output5 = pid5.next_control_output(mot5_measured as f32);
            let mut correction5 = output5.output as i16;
            if correction5.abs() > 5_i16 { correction5 += 20_i16 * correction5.signum(); }
            set_motor_duty(&mut mot5_pwm, 'b', correction5);

            // info!("Setpoint: {}, {}, {}, {}, {}", pid1.setpoint, pid2.setpoint, pid3.setpoint, pid4.setpoint, pid5.setpoint);
            // info!("Correction: {}, {}, {}, {}, {}", correction1, correction2, correction3, correction4, correction5);

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
        // info!("channel: {}", data[2]);
        
        if data[0] == 0xb && data[1] == 0xb0 { // Cable 0, B for control change
            for i in (0..n).step_by(4) {
                // info!("data[{}]: {:x}, {:x}, {:x}, {:x}", i, data[i], data[i+1], data[i+2], data[i+3]);
                if data[i+2] == 16 { SLIDER_1_DES.store(data[i+3] as u16, Ordering::Relaxed); } // Decimal channel 16
                if data[i+2] == 17 { SLIDER_2_DES.store(data[i+3] as u16, Ordering::Relaxed); } // Decimal channel 17
                if data[i+2] == 18 { SLIDER_3_DES.store(data[i+3] as u16, Ordering::Relaxed); } // Decimal channel 18
                if data[i+2] == 19 { SLIDER_4_DES.store(data[i+3] as u16, Ordering::Relaxed); } // Decimal channel 19
                if data[i+2] == 80 { SLIDER_5_DES.store(data[i+3] as u16, Ordering::Relaxed); } // Decimal channel 80
            }
        }
    }
}

async fn midi_send<'d, T: Instance + 'd>(class: &mut midi::Sender<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut previous_position1: u8 = 0;
    let mut previous_position2: u8 = 0;
    let mut previous_position3: u8 = 0;
    let mut previous_position4: u8 = 0;
    let mut previous_position5: u8 = 0;
    
    loop {
        let slider_position1 = SLIDER_1_MEAS.load(Ordering::Relaxed).clamp(0, 127) as u8;
        if (previous_position1 != slider_position1) && (SLIDER_1_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 16, slider_position1]).await?; // First byte is cable 0, B for control change
            previous_position1 = slider_position1;
        }
        let slider_position2 = SLIDER_2_MEAS.load(Ordering::Relaxed).clamp(0, 127) as u8;
        if (previous_position2 != slider_position2) && (SLIDER_2_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 17, slider_position2]).await?; // First byte is cable 0, B for control change
            previous_position2 = slider_position2;
        }
        let slider_position3 = SLIDER_3_MEAS.load(Ordering::Relaxed).clamp(0, 127) as u8;
        if (previous_position3 != slider_position3) && (SLIDER_3_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 18, slider_position3]).await?; // First byte is cable 0, B for control change
            previous_position3 = slider_position3;
        }
        let slider_position4 = SLIDER_4_MEAS.load(Ordering::Relaxed).clamp(0, 127) as u8;
        if (previous_position4 != slider_position4) && (SLIDER_4_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 19, slider_position4]).await?; // First byte is cable 0, B for control change
            previous_position4 = slider_position4;
        }
        let slider_position5 = SLIDER_5_MEAS.load(Ordering::Relaxed).clamp(0, 127) as u8;
        if (previous_position5 != slider_position5) && (SLIDER_5_TOUCH.load(Ordering::Relaxed)) {
            class.write_packet(&[0x0b, 0xb0, 80, slider_position5]).await?; // First byte is cable 0, B for control change
            previous_position5 = slider_position5;
        }
        
        Timer::after_millis(20).await;
    }
}

#[embassy_executor::task]
async fn get_touch(i2c: I2c<'static, I2C1, DMA1_CH4, DMA1_CH5>, sleep1: Output<'static, AnyPin>, sleep2: Output<'static, AnyPin>, sleep3: Output<'static, AnyPin>, sleep4: Output<'static, AnyPin>, sleep5: Output<'static, AnyPin>) {
    let mut data = [0u8; 1];
    let mut i2c_task = i2c;
    let mut sleep1_obj = sleep1;
    let mut sleep2_obj = sleep2;
    let mut sleep3_obj = sleep3;
    let mut sleep4_obj = sleep4;
    let mut sleep5_obj = sleep5;
    loop {
        match i2c_task.write_read(ADDRESS, &[KEY_STATUS], &mut data).await {
            // Ok(()) => info!("Key Status: {:#010b}", data[0]),
            Ok(()) => (),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        let mot1_sense = ((1 << 4) & data[0]) > 0;
        SLIDER_1_TOUCH.store(mot1_sense, Ordering::Relaxed);
        let mot2_sense = ((1 << 3) & data[0]) > 0;
        SLIDER_2_TOUCH.store(mot2_sense, Ordering::Relaxed);
        let mot3_sense = ((1 << 2) & data[0]) > 0;
        SLIDER_3_TOUCH.store(mot3_sense, Ordering::Relaxed);
        let mot4_sense = ((1 << 1) & data[0]) > 0;
        SLIDER_4_TOUCH.store(mot4_sense, Ordering::Relaxed);
        let mot5_sense = ((1 << 0) & data[0]) > 0;
        SLIDER_5_TOUCH.store(mot5_sense, Ordering::Relaxed);
        // info!("Touch: {}, {}, {}, {}, {}", mot1_sense, mot2_sense, mot3_sense, mot4_sense, mot5_sense);

        if mot1_sense { sleep1_obj.set_low(); } else { sleep1_obj.set_high(); }
        if mot2_sense { sleep2_obj.set_low(); } else { sleep2_obj.set_high(); }
        if mot3_sense { sleep3_obj.set_low(); } else { sleep3_obj.set_high(); }
        if mot4_sense { sleep4_obj.set_low(); } else { sleep4_obj.set_high(); }
        if mot5_sense { sleep5_obj.set_low(); } else { sleep5_obj.set_high(); }
        
        Timer::after_millis(25).await;
    }
}

/// Enable a motor by enabling the PWM channels
fn enable_motor<T: CaptureCompare16bitInstance>(pwm: &mut SimplePwm<T>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.enable(channels[0]);
    pwm.enable(channels[1]);
}

/*
/// Disable a motor by disabling the PWM channels
fn disable_motor<T: CaptureCompare16bitInstance>(pwm: &mut SimplePwm<T>, motor: char) {
    let channels = match motor {
        'a' => [Channel::Ch1, Channel::Ch2],
        'b' => [Channel::Ch3, Channel::Ch4],
        _ => [Channel::Ch1, Channel::Ch2], // TODO: Return an error instead of assuming motor a by default
    };

    pwm.disable(channels[0]);
    pwm.disable(channels[1]);
    set_motor_duty(pwm, motor, 0);
}
*/

/// Set the duty cycle of a motor
fn set_motor_duty<T: CaptureCompare16bitInstance>(pwm: &mut SimplePwm<T>, motor: char, duty: i16) {
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