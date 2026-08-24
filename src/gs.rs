use crate::logs::log_str;
use crate::usb_logger::get_serial;
use core::fmt::Write;
use cortex_m_semihosting::hprintln;
use fugit::RateExtU32;
use stm32f4xx_hal::{
    ClearFlags,
    gpio::{Output, PushPull, gpioc, gpioe},
    interrupt,
    pac::{TIM1, TIM2, TIM5},
    timer::{CounterHz, PwmChannel, PwmHzManager},
};

static mut TARGET_POSE_DEG: [f32; 3] = [0.0; 3];
static mut CURRENT_POSE_DEG: [f32; 3] = [0.0; 3];
static mut LAST_TIME_MS: u32 = 0;

static mut SELF_ALT: f32 = 0.0;
static mut SELF_LAT: f32 = 0.0; // 55.8616372;
static mut SELF_LONG: f32 = 0.0; //-4.2448985;
static mut TARGET_DIST: f32 = 0.0;

pub fn update_self_alt(alt: f32) {
    unsafe {
        SELF_ALT = alt;
    }
}

pub fn update_target_alt(alt: f32) {
    unsafe {
        use micromath::F32Ext;
        TARGET_POSE_DEG[1] =
            ((alt - SELF_ALT).atan2(TARGET_DIST) * 180.0 / core::f32::consts::PI + 360.) % 360.0;
        // use cortex_m_semihosting::hprintln;
        // hprintln!("deg phi: {}", TARGET_POSE_DEG[1]);
    }
}

pub fn update_self_lat_long(lat: f32, long: f32) {
    unsafe {
        SELF_LAT = lat;
        SELF_LONG = long;
    }
}

pub fn update_target_deg(around: Option<f32>, updown: Option<f32>) {
    unsafe {
        if let Some(around) = around {
            TARGET_POSE_DEG[0] = around;
        }

        if let Some(updown) = updown {
            TARGET_POSE_DEG[1] = updown;
        }
    }
}

pub fn update_target_lat_long(lat: f32, long: f32) {
    unsafe {
        TARGET_DIST = dist(SELF_LAT, SELF_LONG, lat, long);
        TARGET_POSE_DEG[0] = (bearing(SELF_LAT, SELF_LONG, lat, long).to_degrees() + 360.) % 360.;
        // writeln!(get_serial(), "angle {}", TARGET_POSE_DEG[0]);
        // use cortex_m_semihosting::hprintln;
        // hprintln!("deg theta: {}", TARGET_POSE_DEG[0]);
        // hprintln!("dist: {}", TARGET_DIST);
    }
}

fn haversine(x: f32) -> f32 {
    use micromath::F32Ext;
    return 0.5 * (1. - f32::cos(x));
}

// returns distance in meters
fn dist(lat1: f32, long1: f32, lat2: f32, long2: f32) -> f32 {
    use micromath::F32Ext;

    let lat1 = lat1.to_radians();
    let long1 = long1.to_radians();
    let lat2 = lat2.to_radians();
    let long2 = long2.to_radians();

    let d_lat = lat2 - lat1;
    let d_long = long2 - long1;

    let hsin_x = haversine(d_lat) + f32::cos(lat1) * f32::cos(lat2) * haversine(d_long);
    return 6371200. * 2. * f32::asin(f32::sqrt(hsin_x));
}

// returns the angle (in radians) from coordinate 1 to coordinate 2
fn bearing(lat1: f32, long1: f32, lat2: f32, long2: f32) -> f32 {
    use micromath::F32Ext;

    let lat1 = lat1.to_radians();
    let long1 = long1.to_radians();
    let lat2 = lat2.to_radians();
    let long2 = long2.to_radians();

    let d_lat = lat2 - lat1;
    let d_long = long2 - long1;

    let x = f32::sin(d_long) * f32::cos(lat2);
    let y = f32::cos(lat1) * f32::sin(lat2) - f32::sin(lat1) * f32::cos(lat2) * f32::cos(d_long);

    return f32::atan2(x, y);
}

pub static mut X_PWM_CHANNEL: Option<PwmChannel<TIM1, 1>> = None;
pub static mut Y_PWM_CHANNEL: Option<PwmChannel<TIM2, 2>> = None;
pub static mut X_PWM_MANAGER: Option<PwmHzManager<TIM1>> = None;
pub static mut Y_PWM_MANAGER: Option<PwmHzManager<TIM2>> = None;
pub static mut X_DIR_PIN: Option<gpioe::PE12<Output<PushPull>>> = None;
pub static mut Y_DIR_PIN: Option<gpioc::PC3<Output<PushPull>>> = None;

pub static mut TIMER5: Option<CounterHz<TIM5>> = None;

const MOTOR_TICK_PERIOD_S: f32 = 0.01;
const X_STEPS_PER_REV: f32 = 2080.; //200.0 * 41. * 0.25;
const Y_STEPS_PER_REV: f32 = 770. * 11.; //200.0 * 11.;
const X_DEG_PER_STEP: f32 = 360.0 / X_STEPS_PER_REV;
const Y_DEG_PER_STEP: f32 = 360.0 / Y_STEPS_PER_REV;
const MAX_FREQ_HZ: f32 = 1300.0;
const DEADBAND_DEG: f32 = 0.7;

#[interrupt]
fn TIM5() {
    motor_tick();
    unsafe {
        let timer = TIMER5.as_mut().unwrap();
        timer.clear_all_flags();
    }
}

// called in ISR every 10ms
fn motor_tick() {
    unsafe {
        let x_pwm = X_PWM_CHANNEL.as_mut().unwrap();
        let x_man = X_PWM_MANAGER.as_mut().unwrap();
        let x_dir = X_DIR_PIN.as_mut().unwrap();

        let error_x = TARGET_POSE_DEG[0] - CURRENT_POSE_DEG[0];
        let error_y = TARGET_POSE_DEG[1] - CURRENT_POSE_DEG[1];

        // use cortex_m_semihosting::hprintln;
        // hprintln!("{} {}", error_x, error_y);

        if error_x.abs() < DEADBAND_DEG {
            x_pwm.disable();
        } else {
            if error_x > 0.0 {
                x_dir.set_high();
            } else {
                x_dir.set_low();
            }

            let freq_hz = (error_x.abs() * 100.0).clamp(400., MAX_FREQ_HZ);
            x_man.set_period((freq_hz as u32).Hz());
            let max_duty = x_pwm.get_max_duty();
            x_pwm.set_duty(max_duty / 2);
            x_pwm.enable();

            let steps_this_tick = freq_hz * MOTOR_TICK_PERIOD_S;
            let deg_moved = steps_this_tick * X_DEG_PER_STEP;
            // writeln!(get_serial(), "Commanding {deg_moved}° x at {freq_hz}").unwrap();
            if error_x > 0.0 {
                CURRENT_POSE_DEG[0] += deg_moved;
            } else {
                CURRENT_POSE_DEG[0] -= deg_moved;
            }
        }

        // x_man.set_period(200u32.Hz());
        // x_pwm.set_duty(x_pwm.get_max_duty() / 2);
        // x_pwm.enable();

        let y_pwm = Y_PWM_CHANNEL.as_mut().unwrap();
        let y_man = Y_PWM_MANAGER.as_mut().unwrap();
        let y_dir = Y_DIR_PIN.as_mut().unwrap();

        // y_man.set_period(200u32.Hz());
        // y_pwm.set_duty(y_pwm.get_max_duty() / 2);
        // y_pwm.enable();
        if error_y.abs() < DEADBAND_DEG {
            y_pwm.disable();
        } else {
            if error_y > 0.0 {
                y_dir.set_high();
            } else {
                y_dir.set_low();
            }

            let freq_hz = (error_y.abs() * 100.0).clamp(400., MAX_FREQ_HZ);
            y_man.set_period((freq_hz as u32).Hz());
            let max_duty = y_pwm.get_max_duty();
            y_pwm.set_duty(max_duty / 2);
            y_pwm.enable();
            let steps_this_tick = freq_hz * MOTOR_TICK_PERIOD_S;
            let deg_moved = steps_this_tick * Y_DEG_PER_STEP;
            // writeln!(get_serial(), "Commanding {deg_moved}° y at {freq_hz}").unwrap();
            if error_y > 0.0 {
                CURRENT_POSE_DEG[1] += deg_moved;
            } else {
                CURRENT_POSE_DEG[1] -= deg_moved;
            }
        }
    }
}
