use fugit::RateExtU32;
use stm32f4xx_hal::{
    gpio::{gpioc, gpioe, Output, PushPull},
    interrupt,
    pac::{TIM1, TIM2, TIM5},
    timer::{CounterHz, PwmChannel, PwmHzManager},
    ClearFlags
};

static mut TARGET_POSE_DEG: [f32; 3] = [0.0; 3];
static mut CURRENT_POSE_DEG: [f32; 3] = [0.0; 3];
static mut LAST_TIME_MS: u32 = 0;

static mut SELF_ALT: f32 = 0.0;

pub fn update_self_alt(alt: f32) {
    unsafe {
        SELF_ALT = alt;
    }
}

pub fn update_target_alt(alt: f32) {
    let dist = 2.0; // assume 2m distance

    unsafe {
        use micromath::F32Ext;
        TARGET_POSE_DEG[2] = (alt - SELF_ALT).atan2(dist) * 180.0 / core::f32::consts::PI;
    }
}

pub static mut X_PWM_CHANNEL: Option<PwmChannel<TIM1, 1>> = None;
pub static mut Y_PWM_CHANNEL: Option<PwmChannel<TIM2, 2>> = None;
pub static mut X_PWM_MANAGER: Option<PwmHzManager<TIM1>> = None;
pub static mut Y_PWM_MANAGER: Option<PwmHzManager<TIM2>> = None;
pub static mut X_DIR_PIN: Option<gpioe::PE12<Output<PushPull>>> = None;
pub static mut Y_DIR_PIN: Option<gpioc::PC3<Output<PushPull>>> = None;

pub static mut TIMER5: Option<CounterHz<TIM5>> = None;

const MOTOR_TICK_PERIOD_S: f32 = 0.01;
const X_STEPS_PER_REV: f32 = 200.0 * 41.;
const Y_STEPS_PER_REV: f32 = 200.0 * 11.;
const X_DEG_PER_STEP: f32 = 360.0 / X_STEPS_PER_REV;
const Y_DEG_PER_STEP: f32 = 360.0 / Y_STEPS_PER_REV;
const MAX_FREQ_HZ: f32 = 1300.0;
const DEADBAND_DEG: f32 = 0.1;

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
            // writeln!(get_serial(), "Commanding {deg_moved}° at {freq_hz}").unwrap();
            if error_x > 0.0 {
                CURRENT_POSE_DEG[0] += deg_moved;
            } else {
                CURRENT_POSE_DEG[0] -= deg_moved;
            }
        }

        let y_pwm = Y_PWM_CHANNEL.as_mut().unwrap();
        let y_man = Y_PWM_MANAGER.as_mut().unwrap();
        let y_dir = Y_DIR_PIN.as_mut().unwrap();

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
            // writeln!(get_serial(), "Commanding {deg_moved}° at {freq_hz}").unwrap();
            if error_y > 0.0 {
                CURRENT_POSE_DEG[1] += deg_moved;
            } else {
                CURRENT_POSE_DEG[1] -= deg_moved;
            }
        }
    }
}
