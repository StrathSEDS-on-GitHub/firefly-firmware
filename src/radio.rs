use core::cell::Cell;
use core::cell::RefCell;

use core::fmt::Write;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use crate::logger::get_serial;
use crate::mission;
use crate::mission::Role;
use crate::Dio1PinRefMut;
use crate::NEOPIXEL;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_semihosting::hprintln;
use dummy_pin::DummyPin;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::write;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use heapless::Deque;
use heapless::Vec;
use serde::Deserialize;
use serde::Serialize;
use smart_leds::SmartLedsWrite;
use stm32f4xx_hal::gpio::Output;
use stm32f4xx_hal::gpio::Pin;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::pac::SPI1;
use stm32f4xx_hal::pac::TIM5;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::spi::Spi;
use stm32f4xx_hal::timer::CounterUs;
use stm32f4xx_hal::timer::Event;
use stm32f4xx_hal::timer::SysDelay;
use sx126x::op::IrqMask;
use sx126x::op::IrqMaskBit;
use sx126x::op::RxTxTimeout;

const MAX_PAYLOAD_SIZE: usize = 255;
static TRANSMISSION_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

#[interrupt]
fn EXTI4() {
    let dio1_pin = unsafe { crate::DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        dio1_pin.clear_interrupt_pending_bit();
        let should_recv = cortex_m::interrupt::free(|cs| {
            let mut radio = RADIO.borrow(cs).borrow_mut();
            if let Some(radio) = radio.as_mut() {
                let irq_status = radio
                    .radio
                    .get_irq_status(&mut radio.spi, &mut radio.delay)
                    .unwrap();
                radio
                    .radio
                    .clear_irq_status(
                        &mut radio.spi,
                        &mut radio.delay,
                        IrqMask::none()
                            .combine(IrqMaskBit::Timeout)
                            .combine(IrqMaskBit::RxDone)
                            .combine(IrqMaskBit::TxDone),
                    )
                    .unwrap();
                radio.radio.wait_on_busy(&mut radio.delay).unwrap();
                if let RadioState::Tx(transmitter) = RADIO_STATE.borrow(cs).get() {
                    return transmitter != mission::role()
                        && (transmitter == Role::Ground || mission::role() == Role::Ground);
                }
            }

            false
        });

        if should_recv {
            if mission::role() == Role::Ground {
                cortex_m::interrupt::free(|cs| {
                    writeln!(
                        get_serial(),
                        "Receiving message, {:?}",
                        RADIO_STATE.borrow(cs).get()
                    )
                    .unwrap();
                });
            }
            receive_message();
        }
        TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
        set_radio();
    }
}

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub enum Message {
    GpsBroadCast {
        counter: u32,
        latitudes: [f32; 8],
        longitudes: [f32; 8],
        altitudes: [f32; 8],
    },
    Arm,
    Disarm,
}

static RADIO: Mutex<
    RefCell<
        Option<
            Radio<
                Spi<SPI1, false>,
                Pin<'A', 4, Output>,
                Pin<'B', 0, Output>,
                Pin<'C', 5>,
                DummyPin<dummy_pin::level::High>,
                Dio1PinRefMut<'_>,
                SysDelay,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));
pub struct Radio<TSPI, TNSS: OutputPin, TNRST, TBUSY, TANT, TDIO1, DELAY>
where
    DELAY: DelayMs<u32> + DelayUs<u32>,
{
    radio: sx126x::SX126x<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1>,
    delay: DELAY,
    spi: TSPI,
}
impl
    Radio<
        Spi<SPI1, false>,
        Pin<'A', 4, Output>,
        Pin<'B', 0, Output>,
        Pin<'C', 5>,
        DummyPin<dummy_pin::level::High>,
        Dio1PinRefMut<'static>,
        SysDelay,
    >
{
    pub fn init(
        radio: sx126x::SX126x<
            Spi<SPI1, false>,
            Pin<'A', 4, Output>,
            Pin<'B', 0, Output>,
            Pin<'C', 5>,
            DummyPin<dummy_pin::level::High>,
            Dio1PinRefMut<'static>,
        >,
        spi: Spi<SPI1, false>,
        delay: SysDelay,
    ) {
        cortex_m::interrupt::free(|cs| {
            RADIO.borrow(cs).replace(Some(Self { radio, delay, spi }));
        });
    }
}

pub static QUEUED_PACKETS: Mutex<RefCell<Deque<Message, 32>>> =
    Mutex::new(RefCell::new(Deque::new()));

pub fn queue_packet(msg: Message) {
    cortex_m::interrupt::free(|cs| {
        let mut queued_packets = QUEUED_PACKETS.borrow(cs).borrow_mut();
        if let Err(_) = queued_packets.push_back(msg) {
            queued_packets.clear();
            let _ = queued_packets.push_back(msg);
        }
    });
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum RadioState {
    Tx(Role),
    // Buffer between transmit windows. The Role is the next role to transmit.
    Buffer(Role),
}

pub(crate) static TIMER: Mutex<RefCell<Option<CounterUs<TIM5>>>> = Mutex::new(RefCell::new(None));
static RADIO_STATE: Mutex<Cell<RadioState>> =
    Mutex::new(Cell::new(RadioState::Buffer(Role::Avionics)));

pub fn update_timer(secs: f32) {
    let time = secs % 2.0;

    let (state, remaining_time) =
        get_current_state_and_remaining_time_from_offset((time * 1000.0) as u32);

    cortex_m::interrupt::free(|cs| {
        RADIO_STATE.borrow(cs).set(state);
        let mut timer_ref = TIMER.borrow(cs).borrow_mut();
        let timer = timer_ref.as_mut().unwrap();

        // Avoid setting the timer if it's already set to the (approx) correct value.
        // This prevents bouncing at the edge of the transmit window.
        if remaining_time > 30 {
            timer.start(remaining_time.millis());
            timer.listen(Event::Update);
            set_radio();
        }
        unsafe {
            NVIC::unmask(stm32f4xx_hal::interrupt::TIM5);
            NVIC::unpend(stm32f4xx_hal::interrupt::TIM5);
        }
    });
}

// Set to true when we have launched and before the config has been switched.
static SWITCHED_CONFIGS: AtomicBool = AtomicBool::new(false);

// Prior to launch, we have a slot for the ground station to transmit
// so it can send arm/disarm commands to the avionics.
const PRE_LAUNCH_TDM_CONFIG: [(RadioState, u32); 6] = [
    (RadioState::Buffer(Role::Ground), 100),
    (RadioState::Tx(Role::Ground), 600),
    (RadioState::Buffer(Role::Avionics), 50),
    (RadioState::Tx(Role::Avionics), 600),
    (RadioState::Buffer(Role::Cansat), 50),
    (RadioState::Tx(Role::Cansat), 600),
];

// After launch, we don't need to transmit to the ground station anymore,
// so we can use all the bandwidth for the avionics and cansat.
const POST_LAUNCH_TDM_CONFIG: [(RadioState, u32); 4] = [
    (RadioState::Buffer(Role::Avionics), 50),
    (RadioState::Tx(Role::Avionics), 950),
    (RadioState::Buffer(Role::Cansat), 50),
    (RadioState::Tx(Role::Cansat), 950),
];

fn get_current_state_and_remaining_time_from_offset(offset: u32) -> (RadioState, u32) {
    let mut time = offset;

    let config: &[(RadioState, u32)] = if SWITCHED_CONFIGS.load(Ordering::Relaxed) {
        &POST_LAUNCH_TDM_CONFIG
    } else {
        &PRE_LAUNCH_TDM_CONFIG
    };

    for (state, duration) in config.iter() {
        if *duration > time {
            return (*state, *duration - time);
        }
        time -= duration;
    }
    // Offset too big?
    return PRE_LAUNCH_TDM_CONFIG[0];
}

// Determine the next state and how long it lasts for
fn get_next_state_and_time(current: RadioState) -> (RadioState, u32) {
    if SWITCHED_CONFIGS.swap(false, Ordering::Relaxed) {
        // We just switched configs, so we need to do some maths to figure out
        // where we are in the new config.

        let mut time = 0;
        for (state, duration) in PRE_LAUNCH_TDM_CONFIG.iter() {
            time += duration;
            if *state == current {
                break;
            }
        }

        return get_current_state_and_remaining_time_from_offset(time);
    }

    let launched = mission::is_launched();
    let config: &[(RadioState, u32)] = if launched {
        &POST_LAUNCH_TDM_CONFIG
    } else {
        &PRE_LAUNCH_TDM_CONFIG
    };
    for (i, (state, _)) in config.iter().enumerate() {
        if *state == current {
            return config[(i + 1) % config.len()];
        }
    }
    // This _shouldn't_ ever happen, but let's not panic if it does
    // The worst that can happen is we have a broken config
    // until the GPS gives us the correct time.
    config[0]
}

/// TDM (Time Division Multiplexing) timer
/// We use this timer to schedule the radio transmissions
#[interrupt]
fn TIM5() {
    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = TIMER.borrow(cs).borrow_mut();
        let timer = timer_ref.as_mut().unwrap();
        timer.clear_interrupt(Event::Update);

        let radio_state = RADIO_STATE.borrow(cs).get();
        let (state, next_time) = get_next_state_and_time(radio_state);
        timer.start(next_time.millis());
        timer.listen(Event::Update);
        RADIO_STATE.borrow(cs).set(state);

        set_radio();
    });
}

pub static RECEIVED_MESSAGE_QUEUE: Mutex<RefCell<Deque<Message, 64>>> =
    Mutex::new(RefCell::new(Deque::new()));

fn receive_message() {
    cortex_m::interrupt::free(|cs| {
        let mut radio_ref = RADIO.borrow(cs).borrow_mut();
        let radio = radio_ref.as_mut().unwrap();
        let mut buf = [0u8; MAX_PAYLOAD_SIZE];
        let rx_buf_status = radio
            .radio
            .get_rx_buffer_status(&mut radio.spi, &mut radio.delay)
            .unwrap();
        let size = rx_buf_status.payload_length_rx() as usize;
        let state = radio.radio.get_status(&mut radio.spi, &mut radio.delay).unwrap();
        hprintln!(
            "Radio state: {:?}. Start: {}, size: {}, state: {:?}",
            RADIO_STATE.borrow(cs).get(),
            rx_buf_status.rx_start_buffer_pointer(),
            size,
            state
        );
        radio
            .radio
            .read_buffer(
                &mut radio.spi,
                &mut radio.delay,
                rx_buf_status.rx_start_buffer_pointer(),
                &mut buf[0usize..size],
            )
            .unwrap();

        let message = postcard::from_bytes::<Message>(&buf[..size]).ok();
        hprintln!("Received message: {:?}", message);
        if let Some(message) = message {
            if let Message::Disarm = message {
                // Priority message, executed in interrupt handler
                // Otherwise just add it to the queue
                mission::disarm();
            }
            let mut msg_queue = RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut();
            msg_queue
                .push_back(message)
                .or_else(|_| {
                    msg_queue.pop_front();
                    msg_queue.push_back(message)
                })
                .ok();
        }
    });
}

/// Sets the radio in the correct state.
/// Called from timer interrupt and when the radio interrupt fires.
/// The radio interrupt fires when the radio is done transmitting a packet.
fn set_radio() {
    let state = cortex_m::interrupt::free(|cs| RADIO_STATE.borrow(cs).get());

    match state {
        RadioState::Tx(role) if role == mission::role() => {
            if TRANSMISSION_IN_PROGRESS.load(Ordering::Relaxed) {
                // writeln!(get_serial(), "Transmission in progress, skipping").unwrap();
                return;
            }
            // Our turn to transmit
            cortex_m::interrupt::free(|cs| {
                let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                let radio = radio_ref.as_mut().unwrap();
                let mut queued_packets = QUEUED_PACKETS.borrow(cs).borrow_mut();
                if let Some(msg) = queued_packets.pop_front() {
                    let mut buf = [0u8; MAX_PAYLOAD_SIZE];
                    let bytes = postcard::to_slice(&msg, &mut buf).unwrap();
                    radio
                        .radio
                        .write_buffer(&mut radio.spi, &mut radio.delay, 0, bytes)
                        .unwrap();
                    radio
                        .radio
                        .set_tx(&mut radio.spi, &mut radio.delay, RxTxTimeout::from_ms(500))
                        .unwrap();
                    radio.radio.wait_on_busy(&mut radio.delay).unwrap();

                    TRANSMISSION_IN_PROGRESS.store(true, Ordering::Relaxed);
                } else {
                    // No packets to transmit, stay idle
                }
            });
        }
        RadioState::Tx(other) | RadioState::Buffer(other) => {
            // Someone else is transmitting
            if mission::role() == Role::Ground || other == Role::Ground {
                // Either we're the ground station and should be listening,
                // or the other station is the ground station and we should be listening.
                cortex_m::interrupt::free(|cs| {
                    let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                    let radio = radio_ref.as_mut().unwrap();
                    radio
                        .radio
                        .set_rx(&mut radio.spi, &mut radio.delay, RxTxTimeout::from_ms(5000))
                        .unwrap();
                    TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
                });
            } else {
                // Not our turn to transmit
                // Tell radio to shut up
                cortex_m::interrupt::free(|cs| {
                    let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                    let radio = radio_ref.as_mut().unwrap();
                    radio
                        .radio
                        .set_standby(
                            &mut radio.spi,
                            &mut radio.delay,
                            sx126x::op::StandbyConfig::StbyRc,
                        )
                        .unwrap();
                    TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
                });
            }
        }
    }
    cortex_m::interrupt::free(|cs| {
        let mut neo_ref = NEOPIXEL.borrow(cs).borrow_mut();

        let r = match state {
            RadioState::Buffer(_) | RadioState::Tx(Role::Ground) => 55,
            _ => 0,
        };

        let g = match state {
            RadioState::Tx(Role::Avionics) | RadioState::Tx(Role::Ground) => 55,
            _ => 0,
        };

        let b = match state {
            RadioState::Tx(Role::Cansat) => 55,
            _ => 0,
        };
        neo_ref.as_mut().unwrap().write([[r, g, b]].into_iter());
    });
}
