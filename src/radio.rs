use core::cell::Cell;
use core::cell::RefCell;

use core::ops::DerefMut;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU16;
use core::sync::atomic::Ordering;

use crate::mission;
use crate::mission::MissionStage;
use crate::mission::PyroPin;
use crate::mission::Role;
use crate::neopixel;
use crate::Dio1PinRefMut;
use cortex_m::interrupt::Mutex;
use dummy_pin::DummyPin;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
use heapless::Deque;
use heapless::String;
use serde::Deserialize;
use serde::Serialize;
use stm32f4xx_hal::gpio::Output;
use stm32f4xx_hal::gpio::Pin;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::pac::SPI1;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::rtc;
use stm32f4xx_hal::spi::Spi;
use stm32f4xx_hal::timer::SysDelay;
use sx126x::op::IrqMask;
use sx126x::op::IrqMaskBit;
use sx126x::op::PacketStatus;
use sx126x::op::RxTxTimeout;
use smart_leds::SmartLedsWrite;

const MAX_PAYLOAD_SIZE: usize = 255;
static TRANSMISSION_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static LISTEN_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static COUNTER: AtomicU16 = AtomicU16::new(0);

#[interrupt]
fn EXTI4() {
    let dio1_pin = unsafe { crate::DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        dio1_pin.clear_interrupt_pending_bit();
        let should_recv = cortex_m::interrupt::free(|cs| {
            let mut radio = RADIO.borrow(cs).borrow_mut();
            if let Some(radio) = radio.as_mut() {
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
            receive_message();
        }
        TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
        LISTEN_IN_PROGRESS.store(false, Ordering::Relaxed);
        set_radio();
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum Message {
    GpsBroadCast {
        counter: u16,
        stage: MissionStage,
        role: Role,
        time_of_first_packet: u32,
        latitudes: [f32; 10],
        longitudes: [f32; 10],
        altitudes: [f32; 10],
    },
    PressureTempBroadCast {
        counter: u16,
        stage: MissionStage,
        role: Role,
        time_of_first_packet: u32,
        pressures: [f32; 8],
        temperatures: [f32; 8],
    },
    CurrentSensorBroadcast {
        counter: u16,
        stage: MissionStage,
        role: Role,
        time_of_first_packet: u32,
        currents: [i16; 8],
        voltages: [u16; 8],
    },
    Arm(Role, u8),
    Disarm(Role, u8),
    TestPyro(Role, PyroPin, u32),
    SetStage(Role, MissionStage, u8)
}

pub fn next_counter() -> u16 {
    COUNTER.fetch_add(1, Ordering::Relaxed)
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

static QUEUED_PACKETS: Mutex<RefCell<Deque<Message, 32>>> =
    Mutex::new(RefCell::new(Deque::new()));

pub fn queue_packet(msg: Message) {
    cortex_m::interrupt::free(|cs| {
        let mut queued_packets = QUEUED_PACKETS.borrow(cs).borrow_mut();
        if let Err(msg) = queued_packets.push_back(msg) {
            queued_packets.clear();
            let _ = queued_packets.push_back(msg);
        }
        drop(queued_packets);
    });
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum RadioState {
    Tx(Role),
    // Buffer between transmit windows. The Role is the next role to transmit.
    Buffer(Role),
}

static RADIO_STATE: Mutex<Cell<RadioState>> =
    Mutex::new(Cell::new(RadioState::Buffer(Role::Avionics)));

// Set to true when we have launched and before the config has been switched.
static SWITCHED_CONFIGS: AtomicBool = AtomicBool::new(false);

// Prior to launch, we have a slot for the ground station to transmit
// so it can send arm/disarm commands to the avionics.
const TDM_CONFIG: [(RadioState, u32); 6] = [
    (RadioState::Buffer(Role::Ground), 100),
    (RadioState::Tx(Role::Ground), 600),
    (RadioState::Buffer(Role::Avionics), 50),
    (RadioState::Tx(Role::Avionics), 600),
    (RadioState::Buffer(Role::Cansat), 50),
    (RadioState::Tx(Role::Cansat), 600),
];

pub static RECEIVED_MESSAGE_QUEUE: Mutex<RefCell<Deque<Message, 64>>> =
    Mutex::new(RefCell::new(Deque::new()));


#[interrupt]
fn RTC_WKUP() {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = crate::RTC.borrow(cs).borrow_mut();
        if let Some(rtc) = rtc_ref.deref_mut() {
            if rtc.is_pending(rtc::Event::Wakeup) {
                rtc.clear_interrupt(rtc::Event::Wakeup);
            }

            let time = rtc.get_datetime();

            let (_,_,s,millis) = time.as_hms_milli();
            let t_secs = (s as u32 * 1000 + millis as u32) % 2000;

            let mut offset = 0;
            for (state, duration) in TDM_CONFIG.iter() {
                if t_secs < offset + *duration {
                    RADIO_STATE.borrow(cs).set(*state);
                    break;
                }
                offset += *duration;
            }

            set_radio();
        }
    });
}

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
        if let Some(message) = message {
            let mut msg_queue = RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut();
            msg_queue
                .push_back(message)
                .or_else(|message| {
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
        
    neopixel::update_pixel(2, [r,g,b]);

    match state {
        RadioState::Tx(role) if role == mission::role() => {
            if TRANSMISSION_IN_PROGRESS.load(Ordering::Relaxed) {
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
                        .set_tx(&mut radio.spi, &mut radio.delay, RxTxTimeout::from_ms(5000))
                        .unwrap();
                    radio.radio.wait_on_busy(&mut radio.delay).unwrap();

                    TRANSMISSION_IN_PROGRESS.store(true, Ordering::Relaxed);
                } else {
                    // No packets to transmit, stay idle
                }
                LISTEN_IN_PROGRESS.store(false, Ordering::Relaxed);
                drop(queued_packets);
            });
        }
        RadioState::Tx(other) | RadioState::Buffer(other) => {
            // Someone else is transmitting
            if mission::role() == Role::Ground || other == Role::Ground {
                if LISTEN_IN_PROGRESS.load(Ordering::Relaxed) {
                    return;
                }
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
                    LISTEN_IN_PROGRESS.store(true, Ordering::Relaxed);
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
                });
                TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
                LISTEN_IN_PROGRESS.store(false, Ordering::Relaxed);
            }
        }
    }
}

pub fn get_packet_status() -> PacketStatus {
    cortex_m::interrupt::free(|cs| {
        let mut radio_ref = RADIO.borrow(cs).borrow_mut();
        let radio = radio_ref.as_mut().unwrap();
        radio
            .radio
            .get_packet_status(&mut radio.spi, &mut radio.delay)
            .unwrap()
    })
}
