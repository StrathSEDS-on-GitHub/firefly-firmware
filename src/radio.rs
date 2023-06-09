use core::cell::Cell;
use core::cell::RefCell;

use core::fmt::Write;

use crate::logger::get_serial;
use crate::mission;
use crate::mission::Role;
use crate::Dio1PinRefMut;
use crate::NEOPIXEL;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use dummy_pin::DummyPin;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
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
use sx126x::op::RxTxTimeout;

const MAX_PAYLOAD_SIZE: usize = 255;

#[interrupt]
fn EXTI4() {
    let dio1_pin = unsafe { crate::DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        set_radio();
        dio1_pin.clear_interrupt_pending_bit();

        cortex_m::interrupt::free(|cs| {
            let mut radio = RADIO.borrow(cs).borrow_mut();
            if let Some(radio) = radio.as_mut() {
                radio.radio.clear_irq_status(&mut radio.spi, &mut radio.delay, IrqMask::all()).unwrap();
            }
        });
    }
}

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub enum Message {
    GpsBroadCast {
        latitudes: [f32; 8],
        longitudes: [f32; 8],
        altitudes: [f32; 8],
    },
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

pub static QUEUED_PACKETS: Mutex<RefCell<Vec<Message, 32>>> = Mutex::new(RefCell::new(Vec::new()));

pub fn queue_packet(msg: Message) {
    cortex_m::interrupt::free(|cs| {
        let mut queued_packets = QUEUED_PACKETS.borrow(cs).borrow_mut();
        if let Err(_) = queued_packets.push(msg) {
            queued_packets.clear();
            let _ = queued_packets.push(msg);
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
    // 0.0s                                                           0.5s
    //  | <- 0.01s -> | <------------ 0.48s -----------> | <- 0.01s -> |
    //  |     Idle    |            Radio1 Tx             |    Idle     |
    // 0.5s                                                            1s
    //  | <- 0.01s -> | <------------ 0.48s -----------> | <- 0.01s -> |
    //  |     Idle    |            Radio2 Tx             |    Idle     |

    let time = secs % 1.0;

    let (next_event, state) = match (time * 1000.0) as u32 % 1000 {
        990..=999 | 0..=009 => (0.005, RadioState::Buffer(Role::Avionics)),
        010..=489 => (0.495, RadioState::Tx(Role::Avionics)),
        490..=509 => (0.505, RadioState::Buffer(Role::Cansat)),
        510..=989 => (0.995, RadioState::Tx(Role::Cansat)),
        _ => unreachable!(),
    };

    cortex_m::interrupt::free(|cs| {
        RADIO_STATE.borrow(cs).set(state);
        let mut timer_ref = TIMER.borrow(cs).borrow_mut();
        let timer = timer_ref.as_mut().unwrap();
        timer.start((((next_event - time) * 1_000_000.0) as u32).micros());
        timer.listen(Event::Update);
        unsafe {
            NVIC::unmask(stm32f4xx_hal::interrupt::TIM5);
            NVIC::unpend(stm32f4xx_hal::interrupt::TIM5);
        }
    });

    set_radio();
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
        let (next_time, state) = match radio_state {
            RadioState::Buffer(Role::Avionics) => (20, RadioState::Tx(Role::Avionics)),
            RadioState::Tx(Role::Avionics) => (480, RadioState::Buffer(Role::Cansat)),
            RadioState::Buffer(Role::Cansat) => (20, RadioState::Tx(Role::Cansat)),
            RadioState::Tx(Role::Cansat) => (480, RadioState::Buffer(Role::Avionics)),
            // The ground station doesn't transmit
            _ => return,
        };

        let mut neo_ref = NEOPIXEL.borrow(cs).borrow_mut();

        let r = match radio_state {
            RadioState::Buffer(_) => 255,
            _ => 0,
        };

        let g = match radio_state {
            RadioState::Tx(Role::Avionics) => 255,
            _ => 0,
        };

        let b = match radio_state {
            RadioState::Tx(Role::Cansat) => 255,
            _ => 0,
        };

        neo_ref.as_mut().unwrap().write([[r, g, b]].into_iter());

        timer.start(next_time.millis());
        timer.listen(Event::Update);
        RADIO_STATE.borrow(cs).set(state);
    });
}

pub fn setup_ground_radio() {
    cortex_m::interrupt::free(|cs| {
        let mut radio_ref = RADIO.borrow(cs).borrow_mut();
        let radio = radio_ref.as_mut().unwrap();
        radio.radio.set_rx(&mut radio.spi, &mut radio.delay, RxTxTimeout::continuous_rx()).unwrap();
    });
}

/// Sets the radio in the correct state.
/// Called from timer interrupt and when the radio interrupt fires.
/// The radio interrupt fires when the radio is done transmitting a packet.
fn set_radio() {
    let state = cortex_m::interrupt::free(|cs| RADIO_STATE.borrow(cs).get());
    if matches!(mission::role(), Role::Ground) {
        // We received a packet.
        cortex_m::interrupt::free(|cs| {
            let mut radio_ref = RADIO.borrow(cs).borrow_mut();
            let radio = radio_ref.as_mut().unwrap();
            let mut buf = [0u8; MAX_PAYLOAD_SIZE];
            let rx_buf_status = radio
                .radio
                .get_rx_buffer_status(&mut radio.spi, &mut radio.delay)
                .unwrap();
            radio.radio.read_buffer(
                &mut radio.spi,
                &mut radio.delay,
                rx_buf_status.rx_start_buffer_pointer(),
                &mut buf[0usize..rx_buf_status.payload_length_rx() as usize],
            ).unwrap();

            if let Ok(msg) = postcard::from_bytes::<Message>(&buf) {
                writeln!(get_serial(), "Received: {:?}", msg).unwrap();
            }

        });

        return;
    }

    match state {
        RadioState::Tx(role) if role == mission::role() => {
            // Our turn to transmit
            cortex_m::interrupt::free(|cs| {
                let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                let radio = radio_ref.as_mut().unwrap();
                let mut queued_packets = QUEUED_PACKETS.borrow(cs).borrow_mut();
                if let Some(msg) = queued_packets.pop() {
                    let mut buf = [0u8; MAX_PAYLOAD_SIZE];
                    let bytes = postcard::to_slice(&msg, &mut buf).unwrap();
                    radio
                        .radio
                        .write_buffer(&mut radio.spi, &mut radio.delay, 0, bytes)
                        .unwrap();
                    radio
                        .radio
                        .set_tx(&mut radio.spi, &mut radio.delay, RxTxTimeout::from_ms(300))
                        .unwrap();
                } else {
                    // No packets to transmit, go back to idle
                    radio
                        .radio
                        .set_standby(
                            &mut radio.spi,
                            &mut radio.delay,
                            sx126x::op::StandbyConfig::StbyRc,
                        )
                        .unwrap();
                }
            });
        }
        _ => {
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
        }
    }
}
