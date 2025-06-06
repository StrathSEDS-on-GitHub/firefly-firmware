use core::cell::Cell;
use core::cell::RefCell;

use core::ops::DerefMut;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU16;
use core::sync::atomic::Ordering;

use crate::mission;
use crate::mission::role;
use crate::neopixel;
use crate::Dio1PinRefMut;
use cortex_m::interrupt::Mutex;
use dummy_pin::DummyPin;
use embedded_hal::spi::SpiDevice;
use heapless::Deque;
use stm32f4xx_hal::gpio::Output;
use stm32f4xx_hal::gpio::Pin;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::pac::SPI1;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::rtc;
use stm32f4xx_hal::spi::Spi;
use stm32f4xx_hal::timer::SysDelay;
use storage_types::logs;
use storage_types::logs::Message;
use storage_types::Role;
use sx126x::op::IrqMask;
use sx126x::op::IrqMaskBit;
use sx126x::op::PacketStatus;
use sx126x::op::RxTxTimeout;

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
                        IrqMask::none()
                            .combine(IrqMaskBit::Timeout)
                            .combine(IrqMaskBit::RxDone)
                            .combine(IrqMaskBit::TxDone),
                    )
                    .unwrap();
                radio.radio.wait_on_busy().unwrap();
                if let RadioState::Tx(transmitter) = RADIO_STATE.borrow(cs).get() {
                    return transmitter != mission::role()
                        && (transmitter == Role::GroundMain || mission::role() == Role::GroundMain || mission::role() == Role::GroundBackup);
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

pub const BNO_BROADCAST_BUF_LEN    : usize = 8;
pub const BNO_BROADCAST_DECIMATION : usize = 4;

pub fn next_counter() -> u16 {
    COUNTER.fetch_add(1, Ordering::Relaxed)
}

static RADIO: Mutex<
    RefCell<
        Option<
            Radio<
                Spi1Device,
                Pin<'B', 0, Output>,
                Pin<'C', 5>,
                DummyPin<dummy_pin::level::High>,
                Dio1PinRefMut<'_>,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

pub type Spi1Device =
    embedded_hal_bus::spi::AtomicDevice<'static, Spi<SPI1>, Pin<'A', 4, Output>, SysDelay>;
pub type RadioDevice<'a> = Radio<
    Spi1Device,
    Pin<'B', 0, Output>,
    Pin<'C', 5>,
    DummyPin<dummy_pin::level::High>,
    Dio1PinRefMut<'a>,
>;
pub struct Radio<TSPI: SpiDevice, TNRST, TBUSY, TANT, TDIO1> {
    radio: sx126x::SX126x<TSPI, TNRST, TBUSY, TANT, TDIO1>,
}
impl RadioDevice<'static> {
    pub fn init(
        radio: sx126x::SX126x<
            Spi1Device,
            Pin<'B', 0, Output>,
            Pin<'C', 5>,
            DummyPin<dummy_pin::level::High>,
            Dio1PinRefMut<'static>,
        >,
    ) {
        cortex_m::interrupt::free(|cs| {
            RADIO.borrow(cs).replace(Some(Self { radio }));
        });
    }
}

static QUEUED_PACKETS: Mutex<RefCell<Deque<Message<logs::RadioCtxt>, 32>>> = Mutex::new(RefCell::new(Deque::new()));

pub fn queue_packet(msg: Message<logs::RadioCtxt>) {
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
pub(crate) enum RadioState {
    Tx(Role),
    // Buffer between transmit windows. The Role is the next role to transmit.
    Buffer(Role),
}

static RADIO_STATE: Mutex<Cell<RadioState>> =
    Mutex::new(Cell::new(RadioState::Buffer(Role::Avionics)));

// Prior to launch, we have a slot for the ground station to transmit
// so it can send arm/disarm commands to the avionics.
pub(crate) const TDM_CONFIG_MAIN: [(RadioState, u32); 6] = [
    (RadioState::Buffer(Role::GroundMain), 50),
    (RadioState::Tx(Role::GroundMain), 250),
    (RadioState::Buffer(Role::Avionics), 50),
    (RadioState::Tx(Role::Avionics), 800),
    (RadioState::Buffer(Role::Cansat), 50),
    (RadioState::Tx(Role::Cansat), 800),
];

const fn total_tdm_duration() -> u32 {
    let mut total = 0;
    let mut i = 0;
    while i < TDM_CONFIG_MAIN.len() {
        total += TDM_CONFIG_MAIN[i].1;
        i += 1;
    }
    total
}

const TDM_CONFIG_BACKUP: [(RadioState, u32); 1] = [
    (RadioState::Tx(Role::CansatBackup), total_tdm_duration()),
];

pub static RECEIVED_MESSAGE_QUEUE: Mutex<RefCell<Deque<Message<logs::RadioCtxt>, 64>>> =
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

            let (_, _, s, millis) = time.as_hms_milli();
            let t_secs = (s as u32 * 1000 + millis as u32) % total_tdm_duration();

            let mut offset = 0;
            let tdm_config: &[_] = match role() {
                Role::GroundBackup | Role::CansatBackup => &TDM_CONFIG_BACKUP,
                _ => &TDM_CONFIG_MAIN,
                
            };
            for (state, duration) in tdm_config.iter() {
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
        let rx_buf_status = radio.radio.get_rx_buffer_status().unwrap();
        let size = rx_buf_status.payload_length_rx() as usize;
        radio
            .radio
            .read_buffer(
                
                rx_buf_status.rx_start_buffer_pointer(),
                &mut buf[0usize..size],
            )
            .unwrap();

        let message = postcard::from_bytes::<Message<logs::RadioCtxt>>(&buf[..size]).ok();
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
        RadioState::Buffer(_) | RadioState::Tx(Role::GroundMain) => 55,
        _ => 0,
    };

    let g = match state {
        RadioState::Tx(Role::Avionics) | RadioState::Tx(Role::GroundMain) => 55,
        _ => 0,
    };

    let b = match state {
        RadioState::Tx(Role::Cansat) => 55,
        _ => 0,
    };

    neopixel::update_pixel(2, [r, g, b]);

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
                    radio.radio.write_buffer( 0, bytes).unwrap();
                    radio.radio.wait_on_busy().unwrap();
                    radio
                        .radio
                        .set_tx( RxTxTimeout::from_ms(5000))
                        .unwrap();

                    radio.radio.wait_on_busy().unwrap();

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
            if mission::role() == Role::GroundMain || other == Role::GroundMain || mission::role() == Role::GroundBackup {
                if LISTEN_IN_PROGRESS.load(Ordering::Relaxed) {
                    return;
                }
                // Either we're the ground station and should be listening,
                // or the other station is the ground station and we should be listening.
                cortex_m::interrupt::free(|cs| {
                    let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                    let radio = radio_ref.as_mut().unwrap();
                    radio.radio.wait_on_busy().unwrap();
                    radio
                        .radio
                        .set_rx( RxTxTimeout::from_ms(5000))
                        .unwrap();
                    radio.radio.wait_on_busy().unwrap();
                    TRANSMISSION_IN_PROGRESS.store(false, Ordering::Relaxed);
                    LISTEN_IN_PROGRESS.store(true, Ordering::Relaxed);
                });
            } else {
                // Not our turn to transmit
                // Tell radio to shut up
                cortex_m::interrupt::free(|cs| {
                    let mut radio_ref = RADIO.borrow(cs).borrow_mut();
                    let radio = radio_ref.as_mut().unwrap();
                    radio.radio.wait_on_busy().unwrap();
                    radio
                        .radio
                        .set_standby( sx126x::op::StandbyConfig::StbyRc)
                        .unwrap();
                    radio.radio.wait_on_busy().unwrap();
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
        radio.radio.wait_on_busy().unwrap();
        radio.radio.get_packet_status().unwrap()
    })
}
