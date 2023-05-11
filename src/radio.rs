use core::sync::atomic;

use core::fmt::Write;
use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::blocking::spi::Write as SPIWrite;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use heapless::Deque;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::prelude::_fugit_DurationExtU32;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::timer::CounterMs;
use sx126x::op::ChipMode;
use sx126x::op::IrqMask;
use sx126x::op::RxTxTimeout;

use crate::futures::NbFuture;
use crate::logger::get_serial;
use crate::words::CHECK_WORDS;

static DIO1_RISEN: atomic::AtomicBool = atomic::AtomicBool::new(false);
#[interrupt]
fn EXTI4() {
    let dio1_pin = unsafe { crate::DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        DIO1_RISEN.store(true, atomic::Ordering::Relaxed);

        dio1_pin.clear_interrupt_pending_bit();
    }
}

#[repr(packed)]
struct TestPacket {
    check_word: [u8; 8],
    counter: u16,
}

impl From<[u8; 10]> for TestPacket {
    fn from(bytes: [u8; 10]) -> Self {
        let mut check_word = [0; 8];
        for (i, b) in check_word.iter_mut().enumerate() {
            *b = bytes[i];
        }
        let counter = u16::from_be_bytes([bytes[8], bytes[9]]);

        Self {
            check_word,
            counter,
        }
    }
}

impl From<TestPacket> for [u8; 10] {
    fn from(packet: TestPacket) -> Self {
        let mut bytes = [0; 10];
        for (i, b) in packet.check_word.iter().enumerate() {
            bytes[i] = *b;
        }
        bytes[8..10].copy_from_slice(&packet.counter.to_be_bytes());
        bytes
    }
}

pub struct Radio<TSPI, TNSS: OutputPin, TNRST, TBUSY, TANT, TDIO1, DELAY>
where
    DELAY: DelayMs<u32> + DelayUs<u32>,
{
    radio: sx126x::SX126x<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1>,
    delay: DELAY,
    spi: TSPI,
}
impl<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1, TSPIERR, TPINERR, DELAY>
    Radio<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1, DELAY>
where
    TPINERR: core::fmt::Debug,
    TSPIERR: core::fmt::Debug,
    TSPI: SPIWrite<u8, Error = TSPIERR> + Transfer<u8, Error = TSPIERR>,
    TNSS: OutputPin<Error = TPINERR>,
    TNRST: OutputPin<Error = TPINERR>,
    TBUSY: InputPin<Error = TPINERR>,
    TANT: OutputPin<Error = TPINERR>,
    TDIO1: InputPin<Error = TPINERR>,
    DELAY: DelayMs<u32> + DelayUs<u32>,
{
    pub fn new(
        radio: sx126x::SX126x<TSPI, TNSS, TNRST, TBUSY, TANT, TDIO1>,
        spi: TSPI,
        delay: DELAY,
    ) -> Self {
        Self { radio, delay, spi }
    }

    pub async fn radio_test_tx(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
    ) {
        let mut counter: u16 = 0;
        loop {
            let packet = TestPacket {
                check_word: CHECK_WORDS[counter as usize % CHECK_WORDS.len()].try_into().unwrap(),
                counter,
            };
            hprintln!("Sending packet #{}: {}", unsafe{core::str::from_utf8_unchecked(&packet.check_word)}, counter);

            self.radio
                .write_buffer(&mut self.spi, &mut self.delay, 0x00, &Into::<[u8; 10]>::into(packet))
                .unwrap();
            self.radio
                .set_tx(&mut self.spi, &mut self.delay, RxTxTimeout::from_ms(3000))
                .unwrap();

            // Wait for busy line to go low
            self.radio.wait_on_busy(&mut self.delay).unwrap();

            loop {
                if DIO1_RISEN.swap(false, atomic::Ordering::Relaxed) {
                    self.radio
                        .clear_irq_status(&mut self.spi, &mut self.delay, IrqMask::all())
                        .unwrap();
                    break;
                }
                timer.start(10u32.millis()).unwrap();
                NbFuture::new(|| timer.wait()).await.unwrap();
            }
            timer.start(1000u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
            counter += 1;
        }
    }

    pub async fn radio_test_rx(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
    ) {
        self.radio
            .set_rx(&mut self.spi, &mut self.delay, RxTxTimeout::continuous_rx())
            .unwrap();

        const HISTORY_SIZE: usize = 32;
        let mut previous_rssis : Deque<f32, HISTORY_SIZE> = Deque::new();
        let mut previous_lora_rssis : Deque<f32, HISTORY_SIZE> = Deque::new();
        let mut last_received_counters : Deque<u16, HISTORY_SIZE> = Deque::new();

        loop {
            if DIO1_RISEN.swap(false, atomic::Ordering::Relaxed) {
                let irq_status = self
                    .radio
                    .get_irq_status(&mut self.spi, &mut self.delay)
                    .unwrap();
                self.radio
                    .clear_irq_status(&mut self.spi, &mut self.delay, IrqMask::all())
                    .unwrap();
                if !irq_status.timeout() {
                    if irq_status.crc_err() {
                        writeln!(get_serial(), "[info] CRC error");
                        continue;
                    }
                    let rx_buf_status = self
                        .radio
                        .get_rx_buffer_status(&mut self.spi, &mut self.delay)
                        .unwrap();
                    let mut packet = [0; 10];
                    if rx_buf_status.payload_length_rx() != 10 {
                        writeln!(get_serial(), "[err] Wrong packet length");
                        continue;
                    }

                    self.radio
                        .read_buffer(
                            &mut self.spi,
                            &mut self.delay,
                            rx_buf_status.rx_start_buffer_pointer(),
                            &mut packet,
                        )
                        .unwrap();

                    let packet = TestPacket::from(packet);
                    let counter = packet.counter;
                    writeln!(
                        get_serial(),
                        "[info] Received packet #{}: {}",
                        counter,
                        unsafe { core::str::from_utf8_unchecked(&packet.check_word) },
                    );

                    let packet_status = self.radio.get_packet_status(&mut self.spi, &mut self.delay).unwrap();
                    let rssi = packet_status.rssi_pkt();
                    let snr = packet_status.snr_pkt();
                    let lora_rssi = packet_status.signal_rssi_pkt();

                    writeln!(get_serial(), "[info] Stats: rssi: {:3.3} | snr: {:3.3} | signal_rssi: {:3.3}", rssi, snr, lora_rssi);

                    if previous_rssis.len() == HISTORY_SIZE {
                        previous_rssis.pop_front();
                        previous_lora_rssis.pop_front();
                        last_received_counters.pop_front();
                    }
                    previous_rssis.push_back(rssi);
                    previous_lora_rssis.push_back(lora_rssi);
                    last_received_counters.push_back(counter);

                    if previous_rssis.len() == HISTORY_SIZE {
                        let avg_rssi = previous_rssis.iter().sum::<f32>() / HISTORY_SIZE as f32;
                        let avg_lora_rssi = previous_lora_rssis.iter().sum::<f32>() / HISTORY_SIZE as f32;
                        if last_received_counters.back().unwrap() < last_received_counters.front().unwrap() {
                            writeln!(get_serial(), "[warn] Counter overflow detected, clearing history");
                            previous_rssis.clear();
                            previous_lora_rssis.clear();
                            last_received_counters.clear();
                            continue;
                        }
                        let sent_packets = last_received_counters.back().unwrap() - last_received_counters.front().unwrap() + 1;
                        let packet_loss = 1f32 - HISTORY_SIZE as f32 / sent_packets as f32;
                        writeln!(get_serial(), "[info] Avg Stats: avg_rssi: {:3.3} | avg_lora_rssi: {:3.3} | packet_loss: {:3.3}", avg_rssi, avg_lora_rssi, packet_loss);
                    }
                }

                if self
                    .radio
                    .get_status(&mut self.spi, &mut self.delay)
                    .unwrap()
                    .chip_mode()
                    .unwrap()
                    != ChipMode::RX
                {
                    self.radio
                        .set_rx(&mut self.spi, &mut self.delay, RxTxTimeout::from_ms(2000))
                        .unwrap();
                }
            }
            timer.start(100u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }
}
