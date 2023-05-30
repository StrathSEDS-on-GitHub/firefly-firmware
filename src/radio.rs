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
use heapless::Vec;
use stm32f4xx_hal::interrupt;
use fugit::ExtU32;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::timer::CounterMs;
use sx126x::op::ChipMode;
use sx126x::op::IrqMask;
use sx126x::op::RxTxTimeout;

use crate::futures::NbFuture;
use crate::logger::get_serial;
use crate::words::CHECK_WORDS;

const MAX_PAYLOAD_SIZE: usize = 255;
pub struct Lexer<'a> {
    idx: usize,
    data: &'a [u8],
}

impl<'a> Lexer<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { idx: 0, data }
    }

    pub fn remaining_bytes(&self) -> usize {
        self.data.len() - self.idx
    }

    pub fn next_u8(self: &mut Self) -> Option<u8> {
        self.idx += 1;
        self.data.get(self.idx - 1).copied()
    }

    pub fn next_u16(self: &mut Self) -> Option<u16> {
        let it = u16::from_be_bytes(self.data.get(self.idx..self.idx + 2)?.try_into().ok()?);
        self.idx += 2;
        Some(it)
    }

    pub fn next_u32(self: &mut Self) -> Option<u32> {
        let it = u32::from_be_bytes(self.data.get(self.idx..self.idx + 4)?.try_into().ok()?);
        self.idx += 4;
        Some(it)
    }

    pub fn next_u64(self: &mut Self) -> Option<u64> {
        let it = u64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn next_f32(self: &mut Self) -> Option<f32> {
        let it = f32::from_be_bytes(self.data.get(self.idx..self.idx + 4)?.try_into().ok()?);
        self.idx += 4;
        Some(it)
    }

    pub fn next_f64(self: &mut Self) -> Option<f64> {
        let it = f64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn next_i64(self: &mut Self) -> Option<i64> {
        let it = i64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn consume(self: &mut Self, x: u8) -> Option<u8> {
        if *self.data.get(self.idx)? == x {
            self.idx += 1;
            Some(x)
        } else {
            None
        }
    }

    pub fn next_str(self: &mut Self, len: usize) -> Option<&'a str> {
        let it =
            unsafe { core::str::from_utf8_unchecked(self.data.get(self.idx..self.idx + len)?) };
        self.idx += len;
        Some(it)
    }

    pub fn next_bytes(self: &mut Self, len: usize) -> Option<&'a [u8]> {
        let it = self.data.get(self.idx..self.idx + len)?;
        self.idx += len;
        Some(it)
    }

    pub fn next_null_terminated_str(self: &mut Self) -> Option<&'a str> {
        let mut idx = self.idx;
        while *self.data.get(idx)? != 0 {
            idx += 1;
        }
        let it = unsafe {
            core::str::from_utf8_unchecked(self.data.get(self.idx..idx)?.try_into().ok()?)
        };
        self.idx = idx + 1;
        Some(it)
    }

    pub fn at_end(self: &Self) -> bool {
        self.idx == self.data.len()
    }

    pub fn skip_till(&mut self, arg: u8) -> Option<u8> {
        while self.next_u8()? != arg {}
        Some(arg)
    }

    fn pos(&self) -> usize {
        self.idx
    }

    pub fn set_pos(&mut self, pos: usize) {
        self.idx = pos;
    }
}

static DIO1_RISEN: atomic::AtomicBool = atomic::AtomicBool::new(false);
#[interrupt]
fn EXTI4() {
    let dio1_pin = unsafe { crate::DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        DIO1_RISEN.store(true, atomic::Ordering::Relaxed);

        dio1_pin.clear_interrupt_pending_bit();
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Message {
    Ota(u32,[u8; 128]),
}

impl Message {
    pub fn parse(payload: &[u8]) -> Option<Self> {
        let mut lexer = Lexer::new(payload);
        let message_type = lexer.next_str(4)?;
        hprintln!("message_type: {}", message_type);
        match message_type {
            "OTAD" => {
                let start_addr = lexer.next_u32()?;
                let mut data = [0u8; 128];
                data.copy_from_slice(lexer.next_bytes(128)?);
                Some(Self::Ota(start_addr, data))
            }
        }
    }

    pub fn data(&self) -> Vec<u8, MAX_PAYLOAD_SIZE> {
        // get_logger().log(format_args!("{:?}", self));
        match self {
            Message::Ota(start_addr, data) => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"OTAD").unwrap();
                payload.extend(start_addr.to_be_bytes());
                payload.extend_from_slice(data);
                payload
            }
        }
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
