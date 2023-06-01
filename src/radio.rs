use core::sync::atomic;

use core::fmt::Write;
use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::blocking::spi::Write as SPIWrite;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use f4_w25q::w25q::Address;
use f4_w25q::w25q::SectorAddress;
use f4_w25q::w25q::W25Q;
use fugit::ExtU32;
use heapless::Deque;
use heapless::Vec;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::prelude::_stm32f4xx_hal_gpio_ExtiPin;
use stm32f4xx_hal::qspi::QspiPins;
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
    Ota(u32, [u8; 128]),
    OtaDone,
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
            "OTAQ" => Some(Self::OtaDone),
            _ => None,
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
            Message::OtaDone => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"OTAQ").unwrap();
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

    pub async fn radio_tx_ota(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
        firmware: &[u8],
    ) {
        for (i, chunk) in firmware.chunks(128).enumerate() {
            let start_addr = i as u32;
            let message = Message::Ota(start_addr, chunk.try_into().unwrap());

            self.radio
                .write_buffer(
                    &mut self.spi,
                    &mut self.delay,
                    0x00,
                    message.data().as_slice(),
                )
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
        }
        let message = Message::OtaDone;

        self.radio
            .write_buffer(
                &mut self.spi,
                &mut self.delay,
                0x00,
                message.data().as_slice(),
            )
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
        timer.start(10u32.millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }

    pub async fn radio_rx_ota<PINS>(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
        flash: &mut W25Q<PINS>,
    ) -> u32
    where
        PINS: QspiPins,
    {
        let mut packets_recvd = 0;
        self.radio
            .set_rx(&mut self.spi, &mut self.delay, RxTxTimeout::continuous_rx())
            .unwrap();

        loop {
            if DIO1_RISEN.swap(false, atomic::Ordering::Relaxed) {
                let irq_status = self
                    .radio
                    .get_irq_status(&mut self.spi, &mut self.delay)
                    .unwrap();
                self.radio
                    .clear_irq_status(&mut self.spi, &mut self.delay, IrqMask::all())
                    .unwrap();

                let rx_buf_status = self
                    .radio
                    .get_rx_buffer_status(&mut self.spi, &mut self.delay)
                    .unwrap();

                if !irq_status.timeout() {
                    let mut packet = [0u8; 128 + 4];
                    self.radio
                        .read_buffer(
                            &mut self.spi,
                            &mut self.delay,
                            rx_buf_status.rx_start_buffer_pointer(),
                            &mut packet,
                        )
                        .unwrap();

                    let packet = Message::parse(&packet).unwrap();
                    match packet {
                        Message::Ota(start_addr, data) => {
                            let start: u32 = 4096 + start_addr * 128;
                            if start % 4096 == 0 {
                                flash
                                    .erase_sector(SectorAddress::from_address(start))
                                    .unwrap();
                            }
                            packets_recvd += 1;
                            flash.program_page(start.into(), &data).unwrap();
                        }
                        Message::OtaDone => {
                            return packets_recvd;
                        }
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
            timer.start(10u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }
}
