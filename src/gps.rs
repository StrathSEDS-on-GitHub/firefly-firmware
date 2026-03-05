use core::fmt::Write as _;

use fugit::ExtU32 as _;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::prelude::_embedded_hal_serial_nb_Write;

use hal::{
    dma::Transfer,
    pac::NVIC,
    prelude::_stm32f4xx_hal_time_U32Ext,
    serial::{RxListen, SerialExt},
};
use nmea0183::{ParseResult, Parser, datetime::Time};
use stm32f4xx_hal::{dma::config::DmaConfig, gpio::ExtiPin, pac};
use thingbuf::mpsc::{RecvRef, StaticReceiver};
use time::{Date, PrimitiveDateTime};

use crate::{
    CLOCKS, PPS_PIN, RTC,
    logs::FixedWriter,
    neopixel,
    pins::gps::{GPSRxStream, GPSUsart},
};
use stm32f4xx_hal as hal;

mod dma {
    use core::{
        cell::RefCell,
        ops::{Deref, DerefMut},
    };

    use cortex_m::interrupt::Mutex;
    use stable_deref_trait::StableDeref;
    use stm32f4xx_hal::{
        dma::{PeripheralToMemory, Transfer, traits::StreamISR as _},
        serial::Rx,
    };
    use thingbuf::mpsc::{StaticChannel, StaticSender};

    use crate::pins::gps::{GPSRxStream, GPSUsart};

    pub(crate) const RX_BUFFER_SIZE: usize = 1024;

    pub static STATE: GPSState = GPSState {
        channel: thingbuf::mpsc::StaticChannel::new(),
    };

    pub(crate) static RX_TRANSFER: Mutex<
        RefCell<Option<Transfer<GPSRxStream, 4, Rx<GPSUsart>, PeripheralToMemory, RxSendRef>>>,
    > = Mutex::new(RefCell::new(None));

    #[derive(Clone)]
    pub struct RxBuffer(pub(crate) [u8; RX_BUFFER_SIZE]);

    impl Default for RxBuffer {
        fn default() -> Self {
            RxBuffer([0u8; RX_BUFFER_SIZE])
        }
    }

    pub struct RxSendRef<'a>(pub thingbuf::mpsc::SendRef<'a, RxBuffer>);

    pub struct GPSState {
        pub(super) channel: StaticChannel<RxBuffer, 2>,
    }

    pub static mut SENDER: Option<StaticSender<RxBuffer>> = None;

    pub fn sender() -> &'static StaticSender<RxBuffer> {
        // SAFETY: SENDER is written once in GPS::setup before any calls to this function and is never modified after that.
        unsafe { SENDER.as_ref().unwrap() }
    }

    impl Deref for RxSendRef<'_> {
        type Target = [u8; RX_BUFFER_SIZE];

        fn deref(&self) -> &Self::Target {
            &self.0.deref().0
        }
    }

    impl DerefMut for RxSendRef<'_> {
        fn deref_mut(&mut self) -> &mut Self::Target {
            &mut self.0.deref_mut().0
        }
    }

    unsafe impl StableDeref for RxSendRef<'_> {}

    pub fn dma_interrupt_impl() {
        cortex_m::interrupt::free(|cs| {
            let mut transfer_ref = RX_TRANSFER.borrow(cs).borrow_mut();
            let transfer = transfer_ref.as_mut().unwrap();

            // Its important to clear fifo errors as the transfer is paused until it is cleared
            if transfer.is_fifo_error() {
                transfer.clear_fifo_error();
            }

            if transfer.is_transfer_complete() {
                transfer.clear_transfer_complete();

                // SAFETY: um hopefully an overrun doesn't occur lol?
                unsafe {
                    transfer
                        .next_transfer_with(|buf, _| {
                            (
                                // Try to acquire a new buffer to write into, but if tasked code is running
                                // behind and hasn't released any new buffers to use then just reuse the
                                // same buffer to keep data fresh.
                                (sender().try_send_ref().ok().map(RxSendRef).unwrap_or(buf)),
                                (),
                            )
                        })
                        .unwrap();
                }
            }
        });
    }
}

pub struct GPS {
    receiver: StaticReceiver<dma::RxBuffer>,
    tx: hal::serial::Tx<GPSUsart>,
}

impl GPS {
    pub fn setup(rx_stream: GPSRxStream, serial: hal::serial::Serial<GPSUsart>) -> GPS {
        let (send, recv) = dma::STATE.channel.split();

        unsafe {
            dma::SENDER.replace(send);
        };

        let (tx, mut rx) = serial.split();
        rx.listen_idle();
        let mut rx_transfer = Transfer::init_peripheral_to_memory(
            rx_stream,
            rx,
            dma::sender()
                .try_send_ref()
                .map(dma::RxSendRef)
                .expect("just created the channel"),
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_error_interrupt(true)
                .direct_mode_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        rx_transfer.start(|_rx| {});

        cortex_m::interrupt::free(|cs| {
            dma::RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
        });

        unsafe {
            // all potential interrupts
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM5);
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM2);
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
        }

        GPS { receiver: recv, tx }
    }

    pub async fn tx(&mut self, buf: &[u8]) {
        buf.iter().for_each(|byte| {
            nb::block!(self.tx.write(*byte)).unwrap();
        });
    }

    pub async fn rx(&self) -> RecvRef<'_, dma::RxBuffer> {
        self.receiver
            .recv_ref()
            .await
            .expect("channel is never closed")
    }

    pub async fn set_nmea_output(&mut self) {
        let msg = b"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
        self.tx(msg).await;
    }

    pub async fn reinit_with_rate(self, baudrate: u32) -> Self {
        cortex_m::interrupt::free(|cs| {
            let rx_transfer = dma::RX_TRANSFER.borrow(cs).replace(None).unwrap();

            let (rx_stream, rx, rx_buf, _) = rx_transfer.release();
            let clocks = CLOCKS.borrow(cs).get().unwrap();

            let serial = {
                let serial = rx.join(self.tx);
                let (usart, txrx) = serial.release();
                usart
                    .serial(
                        txrx,
                        hal::serial::config::Config {
                            baudrate: baudrate.bps(),
                            dma: hal::serial::config::DmaConfig::Rx,
                            ..Default::default()
                        },
                        &clocks,
                    )
                    .unwrap()
            };
            let (tx, mut rx) = serial.split();
            rx.listen_idle();
            let mut rx_transfer = Transfer::init_peripheral_to_memory(
                rx_stream,
                rx,
                rx_buf,
                None,
                DmaConfig::default()
                    .memory_increment(true)
                    .fifo_enable(true)
                    .fifo_error_interrupt(true)
                    .transfer_error_interrupt(true)
                    .direct_mode_error_interrupt(true)
                    .transfer_complete_interrupt(true),
            );
            rx_transfer.start(|_rx| {});

            GPS {
                receiver: self.receiver,
                tx,
            }
        })
    }

    pub async fn init_teseo(&mut self) {
        //$PSTMINITGPS,<Lat>,<LatRef>,<Lon>,<LonRef>,<Alt>,<Day>,<Month>,<Year>,<Hour>,<Minute>,<Second>*<checksum><cr><lf>

        const GLASGOW_LAT: &'static str = "5551.675,N";
        const GLASGOW_LONG: &'static str = "00414.616,W";

        let mut command_storage = [0u8; 256];
        let mut writer = FixedWriter::new(&mut command_storage);
        write!(
            &mut writer,
            "$PSTMINITGPS,{GLASGOW_LAT},{GLASGOW_LONG},0035,"
        )
        .unwrap();
        write!(&mut writer, "{}", env!("BUILD_TIME")).unwrap();

        let checksum = writer.data().iter().skip(1).fold(0u8, |acc, &x| acc ^ x);
        write!(&mut writer, "*{checksum:02X}\r\n").unwrap();

        self.tx(writer.data()).await;
    }

    pub async fn set_par(
        &mut self,
        config_block: ConfigBlock,
        id: u8,
        param_value: &[u8],
        mode: Option<Mode>,
    ) {
        //$PSTMSETPAR,<ConfigBlock><ID>,<param_value>[,<mode>]*<checksum><cr><lf>
        const STORAGE_LENGTH: usize = 256;

        let mut command_storage = [0u8; STORAGE_LENGTH];
        let mut writer = FixedWriter::new(&mut command_storage);
        write!(&mut writer, "$PSTMSETPAR,{}{id:03},", config_block as u8).unwrap();

        writer.copy_from_slice(&param_value);

        if let Some(mode) = mode {
            write!(&mut writer, ",{:01}", mode as u8).unwrap();
        }
        let checksum = writer.data().iter().skip(1).fold(0u8, |acc, &x| acc ^ x);
        write!(&mut writer, "*{checksum:02X}\r\n").unwrap();

        self.tx(writer.data()).await;
    }
}

pub struct GPSParser<'buf> {
    parser: Parser,
    buffer: Option<RecvRef<'buf, dma::RxBuffer>>,
    position: usize,
    got_initial_fix: bool,
    searching_fix_flasher: bool,
}

impl<'buf> GPSParser<'buf> {
    pub fn new() -> Self {
        GPSParser {
            parser: Parser::new(),
            buffer: None,
            position: 0,
            got_initial_fix: false,
            searching_fix_flasher: false,
        }
    }
}

impl<'buf> GPSParser<'buf> {
    pub async fn next(&mut self, gps: &'buf GPS) -> ParseResult {
        loop {
            if self.buffer.is_none() || self.position >= self.buffer.as_ref().unwrap().0.len() {
                self.position = 0;
                self.buffer.replace(gps.rx().await);
            }

            let rx_buf = &(*self.buffer.as_ref().unwrap()).0;
            let parsed = self.parser.parse_from_byte(rx_buf[self.position]);
            self.position += 1;

            let Some(Ok(parsed)) = parsed else {
                continue;
            };

            self.searching_fix_flasher = !self.searching_fix_flasher;
            if !self.got_initial_fix {
                neopixel::update_pixel(
                    1,
                    [
                        self.searching_fix_flasher as u8 * 100,
                        self.searching_fix_flasher as u8 * 100,
                        0,
                    ],
                );
                // TODO: RMC instead?
                if let ParseResult::GGA(Some(nmea0183::GGA {
                    fix: Some(_fix), // Don't care about the fix, just want to assure the time is valid.
                    time,
                    ..
                })) = &parsed
                {
                    set_rtc(time.clone());
                    self.got_initial_fix = true;
                }
            }

            return parsed;
        }
    }
}

fn set_rtc(time: Time) {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        rtc.set_datetime(&PrimitiveDateTime::new(
            Date::from_calendar_date(2023, time::Month::June, 26).unwrap(),
            time::Time::from_hms_milli(
                time.hours,
                time.minutes,
                time.seconds as u8,
                ((time.seconds % 1.0) * 1000.0) as u16,
            )
            .unwrap(),
        ))
        .unwrap();
        rtc.enable_wakeup(2u32.millis().into());

        NVIC::unpend(pac::Interrupt::RTC_WKUP);
        unsafe {
            NVIC::unmask(pac::Interrupt::RTC_WKUP);
        }

        neopixel::update_pixel(1, [0, 65, 32])
    });
}

#[allow(unused)]
pub enum ConfigBlock {
    ConfigCurrent = 1,
    ConfigDefault = 2,
    ConfigNVMStored = 3,
}

#[allow(unused)]
pub enum Mode {
    Overwrite = 0,
    OrMask = 1,
    AndMask = 2,
}

#[allow(non_snake_case)]
#[interrupt]
fn DMA1_STREAM5() {
    dma::dma_interrupt_impl();
}

#[allow(non_snake_case)]
#[interrupt]
fn DMA1_STREAM2() {
    dma::dma_interrupt_impl();
}

fn pps_interrupt_impl() {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        let time = rtc.get_datetime().time();
        let carry = if time.millisecond() > 500 { 1 } else { 0 };
        let (second, carry) = ((time.second() + carry) % 60, (time.second() + carry) / 60);
        let (minute, carry) = ((time.minute() + carry) % 60, (time.minute() + carry) / 60);
        let (hour, _carry) = ((time.hour() + carry) % 60, (time.hour() + carry) / 60);

        rtc.set_time(&time::Time::from_hms_milli(hour, minute, second, 0).unwrap())
            .unwrap();

        PPS_PIN
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt_pending_bit();
    });
}

#[cfg(feature = "target-mini")]
#[interrupt]
fn EXTI2() {
    pps_interrupt_impl();
}

#[cfg(feature = "target-ultra")]
#[interrupt]
fn EXTI9_5() {
    pps_interrupt_impl();
}

#[cfg(feature = "target-maxi")]
#[interrupt]
fn EXTI15_10() {
    pps_interrupt_impl();
}
