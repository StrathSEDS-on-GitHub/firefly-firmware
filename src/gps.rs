use core::{
    cell::RefCell,
    cmp::min,
    fmt::Write as _,
    sync::atomic::{AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::gpio::ExtiPin;
use fugit::ExtU32;
use hal::{
    dma::{Transfer, traits::StreamISR},
    pac::NVIC,
    prelude::_stm32f4xx_hal_time_U32Ext,
    serial::{RxISR, RxListen, SerialExt},
};
use heapless::Deque;
use nmea0183::{GGA, ParseResult, datetime::Time};
use stm32f4xx_hal::{
    dma, interrupt, pac,
    prelude::_embedded_hal_serial_nb_Write,
    serial::{Rx, Tx},
};
use time::{Date, PrimitiveDateTime};

use crate::{
    PPS_PIN, RTC,
    futures::YieldFuture,
    interrupt_wake,
    logs::FixedWriter,
    neopixel,
    pins::gps::{GPSPins, GPSRxStream, GPSUsart},
};
use stm32f4xx_hal as hal;

static RX_TRANSFER: Mutex<
    RefCell<
        Option<Transfer<GPSRxStream, 4, Rx<GPSUsart>, dma::PeripheralToMemory, &'static mut [u8]>>,
    >,
> = Mutex::new(RefCell::new(None));

static TX: Mutex<RefCell<Option<Tx<GPSUsart>>>> = Mutex::new(RefCell::new(None));
static RX_BUFFER: Mutex<RefCell<Option<&'static mut [u8; RX_BUFFER_SIZE]>>> =
    Mutex::new(RefCell::new(None));

static RX_BYTES_READ: AtomicUsize = AtomicUsize::new(0);

const RX_BUFFER_SIZE: usize = 2048;

pub fn setup(rx_stream: GPSRxStream, gps: hal::serial::Serial<GPSUsart>) {
    let rx_buf1_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let rx_buf2_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = gps.split();

    rx.listen_idle();
    let mut rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        rx_buf1_gps as &mut [u8],
        None,
        dma::config::DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_error_interrupt(true)
            .direct_mode_error_interrupt(true)
            .transfer_complete_interrupt(true),
    );
    rx_transfer.start(|_rx| {});
    cortex_m::interrupt::free(|cs| {
        TX.borrow(cs).borrow_mut().replace(tx);
        RX_TRANSFER.borrow(cs).borrow_mut().replace(rx_transfer);
        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf2_gps);
        GPS_SENTENCE_BUFFER
            .borrow(cs)
            .borrow_mut()
            .replace(Deque::new())
    });
    unsafe {
        // all potential interrupts
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
    }
}

pub async fn set_nmea_output() {
    let msg = b"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    tx(msg).await;
}

pub async fn change_baudrate(baudrate: u32) {
    cortex_m::interrupt::free(|cs| {
        let rx_transfer = RX_TRANSFER.borrow(cs).replace(None).unwrap();

        let (rx_stream, _, rx_buf, _) = rx_transfer.release();

        let serial = {
            // Seemingly there's no way to simply rejoin the Tx and Rx and then release so we need to get our hands dirty...
            let clocks = crate::CLOCKS.borrow(cs).get().unwrap();

            // FIXME: Holy shit what the fuck is this code
            let usart: GPSUsart = unsafe { core::mem::transmute(()) };
            let pins: GPSPins = unsafe { core::mem::transmute(()) };

            usart
                .serial(
                    pins,
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
            rx_buf as &mut [u8],
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_error_interrupt(true)
                .direct_mode_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );
        rx_transfer.start(|_rx| {});

        RX_BYTES_READ.store(0, Ordering::SeqCst);
        TX.borrow(cs).borrow_mut().replace(tx);
        RX_TRANSFER.borrow(cs).replace(Some(rx_transfer));
    });
}

static GPS_SENTENCE_BUFFER: Mutex<RefCell<Option<Deque<ParseResult, 64>>>> =
    Mutex::new(RefCell::new(None));

pub async fn next_sentence() -> ParseResult {
    loop {
        if let Some(x) = cortex_m::interrupt::free(|cs| {
            let next = GPS_SENTENCE_BUFFER
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .pop_front();

            next
        }) {
            return x;
        }

        YieldFuture::new().await;
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

// Runs as a background job and puts GPS frames into GPS_SENTENCE_BUFFER.
// Call next_sentence() to get the next available sentence.
pub async fn poll_for_sentences() -> ! {
    let mut parser = nmea0183::Parser::new();
    let mut rx_buf = [0u8; RX_BUFFER_SIZE];
    let mut start = 0;

    let mut bytes = crate::gps::rx(&mut rx_buf[start..]).await;

    let mut nth = 0u32;

    let mut searching_fix_color = false;
    let mut got_fix = false;

    loop {
        for (i, c) in rx_buf[start..bytes].iter().enumerate() {
            if let Some(Ok(parse_result)) = parser.parse_from_byte(*c) {
                searching_fix_color = !searching_fix_color;
                if !got_fix {
                    neopixel::update_pixel(
                        1,
                        [
                            searching_fix_color as u8 * 100,
                            searching_fix_color as u8 * 100,
                            0,
                        ],
                    );
                    if let ParseResult::GGA(Some(GGA {
                        fix: Some(_fix), // Don't care about the fix, just want to assure the time is valid.
                        time,
                        ..
                    })) = parse_result.clone()
                    {
                        set_rtc(time);
                        got_fix = true;
                    }
                }

                cortex_m::interrupt::free(|cs| {
                    let mut gps_buffer_ref = GPS_SENTENCE_BUFFER.borrow(cs).borrow_mut();
                    let gps_sentence_buffer = gps_buffer_ref.as_mut().unwrap();
                    gps_sentence_buffer
                        .push_back(parse_result)
                        .or_else(|parse_result| {
                            // Running behind, clear the buffer so old data is dropped.
                            gps_sentence_buffer.clear();
                            gps_sentence_buffer.push_back(parse_result)
                        })
                        .unwrap();
                });
                start = i + 1;
            }
        }
        nth = nth.wrapping_add(1);

        // Once we've processed all the bytes, or if the buffer is totally full then reset the buffer.
        // The buffer may be full if none of the bytes were parseable.
        if start == bytes || bytes == rx_buf.len() {
            start = 0;
            bytes = 0;
        }

        bytes += crate::gps::rx(&mut rx_buf[bytes..]).await;
    }
}

pub async fn init_teseo() {
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

    tx(writer.data()).await;
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

pub async fn set_par(config_block: ConfigBlock, id: u8, param_value: &[u8], mode: Option<Mode>) {
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

    tx(writer.data()).await;
}

#[interrupt]
#[allow(non_snake_case)]
fn DMA1_STREAM5() {
    dma_interrupt_impl();
}

#[interrupt]
#[allow(non_snake_case)]
fn DMA1_STREAM2() {
    dma_interrupt_impl();
}

fn dma_interrupt_impl() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = RX_TRANSFER.borrow(cs).borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if transfer.is_fifo_error() {
            transfer.clear_fifo_error();
        }
        if transfer.is_transfer_complete() {
            transfer.clear_transfer_complete();
            let mut rx_buf = RX_BUFFER.borrow(cs).borrow_mut().take().unwrap();
            rx_buf = transfer
                .next_transfer(rx_buf)
                .unwrap()
                .0
                .try_into()
                .unwrap();
            interrupt_wake!(crate::futures::gps_rx_wake);
            RX_BYTES_READ.store(rx_buf.len(), Ordering::SeqCst);
            RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf);
        }
    });
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

#[interrupt]
#[allow(non_snake_case)]
fn USART2() {
    usart_interrupt_impl();
}

#[interrupt]
#[allow(non_snake_case)]
fn USART1() {
    usart_interrupt_impl();
}

fn usart_interrupt_impl() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = RX_TRANSFER.borrow(cs).borrow_mut();

        let bytes = RX_BUFFER_SIZE as u16 - transfer_ref.as_mut().unwrap().number_of_transfers();
        transfer_ref.as_mut().unwrap().pause(|rx| {
            rx.clear_idle_interrupt();
        });

        let mut rx_buf = RX_BUFFER.borrow(cs).borrow_mut().take().unwrap();

        let transfer = transfer_ref.as_mut().unwrap();
        rx_buf = transfer
            .next_transfer(rx_buf)
            .unwrap()
            .0
            .try_into()
            .unwrap();

        interrupt_wake!(crate::futures::gps_rx_wake);
        RX_BYTES_READ.store(bytes as usize, Ordering::SeqCst);
        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf);
        transfer_ref.as_mut().unwrap().start(|_| {});
    });
}

pub async fn tx(buf: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        let mut tx_ref = TX.borrow(cs).borrow_mut();
        let tx = tx_ref.as_mut().unwrap();
        buf.iter().for_each(|byte| {
            nb::block!(tx.write(*byte)).unwrap();
        });
    });
}

pub async fn rx(rx_buf: &mut [u8]) -> usize {
    crate::futures::gps_rx_wake::future().await;
    cortex_m::interrupt::free(|cs| {
        let mut buf_ref = RX_BUFFER.borrow(cs).borrow_mut();
        let buf = buf_ref.as_mut().unwrap();
        let bytes_available = RX_BYTES_READ.load(Ordering::SeqCst) as usize;

        let bytes_copied = min(rx_buf.len(), bytes_available);
        rx_buf[..bytes_copied].copy_from_slice(&buf[..bytes_copied]);
        RX_BYTES_READ.store(bytes_available - bytes_copied, Ordering::SeqCst);

        for i in bytes_copied..bytes_available {
            buf[i - bytes_copied] = buf[i]
        }
        bytes_copied
    })
}
