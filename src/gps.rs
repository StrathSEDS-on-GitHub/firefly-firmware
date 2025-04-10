use core::{
    cell::RefCell,
    cmp::min,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use fugit::ExtU32;
use futures::poll;
use hal::{
    dma::{traits::StreamISR, Stream2, Stream7, StreamsTuple, Transfer},
    gpio::{self, Input},
    pac::{NVIC, USART1},
    prelude::_stm32f4xx_hal_time_U32Ext,
    serial::{RxISR, RxListen, SerialExt},
};
use heapless::Deque;
use nmea0183::{datetime::Time, ParseResult, GGA};
use stm32f4xx_hal::{
    dma::{self},
    interrupt, pac,
    prelude::_embedded_hal_serial_nb_Read,
    serial::{Rx, Tx},
};
use time::{Date, PrimitiveDateTime};

use crate::{interrupt_wake, neopixel, RTC};
use stm32f4xx_hal as hal;

static TX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream7<pac::DMA2>,
                4,
                Tx<pac::USART1>,
                dma::MemoryToPeripheral,
                &'static mut [u8],
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static RX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream2<pac::DMA2>,
                4,
                Rx<pac::USART1>,
                dma::PeripheralToMemory,
                &'static mut [u8],
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static RX_BUFFER: Mutex<RefCell<Option<&'static mut [u8; RX_BUFFER_SIZE]>>> =
    Mutex::new(RefCell::new(None));

static RX_BYTES_READ: AtomicUsize = AtomicUsize::new(0);

static REFINE_TIME: AtomicBool = AtomicBool::new(false);

const RX_BUFFER_SIZE: usize = 1024;

pub fn setup(dma2: pac::DMA2, gps: hal::serial::Serial<USART1>) {
    let streams = StreamsTuple::new(dma2);
    let tx_stream = streams.7;
    let rx_stream = streams.2;
    let tx_buf_gps = cortex_m::singleton!(:[u8; 128] = [0; 128]).unwrap();
    let rx_buf1_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let rx_buf2_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = gps.split();

    let mut tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        tx_buf_gps as &mut [u8],
        None,
        dma::config::DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .transfer_complete_interrupt(true),
    );
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
    tx_transfer.start(|_tx| {});
    cortex_m::interrupt::free(|cs| {
        // SAFETY: Mutex makes access of static mutable variable safe
        TX_TRANSFER.borrow(cs).borrow_mut().replace(tx_transfer);
        RX_TRANSFER.borrow(cs).borrow_mut().replace(rx_transfer);
        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf2_gps);
        GPS_SENTENCE_BUFFER
            .borrow(cs)
            .borrow_mut()
            .replace(Deque::new())
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM7);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
    }
}

pub async fn set_nmea_output() {
    let msg = b"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    tx(msg).await;
}

pub async fn change_baudrate(baudrate: u32) {
    cortex_m::interrupt::free(|cs| {
        let tx_transfer = TX_TRANSFER.borrow(cs).replace(None).unwrap();
        let rx_transfer = RX_TRANSFER.borrow(cs).replace(None).unwrap();

        let (rx_stream, _, rx_buf, _) = rx_transfer.release();
        let (tx_stream, _, tx_buf, _) = tx_transfer.release();

        let serial = {
            // Seemingly there's no way to simply rejoin the Tx and Rx and then release so we need to get our hands dirty...
            let mut clocks_ref = crate::CLOCKS.borrow(cs).borrow_mut();
            let clocks = clocks_ref.as_mut().unwrap();

            // FIXME: Holy shit what the fuck is this code
            let usart1: USART1 = unsafe { core::mem::transmute(()) };
            let pa10: gpio::Pin<'A', 15, Input> = unsafe { core::mem::transmute(()) };

            let pa15: gpio::Pin<'A', 10, Input> = unsafe { core::mem::transmute(()) };
            usart1
                .serial(
                    (pa10.into_alternate(), pa15.into_alternate()),
                    hal::serial::config::Config {
                        baudrate: baudrate.bps(),
                        dma: hal::serial::config::DmaConfig::TxRx,
                        ..Default::default()
                    },
                    clocks,
                )
                .unwrap()
        };
        let (tx, mut rx) = serial.split();
        rx.listen_idle();
        let tx_transfer = Transfer::init_memory_to_peripheral(
            tx_stream,
            tx,
            tx_buf as &mut [u8],
            None,
            dma::config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .transfer_complete_interrupt(true),
        );
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
        TX_TRANSFER.borrow(cs).replace(Some(tx_transfer));
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

        crate::futures::gps_rx_wake::future().await;
    }
}

/// Interrupt for gps DMA TX,
#[interrupt]
fn DMA2_STREAM7() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = TX_TRANSFER.borrow(cs).borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if transfer.is_fifo_error() {
            transfer.clear_fifo_error();
        }

        if transfer.is_transfer_complete() {
            transfer.clear_transfer_complete();
            interrupt_wake!(crate::futures::gps_tx_wake);
        }
    });
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
                }
                if let ParseResult::GGA(Some(GGA {
                    fix: Some(_fix), // Don't care about the fix, just want to assure the time is valid.
                    ..
                })) = parse_result.clone()
                {
                    got_fix = true;
                    if !got_fix || nth % 20 == 0 {
                        REFINE_TIME.store(true, Ordering::Relaxed);
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

// Interrupt for radio DMA RX.
#[interrupt]
fn DMA2_STREAM2() {
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

#[interrupt]
fn USART1() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = RX_TRANSFER.borrow(cs).borrow_mut();

        let bytes = RX_BUFFER_SIZE as u16 - transfer_ref.as_mut().unwrap().number_of_transfers();
        transfer_ref.as_mut().unwrap().pause(|rx| {
            rx.clear_idle_interrupt();
        });

        let mut rx_buf = RX_BUFFER.borrow(cs).borrow_mut().take().unwrap();

        if REFINE_TIME.load(Ordering::Relaxed) {
            // Take this opportunity to refine the time
            // by performing a single-shot read of the GPS
            // till we get a GGA sentence, then immediately
            // set the RTC.

            let (stream, mut rx, rx_buf_data, _) = transfer_ref.take().unwrap().release();
            let mut parser = nmea0183::Parser::new();

            loop {
                let c = nb::block!(rx.read()).unwrap();
                if let Some(Ok(parse_result)) = parser.parse_from_byte(c) {
                    if let ParseResult::GGA(Some(GGA {
                        time,
                        fix: Some(_fix), // Don't care about the fix, just want to assure the time is valid.
                        ..
                    })) = parse_result
                    {
                        set_rtc(time);
                        break;
                    }
                }
            }

            // Release the stream and reinitialize it
            // to get it back into the DMA RX state.

            transfer_ref.replace(Transfer::init_peripheral_to_memory(
                stream,
                rx,
                rx_buf,
                None,
                dma::config::DmaConfig::default()
                    .memory_increment(true)
                    .fifo_enable(true)
                    .fifo_error_interrupt(true)
                    .transfer_error_interrupt(true)
                    .direct_mode_error_interrupt(true)
                    .transfer_complete_interrupt(true),
            ));
            rx_buf = rx_buf_data.try_into().unwrap();
            REFINE_TIME.store(false, Ordering::Relaxed);
        } else {
            let transfer = transfer_ref.as_mut().unwrap();
            rx_buf = transfer
                .next_transfer(rx_buf)
                .unwrap()
                .0
                .try_into()
                .unwrap();
        }

        interrupt_wake!(crate::futures::gps_rx_wake);
        RX_BYTES_READ.store(bytes as usize, Ordering::SeqCst);

        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf);
        transfer_ref.as_mut().unwrap().start(|_| {});
    });
}

async fn gps_tx_complete() {
    crate::futures::gps_tx_wake::future().await;
}

async fn gps_rx_complete() {
    crate::futures::gps_rx_wake::future().await;
}

pub async fn tx(tx_buf: &[u8]) {
    let mut fut = crate::futures::gps_tx_wake::future();
    let _ = poll!(&mut fut);
    cortex_m::interrupt::free(|cs| {
        let mut tx_transfer_ref = TX_TRANSFER.borrow(cs).borrow_mut();
        let tx_transfer = tx_transfer_ref.as_mut().unwrap();
        unsafe {
            tx_transfer
                .next_transfer_with(|buf, _| {
                    buf[..tx_buf.len()].copy_from_slice(&tx_buf);
                    (buf, ())
                })
                .unwrap();
        }
    });
    fut.await;
}

pub async fn rx(rx_buf: &mut [u8]) -> usize {
    gps_rx_complete().await;
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
