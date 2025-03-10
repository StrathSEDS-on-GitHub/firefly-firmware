/* 
use core::{
    cell::RefCell,
    cmp::min,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use hal::{
    dma::{Stream5, Stream6, StreamsTuple, Transfer},
    pac::USART2,
    serial::{RxISR, RxListen},
};
use stm32f4xx_hal::{
    dma::{
        self,
        traits::StreamISR,
    },
    interrupt, pac,
    serial::{Rx, Tx},
};

use crate::futures::YieldFuture;
use stm32f4xx_hal as hal;

static TX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream6<pac::DMA1>,
                4,
                Tx<pac::USART2>,
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
                Stream5<pac::DMA1>,
                4,
                Rx<pac::USART2>,
                dma::PeripheralToMemory,
                &'static mut [u8],
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static RX_BUFFER: Mutex<RefCell<Option<&'static mut [u8; RX_BUFFER_SIZE]>>> =
    Mutex::new(RefCell::new(None));

static TX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_BYTES_READ: AtomicUsize = AtomicUsize::new(0);

const RX_BUFFER_SIZE: usize = 1024;

pub fn setup(dma2: pac::DMA1, gps: hal::serial::Serial<USART2>) {
    let streams = StreamsTuple::new(dma2);
    let tx_stream = streams.6;
    let rx_stream = streams.5;
    let tx_buf_gps = cortex_m::singleton!(:[u8; 128] = [0; 128]).unwrap();
    let rx_buf1_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let rx_buf2_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = gps.split();

    let tx_transfer = Transfer::init_memory_to_peripheral(
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
    // tx_transfer.start(|_tx| {});
    cortex_m::interrupt::free(|cs| {
        // SAFETY: Mutex makes access of static mutable variable safe
        TX_TRANSFER.borrow(cs).borrow_mut().replace(tx_transfer);
        RX_TRANSFER.borrow(cs).borrow_mut().replace(rx_transfer);
        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf2_gps);
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM6);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }
}

/// Interrupt for gps DMA TX,
#[interrupt]
fn DMA1_STREAM6() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = TX_TRANSFER.borrow(cs).borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if transfer.is_fifo_error() {
            transfer.clear_fifo_error();
        }

        if transfer.is_transfer_complete() {
            transfer.clear_transfer_complete();
            TX_COMPLETE.store(true, Ordering::Relaxed);
        }
    });
}

// Interrupt for radio DMA RX.
#[interrupt]
fn DMA1_STREAM5() {
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
            RX_COMPLETE.store(true, Ordering::SeqCst);
            RX_BYTES_READ.store(rx_buf.len(), Ordering::SeqCst);
            RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf);
        }
    });
}

#[interrupt]
fn USART2() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = RX_TRANSFER.borrow(cs).borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        let bytes = RX_BUFFER_SIZE as u16 - transfer.number_of_transfers();
        transfer.pause(|rx| {
            rx.clear_idle_interrupt();
        });

        let mut rx_buf = RX_BUFFER.borrow(cs).borrow_mut().take().unwrap();

        rx_buf = transfer
            .next_transfer(rx_buf)
            .unwrap()
            .0
            .try_into()
            .unwrap();
        RX_COMPLETE.store(true, Ordering::SeqCst);
        RX_BYTES_READ.store(bytes as usize, Ordering::SeqCst);

        RX_BUFFER.borrow(cs).borrow_mut().replace(rx_buf);
        transfer.start(|_| {});
    });
}

async fn tx_complete() {
    loop {
        if TX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

async fn rx_complete() {
    loop {
        if RX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

pub async fn tx(tx_buf: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        let mut tx_transfer_ref = TX_TRANSFER.borrow(cs).borrow_mut();
        let tx_transfer = tx_transfer_ref.as_mut().unwrap();
        TX_COMPLETE.store(false, Ordering::SeqCst);
        // SAFETY: not double buffered.
        unsafe {
            tx_transfer
                .next_transfer_with(|buf, _| {
                    buf[..tx_buf.len()].copy_from_slice(&tx_buf);
                    buf.split_at_mut(tx_buf.len())
                })
                .unwrap();
            tx_transfer.start(|_tx| {});
        }
    });
    tx_complete().await;
}

pub async fn rx(rx_buf: &mut [u8]) -> usize {
    rx_complete().await;
    cortex_m::interrupt::free(|cs| {
        // SAFETY: not double buffered.
        let mut buf_ref = RX_BUFFER.borrow(cs).borrow_mut();
        let buf = buf_ref.as_mut().unwrap();
        let bytes_available = RX_BYTES_READ.load(Ordering::SeqCst) as usize;

        let bytes_copied = min(rx_buf.len(), bytes_available);
        rx_buf[..bytes_copied].copy_from_slice(&buf[..bytes_copied]);
        RX_COMPLETE.store(bytes_available - bytes_copied != 0, Ordering::SeqCst);
        RX_BYTES_READ.store(bytes_available - bytes_copied, Ordering::SeqCst);

        for i in bytes_copied..bytes_available {
            buf[i - bytes_copied] = buf[i]
        }
        // hprintln!("{:?}", core::str::from_utf8(&rx_buf[..bytes_copied]));
        bytes_copied
    })
}
*/