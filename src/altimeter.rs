
#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
use crate::pins::i2c::Altimeter;
#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
use core::cell::RefCell;
#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
use cortex_m::interrupt::Mutex;
use heapless::Vec;
use stm32f4xx_hal::i2c::dma::I2CMasterWriteReadDMA;

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
use stm32f4xx_hal::interrupt;

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct PressureTemp {
    pub pressure: f32,
    pub temperature: f32,
}

pub type FifoFrames = Vec<PressureTemp, ALTIMETER_FRAME_COUNT>;

#[allow(dead_code)]
pub trait AltimeterFifoDMA<const FRAMES: usize, const BUF_SIZE: usize>:
    I2CMasterWriteReadDMA
{
    const FIFO_READ_REG: u8;
    const ADDRESS: u8;

    fn dma_interrupt(&mut self);
    fn process_fifo_buffer(&self, data: [u8; BUF_SIZE]) -> Vec<PressureTemp, FRAMES>;

    fn decode_frame(pres: &[u8], temp: &[u8]) -> (u32, u32) {
        (
            (pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
            (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16,
        )
    }
}

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
static BMP: Mutex<RefCell<Option<Altimeter>>> = Mutex::new(RefCell::new(None));

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
#[interrupt]
fn DMA1_STREAM() {
    cortex_m::interrupt::free(|cs| {
        let mut i2c = BMP.borrow(cs).borrow_mut();
        i2c.as_mut().unwrap().dma_interrupt();
    });
}

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
#[interrupt]
fn DMA1_STREAM0() {
    cortex_m::interrupt::free(|cs| {
        let mut i2c = BMP.borrow(cs).borrow_mut();
        i2c.as_mut().unwrap().dma_interrupt();
    });
}

#[cfg(feature = "target-mini")]
pub const ALTIMETER_FRAME_COUNT: usize = crate::bmp388::BMP388Wrapper::FRAME_COUNT;

#[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
pub const ALTIMETER_FRAME_COUNT: usize = crate::ms5607::ALTIMETER_FRAME_COUNT;

#[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
pub const ALTIMETER_FRAME_COUNT: usize = crate::bmp581::BMP581::FRAME_COUNT;

#[cfg(feature = "target-mini")]
pub const ALTIMETER_BUF_SIZE: usize = crate::bmp388::BMP388Wrapper::BUF_SIZE;

#[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
pub const ALTIMETER_BUF_SIZE: usize = crate::bmp581::BMP581::BUF_SIZE;

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
pub async fn read_altimeter_fifo(bmp: Altimeter) -> (FifoFrames, Altimeter) {
    use crate::futures::YieldFuture;
    use crate::interrupt_wake;
    use stm32f4xx_hal::pac;
    use crate::futures::bmp_wake;
    use core::ptr::addr_of_mut;
    use core::sync::atomic::AtomicBool;
    static mut DATA: [u8; ALTIMETER_BUF_SIZE] = [0u8; ALTIMETER_BUF_SIZE];

    cortex_m::interrupt::free(|cs| {
        BMP.borrow(cs).replace(Some(bmp));
    });

    unsafe {
        // unmask the DMA interrupt
        pac::NVIC::unmask(pac::Interrupt::DMA1_STREAM0);
        pac::NVIC::unmask(pac::Interrupt::DMA1_STREAM);
    }

    static DONE: AtomicBool = AtomicBool::new(false);
    DONE.store(false, core::sync::atomic::Ordering::Release);

    unsafe {
        let mut done = false;
        while !done {
            cortex_m::interrupt::free(|cs| {
                if let Ok(()) = BMP
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .write_read_dma(
                        Altimeter::ADDRESS,
                        &[Altimeter::FIFO_READ_REG],
                        &mut *addr_of_mut!(DATA),
                        Some(|_| {
                            DONE.store(true, core::sync::atomic::Ordering::Relaxed);
                            interrupt_wake!(bmp_wake);
                        }),
                    )
                {
                    done = true
                }
            });

            // Busy, yield (or BMP is fucked and we aren't going to progress anyway)
            YieldFuture::new().await;
        }
    }
    while !DONE.load(core::sync::atomic::Ordering::Relaxed) {
        // Wait for the DMA to complete
        bmp_wake::future().await;
    }

        // clear the DMA interrupt
    pac::NVIC::mask(pac::Interrupt::DMA1_STREAM0);
    pac::NVIC::mask(pac::Interrupt::DMA1_STREAM);

    let bmp = cortex_m::interrupt::free(|cs| BMP.borrow(cs).replace(None).unwrap());
    unsafe { bmp.dma_complete().expect("Failed to mark DMA complete?") };
    let frames = bmp.process_fifo_buffer(unsafe { DATA });
    (frames, bmp)
}
