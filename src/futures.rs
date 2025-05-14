use core::{
    convert::Infallible, future::Future, pin, sync::atomic::{AtomicBool, AtomicUsize, Ordering}, task::Waker
};
use embedded_hal_async::delay::DelayNs;
use fugit::ExtU32;
use stm32f4xx_hal::{rcc::Clocks, timer::{self, TimerExt}};
use thingbuf::mpsc::{errors::TrySendError, StaticSender};


pub struct TimerDelay<TIM>
where
    TIM: TimerExt,
{
    timer: Option<TIM>,
    clocks: Clocks
}

impl <TIM> TimerDelay<TIM>
where
    TIM: TimerExt,
{
    pub fn new(timer: TIM, clocks: Clocks) -> Self {
        TimerDelay { timer: Some(timer), clocks }
    }
}

impl<TIM: TimerExt + timer::Instance> DelayNs for TimerDelay<TIM> {
    async fn delay_ns(&mut self, _: u32) {
        self.delay_us(1).await; // Best we can do, we don't have nanosecond timing.
    }
    async fn delay_us(&mut self, us: u32) {
        let mut counter = self.timer.take().unwrap().counter_us(&self.clocks);
        counter.start(us.micros()).unwrap();
        NbFuture::new(|| counter.wait()).await.unwrap();
        self.timer.replace(counter.release().release());
    }
    async fn delay_ms(&mut self, ms: u32) {
        let mut counter = self.timer.take().unwrap().counter_us(&self.clocks);
        counter.start(ms.millis()).unwrap();
        NbFuture::new(|| counter.wait()).await.unwrap();
        self.timer.replace(counter.release().release());
    }
}

pub struct NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    function: F,
}

impl<T, E, F> Future for NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    type Output = Result<T, E>;

    fn poll(
        mut self: pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        match (self.function)() {
            Ok(value) => core::task::Poll::Ready(Ok(value)),
            Err(nb::Error::WouldBlock) => {
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            }
            Err(nb::Error::Other(e)) => core::task::Poll::Ready(Err(e)),
        }
    }
}

impl<T, E, F> NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    pub fn new(function: F) -> Self {
        NbFuture { function }
    }
}

/// Empty future. Await on it in order to yield execution to other tasks.
pub struct YieldFuture {
    yielded: bool,
}

impl Future for YieldFuture {
    type Output = ();

    fn poll(
        mut self: pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        match self.yielded {
            true => core::task::Poll::Ready(()),
            false => {
                self.yielded = true;
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            }
        }
    }
}

impl YieldFuture {
    pub fn new() -> Self {
        YieldFuture { yielded: false }
    }
}

macro_rules! interrupt_wake_channel {
    ($name:ident) => {
        pub(crate) mod $name {
            use core::cell::RefCell;
            use core::sync::atomic::AtomicUsize;
            use core::task::Waker;
            use cortex_m::interrupt::Mutex;
            use thingbuf::mpsc::{StaticChannel, StaticReceiver, StaticSender};

            type MutRefOpt<T> = Mutex<RefCell<Option<T>>>;

            pub(crate) static WAKERS: StaticChannel<Option<Waker>, 16> = StaticChannel::new();
            pub(crate) static FLAG: AtomicUsize = AtomicUsize::new(0);
            pub(crate) static SENDER: MutRefOpt<StaticSender<Option<Waker>>> =
                Mutex::new(RefCell::new(None));
            pub(crate) static RECEIVER: MutRefOpt<StaticReceiver<Option<Waker>>> =
                Mutex::new(RefCell::new(None));

            pub fn future() -> crate::futures::InterruptFuture {
                let sender = if let Some((sender, receiver)) = WAKERS.try_split() {
                    let copy = sender.clone();
                    cortex_m::interrupt::free(|cs| {
                        *SENDER.borrow(cs).borrow_mut() = Some(sender);
                        *RECEIVER.borrow(cs).borrow_mut() = Some(receiver);
                    });
                    copy
                } else {
                    cortex_m::interrupt::free(|cs| {
                        SENDER.borrow(cs).borrow().as_ref().unwrap().clone()
                    })
                };
                crate::futures::InterruptFuture {
                    flag: &FLAG,
                    sender,
                    sent_waker: core::sync::atomic::AtomicBool::new(false),
                }
            }
        }
    };
}

#[macro_export]
macro_rules! interrupt_wake {
    ($name:path) => {
        use $name as path;
        cortex_m::interrupt::free(|cs| {
            let mut receiver_ref = if let Some((sender, receiver)) = path::WAKERS.try_split() {
                *path::SENDER.borrow(cs).borrow_mut() = Some(sender);
                *path::RECEIVER.borrow(cs).borrow_mut() = Some(receiver);
                path::RECEIVER.borrow(cs).borrow_mut()
            } else {
                path::RECEIVER.borrow(cs).borrow_mut()
            };
            let receiver = receiver_ref.as_mut().unwrap();

            while let Ok(waker) = receiver.try_recv() {
                if let Some(waker) = waker {
                    path::FLAG.fetch_add(1, core::sync::atomic::Ordering::Release);
                    waker.wake();
                }
            }
        });
    };
}

interrupt_wake_channel!(usb_wake);
interrupt_wake_channel!(gps_rx_wake);
interrupt_wake_channel!(gps_tx_wake);
interrupt_wake_channel!(bmp_wake);

/// Future that is woken by an interrupt.
pub struct InterruptFuture {
    pub flag: &'static AtomicUsize,
    pub sender: StaticSender<Option<Waker>>,
    pub sent_waker: AtomicBool,
}

impl Future for InterruptFuture {
    type Output = ();

    fn poll(
        self: pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if self
            .flag
            .fetch_update(Ordering::Release, Ordering::Acquire, |x| {
                if x > 0 {
                    Some(x - 1)
                } else {
                    None
                }
            })
            .is_ok()
        {
            core::task::Poll::Ready(())
        } else if self
            .sent_waker
            .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            match self.sender.try_send(Some(cx.waker().clone())) {
                Err(TrySendError::Full(_)) => {
                    // No space, so fall back to busy waiting.
                    cx.waker().wake_by_ref();
                }
                Err(e) => {
                    panic!("Receiver dropped {:?}?", e);
                }
                Ok(_) => {
                }
            }

            core::task::Poll::Pending
        } else {
            core::task::Poll::Pending
        }
    }
}
