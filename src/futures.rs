use core::{future::Future, pin};
use usb_device::UsbError;

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
        _cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        match (self.function)() {
            Ok(value) => core::task::Poll::Ready(Ok(value)),
            Err(nb::Error::WouldBlock) => core::task::Poll::Pending,
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
        _cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        match self.yielded {
            true => core::task::Poll::Ready(()),
            false => {
                self.yielded = true;
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

pub struct UsbFuture<T, F, P>
where
    F: FnMut() -> Result<T, UsbError> + Unpin,
    P: FnMut() -> bool + Unpin,
{
    function: F,
    poll: P,
}

impl<T, F, P> Future for UsbFuture<T, F, P>
where
    F: FnMut() -> Result<T, UsbError> + Unpin,
    P: FnMut() -> bool + Unpin,
{
    type Output = Result<T, UsbError>;

    fn poll(
        mut self: pin::Pin<&mut Self>,
        _cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let s = match (self.function)() {
            Ok(value) => core::task::Poll::Ready(Ok(value)),
            Err(UsbError::WouldBlock) => {
                (self.poll)();
                core::task::Poll::Pending
            }
            Err(e) => core::task::Poll::Ready(Err(e)),
        };
        s
    }
}

impl<T, F, P> UsbFuture<T, F, P>
where
    F: FnMut() -> Result<T, UsbError> + Unpin,
    P: FnMut() -> bool + Unpin,
{
    pub fn new(function: F, poll: P) -> Self {
        UsbFuture { function, poll }
    }
}
