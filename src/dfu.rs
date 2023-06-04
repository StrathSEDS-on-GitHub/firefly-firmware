use stm32f4xx_hal::{flash::FlashExt, pac::FLASH};

#[link_section = ".ota_update"]
static SECTOR_SIZES: [u32; 11] = [
    16 * 1024,
    16 * 1024,
    16 * 1024,
    16 * 1024,
    64 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
];

#[link_section = ".ota_update_begin"]
pub fn begin_update(firmware: &[u8], mut internal_flash: FLASH) {
    let mut firmware = firmware;

    unlock_flash(&mut internal_flash);

    let mut sector_idx = 0;
    let mut bytes_written = 0;
    while bytes_written < firmware.len() {
        let sector_size = SECTOR_SIZES[sector_idx as usize];
        erase(&mut internal_flash, sector_idx);
        let bytes_to_write = core::cmp::min(firmware.len() - bytes_written, sector_size as usize);
        let (firmware_to_write, remaining_firmware) = firmware.split_at(bytes_to_write);
        for chunk in firmware_to_write.chunks(128 / 8) {
            let address = bytes_written;
            if chunk.len() < 128 / 8 {
                let mut padded_chunk = [0; 128 / 8];
                padded_chunk[..chunk.len()].copy_from_slice(chunk);
                program(&mut internal_flash, address, padded_chunk.iter());
            } else {
                program(&mut internal_flash, address, chunk.iter());
            }
            bytes_written += chunk.len();
        }
        firmware = remaining_firmware;
        sector_idx += 1;
    }

    lock(&internal_flash);

    // Reset the device
    cortex_m::peripheral::SCB::sys_reset();
}

#[link_section = ".ota_update"]
fn unlock_flash(flash: &mut FLASH) {
    const UNLOCK_KEY1: u32 = 0x45670123;
    const UNLOCK_KEY2: u32 = 0xCDEF89AB;
    flash.keyr.write(|w| w.key().bits(UNLOCK_KEY1));
    flash.keyr.write(|w| w.key().bits(UNLOCK_KEY2));
}

#[link_section = ".ota_update"]
fn erase(flash: &mut FLASH, sector: u8) {
    let snb = if sector < 12 { sector } else { sector + 4 };
    const PSIZE_X8: u8 = 0b00;

    #[rustfmt::skip]
    flash.cr.modify(|_, w| unsafe {
        w
            // start
            .strt().set_bit()
            .psize().bits(PSIZE_X8)
            // sector number
            .snb().bits(snb)
            // sectore erase
            .ser().set_bit()
            // no programming
            .pg().clear_bit()
    });
    wait_ready(flash);
}

#[link_section = ".ota_update"]
fn wait_ready(flash: &mut FLASH) {
    while flash.sr.read().bsy().bit() {}
}

#[link_section = ".ota_update"]
fn program<'a, I>(flash: &mut FLASH, mut offset: usize, mut bytes: I)
where
    I: Iterator<Item = &'a u8>,
{
    const PSIZE_X8: u8 = 0b00;
    let ptr = 0x0800_0000 as *mut u8;
    let mut bytes_written = 1;
    while bytes_written > 0 {
        bytes_written = 0;
        let amount = 16 - (offset % 16);

        #[rustfmt::skip]
        flash.cr.modify(|_, w| 
            w
                .psize().bits(PSIZE_X8)
                // no sector erase
                .ser().clear_bit()
                // programming
                .pg().set_bit()
        );
        for _ in 0..amount {
            match bytes.next() {
                Some(byte) => {
                    unsafe {
                        core::ptr::write_volatile(ptr.add(offset), *byte);
                    }
                    offset += 1;
                    bytes_written += 1;
                }
                None => break,
            }
        }
        wait_ready(flash);
    }
    flash.cr.modify(|_, w| w.pg().clear_bit());
}

#[link_section = ".ota_update"]
fn lock(flash: &FLASH) {
    flash.cr.modify(|_, w| w.lock().set_bit());
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

    pub async fn radio_tx_ota(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
        firmware: &[u8],
    ) {
        let message = Message::OtaBegin {
            size: firmware.len() as u32,
        };
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
            timer.start(3u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
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
                timer.start(3u32.millis()).unwrap();
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
            timer.start(3u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }

    pub async fn radio_rx_ota<PINS>(
        &mut self,
        mut timer: CounterMs<impl stm32f4xx_hal::timer::Instance>,
        flash: &mut W25Q<PINS>,
        neopixel: &mut impl SmartLedsWrite<Color = RGB8, Error = impl core::fmt::Debug>,
    ) -> u32
    where
        PINS: QspiPins,
    {
        let mut packets_recvd = 0;
        let mut max_packets = 0u32;
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
                    let mut packet = [0u8; 128 + 8];
                    self.radio
                        .read_buffer(
                            &mut self.spi,
                            &mut self.delay,
                            rx_buf_status.rx_start_buffer_pointer(),
                            &mut packet,
                        )
                        .unwrap();

                    let packet = Message::parse(&packet).or_else(|| {
                        hprintln!("bp");
                        None
                    });
                    if let Some(packet) = packet {
                        match packet {
                            Message::Ota(start_addr, data) => {
                                let start: u32 = 4096 + start_addr * 128;
                                if start % 4096 == 0 {
                                    flash
                                        .erase_sector(SectorAddress::from_address(start))
                                        .unwrap();
                                }
                                let rgb = RGB::new(0, (packets_recvd * 255 / max_packets) as u8, 0);

                                let rgb2 = RGB::new((packets_recvd % 255) as u8, 0, 0);
                                neopixel.write([rgb, rgb2].into_iter()).unwrap();

                                packets_recvd += 1;
                                flash.wait_on_busy().unwrap();
                                flash.program_page(start.into(), &data).unwrap();
                            }
                            Message::OtaDone => {
                                hprintln!("pcks: {}", packets_recvd);
                                return packets_recvd;
                            }
                            Message::OtaBegin { size } => {
                                // TODO: check size
                                max_packets = size / 128;
                                hprintln!("{:x}", size);
                            }
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
            timer.start(3u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }

}