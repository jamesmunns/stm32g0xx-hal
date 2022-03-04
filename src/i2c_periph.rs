use crate::{rcc::Rcc, gpio::{gpiob::{PB9, PB8}, Output, OpenDrain}, timer::stopwatch::Stopwatch, time::MicroSecond};
pub use crate::stm32::i2c1::RegisterBlock as I2cRegisterBlock;
use cassette::futures::poll_fn;
use core::task::Poll;
use core::future::Future;

use crate::stm32::TIM16;

mod sealed {
    use crate::{
        stm32::{I2C1, I2C2},
        gpio::{
            gpioa::{PA11, PA12},
            gpiob::{PB6, PB7, PB8, PB9},
        },
    };

    pub trait PeriphSealed { }
    pub trait PinSealed { }

    impl PeriphSealed for I2C1 { }
    impl PeriphSealed for I2C2 { }

    impl<T> PinSealed for PA11<T> { }
    impl<T> PinSealed for PA12<T> { }

    impl<T> PinSealed for PB6<T> { }
    impl<T> PinSealed for PB7<T> { }
    impl<T> PinSealed for PB8<T> { }
    impl<T> PinSealed for PB9<T> { }
}

use crate::stm32::i2c1::RegisterBlock;
use core::ops::Deref;

#[derive(Debug, PartialEq, Eq)]
pub enum TransferDir {
    Read,
    Write,
}

pub trait Instance: Deref<Target = RegisterBlock> + sealed::PeriphSealed {
    type SDA: PinInstance + sealed::PinSealed;
    type SCL: PinInstance + sealed::PinSealed;
    fn setup(&self, rcc: &mut Rcc);
}

pub trait PinInstance: sealed::PinSealed {
    fn setup(&self);
}

impl<T> PinInstance for PA11<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

impl<T> PinInstance for PA12<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

impl<T> PinInstance for PB6<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

impl<T> PinInstance for PB7<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

impl<T> PinInstance for PB8<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

impl<T> PinInstance for PB9<T> {
    fn setup(&self) {
        self.set_alt_mode(AltFunction::AF6);
    }
}

use crate::{
    stm32::{I2C1, I2C2},
    gpio::{
        gpioa::{PA11, PA12},
        gpiob::{PB6, PB7},
        Analog,
        AltFunction,
    },
};

impl Instance for I2C2 {
    type SDA = PA12<Analog>;
    type SCL = PA11<Analog>;
    fn setup(&self, rcc: &mut Rcc) {
        // Enable clock for I2C
        rcc.rb.apbenr1.modify(|_, w| w.i2c2en().set_bit());

        // Reset I2C
        rcc.rb.apbrstr1.modify(|_, w| w.i2c2rst().set_bit());
        rcc.rb.apbrstr1.modify(|_, w| w.i2c2rst().clear_bit());
    }
}

impl Instance for I2C1 {
    type SDA = PB9<Output<OpenDrain>>;
    type SCL = PB8<Output<OpenDrain>>;
    fn setup(&self, rcc: &mut Rcc) {
        // Enable clock for I2C
        rcc.rb.apbenr1.modify(|_, w| w.i2c1en().set_bit());

        // Reset I2C
        rcc.rb.apbrstr1.modify(|_, w| w.i2c1rst().set_bit());
        rcc.rb.apbrstr1.modify(|_, w| w.i2c1rst().clear_bit());
    }
}

pub struct I2CPeripheral<P: Instance> {
    i2c: P,
    _sda: P::SDA,
    _scl: P::SCL,
    timer: Stopwatch<TIM16>,
}

pub enum AddrMatchResult {
    Waiting,
    ExpectedMatchOccurred,
    UnexpectedMatchOccurred,
}

impl<P: Instance> I2CPeripheral<P> {
    pub fn new(
        i2c: P,
        sda: P::SDA,
        scl: P::SCL,
        rcc: &mut Rcc,
        address: u8,
        mut timer: Stopwatch<TIM16>,
    ) -> Self {
        // TODO: Why doesn't this return new types?
        sda.setup();
        scl.setup();

        i2c.setup(rcc);

        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // // Setup protocol timings
        // let timing_bits = config.timing_bits(rcc.clocks.apb_clk);
        // TODO(AJM): Number derived from stm32cubemx!
        i2c.timingr.write(|w| unsafe { w.bits(0x10707DBC) });


        // * initial settings
        // * Clear {OA1EN, OA2EN} in I2C_OAR1 and I2C_OAR2
        i2c.oar1.modify(|_, w| {
            w.oa1en().clear_bit()
        });
        i2c.oar2.modify(|_, w| {
            w.oa2en().clear_bit()
        });

        timer.pause();
        timer.reset();

        // 64Mhz -> 1Mhz
        timer.set_prescaler(63);

        // * Configure:
        //     * OA1[9:0]
        //     * OA1MODE
        //     * OA1EN
        i2c.oar1.modify(|_, w| {
            unsafe { w.oa1_7_1().bits(address) }
            .oa1mode().clear_bit() // 7 bit address
            .oa1en().set_bit()
        });


        //     * OA2[6:0]
        //     * OA2MSK[2:0]
        //     * OA2EN
        // TODO(AJM): OA2 handling

        //     * GCEN (CR1)
        // * Configure SBC in I2C_CR1
        // * Enable interrupts and/or DMA in I2C_CR1
        //     * lol later

        // Enable the I2C processing
        i2c.cr1.modify(|_, w| unsafe {
            w.pe()
                .set_bit()
                .dnf()
                // .bits(config.digital_filter)
                .bits(0) // TODO(AJM)
                .anfoff()
                // .bit(!config.analog_filter)
                .bit(false) // TODO(AJM)
                .sbc()
                .clear_bit() // TODO(AJM)
                .gcen()
                .clear_bit()
        });
        Self { i2c, _sda: sda, _scl: scl, timer }
    }

    // TODO(AJM): Remove before release
    #[inline(always)]
    pub fn borrow_pac(&self) -> &I2cRegisterBlock {
        &self.i2c
    }

    /// Check if we have been addressed with a certain transfer direction.
    ///
    /// If we have been addressed with an UNEXPECTED direction, a nak will
    /// be sent.
    ///
    /// Returns true if we were addressed with the expected direction
    /// Returns false if we were NOT addressed, or addressed with an unexpected direction
    pub fn check_addr_match(&self, direction: TransferDir) -> AddrMatchResult {
        let i2cpac = self.borrow_pac();

        if !i2cpac.isr.read().addr().bit_is_set() {
            return AddrMatchResult::Waiting;
        }

        // TODO(AJM): re-enable this when we support registering more than
        // one address!
        //
        // if i2cpac.isr.read().addcode().bits() != self.current_i2c_addr {
        //     self.nak();
        //     self.clear_addr_match();
        //     sprkt_log!(error, "Address Mismatch!");
        //     return false;
        // }

        // 0: Write transfer, slave enters receiver mode.
        // 1: Read transfer, slave enters transmitter mode.
        let dir = if i2cpac.isr.read().dir().bit_is_set() {
            TransferDir::Read
        } else {
            TransferDir::Write
        };

        // Is this in the direction we expected?
        if dir != direction {
            self.nak();
            self.clear_addr_match();
            // sprkt_log!(error, "Direction Mismatch!");
            AddrMatchResult::UnexpectedMatchOccurred
        } else {
            self.ack();

            // sprkt_log!(info, "Acked a correct address+direction");
            if direction == TransferDir::Read {
                i2cpac.isr.write(|w| w.txe().set_bit());
            }
            self.clear_addr_match();
            AddrMatchResult::ExpectedMatchOccurred
        }
    }

    pub fn ack(&self) {
        let i2cpac = self.borrow_pac();

        // Command an ACK
        i2cpac.cr2.write(|w| w.nack().clear_bit());
    }

    pub fn nak(&self) {
        let i2cpac = self.borrow_pac();

        // Command a NAK
        i2cpac.cr2.write(|w| w.nack().set_bit());
    }

    pub fn clear_addr_match(&self) {
        let i2cpac = self.borrow_pac();

        // Clear the ADDR match flag
        i2cpac.icr.write(|w| w.addrcf().set_bit());
    }

    pub fn is_enabled(&self) -> bool {
        self.i2c.cr1.read().pe().bit_is_set()
    }

    /// Checks whether the stop bit was set. DOES NOT clear the flag!
    pub fn is_controller_stopped(&self) -> bool {
        let i2cpac = self.borrow_pac();

        // Is the controller done?
        if i2cpac.isr.read().stopf().bit_is_set() {
            true
        } else {
            false
        }
    }

    pub fn clear_stop_flag(&self) {
        let i2cpac = self.borrow_pac();

        // Is the controller done?
        if i2cpac.isr.read().stopf().bit_is_set() {
            i2cpac.icr.write(|w| w.stopcf().set_bit());
        }
    }

    /// Try to receive a single byte sent from the controller.
    pub fn get_one_byte(&self) -> Option<u8> {
        let i2cpac = self.borrow_pac();

        if i2cpac.isr.read().rxne().bit_is_set() {
            Some(i2cpac.rxdr.read().rxdata().bits())
        } else {
            None
        }
    }

    /// Send a single read byte. Returns true if it was sent successfully
    pub fn send_one_byte(&self, byte: u8) -> bool {
        let i2cpac = self.borrow_pac();

        if i2cpac.isr.read().txis().bit_is_set() {
            i2cpac
                .txdr
                .modify(|_, w| unsafe {
                    w.txdata().bits(byte)
                });
            true
        } else {
            false
        }
    }

    pub fn got_nack(&self) -> bool {
        let i2cpac = self.borrow_pac();

        if i2cpac.isr.read().nackf().bit_is_set() {
            i2cpac.icr.write(|w| w.nackcf().set_bit());
            true
        } else {
            false
        }
    }

    pub unsafe fn disable(&self) {
        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());
    }

    pub unsafe fn enable(&self) {
        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }
}

// These are all the future functions
impl<P: Instance> I2CPeripheral<P> {
    pub fn match_address_write(&mut self) -> impl Future<Output=Result<(), ()>> + '_ {
        poll_fn(move |_| {
            match self.check_addr_match(TransferDir::Write) {
                AddrMatchResult::ExpectedMatchOccurred => Poll::Ready(Ok(())),
                AddrMatchResult::UnexpectedMatchOccurred => Poll::Ready(Err(())),
                AddrMatchResult::Waiting => Poll::Pending
            }
        })
    }

    pub fn match_address_read(&mut self) -> impl Future<Output=Result<(), ()>> + '_ {
        poll_fn(move |_| {
            match self.check_addr_match(TransferDir::Read) {
                AddrMatchResult::ExpectedMatchOccurred => Poll::Ready(Ok(())),
                AddrMatchResult::UnexpectedMatchOccurred => Poll::Ready(Err(())),
                AddrMatchResult::Waiting => Poll::Pending
            }
        })
    }

    /// Recieves an incoming byte sent by the controller
    ///
    /// Returns an Ok(()) if the byte was received successfully, and an Err(()) if
    /// the controller terminated early
    pub fn get_written_byte(&self) -> impl Future<Output=Result<u8, ()>> + '_ {
        poll_fn(move |_| {
            if self.got_nack() {
                return Poll::Ready(Err(()));
            }
            match self.get_one_byte() {
                Some(byte) => Poll::Ready(Ok(byte)),
                None => {
                    if self.is_controller_stopped() {
                        Poll::Ready(Err(()))
                    } else {
                        Poll::Pending
                    }
                }
            }
        })
    }

    /// Send an outgoing byte requested by the controller
    ///
    /// Returns an Ok(()) if the byte was sent successfully, and an Err(()) if
    /// the controller terminated early
    pub fn send_read_byte(&mut self, data: u8) -> impl Future<Output=Result<(), ()>> + '_ {
        poll_fn(move |_| {
            if self.got_nack() {
                return Poll::Ready(Err(()));
            }
            if self.send_one_byte(data) {
                Poll::Ready(Ok(()))
            } else if self.is_controller_stopped() {
                Poll::Ready(Err(()))
            } else {
                Poll::Pending
            }
        })
    }

    /// Wait for the controller to "wrap it up".
    ///
    /// Will discard any incoming data, and send `0x00` bogus bytes on the wire.
    /// all data bytes will be NAK'd
    pub fn wait_for_stop(&mut self) -> impl Future<Output=()> + '_ {
        // self.timer.pause();
        // self.timer.reset();
        // self.timer.resume();
        // let start = self.timer.now();

        poll_fn(move |_| {
            // let i2cpac = self.borrow_pac();

            // if self.timer.elapsed(start) >= MicroSecond(3_000) {
            //     // Reset state machine
            //     i2cpac.cr1.modify(|_, w| w.pe().clear_bit());
            //     while i2cpac.cr1.read().pe().bit_is_set() {}
            //     i2cpac.cr1.modify(|_, w| w.pe().set_bit());

            //     return Poll::Ready(());
            // }

            let i2cpac = self.borrow_pac();

            let isr = i2cpac.isr.read();

            // Is the controller asking us for more data still?
            if isr.txis().bit_is_set() {
                self.nak();
                // sprkt_log!(error, "asked for more read when stop expected!");
                i2cpac.txdr.modify(|_, w| unsafe { w.txdata().bits(0) });
            }

            // Is the controller giving us more data still?
            if isr.rxne().bit_is_clear() {
                self.nak();
                // sprkt_log!(error, "got write when stop expected!");
                let _ = i2cpac.rxdr.read().rxdata().bits();
            }

            // Is the controller finally done?
            if self.is_controller_stopped() {
                self.clear_stop_flag();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    }
}
