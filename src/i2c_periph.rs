use crate::rcc::Rcc;
pub use crate::stm32::i2c1::RegisterBlock as I2cRegisterBlock;


mod sealed {
    use crate::{
        stm32::{I2C1, I2C2},
        gpio::{
            gpioa::{PA11, PA12},
            gpiob::{PB6, PB7},
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
}

use crate::stm32::i2c1::RegisterBlock;
use core::ops::Deref;
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
    type SDA = PB7<Analog>;
    type SCL = PB6<Analog>;
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
}

impl<P: Instance> I2CPeripheral<P> {
    pub fn new(
        i2c: P,
        sda: P::SDA,
        scl: P::SCL,
        rcc: &mut Rcc,
        address: u8,
    ) -> Self {
        // TODO: Why doesn't this return new types?
        sda.setup();
        scl.setup();

        i2c.setup(rcc);

        // Make sure the I2C unit is disabled so we can configure it
        i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // // Setup protocol timings
        // let timing_bits = config.timing_bits(rcc.clocks.apb_clk);
        // i2c.timingr.write(|w| unsafe { w.bits(timing_bits) });


        // * initial settings
        // * Clear {OA1EN, OA2EN} in I2C_OAR1 and I2C_OAR2
        i2c.oar1.modify(|_, w| {
            w.oa1en().clear_bit()
        });
        i2c.oar2.modify(|_, w| {
            w.oa2en().clear_bit()
        });

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
        Self { i2c, _sda: sda, _scl: scl }
    }

    // TODO(AJM): Remove before release
    #[inline(always)]
    pub fn borrow_pac(&self) -> &I2cRegisterBlock {
        &self.i2c
    }

    pub fn is_enabled(&self) -> bool {
        self.i2c.cr1.read().pe().bit_is_set()
    }

    pub unsafe fn disable(&self) {
        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());
    }

    pub unsafe fn enable(&self) {
        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }
}
