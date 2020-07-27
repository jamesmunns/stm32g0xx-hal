use core::fmt;
use core::marker::PhantomData;
use core::ops;
use core::pin::Pin;
use core::sync::atomic::{self, Ordering};

use crate::dma::{DmaChannel, ReadDma, Transfer, TransferDirection, WriteDma};
use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiod::*};
use crate::gpio::{AltFunction, DefaultMode};
use crate::prelude::*;
use crate::rcc::Rcc;
use crate::stm32::*;
use crate::time::Bps;
use as_slice::{AsMutSlice, AsSlice};
use nb::block;

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum WordLength {
    DataBits7,
    DataBits8,
    DataBits9,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

/// Interrupt event
pub enum Event {
    /// TXFIFO reaches the threshold
    TXFT = 1 << 27,
    /// This bit is set by hardware when the threshold programmed in RXFTCFG in USART_CR3 register is reached.
    RXFT = 1 << 26,

    /// RXFIFO full
    RXFF = 1 << 24,
    /// TXFIFO empty
    TXFE = 1 << 23,

    /// Active when a communication is ongoing on the RX line
    BUSY = 1 << 16,

    /// Receiver timeout.This bit is set by hardware when the timeout value,
    /// programmed in the RTOR register has lapsed, without any communication.
    RTOF = 1 << 11,
    /// Transmit data register empty. New data can be sent
    Txe = 1 << 7,

    /// Transmission Complete. The last data written in the USART_TDR has been transmitted out of the shift register.
    TC = 1 << 6,
    /// New data has been received
    Rxne = 1 << 5,
    /// Idle line state detected
    Idle = 1 << 4,

    /// Overrun error
    ORE = 1 << 3,

    /// Noise detection flag
    NE = 1 << 2,

    /// Framing error
    FE = 1 << 1,

    /// Parity error
    PE = 1 << 0,
}
impl Event {
    fn val(self) -> u32 {
        self as u32
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0b00,
    #[doc = "0.5 stop bits"]
    STOP0P5 = 0b01,
    #[doc = "2 stop bits"]
    STOP2 = 0b10,
    #[doc = "1.5 stop bits"]
    STOP1P5 = 0b11,
}

impl StopBits {
    fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub enum FifoThreshold {
    #[doc = "1/8 of its depth"]
    _1_BYTE = 0b000,
    #[doc = "1/4 of its depth"]
    _2_BYTES = 0b001,
    #[doc = "1/2 of its depth"]
    _4_BYTES = 0b010,
    #[doc = "3/4 of its depth"]
    _6_BYTES = 0b011,
    #[doc = "7/8 of its depth"]
    _7_BYTES = 0b100,
    #[doc = "fifo empty/full"]
    _8_BYTES = 0b101,
}

impl FifoThreshold {
    fn bits(self) -> u8 {
        self as u8
    }
}
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct Config {
    baudrate: Bps,
    wordlength: WordLength,
    parity: Parity,
    stopbits: StopBits,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct FullConfig {
    baudrate: Bps,
    wordlength: WordLength,
    parity: Parity,
    stopbits: StopBits,
    fifo_enable: bool,
    tx_fifo_threshold: FifoThreshold,
    rx_fifo_threshold: FifoThreshold,
    tx_fifo_interrupt: bool,
    rx_fifo_interrupt: bool,
    #[doc = "Number of bits no activity on rx line"]
    receiver_timeout: Option<u32>
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }

    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl FullConfig {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }

    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    pub fn fifo_enable(mut self) -> Self {
        self.fifo_enable = true;
        self
    }

    pub fn tx_fifo_threshold(mut self, threshold: FifoThreshold) -> Self {
        self.tx_fifo_threshold = threshold;
        self
    }

    pub fn rx_fifo_threshold(mut self, threshold: FifoThreshold) -> Self {
        self.rx_fifo_threshold = threshold;
        self
    }

    pub fn tx_fifo_enable_interrupt(mut self) -> Self {
        self.tx_fifo_interrupt = true;
        self
    }

    pub fn rx_fifo_enable_interrupt(mut self) -> Self {
        self.rx_fifo_interrupt = true;
        self
    }

    /// Configure receiver timout in microseconds. Call after baudrate is set.
    pub fn receiver_timeout_us(mut self, timeout_us: u32) -> Self {
        let t = timeout_us as u64 * self.baudrate.0 as u64 / 1_000_000u64;
        self.receiver_timeout = Some(t as u32);
        self
    }

}

#[derive(Debug)]
pub struct InvalidConfig;

impl Default for Config {
    fn default() -> Config {
        let baudrate = 19_200.bps();
        Config {
            baudrate,
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

impl Default for FullConfig {
    fn default() -> FullConfig {
        let baudrate = 115_200.bps();
        FullConfig {
            baudrate,
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            fifo_enable: false,
            tx_fifo_threshold: FifoThreshold::_8_BYTES,
            rx_fifo_threshold: FifoThreshold::_8_BYTES,
            tx_fifo_interrupt: false,
            rx_fifo_interrupt: false,
            receiver_timeout: None
        }
    }
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial DMA receiver
pub struct DmaRx<USART, CHANNEL> {
    _usart: PhantomData<USART>,
    channel: CHANNEL,
}

/// Serial DMA transmitter
pub struct DmaTx<USART, CHANNEL> {
    _usart: PhantomData<USART>,
    channel: CHANNEL,
}

/// Serial abstraction
pub struct Serial<USART> {
    tx: Tx<USART>,
    rx: Rx<USART>,
    usart: USART
}

pub trait SerialExt<USART> {
    fn usart<TX, RX>(
        self,
        tx: TX,
        rx: RX,
        config: Config,
        rcc: &mut Rcc,
    ) -> Result<Serial<USART>, InvalidConfig>
    where
        TX: TxPin<USART>,
        RX: RxPin<USART>;
}

// Serial TX pin
pub trait TxPin<USART> {
    fn setup(&self);
}

// Serial RX pin
pub trait RxPin<USART> {
    fn setup(&self);
}

impl<USART> fmt::Write for Serial<USART>
where
    Serial<USART>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}

macro_rules! uart {
    ($USARTX:ident,
        $usartX:ident, $apbXenr:ident, $usartXen:ident, $clk_mul:expr,
        tx: [ $(($PTX:ty, $TAF:expr),)+ ],
        rx: [ $(($PRX:ty, $RAF:expr),)+ ],
    ) => {
        $(
            impl TxPin<$USARTX> for $PTX {
                fn setup(&self) {
                    self.set_alt_mode($TAF)
                }
            }
        )+

        $(
            impl RxPin<$USARTX> for $PRX {
                fn setup(&self) {
                    self.set_alt_mode($RAF)
                }
            }
        )+

        impl SerialExt<$USARTX> for $USARTX {
            fn usart<TX, RX>(
                self,
                tx: TX,
                rx: RX,
                config: Config,
                rcc: &mut Rcc) -> Result<Serial<$USARTX>, InvalidConfig>
            where
                TX: TxPin<$USARTX>,
                RX: RxPin<$USARTX>,
            {
                Serial::$usartX(self, tx, rx, config, rcc)
            }
        }

        impl Serial<$USARTX> {
            pub fn $usartX<TX, RX>(
                usart: $USARTX,
                tx: TX,
                rx: RX,
                config: Config,
                rcc: &mut Rcc,
            ) -> Result<Self, InvalidConfig>
            where
                TX: TxPin<$USARTX>,
                RX: RxPin<$USARTX>,
            {
                tx.setup();
                rx.setup();

                // Enable clock for USART
                rcc.rb.$apbXenr.modify(|_, w| w.$usartXen().set_bit());
                let clk = rcc.clocks.apb_clk.0 as u64;
                let bdr = config.baudrate.0 as u64;
                let div = ($clk_mul * clk) / bdr;
                usart
                    .brr
                    .write(|w| unsafe { w.bits(div as u32) });
                // Reset other registers to disable advanced USART features
                usart.cr2.reset();
                usart.cr3.reset();

                // Enable transmission and receiving
                usart.cr1.write(|w| {
                    w.ue()
                        .set_bit()
                        .te()
                        .set_bit()
                        .re()
                        .set_bit()
                        .m0()
                        .bit(config.wordlength == WordLength::DataBits7)
                        .m1()
                        .bit(config.wordlength == WordLength::DataBits9)
                        .pce()
                        .bit(config.parity != Parity::ParityNone)
                        .ps()
                        .bit(config.parity == Parity::ParityOdd)
                });
                usart.cr2.write(|w| unsafe {
                    w.stop().bits(match config.stopbits {
                        StopBits::STOP1 => 0b00,
                        StopBits::STOP0P5 => 0b01,
                        StopBits::STOP2 => 0b10,
                        StopBits::STOP1P5 => 0b11,
                    })
                });
                Ok(Serial {
                    tx: Tx { _usart: PhantomData },
                    rx: Rx { _usart: PhantomData },
                    usart
                })
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                    Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                    Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
                    _ => {},
                }
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self, event: Event) {
                match event {
                    Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                    Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                    Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
                    _ => {},
                }
            }

            /// Check if interrupt event is pending
            pub fn is_pending(&mut self, event: Event) -> bool {
                (self.usart.isr.read().bits() & event.val()) != 0
            }

            /// Clear pending interrupt
            pub fn unpend(&mut self, event: Event) {
                // mask the allowed bits
                let mask: u32 = 0x123BFF;
                self.usart.icr.write(|w| unsafe {
                    w.bits(event.val() & mask)
                });
            }


            // pub fn is_pending(&mut self, event: Event) {
            //     self.usart.isr().read

            // }

            /// Separates the serial struct into separate channel objects for sending (Tx) and
            /// receiving (Rx)
            pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                (self.tx, self.rx)
            }
        }

        impl Tx<$USARTX> {
            pub fn with_dma<CHANNEL>(self, channel: CHANNEL) -> DmaTx<$USARTX, CHANNEL>
                where CHANNEL: DmaChannel
            {
                let usart = unsafe { &(*$USARTX::ptr()) };
                let usart_ptr = &usart.tdr as *const _ as _;

                let mut channel = channel;
                channel.set_direction(TransferDirection::MemoryToPeriph);
                channel.set_peripheral_address(usart_ptr, false);
                DmaTx {
                    channel,
                    _usart: PhantomData,
                }
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1.modify(|_, w| w.txeie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1.modify(|_, w| w.txeie().clear_bit());
            }

        }

        impl Rx<$USARTX> {
            pub fn with_dma<CHANNEL>(self, channel: CHANNEL) -> DmaRx<$USARTX, CHANNEL>
                where CHANNEL: DmaChannel
            {
                let usart = unsafe { &(*$USARTX::ptr()) };
                let usart_ptr = &usart.rdr as *const _ as _;

                let mut channel = channel;
                channel.set_direction(TransferDirection::PeriphToMemory);
                channel.set_peripheral_address(usart_ptr, false);
                DmaRx {
                    channel,
                    _usart: PhantomData,
                }
            }

            /// Starts listening for an interrupt event
            pub fn listen(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1.modify(|_, w| w.rxneie().set_bit());
            }

            /// Stop listening for an interrupt event
            pub fn unlisten(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.cr1.modify(|_, w| w.rxneie().clear_bit());
            }

            /// Check if receiver timeout has lapsed
            /// Returns the current state of the ISR RTOF bit
            pub fn timeout_lapsed(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr.read().rtof().bit_is_set()
            }

            /// Clear pending receiver timeout interrupt
            pub fn clear_timeout(&mut self) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.icr.write(|w| w.rtocf().set_bit());
            }

            /// Check if overrun error has occurred
            pub fn overrun_error(&self) -> bool {
                let usart = unsafe { &(*$USARTX::ptr()) };
                usart.isr.read().ore().bit_is_set()
            }

            pub fn clear_error(&mut self, event: Event) {
                let usart = unsafe { &(*$USARTX::ptr()) };
                match event {
                    Event::ORE => usart.icr.write(|w| w.orecf().set_bit()),
                    Event::NE => usart.icr.write(|w| w.ncf().set_bit()),
                    Event::FE => usart.icr.write(|w| w.fecf().set_bit()),
                    Event::PE => usart.icr.write(|w| w.pecf().set_bit()),
                    _ => {},
                }
            }

        }

        impl<CHANNEL, B> ReadDma<B> for DmaRx<$USARTX, CHANNEL>
        where
            CHANNEL: DmaChannel,
            B: ops::DerefMut + 'static,
            B::Target: AsMutSlice<Element = u8> + Unpin,
            Self: core::marker::Sized,
        {
            fn read(mut self, buffer: Pin<B>) -> Transfer<Self, Pin<B>> {
                let mut buffer = buffer;
                let slice = buffer.as_mut_slice();
                let (ptr, len) = (slice.as_ptr(), slice.len());

                let dma_channel = &mut self.channel;
                dma_channel.set_memory_address(ptr as u32, true);
                dma_channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::SeqCst);
                dma_channel.start();

                Transfer {
                    buffer,
                    channel: self,
                }
            }
        }

        impl<CHANNEL, B> WriteDma<B> for DmaTx<$USARTX, CHANNEL>
        where
            CHANNEL: DmaChannel,
            B: ops::Deref + 'static,
            B::Target: AsSlice<Element = u8> + Unpin,
            Self: core::marker::Sized,
        {
            fn write(mut self, buffer: Pin<B>) -> Transfer<Self, Pin<B>> {
                let slice = buffer.as_slice();
                let (ptr, len) = (slice.as_ptr(), slice.len());

                let dma_channel = &mut self.channel;
                dma_channel.set_memory_address(ptr as u32, true);
                dma_channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::SeqCst);
                dma_channel.start();

                Transfer {
                    buffer,
                    channel: self,
                }
            }
        }

        impl hal::serial::Read<u8> for Rx<$USARTX> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                let isr = usart.isr.read();
                Err(
                    if isr.pe().bit_is_set() {
                        usart.icr.write(|w| w.pecf().set_bit());
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        usart.icr.write(|w| w.fecf().set_bit());
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        usart.icr.write(|w| w.ncf().set_bit());
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        usart.icr.write(|w| w.orecf().set_bit());
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        return Ok(usart.rdr.read().bits() as u8)
                    } else {
                        nb::Error::WouldBlock
                    }
                )
            }
        }

        impl hal::serial::Read<u8> for Serial<$USARTX> {
            type Error = Error;

            fn read(&mut self) -> nb::Result<u8, Error> {
                self.rx.read()
            }
        }

        impl hal::serial::Write<u8> for Tx<$USARTX> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                if usart.isr.read().tc().bit_is_set() {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                let usart = unsafe { &(*$USARTX::ptr()) };
                if usart.isr.read().txe().bit_is_set() {
                    usart.tdr.write(|w| unsafe { w.bits(byte as u32) });
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl hal::serial::Write<u8> for Serial<$USARTX> {
            type Error = Error;

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                self.tx.flush()
            }

            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.tx.write(byte)
            }
        }
    }
}

uart!(
    USART1, usart1, apbenr2, usart1en, 1,
    tx: [
        (PA9<DefaultMode>, AltFunction::AF1),
        (PB6<DefaultMode>, AltFunction::AF0),
        (PC4<DefaultMode>, AltFunction::AF1),
    ],
    rx: [
        (PA10<DefaultMode>, AltFunction::AF1),
        (PB7<DefaultMode>, AltFunction::AF0),
        (PC5<DefaultMode>, AltFunction::AF1),
    ],
);

uart!(
    USART2, usart2, apbenr1, usart2en, 1,
    tx: [
        (PA2<DefaultMode>, AltFunction::AF1),
        (PA14<DefaultMode>, AltFunction::AF1),
        (PD5<DefaultMode>, AltFunction::AF0),
    ],
    rx: [
        (PA3<DefaultMode>, AltFunction::AF1),
        (PA15<DefaultMode>, AltFunction::AF1),
        (PD6<DefaultMode>, AltFunction::AF0),
    ],
);

#[cfg(any(feature = "stm32g070",feature = "stm32g071", feature = "stm32g081"))]
uart!(
    USART3, usart3, apbenr1, usart3en, 1,
    tx: [
        (PA5<DefaultMode>, AltFunction::AF4),
        (PB2<DefaultMode>, AltFunction::AF4),
        (PB8<DefaultMode>, AltFunction::AF4),
        (PB10<DefaultMode>, AltFunction::AF4),
        (PC4<DefaultMode>, AltFunction::AF1),
        (PC10<DefaultMode>, AltFunction::AF1),
        (PD8<DefaultMode>, AltFunction::AF1),
    ],
    rx: [
        (PB0<DefaultMode>, AltFunction::AF4),
        (PB9<DefaultMode>, AltFunction::AF4),
        (PB11<DefaultMode>, AltFunction::AF4),
        (PC5<DefaultMode>, AltFunction::AF1),
        (PC11<DefaultMode>, AltFunction::AF1),
        (PD9<DefaultMode>, AltFunction::AF1),
    ],
);

#[cfg(any(feature = "stm32g070",feature = "stm32g071", feature = "stm32g081"))]
uart!(
    USART4, usart4, apbenr1, usart4en, 1,
    tx: [
        (PA0<DefaultMode>, AltFunction::AF4),
        (PC10<DefaultMode>, AltFunction::AF1),
    ],
    rx: [
        (PC11<DefaultMode>, AltFunction::AF1),
        (PA1<DefaultMode>, AltFunction::AF4),
    ],
);

// #[cfg(feature = "stm32g0x1")]
// uart!(
//     LPUART, lpuart, apbenr1, lpuart1en, 256,
//     tx: [
//         (PA2<DefaultMode>, AltFunction::AF6),
//         (PB11<DefaultMode>, AltFunction::AF1),
//         (PC1<DefaultMode>, AltFunction::AF1),
//     ],
//     rx: [
//         (PA3<DefaultMode>, AltFunction::AF6),
//         (PB10<DefaultMode>, AltFunction::AF1),
//         (PC0<DefaultMode>, AltFunction::AF1),
//     ],
// );


/// This traits unlocks full functionality of the USART peripheral
pub trait FullSerial {

    fn usart_full<TX, RX>(
        usart: USART1,
        tx: TX,
        rx: RX,
        config: FullConfig,
        rcc: &mut Rcc,
    ) -> Result<Serial<USART1>, InvalidConfig>
    where
        TX: TxPin<USART1>,
        RX: RxPin<USART1>;
}

impl FullSerial for Serial<USART1> {

    fn usart_full<TX, RX>(
        usart: USART1,
        tx: TX,
        rx: RX,
        config: FullConfig,
        rcc: &mut Rcc,
    ) -> Result<Self, InvalidConfig> where
        TX: TxPin<USART1>,
        RX: RxPin<USART1>
    {
        tx.setup();
        rx.setup();

        // Enable clock for USART
        // rcc.rb.$apbXenr.modify(|_, w| w.$usartXen().set_bit());
        rcc.rb.apbenr2.modify(|_, w| w.usart1en().set_bit());

        let clk = rcc.clocks.apb_clk.0 as u64;
        let bdr = config.baudrate.0 as u64;
        let clk_mul = 1;
        let div = (clk_mul * clk) / bdr;
        usart.brr.write(|w| unsafe { w.bits(div as u32) });

        usart.cr1.reset();
        usart.cr2.reset();
        usart.cr3.reset();


        usart.cr2.write(|w| unsafe {
            w.stop().bits(config.stopbits.bits())
        });

        if let Some(timeout) = config.receiver_timeout {
            usart.cr1.write(|w| w.rtoie().set_bit());
            usart.cr2.modify(|_, w| w.rtoen().set_bit());
            usart.rtor.write(|w| unsafe { w.rto().bits(timeout) });
        }

        usart.cr3.write(|w| unsafe {
            w.txftcfg().bits(config.tx_fifo_threshold.bits())
            .rxftcfg().bits(config.rx_fifo_threshold.bits())
            .txftie().bit(config.tx_fifo_interrupt)
            .rxftie().bit(config.rx_fifo_interrupt)
        });

        usart.cr1.modify(|_, w| {
            w.ue()
            .set_bit()
            .te()
            .set_bit()
            .re()
            .set_bit()
            .m0()
            .bit(config.wordlength == WordLength::DataBits7)
            .m1()
            .bit(config.wordlength == WordLength::DataBits9)
            .pce()
            .bit(config.parity != Parity::ParityNone)
            .ps()
            .bit(config.parity == Parity::ParityOdd)
            .fifoen()
            .bit(config.fifo_enable)
        });


        Ok(Serial {
            tx: Tx { _usart: PhantomData },
            rx: Rx { _usart: PhantomData },
            usart
        })


    }

    // fn is_pending(&mut self, event: Event) -> bool {
    //     (self.usart.isr.read().bits() & event.val()) != 0
    // }

    // fn unpend(&mut self, event: Event) {
    //     // mask the allowed bits
    //     let mask: u32 = 0x123BFF;
    //     self.usart.icr.write(|w| unsafe {
    //         w.bits(event.val() & mask)
    //     });
    // }



}