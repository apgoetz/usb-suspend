//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

extern crate panic_halt;

// device specific imports
use stm32l0xx_hal::usb::{UsbBus, USB};
use stm32l0xx_hal::{pac, prelude::*, rcc, serial, syscfg::SYSCFG};
use stm32l0xx_hal::serial::Serial1Ext;
use cortex_m;
use cortex_m_rt::entry;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::fmt::Write;
use usb_device::device::UsbDeviceState;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let hsi48 = rcc.enable_hsi48(&mut syscfg, dp.CRS);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let usb = USB::new(dp.USB, gpioa.pa11, gpioa.pa12, hsi48);
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    let gpiob = dp.GPIOB.split(&mut rcc);
    let txpin = gpiob.pb6;
    let rxpin = gpiob.pb7;
    let uart = dp
        .USART1
        .usart(txpin, rxpin, serial::Config::default().baudrate(115_200_u32.bps()), &mut rcc)
        .unwrap();
    let (mut uart, _) = uart.split();
    writeln!(uart, "reset\r").unwrap();

    loop {

        let (istr,fnr) = cortex_m::interrupt::free(|_| { unsafe {
            (pac::Peripherals::steal().USB.istr.read().bits(),
             pac::Peripherals::steal().USB.fnr.read().bits())
        }});
        let oldstate = usb_dev.state();
        usb_dev.poll(&mut [&mut serial]);
        let state = usb_dev.state();
        if state != oldstate {
            writeln!(
                uart,
                "i:{:04x} f:{:04x} s:{}->{}\r",
                istr,
                fnr,
                print_state(&oldstate),
                print_state(&state)).ok();

            // Note: stm32_usb handles FSUSP and LPMOODE if a
            // suspend/resume event occurs during the poll() call above
        }
    }
}
fn print_state (state : &UsbDeviceState) -> &str {
    match state {
        UsbDeviceState::Default => "def",
        UsbDeviceState::Addressed => "addr",
        UsbDeviceState::Configured => "conf",
        UsbDeviceState::Suspend => "susp",
    }
}
