//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

extern crate panic_halt;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;

use core::fmt::Write;
use cortex_m_rt::entry;
use stm32l0::stm32l0x3;
use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use stm32l0xx_hal::pac::{interrupt, Interrupt};
use stm32l0xx_hal::serial::Serial1Ext;
use stm32l0xx_hal::usb::{UsbBus, UsbBusType, USB};
use stm32l0xx_hal::{pac, prelude::*, rcc, serial, syscfg::SYSCFG};
use usb_device::{bus, device, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

struct Resources {
    uart: serial::Tx<stm32l0x3::USART1>,
    usb_dev: UsbDevice<'static, UsbBusType>,
    serial: SerialPort<'static, UsbBusType>,
}
static RESOURCES: Mutex<RefCell<Option<Resources>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let hsi48 = rcc.enable_hsi48(&mut syscfg, dp.CRS);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let usb = USB::new(dp.USB, gpioa.pa11, gpioa.pa12, hsi48);
    *USB_BUS = Some(UsbBus::new(usb));

    let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

    let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
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
    writeln!(uart, "finished init\r").unwrap();
    cortex_m::interrupt::free(|cs| {
        *RESOURCES.borrow(cs).borrow_mut() = Some(Resources {
            uart,
            usb_dev,
            serial,
        });
    });

    unsafe { NVIC::unmask(Interrupt::USB); }
    loop {
        asm::wfi();
    }
}

#[interrupt]
fn USB() {
    static mut OLDSTATE: Option<device::UsbDeviceState> = None;

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut r) = RESOURCES.borrow(cs).borrow_mut().deref_mut() {
            let pr = r.usb_dev.poll(&mut [&mut r.serial]);

            let state = r.usb_dev.state();
            if Some(state) != *OLDSTATE || false {
                *OLDSTATE = Some(state);
                writeln!(
                    r.uart,
                    "{}\r",
                    match state {
                        usb_device::device::UsbDeviceState::Default => "def",
                        usb_device::device::UsbDeviceState::Addressed => "addr",
                        usb_device::device::UsbDeviceState::Configured => "conf",
                        usb_device::device::UsbDeviceState::Suspend => "susp",
                    }
                )
                .ok();

                // Note: stm32_usb handles FSUSP and LPMOODE if a
                // suspend/resume event occurs during the poll() call above
            }

            if !pr {
                return;
            }

            let mut buf = [0u8; 64];

            match r.serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match r.serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    });
}
