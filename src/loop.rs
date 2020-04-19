//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

extern crate panic_halt;

// device specific imports
use stm32l0xx_hal::usb::{UsbBus, USB};
use stm32l0xx_hal::{pac, prelude::*, rcc, serial, syscfg::SYSCFG};
use stm32l0xx_hal::serial::Serial1Ext;

use cortex_m_rt::entry;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::fmt::Write;


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
    writeln!(uart, "finished init\r").unwrap();

    let mut oldstate = usb_dev.state();
    loop {
        let pr = usb_dev.poll(&mut [&mut serial]);

        let state = usb_dev.state();
        if state != oldstate {
            oldstate = state;
            writeln!(
                uart,
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
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
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
}
