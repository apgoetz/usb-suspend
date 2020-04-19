This repository contains two test programs to explore the
suspend/resume logic of STM32L0XX chips. 

You can build them by using :

```
cargo build --bin irq --release
cargo build --bin loop --release
```

Both programs are based on the usb-serial example in stm32l0xx-hal usb-serial.rs.

The `loop` program uses busy-waiting to poll the usb stack, and it
prints out the contents of istr and fnr whenever the usb stack changes
state.

The `irq` program is similiar: it uses IRQs to handle the USB stack,
and it prints out the usb registers and usb status for every interrupt
that is received.

The Cargo.toml file can be used to select between the official
released version of the usb-stack, or a patched version at
https://github.com/apgoetz/stm32-usbd

