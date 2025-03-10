#![no_std]
#![no_main]
use core::fmt::Write as _;

use panic_halt as _;
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal::Clock};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::{
    descriptor::{MouseReport, SerializedDescriptor},
    hid_class::HIDClass,
};

static mut USB_DEVICE: Option<UsbDevice<rp_pico::hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<rp_pico::hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<rp_pico::hal::usb::UsbBus>> = None;

#[entry]
#[allow(static_mut_refs)]
fn main() -> ! {
    let mut pac = rp_pico::pac::Peripherals::take().unwrap();
    let mut watchdog = rp_pico::hal::Watchdog::new(pac.WATCHDOG);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = rp_pico::hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = usb_device::bus::UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 1);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Twitchy Mousey")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        rp_pico::pac::NVIC::unmask(rp_pico::hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = rp_pico::pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        let input = "563748265478\n";
        for c in input.chars() {
            let h = char_to_hid(c);
            let kb = KeyboardReport {
                keycodes: [h.0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: h.1,
                reserved: 0,
            };
            push_keyboard(kb).ok().unwrap_or(0);
            delay.delay_ms(10);
        }

        let kb = KeyboardReport {
            keycodes: [0, 0, 0, 0, 0, 0],
            leds: 0,
            modifier: 0,
            reserved: 0,
        };
        push_keyboard(kb).ok().unwrap_or(0);

        delay.delay_ms(100);
    }
}

#[allow(static_mut_refs)]
fn push_keyboard(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[allow(static_mut_refs)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let usb_hid = unsafe { USB_HID.as_mut().unwrap() };
    usb_dev.poll(&mut [usb_hid]);
}

const MODIFIER_LEFT_SHIFT: u8 = 0x02;
fn char_to_hid(c: char) -> (u8, u8) {
    // Returns (keycode, modifier)
    match c {
        'a'..='z' => ((c as u8 - b'a' + 4), 0),
        'A'..='Z' => ((c as u8 - b'A' + 4), MODIFIER_LEFT_SHIFT),
        '0' => (0x27, 0),
        '1' => (0x1E, 0),
        '2' => (0x1F, 0),
        '3' => (0x20, 0),
        '4' => (0x21, 0),
        '5' => (0x22, 0),
        '6' => (0x23, 0),
        '7' => (0x24, 0),
        '8' => (0x25, 0),
        '9' => (0x26, 0),
        '!' => (0x1E, MODIFIER_LEFT_SHIFT),
        '@' => (0x1F, MODIFIER_LEFT_SHIFT),
        '#' => (0x20, MODIFIER_LEFT_SHIFT),
        '$' => (0x21, MODIFIER_LEFT_SHIFT),
        '%' => (0x22, MODIFIER_LEFT_SHIFT),
        '^' => (0x23, MODIFIER_LEFT_SHIFT),
        '&' => (0x24, MODIFIER_LEFT_SHIFT),
        '*' => (0x25, MODIFIER_LEFT_SHIFT),
        '(' => (0x26, MODIFIER_LEFT_SHIFT),
        ')' => (0x27, MODIFIER_LEFT_SHIFT),
        '-' => (0x2D, 0),
        '_' => (0x2D, MODIFIER_LEFT_SHIFT),
        '=' => (0x2E, 0),
        '+' => (0x2E, MODIFIER_LEFT_SHIFT),
        '[' => (0x2F, 0),
        ']' => (0x30, 0),
        '{' => (0x2F, MODIFIER_LEFT_SHIFT),
        '}' => (0x30, MODIFIER_LEFT_SHIFT),
        ';' => (0x33, 0),
        ':' => (0x33, MODIFIER_LEFT_SHIFT),
        '\'' => (0x34, 0),
        '"' => (0x34, MODIFIER_LEFT_SHIFT),
        ',' => (0x36, 0),
        '<' => (0x36, MODIFIER_LEFT_SHIFT),
        '.' => (0x37, 0),
        '>' => (0x37, MODIFIER_LEFT_SHIFT),
        '/' => (0x38, 0),
        '?' => (0x38, MODIFIER_LEFT_SHIFT),
        '`' => (0x35, 0),
        '~' => (0x35, MODIFIER_LEFT_SHIFT),
        '\\' => (0x31, 0),
        '|' => (0x31, MODIFIER_LEFT_SHIFT),
        ' ' => (0x2C, 0),  // Space
        '\n' => (0x28, 0), // Enter
        _ => (0, 0),       // Unknown characters produce no key
    }
}
