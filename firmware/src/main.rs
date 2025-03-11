#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use alloc_cortex_m::CortexMHeap;
use blocking_mfrc522::MFRC522;
use core::alloc::Layout;
use cortex_m::delay::Delay;
use embedded_hal::delay::DelayNs;
use panic_halt as _;
use rp_pico::Pins;
use rp_pico::hal::fugit::RateExtU32;
use rp_pico::hal::gpio::FunctionSpi;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::rom_data::reset_to_usb_boot;
use rp_pico::hal::{Sio, Timer};
use rp_pico::{entry, hal::Clock};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::{descriptor::SerializedDescriptor, hid_class::HIDClass};

extern crate alloc;

const KEY_PRESS_TIME: u32 = 8;

static mut USB_DEVICE: Option<UsbDevice<rp_pico::hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<rp_pico::hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<rp_pico::hal::usb::UsbBus>> = None;
static mut OWN_DELAY: Option<Delay> = None;
static mut TIMER: Option<Timer> = None;

pub struct OwnDelay {}

impl DelayNs for OwnDelay {
    fn delay_ns(&mut self, ns: u32) {
        unsafe {
            #[allow(static_mut_refs)]
            if let Some(delay) = OWN_DELAY.as_mut() {
                delay.delay_us(ns / 1000);
            }
        }
    }
}

impl DelayNs for &OwnDelay {
    fn delay_ns(&mut self, ns: u32) {
        unsafe {
            #[allow(static_mut_refs)]
            if let Some(delay) = OWN_DELAY.as_mut() {
                delay.delay_us(ns / 1000);
            }
        }
    }
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    cortex_m::asm::bkpt();
    loop {}
}

#[entry]
#[allow(static_mut_refs)]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    let size = 32768;
    unsafe { ALLOCATOR.init(start, size) }

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

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let core = rp_pico::pac::CorePeripherals::take().unwrap();
    unsafe {
        OWN_DELAY = Some(cortex_m::delay::Delay::new(
            core.SYST,
            clocks.system_clock.freq().to_Hz(),
        ));

        TIMER = Some(rp_pico::hal::Timer::new(
            pac.TIMER,
            &mut pac.RESETS,
            &clocks,
        ));
    }

    let cs = pins.gpio17.into_push_pull_output();
    let miso = pins.gpio16.into_function::<FunctionSpi>();
    let mosi = pins.gpio19.into_function::<FunctionSpi>();
    let sck = pins.gpio18.into_function::<FunctionSpi>();

    let spi = rp_pico::hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        5.MHz(),
        embedded_hal::spi::MODE_0,
    );
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, OwnDelay {}).unwrap();

    let mut mfrc522 = MFRC522::new(spi_device, || unsafe {
        if let Some(timer) = TIMER {
            timer.get_counter().duration_since_epoch().to_micros()
        } else {
            0
        }
    });
    _ = mfrc522.pcd_init();

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
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        rp_pico::pac::NVIC::unmask(rp_pico::hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut delay = OwnDelay {};
    loop {
        if mfrc522.picc_is_new_card_present().is_ok() {
            let card = mfrc522.get_card(blocking_mfrc522::consts::UidSize::Four);
            if let Ok(card) = card {
                if card.get_number() == 1264825046 {
                    cortex_m::interrupt::disable();
                    reset_to_usb_boot(0, 0);
                }

                let input = alloc::format!("{}\n", card.get_number());
                for c in input.chars() {
                    let h = char_to_hid(c);
                    let kb = KeyboardReport {
                        keycodes: [h.0, 0, 0, 0, 0, 0],
                        leds: 0,
                        modifier: h.1,
                        reserved: 0,
                    };
                    push_keyboard(kb).ok().unwrap_or(0);
                    delay.delay_ms(KEY_PRESS_TIME);

                    let kb = KeyboardReport {
                        keycodes: [0, 0, 0, 0, 0, 0],
                        leds: 0,
                        modifier: 0,
                        reserved: 0,
                    };
                    push_keyboard(kb).ok().unwrap_or(0);
                    delay.delay_ms(KEY_PRESS_TIME);
                }

                _ = mfrc522.picc_halta();
            }
        }
    }
}

#[allow(static_mut_refs)]
fn push_keyboard(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}

#[allow(non_snake_case)]
#[allow(static_mut_refs)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let usb_hid = unsafe { USB_HID.as_mut().unwrap() };
    usb_dev.poll(&mut [usb_hid]);
}

const MODIFIER_LEFT_SHIFT: u8 = 0x02;
fn char_to_hid(c: char) -> (u8, u8) {
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
