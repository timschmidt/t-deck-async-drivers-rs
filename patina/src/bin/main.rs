#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

extern crate alloc;

use alloc::rc::Rc;
use core::fmt::Write;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::rwlock::RwLock;
use embassy_time::{Delay, Duration, Timer};

use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

// Shared-bus helpers (your workspace crate)
use embedded_bus_async::i2c::RwLockI2cDevice;
use embedded_bus_async::spi::RwLockDevice;

// Device/service crates (your workspace crates)
use t_deck_pro_battery_async::BatteryService;
use t_deck_pro_epd_async::EInkDisplay;
use t_deck_pro_keyboard_async::keyboard::{KeyState, KeyboardController};
use t_deck_pro_touch_async::touch::TouchController;

// Display text rendering (from your epd example)
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

// Shared battery text for the display
type BatteryText = heapless::String<192>;
static BATTERY_TEXT: Mutex<CriticalSectionRawMutex, BatteryText> =
    Mutex::new(heapless::String::new());

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see:
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // RTT/defmt init
    rtt_target::rtt_init_defmt!();

    // HAL init
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heaps (same pattern as your template)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Start esp-rtos timer backing Embassy time
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Boot: Embassy initialized");

    // ---------------------------------------------------------------------
    // Shared reset line (GPIO45)
    //  - Used by multiple peripherals on T-Deck Pro
    //  - We do one hardware reset pulse up-front, then we MOVE the pin into
    //    the EPD driver (EPD init commonly needs it).
    // ---------------------------------------------------------------------
    let mut shared_rst = Output::new(peripherals.GPIO45, Level::High, OutputConfig::default());
    shared_rst.set_low();
    Timer::after(Duration::from_millis(20)).await;
    shared_rst.set_high();
    Timer::after(Duration::from_millis(300)).await;
    info!("Shared reset pulse complete");

    // ---------------------------------------------------------------------
    // I2C0 shared bus (GPIO14=SCL, GPIO13=SDA) — used by battery/keyboard/touch
    // ---------------------------------------------------------------------
    let i2c_scl = peripherals.GPIO14;
    let i2c_sda = peripherals.GPIO13;
    let i2c_cfg = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));

    let i2c = I2c::new(peripherals.I2C0, i2c_cfg)
        .expect("Failed to create I2C0")
        .with_sda(i2c_sda)
        .with_scl(i2c_scl)
        .into_async();

    let i2c_bus = Rc::new(RwLock::new(i2c));

    // Each service gets its own bus handle (RwLock-guarded shared bus)
    let battery_i2c = RwLockI2cDevice::new(i2c_bus.clone());
    let keyboard_i2c = RwLockI2cDevice::new(i2c_bus.clone());
    let touch_i2c = RwLockI2cDevice::new(i2c_bus.clone());

    // Battery service (polled; no INT pin needed)
    let battery_service = BatteryService::new(battery_i2c);

    // Keyboard controller (INT on GPIO15 from your keyboard example)
    let keyboard_int = Input::new(peripherals.GPIO15, InputConfig::default());
    let mut keyboard_controller = KeyboardController::new(keyboard_i2c, keyboard_int, None);
    match keyboard_controller.init().await {
        Ok(_) => info!("Keyboard initialized"),
        Err(_) => warn!("Keyboard init failed"),
    };

    // Touch controller (INT on GPIO12 from your touch example)
    let touch_int = Input::new(peripherals.GPIO12, InputConfig::default());
    let mut touch_controller = TouchController::new(touch_i2c, touch_int, None);
    match touch_controller.init().await {
        Ok(_) => info!("Touch initialized"),
        Err(_) => warn!("Touch init failed"),
    };

    // ---------------------------------------------------------------------
    // EPD (SPI2) init wiring as in your epd example:
    //   SCLK=GPIO36, MOSI=GPIO33, CS=GPIO34, DC=GPIO35, BUSY=GPIO37, RST=GPIO45
    // ---------------------------------------------------------------------
    let epd_dc = Output::new(peripherals.GPIO35, Level::Low, OutputConfig::default());
    let epd_busy = Input::new(peripherals.GPIO37, InputConfig::default());
    let epd_rst = shared_rst; // move the reset pin into EPD ownership

    let sclk = peripherals.GPIO36;
    let mosi = peripherals.GPIO33;
    let cs = Output::new(peripherals.GPIO34, Level::High, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(4))
            .with_mode(Mode::_0),
    )
    .expect("Failed to create SPI2")
    .with_sck(sclk)
    .with_mosi(mosi)
    .into_async();

    let spi_bus = Rc::new(RwLock::new(spi));
    let spi_device = RwLockDevice::new(spi_bus, cs, Delay);

    let display = EInkDisplay::new(epd_dc, epd_busy, Some(epd_rst), 240, 320, false);

    // ---------------------------------------------------------------------
    // Spawn tasks
    // ---------------------------------------------------------------------
    spawner
        .spawn(battery_task(battery_service, &BATTERY_TEXT))
        .expect("spawn battery_task failed");
    spawner
        .spawn(keyboard_task(keyboard_controller))
        .expect("spawn keyboard_task failed");
    spawner
        .spawn(touch_task(touch_controller))
        .expect("spawn touch_task failed");
    spawner
        .spawn(epd_task(display, spi_device, &BATTERY_TEXT))
        .expect("spawn epd_task failed");

    info!("All tasks spawned; entering idle loop");
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn battery_task(
    mut battery: BatteryService<
        RwLockI2cDevice<I2c<'static, esp_hal::Async>, esp_hal::i2c::master::Error>,
        esp_hal::i2c::master::Error,
    >,
    battery_text: &'static Mutex<CriticalSectionRawMutex, BatteryText>,
) {
    loop {
        match battery.measure().await {
            Ok(data) => {
                info!("--- Battery Status ---");
                info!("Voltage: {=f32} V", data.voltage);
                info!("VBUS Voltage: {=f32} V", data.vbus_voltage);
                info!("Charge Current: {=f32} A", data.charge_current);
                info!("Battery Temp Percent: {=f32}%", data.battery_temp_percent);
                info!("Charge Status: {:?}", data.charging_status);
                info!("VBUS Status: {:?}", data.vbus_status);
                info!("Power Good: {}", data.power_good);
                info!("Faults: {:?}", data.faults);
                info!("----------------------");
                
                // Format a concise multi-line summary for the EPD.
                let mut s = BatteryText::new();

                // Keep it short enough to fit comfortably on screen.
                // (You can tweak wording / add more fields if you like.)
                let _ = write!(
                    s,
                    "Battery:\n\
                     Vbat: {v:.2}V\n\
                     Vbus: {vb:.2}V\n\
                     Ichg: {i:.2}A\n\
                     Chg: {chg:?}\n\
                     PG: {pg}",
                    v = data.voltage,
                    vb = data.vbus_voltage,
                    i = data.charge_current,
                    chg = data.charging_status,
                    pg = data.power_good,
                );

                // Publish to shared state
                let mut guard = battery_text.lock().await;
                *guard = s;
            }
            Err(_) => {
				warn!("Battery: measure() failed");
				
				// Still publish something so the display shows a sane state.
                let mut guard = battery_text.lock().await;
                guard.clear();
                let _ = write!(*guard, "Battery:\n(read error)");
			}
        }

        if battery.disable_adc().await.is_err() {
            warn!("Battery: disable_adc() failed");
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn keyboard_task(
    mut keyboard: KeyboardController<
        'static,
        RwLockI2cDevice<I2c<'static, esp_hal::Async>, esp_hal::i2c::master::Error>,
        esp_hal::i2c::master::Error,
    >,
) {
    loop {
        match keyboard.read_key_events().await {
            Ok(events) => {
                for ev in events {
                    // Your gps example treated KeyState::Up as "pressed/released" boundary.
                    if ev.state == KeyState::Up {
                        info!("Key: {:?} (up)", ev.key);
                    } else {
                        info!("Key: {:?} (down)", ev.key);
                    }
                }
            }
            Err(_) => {
                // If this gets chatty, reduce to warn! occasionally.
                warn!("Keyboard: read_key_events() error");
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn touch_task(
    mut touch: TouchController<
        'static,
        RwLockI2cDevice<I2c<'static, esp_hal::Async>, esp_hal::i2c::master::Error>,
        esp_hal::i2c::master::Error,
    >,
) {
    loop {
        match touch.read_touches().await {
            Ok(touches) => {
                if !touches.is_empty() {
                    info!("Touch: {:?}", touches);
                }
            }
            Err(_) => {
                // Touch can be noisy on startup; keep it as warn.
                warn!("Touch: read_touches() error");
            }
        }
        Timer::after(Duration::from_millis(20)).await;
    }
}

#[embassy_executor::task]
async fn epd_task(
    mut display: EInkDisplay<'static>,
    mut spi_dev: RwLockDevice<'static, esp_hal::spi::master::Spi<'static, esp_hal::Async>, Delay>,
    battery_text: &'static Mutex<CriticalSectionRawMutex, BatteryText>,
) {
    info!("EPD: initializing...");
    if let Err(_) = display.init(&mut spi_dev).await {
        error!("EPD: init failed");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
    info!("EPD: init OK");

    let mut counter: u32 = 0;

    loop {
        display.clear(BinaryColor::Off).ok();

        // Grab latest battery text without holding the lock while drawing.
        let batt = {
            let guard = battery_text.lock().await;
            guard.clone()
        };

        let mut text_buf = heapless::String::<256>::new();
        let _ = write!(
            text_buf,
            "T-Deck Pro\n\
             Counter: {counter}\n\n\
             {batt}",
        );

        Text::new(
            &text_buf,
            Point::new(20, 40),
            MonoTextStyle::new(&FONT_10X20, BinaryColor::On),
        )
        .draw(&mut display)
        .ok();

        if let Err(_) = display.refresh_display(&mut spi_dev).await {
            warn!("EPD: refresh failed");
        }

        counter = counter.wrapping_add(1);
        Timer::after(Duration::from_secs(5)).await;
    }
}

