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
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::rwlock::RwLock;
use embassy_time::{with_timeout, Delay, Duration, Instant, Timer};

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

// Shared battery text type
type BatteryText = heapless::String<192>;

/// App -> Display commands
#[derive(Clone)]
enum DisplayCmd {
    Render(AppModel),
}

/// Events produced by peripheral tasks
#[derive(Clone)]
enum AppEvent {
    Battery(BatteryText),
    Key(heapless::String<64>),
    Touch(heapless::String<64>),
    Network(heapless::String<64>),
}

/// Central app model (what the UI renders)
#[derive(Clone)]
struct AppModel {
    battery: BatteryText,
    last_key: heapless::String<64>,
    last_touch: heapless::String<64>,
    net: heapless::String<64>,
    counter: u32,
}

impl Default for AppModel {
    fn default() -> Self {
        let mut battery = BatteryText::new();
        let _ = write!(battery, "Battery:\n(no data)");
        Self {
            battery,
            last_key: heapless::String::new(),
            last_touch: heapless::String::new(),
            net: heapless::String::new(),
            counter: 0,
        }
    }
}

#[derive(Clone, Copy)]
enum PendingKind {
    Immediate,
    Coalesced,
    Debounced,
}

#[derive(Clone, Copy)]
struct PendingRefresh {
    at: Instant,
    kind: PendingKind,
}

static EVENT_CH: Channel<CriticalSectionRawMutex, AppEvent, 32> = Channel::new();
static DISPLAY_CH: Channel<CriticalSectionRawMutex, DisplayCmd, 4> = Channel::new();

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
    let ev_tx = EVENT_CH.sender();
    let ev_rx = EVENT_CH.receiver();

    let disp_tx = DISPLAY_CH.sender();
    let disp_rx = DISPLAY_CH.receiver();

    spawner
        .spawn(battery_task(battery_service, ev_tx.clone()))
        .expect("spawn battery_task failed");
    spawner
        .spawn(keyboard_task(keyboard_controller, ev_tx.clone()))
        .expect("spawn keyboard_task failed");
    spawner
        .spawn(touch_task(touch_controller, ev_tx.clone()))
        .expect("spawn touch_task failed");

    // Optional placeholder network producer (replace with real networking)
    spawner
        .spawn(network_task(ev_tx.clone()))
        .expect("spawn network_task failed");

    // Central coordinator that decides refresh timing
    spawner
        .spawn(app_task(ev_rx, disp_tx))
        .expect("spawn app_task failed");

    // EPD is now a pure renderer/driver: refresh only on commands
    spawner
        .spawn(epd_task(display, spi_device, disp_rx))
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
    ev_tx: Sender<'static, CriticalSectionRawMutex, AppEvent, 32>,
) {
    let mut last_sent = BatteryText::new();

    loop {
        let mut s = BatteryText::new();

        match battery.measure().await {
            Ok(data) => {
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
            }
            Err(_) => {
                warn!("Battery: measure() failed");
                let _ = write!(s, "Battery:\n(read error)");
            }
        }

        // Only send if what would be displayed changed
        if s != last_sent {
            last_sent = s.clone();
            ev_tx.send(AppEvent::Battery(s)).await;
        }

        if battery.disable_adc().await.is_err() {
            warn!("Battery: disable_adc() failed");
        }

        Timer::after(Duration::from_secs(30)).await;
    }
}

#[embassy_executor::task]
async fn keyboard_task(
    mut keyboard: KeyboardController<
        'static,
        RwLockI2cDevice<I2c<'static, esp_hal::Async>, esp_hal::i2c::master::Error>,
        esp_hal::i2c::master::Error,
    >,
    ev_tx: Sender<'static, CriticalSectionRawMutex, AppEvent, 32>,
) {
    loop {
        match keyboard.read_key_events().await {
            Ok(events) => {
                for ev in events {
                    // Treat KeyState::Up as the "action" boundary (matches your prior behavior).
                    if ev.state == KeyState::Up {
                        info!("Key: {:?} (up)", ev.key);

                        let mut s = heapless::String::<64>::new();
                        let _ = write!(s, "{:?}", ev.key);
                        ev_tx.send(AppEvent::Key(s)).await;
                    }
                }
            }
            Err(_) => {
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
    ev_tx: Sender<'static, CriticalSectionRawMutex, AppEvent, 32>,
) {
    let mut last_sent = heapless::String::<64>::new(); // "" means "none"

    loop {
        match touch.read_touches().await {
            Ok(touches) => {
                // Represent "no touch" as empty string, so UI prints "(none)"
                let mut s = heapless::String::<64>::new();
                if !touches.is_empty() {
                    let _ = write!(s, "{:?}", touches);
                }

                // Only send if the displayed representation changed
                if s != last_sent {
                    last_sent = s.clone();
                    ev_tx.send(AppEvent::Touch(s)).await;
                }
            }
            Err(_) => {
                warn!("Touch: read_touches() error");
            }
        }

        // You can likely increase this now; 20ms is very aggressive for EPD-driven UI.
        Timer::after(Duration::from_millis(30)).await;
    }
}

#[embassy_executor::task]
async fn network_task(
    ev_tx: Sender<'static, CriticalSectionRawMutex, AppEvent, 32>,
) {
    // Placeholder producer:
    // Replace with your actual WiFi/LwIP/etc task that emits Network(...) events.
    // Keeping it very quiet by default.
    let mut last = heapless::String::<64>::new();
    let _ = write!(last, "net: (not started)");
    ev_tx.send(AppEvent::Network(last)).await;

    loop {
        Timer::after(Duration::from_secs(60)).await;
        // No spam by default. When you have real signals, emit AppEvent::Network(...)
    }
}

#[embassy_executor::task]
async fn app_task(
    ev_rx: Receiver<'static, CriticalSectionRawMutex, AppEvent, 32>,
    disp_tx: Sender<'static, CriticalSectionRawMutex, DisplayCmd, 4>,
) {
    // Refresh policy knobs
    const MIN_REFRESH_INTERVAL: Duration = Duration::from_secs(3);

    const TOUCH_DEBOUNCE: Duration = Duration::from_millis(150);
    const NET_COALESCE: Duration = Duration::from_millis(500);
    const BATTERY_DEBOUNCE: Duration = Duration::from_secs(3);

    let mut model = AppModel::default();
    let mut dirty = true; // draw once at boot
    let mut pending: Option<PendingRefresh> = Some(PendingRefresh {
        at: Instant::now(),
        kind: PendingKind::Immediate,
    });
    let mut last_refresh_at = Instant::from_ticks(0);

    loop {
        // If a refresh is due and we have dirty UI, render now.
        if let Some(p) = pending {
            let now = Instant::now();
            if now >= p.at {
                if dirty {
                    // Enforce a minimum interval so we don't hammer e-ink.
                    let earliest = last_refresh_at + MIN_REFRESH_INTERVAL;
                    if now < earliest {
                        pending = Some(PendingRefresh {
                            at: earliest,
                            kind: p.kind,
                        });
                    } else {
                        model.counter = model.counter.wrapping_add(1);
                        disp_tx.send(DisplayCmd::Render(model.clone())).await;
                        last_refresh_at = Instant::now();
                        dirty = false;
                        pending = None;
                    }
                    continue;
                } else {
                    pending = None;
                }
            }
        }

        // Compute how long we can wait for an event before a scheduled refresh.
        let next_wait = pending
            .map(|p| {
                let now = Instant::now();
                if p.at <= now {
                    Duration::from_millis(0)
                } else {
                    p.at - now
                }
            });

        let ev = if let Some(wait) = next_wait {
            match with_timeout(wait, ev_rx.receive()).await {
                Ok(ev) => Some(ev),
                Err(_) => None, // timeout => we'll loop and trigger refresh if due
            }
        } else {
            Some(ev_rx.receive().await)
        };

        let Some(ev) = ev else { continue };

        let now = Instant::now();
		let mut changed = false;

		// Apply event to model + decide refresh scheduling
		match ev {
			AppEvent::Key(k) => {
				if model.last_key != k {
					model.last_key = k;
					changed = true;

					pending = Some(PendingRefresh {
						at: now,
						kind: PendingKind::Immediate,
					});
				}
			}
			AppEvent::Touch(t) => {
				if model.last_touch != t {
					model.last_touch = t;
					changed = true;

					let desired = now + TOUCH_DEBOUNCE;
					pending = match pending {
						Some(p) if matches!(p.kind, PendingKind::Immediate) => Some(p),
						Some(p) => Some(PendingRefresh {
							at: core::cmp::max(p.at, desired),
							kind: PendingKind::Debounced,
						}),
						None => Some(PendingRefresh {
							at: desired,
							kind: PendingKind::Debounced,
						}),
					};
				}
			}
			AppEvent::Network(n) => {
				if model.net != n {
					model.net = n;
					changed = true;

					let desired = now + NET_COALESCE;
					pending = match pending {
						Some(p) if matches!(p.kind, PendingKind::Immediate) => Some(p),
						Some(p) if matches!(p.kind, PendingKind::Debounced) => Some(PendingRefresh {
							at: core::cmp::min(p.at, desired),
							kind: PendingKind::Coalesced,
						}),
						Some(p) => Some(PendingRefresh {
							at: core::cmp::min(p.at, desired),
							kind: PendingKind::Coalesced,
						}),
						None => Some(PendingRefresh {
							at: desired,
							kind: PendingKind::Coalesced,
						}),
					};
				}
			}
			AppEvent::Battery(b) => {
				if model.battery != b {
					model.battery = b;
					changed = true;

					let desired = now + BATTERY_DEBOUNCE;
					pending = match pending {
						Some(p) if matches!(p.kind, PendingKind::Immediate) => Some(p),
						Some(p) => Some(PendingRefresh {
							at: core::cmp::min(p.at, desired),
							kind: p.kind,
						}),
						None => Some(PendingRefresh {
							at: desired,
							kind: PendingKind::Debounced,
						}),
					};
				}
			}
		}

		if changed {
			dirty = true;
		}
    }
}

#[embassy_executor::task]
async fn epd_task(
    mut display: EInkDisplay<'static>,
    mut spi_dev: RwLockDevice<'static, esp_hal::spi::master::Spi<'static, esp_hal::Async>, Delay>,
    disp_rx: Receiver<'static, CriticalSectionRawMutex, DisplayCmd, 4>,
) {
    info!("EPD: initializing...");
    if let Err(_) = display.init(&mut spi_dev).await {
        error!("EPD: init failed");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
    info!("EPD: init OK");

    loop {
        match disp_rx.receive().await {
            DisplayCmd::Render(model) => {
                display.clear(BinaryColor::Off).ok();

                let mut text_buf = heapless::String::<320>::new();
                let _ = write!(
                    text_buf,
                    "T-Deck Pro\n\
                     Frame: {ctr}\n\n\
                     Key: {key}\n\
                     Touch: {touch}\n\
                     {net}\n\n\
                     {batt}",
                    ctr = model.counter,
                    key = if model.last_key.is_empty() { "(none)" } else { model.last_key.as_str() },
                    touch =
                        if model.last_touch.is_empty() { "(none)" } else { model.last_touch.as_str() },
                    net = if model.net.is_empty() { "" } else { model.net.as_str() },
                    batt = model.battery,
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
            }
        }
    }
}
