#![no_std]
#![no_main]

//! Native J23 CDC bench echo. Keep J23 attached to a powered host while enabled;
//! J21 debug power is present and the USB descriptor is self-powered.

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_mcxa as hal;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::device::{CdcAcmClass, State as CdcState};
use embassy_usb::{Builder, Config};
use hal::gpio::{DriveStrength, Level, Output, SlewRate};
use hal::lpuart::{Blocking, Lpuart};
use hal::usb::khci::{self, Driver};
use panic_probe as _;
use static_cell::StaticCell;

hal::bind_interrupts!(struct Irqs { USB0 => khci::InterruptHandler; });

static CONFIG: StaticCell<[u8; 256]> = StaticCell::new();
static BOS: StaticCell<[u8; 64]> = StaticCell::new();
static MSOS: StaticCell<[u8; 64]> = StaticCell::new();
static CONTROL: StaticCell<[u8; 64]> = StaticCell::new();
static CDC: StaticCell<CdcState<'static>> = StaticCell::new();

fn clock_snapshot() -> [u32; 13] {
    let s = hal::pac::SCG0;
    [
        s.csr().read().0,
        s.rccr().read().0,
        hal::pac::SYSCON.ahbclkdiv().read().0,
        s.firccsr().read().0,
        s.firccfg().read().0,
        s.firctcfg().read().0,
        s.firctrim().read().0,
        s.fircstat().read().0,
        s.fircatc7().read().0,
        s.fircatc8().read().0,
        s.fircatc9().read().0,
        s.fircatc10().read().0,
        s.fircatc11().read().0,
    ]
}

fn print_snapshot(uart: &mut Lpuart<'_, Blocking>, phase: &str, values: [u32; 13]) {
    uart.write_str_blocking(phase);
    for (name, value) in [
        " CSR=",
        " RCCR=",
        " AHB=",
        " FIRCCSR=",
        " FIRCCFG=",
        " FIRCTCFG=",
        " FIRCTRIM=",
        " FIRCSTAT=",
        " ATC7=",
        " ATC8=",
        " ATC9=",
        " ATC10=",
        " ATC11=",
    ]
    .into_iter()
    .zip(values)
    {
        print_word(uart, name, value);
    }
    uart.write_str_blocking("\r\n");
}

fn print_word(uart: &mut Lpuart<'_, Blocking>, name: &str, value: u32) {
    uart.write_str_blocking(name);
    let mut hex = [0; 8];
    for (i, b) in hex.iter_mut().enumerate() {
        *b = b"0123456789abcdef"[((value >> (28 - i * 4)) & 15) as usize];
    }
    uart.blocking_write(&hex).unwrap();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let before = clock_snapshot();
    let mut config = hal::config::Config::default();
    config.clock_cfg.enable_firc48m = true;
    let p = hal::init(config);
    let mut uart = Lpuart::new_blocking(
        p.LPUART0,
        p.P0_3,
        p.P0_2,
        hal::lpuart::Config {
            baudrate_bps: 115_200,
            ..Default::default()
        },
    )
    .unwrap();
    print_snapshot(&mut uart, "clock-before", before);
    print_snapshot(&mut uart, "clock-enabled", clock_snapshot());
    let driver = Driver::new(p.USB0, Irqs, khci::Config::assume_vbus_present()).unwrap();
    uart.write_str_blocking("FRDM-MCXA156 native USB bench ready\r\n");
    // Laboratory identity inherited from the prior isolated experiment. This
    // example makes no claim of a production VID/PID allocation.
    let mut config = Config::new(0xcafe, 0x1560);
    config.manufacturer = Some("Wavenumber");
    config.product = Some("FRDM-MCXA156 MetaPAC CDC bench");
    config.serial_number = Some("WN-META-A156-0001");
    config.device_class = 0xef;
    config.device_sub_class = 2;
    config.device_protocol = 1;
    config.composite_with_iads = true;
    config.self_powered = true;
    config.max_power = 100;
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG.init([0; 256]),
        BOS.init([0; 64]),
        MSOS.init([0; 64]),
        CONTROL.init([0; 64]),
    );
    let cdc = CdcAcmClass::new(&mut builder, CDC.init(CdcState::new()), 64);
    let mut device = builder.build();
    let led = Output::new(p.P3_13, Level::High, DriveStrength::Normal, SlewRate::Fast);
    join3(device.run(), echo(cdc), monitor(uart, led)).await;
}

async fn echo(mut cdc: CdcAcmClass<'static, Driver<'static>>) {
    let mut data = [0; 64];
    loop {
        cdc.wait_connection().await;
        loop {
            let length = match cdc.read_packet(&mut data).await {
                Ok(length) => length,
                Err(_) => break,
            };
            if cdc.write_packet(&data[..length]).await.is_err() {
                break;
            }
            if length == 64 && cdc.write_packet(&[]).await.is_err() {
                break;
            }
        }
    }
}

async fn monitor(mut uart: Lpuart<'static, Blocking>, mut led: Output<'static>) {
    Timer::after(Duration::from_secs(3)).await;
    print_snapshot(&mut uart, "clock-running", clock_snapshot());
    loop {
        let counters = khci::diagnostic_counters();
        uart.write_str_blocking("usb-running");
        for (name, value) in [" RESETS=", " TOKENS=", " PAUSES=", " ERRORS=", " ERROR_BITS="]
            .into_iter()
            .zip(counters)
        {
            print_word(&mut uart, name, value);
        }
        let snapshot = cortex_m::interrupt::free(|_| {
            let mrcc = hal::pac::MRCC0;
            if !mrcc.mrcc_glb_cc0().read().usb0() || !mrcc.mrcc_glb_rst0().read().usb0() {
                return None;
            }
            let usb = hal::pac::USB0;
            Some([
                usb.ctl().read().0 as u32,
                usb.istat().read().0 as u32,
                usb.addr().read().0 as u32,
                usb.frmnuml().read().0 as u32,
            ])
        });
        if let Some(snapshot) = snapshot {
            for (name, value) in [" CTL=", " ISTAT=", " ADDR=", " FRAME="].into_iter().zip(snapshot) {
                print_word(&mut uart, name, value);
            }
        }
        uart.write_str_blocking("\r\n");
        let (trace, count) = khci::diagnostic_trace();
        for event in &trace[..count] {
            uart.write_str_blocking("usb-trace");
            for (name, value) in [" TYPE=", " A=", " B=", " C="].into_iter().zip(*event) {
                print_word(&mut uart, name, value);
            }
            uart.write_str_blocking("\r\n");
        }
        led.toggle();
        Timer::after(Duration::from_secs(3)).await;
    }
}
