# Embassy NXP MCX-A MCUs HAL

A Hardware Abstraction Layer (HAL) for the NXP MCX-A family of
microcontrollers using the Embassy async framework. This HAL provides
safe, idiomatic Rust interfaces for GPIO, UART, and OSTIMER
peripherals.


## Initial MCXA156 support

Select `mcxa156` for the exact device. This implementation keeps the 12 MHz
reset system clock and provides GPIO on all five banks, OSTIMER, and blocking
LPUART0. Full clock configuration, other peripheral drivers, the LPTMR time
backend and the HAL low-power executor are not implemented for this part.
The examples are under `examples/mcxa156`; probe VCOM is UART, not native USB.

For peripheral clock preparation, set `config.clock_cfg.enable_firc48m = true`
before `embassy_mcxa::init(config)`. This enables the nominal fixed 48 MHz
FIRC output while preserving the 12 MHz system clock and an already running
FIRC's frequency configuration. It does not enable USB0, establish USB clock
tolerance, or construct a native USB driver. The option defaults to `false`.
The FIRC startup protocol has a bounded readiness poll and reports faults;
the existing SIRC initialization wait is unchanged.

This USB development branch temporarily pins the reviewed USB0 metadata in
the Wavenumber PAC fork. Replace that dependency with its upstream revision
before requesting upstream review; base support uses the merged upstream PAC.

The USB development branch includes experimental `usb::khci` support for
exact MCXA156. `Config::assume_vbus_present()` explicitly selects a bench
policy without VBUS detection. With independent debug power, keep native USB
connected to a powered host while enabled and disable before unplugging it.
The driver uses private static DMA bounce buffers and unique endpoint numbers
across directions; it supports full-speed bulk/interrupt endpoints through
64 bytes. It rejects isochronous endpoints, and reports force-reset and remote
wakeup as unsupported. Operating-range clock qualification and unrestricted
hotplug remain pending. See `examples/mcxa156/src/bin/usb_cdc_echo.rs` and the
native contract harness under `embassy-mcxa/tests/khci`.

Dedicated pins are excluded from safe HAL pin tokens unless their feature is
enabled. Opted-in pins are initialized in the disabled GPIO state, consistent
with the other MCXA implementations:

| Feature | MCXA156 pins released |
| --- | --- |
| `swd-as-gpio` | P0_0 (SWDIO/TMS), P0_1 (SWCLK/TCLK); prevents normal SWD use |
| `swd-swo-as-gpio` | P0_2 (SWO/TDO) |
| `jtag-extras-as-gpio` | P0_3 (TDI), P0_6 and P3_29 (package-dependent ISPMODE_N) |
| `dangerous-reset-as-gpio` | P1_29 (RESET_B); recovery is not validated |
| `sosc-as-gpio` | P1_30 (XTAL48M), P1_31 (EXTAL48M) |

A156 has no ROSC 32 kHz pin pair in this mapping. Pin tokens describe the
PAC's package union, not the bond-out of every package. Both ISP locations
are conservatively reserved; only one is the ISP input on a given package.
Consult the exact package pinout before connecting peripherals.
