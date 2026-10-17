# KHCI native contract tests

Run from the repository root with your native host target:

```sh
cargo test --manifest-path embassy-mcxa/tests/khci/Cargo.toml --target x86_64-pc-windows-msvc
```

The harness compiles the actual private ownership, DMA-storage, engine and
control-poll modules. The fake register backend models the completion FIFO,
exact W1C acknowledgments, rejected endpoint-disable requests and event arrival at
publication boundaries, and SETUP busy on endpoint restoration. Tests cover cancellation quarantine, generation and
bank ordering, unrelated completions, early control status, new SETUP during
old publication/rejection, reject recovery, bounded reset, and suspend/reset
event coalescing. The ARM PAC adapter additionally requires an ARM build.

This manual lane does not replace hardware tests or establish the engine-stop
engineering inference experimentally. Normal completion/publication keeps endpoint acceptance live and uses the
checked SETUP hold. Address changes retain RX suspension; actual owned-IN
cancellation suspends only its TX while preserving USB enable; readback/drain is not a separately
documented in-flight DMA-idle acknowledgment. Shared CI changes are a separate scope.
