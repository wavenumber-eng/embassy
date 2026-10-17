# MCXA156 FIRC peripheral clock contract tests

From the repository root, run on a native host (substitute your host triple):

```sh
cargo test --manifest-path embassy-mcxa/tests/firc48/Cargo.toml --target x86_64-pc-windows-msvc
```

This standalone harness runs the production FIRC startup protocol against a
register model implementing write protection, read-only status and W1C faults.
It checks preservation of a running source, fault arrival during updates,
configuration readback, delayed readiness and bounded failure. It does not
access hardware or establish USB frequency tolerance. The native lane is
manual; no shared CI change is included.
