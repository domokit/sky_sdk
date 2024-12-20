# Flutter runner integration tests

To run the Flutter runner integration tests locally,
first start a Fuchsia package server:

```shell
cd "$FUCHSIA_DIR"
fx serve
```

Then run the integration test:

```shell
$ENGINE_DIR/flutter/tools/fuchsia/devshell/run_integration_test.sh <integration_test_folder_name> --no-lto
```

For example, to run the `embedder` integration test:

```shell
$ENGINE_DIR/flutter/tools/fuchsia/devshell/run_integration_test.sh embedder --no-lto
```

Command-line options:

* Pass `--unoptimized` to disable C++ compiler optimizations.
* Add `--fuchsia-cpu x64` or `--fuchsia-cpu arm64` to target a particular architecture.
  The default is x64.
* Add `--runtime-mode debug` or `--runtime-mode profile` to switch between JIT and AOT
  builds.  These correspond to a vanilla Fuchsia build and a `--release` Fuchsia build
  respectively.  The default is debug/JIT builds.
* Remove `--no-lto` if you care about performance or binary size; unfortunately it results
  in a *much* slower build.

## Iterating on tests

By default, `run_integration_test.sh` will build Fuchsia and start up a Fuchsia emulator
to ensure that the test runs on the correct environment.

However, this is slow for iterating on tests. Once you've run `run_integration_tests.sh`
once, you don't need to build Fuchsia or start the emulator anymore, and can pass
`--skip-fuchsia-build` and `--skip-fuchsia-emu` to skip those steps.

```shell
$ENGINE_DIR/flutter/tools/fuchsia/devshell/run_integration_test.sh embedder --no-lto --skip-fuchsia-build --skip-fuchsia-emu
```
