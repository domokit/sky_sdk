# dart_jit_runner

Contains the integration test for the Dart JIT runner.

### Running Tests
<!-- TODO(erkln): Replace steps once test runner script is updated to run Dart runner tests -->
#### Setup emulator and PM serve
```
fx set terminal.qemu-x64
ffx emu start --headless

fx serve
```

#### Prepare build files and build binary
```
$ENGINE_DIR/flutter/tools/gn --fuchsia --no-lto
ninja -C $ENGINE_DIR/out/fuchsia_debug_x64 flutter/shell/platform/fuchsia fuchsia_tests
```

#### Publish files to PM
```
$FUCHSIA_DIR/.jiri_root/bin/ffx repository publish $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files --package-archive $ENGINE_DIR/out/fuchsia_debug_x64/dart-jit-runner-integration-test-0.far

$FUCHSIA_DIR/.jiri_root/bin/ffx repository publish $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files --package-archive $ENGINE_DIR/out/fuchsia_debug_x64/oot_dart_jit_runner-0.far

$FUCHSIA_DIR/.jiri_root/bin/ffx repository publish $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files --package-archive $ENGINE_DIR/out/fuchsia_debug_x64/gen/flutter/shell/platform/fuchsia/dart_runner/tests/startup_integration_test/dart_echo_server/dart_jit_echo_server/dart_jit_echo_server.far
```

#### Run test
```
ffx test run "fuchsia-pkg://fuchsia.com/dart-jit-runner-integration-test#meta/dart-jit-runner-integration-test.cm"
```

## Notes
The `debug` runtime should be used when running the `dart_jit_runner` integration test. Snapshots will fail to generate or generate incorrectly if the wrong runtime is used.
