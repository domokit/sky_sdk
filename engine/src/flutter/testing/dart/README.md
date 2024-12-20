# Dart UI Tests

The tests in this directory are written in Dart and run on the Flutter engine,
typically testing functionality that is not easily tested with C++ tests or
is best tested at a higher level (i.e. importing `dart:ui`).

## Running the tests

The simplest way to run these tests is using [`run_tests.py`][]:

```sh
./testing/run_tests.py --type=dart
```

Or, to run a specific test, provide the base file name as `--dart-filter`:

```sh
./testing/run_tests.py --type=dart --dart-filter=image_filter_test.dart
```

[`run_tests.py`]: ../run_tests.py

Note that the tests are _compiled_ as Dart kernel files, and not run from
source, if any changes are made to the tests, you will need to recompile them
before running the tests:

```sh
# At the time of this writing, 'et build' did not work with Dart targets.
# Instead, use the following command to build the tests.

ninja -C ../out/host_debug_unopt_arm64 compile_image_filter_test.dart
```

To view the outputted golden files locally, you'll need to open the generated
output folder, which will vary based on both your current target architecture
and the test suite you're running.

For example, for the above test on `host_debug_unopt_arm64`:

```sh
open ../out/host_debug_unopt_arm64/gen/skia_gold_image_filter_test.dart_iplr
```
