# API consistency check tool

Verifies that enums in each of the platform-specific embedders, and the embedder
API remain consistent with their API in dart:ui.

## Running the tool

This tool is run as part of `testing/run_tests.sh`.

To run the tool, invoke with the path of the Flutter engine repo as the first
argument.

```sh
../../../out/host_debug_unopt/dart-sdk/bin/dart \
  test/apicheck_test.dart                       \
  "$(dirname $(dirname $PWD))"
```
