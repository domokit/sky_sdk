# Impeller Golden Tests

This is the executable that will generate the golden image results that can then
be sent to Skia Gold vial the
[golden_tests_harvester]("../../tools/golden_tests_harvester").

Running these tests should happen from
[//flutter/testing/run_tests.py](../../testing/run_tests.py). That will do all
the steps to generate the golden images and transmit them to Skia Gold. If you
run the tests locally it will not actually upload anything. That only happens if
the script is executed from LUCI.

Example invocation:

```sh
./run_tests.py --variant="host_debug_unopt_arm64" --type="impeller-golden"
```

Currently these tests are only supported on macOS and only test the Metal
backend to Impeller.

## Adding tests

To add a golden image test, the `impeller_golden_tests` target must be modified
to generate the correct image and modification to its generated `digest.json`.
If a test case is added to [golden_tests.cc](./golden_tests.cc), for example
"GoldenTests.FooBar", that will turn into the golden test
"impeller_GoldenTests_Foobar" automatically if the `SaveScreenshot()` function
is used.

The examples in `golden_tests.cc` use GLFW for rendering the tests, but
technically anything could be used.  Using the `SaveScreenshot()` function will
automatically update the `GoldenDigest::Instance()` which will make sure that it
is included in the generated `digest.json`. If that function isn't used the
`GoldenDigest` should be updated manually.
