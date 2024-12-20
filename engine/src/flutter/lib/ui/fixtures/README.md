# ui_unittest Fixtures

The files in this directory are used by the ui_unittests binary.

The `ui_test.dart` file is either JIT or AOT compiled depending on the runtime
mode of the test binary. Other files in this folder are used by tests to verify
functionality.

See `//lib/ui/BUILD.gn` and `//testing/testing.gni` for the build rules and
GN template definitions that determine which files get included and compiled
here.
