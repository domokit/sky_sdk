...............................................................................
# Flutter Web Engine Test Suites
The flutter engine unit tests can be run with a number of different
configuration options that affect both compile time and run time. The
permutations of these options are specified in the `felt_config.yaml` file that
is colocated with this README. Here is an overview of the way the test suite
configurations are structured:

## `compile-configs`
Specifies how the tests should be compiled. Each compile config specifies the
following:
  * `name` - The name of the compile configuration.
  * `compiler` - What compiler is used to compile the tests. Currently we support
    `dart2js` and `dart2wasm` as values.
  * `renderer` - Which renderer to use when compiling the tests. Currently we
    support `html`, `canvaskit`, and `skwasm`.

## `test-sets`
A group of files that contain unit tests. Each test set specifies the following:
  * `name` - The name of the test set.
  * `directory` - The name of the directory under `flutter/lib/web_ui/test` that
    contains all the test files.

## `test-bundles`
Specifies a group of tests and a compile configuration of those tests. The output
of the test bundles appears in `flutter/lib/web_ui/build/test_bundles/<name>`
where `<name>` is replaced by the name of the bundle. Each test bundle may be used
by multiple test suites. Each test bundle specifies the following:
  * `name` - The name of the test bundle.
  * `test-set` - The name of the test set that contains the tests to be compiled.
  * `compile-config` - The name of the compile configuration to use.

## `run-configs`
Specifies the test environment that should be provided to a unit test. Each run
config specifies the following:
  * `name` - Name of the run configuration.
  * `browser` - The browser with which to run the tests. Valid values for this are
    `chrome`, `firefox`, `safari` or `edge`.
  * `canvaskit-variant` - An optionally supplied argument that forces the tests to
    use a particular variant of CanvasKit, either `full` or `chromium`. If none
    is specified, the engine will select the variant based on its normal selection
    logic.

## `test-suites`
This is a fully specified run of a group of unit tests. They specify the following:
  * `name` - Name of the test suite.
  * `test-bundle` - Which compiled test bundle to use when running the suite.
  * `run-config` - Which run configuration to use when runnin the tests.
  * `artifact-deps` - Which gn/ninja build artifacts are needed to run the suite.
    Valid values are `canvaskit`, `canvaskit_chromium` or `skwasm`.