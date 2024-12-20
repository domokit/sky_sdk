# Running the Fuchsia unit tests locally

These instructions assume you have set `$FUCHSIA_DIR` to your Fuchsia checkout
and `$ENGINE_DIR` to the `src/` folder of your Engine checkout. For example for
zsh, add these lines to your `~/.zprofile`:

```sh
export FUCHSIA_DIR=~/fuchsia
export ENGINE_DIR=~/engine/src
```

1. In a separate terminal, start a Fuchsia package server:

```sh
cd "$FUCHSIA_DIR"
fx serve
```

2. Run the unit tests:

```sh
$ENGINE_DIR/flutter/tools/fuchsia/devshell/run_unit_tests.sh
```

- Pass `--unopt` to turn off C++ compiler optimizations.
- Pass `--count N` to do N test runs. Useful for testing for flakes.
- Pass `--package-filter` to run a specific test package instead of all the test packages. For example:

  ```sh
  $ENGINE_DIR/flutter/tools/fuchsia/devshell/run_unit_tests.sh --package-filter flow_tests-0.far
  ```

- Pass `--gtest-filter` to run specific tests from the test package instead of all the tests. For example:

  ```sh
  $ENGINE_DIR/flutter/tools/fuchsia/devshell/run_unit_tests.sh --package-filter flutter_runner_tests-0.far --gtest-filter "*FlatlandConnection*"
  ```
