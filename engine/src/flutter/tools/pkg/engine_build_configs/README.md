## Overview

This package contains libraries and example code for working with the engine_v2
build config json files that live under `flutter/ci/builders`.

* `lib/src/build_config.dart`: Contains the Dart object representations of the
  build config json files.
* `lib/src/build_config_loader.dart`: Contains a helper class for loading all
  of the build configuration json files in a directory tree into the Dart
  objects.
* `lib/src/build_config_runner.dart`: Contains classes that run a loaded build
  config on the local machine.

There is some example code using these APIs under the `bin/` directory.

* `bin/check.dart`: Checks the validity of the build config json files. This
  runs on CI in pre and post submit in `ci/check_build_configs.sh` through
  `ci/builders/linux_unopt.json`.
* `bin/run.dart`: Runs one build from a build configuration on the local
  machine. It doesn't run generators or tests, and it isn't run on CI.

## Usage

### `run.dart` usage:


```
$ dart bin/run.dart [build config name] [build name]
```

For example:

```
$ dart bin/run.dart mac_unopt host_debug_unopt
```

The build config names are the names of the json files under ci/builders.
The build names are the "name" fields of the maps in the list of "builds".

### `check.dart` usage:

```
$ dart bin/check.dart [/path/to/engine/src]
```

The path to the engine source is optional when the current working directory is
inside of an engine checkout.
