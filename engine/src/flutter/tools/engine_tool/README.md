# Engine Tool

🔗 Permalink: [flutter.dev/to/et](https://flutter.dev/to/et)

---

`et`, or _engine tool_, is a command-line tool that intends to provide a
unified interface for building and working in the flutter engine.

[![Open `e: engine-tool` issues](https://img.shields.io/github/issues/flutter/flutter/e%3A%20engine-tool)](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22e%3A+engine-tool%22)

<img width="500" src="https://github.com/user-attachments/assets/879e6442-016d-40e5-909c-f3210065d878" />

**Table of Contents**:

- [Getting Started](#getting-started)
- [Understanding the concept of a build configuration](#understanding-the-concept-of-a-build-configuration)
- [Common Tasks](#common-tasks)
  - [Building a host engine](#building-a-host-engine)
  - [Building a target engine](#building-a-target-engine)
  - [Building specific targets](#building-specific-targets)
  - [Running C++ tests](#running-c-tests)
  - [Running formatters](#running-formatters)
  - [Running linters](#running-linters)
  - [Running a Flutter app with a local engine build](#running-a-flutter-app-with-a-local-engine-build)
- [Advanced Features](#advanced-features)
  - [Enabling remote build execution](#enabling-remote-build-execution)
  - [Running Dart tests](#running-dart-tests)
  - [Using a custom engine configuration](#using-a-custom-engine-configuration)
  - [Reclaiming older output directories](#reclaiming-older-output-directories)
- [Contributing](#contributing)

## Getting Started

`et` assumes that you have a working knowledge of the Flutter framework and
engine (see: [architectural layers][]), and have a valid checkout of the engine
source on a supported platform (see:
[setting up the engine development environment][]); in fact, the tool will not
run at all outside of a valid repository setup.

[architectural layers]: https://docs.flutter.dev/resources/architectural-overview#architectural-layers
[setting up the engine development environment]: https://github.com/flutter/engine/blob/main/docs/contributing/Setting-up-the-Engine-development-environment.md

It is recommended to add `et` to your `PATH` by adding the [`bin` folder][]:

```sh
PATH=$PATH:/path/to/engine/flutter/bin
```

[`bin` folder]: ../../bin

To verify you have a working installation, try `et help`:

```sh
$ et help
A command line tool for working on the Flutter Engine.

This is a community supported project, file a bug or feature request:
https://flutter.dev/to/engine-tool-bug.

Usage: et <command> [arguments]

Global options:
-h, --help       Print this usage information.
-v, --verbose    Prints verbose output

Available commands:
  build    Builds the engine
  fetch    Download the Flutter engine's dependencies
  format   Formats files using standard formatters and styles.
  lint     Lint the engine repository.
  query    Provides information about build configurations and tests.
  run      Run a Flutter app with a local engine build.
  test     Runs a test target

Run "et help <command>" for more information about a command.
```

## Understanding the concept of a build configuration

Many commands in `et` use or reference a _build configuration_, often explicitly
specified using `--config` (or the short-hand `-c`).

A build configuration has at _least_:

- A known platform the build can run on (`drone_dimensions`);
- Compile-time flags used to configure the build (`gn`);
- A human-readable name and description (`name`, `description`);

Build configurations are _typically_ defined in
[`ci/builders`](../../ci/builders/), where they either live in a task-specific
configuration file (i.e. to build and run tests on CI), such as
[`mac_unopt.json`](../../ci/builders/mac_unopt.json), or one of many
configurations that are used only for local development and iteration in
[`local_engine.json`](../../ci/builders/local_engine.json). The builds specified
in these files are referenced _by name_ as `--config`:

```sh
# Implicitly references ci/builders/mac_unopt.json
et build --config ci/host_debug_unopt_arm64

# Implicitly references ci/builders/local_engine.json
et build --config host_debug_unopt_arm64
```

See [building a host engine](#building-a-host-engine) and
[building a target engine](#building-a-target-engine) for more details.

> [!CAUTION]
> Each build configuration (sometimes called a _variant_) produces a different
> set of output files in `$ENGINE/src/out`, e.g. ``$ENGINE/src/out/host_debug`;
> these outputs can be multiple GBs, and add up quickly. Consider using
> [`et cleanup`](#reclaiming-older-output-directories) to delete older output
> directories automatically.

## Common Tasks

Tasks we expect the majority of contributors and users of the engine to need.

### Building a host engine

The default and most common operation is to build a _host_ variant of the
engine, or an engine that runs on the local (desktop) operating system currently
being used. For example, when running on an ARM64 macOS laptop, an ARM64 macOS
desktop engine build, or when running an x64 Linux desktop, a x64 Linux desktop
engine build.

```sh
# Builds the current platform's (host) debug build.
et build

# Equivalent to the above.
et build --config host_debug
```

> [!TIP]
> To understand where the names come from, see
> [understanding the concept of a build configuration](#understanding-the-concept-of-a-build-configuration).

A host engine is useful when:

- You want to test, debug, or iterate functionality independent of a specific device or platform;
- You are working on functionality specific to the (desktop) platform you currently have;
- You want to use a combination of a host engine and target engine to [run a Flutter app](#running-a-flutter-app-with-a-local-engine-build).

### Building a target engine

The Flutter engine supports multiple _target_ engines, or engines not native to
the _current_ desktop-class operating system, such as Android or iOS. For
example, when running on a MacOS laptop or desktop, you can build an iOS
simulator or application engine:

```sh
# Builds an iOS device (non-simulator) engine.
et build --config ios_debug

# Builds an iOS simulator engine.
et build --config ios_debug_sim
```

By convention, target engines are not prefixed with `host`.

A target engine is useful when:

- You are working on functionality specific to the (target) platform;
- You want to use a combination of a host engine and target engine to [run a Flutter app](#running-a-flutter-app-with-a-local-engine-build).

### Building specific targets

By default, the _entire engine_ is built.

For example these commands are equivalent:

```sh
et build --config host_debug

et build --config host_debug //flutter/...
```

While caching often avoids rebuilding parts of the engine that have not changed,
sometimes you as the developer will have more information than the dependency
tree on what exactly needs to be rebuilt between changes. To build _specific_
targets, provide the fully qualified path to the `GN` target:

```sh
# Builds only the "flutter.jar" artifact.
et build --config android_debug_unopt_arm64 //flutter/shell/platform/android:android_jar
```

To build all targets in particular directory, _recursively_, use `/dirname/...`:

```sh
# Builds all targets, recursively, in //flutter/shell/platform.
et build --config android_debug_unopt_arm64 //flutter/shell/platform/...
```

To build all targets in particular directory, _non-recursively_, use `:all`:

```sh
# Builds all targets, non-recursively, in //flutter/shell/platform.
et build --config android_debug_unopt_arm64 //flutter/shell/platform:all
```

### Running C++ tests

C++ unit tests can be simulatenously rebuilt and run using `et test`:

```sh
et test //flutter/impeller:impeller_unittests
```

Both `/...` and `:all` are supported as well.

> [!NOTE]
> Support for non-C++ tests is limited. See [running Dart tests](#running-dart-tests).

### Running formatters

To run all formatters on _changed_ files, use `et format`:

```sh
et format
```

Sometimes a dependency change (or change in a tool) might invalidate the format
checks of a file you did not change (dirty):

```sh
# Checks *all* files, which is *much* slower!
et format --all
```

### Running linters

Similar to formatters, global linters can be run with `et lint`:

```sh
et lint
```

At the time of this writing, the linters always operate on the entire
repository.

### Running a Flutter app with a local engine build

Normally to run a Flutter application with a prebuilt engine, you'd run:

```sh
cd to/project/dir
flutter run
```

While iterating on the engine source, you may want to use the engine outputs
(both host and target) built above. `et run` can help:

```sh
cd to/project/dir
et run
```

> [!NOTE] > `et run` will rebuild (if necessary) host and target builds, which can take
> a significant amount of time.

## Advanced Features

Tasks that might be restricted to a subset of the engine team, or for upstream
users (such as developers that work on the Dart VM or SDK teams). Features may
be in a somewhat incomplete state compared to common tasks, or might be a bit
less intuitive or easy to use.

### Enabling remote build execution

Google employees have the option of using remote-build execution, or RBE, to
greatly speed up many builds by reusing previously built (and cached) artifacts
as well as delegating the compiler to a high-powered (remote) virtual machine.

To enable RBE, follow [flutter.dev/to/engine-rbe](https://flutter.dev/to/engine-rbe).

Once enabled, by default, `et` builds will use RBE where possible, which also
(implicitly) requires an active internet connection. It is possible to
temporarily (for one command) change whether to prefer remote builds or
exclusively use local builds by using `--build-strategy`:

```sh
# Exclusively builds locally, which can be faster for some incremental builds.
# Does not require an internet connection.
et build --build-strategy=local

# Exclusively builds remotely, which is less taxing on your local machine.
# Requires a fast internet connection.
et build --build-strategy=remote
```

To _disable_ RBE once it is enabled, a build can use `--no-rbe`:

```sh
et build --no-rbe
```

> [!CAUTION]
> Disabling RBE invalidates the build context, which means that previously built
> artifacts (when the flag was enabled) are _not_ re-used. It is recommended to
> use `--build-strategy=local` instead unless you are debugging the tool or the
> RBE configuration itself.

### Running Dart tests

There is limited support for running _Dart_ unittests using `et`:

```sh
et test //flutter/tools/engine_tool/...
```

> [!NOTE]
> Unlike C++, it is not currently required to have `BUILD.gn` targets declared
> for Dart tests, and the vast majority of packages do not have them. As we add
> and adopt GN more broadly this command will become more generally useful.

### Using a custom engine configuration

Most of the time developers will use a pre-configured engine configuration
(see [understanding the concept of a build configuration](#understanding-the-concept-of-a-build-configuration))
as these configurations are already generally supported, and often tested on CI.
If you need to build a configuration _not-specified_, consider the following:

1. Does my configuration represent a combination of flags that should be
   tested on CI or re-used by others?

   If so, the best option may be adding the build to
   [ci/builders](../../ci/builders/), either as a CI build or merely as a
   [local engine build](../../ci/builders/local_engine.json). By adding your
   build here it will be reproducible for other developers, documented, and
   automatically usable within the `et` command-line tool.

2. Is my configuration for 1-off testing or validation only?

   If so, any combination of _additional_ GN arguments (i.e. arguments that
   otherwise would be parsed by [tools/gn](../gn)) can be provided by passing
   `--gn-args`, often in conjunction with an existing configuration template.

   For example, using link-time optimization (LTO):

   ```sh
   et build --config host_release --lto
   ```

   Or, using a from-source Dart SDK (often used by Dart SDK and VM developers):

   ```sh
   et build --config host_debug --gn-args="--no-prebuilt-dart-sdk"
   ```

> [!TIP]
> For more information on [build configurations, see the README](../../ci/builders/README.md).

### Reclaiming older output directories

The `et cleanup` command removes older output directories that have not been
accessed, by default in the last 30 days, but customizable with the command-line
argument `--untouched-since`.

Consider using `dry-run` to preview what _would_ be deleted:

```sh
# Deletes all output directories older than 30 days.
et cleanup

# Shows what output directories would be deleted by the above command.
et cleanup --dry-run

# Deletes all output directories accessed last in 2023.
et cleanup --untouched-since=2024-01-01
```

## Contributing

We welcome contributions to improve `et` for our all developers.

- Follow the [Flutter style guide](https://github.com/flutter/flutter/wiki/Style-guide-for-Flutter-repo)
  for Dart code that are relevant outside of the framework repo. It contains
  conventions that go beyond code formatting, which we'll follow even if using
  `dart format` in the future.
- Do not call directly into `dart:io` except from `main.dart`. Instead access
  the system only through the `Enviroment` object.
- All commands must have unit tests. If some functionality needs a fake
  implementation, then write a fake implementation.
- When adding or changing functionality, update this README.md file.
- _Begin with the end in mind_ - Start working from what the interface provided
  by this tool _should_ be, then modify underlying scripts and tools to provide
  APIs to support that.

Run tests using `et`:

```shell
et test //flutter/tools/engine_tool/...
```

If you're not sure what to work on, consider our existing label of `e: engine-tool`:

[![Open `e: engine-tool` issues](https://img.shields.io/github/issues/flutter/flutter/e%3A%20engine-tool)](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22e%3A+engine-tool%22)
