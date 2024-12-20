# Flutter macOS Embedder

This directory contains files specific to the Flutter macOS embedder. The
embedder is built as a macOS framework that is linked into the target
application. The framework is composed both of macOS-specific code and code
shared with the iOS embedder. These can be found in:
```
flutter/shell/platform/darwin/common/framework
flutter/shell/platform/darwin/macos/framework
```

Additionally, the framework relies on utility code shared across all embedders,
which is found in:
```
flutter/shell/platform/common
```

To learn more, see the [Engine architecture wiki][wiki_arch].

## Building

Building all artifacts required for macOS occurs in two steps:
* Host binary build: Builds tooling used by the Flutter tool to build Flutter
  applications targeting macOS.
* macOS Target build: Builds the framework that implements the macOS Flutter
  embedder and exposes public API such as `FlutterViewController` used by
  Flutter applications.

Once you've [prepared your environment for engine development][wiki_engine_env],
you can build the macOS embedder from the `src/flutter` directory using the
following commands:
```sh
# Perform the host build.
./tools/gn --unopt
ninja -C ../out/host_debug_unopt

# Perform the macOS target build.
./tools/gn --unopt --mac
ninja -C ../out/mac_debug_unopt
```
Builds are architecture-specific, and can be controlled by specifying
`--mac-cpu=arm64` or `--mac-cpu=x64` (default) when invoking `gn`.

## Testing

The macOS-specific embedder tests are built as the
`flutter_desktop_darwin_unittests` binary. Like all gtest-based test binaries, a
subset of tests can be run by applying a filter such as
`--gtest_filter='FlutterViewControllerTest.*Key*'`.

More general details on testing can be found on the [Wiki][wiki_engine_testing].

[wiki_arch]: https://github.com/flutter/flutter/wiki/The-Engine-architecture
[wiki_engine_env]: https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment
[wiki_engine_testing]: https://github.com/flutter/flutter/wiki/Testing-the-engine
