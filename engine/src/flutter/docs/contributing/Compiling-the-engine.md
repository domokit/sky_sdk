_If you've never built the engine before, first see [Setting up the Engine development environment](Setting-up-the-Engine-development-environment.md)._

# Contents

Depending on the platform you are making changes for, you may be interested in all or only some of the sections below:

* [General Compilation Tips](#general-compilation-tips)
* [Using a custom Dart SDK](#using-a-custom-dart-sdk)
* [Compiling for Android](#compiling-for-android-from-macos-or-linux)
* [Compiling for iOS (from macOS)](#compiling-for-ios-from-macos)
* [Compiling for macOS or Linux](#compiling-for-macos-or-linux)
* [Compiling for Windows](#compiling-for-windows)
* [Compiling for Fuchsia](#compiling-for-fuchsia)
* [Compiling for the Web](#compiling-for-the-web)
* [Compiling for testing](#compiling-for-testing)

## General Compilation Tips

- For local development and testing, it's generally preferable to use `--unopt` builds.
  These builds will have additional logging and checks enabled, and generally use build
  and link flags that lead to faster compilation and better debugging symbols.
  If you are trying to do performance testing with a local build, do not use the `--unopt`
  flag.
- Link Time Optimization: Optimized builds also perform Link Time Optimization of all
  binaries. This makes the linker take a lot of time and memory to produce binaries. If
  you need optimized binaries but don't want to perform LTO, add the `--no-lto` flag.
- Android and iOS expect both a `host` and `android` (or `ios`) build. It is critical to
  recompile the host build after upgrading the Dart SDK (e.g. via a `gclient sync` after
  merging up to head), since artifacts from the host build need to be version matched to
  artifacts in the Android/iOS build.
- Web, Desktop, and Fuchsia builds have only one build target (i.e. `host` or `fuchsia`).
- Make sure to exclude the `out` directory from any backup scripts, as many large binary
  artifacts are generated. This is also generally true for all of the directories outside
  of the `engine/src/flutter` directory.

## Using a custom Dart SDK

When targeting the host and desktop, on CI we use a pre-built Dart SDK vended by the Dart team.
To build and use the SDK from the Dart sources downloaded by `gclient sync`, after editing those
source files, pass the flag `--no-prebuilt-dart-sdk` to `//flutter/tools/gn`.

## Compiling for Android (from macOS or Linux)

These steps build the engine used by `flutter run` for Android devices.

Run the following steps, from the `src` directory created in [Setting up the Engine development environment](Setting-up-the-Engine-development-environment.md):

1. `git pull upstream main` in `src/flutter` to update the Flutter Engine repo.

2. `gclient sync` to update dependencies.

3. Prepare your build files
    * `./flutter/tools/gn --android --unoptimized` for device-side executables.
    * `./flutter/tools/gn --android --android-cpu arm64 --unoptimized` for newer 64-bit Android devices.
    * `./flutter/tools/gn --android --android-cpu x86 --unoptimized` for x86 emulators.
    * `./flutter/tools/gn --android --android-cpu x64 --unoptimized` for x64 emulators.
    * `./flutter/tools/gn --unoptimized` for host-side executables, needed to compile the code.
      * On Apple Silicon ("M" chips), add `--mac-cpu arm64` to avoid using emulation. This will generate `host_debug_unopt_arm64`.

> 💡 **TIP**: When developing on a Mac with ARM (M CPU), prefer `host_debug_unopt_arm64`.
>
> You can continue to use `host_debug_unopt` (required for Intel Macs), but the engine will be run under Rosetta
> which may be slower. See [Developing with Flutter on Apple Silicon](https://github.com/flutter/flutter/blob/master/docs/platforms/desktop/macos/Developing-with-Flutter-on-Apple-Silicon.md)
> for more information.

4. Build your executables
    * `ninja -C out/android_debug_unopt` for device-side executables.
    * `ninja -C out/android_debug_unopt_arm64` for newer 64-bit Android devices.
    * `ninja -C out/android_debug_unopt_x86` for x86 emulators.
    * `ninja -C out/android_debug_unopt_x64` for x64 emulators.
    * `ninja -C out/host_debug_unopt` (or `ninja -C out/host_debug_unopt_arm64`, see above) for host-side executables.
    * These commands can be combined. Ex: `ninja -C out/android_debug_unopt && ninja -C out/host_debug_unopt`
    * For MacOS, you will need older version of XCode(9.4 or below) to compile android_debug_unopt and android_debug_unopt_x86. If you only care about x64, you can ignore this

This builds a debug-enabled ("unoptimized") binary configured to run Dart in
checked mode ("debug"). There are other versions, see [Flutter's modes](../Flutter's-modes.md).

If you're going to be debugging crashes in the engine, make sure you add
`android:debuggable="true"` to the `<application>` element in the
`android/AndroidManifest.xml` file for the Flutter app you are using
to test the engine.

See [The flutter tool](https://github.com/flutter/flutter/blob/master/docs/tool/README.md) for instructions on how to use the `flutter` tool with a local engine.
You will typically use the `android_debug_unopt` build to debug the engine on a device, and
`android_debug_unopt_x64` to debug in on a simulator. Modifying dart sources in the engine will
require adding a `dependency_override` section in you app's `pubspec.yaml` as detailed
[here](https://github.com/flutter/flutter/blob/master/docs/tool/README.md#using-a-locally-built-engine-with-the-flutter-tool).

Note that if you use particular android or ios engine build, you will need to have corresponding
host build available next to it: if you use `android_debug_unopt`, you should have built `host_debug_unopt`,
`android_profile` -> `host_profile`, etc. One caveat concerns cpu-flavored builds like `android_debug_unopt_x86`: you won't be able to build `host_debug_unopt_x86` as that configuration is not supported. What you are expected to do is to build `host_debug_unopt` and symlink `host_debug_unopt_x86` to it.

### Compiling everything that matters on Linux

The following script will update all the builds that matter if you're developing on Linux and testing on Android and created the `.gclient` file in `~/dev/engine`:

```bash
set -ex

cd ~/dev/engine/src/flutter
git fetch upstream
git rebase upstream/main
gclient sync
cd ..

flutter/tools/gn --unoptimized --runtime-mode=debug
flutter/tools/gn --android --unoptimized --runtime-mode=debug
flutter/tools/gn --android --runtime-mode=profile
flutter/tools/gn --android --runtime-mode=release

cd out
find . -mindepth 1 -maxdepth 1 -type d | xargs -n 1 sh -c 'ninja -C $0 || exit 255'
```
For `--runtime-mode=profile` build, please also consider adding `--no-lto` option to the `gn` command. It will make linking much faster with a small sacrifice on the binary size and memory usage (which probably doesn't matter for debugging or performance benchmark purposes.)

## Compiling for iOS (from macOS)

These steps build the engine used by `flutter run` for iOS devices.

Run the following steps, from the `src` directory created in the steps above:

1. `git pull upstream main` in `src/flutter` to update the Flutter Engine repo.

2. `gclient sync` to update dependencies.

3. `./flutter/tools/gn --ios --unoptimized` to prepare build files for device-side executables (or `--ios --simulator --unoptimized` for simulator).
   * This also produces an Xcode project for working with the engine source code at `out/ios_debug_unopt/flutter_engine.xcodeproj`
   * For a discussion on the various flags and modes, see [Flutter's modes](../Flutter's-modes.md).
   * Add the `--simulator-cpu=arm64` argument for an arm64 Mac simulator to output to `out/ios_debug_sim_unopt_arm64`.

4. `./flutter/tools/gn --unoptimized` to prepare the build files for host-side executables.
   * On Apple Silicon ("M" chips), add `--mac-cpu arm64` to avoid using emulation. This will generate `host_debug_unopt_arm64`.

5. `ninja -C out/ios_debug_unopt && ninja -C out/host_debug_unopt` to build all artifacts (use `out/ios_debug_sim_unopt` for Simulator).

See [The flutter tool](https://github.com/flutter/flutter/blob/master/docs/tool/README.md) for instructions on how to use the `flutter` tool with a local engine.
You will typically use the `ios_debug_unopt` build to debug the engine on a device, and
`ios_debug_sim_unopt` to debug in on a simulator. Modifying dart sources in the engine will
require adding a `dependency_override` section in you app's `pubspec.yaml` as detailed
[here](https://github.com/flutter/flutter/blob/master/docs/tool/README.md#using-a-locally-built-engine-with-the-flutter-tool).

See also [instructions for debugging the engine in a Flutter app in Xcode](../Debugging-the-engine.md#debugging-ios-builds-with-xcode).

## Compiling for macOS or Linux

These steps build the desktop embedding, and the engine used by `flutter test` on a host workstation.

1. `git pull upstream main` in `src/flutter` to update the Flutter Engine repo.

2. `gclient sync` to update your dependencies.

3. `./flutter/tools/gn --unoptimized` to prepare your build files.
   * `--unoptimized` disables C++ compiler optimizations. On macOS, binaries are emitted unstripped; on Linux, unstripped binaries are emitted to an `exe.unstripped` subdirectory of the build.

4. `ninja -C out/host_debug_unopt` to build a desktop unoptimized binary.
    * If you skipped `--unoptimized`, use `ninja -C out/host_debug` instead.

See [The flutter tool](https://github.com/flutter/flutter/blob/master/docs/tool/README.md) for instructions on how to use the `flutter` tool with a local engine.
You will typically use the `host_debug_unopt` build in this setup. Modifying dart sources in the engine will
require adding a `dependency_override` section in you app's `pubspec.yaml` as detailed
[here](https://github.com/flutter/flutter/blob/master/docs/tool/README.md#using-a-locally-built-engine-with-the-flutter-tool).


## Compiling for Windows

> [!WARNING]
> You can only build selected binaries on Windows (mainly `gen_snapshot` and the desktop embedding).

On Windows, ensure that the engine checkout is not deeply nested. This avoid the issue of the build scripts working with excessively long paths.

1. Make sure you have Visual Studio installed (non-Googlers only). [Debugging Tools for Windows 10](https://docs.microsoft.com/en-us/windows-hardware/drivers/debugger/debugger-download-tools#small-classic-windbg-preview-logo-debugging-tools-for-windows-10-windbg) must be installed.

2. `git pull upstream main` in `src/flutter` to update the Flutter Engine repo.

3. Ensure long path support is enabled on your machine. Launch PowerShell as an administrator and run:
```
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -Force
```

4. If you are not a Google employee, you must set the following environment variables to point the depot tools at Visual Studio:
```shell
DEPOT_TOOLS_WIN_TOOLCHAIN=0
GYP_MSVS_OVERRIDE_PATH="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community" # (or your location for Visual Studio)
WINDOWSSDKDIR="C:\Program Files (x86)\Windows Kits\10" # (or your location for Windows Kits)
```
Also, be sure that Python27 is before any other python in your Path.

5. `gclient sync` to update your dependencies.

6. switch to `src/` directory.

7. `python .\flutter\tools\gn --unoptimized` to prepare your build files.
   * If you are only building `gen_snapshot`: `python .\flutter\tools\gn [--unoptimized] --runtime-mode=[debug|profile|release] [--android]`.

8. `ninja -C .\out\<dir created by previous step>` to build.
   * If you used a non-debug configuration, use `ninja -C .\out\<dir created by previous step> gen_snapshot`.
     Release and profile are not yet supported for the desktop shell.

## Compiling for Fuchsia

### Build components for Fuchsia

1. Building fuchsia is only supported on linux. You need to update `engine/.gclient`, or `../.gclient` if current directory is `engine/src`, with `custom_vars`.

```
solutions = [
  {
    # ...
    "custom_vars": {
      "download_fuchsia_deps": True,
      "run_fuchsia_emu": True,
    },
  },
]
```

> You may ignore `"run_fuchsia_emu": True` if you won't run tests locally.

Run `gclient sync`.

> [!WARNING]
> When running tests locally, you will also need kvm enabled, or nested virtualization on the gcloud VMs. Fuchsia and the tests will all be executed on the qemu.

2. Prepare and build

```
./flutter/tools/gn --fuchsia --no-lto
```

  * It will create a `out/fuchsia_debug_x64`.
  * Use `--fuchsia-cpu arm64` to build components for arm64. It will be created in a folder `out/fuchsia_debug_arm64`.
  * Use `--runtime-mode=release` or `--runtime-mode=profile` to select other profiles as other platforms.
  * Ignore `--no-lto` to use lto or link-time optimization.

```
ninja -C out/fuchsia_debug_x64 -k 0
```

  * It builds all but ignores known errors.
  * Or specify following targets to avoid using `-k 0`.

```
flutter/shell/platform/fuchsia:fuchsia \
flutter/shell/platform/fuchsia/dart_runner:dart_runner_tests \
fuchsia_tests
```

  * Use `autoninja` if it's available.
  * `-C out/fuchsia_release_x64` for release build; other configurations are similar with a different folder name in `out/`.

3. Run all tests locally

```
python3 flutter/tools/fuchsia/with_envs.py flutter/testing/fuchsia/run_tests.py
```

  * It runs the tests in `out/fuchsia_debug_x64` by default. According to the configuration, it may take 5 minutes with regular gtest output to the terminal.
  * Add `fuchsia_release_x64` at the end of the command for release build; other configurations are similar with a different folder name in `out/`.

```
python3 flutter/tools/fuchsia/with_envs.py flutter/testing/fuchsia/run_tests.py fuchsia_release_x64
```

## Compiling for the Web

For building the engine for the Web we use the [felt](https://github.com/flutter/engine/blob/main/lib/web_ui/README.md) tool.

To test Flutter with a local build of the Web engine, add `--local-web-sdk=wasm_release` to your `flutter` command, e.g.:

```
flutter run --local-web-sdk=wasm_release -d chrome
flutter test --local-web-sdk=wasm_release test/path/to/your_test.dart
```

## Compiling for the Web on Windows

Compiling the web engine might take a few extra steps on Windows. Use cmd.exe and "run as administrator".

1. Make sure you have Visual Studio installed. Set the following environment variables. For Visual Studio use the path of the version you installed.
   * `GYP_MSVS_OVERRIDE_PATH = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community"`
   * `GYP_MSVS_VERSION = 2017`
2. Make sure, depot_tools, ninja and python are installed and added to the path. Also set the following environment variable for depot tools:
   * `DEPOT_TOOLS_WIN_TOOLCHAIN = 0`
   * Tip: if you get a python error try to use Python 2 instead of 3
3. `git pull upstream main` in `src/flutter` to update the Flutter Engine repo.
4. `gclient sync` to update your dependencies.
   * Tip: If you get a git authentication errors on this step try Git Bash instead
5. `python .\flutter\tools\gn --unoptimized --full-dart-sdk` to prepare your build files.
6. `ninja -C .\out\<dir created by previous step>` to build.

To test Flutter with a local build of the Web engine, add `--local-web-sdk=wasm_release` to your `flutter` command, e.g.:

```
flutter run --local-web-sdk=wasm_release -d chrome
flutter test --local-web-sdk=wasm_release test/path/to/your_test.dart
```

For testing the engine again use [felt](https://github.com/flutter/engine/blob/main/lib/web_ui/README.md) tool
this time with felt_windows.bat.

```
felt_windows.bat test
```

## Compiling for testing

### Dart tests

To run dart tests, build the engine:

```
flutter/tools/gn --unoptimized
ninja -C out/host_debug_unopt/
```

execute `run_tests` for native:
```
python3 flutter/testing/run_tests.py --type dart
```

and `felt` for web:
```
cd flutter/lib/web_ui
dev/felt test [test file]
```


## Troubleshooting Compile Errors

### Version Solving Failed

From time to time, as the Dart versions increase, you might see dependency errors such as:

```
The current Dart SDK version is 2.7.0-dev.0.0.flutter-1ef444139c.

Because ui depends on <a pub package> 1.0.0 which requires SDK version >=2.7.0 <3.0.0, version solving failed.
```

Running `gclient sync` does not update the tags, there are two solutions:
1. under `engine/src/third_party/dart` run `git fetch --tags origin`
2. or run gclient sync with with tags parameter: `gclient sync --with_tags`

_See also: [Debugging the engine](../Debugging-the-engine.md), which includes instructions on running a Flutter app with a local engine._
