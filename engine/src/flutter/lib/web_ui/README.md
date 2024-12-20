# Flutter Web Engine

This directory contains the source code for the Web Engine.

## Hacking on the Web Engine

If you are setting up a workspace for the first time, start by following the
instructions at [Setting up the Engine development environment][1]. In addition,
it is useful to add the following to your `PATH` environment variable:

- `ENGINE_ROOT/src/flutter/lib/web_ui/dev`, so you can run the `felt` command
  from anywhere.
- `FLUTTER_ROOT/bin`, so you can run `dart` and `flutter` commands from
  anywhere.

### Using `felt`

`felt` (stands for "Flutter Engine Local Tester") is a command-line tool that
aims to make development in the Flutter web engine more productive and pleasant.

To tell `felt` to do anything you call `felt SUBCOMMAND`, where `SUBCOMMAND` is
one of the available subcommands, which can be listed by running `felt help`. To
get help for a specific subcommand, run `felt help SUBCOMMAND`.

#### `felt build`
The `build` subcommand builds web engine gn/ninja targets. Targets can be
individually specified in the command line invocation, or if none are specified,
all web engine targets are built. Common targets are as follows:
  * `sdk` - The flutter_web_sdk itself.
  * `canvaskit` - Flutter's version of canvakit.
  * `canvaskit_chromium` - A version of canvaskit optimized for use with
    chromium-based browsers.
  * `skwasm` - Builds experimental skia wasm module renderer.
The output of these steps is used in unit tests, and can be used with the flutter
command via the `--local-web-sdk=wasm_release` command.

The `build` command also accepts either the `--profile` or `--debug` flags, which
can be used to change the build profile of the artifacts.

##### Examples
Builds all web engine targets, then runs a Flutter app using it:
```
felt build
cd path/to/some/app
flutter --local-web-sdk=wasm_release run -d chrome
```

Builds only the `sdk` and the `canvaskit` targets:
```
felt build sdk canvaskit
```

#### `felt test`
The `test` subcommand will compile and/or run web engine unit test suites. For
information on how test suites are structured, see the test configuration
[readme][2].

By default, `felt test` compiles and runs all suites that are compatible with the
host system. Some useful flags supported by this command:
  * Action flags which say what parts of the test pipeline to perform. More of one
    of these can be specified to run multiple actions. If none are specified, then
    *all* of these actions are performed
    * `--compile` performs compilation of the test bundles.
    * `--run` runs the unit tests
    * `--copy-artifacts` will copy build artifacts needed for the tests to run.
      * The `--profile` or `--debug` flags can be specified to copy over artifacts
        from the profile or debug build folders instead of release.
  * `--list` will list all the test suites and test bundles and exit without
    compiling or running anything.
  * `--verbose` will output some extra information that may be useful for debugging.
  * `--start-paused` will open a browser window and pause the tests before starting
    so that breakpoints can be set before starting the test suites.

Several other flags can be passed that filter which test suites should be run:
  * `--browser` runs only the test suites that test on the browsers passed. Valid
    values for this are `chrome`, `firefox`, `safari`, or `edge`.
  * `--compiler` runs only the test suites that use a particular compiler. Valid
    values for this are `dart2js` or `dart2wasm`
  * `--renderer` runs only the test suites that use a particular renderer. Valid
    values for this are `html`, `canvakit`, or `skwasm`
  * `--suite` runs a suite by name.
  * `--bundle` runs suites that target a particular test bundle.

Filters of different types are logically ANDed together, but multiple filter flags
of the same type are logically ORed together.

The `test` command will also accept a list of paths to specific test files to be
compiled and run. If none of these paths are specified, all tests are run, otherwise
only the tests that are specified will run.

##### Examples
Runs all test suites in all compatible browsers:
```
felt test
```
Runs a specific test on all compatible browsers:
```
felt test test/engine/util_test.dart
```
Runs multiple specific tests on all compatible browsers:
```
felt test test/engine/util_test.dart test/engine/alarm_clock_test.dart
```
Runs only test suites that compile via dart2wasm:
```
felt test --compiler dart2wasm
```
Runs only test suites that run in Chrome and Safari:
```
felt test --browser chrome --browser safari
```

### Optimizing local builds

Concurrency of various build steps can be configured via environment variables:

- `FELT_COMPILE_CONCURRENCY` specifies the number of concurrent compiler
  processes used to compile tests. Default value is 8.

If you are a Google employee, you can use an internal instance of Goma (go/ma)
to parallelize your ninja builds. Because Goma compiles code on remote servers,
this option is particularly effective for building on low-powered laptops.

### Test browsers

Chromium, Firefox, and Safari for iOS are version-locked using the
[package_lock.yaml][3] configuration file. Safari for macOS is supplied by the
computer's operating system. Tests can be run in Edge locally, but Edge is not
enabled on LUCI. Chromium is used as a proxy for Chrome, Edge, and other
Chromium-based browsers.

Changing parameters in the browser lock is effective immediately when running
tests locally. To make changes effective on LUCI follow instructions in
[Rolling Browsers][#rolling-browsers].

### Rolling browsers

When running tests on LUCI using Chrome, LUCI uses the version of Chrome for 
Testing fetched from CIPD.

Since the engine code and infra recipes do not live in the same repository
there are few steps to follow in order to upgrade a browser's version.

### Rolling fallback fonts

To generate new fallback font data and push the fallback fonts into a CIPD
package for engine unit tests to consume, run the following felt command:

```
cipd auth-login
felt roll-fallback-fonts --key=<Google Fonts API key>
```

You can obtain a GoogleFonts API key from here: https://developers.google.com/fonts/docs/developer_api#APIKey

This will take the following steps:
* Fetch a list of fonts from the Google Fonts API
* Download each font we use for fallbacks and calculate its unicode ranges
* Generate the `font_fallback_data.dart` file that is used in the engine
* Push the fonts up to a CIPD package called `flutter/flutter_font_fallbacks`
* Update the `DEPS` file in the engine to use the new version of the package

To perform all these steps except actually uploading the package to CIPD, pass
the `--dry-run` flag to the felt command.

NOTE: Because this script uses `fc-config`, this roll step only actually works
on Linux, not on macOS or Windows.

#### Chrome for Testing

Chrome for Testing is an independent project that gets rolled into Flutter
manually, and as needed. Flutter consumes a pre-built Chrome for Testing build.
The available versions of Chrome for Testing available can be found [here](https://googlechromelabs.github.io/chrome-for-testing/). To roll to a newer version:

- Make sure you have `depot_tools` installed (if you are regularly hacking on
  the engine code, you probably do).
- If not already authenticated with CIPD, run `cipd auth-login` and follow
  instructions (this step requires sufficient privileges; contact
  #hackers-infra-🌡 on [Flutter's Discord server](https://github.com/flutter/flutter/wiki/Chat)).
- Edit `dev/package_lock.yaml` and update the following values under `chrome`:
  - Set `version` to the full four part version number of the build of Chrome 
    for Testing you want to roll (for example, `118.0.5993.70`)
- Run `dart dev/package_roller.dart` and make sure it completes successfully.
  The script uploads the specified versions of Chromium (and Chromedriver) to the
  right locations in CIPD: [Chrome](https://chrome-infra-packages.appspot.com/p/flutter_internal/browsers/chrome),
  [Chromedriver](https://chrome-infra-packages.appspot.com/p/flutter_internal/browser-drivers/chrome).
- Send a pull request containing the above file changes. Newer versions of Chromium
  might break some tests or Goldens. Get those fixed too!

If you have questions, contact the Flutter Web team on Flutter Discord on the
\#hackers-web-🌍 channel.

#### Firefox

We test with Firefox on LUCI in the Linux Web Engine builder. The process for
rolling Firefox is even easier than Chromium. Simply update `package_lock.yaml`
with the latest version of Firefox, and run `package_roller.dart`.

#### .ci.yaml

After rolling Chrome and/or Firefox, also update the CI dependencies in
`.ci.yaml` to make use of the new versions. The lines look like

```yaml
      dependencies: >-
        [
          {"dependency": "chrome_and_driver", "version": "version:107.0"},
          {"dependency": "firefox", "version": "version:83.0"},
          {"dependency": "goldctl", "version": "git_revision:720a542f6fe4f92922c3b8f0fdcc4d2ac6bb83cd"}
        ]
```

##### **package_roller.dart**

The script has the following command-line options:

- `--dry-run` - The script will stop before uploading artifacts to CIPD. The location of the data will be reported at the end of the script, if the script finishes successfullyThe output of the script will be visible in /tmp/browser-roll-RANDOM_STRING
- `--verbose` - Greatly increase the amount of information printed to `stdout` by the script.

> Try the following!
>
> ```bash
> dart dev/package_roller.dart --dry-run --verbose
> ```

#### **Other browsers / manual upload**

In general, the manual process goes like this:

1. Dowload the binaries for the new browser/driver for each operating system
   (macOS, linux, windows).
2. Create CIPD packages for these packages (more documentation is available for
   Googlers at go/cipd-flutter-web)
3. Update the version in this repo. Do this by changing the related fields in
   `package_lock.yaml` file.

Resources:

1. Browser and driver CIPD [packages][4] (requires special access; ping
   hackers-infra on Discord for more information)
2. LUCI web [recipe][5]
3. More general reading on CIPD packages [link][6]

### Configuration files

`package_lock.yaml` contains the version of browsers we use to test Flutter for
web. Versions are not automatically updated whenever a new release is available.
Instead, we update this file manually once in a while.

`canvaskit_lock.yaml` locks the version of CanvasKit for tests and production
use.

## Building CanvasKit

To build CanvasKit locally, you must first set up your gclient config to
activate the Emscripten SDK, which is the toolchain used to build CanvasKit.
To do this, replace the contents of your .gclient file at the root of the
project (i.e. in the parent directory of the `src` directory) with:

```
solutions = [
  {
    "managed": False,
    "name": "src/flutter",
    "url": "git@github.com:<your_username_here>/engine.git",
    "custom_deps": {},
    "deps_file": "DEPS",
    "safesync_url": "",
    "custom_vars": {
      "download_emsdk": True,
    },
  },
]
```

Now run `gclient sync` and it should pull in the Emscripten SDK and activate it.

To build CanvasKit with `felt`, run:

```
felt build --build-canvaskit
```

This will build CanvasKit in `out/wasm_debug`. If you now run

```
felt test
```

it will detect that you have built CanvasKit and use that instead of the one
from CIPD to run the tests against.

### Upgrading the Emscripten SDK for the CanvasKit build

The version of the Emscripten SDK should be kept up to date with the version
used in the Skia build. That version can be found in
`third_party/skia/bin/activate-emsdk`. It will probably also be necessary to
roll the dependency on `third_party/emsdk` in DEPS to the same version as in
`third_party/skia/DEPS`.

Once you know the version for the Emscripten SDK, change the line in
`tools/activate_emsdk.py` which defines `EMSDK_VERSION` to match Skia.



[1]: https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment
[2]: https://github.com/flutter/flutter/blob/main/lib/web_ui/test/README
[3]: https://github.com/flutter/engine/blob/main/lib/web_ui/dev/package_lock.yaml
[4]: https://chrome-infra-packages.appspot.com/p/flutter_internal
[5]: https://cs.opensource.google/flutter/recipes/+/master:recipes/engine/web_engine.py
[6]: https://chromium.googlesource.com/chromium/src.git/+/main/docs/cipd_and_3pp.md#What-is-CIPD

## Unicode properties

We pull the unicode properties we need from `third_party/web_unicode`. See `third_party/web_unicode/README.md` for more details on how we generate Dart code from unicode properties.
