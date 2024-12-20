# Scenario App: Android Tests and Test Runner

End-to-end tests and test infrastructure for the Flutter engine on Android.

> [!IMPORTANT]
> There are several **known issues** with this test suite:
>
> - [#144407](https://github.com/flutter/flutter/issues/144407): "Newer Android"
>   tests are missing expected cropping and rotation.
> - [#144365](https://github.com/flutter/flutter/issues/144365): Tests that use
>   `ExternalTextureFlutterActivity` sometimes do not render.
> - [#144232](https://github.com/flutter/flutter/issues/144232): Impeller Vulkan
>   sometimes hangs on emulators.
> - [#144352](https://github.com/flutter/flutter/issues/144352): Skia Gold
>   sometimes is missing expected diffs.

---

Top topics covered in this document include (but are not limited to):

- [Running the Tests](#running-the-tests)
- [Contributing](#contributing)
- [Project History](#project-history)
- [Troubleshooting](#troubleshooting)
- [Getting Help](#getting-help)

## Introduction

This package simulates a Flutter app that uses the engine (`dart:ui`) only, in
conjunction with Android-specific embedding code that simulates the use of the
engine in a real app (such as plugins and platform views).

A custom test runner, [`run_android_tests.dart`](bin/run_android_tests.dart), is
used to run the tests on a connected device or emulator, and report the results,
including screenshots (golden files) and logs from `adb logcat`.

In the following architecture diagram:

> <a
>   href="https://github.com/flutter/flutter/wiki/The-Engine-architecture"
>   title="The Engine architecture"> <img
>   width="300"
>   alt="Anatomy of a Flutter app"
>   src="https://raw.githubusercontent.com/flutter/engine/main/docs/app_anatomy.svg"
> /> </a>

- The Dart app is represented by [`lib/main.dart`](../lib/main.dart).
- There is no framework code.
- `dart:ui` and the engine are the same as in a real app.
- Android-specific application code is in this directory
  ([`android/`](./)).
- The runner is a _custom_ test runner,
  [`run_android_tests.dart`](bin/run_android_tests.dart).

### Scope of Testing

The tests in this package are end-to-end tests that _specifically_ exercise the
engine and Android-specific embedding code. They are not [unit tests][1] for the
engine or the framework, and are not designed to test the behavior of the
framework or specific plugins, but rather to simulate the use of a framework
or plugins downstream of the engine.

In other words, we test "does the engine work on Android?" without a dependency
on either the Flutter framework, Flutter tooling, or any specific plugins.

[1]: ../../../shell/platform/android

### Golden Comparisons

Many of the Android-specific interactions with the engine are visual, such as
[external textures](https://api.flutter.dev/flutter/widgets/Texture-class.html)
or [platform views](https://docs.flutter.dev/platform-integration/android/platform-views),
and as such, the tests in this package use golden screenshot file comparisons to
verify the correctness of the engine's output.

For example, in [`ExternalTextureTests_testMediaSurface`](https://flutter-engine-gold.skia.org/search?corpus=flutter-engine&include_ignored=false&left_filter=name%3DExternalTextureTests_testMediaSurface&max_rgba=0&min_rgba=0&negative=true&not_at_head=false&positive=true&reference_image_required=false&right_filter=&sort=descending&untriaged=true), a [video](app/src/main/assets/sample.mp4) is converted to an external texture and displayed in a Flutter app. The test takes a screenshot of the app and compares it to a golden file:

<img 
  alt="Two pictures, the top one Flutter and the bottom Android"
  src="https://github.com/flutter/flutter/assets/168174/e2c34b88-d03d-4732-87e4-a86c97d006c5"
  width="300"
/>

_The top picture is the Flutter app, and the bottom picture is just Android._

See also:

- [`tools/compare_goldens`](../../../tools/compare_goldens).

## Prerequisites

If you've never worked in the `flutter/engine` repository before, you will
need to setup a development environment that is quite different from a typical
Flutter app or even working on the Flutter framework. It will take roughly
30 minutes to an hour to setup an environment, depending on your familiarity
with the tools and the speed of your internet connection.

See also:

- [Setting up the Engine](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment)
- [Rebuilding the Tests](#rebuilding-the-tests)

### Android SDK

It's highly recommended to use the engine's vendored Android SDK, which once
you have the engine set up, you can find at
`$ENGINE/src/flutter/third_party/android_tools/sdk`. Testing or running with
other versions of the SDK may work, but it's _not guaranteed_, and might have
different results.

Consider also placing this directory in the `ANDROID_HOME` environment variable:

```sh
export ANDROID_HOME=$ENGINE/src/flutter/third_party/android_tools/sdk
```

### Device or Emulator

The tests in this package require a connected device or emulator to run. The
device or emulator should be running the same version of Android as the CI
configuration (there are issues with crashes and other problems on older
emulators in particular).

> [!CAUTION]
>
> [#144561](https://github.com/flutter/flutter/issues/144561): The emulator
> vendored in the engine checkout is old and [has a known issue with Vulkan](https://github.com/flutter/flutter/issues/144232).
>
> If you're working locally, you can update your copy by running:
>
> ```sh
> $ANDROID_HOME/cmdline-tools/latest/bin/sdkmanager --install emulator
> ```

### Additional Dependencies

While not required, it is **strongly recommended** to use an IDE such as
[Android Studio](https://developer.android.com/studio) when contributing to the
Android side of the test suite, as it will provide a better development
experience such as code completion, debugging, and more:

<img
  width="500"
  alt="Screenshot of Android Studio with Autocompletion"
  src="https://github.com/flutter/flutter/assets/168174/79b685f5-8da7-4396-abe6-9baeb29d7ce3"
/>

> [!TIP]
>
> Android Studio is expected to work _out of the box_ in this directory.
>
> If you encounter any issues, please [file a bug](#getting-help).

## Running the Tests

The [test runner](../bin/run_android_tests.dart) is a Dart script that installs
the test app and test suite on a connected device or emulator, runs the tests,
and reports the results including screenshots (golden files) and logs from
`adb logcat`.

From the `$ENGINE/src/flutter` directory, run:

```sh
dart ./testing/scenario_app/bin/run_android_tests.dart
```

By default when run locally, the test runner:

- Uses the engine-wide default graphics backend for Android.
- Uses the last built Android-specific engine artifacts (i.e. `$ENGINE/src/out/android_*/`).
- Will not diff screenshots, but does save them to a logs directory.

### Rebuilding the Tests

If you've made changes to any file in `scenario_app`, incluing the Dart code
in `lib/` or the Android code in `android/`, you will need to rebuild the
tests before running them.

To rebuild the `scenario_app` for the `android_debug_unopt_arm64` variant:

```sh
ninja -C out/android_debug_unopt_arm64 scenario_app
```

See also:

- [Compiling the Engine](https://github.com/flutter/flutter/wiki/Compiling-the-engine)

### Common Options

A list of options can be found by running:

```sh
dart ./testing/scenario_app/bin/run_android_tests.dart --help
```

Frequently used options include:

- `--out-dir`: Which engine artifacts to use (e.g.
  `--out-dir=../out/android_debug_unopt_arm64`).

- `--logs-dir`: Where to save full `adb logcat` logs and screenshots.

- `--[no]-enable-impeller`: Enables/disables use of the Impeller graphics
  backend.

- `--impeller-backend`: Use a specific Impeller backend (e.g.
  `--impeller-backend=opengles`).

- `--force-surface-producer-surface-texture`: Force the use of `SurfaceTexture`s
  for plugin code that uses `SurfaceProducer`s. This instruments the same code
  path that is used for Android API versions that are <= 28 without requiring
  an older emulator or device.

- `--smoke-test=<full.class.Name>`: Runs a specific test, instead of all tests.

### Advanced Options

When debugging the runner itself, you can use the `--verbose` flag to see more
detailed output, including additional options such as configuring which binary
of `adb` to use:

```sh
dart ./testing/scenario_app/bin/run_android_tests.dart --help --verbose
```

See also:

- [`bin/utils/options.dart`](../bin/utils/options.dart).

### CI Configuration

See [`ci/builders`](../../../ci/builders) and grep for `run_android_tests.dart`:

```sh
grep -r run_android_tests.dart ci/builders
```

> [!NOTE]
> The Impeller OpenGLES backend tests are only run on staging (`bringup: true`)
> and as such are **non-blocking**. We expect to stabilize and run these tests
> as part of a wider release of the Impeller OpenGLES backend.

#### Older Android

"Older Android" refers to "code paths to support older Android API levels".
Specifically, these configurations use
`--force-surface-producer-surface-texture` (see [above](#common-options) for
details).

| Backend           | CI Configuration            | CI History                                     | Skia Gold                 |
| ----------------- | --------------------------- | ---------------------------------------------- | ------------------------- |
| Skia              | [`ci/builders`][skia-st-ci] | [Presubmit][skia-try], [Postsubmit][skia-prod] | [Skia Gold][skia-st-gold] |
| Impeller OpenGLES | [`ci/builders`][imp-st-ci]  | [Staging][imp-staging]                         | N/A                       |

[skia-try]: https://ci.chromium.org/ui/p/flutter/builders/try/Linux%20linux_android_emulator_skia_tests
[skia-prod]: https://ci.chromium.org/ui/p/flutter/builders/prod/Linux%20linux_android_emulator_skia_tests
[imp-staging]: https://ci.chromium.org/ui/p/flutter/builders/staging/Linux%20linux_android_emulator_opengles_tests
[impeller-try]: https://ci.chromium.org/ui/p/flutter/builders/try/Linux%20linux_android_emulator_tests
[impeller-prod]: https://ci.chromium.org/ui/p/flutter/builders/prod/Linux%20linux_android_emulator_tests
[skia-st-ci]: https://github.com/search?q=repo%3Aflutter%2Fengine+path%3Aci%2Fbuilders+%22--no-enable-impeller%22+%22run_android_tests.dart%22+%22--force-surface-producer-surface-texture%22&type=code
[skia-st-gold]: https://flutter-engine-gold.skia.org/search?left_filter=ForceSurfaceProducerSurfaceTexture%3Dtrue%26GraphicsBackend%3Dskia&negative=true&positive=true&right_filter=AndroidAPILevel%3D34%26GraphicsBackend%3Dskia
[imp-st-ci]: https://github.com/search?q=repo%3Aflutter%2Fengine+path%3Aci%2Fbuilders+%22--impeller-backend%3Dopengles%22+%22run_android_tests.dart%22+%22--force-surface-producer-surface-texture%22&type=code
[imp-st-gold]: https://flutter-engine-gold.skia.org/search?left_filter=ForceSurfaceProducerSurfaceTexture%3Dtrue%26GraphicsBackend%3Dimpeller-opengles&negative=true&positive=true&right_filter=AndroidAPILevel%3D34%26GraphicsBackend%3Dskia

#### Newer Android

| Backend           | CI Configuration             | CI History                                             | Skia Gold                |
| ----------------- | ---------------------------- | ------------------------------------------------------ | ------------------------ |
| Skia              | [`ci/builders`][skia-ci]     | [Presubmit][skia-try], [Postsubmit][skia-prod]         | [Skia Gold][skia-gold]   |
| Impeller OpenGLES | [`ci/builders`][imp-gles-ci] | [Staging][imp-staging]                                 | N/A                      |
| Impeller Vulkan   | [`ci/builders`][imp-vk-ci]   | [Presubmit][impeller-try], [Postsubmit][impeller-prod] | [Skia Gold][imp-vk-gold] |

[skia-ci]: https://github.com/search?q=repo%3Aflutter%2Fengine+path%3Aci%2Fbuilders+%22--no-enable-impeller%22+%22run_android_tests.dart%22&type=code
[skia-gold]: https://flutter-engine-gold.skia.org/search?left_filter=ForceSurfaceProducerSurfaceTexture%3Dfalse%26GraphicsBackend%3Dskia&negative=true&positive=true&right_filter=AndroidAPILevel%3D34%26GraphicsBackend%3Dskia
[imp-gles-ci]: https://github.com/search?q=repo%3Aflutter%2Fengine+path%3Aci%2Fbuilders+%22--impeller-backend%3Dopengles%22+%22run_android_tests.dart%22&type=code
[imp-vk-ci]: https://github.com/search?q=repo%3Aflutter%2Fengine+path%3Aci%2Fbuilders+%22--impeller-backend%3Dvulkan%22+%22run_android_tests.dart%22&type=code
[imp-vk-gold]: https://flutter-engine-gold.skia.org/search?left_filter=ForceSurfaceProducerSurfaceTexture%3Dfalse%26GraphicsBackend%3Dimpeller-vulkan&negative=true&positive=true&right_filter=AndroidAPILevel%3D34%26GraphicsBackend%3Dskia

## Contributing

[![GitHub Issues or Pull Requests by label](https://img.shields.io/github/issues/flutter/flutter/e%3A%20scenario-app)](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22e%3A+scenario-app%22)

Contributions to this package are welcome, as it is a critical part of the
engine's test suite.

### Anatomy of a Test

A "test" in practice is a combination of roughly 3 components:

1. An Android _[JUnit test][]_ that configures and launches an Android activity.
2. An Android _[activity][]_, which simulates the Android side of a plugin or
   platform view.
3. A Dart _[scenario][]_, which simulates the Flutter side of an application.

[junit test]: ./app/src/androidTest/java/dev/flutter/scenariosui/DrawSolidBlueScreenTest.java
[activity]: ./app/src/main/java/dev/flutter/scenarios/ExternalTextureFlutterActivity.java
[scenario]: ../lib/src/solid_blue.dart

While every test suite has exactly one JUnit-instrumented class, each test can
have many activities and scenarios, each with their own configuration, setup,
and assertions. Not all of this is desirable, but it is the current state of
the test suite.

A test might also take a screenshot. See the _Skia Gold_ links in
[CI Configuration](#ci-configuration) for examples.

### Project History

This test suite was [originally written in 2019](https://github.com/flutter/engine/pull/10007)
with a goal of:

> \[being\] suitable for embedders to do integration testing with - it has no
> dependencies on the flutter_tools or framework, and so will not fail/flake
> based on variances in those downstream.

Unfortunately, the Android side of the test suite was never fully operational,
and the tests, even if failing, were accidentally be reported as passing on CI.
In 2024, as the team got closer to shipping our new graphics backend,
[Impeller](https://docs.flutter.dev/perf/impeller) on Android, it was clear that
we needed a reliable test suite for the engine on Android, particularly for
visual tests around external textures and platform views.

So, this package was revived and updated to be a (more) reliable test suite for
the engine on Android. It's by no means complete
([contributions welcome](#contributing)), but it did successfully catch at least
one bug that would not have been detected automatically otherwise.

_Go forth and test the engine on Android!_

## Troubleshooting

If you encounter any issues, please [file a bug](#getting-help).

### My test is failing on CI

If a test is failing on CI, it's likely that the test is failing locally as
well. Try the steps in [running the Tests](#running-the-tests) to reproduce the
failure locally, and then debug the failure as you would any other test. If this
is your first time working on the engine, you may need to setup a development
environment first (see [prerequisites](#prerequisites)).

The test runner makes extensive use of logging and screenshots to help debug
failures. If you're not sure where to start, try looking at the logs and
screenshots to see if they provide any clues (you'll need them to file a bug
anyway).

`test: Android Scenario App Integration Tests (Impeller/Vulkan)`
on [LUCI](https://ci.chromium.org/ui/p/flutter/builders/try/Linux%20Engine%20Drone/2079486/overview) produces these logs:

![Screenshot of Logs](https://github.com/flutter/flutter/assets/168174/fed8fae6-bcd2-4e6e-a074-c2c37f34a770)

The files include a screenshot of each test, and a _full_ logcat log from the
device (you might find it easier trying the `stdout` of the test first, which
uses rudimentary log filtering). In the case of multiple runs, the logs are
prefixed with the test configuration and run attempt.

### Gradle is failing due to an "not part of the dependency lock state" error

If you update dependencies in the `app/build.gradle` file, you may encounter an
error (probably in CI) that looks like:

```txt
FAILED: scenario_app/reports/lint-results.xml
vpython3 ../../flutter/testing/rules/run_gradle.py /b/s/w/ir/cache/builder/src/flutter/testing/scenario_app/android lint --no-daemon -Pflutter_jar=/b/s/w/ir/cache/builder/src/out/android_emulator_skia_debug_x64/flutter.jar -Pout_dir=/b/s/w/ir/cache/builder/src/out/android_emulator_skia_debug_x64/scenario_app --project-cache-dir=/b/s/w/ir/cache/builder/src/out/android_emulator_skia_debug_x64/scenario_app/.gradle --gradle-user-home=/b/s/w/ir/cache/builder/src/out/android_emulator_skia_debug_x64/scenario_app/.gradle

FAILURE: Build failed with an exception.

* What went wrong:
Execution failed for task ':app:checkDebugAarMetadata'.
> Could not resolve all files for configuration ':app:debugRuntimeClasspath'.
   > Resolved 'org.hamcrest:hamcrest-core:1.3' which is not part of the dependency lock state
   > Resolved 'com.google.code.findbugs:jsr305:2.0.2' which is not part of the dependency lock state
```

This is because [`gradle.lockfile`](./app/gradle.lockfile) is out of date. To
update it, run:

```sh
cd testing/scenario_app/android
$ENGINE_SRC/flutter/third_party/gradle/bin/gradle app:dependencies --write-locks
```

## Getting Help

To suggest changes, or highlight problems, please [file an issue](https://github.com/flutter/flutter/issues/new?labels=e:%20scenario-app,engine,platform-android,fyi-android,team-engine).

If you're not sure where to start, or need help debugging or contributing, you
can also reach out to [`hackers-android`](https://discord.com/channels/608014603317936148/846507907876257822) on Discord, or the Flutter engine team internally. There is
no full-time maintainer of this package, so be prepared to do some of the
legwork yourself.
