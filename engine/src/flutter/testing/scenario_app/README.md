# Scenario App

[![GitHub Issues or Pull Requests by label](https://img.shields.io/github/issues/flutter/flutter/e%3A%20scenario-app)](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22e%3A+scenario-app%22)

This package simulates a Flutter app that uses the engine (`dart:ui`) only,
in conjunction with Android and iOS-specific embedding code that simulates the
use of the engine in a real app (such as plugins and platform views).

The [`bin/run_android_tests.dart`](bin/run_android_tests.dart) and
[`run_ios_tests.sh`](run_ios_tests.sh) are then used to run the tests on a
connected device or emulator.

See also:

- [File an issue][file_issue] with the `e: scenario-app` label.
- [`bin/`](bin/), the entry point for running Android integration tests.
- [`lib/`](lib/), the Dart code and instrumentation for the scenario app.
- [`ios/`](ios/), the iOS-side native code and tests.
- [`android/`](android/), the Android-side native code and tests.

[file_issue]: https://github.com/flutter/flutter/issues/new?labels=e:%20scenario-app,engine,team-engine

## Running a smoke test on Firebase TestLab

To run the smoke test on Firebase TestLab test, build `android_profile_arm64`,
and run [`./ci/firebase_testlab.py`](../../ci/firebase_testlab.py), or pass
`--variant` to run a different configuration.

```sh
# From the root of the engine repository
$ ./ci/firebase_testlab.py --variant android_debug_arm64
```

> [!NOTE]
> These instructions were not verified at the time of writing/refactoring.

## Adding a New Scenario

Create a new subclass of [Scenario](lib/src/scenario.dart) and add it to the map
in [scenarios.dart](lib/src/scenarios.dart). For an example, see
[animated_color_square.dart](lib/src/animated_color_square.dart), which draws a
continuously animating colored square that bounces off the sides of the
viewport.

Then set the scenario from the Android or iOS app by calling `set_scenario` on
platform channel `driver`.
