# Scenario App

[![GitHub Issues or Pull Requests by label](https://img.shields.io/github/issues/flutter/flutter/e%3A%20scenario-app)](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22e%3A+scenario-app%22)

This package simulates a Flutter app that uses the engine (`dart:ui`) only,
in conjunction with iOS-specific embedding code that simulates the use of the
engine in a real app (such as plugins and platform views).

[`run_ios_tests.sh`](run_ios_tests.sh) is used to run the tests on a simulator.

See also:

- [File an issue][file_issue] with the `e: scenario-app` label.
- [`lib/`](lib/), the Dart code and instrumentation for the scenario app.
- [`ios/`](ios/), the iOS-side native code and tests.

[file_issue]: https://github.com/flutter/flutter/issues/new?labels=e:%20scenario-app,engine,team-ios

## Adding a New Scenario

Create a new subclass of [Scenario](lib/src/scenario.dart) and add it to the map
in [scenarios.dart](lib/src/scenarios.dart). For an example, see
[animated_color_square.dart](lib/src/animated_color_square.dart), which draws a
continuously animating colored square that bounces off the sides of the
viewport.

Then set the scenario from the iOS app by calling `set_scenario` on platform
channel `driver`.
