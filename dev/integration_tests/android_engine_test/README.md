# android_engine_test

This directory contains a sample app and tests that demonstrate how to use the
(experimental) _native_ Flutter Driver API to drive Flutter apps that run on
Android devices or emulators, interact with and capture screenshots of the app,
and compare the screenshots against golden images.

## Running the apps and tests

Each `lib/{prefix}_main.dart` file is a standalone Flutter app that you can run
on an Android device or emulator:

### `flutter_rendered_blue_rectangle`

This app displays a full screen blue rectangle. It mostly serves as a test that
Flutter can run at all on the target device, and that the Flutter (native)
driver can take a screenshot and compare it to a golden image. If this app or
test fails, it's likely none of the other apps or tests will work either.

```sh
# Run the app
$ flutter run lib/flutter_rendered_blue_rectangle_main.dart

# Run the test
$ flutter drive lib/flutter_rendered_blue_rectangle_main.dart
```

Files of significance:

- [Entrypoint](lib/flutter_rendered_blue_rectangle_main.dart)
- [Test](test_driver/flutter_rendered_blue_rectangle_main_test.dart)

## Debugging tips

- Use `flutter drive --keep-app-running` to keep the app running after the test.
- USe `flutter run` followed by `flutter drive --use-existing-app` for faster
  test iterations.
