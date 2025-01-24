# Flutter Web integration tests

To run the tests in this package [download][1] the chromedriver matching the
version of Chrome. To find out the version of your Chrome installation visit
chrome://version.

Start `chromedriver` using the following command:

```sh
chromedriver --port=4444
```

An integration test is run using the `flutter drive` command. Some tests are
written for a specific [web renderer][2] and/or specific [build mode][4].
Before running a test, check the `_runWebLongRunningTests` function defined in
[dev/bots/test.dart][3], and determine the right web renderer and the build
mode you'd like to run the test in.

Here's an example of running an integration test:

```sh
flutter drive --target=test_driver/text_editing_integration.dart \
  -d web-server \
  --browser-name=chrome \
  --profile
```

This example runs the test in profile mode (`--profile`).

More resources:

* chromedriver: https://chromedriver.chromium.org/getting-started
* FlutterDriver: https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Running-Flutter-Driver-tests-with-Web.md
* `package:integration_test`: https://pub.dev/packages/integration_test

[1]: https://chromedriver.chromium.org/downloads
[2]: https://flutter.dev/docs/development/tools/web-renderers
[3]: https://github.com/flutter/flutter/blob/main/dev/bots/test.dart
[4]: https://flutter.dev/docs/testing/build-modes
