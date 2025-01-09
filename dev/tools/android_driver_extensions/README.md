# Android Extensions for Flutter Driver

This a minimal library on top of `flutter_driver` that provides extensions for
interacting with the native Android platform, to otherwise perform actions that
are not possible purely through Flutter Driver, and would want to run as an
_external_ test (run on the host, not on the device):

- Take a screenshot, including of _native_ widgets (platform views, textures)
- Tap on a native widget
- Rotate the device
- Background an app and send a "trim memory" signal to the device

> [!NOTE]
>
> While this library runs on Flutter's own CI, and is used to test Flutter's
> Platform Views, it is not officially supported as an external API, and may
> change or be removed at any time. We recommend you use existing testing
> infrastructure, such as:
>
> - [Integration Test](https://docs.flutter.dev/testing/integration-tests)
> - [Patrol](https://github.com/leancodepl/patrol)
