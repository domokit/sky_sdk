**Flutter GPU** is a low level graphics API that ships as a package in the Flutter SDK. Flutter GPU enables you to build arbitrary renderers from scratch using just Dart and GLSL. No native platform code required.

> [!Warning]
> - Flutter GPU is in an early preview state and does not guarantee API stability.
> - Flutter GPU currently requires [Impeller to be enabled](https://docs.flutter.dev/perf/impeller#availability).
> - Automated shader building relies on the experimental [Dart "Native Assets"](https://github.com/dart-lang/sdk/issues/50565) feature.
> - Because Flutter GPU is experimental and relies on experimental features, switching to the [master channel](https://docs.flutter.dev/release/upgrade#other-channels) is strongly recommended.

## How to use

Currently, our best getting started resource is [this article](https://medium.com/flutter/getting-started-with-flutter-gpu-f33d497b7c11).

There is also an experimental 3D rendering package powered by Flutter GPU called [Flutter Scene](https://pub.dev/packages/flutter_scene).

Flutter GPU is a low level API for building rendering packages from scratch. Graphics programming has a steep learning curve, and it's likely that most users will opt to use a higher level rendering package rather than build their own.

## Distribution

Flutter GPU can be used on the Flutter [master channel](https://docs.flutter.dev/release/upgrade#other-channels).

Flutter GPU is distributed using the same mechanism as `dart:ui`/`sky_engine`. While fetching artifacts, the Flutter tool downloads a zip containing the `flutter_gpu` package and places it in a package cache location searched when importing SDK packages.
And so Flutter GPU can be used by adding an SDK dependency to a package pubspec:
```yaml
dependencies:
  flutter:
    sdk: flutter
  flutter_gpu:
    sdk: flutter
```

## Useful links

- [Project dashboard](https://github.com/orgs/flutter/projects/134/views/1)
- [Original design doc](https://flutter.dev/go/impeller-dart) (now outdated!)
- [flutter_gpu package source](https://github.com/flutter/engine/tree/main/lib/gpu)

## Dart FFI

Under the hood, the API communicates with Flutter Engine via Dart FFI, calling symbols publicly exported by libflutter and/or embedders. These symbols are prefixed with `InternalFlutterGpu`, and are considered unstable. Direct usage of the exported symbols is not supported and will break without notice; the only supported way to use Flutter GPU is by importing `package:flutter_gpu`.

## Reporting bugs

If you run into issues while using Flutter GPU, please file a bug using the standard [bug report template](https://github.com/flutter/flutter/issues/new?template=2_bug.yml). Additionally, mention "Flutter GPU" in the title, label the bug with the `flutter-gpu` label.
