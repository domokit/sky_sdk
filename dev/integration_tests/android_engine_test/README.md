# android_engine_test

This directory contains a sample app and tests that demonstrate how to use the
(experimental) _native_ Flutter Driver API to drive Flutter apps that run on
Android devices or emulators, interact with and capture screenshots of the app,
and compare the screenshots against golden images.

> [!CAUTION]
> This test suite is a _very_ end-to-end suite that is testing a combination of
> the graphics backend, the Android embedder, the Flutter framework, and Flutter
> tools, and only useful when the documentation and naming stays up to date and
> is clearly actionable.
>
> Please take extra care when updating the test suite to also update the REAMDE.

## How it runs on CI (LUCI)

See [`dev/bots/suite_runners/run_android_engine_tests.dart`](../../bots/suite_runners/run_android_engine_tests.dart), but tl;dr:

```sh
# TIP: If golden-files do not exist locally, this command will fail locally.
SHARD=android_engine_vulkan_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
SHARD=android_engine_opengles_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
```

## Running the apps and tests

Each `lib/{prefix}_main.dart` file is a standalone Flutter app that you can run
on an Android device or emulator.

- [`flutter_rendered_blue_rectangle`](#flutter_rendered_blue_rectangle)
- [`external_texture/surface_producer_smiley_face`](#external_texturesurface_producer_smiley_face)
- [`external_texture/surface_texture_smiley_face`](#external_texturesurface_texture_smiley_face)
- [`platform_view/hybrid_composition_platform_view`](#platform_viewhybrid_composition_platform_view)
- [`platform_view/texture_layer_hybrid_composition_platform_view`](#platform_viewtexture_layer_hybrid_composition_platform_view)
- [`platform_view/virtual_display_platform_view`](#platform_viewvirtual_display_platform_view)
- [`platform_view_tap_color_change`](#platform_view_tap_color_change)

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

### `external_texture/surface_producer_smiley_face`

This app displays a full screen rectangular deformed smiley face with a yellow
background. It tests the [`SurfaceProducer`](https://api.flutter.dev/javadoc/io/flutter/view/TextureRegistry.SurfaceProducer.html) API end-to-end, including historic regression cases around
backgrounding the app, trimming memory, and resuming the app.

```sh
# Run the app
$ flutter run lib/external_texture/surface_producer_smiley_face_main.dart

# Run the test
$ flutter drive lib/external_texture/surface_producer_smiley_face_main.dart
```

### `external_texture/surface_texture_smiley_face`

This app displays a full screen rectangular deformed smiley face with a yellow
background. It tests the [`SurfaceTexture`](https://api.flutter.dev/javadoc/io/flutter/view/TextureRegistry.SurfaceTexture.html) API end-to-end.

```sh
# Run the app
$ flutter run lib/external_texture/surface_texture_smiley_face_main.dart

# Run the test
$ flutter drive lib/external_texture/surface_texture_smiley_face_main.dart
```

### `platform_view/hybrid_composition_platform_view`

This app displays a blue orange gradient, the app is backgrounded, and then
resumed. It tests the [Hybrid Composition](../../../docs/platforms/android/Android-Platform-Views.md#hybrid-composition) implementation.

```sh
# Run the app
$ flutter run lib/platform_view/hybrid_composition_platform_view_main.dart

# Run the test
$ flutter drive lib/platform_view/hybrid_composition_platform_view_main.dart
```

### `platform_view/texture_layer_hybrid_composition_platform_view`

This app displays a blue orange gradient, the app is backgrounded, and then
resumed. It tests the [Texture Layer Hybrid Composition](../../../docs/platforms/android/Android-Platform-Views.md#texture-layer-hybrid-composition) implementation.

```sh
# Run the app
$ flutter run lib/platform_view/texture_layer_hybrid_composition_platform_view_main.dart

# Run the test
$ flutter drive lib/platform_view/texture_layer_hybrid_composition_platform_view_main.dart
```

### `platform_view/virtual_display_platform_view`

This app displays a blue orange gradient, the app is backgrounded, and then
resumed. It tests the [Virtual Display](../../../docs/platforms/android/Android-Platform-Views.md#virtual-display) implementation.

```sh
# Run the app
$ flutter run lib/platform_view/virtual_display_platform_view_main.dart

# Run the test
$ flutter drive lib/platform_view/virtual_display_platform_view_main.dart
```

### `platform_view_tap_color_change`

This app displays a blue rectangle, using platform views, which upon
being tapped (natively, not by Flutter), changes from blue to red.

```sh
# Run the app
$ flutter run lib/platform_view_tap_color_change_main.dart

# Run the test
$ flutter drive lib/platform_view_tap_color_change_main_test.dart
```

## Deflaking

Use `tool/deflake.dart <path/to/lib/main.dart>` to, in 1-command:

- Build an APK.
- Establish a baseline set of golden-files locally.
- Run N tests (by default, 10) in the same state, asserting the same output.

For example:

```sh
dart tool/deflake.dart lib/flutter_rendered_blue_rectangle_main.dart
```

For more options, see `dart tool/deflake.dart --help`.
