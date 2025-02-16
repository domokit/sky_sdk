# Android Platform Embedder

Android embedder for Flutter, including Java [`io.flutter`](io/flutter/) sources
and Android specific engine-side C++.

This code provides the glue between the Flutter engine and the Android platform,
and is responsible for:

- Initializing the Flutter engine.
- Providing a platform view for the Flutter engine to render into.
- Dispatching events to the Flutter engine.

> [!CAUTION]
> This is a best effort attempt to document the Android embedder. It is not
> guaranteed to be up to date or complete. If you find a discrepancy, please
> [send a pull request](https://github.com/flutter/engine/compare)!

See also:

- [`../../tools/android_lint/bin/main.dart`](../../../tools/android_lint/bin/main.dart)
- [Android Platform Views](https://github.com/flutter/flutter/wiki/Android-Platform-Views)
- [Hosting native Android views in your Flutter app with Platform Views](https://docs.flutter.dev/platform-integration/android/platform-views)
- [Testing Android Changes in the Devicelab on an Emulator](https://github.com/flutter/flutter/wiki/Testing-Android-Changes-in-the-Devicelab-on-an-Emulator)
- [Texture Layer Hybrid Composition](https://github.com/flutter/flutter/wiki/Texture-Layer-Hybrid-Composition)

## Testing

There are two classes of tests for the Android embedder: unit tests that tend
to test _contracts_ within the embedder, and integration tests that test the
engine and embedder together, typically coupled with a Flutter app (that's how
our users interact with the engine and embedder).

### Unit tests

Unit tests for the Android embedder are located in:

- [`test`](test): Java unit tests.
- C++ files that end in `_unittests.cc` in [`shell/platform/android`](./).

The easiest way (though not the quickest) is to use `run_tests.py`:

```shell
# Assuming you're at the root of the engine repo where `run_tests.py` is located.
./testing/run_tests.py --type java

# Confusingly, these are C++ tests for Android.
./testing/run_tests.py --type android

# If you're using android_debug_unopt_arm64 builds:
./testing/run_tests.py --type android --android-variant android_debug_unopt_arm64
./testing/run_tests.py --type java --android-variant android_debug_unopt_arm64
```

> [!TIP]
> Trying to debug tests or timeouts/hangs?
>
> Run the tests directly from Android Studio, or [set `maxParallelForks` to `1`](https://github.com/flutter/flutter/blob/1cd3ab16ca1cba35e187ae75063fce0915985b94/engine/src/flutter/shell/platform/android/test_runner/build.gradle#L55):
>
> ```diff
> + maxParallelForks availableProcessors // The CI bot has 8 CPUs.
> - maxParallelForks 1
> ```

### Integration tests

Integration tests for the Android embedder mostly exist outside of the engine
for dubious historical reasons.

To run these tests, you'll need to have a Flutter checkout and a working Android
emulator or device:

```shell
# Build an up-to-date Flutter engine for Android.
cd $ENGINE/src

# Or, use your favorite arguments and build variant.
ninja -j1000 -C ../out/android_debug_unopt_arm64 android_jar

# Run the tests. Here is *1* example:
cd $FLUTTER/dev/integration_tests/external_textures
flutter drive \
  --local-engine-host=$ENGINE/out/host_debug_unopt_arm64 \
  --local-engine=$ENGINE/out/android_debug_unopt_arm64
```

Another good source of (unfortunately, manual) testing is `flutter/packages`:

```shell
cd $PACKAGES/packages/video_player/video_player_android/example
flutter run \
  --local-engine-host=$ENGINE/out/host_debug_unopt_arm64 \
  --local-engine=$ENGINE/out/android_debug_unopt_arm64
```

> [!NOTE]
> External texture rendering on Android is based on the device API level. For
> example to test the OpenGLES branch (which uses `SurfaceTexture`), you'll
> typically need an older device or emulator with an API version 29 or lower.
>
> You can also (locally) "force" the engine to use `SurfaceTextures`:
>
> ```diff
> // shell/platform/android/io/flutter/embedding/engine/renderer/FlutterRenderer.java
> -  @VisibleForTesting static boolean debugForceSurfaceProducerGlTextures = false;
> +  @VisibleForTesting static boolean debugForceSurfaceProducerGlTextures = true;
> ```
>
> ... and rebuild the engine.

See [our wiki](https://github.com/flutter/flutter/wiki/Testing-the-engine#java---android-embedding) also.

## Developing

How to edit and contribute to the Android embedder.

> [!TIP]
> This guide assumes you already have a working Engine development environment:
>
> - [Setting up the Engine development environment](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment)
> - [Compiling for Android](https://github.com/flutter/flutter/wiki/Compiling-the-engine#compiling-for-android-from-macos-or-linux)
>
> You should also have a working Android development environment:
>
> - [Android Studio](https://developer.android.com/studio)
> - [Install Flutter > Test Drive](https://docs.flutter.dev/get-started/test-drive?tab=androidstudio)
>
> _It is also recommended (but not required) to install
> [Visual Studio Code](https://code.visualstudio.com/)._

Depending on what you are trying to do, you may need to edit the Java code in
[`io.flutter`](io/flutter/) or the C++ code in [`shell/platform/android`](./),
sometimes both. Let's start with the C++ code, as it is more similar to
developing for other platforms or platform-agnostic parts of the engine.

### Editing C++ code

The C++ code for the Android embedder is located in
[`shell/platform/android`](./) and subdirectories.

Some notable files include:

- [`context/android_context.h`](./context/android_context.h): Holds state that
  is shared across Android surfaces.
- [`jni/platform_view_android_jni.h`](./jni/platform_view_android_jni.h): Allows
  calling Java code running in the JVM.
- [`AndroidManifest.xml`](./AndroidManifest.xml): Used by [`android_lint`](../../../tools/android_lint/).
- [`BUILD.gn`](./BUILD.gn): Used by GN to build the C++-side embedder tests and
  the `flutter.jar` file for the engine.
- [`ndk_helpers.h`](./ndk_helpers.h): Helper functions for dynamically loading
  and calling Android NDK (C/C++) functions.
- [`platform_view_android.h`](./platform_view_android.h): The main entry point
  for the Android embedder.

See [VSCode with C/C++ Intellisense](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment#vscode-with-cc-intellisense-cc)
for how to use the [`clangd`](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) extension to get C++ code
completion:

![Example](https://github.com/flutter/flutter/assets/168174/8a75dd27-66e1-4c4f-88af-667a73b909b6)

> [!NOTE]
>
> `--compile-commands-dir` must point to an Android build output:
>
> ```jsonc
> {
>   /* ... */
>   "clangd.path": "buildtools/mac-arm64/clang/bin/clangd",
>   "clangd.arguments": ["--compile-commands-dir=out/android_debug_unopt_arm64"]
>   /* ... */
> }
> ```
>
> ... but remember to change it back when editing other parts of the engine.

### Editing Java code

The Java code for the Android embedder is located in
[`io/flutter/`](io/flutter/) and subdirectories.

The tests are located in [`test/io/flutter/`](test/io/flutter/), and the test
runner in [`test_runner`](test_runner/).

Some notable files include:

- [`io/flutter/embedding/android/FlutterActivity.java`](io/flutter/embedding/android/FlutterActivity.java):
  An activity that displays a full-screen Flutter UI.
- [`io/flutter/embedding/engine/FlutterJNI.java`](io/flutter/embedding/engine/FlutterJNI.java):
  The Java interface for the C++ engine.
- [`io/flutter/view/TextureRegistry.java`](io/flutter/view/TextureRegistry.java):
  Registry of backend textures used by a Flutter View.

It is non-trivial to get a working IDE setup for editing Java code in the
Flutter engine. Some developers have had success [using VSCode as an IDE for the Android Embedding](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment#using-vscode-as-an-ide-for-the-android-embedding-java),
but the following instructions are for if that doesn't work, or you want to use
Android Studio:

1. Open `shell/platform/android` in Android Studio.
1. Configure the following:

   - [`Preferences | Build, Execution, Deployment | Gradle-Android Compiler`](jetbrains://AndroidStudio/settings?name=Build%2C+Execution%2C+Deployment--Gradle-Android+Compiler)

     Command-line Options:

     ```txt
     -Pbuild_dir="/tmp/build_dir" -Pflutter_jar="$ENGINE/src/out/android_debug_unopt_arm64/flutter.jar"
     ```

   - [`Preferences | Build, Execution, Deployment | Build Tools | Gradle`](jetbrains://AndroidStudio/settings?name=Build%2C+Execution%2C+Deployment--Build+Tools--Gradle)

     Distribution of `Local Installation` with:

     ```txt
     $ENGINE/src/third_party/gradle
     ```

     Gradle SDK using Android Studio (path depends on your machine):

     ```txt
     /Applications/Android Studio.app/Contents/jbr/Contents/Home
     ```

1. Sync Gradle.

   ![Example](https://github.com/flutter/flutter/assets/168174/02fe0e6f-f0c4-47b2-8dae-9aa0b9520503)

At this point you should be able to open Java files in Android Studio and get
code completion in the `io/flutter` and `test/io/flutter` folders. For example, `FlutterJNI.java`:

![Example](https://github.com/flutter/flutter/assets/168174/387550d4-eab7-4097-9da3-7713a6ec4da7)

To get code coverage displayed in line: go to the test class you wish to run and

1. Right click > Modify Run Configuration...,
2. In the window that pops up click Modify options > Specify classes
   and packages (under "code coverage").
3. In the new box that appears at the bottom of the window, click the + > Add package, and then add `io.flutter.*`.
