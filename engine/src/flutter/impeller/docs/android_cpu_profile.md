# Android CPU Profiling

Android devices have different performance characteristics than iOS devices, CPU traces frequently reveal surprising performance issues, such as https://github.com/flutter/engine/pull/48303 . This document describes the steps to capture an equivalent [flame graph](https://cacm.acm.org/magazines/2016/6/202665-the-flame-graph/abstract) on your local Android device.

### Build Engine with Symbols

Add the `--no-stripped` flag to the `gn` config when building the android engine.

Example:

```sh
gn --no-lto --runtime-mode=profile --android --android-cpu=arm64 --no-stripped
```

### Configure Gradle to Not Strip Sources

In the flutter project file `android/app/build.gradle` , add the following line under the `android` block:

```gradle
 packagingOptions{
     doNotStrip "**/*.so"
 }
```

### Mark the App as Debuggable

In case you are profiling a `--release` mode app, mark the app as debuggable by adding the following in the `application` tag of `AndroidManifest.xml`.

```xml
android:debuggable="true"
```

### Run the App with a Locally Built Engine

`flutter run` the app with the local engine flags (`--local-engine`,  `--local-engine-host`,  `--local-engine-src-path`).

Example:

```sh
flutter --local-engine android_profile_arm64 --local-engine-host host_profile_arm64 run --enable-impeller --profile
```

### Launch Android Studio

Open Android Studio. You can create a new blank project if you don't have one already. You do not need to open the application project nor do you need to run the app via Android Studio.

> [!TIP]
> Unless you are already a frequest user of Android Studio, it is recommended that you start with a blank project instead of opening the current project in Android Studio. The location of the various UI elements referenced below may change depending on Android Studio versions or project settings.

### Open the Profiler

> [!IMPORTANT]
> This may be in a different location or missing depending on the exact version of Android Studio that you have installed. Start a new Android Studio project if you can't find this link.

![Open the Profiler](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/android_profiling/image.avif)

### Start a New Profiling Session

Click the plus button to start a new session, then look for the attached devices, then finally the name of the application to profile. It usually takes a few seconds for the drop downs to populate. The IDE will warn about the build not being a release build, but this doesn't impact the C++ engine so ignore it.

![Start a new Profiling Session](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/android_profiling/dropdown.avif)

### Capture a CPU Profile

Click on the CPU section of the chart highlighted below. This will open a side panel that allows you to select the type of profile. Choose "Callstack Sample Recording" and then hit "Record" to start the profile and "Stop" to end the profile

![Capture a CPU Profile](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/android_profiling/where_do_i_click.avif)

### Analyze Raster performance

Samples will be collected from all threads, but for analyzing the engine performance we really only care about the raster thread. Note that if you are benchmarking an application that uses platform views, _and_ that platform view uses Hybrid Composition, then the raster thread will be merged with the platform thread.

Select the raster thread by clicking on that area and then choose flame graph (or any of the other options). The flame graph can be navigated using `WASD` and the chart area expanded to make inspection easier.

![Analyze Performance](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/android_profiling/so_many_options.avif)
