In general, our philosophy is to update the `stable` channel on a quarterly basis with feature updates. In the intervening period, occasionally we may decide a bug or regression warrants a hotfix. We tend to be extremely conservative with these hotfixes, since there's always a risk that fixing one bug introduces a new one, and we want the `stable` channel to always represent our most tested builds.

We intend to announce hotfixes to the [flutter-announce](https://groups.google.com/forum/#!forum/flutter-announce) group, and we recommend that you subscribe to that list if you publish an application using Flutter.

Note that we only hotfix the latest version -- if you see bugs on older versions of the `stable` channel, please consider moving to the latest `stable` channel version.

To ensure that you have the latest stable version with the hotfixes listed below, use the flutter tool at the command line as follows:

```
$ flutter channel stable
$ flutter upgrade
```

<!--
INTERNAL NOTE: PLEASE DON'T JUST PASTE ISSUE TITLES!

Make sure that the text here helps customers understand
whether they are likely to be affected by the issue,
without them needing to read each issue individually.
Our goal is to make the list easy for them to scan.

More information and tips:
docs/releases/Hotfix-Documentation-Best-Practices.md

INTERNAL NOTE
-->

## Flutter 3.27 Changes

### [3.27.3](https://github.com/flutter/flutter/releases/tag/3.27.3)
- [flutter/159212](https://github.com/flutter/flutter/issues/159212) Track (via Google Analytics) if the Dart AOT Android "Deferred Components" feature is being meaningfully used.
- [flutter/160631](https://github.com/flutter/flutter/issues/160631) Fixes an issue with Material 3 Tab Bar animations.
- [flutter/159289](https://github.com/flutter/flutter/issues/159289) Fixes an issue with fullscreen route transitions.

### [3.27.2](https://github.com/flutter/flutter/releases/tag/3.27.2)

- [flutter/159729](https://github.com/flutter/flutter/issues/159729) Flutter module template triggers a warning when built for Android.
- [flutter/161176](https://github.com/flutter/flutter/issues/161176) Dropdown Menu can create an infinite loop.
- [flutter/161330](https://github.com/flutter/flutter/issues/161330) Using ScrollViewKeyboardDismissBehavior.onDrag in a SingleChildScrollView causes text fields to immediately unfocus if the keyboard opening scrolls the text field to keep it visible.
- [flutter/160127](https://github.com/flutter/flutter/issues/160127) Some Flutter web plugins do not add the `crossOrigin` property to <img> tags.
- [flutter/160155](https://github.com/flutter/flutter/issues/160155) Failed assertion in web engine: "The targeted input element must be the active input element".
- [flutter/160199](https://github.com/flutter/flutter/issues/160199) Some images on the web render blank.
- [flutter/160459](https://github.com/flutter/flutter/issues/160459) Incorrect Z order rendering in drawPoints may cause lines to overlap when one should be drawn in front of the other.
- [flutter/160409](https://github.com/flutter/flutter/issues/160409) App may crashes because of obsolete engine assertion.
- [flutter/158192](https://github.com/flutter/flutter/issues/158192) Positions of display cutouts on Android may not update - as returned by MediaQuery and used by SafeArea - upon screen orientation change.

### [3.27.1](https://github.com/flutter/flutter/releases/tag/3.27.1)

- [flutter/160041](https://github.com/flutter/flutter/issues/160041) - [Impeller][Android] Disables Impeller on older Android devices.
- [flutter/160206](https://github.com/flutter/flutter/issues/160206) - [Impeller][Android] Disables Android HardwareBuffer based swapchains on all devices.
- [flutter/160208](https://github.com/flutter/flutter/issues/160208) - [iOS] Fixes an issue on iOS preventing the ability to tap web view links in some plugins.

### [3.27.0](https://github.com/flutter/flutter/releases/tag/3.27.0)
Initial stable release.

## Flutter 3.24 Changes

### [3.24.5](https://github.com/flutter/flutter/releases/tag/3.24.5)
- [flutter/158125](https://github.com/flutter/flutter/pull/158125) - [iOS] Fixed a tool issue causing failures when `flutter build ios-framework --xcframework` copies Flutter debug symbols.
- [flutter/56301](https://github.com/flutter/engine/pull/56301) - [Android] Fixes a crash on Android devices when the surface is released unexpectedly when using PlatformView's.

### [3.24.4](https://github.com/flutter/flutter/releases/tag/3.24.4)
- [dart 3.5.4 changelog](https://github.com/dart-lang/sdk/blob/stable/CHANGELOG.md#354---2024-10-17)
- [flutter/154915](https://github.com/flutter/engine/pull/55366) - [macOS] Comply with the new Apple privacy manifest policy for the macOS Flutter engine framework and prevent the "Missing privacy manifest" warning when submitting a macOS app to the App Store.
- [flutter/153471](https://github.com/flutter/flutter/issues/153471) - [Tool] Fixes RPCError crash when setting up log filtering for Android devices.

### [3.24.3](https://github.com/flutter/flutter/releases/tag/3.24.3)
- [dart 3.5.3 changelog](https://github.com/dart-lang/sdk/blob/stable/CHANGELOG.md#353---2024-09-11)
- [flutter/154275](https://github.com/flutter/flutter/issues/154275) - [Android] Fixes performance issues on Android caused by engine threads not matching the core count.
- [flutter/154276](https://github.com/flutter/flutter/issues/154276) - [Impeller] Fixes an issue on iOS preventing mesh gradients from rendering correctly.
- [flutter/154349](https://github.com/flutter/flutter/issues/154349) - [Wasm] Fixes an issue on web causing Platform Views to break when compiled to Wasm.
- [flutter/154564](https://github.com/flutter/flutter/issues/154564) - [Impeller][iOS] Fixes an issue when using Impeller on iOS when using backdrop filters on older iPads, causing the GPU to hang.
- [flutter/154712](https://github.com/flutter/flutter/issues/154712) - [iOS] Fixes an issue on iOS causing video playback to flicker.
- [flutter/154892](https://github.com/flutter/flutter/issues/154892) - [Impeller][iOS] Fixes an issue when using Impeller on iOS causing a memory leak when using Platform Views.
- [flutter/154536](https://github.com/flutter/flutter/issues/154536) - [Tool] Fixes a CLI crash that occurs when shutting down after running a Flutter app on a browser.
- [flutter/154720](https://github.com/flutter/flutter/pull/154720) - Fixes an issue with the `Drawer` widget, causing it to open or close incorrectly.
- [flutter/154944](https://github.com/flutter/flutter/pull/154944) - [Tool] Fixes a Flutter tool crash that occurs when building Flutter modules for Android when using AGP 8.0+.

### [3.24.2](https://github.com/flutter/flutter/releases/tag/3.24.2)
- [Dart 3.5.2 Changelog](https://github.com/dart-lang/sdk/blob/stable/CHANGELOG.md#352---2024-08-28)
- [flutter/153949](https://github.com/flutter/flutter/issues/153949) - Fixes a crash on Android when deleting `EditableText` inside `CupertinoPageRoute`, with a CJK (chinese, japanese, korean) keyboard.
- [flutter/153939](https://github.com/flutter/flutter/issues/153939) - Fixes an issue on iOS where Flutter `TextField`s may stop accepting input.
- [flutter/152420](https://github.com/flutter/flutter/issues/152420) - Fixes scrolling jank on Android and iOS when a `SelectionArea`/`SelectableRegion` is used as a child of a Scrollable like `ListView` or `PageView`.
- [flutter/154199](https://github.com/flutter/flutter/pull/154199) - Removes excessive logging when building a freshly created template app for Android.
- [flutter/153967](https://github.com/flutter/flutter/pull/153967) - Fixes a host build failure on macOS when the `native assets` experiment is enabled, and there are no native asset frameworks to codesign.
- [flutter/153769](https://github.com/flutter/flutter/pull/153769) - When running a Flutter app, display a concise error message when connection to the device is lost.
- [flutter/154270](https://github.com/flutter/flutter/pull/154270) - Prevent preemptive gradle crash for android builds that would fail to build anyway but with a confusing error message.
- [flutter/54735](https://github.com/flutter/engine/pull/54735) - Fixes an error on Flutter Web where `onTap` is called twice on various widgets (`GestureDetector`, `InkWell`) when semantics are enabled.

### [3.24.1](https://github.com/flutter/flutter/releases/tag/3.24.1)

- [dart/56464](https://github.com/dart-lang/sdk/issues/56464) - Fixes resolving `include:` in `analysis_options.yaml` file in a nested folder in the workspace.
- [dart/56423](https://github.com/dart-lang/sdk/issues/56423) - Fixes source maps generated by `dart compile wasm` when optimizations are enabled.
- [dart/56374](https://github.com/dart-lang/sdk/issues/56374) - Fixes a bug in the `dart2wasm` compiler in unsound `-O3` / `-O4` modes where a implicit setter for a field of generic type will store null instead of the field value.
- [dart/56440](https://github.com/dart-lang/sdk/issues/56440) - Fixes a bug in the `dart2wasm` compiler that can trigger in certain situations when using partial instantiations of generic tear-offs (constructors or static methods) in constant expressions.
- [dart/56457](https://github.com/dart-lang/sdk/issues/56457) - The algorithm for computing the standard upper bound of two types, also known as UP, is provided the missing implementation for `StructuralParameterType` objects. In some corner cases the lacking implementation resulted in a crash of the compiler.
- [flutter/152047](https://github.com/flutter/flutter/issues/152047) - [Web] Fixes an issue in Flutter Web apps where when semantics are enabled, tapping on the label of a checkbox in a mobile browser won't togle the checkbox.
- [flutter/153308](https://github.com/flutter/flutter/issues/153308) - [Web] Adds source map support in `flutter run` / `flutter build` for `dart2wasm` for debugging in Chrome DevTools.
- [flutter/54446](https://github.com/flutter/engine/pull/54446) - [Web] Fixes an issue in Flutter Web apps where the app may crash if CanvasKit is loaded from the network instead of a cache.
- [flutter/152955](https://github.com/flutter/flutter/issues/152955) - [Impeller] Fixes an issue where when using unbound `saveLayers` rendering issues would occur.
- [flutter/153037](https://github.com/flutter/flutter/issues/153037) - [Impeller] Fixes an issue where RTL glyphs would render incorrectly.
- [flutter/153038](https://github.com/flutter/flutter/issues/153038) - [Impeller] Fixes an issue where padding would be applied incorrectly in `Canvas.drawVerticies` when using texture coordinates.
- [flutter/153041](https://github.com/flutter/flutter/issues/153041) - [Impeller] Fixes an rare issue causing applications to crash when using platform views on older iPhones.
- [flutter/153188](https://github.com/flutter/flutter/issues/153188) - [Impeller] Fixes a rendering issue on iOS devices using Impeller where clips do not appear around entities drawn with certain advanced blend modes.
- [flutter/54513](https://github.com/flutter/engine/pull/54513) - [iOS/MacOS] Fixes an issue preventing iOS Apps Store validation from failing for Flutter apps using Xcode versions before Xcode 16.
- [flutter/54518](https://github.com/flutter/engine/pull/54518) - Fixes an issue on OpenGL ES devices where a black screen would appear instead of the Flutter app output.
- [flutter/153117](https://github.com/flutter/flutter/pull/153117) [iOS/MacOS] Fixes an issue where compilation errors are not displayed in the output of `flutter run` when using Xcode 16.
- [flutter/153321](https://github.com/flutter/flutter/issues/153321) - [Desktop] Fixes an issue where older Windows devices could not run Flutter apps built using Flutter  3.21 or later.
- [flutter/153294](https://github.com/flutter/flutter/pull/153294) [Tool] Fixes an issue in the Flutter tool streamlining the crash message that occurs when running `flutter run -d chrome` and Chrome is closed before Flutter tries to close it.
- [flutter/153579](https://github.com/flutter/flutter/pull/153579) [Tool] Fixes an issue where users would experience large crash messages when `flutter run` or `flutter debug-adapter` are unable to connect to the Flutter web app.

### [3.24.0](https://github.com/flutter/flutter/releases/tag/3.24.0)
Initial stable release.

## Flutter 3.22 Changes

### [3.22.3](https://github.com/flutter/flutter/releases/tag/3.22.3) (July 17, 2024)

- [dart/55979](https://github.com/dart-lang/sdk/issues/55979) - Fixes an issue where `const bool.fromEnvironment('dart.library.ffi')` is true and conditional import condition `dart.library.ffi` is true in dart2wasm.
- [dart/55943](https://github.com/dart-lang/sdk/issues/55943) - Fixes an issue where FFI calls with variadic arguments on MacOS Arm64 would mangle the arguments.
- [flutter/149700](https://github.com/flutter/flutter/issues/149700) - [Impeller] Fixes rendering corruption when running on Intel mac simulators.
- [flutter/149701](https://github.com/flutter/flutter/issues/149701) - [Impeller] Fixes an issue on iOS that causese paths to render incorrectly.
- [flutter/149702](https://github.com/flutter/flutter/issues/149702) - [Impeller] Corrects and issue on iOs where coverage computation results in distored pixels in Impeller targets.
- [flutter/149704](https://github.com/flutter/flutter/issues/149704) - [Impeller] Fixes and issue on iOS where flickering may be occur when translating a blurred rounded rectangle.
- [flutter/149745](https://github.com/flutter/flutter/issues/149745) - [Impeller] Fixes a segfault on iOS when tessellating empty convex polygons.
- [flutter/149771](https://github.com/flutter/flutter/issues/149771) - [Impeller] Fixes a rendering error on iOS when advanced blend is double scaled.
- [flutter/53183](https://github.com/flutter/engine/pull/53183) - Fixes an issue where Linux apps show visual corruption on some frames
- [flutter/149856](https://github.com/flutter/flutter/issues/149856) - Clarifies Flutter Fix log on how to update Kotlin Gradle Plugin that was introduced in Flutter 3.19.
- [flutter/150617](https://github.com/flutter/flutter/pull/150617) - Fixes a bug in `flutter test` where `--flavor` wasn't considered when validating cached assets, causing the flavor-conditional asset bundling feature to not work as expected.
- [flutter/150724](https://github.com/flutter/flutter/issues/150724) - Fixes an issue on Web+Linux that prevents users from inputting data using the numpad.
- [flutter/150787](https://github.com/flutter/flutter/pull/150787) - Fixes and issue on Windows when running certain commands, such as `flutter run` or `flutter build`, users get a lengthy crash message including the full contents of a FileSystemException.

### [3.22.2](https://github.com/flutter/flutter/releases/tag/3.22.2) (June 06, 2024)
* [dart/55818](https://github.com/dart-lang/sdk/issues/55818) - Fixes an issue where `DART_VM_OPTIONS` were not correctly parsed for standalone Dart executables created with `dart compile exe`.
* [dart/55873](https://github.com/dart-lang/sdk/issues/55873) - Fixes a bug in dart2wasm that can result in a runtime error that says `array.new_fixed()` has a constant larger than 10000.
* [dart/55894](https://github.com/dart-lang/sdk/issues/55894) - Adds support for `--enable-experiment` flag to `dart compile` wasm.
* [dart/55895](https://github.com/dart-lang/sdk/issues/55895) - Fixes an issue in dart2wasm compiler that can result in incorrect nullability of type parameter.
* [dart/55890](https://github.com/dart-lang/sdk/issues/55890) - Disallows `dart:ffi` imports in user code in dart2wasm as dart2wasm's currently only supports a small subset of `dart:ffi`.
* [flutter/148885](https://github.com/flutter/flutter/issues/148885) - Fixes a platform view issue on android 14 when multiple activities are used and `onMemoryTrim` is called.
* [flutter/149178](https://github.com/flutter/flutter/issues/149178) - Fixes an issue on iOS where users are unable to focus on a `TextField` or open the keyboard again after side pop from another screen.
* [flutter/149210](https://github.com/flutter/flutter/issues/149210) - Fixes an `EditableText` crash that occurs when a custom `TextEditingController` only implements the `TextEditingController` interface.
* [flutter/149588](https://github.com/flutter/flutter/issues/149588) - Fixes a crash that occurs when rendering children in `TwoDimentionalViewport` using keep alive widgets (e.g InkWell).
* [flutter/148916](https://github.com/flutter/flutter/pull/148916) - Fixes an issue in the `ColorScheme.fromSeed` method to respect the seed color even if the seed color is very bright.
* [flutter/149345](https://github.com/flutter/flutter/pull/149345) - Adds a service extension that DevTools uses to support a "Track widget build counts" feature in DevTools 2.36.0.
* [flutter/149378](https://github.com/flutter/flutter/pull/149378) - Fixes a focus issue on iOS and MacOS that causes `TextFields` to not function after cupertino back swipes.
* [flutter/52987](https://github.com/flutter/engine/pull/52987) - Fixes an issue on Android where platform view inputs are mapped to the wrong location.

### [3.22.1](https://github.com/flutter/flutter/releases/tag/3.22.1) (May 22, 2024)
* [dart/55714](https://github.com/dart-lang/sdk/issues/55714) - Fixes a bug in the CFE which could manifest as compilation errors of Flutter
  web apps when compiled with dart2wasm.
* [dart/55758](https://github.com/dart-lang/sdk/issues/55758) - Fixes a bug in the pub client, such that `dart run` will not interfere with
  Flutter l10n (at least for most cases).
* [flutter/147142](https://github.com/flutter/flutter/issues/147142) - Fixes a read/write permission issue when building Flutter apps for MacOS.

### [3.22.0](https://github.com/flutter/flutter/releases/tag/3.22.0) (May 14, 2024)
Initial stable release.

## Flutter 3.19 Changes

### [3.19.6](https://github.com/flutter/flutter/releases/tag/3.19.6) (April 17, 2024)
* [dart/55430](https://github.com/dart-lang/sdk/issues/55430) - Fixes an issue with JS interop in dart2wasm where JS interop methods that used the enclosing library‘s @JS annotation were actually using the invocation’s enclosing library's @JS annotation.
* [flutter/145563](https://github.com/flutter/flutter/issues/145563) - Fixes severe performance regression on Firefox in v. 3.19.
* [flutter/144439](https://github.com/flutter/flutter/issues/144439) - Removes the --enable-impeller run flag and FLTEnableImpeller plist key on iOS.

### [3.19.5](https://github.com/flutter/flutter/releases/tag/3.19.5) (March 28, 2024)
* [dart/55211](https://github.com/dart-lang/sdk/issues/55211) - Fixes an issue where dart vm crashed when running on pre-SSE41 older CPUs on Windows.

### [3.19.4](https://github.com/flutter/flutter/releases/tag/3.19.4) (March 21, 2024)
* [flutter/144211](https://github.com/flutter/flutter/issues/144211) - Reverts a clipping optimization that is broken when multiple clips are applied with a backdrop filter.
* [flutter/144213](https://github.com/flutter/flutter/issues/144213) - Fix flickering of gaussian blurs in scrolling containers.
* [dart/55158](https://github.com/dart-lang/sdk/issues/55158) - Fixes an exception when executing hot reload after making compilation-successful changes.
* [dart/55194](https://github.com/dart-lang/sdk/issues/55194) - ​​Fix crashes on web platforms that contains an extension type declaration where the extension type constructor invokes a redirecting factory in its initializer.
* [dart/55184](https://github.com/dart-lang/sdk/issues/55184) - Fix issues where it is unable to run commit queue and post-submit testing on beta and stable when Goma is shut down.
* [dart/55240](https://github.com/dart-lang/sdk/issues/55240) - ​​Fix DateTime.timeZoneName on Windows.

### [3.19.3](https://github.com/flutter/flutter/releases/tag/3.16.3) (March 07, 2024)
* [flutter/144565](https://github.com/flutter/flutter/issues/144565) - Fixes a tool crash when attempting to render a frame with raster stats on an application with the Impeller backend.
* [dart/55057](https://github.com/dart-lang/sdk/issues/55057) - Fixes an issue in dart2js where object literal constructors in interop extension types would fail to compile without an `@JS` annotation on the library.
* [dart/55095](https://github.com/dart-lang/sdk/issues/55095) - ​​Disallows certain types involving extension types from being used as the operand of an `await` expression, unless the extension type itself implements`Future`.

### [3.19.2](https://github.com/flutter/flutter/releases/tag/3.19.2) (February 28, 2024)
* [flutter/143886](https://github.com/flutter/flutter/issues/143886) - Fixes a parsing issue that caused the Flutter tool to crash in some circumstances.

### [3.19.1](https://github.com/flutter/flutter/releases/tag/3.19.1) (February 21, 2024)
* [flutter/143574](https://github.com/flutter/flutter/issues/143574) - Fixes an issue in Flutter web builds that disallowed the use of`--flavor` while launching.

## Flutter 3.16 Changes

### [3.16.9](https://github.com/flutter/flutter/releases/tag/3.16.9) (January 25, 2024)
* [dart/54699](https://github.com/dart-lang/sdk/issues/54699) - Fix an issue that causes Flutter apps to freeze when breakpoints are added to multiple isolates at the same time and an issue that causes Flutter apps to crash during hot reload.

### [3.16.8](https://github.com/flutter/flutter/releases/tag/3.16.8) (January 17, 2024)
* [dart/54494](https://github.com/dart-lang/sdk/issues/54494) - Fix Dart2js stack overflow in value range analysis.

### [3.16.7](https://github.com/flutter/flutter/releases/tag/3.16.7) (January 11, 2024)
* [dart/54427](https://github.com/dart-lang/sdk/issues/54427) - Upgrades Dart DevTools to version 2.28.5.
* [dart/54428](https://github.com/dart-lang/sdk/issues/54428) - Fixes an issue with serving static DevTools assets.

### [3.16.6](https://github.com/flutter/flutter/releases/tag/3.16.6) (January 10, 2024)
* [flutter/141017](https://github.com/flutter/flutter/issues/141017) - Migrates event sent with every command for analytics.
* [flutter/136060](https://github.com/flutter/flutter/issues/136060) - Fixes Xcode 15 crashes EXC_BAD_ACCESS when using the Networking framework.
* [flutter/140416](https://github.com/flutter/flutter/issues/140416) - Fixes PathNotFoundException deleting temp dir in IOSCoreDeviceControl._listCoreDevices.
* [dartlang/webdev/2297](https://github.com/dart-lang/webdev/issues/2297) - Fixes DWDS error when debugging on web.

### [3.16.5](https://github.com/flutter/flutter/releases/tag/3.16.5) (December 20, 2023)
* [flutter/138711](https://github.com/flutter/flutter/issues/138711) - Fixes AvailabilityVersionCheck failure on iOS
* [flutter/139571](https://github.com/flutter/flutter/issues/139571) - Fixes AnimatedOpacity affecting blended color overlay render
* [flutter/139294](https://github.com/flutter/flutter/issues/139294) - Fixes ImageFiltered flickers when widget is rendered on top
* [flutter/138193](https://github.com/flutter/flutter/issues/138193) - Fixes testMultiplePlatformViewsWithOverlays test on MacOS

### [3.16.4](https://github.com/flutter/flutter/releases/tag/3.16.4) (December 13, 2023)
* [flutter/139180](https://github.com/flutter/flutter/issues/139180) - Fix tool crash on flutter create when unable to run Java.
* [flutter/138434](https://github.com/flutter/flutter/issues/138434) - Fix tool crash on deleting directories that do not exist
* [flutter/135277](https://github.com/flutter/flutter/issues/135277) - Eliminates an excessive amount of Xcode error/warning output to the console when building or running macOS Flutter apps.

### [3.16.3](https://github.com/flutter/flutter/releases/tag/3.16.3) (December 5, 2023)
* [CVE-2023-6345](https://nvd.nist.gov/vuln/detail/CVE-2023-6345) - Skia fix for possible integer overflow on Canvas calls with user generated data
* [flutter/138550](https://github.com/flutter/flutter/issues/138550) - Fixes crash on iPad when selection "Share..." from selection controls.
* [flutter/138842](https://github.com/flutter/flutter/issues/138842) - Fix rendering bug with elevation 0 material components.
* [flutter/138850](https://github.com/flutter/flutter/issues/138850) - Add ability to customize NavigationBar indicator overlay and fixes a bug with the indicator shape.
* [dart/53086](https://github.com/dart-lang/sdk/issues/53086) - DDS fix to ensure threadID integers are serialized correctly by Debug Adapter Protocol (DAP) clients.
* [dart/53999](https://github.com/dart-lang/sdk/issues/53999) - Adjusts the nullablity computations in the implementation of the upper bound algorithm in the CFE
* [dart/54112](https://github.com/dart-lang/sdk/issues/54112) - Fixes missing closure code completion entries for function parameters for LSP-based editors like VS Code.

### [3.16.2](https://github.com/flutter/flutter/releases/tag/3.16.2) (November 30, 2023)
* [flutter/138535](https://github.com/flutter/flutter/issues/138535) - Fixes android execution failed for task ':app:mergeDebugNativeLibs'.
* [flutter/138598](https://github.com/flutter/flutter/issues/138598) - Fixes SVG rendering issue on IOS.

### [3.16.1](https://github.com/flutter/flutter/releases/tag/3.16.1) (November 27, 2023)
* [flutter/138030](https://github.com/flutter/flutter/issues/138030) - Fixes file deletion crash which can occur during iOS archive.
* [flutter/134716](https://github.com/flutter/flutter/issues/134716) - Fix iOS 17 keyboard freeze when switching languages
* [flutter/138180](https://github.com/flutter/flutter/issues/138180) - Prevents a crash in flutter doctor for macOS users who have an IntelliJ or Android Studio installation with a missing CFBundleIdentifier in its plist.
* [flutter/138040](https://github.com/flutter/flutter/issues/138040) - Ignore exceptions in Flutter tool when trying to set the echo mode of the terminal when the STDIN pipe has been broken.
* [flutter/124145](https://github.com/flutter/flutter/issues/124145) - Fixes a JSON array parsing bug that causes seg fault when --coverage is used

### [3.16.0](https://github.com/flutter/flutter/releases/tag/3.16.0) (Nov 15, 2023)
Initial stable release.

## Flutter 3.13 Changes

### [3.13.9](https://github.com/flutter/flutter/releases/tag/3.13.9) (October 25, 2023)
* [dart/53784](https://github.com/dart-lang/sdk/issues/53784) - [dart2js] Fixes compatibility with Node.js 21

### [3.13.8](https://github.com/flutter/flutter/releases/tag/3.13.8) (October 18, 2023)
* [dart/53747](https://github.com/dart-lang/sdk/issues/53747) -  Fixes a visual issue in the Dart VM preventing users from seeing variable values when debugging.
* [flutter/136552](https://github.com/flutter/flutter/issues/136552) - [iOS] Fixes issues with voice over when visiting a PlatformView in iOS applications.
* [flutter/136654](https://github.com/flutter/flutter/issues/136654) - [Android] Fixes rendering issues when using PlatformViews in Android applications on high refresh rate phones.

### [3.13.7](https://github.com/flutter/flutter/releases/tag/3.13.7) (October 11, 2023)
* [flutter/135442](https://github.com/flutter/flutter/issues/135442) - Fix Xcode 15 launch failure with iOS 17

### [3.13.6](https://github.com/flutter/flutter/releases/tag/3.13.6) (September 27, 2023)
* [flutter/133013](https://github.com/flutter/flutter/issues/133013) - [Impeller] Fix issues with PNG decompression
* [flutter/132838](https://github.com/flutter/flutter/issues/132838)  - Fix clip Imagefilter.blur on iOS
* [dart/53579](https://github.com/dart-lang/sdk/issues/53579) - Fixes a compiler crash when using @staticInterop or @anonymous factory constructors with type parameters.
* [dart/53503](https://github.com/dart-lang/sdk/issues/53503) - Fixes segmentation faults that terminate processes when encountering handled exceptions in the FFI library.
* [dart/53541](https://github.com/dart-lang/sdk/issues/53541) - Fixes slow variable access while debugging Flutter applications.

### [3.13.5](https://github.com/flutter/flutter/releases/tag/3.13.5) (September 20, 2023)
* [flutter/134825](https://github.com/flutter/flutter/issues/134825) - Fixes an issue where apps built in profile mode would not install or run on physical iOS 17 devices.
* [flutter/45598](https://github.com/flutter/engine/pull/45598) - Fix permissions on macos artifacts making mac framework readable and executable by all

### [3.13.4](https://github.com/flutter/flutter/releases/tag/3.13.4) (September 13, 2023)
* [dart/53449](https://github.com/dart-lang/sdk/issues/53449) - Fixes a dart2js issue causing a compiler crash when using a typed record pattern outside of the scope of a function body.
* [dart/53450](https://github.com/dart-lang/sdk/issues/53450) - Fixes a pause in the debugger when reaching an unhandled exception.
* [flutter/133658](https://github.com/flutter/flutter/issues/133658) - Fixes crash when using the --analyze-size argument.
* [flutter/133890](https://github.com/flutter/flutter/issues/133890) - Fixes incorrect autocorrect highlights in text fields in iOS 17.
* [flutter/134468](https://github.com/flutter/flutter/issues/134468) - Fixes an issue where users are not able to input text for IME language in iOS 17.
* [flutter/45742](https://github.com/flutter/engine/pull/45742) - Fixes CVE-2023-4863 - Security vulnerability in WebP.

### [3.13.3](https://github.com/flutter/flutter/releases/tag/3.13.3) (September 7, 2023)

* [flutter/133147](https://github.com/flutter/flutter/issues/133147) - fixes image-picker crashes on iOS
* [flutter/133069](https://github.com/flutter/flutter/issues/133069) - fixes issue where console prints dart:ui_web warnings in new flutter project
* [flutter/133441](https://github.com/flutter/flutter/issues/133441) - fixes issue where `flutter upgrade` crashes and reports "unknown flutter tag".
* [flutter/133055](https://github.com/flutter/flutter/issues/133055) - fixes issue where running `flutter doctor` crashes on FileSystemException
* [flutter/132788](https://github.com/flutter/flutter/issues/132788) - fixes a visual overflow caused by SliverMainAxisGroup where clip behavior isn’t applied

### [3.13.2](https://github.com/flutter/flutter/releases/tag/3.13.2) (August 30, 2023)

* [flutter/132764](https://github.com/flutter/flutter/pull/132764) - Fixes lower bound of children from TwoDimensionalChildBuilderDelegate.

### [3.13.1](https://github.com/flutter/flutter/releases/tag/3.13.1) (August 23, 2023)

* [flutter/132883](https://github.com/flutter/flutter/issues/132883) - Fixes an issue where Flutter apps would not compile when using custom icon fonts that contain spaces.
* [flutter/132959](https://github.com/flutter/flutter/issues/132959) - Fixes an issue where macOS applications using plugins with Xcode 15 would not compile.
* [flutter/132763](https://github.com/flutter/flutter/issues/132763) - Fixes auto-correction position in iOS 17.
* [flutter/132982](https://github.com/flutter/flutter/issues/132982) - [Impeller] Fixes an issue where applications would freeze if the app was minimized while an animation was occurring.

## Flutter 3.10 Changes

### [3.10.6](https://github.com/flutter/flutter/releases/tag/3.10.6) (July 12, 2023)

* [flutter/129161](https://github.com/flutter/flutter/issues/129161) - Fix regression in the GestureRecognizers used by the TextField where it would not fire the onTapDown or onTapUp callbacks which made selection not work
* [flutter/130084](https://github.com/flutter/flutter/issues/130084) - Using canvas.drawPicture where the nested picture fails to restore clips established in the child picture and makes content disappear.
* [dart/52767](https://github.com/dart-lang/sdk/issues/52767) - Fixes a flow in flow analysis that causes it to sometimes ignore destructuring assignments.
* [dart/52869](https://github.com/dart-lang/sdk/issues/52869) - Fixes an infinite loop in some web development compiles that include `is` or `as` expressions involving record types with named fields.
* [dart/52791](https://github.com/dart-lang/sdk/issues/52791) - Fixes a memory leak in Dart analyzer's file-watching.
* [dart/52793](https://github.com/dart-lang/sdk/issues/52793) - Fixes a memory leak of file system watcher related data structures.

### [3.10.5](https://github.com/flutter/flutter/releases/tag/3.10.5) (June 14, 2023)

* [flutter/127628](https://github.com/flutter/flutter/pull/127628) -  Fixes an issue preventing the use of `integration_test` when using AGP 8.0.
* [flutter/126043](https://github.com/flutter/flutter/issues/126403) - Fixes an error encountered when attempting to use `add-to-app` on Android when generating Flutter modules.
* [flutter/127090](https://github.com/flutter/flutter/issues/127090) - Fixes an issue preventing assets from being displayed properly on low pixel density devices.
* [flutter/128320](https://github.com/flutter/flutter/issues/128230) - Fixes an issue where image assets are not displayed when serving with Microsoft IIS.
* [dart/52403](https://github.com/dart-lang/sdk/issues/52403) - Fixes a bad cast in the frontend which can manifest as a crash in the dart2js
`ListFactorySpecializer` during Flutter web builds.
* [dart/1224](https://github.com/dart-lang/dart_style/issues/1224) - Handles formatting nullable record types with no fields.
* [dart/52480](https://github.com/dart-lang/sdk/issues/52480) - Fixes error when using records when targeting the web in development mode.

### [3.10.4](https://github.com/flutter/flutter/releases/tag/3.10.4) (June 07, 2023)

* [flutter/127836](https://github.com/flutter/flutter/issues/127836) - Fixes SliverAppBar's FlexibleSpaceBar overlaps


### [3.10.3](https://github.com/flutter/flutter/releases/tag/3.10.3) (June 02, 2023)

* [flutter/126435](https://github.com/flutter/flutter/issues/126435) - Fixes the position of `SearchAnchor` when used in a nested navigator.
* [flutter/127486](https://github.com/flutter/flutter/issues/127486) - [Impeller] Fixes an issue causing noise when using combinations of UV mapping and color blending.
* [flutter/126878](https://github.com/flutter/flutter/issues/126878) - [Impeller] Fixes an issue where images do not appear on iOS devices.
* [flutter/1127587](https://github.com/flutter/flutter/issues/124612) - [Impeller] Fixes a crash when applying backdrop blurs to platform views.
* [flutter/127103](https://github.com/flutter/flutter/issues/127103) - [Impeller] Fixes an issue where text is not rendered correctly when a transform is applied.
* [flutter/126487](https://github.com/flutter/flutter/issues/126487) - [Impeller] Fixes an issue where blur is not respected at certain value.
* [dart/52449](https://github.com/dart-lang/sdk/issues/52449) - Fixes an AOT compiler crash when generating an implicit getter returning an unboxed record.
* [dart/52373](https://github.com/dart-lang/sdk/issues/52373) - Fixes a situation in which variables appearing in multiple branches of an or-pattern might be erroneously reported as being mismatched.
* [dart/52334](https://github.com/dart-lang/sdk/issues/52334) - Adds missing `interface` modifiers on the purely abstract classes `MultiStreamController`, `StreamConsumer`, `StreamIterator` and `StreamTransformer`.
* [dart/52373](https://github.com/dart-lang/sdk/issues/52373) - Fixes an error during debugging when `InternetAddress.tryParse` is used.
* [dart/126884](https://github.com/flutter/flutter/issues/126884) - Fixes a VM issue causing crashes on hot reload.
* [dart/4195](https://github.com/dart-lang/linter/issues/4195) - Improves linter support.
* [dart/52439](https://github.com/dart-lang/sdk/issues/52439) - Fixes an issue in variable patterns preventing users from expressing a pattern match using a variable or wildcard pattern with a nullable record type.
* [dart/52386](https://github.com/dart-lang/sdk/issues/52386) - Updates warnings and provide instructions for updating the Dart pub cache on Windows.

### [3.10.2](https://github.com/flutter/flutter/releases/tag/3.10.2) (May 24, 2023)
This hotfix release addresses the following issues:
* [flutter/126532](https://github.com/flutter/flutter/issues/126532) - [Impeller] Fixes saveLayer ignores opacity of paint with blend mode lighten.
* [flutter/126739](https://github.com/flutter/flutter/issues/126739) - [Impeller] Fixes ImageShader alignment is different for different PaintingStyle.
* [flutter/126701](https://github.com/flutter/flutter/issues/126701) - [Impeller] Fixes InkSparkle splash not clipping on iOS.
* [flutter/126661](https://github.com/flutter/flutter/issues/126661) - Fixes PointerInterceptor reverses transformHitTests in a scaled context.
* [flutter/127183](https://github.com/flutter/flutter/issues/127183) - [Impeller] Fixes drawing path with image shader is not correct.
* [dart/52438](https://github.com/dart-lang/sdk/issues/52438) - Fixes a dart2js crash when using a switch case expression on a record where the fields don't match the cases.
* [dart/3392](https://github.com/dart-lang/dartdoc/issues/3392) - Add chips for class and mixin pages on dartdoc generated pages.
* [dart/52352](https://github.com/dart-lang/sdk/issues/52352) - Fixes a situation causing the parser to fail resulting in an infinite loop leading to higher memory usage.
* [dart/52078](https://github.com/dart-lang/sdk/issues/52078) -  Add clear errors when mixing inheritance in pre and post Dart 3 libraries.


### [3.10.1](https://github.com/flutter/flutter/releases/tag/3.10.1) (May 17, 2023)

This hotfix release addresses the following issues:
* [flutter/126510](https://github.com/flutter/flutter/issues/125276) - [Impeller] Fixes errors in text transformation when using impeller.
* [flutter/126854](https://github.com/flutter/flutter/issues/126854) - [Impeller] Fixes visual glitches and crashes when using wide gamut color support on iOS.
* [flutter/124883](https://github.com/flutter/flutter/issues/124883) - Fixes an issue where images do not render on Flutter web apps when the host machine has Internet Download Manager installed.
* [flutter/126491](https://github.com/flutter/flutter/issues/126491) - Fixes an issue where `CupertinoPicker` and `ListWheelViewport` crash with certain configurations on development builds.
* [flutter/124529](https://github.com/flutter/flutter/issues/124529) - Fixes an issue where iOS and macOS apps will not build when using Xcode 14.3 and adding dependencies with low iOS target versions.
* [flutter/122376](https://github.com/flutter/flutter/issues/122376) - Adds a migrator to update the Gradle version when it conflicts with the Android Studio version of Java is detected.
* [dart/124369](https://github.com/flutter/flutter/issues/124369) - Fixes a compiler crash involving redirecting factories and FFI.
* [dart/51899](https://github.com/dart-lang/sdk/issues/51899) - Fixes a dart2js crash when using a combination of local functions, generics, and records.
* [dart/52191](https://github.com/dart-lang/sdk/issues/52191) - Fixes incorrect error using a void in a switch case expression.
* [dart/52041](https://github.com/dart-lang/sdk/issues/52041) - Fixes a false error when using in switch case expressions when the switch refers to a private getter.
* [dart/52260](https://github.com/dart-lang/sdk/issues/52260) - Prevent the use of when and as as variable names in patterns.
* [dart/52241](https://github.com/dart-lang/sdk/issues/52241) - Fixes an inconsistency in type promotion between the analyzer and VM.
* [dart/1212](https://github.com/dart-lang/dart_style/issues/1212) - Improve performance on functions with many parameters.

### [3.10.0](https://github.com/flutter/flutter/releases/tag/3.10.0) (May 10, 2023)
Initial stable release.

## Flutter 3.7 Changes

### [3.7.12](https://github.com/flutter/flutter/releases/tag/3.7.12) (Apr 19, 2023)

This hotfix release addresses the following issues:

* [flutter/124838](https://github.com/flutter/flutter/issues/124838) - Support Gradle 8

### [3.7.11](https://github.com/flutter/flutter/releases/tag/3.7.11) (Apr 12, 2023)

This hotfix release addresses the following issues:

* [flutter/124529](https://github.com/flutter/flutter/issues/124529) - Fix Xcode 14.3 will not build when plugin transitive dependencies have a low deployment target
  * [flutter/124340](https://github.com/flutter/flutter/issues/124340) - Fixes an issue where iOS and MacOS fail to build when targeting low deployment targets when using xCode 14.3.
* [flutter/124208](https://github.com/flutter/flutter/issues/124208) - Fix orientation preferences on iOS 16+
  * [flutter/116711](https://github.com/flutter/flutter/issues/116711) - Fixes an issue where orientation preferences are not respected on iOS 16 and above.
* [flutter/124403](https://github.com/flutter/flutter/issues/124403) - Clarify errors around Java/Gradle incompatibility
  * [flutter/122376](https://github.com/flutter/flutter/issues/122376) - Clarify errors around Java/Gradle incompatibility.

### [3.7.10](https://github.com/flutter/flutter/releases/tag/3.7.10) (Apr 05, 2023)
This hotfix release addresses the following issues:
* [flutter/123890](https://github.com/flutter/flutter/issues/123890) - Fixes an issue where upgrading to Xcode 14.3 breaks the ability to publish iOS and macOS applications.

### [3.7.9](https://github.com/flutter/flutter/releases/tag/3.7.9) (Mar 30, 2023)
This hotfix release addresses the following issues:
* [dart/51798](https://github.com/dart-lang/sdk/issues/51798) - Fixes a false `Out of Memory` exception causing slowdowns.

### [3.7.8](https://github.com/flutter/flutter/releases/tag/3.7.8) (Mar 22, 2023)
This hotfix release addresses the following issues:
* [flutter/119441](https://github.com/flutter/flutter/issues/119441) - Fixes an issue where the `Toolbar` widget is incorrectly positioned when inside of a textfield in the Appbar.

### [3.7.7](https://github.com/flutter/flutter/releases/tag/3.7.7) (Mar 08, 2023)
This hotfix release addresses the following issues:
* [flutter/121256](https://github.com/flutter/flutter/issues/121256) - Fixes an issue where Android users can not use add2app because it can not locate build/host/apk/app-debug.apk.
* [engine/120455](https://github.com/flutter/flutter/issues/120455)
Cached DisplayList opacity inheritance fix.
* [dart/121270](https://github.com/flutter/flutter/issues/121270) - Fixes mobile device VM crashes caused by particular use of RegExp on mobile devices.

### [3.7.6](https://github.com/flutter/flutter/releases/tag/3.7.6) (Mar 01, 2023)
This hotfix release addresses the following issues:
* [dart/50981](https://github.com/dart-lang/sdk/issues/50981) - Improve performance of Dart Analysis Server by limiting the analysis context to 1.
* [dart/51481](https://github.com/dart-lang/sdk/issues/51481) - Update DDC test and builder configuration
* [flutter/114031](https://github.com/flutter/flutter/issues/114031) - Fixes a crash when using `flutter doctor --android-licenses` on macOS.
* [flutter/106674](https://github.com/flutter/flutter/issues/106674) - Fixes an issue where Flutter is unable to find the current JDK in specific versions of Android Studio.

### [3.7.5](https://github.com/flutter/flutter/releases/tag/3.7.5) (Feb 22, 2023)
This hotfix release addresses the following issues:
* [flutter/119180](https://github.com/flutter/flutter/issues/119180) - Apple Pencil writes on Flutter apps instead of scrolling when outside of a text field.
* [flutter/120220](https://github.com/flutter/flutter/issues/120220) - [Impeller] Flutter apps may crash when some clip operations are used.

### [3.7.4](https://github.com/flutter/flutter/releases/tag/3.7.4) (Feb 21, 2023)
This hotfix release addresses the following issues:
* [flutter/116360](https://github.com/flutter/flutter/issues/116360) - Flutter web apps will not load if accessed through any other path than `/`.
* [flutter/119557](https://github.com/flutter/flutter/issues/119557) - Localization files incorrectly overridden stopping Flutter applications from running.
* [flutter/116459](https://github.com/flutter/flutter/issues/116459) - Localization files do not parse when using numbers as select cases.

### [3.7.3](https://github.com/flutter/flutter/releases/tag/3.7.3) (Feb 9, 2023)
This hotfix release addresses the following issues:
* [flutter/119507](https://github.com/flutter/flutter/issues/119507) - Asset inclusion regression can cause unexpected app bundle size increase
* [flutter/119289](https://github.com/flutter/flutter/issues/119289) - [Impeller] ImageFilter.blur Edge sampling issue.
* [flutter/119950](https://github.com/flutter/flutter/issues/119950) - [Impeller] Improve blur performance for Android and iPad Pro.
* [flutter/119190](https://github.com/flutter/flutter/pull/119190) - Fix lexer issue where select/plural/other/underscores cannot be in identifier names.

### [3.7.2](https://github.com/flutter/flutter/releases/tag/3.7.2) (Feb 8, 2023)
This hotfix release addresses the following issues:
* [flutter/119881](https://github.com/flutter/flutter/issues/119881) - [Impeller] App performance decreases when using emulated dashed lines.
* [flutter/119245](https://github.com/flutter/flutter/issues/119245) - [Impeller] App crashes due to invalid textures when using impeller.
* [flutter/119489](https://github.com/flutter/flutter/issues/119489) - [Impeller] Text glyphs render incorrectly on different font weights
* [flutter/103847](https://github.com/flutter/flutter/issues/103847) - Fix animation jank on some iPhone models.
* [flutter/119593](https://github.com/flutter/flutter/issues/119593) - Localization files fail to generate when `FLUTTER_STORAGE_BASE_URL` is overridden.
* [flutter/119084](https://github.com/flutter/flutter/issues/119084) - When requesting to evaluate multiple expressions while debugging Flutter web apps, tooling fails before finishing operations.
* [flutter/119261](https://github.com/flutter/flutter/issues/119261) - Flutter tool crashes when attempting to update the artifact cache.
* [flutter/117420](https://github.com/flutter/flutter/issues/117420) - Ink ripple is rendered incorrectly inside of the `NavigationBar` widget when using Material 3.
* [dart/50622](https://github.com/dart-lang/sdk/issues/50622) - VM crashes when mixing the use of double and float calculations in debug/JIT configuration.
* [flutter/119220](https://github.com/flutter/flutter/issues/119220) - Compiler may crash when attempting to inline a method with lots of optional parameters with distinct default values.
* [dart/51087](https://github.com/dart-lang/sdk/issues/51087) - `part_of_different_library` error may be encountered when using `PackageBuildWorkspace`.

### [3.7.1](https://github.com/flutter/flutter/releases/tag/3.7.1) (Feb 1, 2023)
This hotfix release addresses the following issues:
* [flutter/116782](https://github.com/flutter/flutter/issues/116782) - Material 3 Navigation Drawer does not support scrolling or safe areas
* [flutter/119414](https://github.com/flutter/flutter/issues/119414) - ImageFilter in ListView causes wrong offset on Android and iOS
* [flutter/119181](https://github.com/flutter/flutter/issues/119181) - CastError when running `flutter pub get`
* [flutter/118613](https://github.com/flutter/flutter/issues/118613) - [Impeller] Fonts are blurry when rendering on iOS
* [flutter/118945](https://github.com/flutter/flutter/issues/118945) - [Impeller] Objects with large stroke width not drawn correctly on iOS
* [flutter/117428](https://github.com/flutter/flutter/issues/117428) - [Impeller] Text is transformed incorrectly on iOS
* [flutter/119072](https://github.com/flutter/flutter/issues/119072) - [Impeller] Draw calls could be improperly culled
* [flutter/118847](https://github.com/flutter/flutter/issues/118847) - [Impeller] Float samplers can get re-ordered compared to SkSL
* [flutter/119014](https://github.com/flutter/flutter/issues/119014) - Replace iPhone 6s with iPhone 11 as flutter test devices

### [3.7.0](https://github.com/flutter/flutter/releases/tag/3.7.0) (Jan 24, 2023)
Initial stable release.

## Flutter 3.3 Changes

### [3.3.10](https://github.com/flutter/flutter/releases/tag/3.3.10) (Dec 16, 2022)
This hotfix release addresses the following issues:
* [flutter/113314](https://github.com/flutter/flutter/issues/113314) - Glitches appear when scrolling on Android TV devices.
* [flutter/80401](https://github.com/flutter/flutter/issues/80401) - Some widgets are not visible when nested inside of `ClipRRect` in CanvasKit mode when using Flutter web on Safari.

### [3.3.9](https://github.com/flutter/flutter/releases/tag/3.3.9) (Nov 23, 2022)
This hotfix release addresses the following issues:
* [dart/50199](https://github.com/dart-lang/sdk/issues/50119) - fix error when using private variable setters in mixins on dart web.
* [dart/50392](https://github.com/dart-lang/sdk/issues/50392) - Type parameter nullability performs incorrectly in factory constructors.

### [3.3.8](https://github.com/flutter/flutter/releases/tag/3.3.8) (Nov 09, 2022)
This hotfix release addresses the following issues:
* [flutter/113973](https://github.com/flutter/flutter/issues/113973) - Fix null safety issue in TextFormField when Android devices pass no data
* [flutter/109632](https://github.com/flutter/flutter/issues/109632) - Fix type conversion in TextInput that didn’t allow num types

### [3.3.7](https://github.com/flutter/flutter/releases/tag/3.3.6) (Nov 2, 2022)
This hotfix release addresses the following issues:
* [flutter/113550](https://github.com/flutter/flutter/issues/113550) - Fix unnecessary null safe exceptions in input decorators on Android
* [flutter/100522](https://github.com/flutter/flutter/issues/100522) - Speculative fix for iOS screen flickering

### [3.3.6](https://github.com/flutter/flutter/releases/tag/3.3.6) (Oct 26, 2022)
This hotfix release addresses the following issues:
* [flutter/111255](https://github.com/flutter/flutter/issues/111255) - Using WebView leads to size error in platform_views since Flutter 3.3.0

### [3.3.5](https://github.com/flutter/flutter/releases/tag/3.3.5) (Oct 19, 2022)
This hotfix release addresses the following issues:
* [flutter/113035](https://github.com/flutter/flutter/pull/113035) - Apps crash when `FadeInImage` switches from cached to uncached images.
* [flutter/112228](https://github.com/flutter/flutter/pull/112228) - Move documentation build and deployment to post-submit.
* [flutter/36807](https://github.com/flutter/engine/pull/36807) - Apps crash when combining emojis and Korean text.
* [flutter/112887](https://github.com/flutter/flutter/pull/112887) - When debugging web apps, erroneous errors are displayed.

### [3.3.4](https://github.com/flutter/flutter/releases/tag/3.3.4) (Oct 05, 2022)
This hotfix release addresses the following issues:
* [Flutter/36181](https://github.com/flutter/engine/pull/36181) - On Flutter desktop apps, pixel snapping performs incorrectly when using opacity layers at certain DPRs and screen sizes.
* [flutter/36491](https://github.com/flutter/engine/pull/36491) - On android devices with a refresh rate greater than 60hz, frames jump when scrolling.

### [3.3.3](https://github.com/flutter/flutter/releases/tag/3.3.3) (Sept 28, 2022)
This hotfix release addresses the following issues:
* [flutter/111475](https://github.com/flutter/flutter/issues/111475) - Signing errors on iOS pod bundle resources on Xcode 14 "Signing for "x" requires a development team."
* [flutter/110671](https://github.com/flutter/flutter/issues/110671) - App crashes on latest versions when AnimatedContainer / Container height is set to 0 and throws uncaught exception
* [flutter/107590](https://github.com/flutter/flutter/issues/107590) - Flutter tools ShaderCompilerException with exit code -1073740791.
* [flutter/110640](https://github.com/flutter/flutter/issues/110640) - Fatal crash with java.lang.AssertionError when selecting text in TextField.
* [dart/50075](https://github.com/dart-lang/sdk/issues/50075) - Security vulnerability: There is a auth bypass vulnerability in Dart SDK, specifically dart:uri core library, used to parse and validate URLs.
* [dart/50052](https://github.com/dart-lang/sdk/issues/50052) - Avoid CFE crash when input contains invalid super parameters usage.

### [3.3.2](https://github.com/flutter/flutter/releases/tag/3.3.2) (Sept 14, 2022)
This hotfix release addresses the following issues:
* [flutter/111411](https://github.com/flutter/flutter/issues/111411) - Package assets fail to load.
* [flutter/111296](https://github.com/flutter/flutter/issues/111296) - Custom embedders fail to build for 32 bit targets.
* [flutter/111274](https://github.com/flutter/flutter/issues/111274) - Android plugins crash when using platform view's Virtual Display fallback.
* [flutter/111231](https://github.com/flutter/flutter/issues/111231) - Text rendering is handled incorrectly.
* [dart/49923](https://github.com/dart-lang/sdk/issues/49923) - Incorrect type propagation when using `late` variables in catch blocks.

### [3.3.1](https://github.com/flutter/flutter/releases/tag/3.3.1) (Sept 7, 2022)
This hotfix release addresses the following issues:
* [flutter/110820](https://github.com/flutter/flutter/issues/110820) - Windows apps crash when accessibility is enabled on apps that use widgets with custom semantic actions.

### [3.3.0](https://github.com/flutter/flutter/releases/tag/3.3.0) (Aug 30, 2022)
Initial stable release.
## Flutter 3.0 Changes
### [3.0.5](https://github.com/flutter/flutter/releases/tag/3.0.5) (July 13, 2022)
This hotfix release addresses the following issues:
* [flutter/106601](https://github.com/flutter/flutter/issues/106601) - Flutter tool fails on visual studio on certain locales on Windows.
* [flutter/106510](https://github.com/flutter/flutter/issues/106510) - Flutter crashes on launch on ARM devices.
* [dart/49054](https://github.com/dart-lang/sdk/issues/49054) - Improves code completion for Flutter.
* [dart/49402](https://github.com/dart-lang/sdk/issues/49402) - Compiler crashes when using Finalizable parameters.
### [3.0.4](https://github.com/flutter/flutter/releases/tag/3.0.4) (July 1, 2022)
This hotfix release addresses the following issues:
* [flutter/105183](https://github.com/flutter/flutter/issues/105183) - Pointer compression on iOS causes OOM
* [flutter/103870](https://github.com/flutter/flutter/issues/103870) - Application crashes on system low memory events
* [flutter/105674](https://github.com/flutter/flutter/issues/105674) - Rendering artifacts from ImagedFiltered/ColorFiltered in animated views
### [3.0.3](https://github.com/flutter/flutter/releases/tag/3.0.3) (June 22, 2022)
This hotfix release addresses the following issues:
* [dart/49188](https://github.com/dart-lang/sdk/issues/49188) - Improve analysis of enums and switch.
* [dart/49075](https://github.com/dart-lang/sdk/issues/49075) - Fix compiler crash when initializing Finalizable objects.
### [3.0.2](https://github.com/flutter/flutter/releases/tag/3.0.2) (June 10, 2022)
This hotfix release addresses the following issues:
* [flutter/104785](https://github.com/flutter/flutter/issues/104785) - Flutter web apps show a black screen on Safari 13.
* [flutter/102451](https://github.com/flutter/flutter/issues/102451) - `flutter doctor` crashes for Windows users using Visual Studio 2022.
* [flutter/103846](https://github.com/flutter/flutter/issues/103846) - Unexpected line breaks occur when using new text renderer.
* [flutter/104569](https://github.com/flutter/flutter/pull/104569) - Ink Sparkle slows down applications using Material 3.
* [flutter/103404](https://github.com/flutter/flutter/issues/103404) - SliverReorderableList does not drag on Android devices.
* [flutter/103556](https://github.com/flutter/flutter/issues/103566) - Nested horizontal sliders in widgets with horizontal drag gestures do not work in Android applications.
 * [flutter/100375](https://github.com/flutter/flutter/issues/100375) - Build process fails when building Windows applications.
 * [dart/49027](https://github.com/dart-lang/sdk/issues/49027) - Code suggestion for initState/dispose/setState no longer work on intellij.
* [dart/3424](https://github.com/dart-lang/pub/issues/3424) - `dart pub login` fails when attempting to publish a package.
* [dart/49097](https://github.com/dart-lang/sdk/issues/49097) - `dart analyze` throws errors when using enhance Enums feature.
### [3.0.1](https://github.com/flutter/flutter/releases/tag/3.0.1) (May 19, 2022)
This hotfix release addresses the following issues:
 * [flutter/102947](https://github.com/flutter/flutter/issues/102947) - Radial gradients behave incorrectly when painting text.
### [3.0.0](https://github.com/flutter/flutter/releases/tag/3.0.0) (May 11, 2022)
Initial stable release.
## Flutter 2.10 Changes
### [2.10.5](https://github.com/flutter/flutter/releases/tag/2.10.5) (April 18, 2022)
This hotfix release addresses the following issues:
 * [flutter/101224](https://github.com/flutter/flutter/issues/101224) - Flutter web debugger fails when using chrome 100 or greater.
### [2.10.4](https://github.com/flutter/flutter/releases/tag/2.10.4) (March 28, 2022)
This hotfix release addresses the following issues:
 * [flutter/93871](https://github.com/flutter/flutter/issues/93871) - Custom embedders fail to build when using default sysroot (GCC 11).
 * [dart/48559](https://github.com/dart-lang/sdk/issues/48559) - Flutter web apps crash when using package:freezed.
### [2.10.3](https://github.com/flutter/flutter/releases/tag/2.10.3) (March 02, 2022)
This hotfix release addresses the following issues:
 * [flutter/98973](https://github.com/flutter/flutter/issues/98973) - Deadlock in application startup in profile/release mode.
 * [flutter/98739](https://github.com/flutter/flutter/issues/98739) - ios: Visual glitch when scrolling a list in a Scaffold that has a Material and Container as bottomNavigationBar.
 * [flutter/97086](https://github.com/flutter/flutter/issues/97086) - Windows: Fail to launch app in debug mode.
### [2.10.2](https://github.com/flutter/flutter/releases/tag/2.10.2) (February 18, 2022)
This hotfix release addresses the following issues:
 * [flutter/95211](https://github.com/flutter/flutter/issues/95211) - Transform animation with BackdropFilter is causing a crash.
 * [flutter/98155](https://github.com/flutter/flutter/issues/98155) - App crashes after upgrading to 2.10.x using webview + video_player plugin.
 * [flutter/98361](https://github.com/flutter/flutter/issues/98361) - Error in DL bounds calculations causes incorrect SVG rendering.
 * [flutter/97767](https://github.com/flutter/flutter/issues/97767) - New material icons are not properly rendered.
 * [flutter/95711](https://github.com/flutter/flutter/issues/95711) - Linux builds default to building GLFW.
### [2.10.1](https://github.com/flutter/flutter/releases/tag/2.10.1) (February 9, 2022)
This hotfix release addresses the following issues:
 * [flutter/94043](https://github.com/flutter/flutter/issues/94043) - Autofill does not work in `TextField`.
 * [flutter/96411](https://github.com/flutter/flutter/issues/96411) - Safari: Unable to enter text into `TextField`.
 * [flutter/96661](https://github.com/flutter/flutter/issues/96661) - Platform views throw fatal exception: Methods marked with @UiThread must be executed on the main thread.
 * [flutter/97103](https://github.com/flutter/flutter/issues/97103) - Images become corrupted when using CanvasKit.
 * [flutter/97679](https://github.com/flutter/flutter/issues/97679) - Don't remove overlay views when the rasterizer is being torn down.
 * [dart/48301](https://github.com/dart-lang/sdk/issues/48301) - Avoid speculative conversion in ffi Pointer.asTypedList.
### [2.10.0](https://github.com/flutter/flutter/releases/tag/2.10.0) (February 3, 2022)
Initial stable release.
## Flutter 2.8 Changes
### [2.8.1](https://github.com/flutter/flutter/releases/tag/2.8.1) (December 16, 2021)
This hotfix release addresses the following issues:
 * [flutter/94914](https://github.com/flutter/flutter/issues/94914) - Apps using `google_sign_in` or `google_maps` don't build in iOS Simulator on ARM macOS
 * [flutter/90783](https://github.com/flutter/flutter/issues/90783) - In rare circumstances, engine may crash during app termination on iOS and macOS
 * [dart/47914](https://github.com/dart-lang/sdk/issues/47914) - AOT compilation fails with error "Invalid argument(s): Missing canonical name for Reference"
 * [dart/47815](https://github.com/dart-lang/sdk/issues/47815) - Running `dart pub publish` with custom pub package server that has URL containing a path may fail.

### [2.8.0](https://github.com/flutter/flutter/releases/tag/2.8.0) (December 8, 2021)
Initial stable release.

## Flutter 2.5 Changes
### [2.5.3](https://github.com/flutter/flutter/releases/tag/2.5.3) (October 15, 2021)
This hotfix release addresses the following issues:
 * [dart/47321](https://github.com/dart-lang/sdk/issues/47321) - Fix a potential out-of-memory condition with analysis server plugins
 * [dart/47432](https://github.com/dart-lang/sdk/issues/47432) - Fix certificate loading on Windows when there are expired certificates
 * [flutter/83792](https://github.com/flutter/flutter/issues/83792) - Fix HTTPS issue related to: "HttpClient throws Invalid argument(s): Invalid internet address"

### [2.5.2]((https://github.com/flutter/flutter/releases/tag/2.5.2)) (September 30, 2021)
This hotfix release addresses the following issues:
 * [dart/47285](https://github.com/dart-lang/sdk/issues/47285) - Fix a regression to the performance of code completions
 * [dart/47316](https://github.com/dart-lang/sdk/issues/47316) - Dynamic tables in ELF files have invalid relocated addresses
 * [flutter/89912](https://github.com/flutter/flutter/issues/89912) - Building iOS app generates unnecessary Flutter.build folder

### [2.5.1]((https://github.com/flutter/flutter/releases/tag/2.5.1)) (September 17, 2021)
This hotfix release addresses the following issues:
 * [flutter/88767](https://github.com/flutter/flutter/issues/88767) - java.lang.SecurityException: Permission denial crash at launch
 * [flutter/88236](https://github.com/flutter/flutter/issues/88236) - null check exception during keyboard keypress
 * [flutter/88221](https://github.com/flutter/flutter/issues/88221) - Material routes delayed on push and pop
 * [flutter/84113](https://github.com/flutter/flutter/issues/84113) - HTTP exceptions talking to VM Service
 * [flutter/83632](https://github.com/flutter/flutter/issues/83632) - Scroll view velocity too high

### 2.5.0 (September 8, 2021)
Initial stable release.

## Flutter 2.2 Changes
### [2.2.3](https://github.com/flutter/flutter/pull/85719) (July 2, 2021)
This hotfix release addresses the following issues:
  * [flutter/84212](https://github.com/flutter/flutter/issues/84212) - Upgrading to 2.2.1 cause main.dart to crash
  * [flutter/83213](https://github.com/flutter/flutter/issues/83213) - TextFormField not responding to inputs on Android when typing on Microsoft SwiftKey
  * [flutter/82838](https://github.com/flutter/flutter/issues/82838) - Flutter Web failing to compile with "Undetermined Nullability"
  * [flutter/82874](https://github.com/flutter/flutter/issues/82874) - PopupMenuButton is broken after upgrade to Flutter 2.2.

### [2.2.2](https://github.com/flutter/flutter/pull/84364) (June 11, 2021)
This hotfix release addresses the following issues:
  *  [dart/46249](https://github.com/dart-lang/sdk/issues/46249) - Ensure start/stop file watching requests are run on the dart thread.
  *  [dart/46210](https://github.com/dart-lang/sdk/issues/46210) - Fix an analyze crash when analyzing against package:meta v1.4.0
  *  [dart/46173](https://github.com/dart-lang/sdk/issues/46173) - Merge a3767f7db86a85fcd6201e9357ad47b884002b66 to stable channel (2.13)
  *  [dart/46300](https://github.com/dart-lang/sdk/issues/46300) - Fix OOM VM test (`transferable_throws_oom_test` crashing after upgrade from Ubuntu 16)
  *  [dart/46298](https://github.com/dart-lang/sdk/issues/46298) - Ensure start/stop file watching requests are run on the Dart thread
  *  [flutter/83799](https://github.com/flutter/flutter/issues/83799) - Tool may crash if pub is missing from the artifact cache
  *  [flutter/83102](https://github.com/flutter/flutter/issues/83102) - Generated l10n file is missing ‘intl’ import with Flutter 2.2.0
  *  [flutter/83094](https://github.com/flutter/flutter/issues/83094) - Flutter AOT precompiler crash
  *  [flutter/82874](https://github.com/flutter/flutter/issues/82874) - PopupMenuButton is broken after upgrade to Flutter 2.2.

### [2.2.1](https://github.com/flutter/flutter/pull/83372) (May 27, 2021)
This hotfix release addresses the following issues:
 - [flutter/80978](https://github.com/flutter/flutter/issues/80978) - Error "Command PhaseScriptExecution failed with a nonzero exit code" when building on macOS
 - [dart/45990](https://github.com/dart-lang/sdk/issues/45990) - CastMap performs an invalid cast on 'remove', breaking shared_preferences plugin
 - [dart/45907](https://github.com/dart-lang/sdk/issues/45907) - DDC missing nullability information from recursive type hierarchies
 - [flutter/52106](https://github.com/flutter/flutter/issues/52106) - [Web] Accessibility focus border doesn’t follow when navigating through interactive elements with tab key
 - [flutter/82768](https://github.com/flutter/flutter/issues/82768) - [Web] svgClip memory leak in Canvaskit renderer

### 2.2.0 (May 18, 2021)
Initial stable release.

## Flutter 2.0 Changes
### [2.0.6](https://github.com/flutter/flutter/pull/81508) (April 29, 2021)
This hotfix release addresses the following issue:
 - [flutter/81326](https://github.com/flutter/flutter/issues/81326) - macOS binaries not codesigned

### [2.0.5](https://github.com/flutter/flutter/pull/80570) (April 16, 2021)
This hotfix release addresses the following issue:
 - [dart/45306](https://github.com/dart-lang/sdk/issues/45306) - Segmentation fault on specific code

### [2.0.4](https://github.com/flutter/flutter/pull/79486) (April 2, 2021)
This hotfix release addresses the following issues:
 - [flutter/78589](https://github.com/flutter/flutter/issues/78589) - Cocoapod transitive dependencies with bitcode fail to link against debug Flutter framework
 - [flutter/76122](https://github.com/flutter/flutter/issues/76122) - Adding a WidgetSpan widget causes web HTML renderer painting issue
 - [flutter/75280](https://github.com/flutter/flutter/issues/75280) - Dragging the "draggable" widget causes widget to freeze in the overlay layer on Web

### [2.0.3](https://github.com/flutter/flutter/pull/78489) (March 19, 2021)
This hotfix release addresses the following issues:
 - [flutter/75261](https://github.com/flutter/flutter/issues/75261) - Unable to deep link into Android app
 - [flutter/78167](https://github.com/flutter/flutter/issues/78167) - Flutter crash after going to version 2
 - [flutter/75677](https://github.com/flutter/flutter/issues/75677) - NoSuchMethodError: The method 'cancel' was called on null at AnsiSpinner.finish
 - [flutter/77419](https://github.com/flutter/flutter/pull/77419) - Fix Autovalidate enum references in fix data

### [2.0.2](https://github.com/flutter/flutter/pull/77850) (March 12, 2021)
This hotfix release addresses the following issues:
  - [flutter/77251](https://github.com/flutter/flutter/issues/77251) - Flutter may show multiple snackbars when Scaffold is nested
  - [flutter/75473](https://github.com/flutter/flutter/issues/75473) - CanvasKit throws error when using Path.from
  - [flutter/76597](https://github.com/flutter/flutter/issues/76597) - When multiple Flutter engines are active, destroying one engine causes crash
  - [flutter/75061](https://github.com/flutter/flutter/issues/75061) - '_initialButtons == kPrimaryButton': is not true
  - [flutter/77419](https://github.com/flutter/flutter/pull/77419) - Fix Autovalidate enum references in fix data
  - [dart/45214](https://github.com/dart-lang/sdk/issues/45214) - Bad state exception can occur when HTTPS connection attempt errors or is aborted
  - [dart/45140](https://github.com/dart-lang/sdk/issues/45140) - Uint8List reports type exception while using + operator in null safety mode

### [2.0.1](https://github.com/flutter/flutter/pull/77194) (March 5, 2021)
This hotfix release addresses the following issue:
  - [flutter/77173](https://github.com/flutter/flutter/issues/77173) - Building for macOS target fails when Flutter is installed from website

### 2.0.0 (March 3, 2021)
Initial stable release.

## Flutter 1.22 Changes
### [1.22.6](https://github.com/flutter/flutter/pull/74355) (Jan 25, 2021)
This hotfix release addresses the following issue:
  - [flutter/70895](https://github.com/flutter/flutter/issues/70895) - Build error when switching between dev/beta and stable branches.

### [1.22.5](https://github.com/flutter/flutter/pull/72079) (Dec 10, 2020)
This hotfix release addresses the following issue:
  - [flutter/70577](https://github.com/flutter/flutter/issues/70577) - Reliability regression in the camera plugin on iOS

### [1.22.4](https://github.com/flutter/flutter/pull/70327) (Nov 13, 2020)
This hotfix release addresses the following issues:
  - [flutter/43620](https://github.com/flutter/flutter/issues/43620) - Dart analyzer terminates during development
  - [flutter/58200](https://github.com/flutter/flutter/issues/58200) - Apple AppStore submission fails with error: “The bundle Runner.app/Frameworks/App.framework does not sue Infpport the minimum OS Version specified in the Info.plist”
  - [flutter/69722](https://github.com/flutter/flutter/issues/69722) - Setting a custom observatory port for debugging does not take effect
  - [flutter/66144](https://github.com/flutter/flutter/issues/66144) - Setting autoFillHint to text form field may cause focus issues
  - [flutter/69449](https://github.com/flutter/flutter/issues/69449) - Potential race condition in FlutterPlatformViewsController
  - [flutter/65133](https://github.com/flutter/flutter/issues/65133) - Support targeting physical iOS devices on Apple Silicon

### [1.22.3](https://github.com/flutter/flutter/pull/69234) (October 30, 2020)
This hotfix release addresses the following issues:
  - [flutter/67828](https://github.com/flutter/flutter/issues/67828) - Multiple taps required to delete text in some input fields.
  - [flutter/66108](https://github.com/flutter/flutter/issues/66108) - Reading Android clipboard may throw a security exception if it contains media

### [1.22.2](https://github.com/flutter/flutter/pull/68135)  (October 16, 2020)
This hotfix release addresses the following issues:
  - [flutter/67869](https://github.com/flutter/flutter/issues/67869) - Stylus tap gesture is improperly registered.
  - [flutter/67986](https://github.com/flutter/flutter/issues/67986) - Android Studio 4.1 not properly supported.
  - [flutter/67213](https://github.com/flutter/flutter/issues/67213) - Webviews in hybrid composition can cause a crash.
  - [flutter/67345](https://github.com/flutter/flutter/issues/67345) - VoiceOver accessibility issue with some pages.
  - [flutter/66764](https://github.com/flutter/flutter/issues/66764) - Native webviews may not be properly disposed of in hybrid composition.

### [1.22.1](https://github.com/flutter/flutter/pull/67552) (October 8, 2020)
This hotfix release addresses the following issues:
  - [flutter/66940](https://github.com/flutter/flutter/issues/66940) - autovalidate property inadvertently removed.
  - [flutter/66962](https://github.com/flutter/flutter/issues/66962) - The new --analyze-size flag crashes when used with --split-debug-info
  - [flutter/66908](https://github.com/flutter/flutter/issues/66908) - Flutter Activity causing exceptions in some Android versions.
  - [flutter/66647](https://github.com/flutter/flutter/issues/66647) - Layout modifications performed by background threads causes exceptions on IOS14.

### 1.22.0 (October 1, 2020)
Initial stable release.

## Flutter 1.20 Changes
### [1.20.4](https://github.com/flutter/flutter/pull/65787) (September 15, 2020)
This hotfix release addresses the following issues:
  - [flutter/64045](https://github.com/flutter/flutter/issues/64045) - Cannot deploy to physical device running iOS 14

### [1.20.3](https://github.com/flutter/flutter/pull/64984) (September 2, 2020)
This hotfix release addresses the following issues:
  - [flutter/63876](https://github.com/flutter/flutter/issues/63876) - Performance regression for Image animation.
  - [flutter/64228](https://github.com/flutter/flutter/issues/64228) - WebView may freeze in release mode on iOS.
  - [flutter/64414](https://github.com/flutter/flutter/issues/64414) - Task switching may freeze on some Android versions.
  - [flutter/63560](https://github.com/flutter/flutter/issues/63560) - Building AARs may cause a stack overflow.
  - [flutter/57210](https://github.com/flutter/flutter/issues/57210) - Certain assets may cause issues with iOS builds.
  - [flutter/63590](https://github.com/flutter/flutter/issues/63590) - Passing null values from functions run via Isolates throws an exception.
  - [flutter/63427](https://github.com/flutter/flutter/issues/63427) - Wrong hour/minute order in timePicker in RTL mode.

### [1.20.2](https://github.com/flutter/flutter/pull/63591) (August 13, 2020)
This hotfix release addresses the following issues:
  - [flutter/63038](https://github.com/flutter/flutter/issues/63038) - Crash due to serialization of generic DartType (UnknownType)
  - [flutter/46167](https://github.com/flutter/flutter/issues/46167) - iOS platform view cancels gesture while a new clip layer is added during the gesture
  - [flutter/62198](https://github.com/flutter/flutter/issues/62198) - SliverList throws Exception when first item is SizedBox.shrink()
  - [flutter/59029](https://github.com/flutter/flutter/issues/59029) - build ios --release can crash with ArgumentError: Invalid argument(s)
  - [flutter/62775](https://github.com/flutter/flutter/issues/62775) - TimePicker is not correct in RTL (right-to-left) languages
  - [flutter/55535](https://github.com/flutter/flutter/issues/55535) - New DatePicker widget is not fully  localized
  - [flutter/63373](https://github.com/flutter/flutter/issues/63373) - Double date separators appearing in DatePicker, preventing date selection
  - [flutter/63176](https://github.com/flutter/flutter/issues/63176) -  App.framework path in Podfile incorrect

### [1.20.1](https://github.com/flutter/flutter/pull/62990) (August 6, 2020)
This hotfix release addresses the following issues:
  - [flutter/60215](https://github.com/flutter/flutter/issues/60215) - Creating an Android-only plug-in creates a no-op iOS folder.

### 1.20.0 (August 5, 2020)
Initial stable release.

## Flutter 1.17 Changes
### [1.17.5](https://github.com/flutter/flutter/pull/60611) (June 30, 2020)
This hotfix release addresses the following issues:
  - [flutter-intellij/4642]https://github.com/flutter/flutter-intellij/issues/4642  - Intellij/Android Studio plugins fail to show connected Android devices.

### [1.17.4](https://github.com/flutter/flutter/pull/59695) (June 18, 2020)
This hotfix release addresses the following issues:
  - [flutter/56826](https://github.com/flutter/flutter/issues/56826)  - xcdevice polling may use all free hard drive space

### [1.17.3](https://github.com/flutter/flutter/pull/58646) (June 4, 2020)
This hotfix release addresses the following issues:
 - [flutter/54420](https://github.com/flutter/flutter/issues/54420)  - Exhausted heap space can cause machine to freeze

### [1.17.2](https://github.com/flutter/flutter/pull/58050) (May 28, 2020)
This hotfix release addresses the following issues:
 - [flutter/57326](https://github.com/flutter/flutter/issues/57326)  - CupertinoSegmentedControl does not always respond to selections
 - [flutter/56898](https://github.com/flutter/flutter/issues/56898) - DropdownButtonFormField is not re-rendered after value is changed programmatically
 - [flutter/56853](https://github.com/flutter/flutter/issues/56853) - Incorrect git error may be presented when flutter upgrade fails
 - [flutter/55552](https://github.com/flutter/flutter/issues/55552) - Hot reload may fail after a hot restart
 - [flutter/56507](https://github.com/flutter/flutter/issues/56507) - iOS builds may fail with “The path does not exist” error message

### [1.17.1](https://github.com/flutter/flutter/pull/57052) (May 13, 2020)
This hotfix release addresses the following issues:
 - [flutter/26345](https://github.com/flutter/flutter/issues/26345) - Updating `AndroidView` layer tree causes crash on Xiaomi and Moto devices
 - [flutter/56567](https://github.com/flutter/flutter/issues/56567) - Xcode legacy build system causes build failures on iOS
 - [flutter/56473](https://github.com/flutter/flutter/issues/56473) - Build `--tree-shake-icons` build option crashes computer
 - [flutter/56688](https://github.com/flutter/flutter/issues/56688) - Regression in `Navigator.pushAndRemoveUntil`
 - [flutter/56479](https://github.com/flutter/flutter/issues/56479) - Crash while getting static type context for signature shaking

### 1.17.0 (May 5, 2020)
Initial stable release.

## Flutter 1.12 Changes
### Hotfix.9 (April 1, 2020)
This hotfix release addresses the following issues:
 - [flutter/47819](https://github.com/flutter/flutter/issues/47819) - Crashes on ARMv8 Android devices
 - [flutter/49185](https://github.com/flutter/flutter/issues/49185) - Issues using Flutter 1.12 with Linux 5.5
 - [flutter/51712](https://github.com/flutter/flutter/issues/51712) - fixes for licensing from Android sdkmanager tool not being found

### [Hotfix.8](https://github.com/flutter/flutter/pull/50591) (February 11, 2020)
This hotfix release addresses the following issues:
 - [flutter/50066](https://github.com/flutter/flutter/issues/50066) - binaries unsigned in last hotfix
 - [flutter/49787](https://github.com/flutter/flutter/issues/49787) - in a previous hotfix, we inadvertently broke Xcode 10 support. Reverting this change would have caused other problems (and users would still have to upgrade their Xcode with the next stable release), we decided to increase our minimum supported Xcode version. Please see the linked issue for more context on this decision.
 - [flutter/45732](https://github.com/flutter/flutter/issues/45732) - Android log reader fix
 - [flutter/47609](https://github.com/flutter/flutter/issues/47609) - Android log reader fix

### [Hotfix.7](https://github.com/flutter/flutter/pull/49437) (January 26, 2020)
This hotfix release addresses the following issues:
- [flutter/47164](https://github.com/flutter/flutter/issues/47164) - blackscreen / crash on certain Huawei devices
- [flutter/47804](https://github.com/flutter/flutter/issues/47804) - Flutter engine crashes on some Android devices due to "Failed to setup Skia Gr context"
- [flutter/46172](https://github.com/flutter/flutter/issues/46172) - reportFullyDrawn causes crash on Android KitKat

### Hotfix.5 (December 11, 2019)
Initial stable release.
