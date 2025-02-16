## The Flutter Gradle Plugin

This directory contains Gradle code used to by the Flutter tool to build Flutter apps for Android,
primarily the Flutter Gradle Plugin (FGP) - a Gradle plugin built on top of Gradle and the Android 
Gradle Plugin (AGP).

### Editing in Android Studio

To get code completion in Android Studio, you must open a new Android Studio window with this 
directory as the root. Code completion will not work when navigating to files 
in an Android Studio window opened at the root of the entire Flutter repo.

### Contributing

The Flutter Gradle Plugin is [currently being re-written](https://github.com/flutter/flutter/issues/121541) from Groovy to Kotlin 
(Kotlin source specifically, i.e. not `.kts`). As such, outside of critical bug fixes, 
new contributions will only be accepted in `src/main/kotlin` (and `src/test/kotlin`).

### Testing

To run the tests from the CLI, you first need to download the Gradle wrapper.
1. Ensure you have run gclient sync recently (i.e., from the root of your framework checkout, run `gclient sync -D`).
2. From this directory, run `../../../engine/src/flutter/third_party/gradle/bin/gradle wrapper`.

Tests can be run in Android Studio, or directly with Gradle: `./gradlew test` 
(note that this directory does not contain a version controlled Gradle file. You can find one in 
the engines `third_party` directory at 
`<flutter_root>/engine/src/flutter/third_party/gradle/bin/gradle`).

Alternatively, you can run all the tests in one file by passing in the fully qualified class name, 
e.g. `./gradlew test --tests com.flutter.gradle.BaseApplicationNameHandlerTest`, or one test in 
one file by passing in the fully qualified class name followed by the method name, 
e.g `./gradlew test --tests "com.flutter.gradle.BaseApplicationNameHandlerTest.setBaseName respects Flutter tool property"`.

Sometimes changing a test name and then running it will cause an IDE error. To get Android Studio back
to a good state on Mac, run `Help > "Repair IDE"`, and then in the popup window `"Rescan project indexes > Everything works now."`

To add a new test, add a class under `src/test/kotlin`, with methods annotated with `@Test`.
These tests will get automatically run on CI by `packages/flutter_tools/test/integration.shard/android_run_flutter_gradle_plugin_tests_test.dart`.
