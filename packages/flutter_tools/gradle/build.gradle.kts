// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

plugins {
    `java-gradle-plugin`
    groovy
}

group = "dev.flutter.plugin"
version = "1.0.0"

gradlePlugin {
    plugins {
        // The "flutterPlugin" name isn't used anywhere.
        create("flutterPlugin") {
            id = "dev.flutter.flutter-gradle-plugin"
            implementationClass = "FlutterPlugin"
        }
        // The "flutterAppPluginLoaderPlugin" name isn't used anywhere.
        create("flutterAppPluginLoaderPlugin") {
            id = "dev.flutter.flutter-plugin-loader"
            implementationClass = "FlutterAppPluginLoaderPlugin"
        }
    }
}

dependencies {
    // When bumping, also update:
    //  * ndkVersion in FlutterExtension in packages/flutter_tools/gradle/src/main/groovy/flutter.groovy
    //  * AGP version in the buildscript block in packages/flutter_tools/gradle/src/main/kotlin/dependency_version_checker.gradle.kts
    //  * AGP version constants in packages/flutter_tools/lib/src/android/gradle_utils.dart
    compileOnly("com.android.tools.build:gradle:7.3.0")
}
