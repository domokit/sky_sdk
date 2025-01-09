import org.jetbrains.kotlin.gradle.dsl.JvmTarget


// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

plugins {
    `java-gradle-plugin`
    groovy
    `kotlin-dsl`
    id("org.jetbrains.kotlin.plugin.serialization") version "1.8.0"
}

group = "dev.flutter.plugin"
version = "1.0.0"

// Optional: enable stricter validation, to ensure Gradle configuration is correct
tasks.validatePlugins {
    enableStricterValidation.set(true)
}

// We need to compile Kotlin first so we can call it from Groovy. See https://stackoverflow.com/q/36214437/7009800
tasks.withType<GroovyCompile> {
    dependsOn(tasks.compileKotlin)
    classpath += files(tasks.compileKotlin.get().destinationDirectory)
}

tasks.classes {
    dependsOn(tasks.compileGroovy)
}

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

tasks.withType<JavaCompile> {
    options.release.set(11)
}

kotlin {
    compilerOptions {
        jvmTarget.set(JvmTarget.JVM_11)
    }
}


dependencies {
    // When bumping, also update:
    //  * ndkVersion in FlutterExtension in packages/flutter_tools/gradle/src/main/groovy/flutter.groovy
    //  * AGP version in the buildscript block in packages/flutter_tools/gradle/src/main/kotlin/kotlin_scripts/dependency_version_checker.gradle.kts
    //  * AGP version constants in packages/flutter_tools/lib/src/android/gradle_utils.dart
    compileOnly("com.android.tools.build:gradle:7.3.0")

    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.7.3")
}
