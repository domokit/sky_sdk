// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

group = "dev.flutter.plugins.integration_test"
version = "1.0-SNAPSHOT"

buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath("com.android.tools.build:gradle:8.1.0")
    }
}

plugins {
    id("com.android.library")
}

rootProject.allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

// project.getTasks().withType(JavaCompile){
//     options.compilerArgs << "-Xlint:all" << "-Werror"
// }

android {
    namespace = "dev.flutter.integration_test"
    compileSdk = flutter.compileSdkVersion
    ndkVersion = flutter.ndkVersion

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    defaultConfig {
        minSdk = flutter.minSdkVersion
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("lib-proguard-rules.txt")
    }

    dependencies {
        // TODO(egarciad): These dependencies should not be added to release builds.
        // https://github.com/flutter/flutter/issues/56591
        testImplementation("junit:junit:4.13.2")
        testImplementation("org.mockito:mockito-inline:5.1.0")

        api("androidx.test:runner:1.2+")
        api("androidx.test:rules:1.2+")
        api("androidx.test.espresso:espresso-core:3.2+")

        implementation("com.google.guava:guava:28.1-android")
    }
}
