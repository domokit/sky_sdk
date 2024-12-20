// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext(
    'gradle prints warning when Flutter\'s Gradle plugins are applied using deprecated "apply plugin" way',
    () async {
      // Create a new flutter project.
      ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'create',
        tempDir.path,
        '--project-name=testapp',
      ], workingDirectory: tempDir.path);
      expect(result.exitCode, 0);
      // Ensure that gradle files exists from templates.
      result = await processManager.run(<String>[
        flutterBin,
        'build',
        'apk',
        '--config-only',
      ], workingDirectory: tempDir.path);
      expect(result.exitCode, 0);

      // Change build files to use deprecated "apply plugin:" way.
      // Contents are taken from https://github.com/flutter/flutter/issues/135392 (for Flutter 3.10)
      final File settings = tempDir.childDirectory('android').childFile('settings.gradle');
      final File buildGradle = tempDir.childDirectory('android').childFile('build.gradle');
      final File appBuildGradle = tempDir
          .childDirectory('android')
          .childDirectory('app')
          .childFile('build.gradle');
      settings.writeAsStringSync(r'''
include ':app'

def localPropertiesFile = new File(rootProject.projectDir, "local.properties")
def properties = new Properties()

assert localPropertiesFile.exists()
localPropertiesFile.withReader("UTF-8") { reader -> properties.load(reader) }

def flutterSdkPath = properties.getProperty("flutter.sdk")
assert flutterSdkPath != null, "flutter.sdk not set in local.properties"
apply from: "$flutterSdkPath/packages/flutter_tools/gradle/app_plugin_loader.gradle"
''');
      buildGradle.writeAsStringSync(r'''
buildscript {
    ext.kotlin_version = '1.8.22'
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath 'com.android.tools.build:gradle:8.1.0'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.layout.buildDirectory.value(rootProject.layout.buildDirectory.dir("../../build").get())
subprojects {
    project.layout.buildDirectory.value(rootProject.layout.buildDirectory.dir(project.name).get())
}
subprojects {
    project.evaluationDependsOn(':app')
}

tasks.register("clean", Delete) {
    delete rootProject.layout.buildDirectory
}
''');
      appBuildGradle.writeAsStringSync(r'''
def localProperties = new Properties()
def localPropertiesFile = rootProject.file('local.properties')
if (localPropertiesFile.exists()) {
    localPropertiesFile.withReader('UTF-8') { reader ->
        localProperties.load(reader)
    }
}

def flutterRoot = localProperties.getProperty('flutter.sdk')
if (flutterRoot == null) {
    throw new GradleException("Flutter SDK not found. Define location with flutter.sdk in the local.properties file.")
}

def flutterVersionCode = localProperties.getProperty('flutter.versionCode')
if (flutterVersionCode == null) {
    flutterVersionCode = '1'
}

def flutterVersionName = localProperties.getProperty('flutter.versionName')
if (flutterVersionName == null) {
    flutterVersionName = '1.0'
}

apply plugin: 'com.android.application'
apply plugin: 'kotlin-android'
apply from: "$flutterRoot/packages/flutter_tools/gradle/flutter.gradle"

android {
    compileSdkVersion flutter.compileSdkVersion
    ndkVersion flutter.ndkVersion
    namespace "com.example.testapp"

    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = '1.8'
    }

    sourceSets {
        main.java.srcDirs += 'src/main/kotlin'
    }

    defaultConfig {
        // TODO: Specify your own unique Application ID (https://developer.android.com/studio/build/application-id.html).
        applicationId "com.example.testapp"
        // You can update the following values to match your application needs.
        // For more information, see: https://flutter.dev/to/review-gradle-config.
        minSdkVersion flutter.minSdkVersion
        targetSdkVersion flutter.targetSdkVersion
        versionCode flutterVersionCode.toInteger()
        versionName flutterVersionName
        testInstrumentationRunner "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            // TODO: Add your own signing config for the release build.
            // Signing with the debug keys for now, so `flutter run --release` works.
            signingConfig signingConfigs.debug
        }
    }
}

flutter {
    source '../..'
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"
}
''');

      result = await processManager.run(<String>[
        flutterBin,
        'build',
        'apk',
        '--debug',
      ], workingDirectory: tempDir.path);
      expect(result.exitCode, 0);
      // Verify that stderr output contains deprecation warnings.
      final List<String> actualLines = LineSplitter.split(result.stderr.toString()).toList();
      expect(
        actualLines.any(
          (String msg) =>
              msg.contains("You are applying Flutter's main Gradle plugin imperatively"),
        ),
        isTrue,
      );
      expect(
        actualLines.any(
          (String msg) => msg.contains(
            "You are applying Flutter's app_plugin_loader Gradle plugin imperatively",
          ),
        ),
        isTrue,
      );
    },
  );
}
