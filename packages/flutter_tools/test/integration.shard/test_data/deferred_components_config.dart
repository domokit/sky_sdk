// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import '../test_utils.dart';

abstract class DeferredComponentsConfig {
  String get deferredLibrary;
  String? get deferredComponentsGolden;
  String get androidSettings;
  String get androidBuild;
  String get androidLocalProperties;
  String get androidGradleProperties;
  String get androidKeyProperties;
  List<int> get androidKey;
  String get appBuild;
  String get appManifest;
  String get appStrings;
  String get appStyles;
  String get appLaunchBackground;
  String get asset1;
  String get asset2;
  List<DeferredComponentModule> get deferredComponents;

  void setUpIn(Directory dir) {
    writeFile(fileSystem.path.join(dir.path, 'lib', 'deferred_library.dart'), deferredLibrary);
    final String? golden = deferredComponentsGolden;
    if (golden != null) {
      writeFile(fileSystem.path.join(dir.path, 'deferred_components_loading_units.yaml'), golden);
    }
    writeFile(fileSystem.path.join(dir.path, 'android', 'settings.gradle'), androidSettings);
    writeFile(fileSystem.path.join(dir.path, 'android', 'build.gradle'), androidBuild);
    writeFile(
      fileSystem.path.join(dir.path, 'android', 'local.properties'),
      androidLocalProperties,
    );
    writeFile(
      fileSystem.path.join(dir.path, 'android', 'gradle.properties'),
      androidGradleProperties,
    );
    writeFile(fileSystem.path.join(dir.path, 'android', 'key.properties'), androidKeyProperties);
    writeBytesFile(fileSystem.path.join(dir.path, 'android', 'app', 'key.jks'), androidKey);
    writeFile(fileSystem.path.join(dir.path, 'android', 'app', 'build.gradle'), appBuild);
    writeFile(
      fileSystem.path.join(dir.path, 'android', 'app', 'src', 'main', 'AndroidManifest.xml'),
      appManifest,
    );
    writeFile(
      fileSystem.path.join(
        dir.path,
        'android',
        'app',
        'src',
        'main',
        'res',
        'values',
        'strings.xml',
      ),
      appStrings,
    );
    writeFile(
      fileSystem.path.join(
        dir.path,
        'android',
        'app',
        'src',
        'main',
        'res',
        'values',
        'styles.xml',
      ),
      appStyles,
    );
    writeFile(
      fileSystem.path.join(
        dir.path,
        'android',
        'app',
        'src',
        'main',
        'res',
        'drawable',
        'launch_background.xml',
      ),
      appLaunchBackground,
    );
    writeFile(fileSystem.path.join(dir.path, 'test_assets/asset1.txt'), asset1);
    writeFile(fileSystem.path.join(dir.path, 'test_assets/asset2.txt'), asset2);
    for (final DeferredComponentModule component in deferredComponents) {
      component.setUpIn(dir);
    }
  }
}

class DeferredComponentModule {
  DeferredComponentModule(this.name);

  String name;

  void setUpIn(Directory dir) {
    writeFile(
      fileSystem.path.join(dir.path, 'android', name, 'build.gradle'),
      r'''
    def localProperties = new Properties()
    def localPropertiesFile = rootProject.file('local.properties')
    if (localPropertiesFile.exists()) {
        localPropertiesFile.withReader('UTF-8') { reader ->
            localProperties.load(reader)
        }
    }

    def flutterVersionCode = localProperties.getProperty('flutter.versionCode')
    if (flutterVersionCode == null) {
        flutterVersionCode = '1'
    }

    def flutterVersionName = localProperties.getProperty('flutter.versionName')
    if (flutterVersionName == null) {
        flutterVersionName = '1.0'
    }

    apply plugin: "com.android.dynamic-feature"

    android {
        namespace = "com.example.''' +
          name +
          r'''
"
        compileSdk 35

        sourceSets {
            applicationVariants.all { variant ->
                main.assets.srcDirs += "${project.layout.buildDirectory.get()}/intermediates/flutter/${variant.name}/deferred_assets"
                main.jniLibs.srcDirs += "${project.layout.buildDirectory.get()}/intermediates/flutter/${variant.name}/deferred_libs"
            }
        }

        defaultConfig {
            minSdkVersion 21
            targetSdkVersion 35
            versionCode flutterVersionCode.toInteger()
            versionName flutterVersionName
        }
        compileOptions {
            sourceCompatibility 1.8
            targetCompatibility 1.8
        }
    }

    dependencies {
        implementation project(":app")
    }
    ''',
    );

    writeFile(
      fileSystem.path.join(dir.path, 'android', name, 'src', 'main', 'AndroidManifest.xml'),
      '''
    <manifest xmlns:android="http://schemas.android.com/apk/res/android"
        xmlns:dist="http://schemas.android.com/apk/distribution">

        <dist:module
            dist:instant="false"
            dist:title="@string/component1Name">
            <dist:delivery>
                <dist:on-demand />
            </dist:delivery>
            <dist:fusing dist:include="true" />
        </dist:module>
    </manifest>
    ''',
    );
  }
}
