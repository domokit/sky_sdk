// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'basic_project.dart';
import 'deferred_components_config.dart';
import 'deferred_components_project.dart';

/// Project which can load native plugins
class PluginProject extends BasicProject {
  @override
  final DeferredComponentsConfig? deferredComponents = PluginDeferredComponentsConfig();
}

class PluginDeferredComponentsConfig extends BasicDeferredComponentsConfig {
  @override
  String get androidBuild => r'''
buildscript {
    ext.kotlin_version = '2.1.0'
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath 'com.android.tools.build:gradle:8.1.0'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
    configurations.classpath {
        resolutionStrategy.activateDependencyLocking()
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
    dependencyLocking {
        ignoredDependencies.add('io.flutter:*')
        lockFile = file("${rootProject.projectDir}/project-${project.name}.lockfile")
        if (!project.hasProperty('local-engine-repo')) {
          lockAllConfigurations()
        }
    }
}
tasks.register("clean", Delete) {
    delete rootProject.layout.buildDirectory
}
''';

  @override
  String get androidSettings => r'''
include ':app'

def localPropertiesFile = new File(rootProject.projectDir, "local.properties")
def properties = new Properties()

assert localPropertiesFile.exists()
localPropertiesFile.withReader("UTF-8") { reader -> properties.load(reader) }

def flutterSdkPath = properties.getProperty("flutter.sdk")
assert flutterSdkPath != null, "flutter.sdk not set in local.properties"
apply from: "$flutterSdkPath/packages/flutter_tools/gradle/app_plugin_loader.gradle"
''';

  @override
  String get appManifest => r'''
<manifest xmlns:android="http://schemas.android.com/apk/res/android">
    <uses-permission android:name="android.permission.INTERNET"/>
    <application
        android:name="${applicationName}"
        android:label="flavors">
        <activity
            android:name=".MainActivity"
            android:exported="true"
            android:launchMode="singleTop"
            android:configChanges="orientation|keyboardHidden|keyboard|screenSize|locale|layoutDirection|fontScale|screenLayout|density|uiMode"
            android:hardwareAccelerated="true"
            android:windowSoftInputMode="adjustResize">
            <intent-filter>
                <action android:name="android.intent.action.MAIN"/>
                <category android:name="android.intent.category.LAUNCHER"/>
            </intent-filter>
        </activity>
        <meta-data
            android:name="flutterEmbedding"
            android:value="2" />
    </application>
</manifest>
''';
}
