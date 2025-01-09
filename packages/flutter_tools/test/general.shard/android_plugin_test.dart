// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/platform_plugins.dart';

import '../src/common.dart';

void main() {
  testWithoutContext(
    'AndroidPlugin throws tool exit if the plugin main class can not be found',
    () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final AndroidPlugin androidPlugin = AndroidPlugin(
        name: 'pluginA',
        package: 'com.company',
        pluginClass: 'PluginA',
        pluginPath: '.pub_cache/plugin_a',
        fileSystem: fileSystem,
      );

      expect(
        () => androidPlugin.toMap(),
        throwsToolExit(
          message:
              "The plugin `pluginA` doesn't have a main class defined in "
              '.pub_cache/plugin_a/android/src/main/java/com/company/PluginA.java '
              'or .pub_cache/plugin_a/android/src/main/kotlin/com/company/PluginA.kt',
        ),
      );
    },
  );

  testWithoutContext('AndroidPlugin does not validate the main class for Dart-only plugins', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      dartPluginClass: 'PluginA',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'dartPluginClass': 'PluginA',
      'supportsEmbeddingV1': false,
      'supportsEmbeddingV2': false,
    });
  });

  testWithoutContext('AndroidPlugin does not validate the main class for default_package', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      defaultPackage: 'plugin_a_android',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'default_package': 'plugin_a_android',
      'supportsEmbeddingV1': false,
      'supportsEmbeddingV2': false,
    });
  });

  testWithoutContext('AndroidPlugin parses embedding version 2 from the Java search path', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      package: 'com.company',
      pluginClass: 'PluginA',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    fileSystem.file('.pub_cache/plugin_a/android/src/main/java/com/company/PluginA.java')
      ..createSync(recursive: true)
      ..writeAsStringSync('io.flutter.embedding.engine.plugins.FlutterPlugin');

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'package': 'com.company',
      'class': 'PluginA',
      'supportsEmbeddingV1': false,
      'supportsEmbeddingV2': true,
    });
  });

  testWithoutContext('AndroidPlugin parses embedding version 1 from the Java search path', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      package: 'com.company',
      pluginClass: 'PluginA',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    fileSystem.file('.pub_cache/plugin_a/android/src/main/java/com/company/PluginA.java')
      ..createSync(recursive: true)
      ..writeAsStringSync('some.other.string');

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'package': 'com.company',
      'class': 'PluginA',
      'supportsEmbeddingV1': true,
      'supportsEmbeddingV2': false,
    });
  });

  testWithoutContext('AndroidPlugin parses embedding version 2 from the Kotlin search path', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      package: 'com.company',
      pluginClass: 'PluginA',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    fileSystem.file('.pub_cache/plugin_a/android/src/main/kotlin/com/company/PluginA.kt')
      ..createSync(recursive: true)
      ..writeAsStringSync('io.flutter.embedding.engine.plugins.FlutterPlugin');

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'package': 'com.company',
      'class': 'PluginA',
      'supportsEmbeddingV1': false,
      'supportsEmbeddingV2': true,
    });
  });

  testWithoutContext('AndroidPlugin parses embedding version 1 from the Kotlin search path', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final AndroidPlugin androidPlugin = AndroidPlugin(
      name: 'pluginA',
      package: 'com.company',
      pluginClass: 'PluginA',
      pluginPath: '.pub_cache/plugin_a',
      fileSystem: fileSystem,
    );

    fileSystem.file('.pub_cache/plugin_a/android/src/main/kotlin/com/company/PluginA.kt')
      ..createSync(recursive: true)
      ..writeAsStringSync('some.other.string');

    expect(androidPlugin.toMap(), <String, Object>{
      'name': 'pluginA',
      'package': 'com.company',
      'class': 'PluginA',
      'supportsEmbeddingV1': true,
      'supportsEmbeddingV2': false,
    });
  });
}
