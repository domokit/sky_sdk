// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(54566): Remove this file when issue is resolved.

import 'deferred_components_config.dart';
import 'plugin_project.dart';

/// Project to test the deprecated `settings.gradle` (PluginEach) that apps were
/// created with until Flutter v1.22.0.
/// It uses the `.flutter-plugins` file to load EACH plugin.
class PluginEachSettingsGradleProject extends PluginProject {
  @override
  DeferredComponentsConfig get deferredComponents =>
      PluginEachSettingsGradleDeferredComponentsConfig();
}

class PluginEachSettingsGradleDeferredComponentsConfig extends PluginDeferredComponentsConfig {
  @override
  String get androidSettings => r'''
include ':app'
def flutterProjectRoot = rootProject.projectDir.parentFile.toPath()
def plugins = new Properties()
def pluginsFile = new File(flutterProjectRoot.toFile(), '.flutter-plugins')
if (pluginsFile.exists()) {
    pluginsFile.withReader('UTF-8') { reader -> plugins.load(reader) }
}
plugins.each { name, path ->
    def pluginDirectory = flutterProjectRoot.resolve(path).resolve('android').toFile()
    include ":$name"
    project(":$name").projectDir = pluginDirectory
}
  ''';
}

/// Project to test the deprecated `settings.gradle` (PluginEach) that apps were
/// created with until Flutter v1.22.0.
/// It uses the `.flutter-plugins` file to get EACH plugin.
/// It is compromised by removing the 'include' statement of the plugins.
class PluginCompromisedEachSettingsGradleProject extends PluginProject {
  @override
  DeferredComponentsConfig get deferredComponents =>
      PluginCompromisedEachSettingsGradleDeferredComponentsConfig();
}

class PluginCompromisedEachSettingsGradleDeferredComponentsConfig
    extends PluginDeferredComponentsConfig {
  @override
  String get androidSettings => r'''
include ':app'
def flutterProjectRoot = rootProject.projectDir.parentFile.toPath()
def plugins = new Properties()
def pluginsFile = new File(flutterProjectRoot.toFile(), '.flutter-plugins')
if (pluginsFile.exists()) {
    pluginsFile.withReader('UTF-8') { reader -> plugins.load(reader) }
}
  ''';
}
