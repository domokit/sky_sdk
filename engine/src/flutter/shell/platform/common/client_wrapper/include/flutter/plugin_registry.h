// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRY_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRY_H_

#include <string>

#include <flutter_plugin_registrar.h>

namespace flutter {

// Vends PluginRegistrars for named plugins.
//
// Plugins are identified by unique string keys, typically the name of the
// plugin's main class.
class PluginRegistry {
 public:
  PluginRegistry() = default;
  virtual ~PluginRegistry() = default;

  // Prevent copying.
  PluginRegistry(PluginRegistry const&) = delete;
  PluginRegistry& operator=(PluginRegistry const&) = delete;

  // Returns the FlutterDesktopPluginRegistrarRef to register a plugin with the
  // given name.
  //
  // The name must be unique across the application.
  virtual FlutterDesktopPluginRegistrarRef GetRegistrarForPlugin(
      const std::string& plugin_name) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRY_H_
