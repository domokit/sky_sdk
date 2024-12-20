// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>

#include "flutter/shell/platform/glfw/client_wrapper/include/flutter/plugin_registrar_glfw.h"
#include "flutter/shell/platform/glfw/client_wrapper/testing/stub_flutter_glfw_api.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// A test plugin that tries to access registrar state during destruction and
// reports it out via a flag provided at construction.
class TestPlugin : public Plugin {
 public:
  // registrar_valid_at_destruction will be set at destruction to indicate
  // whether or not |registrar->window()| was non-null.
  TestPlugin(PluginRegistrarGlfw* registrar,
             bool* registrar_valid_at_destruction)
      : registrar_(registrar),
        registrar_valid_at_destruction_(registrar_valid_at_destruction) {}
  virtual ~TestPlugin() {
    *registrar_valid_at_destruction_ = registrar_->window() != nullptr;
  }

 private:
  PluginRegistrarGlfw* registrar_;
  bool* registrar_valid_at_destruction_;
};

}  // namespace

TEST(PluginRegistrarGlfwTest, GetView) {
  testing::ScopedStubFlutterGlfwApi scoped_api_stub(
      std::make_unique<testing::StubFlutterGlfwApi>());
  PluginRegistrarGlfw registrar(
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1));
  EXPECT_NE(registrar.window(), nullptr);
}

// Tests that the registrar runs plugin destructors before its own teardown.
TEST(PluginRegistrarGlfwTest, PluginDestroyedBeforeRegistrar) {
  auto dummy_registrar_handle =
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1);
  bool registrar_valid_at_destruction = false;
  {
    PluginRegistrarGlfw registrar(dummy_registrar_handle);

    auto plugin = std::make_unique<TestPlugin>(&registrar,
                                               &registrar_valid_at_destruction);
    registrar.AddPlugin(std::move(plugin));
  }
  EXPECT_TRUE(registrar_valid_at_destruction);
}

}  // namespace flutter
