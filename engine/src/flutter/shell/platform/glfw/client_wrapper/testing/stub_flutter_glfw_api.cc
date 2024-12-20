// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/client_wrapper/testing/stub_flutter_glfw_api.h"

static flutter::testing::StubFlutterGlfwApi* s_stub_implementation;

namespace flutter {
namespace testing {

// static
void StubFlutterGlfwApi::SetTestStub(StubFlutterGlfwApi* stub) {
  s_stub_implementation = stub;
}

// static
StubFlutterGlfwApi* StubFlutterGlfwApi::GetTestStub() {
  return s_stub_implementation;
}

ScopedStubFlutterGlfwApi::ScopedStubFlutterGlfwApi(
    std::unique_ptr<StubFlutterGlfwApi> stub)
    : stub_(std::move(stub)) {
  previous_stub_ = StubFlutterGlfwApi::GetTestStub();
  StubFlutterGlfwApi::SetTestStub(stub_.get());
}

ScopedStubFlutterGlfwApi::~ScopedStubFlutterGlfwApi() {
  StubFlutterGlfwApi::SetTestStub(previous_stub_);
}

}  // namespace testing
}  // namespace flutter

// Forwarding dummy implementations of the C API.

bool FlutterDesktopInit() {
  if (s_stub_implementation) {
    s_stub_implementation->Init();
  }
  return true;
}

void FlutterDesktopTerminate() {
  if (s_stub_implementation) {
    s_stub_implementation->Terminate();
  }
}

FlutterDesktopWindowControllerRef FlutterDesktopCreateWindow(
    const FlutterDesktopWindowProperties& window_properties,
    const FlutterDesktopEngineProperties& engine_properties) {
  if (s_stub_implementation) {
    return s_stub_implementation->CreateWindow(window_properties,
                                               engine_properties);
  }
  return nullptr;
}

void FlutterDesktopDestroyWindow(FlutterDesktopWindowControllerRef controller) {
  if (s_stub_implementation) {
    s_stub_implementation->DestroyWindow();
  }
}

void FlutterDesktopWindowSetHoverEnabled(FlutterDesktopWindowRef flutter_window,
                                         bool enabled) {
  if (s_stub_implementation) {
    s_stub_implementation->SetHoverEnabled(enabled);
  }
}

void FlutterDesktopWindowSetTitle(FlutterDesktopWindowRef flutter_window,
                                  const char* title) {
  if (s_stub_implementation) {
    s_stub_implementation->SetWindowTitle(title);
  }
}

void FlutterDesktopWindowSetIcon(FlutterDesktopWindowRef flutter_window,
                                 uint8_t* pixel_data,
                                 int width,
                                 int height) {
  if (s_stub_implementation) {
    s_stub_implementation->SetWindowIcon(pixel_data, width, height);
  }
}

void FlutterDesktopWindowGetFrame(FlutterDesktopWindowRef flutter_window,
                                  int* x,
                                  int* y,
                                  int* width,
                                  int* height) {
  if (s_stub_implementation) {
    s_stub_implementation->GetWindowFrame(x, y, width, height);
  }
}

void FlutterDesktopWindowSetFrame(FlutterDesktopWindowRef flutter_window,
                                  int x,
                                  int y,
                                  int width,
                                  int height) {
  if (s_stub_implementation) {
    s_stub_implementation->SetWindowFrame(x, y, width, height);
  }
}

void FlutterDesktopWindowSetSizeLimits(FlutterDesktopWindowRef flutter_window,
                                       FlutterDesktopSize minimum_size,
                                       FlutterDesktopSize maximum_size) {
  if (s_stub_implementation) {
    s_stub_implementation->SetSizeLimits(minimum_size, maximum_size);
  }
}

double FlutterDesktopWindowGetScaleFactor(
    FlutterDesktopWindowRef flutter_window) {
  if (s_stub_implementation) {
    return s_stub_implementation->GetWindowScaleFactor();
  }
  return 1.0;
}

void FlutterDesktopWindowSetPixelRatioOverride(
    FlutterDesktopWindowRef flutter_window,
    double pixel_ratio) {
  if (s_stub_implementation) {
    return s_stub_implementation->SetPixelRatioOverride(pixel_ratio);
  }
}

bool FlutterDesktopRunWindowEventLoopWithTimeout(
    FlutterDesktopWindowControllerRef controller,
    uint32_t millisecond_timeout) {
  if (s_stub_implementation) {
    return s_stub_implementation->RunWindowEventLoopWithTimeout(
        millisecond_timeout);
  }
  return true;
}

FlutterDesktopEngineRef FlutterDesktopRunEngine(
    const FlutterDesktopEngineProperties& properties) {
  if (s_stub_implementation) {
    return s_stub_implementation->RunEngine(properties);
  }
  return nullptr;
}

void FlutterDesktopRunEngineEventLoopWithTimeout(
    FlutterDesktopEngineRef engine,
    uint32_t timeout_milliseconds) {
  if (s_stub_implementation) {
    s_stub_implementation->RunEngineEventLoopWithTimeout(timeout_milliseconds);
  }
}

bool FlutterDesktopShutDownEngine(FlutterDesktopEngineRef engine_ref) {
  if (s_stub_implementation) {
    return s_stub_implementation->ShutDownEngine();
  }
  return true;
}

FlutterDesktopWindowRef FlutterDesktopGetWindow(
    FlutterDesktopWindowControllerRef controller) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopWindowRef>(1);
}

FlutterDesktopEngineRef FlutterDesktopGetEngine(
    FlutterDesktopWindowControllerRef controller) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopEngineRef>(3);
}

FlutterDesktopPluginRegistrarRef FlutterDesktopGetPluginRegistrar(
    FlutterDesktopEngineRef engine,
    const char* plugin_name) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopPluginRegistrarRef>(2);
}

FlutterDesktopWindowRef FlutterDesktopPluginRegistrarGetWindow(
    FlutterDesktopPluginRegistrarRef registrar) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopWindowRef>(3);
}

void FlutterDesktopPluginRegistrarEnableInputBlocking(
    FlutterDesktopPluginRegistrarRef registrar,
    const char* channel) {
  if (s_stub_implementation) {
    s_stub_implementation->PluginRegistrarEnableInputBlocking(channel);
  }
}
