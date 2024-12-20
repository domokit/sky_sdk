// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/client_wrapper/testing/stub_flutter_windows_api.h"

static flutter::testing::StubFlutterWindowsApi* s_stub_implementation;

namespace flutter {
namespace testing {

// static
void StubFlutterWindowsApi::SetTestStub(StubFlutterWindowsApi* stub) {
  s_stub_implementation = stub;
}

// static
StubFlutterWindowsApi* StubFlutterWindowsApi::GetTestStub() {
  return s_stub_implementation;
}

ScopedStubFlutterWindowsApi::ScopedStubFlutterWindowsApi(
    std::unique_ptr<StubFlutterWindowsApi> stub)
    : stub_(std::move(stub)) {
  previous_stub_ = StubFlutterWindowsApi::GetTestStub();
  StubFlutterWindowsApi::SetTestStub(stub_.get());
}

ScopedStubFlutterWindowsApi::~ScopedStubFlutterWindowsApi() {
  StubFlutterWindowsApi::SetTestStub(previous_stub_);
}

}  // namespace testing
}  // namespace flutter

// Forwarding dummy implementations of the C API.

FlutterDesktopViewControllerRef FlutterDesktopViewControllerCreate(
    int width,
    int height,
    FlutterDesktopEngineRef engine) {
  if (s_stub_implementation) {
    return s_stub_implementation->ViewControllerCreate(width, height, engine);
  }
  return nullptr;
}

void FlutterDesktopViewControllerDestroy(
    FlutterDesktopViewControllerRef controller) {
  if (s_stub_implementation) {
    s_stub_implementation->ViewControllerDestroy();
  }
}

FlutterDesktopViewId FlutterDesktopViewControllerGetViewId(
    FlutterDesktopViewControllerRef controller) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return static_cast<FlutterDesktopViewId>(1);
}

FlutterDesktopEngineRef FlutterDesktopViewControllerGetEngine(
    FlutterDesktopViewControllerRef controller) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopEngineRef>(1);
}

FlutterDesktopViewRef FlutterDesktopViewControllerGetView(
    FlutterDesktopViewControllerRef controller) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopViewRef>(1);
}

void FlutterDesktopViewControllerForceRedraw(
    FlutterDesktopViewControllerRef controller) {
  if (s_stub_implementation) {
    s_stub_implementation->ViewControllerForceRedraw();
  }
}

bool FlutterDesktopViewControllerHandleTopLevelWindowProc(
    FlutterDesktopViewControllerRef controller,
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam,
    LRESULT* result) {
  if (s_stub_implementation) {
    return s_stub_implementation->ViewControllerHandleTopLevelWindowProc(
        hwnd, message, wparam, lparam, result);
  }
  return false;
}

FlutterDesktopEngineRef FlutterDesktopEngineCreate(
    const FlutterDesktopEngineProperties* engine_properties) {
  if (s_stub_implementation) {
    return s_stub_implementation->EngineCreate(*engine_properties);
  }
  return nullptr;
}

bool FlutterDesktopEngineDestroy(FlutterDesktopEngineRef engine_ref) {
  if (s_stub_implementation) {
    return s_stub_implementation->EngineDestroy();
  }
  return true;
}

bool FlutterDesktopEngineRun(FlutterDesktopEngineRef engine,
                             const char* entry_point) {
  if (s_stub_implementation) {
    return s_stub_implementation->EngineRun(entry_point);
  }
  return true;
}

uint64_t FlutterDesktopEngineProcessMessages(FlutterDesktopEngineRef engine) {
  if (s_stub_implementation) {
    return s_stub_implementation->EngineProcessMessages();
  }
  return 0;
}

void FlutterDesktopEngineSetNextFrameCallback(FlutterDesktopEngineRef engine,
                                              VoidCallback callback,
                                              void* user_data) {
  if (s_stub_implementation) {
    s_stub_implementation->EngineSetNextFrameCallback(callback, user_data);
  }
}

void FlutterDesktopEngineReloadSystemFonts(FlutterDesktopEngineRef engine) {
  if (s_stub_implementation) {
    s_stub_implementation->EngineReloadSystemFonts();
  }
}

FlutterDesktopPluginRegistrarRef FlutterDesktopEngineGetPluginRegistrar(
    FlutterDesktopEngineRef engine,
    const char* plugin_name) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1);
}

FlutterDesktopMessengerRef FlutterDesktopEngineGetMessenger(
    FlutterDesktopEngineRef engine) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopMessengerRef>(2);
}

FlutterDesktopTextureRegistrarRef FlutterDesktopEngineGetTextureRegistrar(
    FlutterDesktopEngineRef engine) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopTextureRegistrarRef>(3);
}

HWND FlutterDesktopViewGetHWND(FlutterDesktopViewRef controller) {
  if (s_stub_implementation) {
    return s_stub_implementation->ViewGetHWND();
  }
  return reinterpret_cast<HWND>(-1);
}

IDXGIAdapter* FlutterDesktopViewGetGraphicsAdapter(FlutterDesktopViewRef view) {
  if (s_stub_implementation) {
    return s_stub_implementation->ViewGetGraphicsAdapter();
  }
  return nullptr;
}

bool FlutterDesktopEngineProcessExternalWindowMessage(
    FlutterDesktopEngineRef engine,
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam,
    LRESULT* result) {
  if (s_stub_implementation) {
    return s_stub_implementation->EngineProcessExternalWindowMessage(
        engine, hwnd, message, wparam, lparam, result);
  }
  return false;
}

void FlutterDesktopEngineRegisterPlatformViewType(
    FlutterDesktopEngineRef engine,
    const char* view_type_name,
    FlutterPlatformViewTypeEntry view_type) {
  if (s_stub_implementation) {
    s_stub_implementation->EngineRegisterPlatformViewType(view_type_name,
                                                          view_type);
  }
}

FlutterDesktopViewRef FlutterDesktopPluginRegistrarGetView(
    FlutterDesktopPluginRegistrarRef controller) {
  if (s_stub_implementation) {
    return s_stub_implementation->PluginRegistrarGetView();
  }
  return nullptr;
}

FlutterDesktopViewRef FlutterDesktopPluginRegistrarGetViewById(
    FlutterDesktopPluginRegistrarRef controller,
    FlutterDesktopViewId view_id) {
  if (s_stub_implementation) {
    return s_stub_implementation->PluginRegistrarGetViewById(view_id);
  }
  return nullptr;
}

void FlutterDesktopPluginRegistrarRegisterTopLevelWindowProcDelegate(
    FlutterDesktopPluginRegistrarRef registrar,
    FlutterDesktopWindowProcCallback delegate,
    void* user_data) {
  if (s_stub_implementation) {
    return s_stub_implementation
        ->PluginRegistrarRegisterTopLevelWindowProcDelegate(delegate,
                                                            user_data);
  }
}

void FlutterDesktopPluginRegistrarUnregisterTopLevelWindowProcDelegate(
    FlutterDesktopPluginRegistrarRef registrar,
    FlutterDesktopWindowProcCallback delegate) {
  if (s_stub_implementation) {
    return s_stub_implementation
        ->PluginRegistrarUnregisterTopLevelWindowProcDelegate(delegate);
  }
}
