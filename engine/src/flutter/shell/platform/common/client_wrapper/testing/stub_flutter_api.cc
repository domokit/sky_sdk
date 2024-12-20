// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/testing/stub_flutter_api.h"

#include <cassert>

static flutter::testing::StubFlutterApi* s_stub_implementation;

namespace flutter {
namespace testing {

// static
void StubFlutterApi::SetTestStub(StubFlutterApi* stub) {
  s_stub_implementation = stub;
}

// static
StubFlutterApi* StubFlutterApi::GetTestStub() {
  return s_stub_implementation;
}

ScopedStubFlutterApi::ScopedStubFlutterApi(std::unique_ptr<StubFlutterApi> stub)
    : stub_(std::move(stub)) {
  previous_stub_ = StubFlutterApi::GetTestStub();
  StubFlutterApi::SetTestStub(stub_.get());
}

ScopedStubFlutterApi::~ScopedStubFlutterApi() {
  StubFlutterApi::SetTestStub(previous_stub_);
}

}  // namespace testing
}  // namespace flutter

// Forwarding dummy implementations of the C API.

FlutterDesktopMessengerRef FlutterDesktopPluginRegistrarGetMessenger(
    FlutterDesktopPluginRegistrarRef registrar) {
  // The stub ignores this, so just return an arbitrary non-zero value.
  return reinterpret_cast<FlutterDesktopMessengerRef>(1);
}

void FlutterDesktopPluginRegistrarSetDestructionHandler(
    FlutterDesktopPluginRegistrarRef registrar,
    FlutterDesktopOnPluginRegistrarDestroyed callback) {
  if (s_stub_implementation) {
    s_stub_implementation->PluginRegistrarSetDestructionHandler(callback);
  }
}

bool FlutterDesktopMessengerSend(FlutterDesktopMessengerRef messenger,
                                 const char* channel,
                                 const uint8_t* message,
                                 const size_t message_size) {
  bool result = false;
  if (s_stub_implementation) {
    result =
        s_stub_implementation->MessengerSend(channel, message, message_size);
  }
  return result;
}

bool FlutterDesktopMessengerSendWithReply(FlutterDesktopMessengerRef messenger,
                                          const char* channel,
                                          const uint8_t* message,
                                          const size_t message_size,
                                          const FlutterDesktopBinaryReply reply,
                                          void* user_data) {
  bool result = false;
  if (s_stub_implementation) {
    result = s_stub_implementation->MessengerSendWithReply(
        channel, message, message_size, reply, user_data);
  }
  return result;
}

void FlutterDesktopMessengerSendResponse(
    FlutterDesktopMessengerRef messenger,
    const FlutterDesktopMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length) {
  if (s_stub_implementation) {
    s_stub_implementation->MessengerSendResponse(handle, data, data_length);
  }
}

void FlutterDesktopMessengerSetCallback(FlutterDesktopMessengerRef messenger,
                                        const char* channel,
                                        FlutterDesktopMessageCallback callback,
                                        void* user_data) {
  if (s_stub_implementation) {
    s_stub_implementation->MessengerSetCallback(channel, callback, user_data);
  }
}

FlutterDesktopMessengerRef FlutterDesktopMessengerAddRef(
    FlutterDesktopMessengerRef messenger) {
  assert(false);  // not implemented
  return nullptr;
}

void FlutterDesktopMessengerRelease(FlutterDesktopMessengerRef messenger) {
  assert(false);  // not implemented
}

bool FlutterDesktopMessengerIsAvailable(FlutterDesktopMessengerRef messenger) {
  assert(false);  // not implemented
  return false;
}

FlutterDesktopMessengerRef FlutterDesktopMessengerLock(
    FlutterDesktopMessengerRef messenger) {
  assert(false);  // not implemented
  return nullptr;
}

void FlutterDesktopMessengerUnlock(FlutterDesktopMessengerRef messenger) {
  assert(false);  // not implemented
}

FlutterDesktopTextureRegistrarRef FlutterDesktopRegistrarGetTextureRegistrar(
    FlutterDesktopPluginRegistrarRef registrar) {
  return reinterpret_cast<FlutterDesktopTextureRegistrarRef>(1);
}

int64_t FlutterDesktopTextureRegistrarRegisterExternalTexture(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    const FlutterDesktopTextureInfo* info) {
  uint64_t result = -1;
  if (s_stub_implementation) {
    result =
        s_stub_implementation->TextureRegistrarRegisterExternalTexture(info);
  }
  return result;
}

void FlutterDesktopTextureRegistrarUnregisterExternalTexture(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    int64_t texture_id,
    void (*callback)(void* user_data),
    void* user_data) {
  if (s_stub_implementation) {
    s_stub_implementation->TextureRegistrarUnregisterExternalTexture(
        texture_id, callback, user_data);
  } else if (callback) {
    callback(user_data);
  }
}

bool FlutterDesktopTextureRegistrarMarkExternalTextureFrameAvailable(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    int64_t texture_id) {
  bool result = false;
  if (s_stub_implementation) {
    result = s_stub_implementation->TextureRegistrarMarkTextureFrameAvailable(
        texture_id);
  }
  return result;
}
