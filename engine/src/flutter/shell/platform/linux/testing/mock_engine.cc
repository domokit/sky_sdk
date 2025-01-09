// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is a historical legacy, predating the proc table API. It has been
// updated to continue to work with the proc table, but new tests should not
// rely on replacements set up here, but instead use test-local replacements
// for any functions relevant to that test.
//
// Over time existing tests should be migrated and this file should be removed.

#include <cstring>
#include <unordered_map>
#include <utility>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_response.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "gtest/gtest.h"

struct _FlutterEngine {
  _FlutterEngine() {}
};

namespace {

FlutterEngineResult FlutterEngineCreateAOTData(
    const FlutterEngineAOTDataSource* source,
    FlutterEngineAOTData* data_out) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineCollectAOTData(FlutterEngineAOTData data) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineInitialize(size_t version,
                                            const FlutterRendererConfig* config,
                                            const FlutterProjectArgs* args,
                                            void* user_data,
                                            FLUTTER_API_SYMBOL(FlutterEngine) *
                                                engine_out) {
  *engine_out = new _FlutterEngine();
  return kSuccess;
}

FlutterEngineResult FlutterEngineRunInitialized(
    FLUTTER_API_SYMBOL(FlutterEngine) engine) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineRun(size_t version,
                                     const FlutterRendererConfig* config,
                                     const FlutterProjectArgs* args,
                                     void* user_data,
                                     FLUTTER_API_SYMBOL(FlutterEngine) *
                                         engine_out) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineShutdown(FLUTTER_API_SYMBOL(FlutterEngine)
                                              engine) {
  delete engine;
  return kSuccess;
}

FlutterEngineResult FlutterEngineDeinitialize(FLUTTER_API_SYMBOL(FlutterEngine)
                                                  engine) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineSendWindowMetricsEvent(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterWindowMetricsEvent* event) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineSendPointerEvent(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPointerEvent* events,
    size_t events_count) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineSendKeyEvent(FLUTTER_API_SYMBOL(FlutterEngine)
                                                  engine,
                                              const FlutterKeyEvent* event,
                                              FlutterKeyEventCallback callback,
                                              void* user_data) {
  return kSuccess;
}

FLUTTER_EXPORT
FlutterEngineResult FlutterEngineSendPlatformMessage(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessage* message) {
  return kSuccess;
}

FlutterEngineResult FlutterPlatformMessageCreateResponseHandle(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterDataCallback data_callback,
    void* user_data,
    FlutterPlatformMessageResponseHandle** response_out) {
  return kSuccess;
}

FlutterEngineResult FlutterPlatformMessageReleaseResponseHandle(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterPlatformMessageResponseHandle* response) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineSendPlatformMessageResponse(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    const FlutterPlatformMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineRunTask(FLUTTER_API_SYMBOL(FlutterEngine)
                                             engine,
                                         const FlutterTask* task) {
  return kSuccess;
}

bool FlutterEngineRunsAOTCompiledDartCode() {
  return false;
}

FlutterEngineResult FlutterEngineUpdateLocales(FLUTTER_API_SYMBOL(FlutterEngine)
                                                   engine,
                                               const FlutterLocale** locales,
                                               size_t locales_count) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineUpdateSemanticsEnabled(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    bool enabled) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineUpdateAccessibilityFeatures(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterAccessibilityFeature features) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineDispatchSemanticsAction(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    uint64_t id,
    FlutterSemanticsAction action,
    const uint8_t* data,
    size_t data_length) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineRegisterExternalTexture(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineMarkExternalTextureFrameAvailable(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineUnregisterExternalTexture(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    int64_t texture_identifier) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineNotifyDisplayUpdate(
    FLUTTER_API_SYMBOL(FlutterEngine) engine,
    FlutterEngineDisplaysUpdateType update_type,
    const FlutterEngineDisplay* displays,
    size_t display_count) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineAddView(FLUTTER_API_SYMBOL(FlutterEngine)
                                             engine,
                                         const FlutterAddViewInfo* info) {
  return kSuccess;
}

FlutterEngineResult FlutterEngineRemoveView(FLUTTER_API_SYMBOL(FlutterEngine)
                                                engine,
                                            const FlutterRemoveViewInfo* info) {
  return kSuccess;
}

}  // namespace

FlutterEngineResult FlutterEngineGetProcAddresses(
    FlutterEngineProcTable* table) {
  if (!table) {
    return kInvalidArguments;
  }

  FlutterEngineProcTable empty_table = {};
  *table = empty_table;

  table->CreateAOTData = &FlutterEngineCreateAOTData;
  table->CollectAOTData = &FlutterEngineCollectAOTData;
  table->Run = &FlutterEngineRun;
  table->Shutdown = &FlutterEngineShutdown;
  table->Initialize = &FlutterEngineInitialize;
  table->Deinitialize = &FlutterEngineDeinitialize;
  table->RunInitialized = &FlutterEngineRunInitialized;
  table->SendWindowMetricsEvent = &FlutterEngineSendWindowMetricsEvent;
  table->SendPointerEvent = &FlutterEngineSendPointerEvent;
  table->SendKeyEvent = &FlutterEngineSendKeyEvent;
  table->SendPlatformMessage = &FlutterEngineSendPlatformMessage;
  table->PlatformMessageCreateResponseHandle =
      &FlutterPlatformMessageCreateResponseHandle;
  table->PlatformMessageReleaseResponseHandle =
      &FlutterPlatformMessageReleaseResponseHandle;
  table->SendPlatformMessageResponse =
      &FlutterEngineSendPlatformMessageResponse;
  table->RunTask = &FlutterEngineRunTask;
  table->UpdateLocales = &FlutterEngineUpdateLocales;
  table->UpdateSemanticsEnabled = &FlutterEngineUpdateSemanticsEnabled;
  table->DispatchSemanticsAction = &FlutterEngineDispatchSemanticsAction;
  table->RunsAOTCompiledDartCode = &FlutterEngineRunsAOTCompiledDartCode;
  table->RegisterExternalTexture = &FlutterEngineRegisterExternalTexture;
  table->MarkExternalTextureFrameAvailable =
      &FlutterEngineMarkExternalTextureFrameAvailable;
  table->UnregisterExternalTexture = &FlutterEngineUnregisterExternalTexture;
  table->UpdateAccessibilityFeatures =
      &FlutterEngineUpdateAccessibilityFeatures;
  table->NotifyDisplayUpdate = &FlutterEngineNotifyDisplayUpdate;
  table->AddView = &FlutterEngineAddView;
  table->RemoveView = &FlutterEngineRemoveView;
  return kSuccess;
}
