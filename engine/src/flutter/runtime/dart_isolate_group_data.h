// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_ISOLATE_GROUP_DATA_H_
#define FLUTTER_RUNTIME_DART_ISOLATE_GROUP_DATA_H_

#include <map>
#include <mutex>
#include <string>

#include "assets/native_assets.h"
#include "flutter/common/settings.h"
#include "flutter/fml/closure.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/lib/ui/window/platform_configuration.h"

namespace flutter {

class DartIsolate;
class DartSnapshot;
class PlatformMessageHandler;

using ChildIsolatePreparer = std::function<bool(DartIsolate*)>;

// Object holding state associated with a Dart isolate group.  An instance of
// this class will be provided to Dart_CreateIsolateGroup as the
// isolate_group_data.
//
// This object must be thread safe because the Dart VM can invoke the isolate
// group cleanup callback on any thread.
class DartIsolateGroupData : public PlatformMessageHandlerStorage {
 public:
  DartIsolateGroupData(
      const Settings& settings,
      fml::RefPtr<const DartSnapshot> isolate_snapshot,
      std::string advisory_script_uri,
      std::string advisory_script_entrypoint,
      const ChildIsolatePreparer& child_isolate_preparer,
      const fml::closure& isolate_create_callback,
      const fml::closure& isolate_shutdown_callback,
      std::shared_ptr<NativeAssetsManager> native_assets_manager = nullptr);

  ~DartIsolateGroupData();

  const Settings& GetSettings() const;

  fml::RefPtr<const DartSnapshot> GetIsolateSnapshot() const;

  const std::string& GetAdvisoryScriptURI() const;

  const std::string& GetAdvisoryScriptEntrypoint() const;

  ChildIsolatePreparer GetChildIsolatePreparer() const;

  const fml::closure& GetIsolateCreateCallback() const;

  const fml::closure& GetIsolateShutdownCallback() const;

  void SetChildIsolatePreparer(const ChildIsolatePreparer& value);

  std::shared_ptr<NativeAssetsManager> GetNativeAssetsManager() const;

  /// Adds a kernel buffer mapping to the kernels loaded for this isolate group.
  void AddKernelBuffer(const std::shared_ptr<const fml::Mapping>& buffer);

  /// A copy of the mappings for all kernel buffer objects loaded into this
  /// isolate group.
  std::vector<std::shared_ptr<const fml::Mapping>> GetKernelBuffers() const;

  // |PlatformMessageHandlerStorage|
  void SetPlatformMessageHandler(
      int64_t root_isolate_token,
      std::weak_ptr<PlatformMessageHandler> handler) override;

  // |PlatformMessageHandlerStorage|
  std::weak_ptr<PlatformMessageHandler> GetPlatformMessageHandler(
      int64_t root_isolate_token) const override;

 private:
  std::vector<std::shared_ptr<const fml::Mapping>> kernel_buffers_;
  const Settings settings_;
  const fml::RefPtr<const DartSnapshot> isolate_snapshot_;
  const std::string advisory_script_uri_;
  const std::string advisory_script_entrypoint_;
  mutable std::mutex child_isolate_preparer_mutex_;
  ChildIsolatePreparer child_isolate_preparer_;
  const fml::closure isolate_create_callback_;
  const fml::closure isolate_shutdown_callback_;
  std::shared_ptr<NativeAssetsManager> native_assets_manager_;
  std::map<int64_t, std::weak_ptr<PlatformMessageHandler>>
      platform_message_handlers_;
  mutable std::mutex platform_message_handlers_mutex_;

  FML_DISALLOW_COPY_AND_ASSIGN(DartIsolateGroupData);
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_DART_ISOLATE_GROUP_DATA_H_
