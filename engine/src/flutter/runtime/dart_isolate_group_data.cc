// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_isolate_group_data.h"

#include <utility>

#include "flutter/runtime/dart_snapshot.h"

namespace flutter {

DartIsolateGroupData::DartIsolateGroupData(
    const Settings& settings,
    fml::RefPtr<const DartSnapshot> isolate_snapshot,
    std::string advisory_script_uri,
    std::string advisory_script_entrypoint,
    const ChildIsolatePreparer& child_isolate_preparer,
    const fml::closure& isolate_create_callback,
    const fml::closure& isolate_shutdown_callback,
    std::shared_ptr<NativeAssetsManager> native_assets_manager)
    : settings_(settings),
      isolate_snapshot_(std::move(isolate_snapshot)),
      advisory_script_uri_(std::move(advisory_script_uri)),
      advisory_script_entrypoint_(std::move(advisory_script_entrypoint)),
      child_isolate_preparer_(child_isolate_preparer),
      isolate_create_callback_(isolate_create_callback),
      isolate_shutdown_callback_(isolate_shutdown_callback),
      native_assets_manager_(std::move(native_assets_manager)) {
  FML_DCHECK(isolate_snapshot_) << "Must contain a valid isolate snapshot.";
}

DartIsolateGroupData::~DartIsolateGroupData() = default;

const Settings& DartIsolateGroupData::GetSettings() const {
  return settings_;
}

fml::RefPtr<const DartSnapshot> DartIsolateGroupData::GetIsolateSnapshot()
    const {
  return isolate_snapshot_;
}

const std::string& DartIsolateGroupData::GetAdvisoryScriptURI() const {
  return advisory_script_uri_;
}

const std::string& DartIsolateGroupData::GetAdvisoryScriptEntrypoint() const {
  return advisory_script_entrypoint_;
}

ChildIsolatePreparer DartIsolateGroupData::GetChildIsolatePreparer() const {
  std::scoped_lock lock(child_isolate_preparer_mutex_);
  return child_isolate_preparer_;
}

const fml::closure& DartIsolateGroupData::GetIsolateCreateCallback() const {
  return isolate_create_callback_;
}

const fml::closure& DartIsolateGroupData::GetIsolateShutdownCallback() const {
  return isolate_shutdown_callback_;
}

void DartIsolateGroupData::SetChildIsolatePreparer(
    const ChildIsolatePreparer& value) {
  std::scoped_lock lock(child_isolate_preparer_mutex_);
  child_isolate_preparer_ = value;
}

std::shared_ptr<NativeAssetsManager>
DartIsolateGroupData::GetNativeAssetsManager() const {
  return native_assets_manager_;
}

void DartIsolateGroupData::SetPlatformMessageHandler(
    int64_t root_isolate_token,
    std::weak_ptr<PlatformMessageHandler> handler) {
  std::scoped_lock lock(platform_message_handlers_mutex_);
  platform_message_handlers_[root_isolate_token] = handler;
}

std::weak_ptr<PlatformMessageHandler>
DartIsolateGroupData::GetPlatformMessageHandler(
    int64_t root_isolate_token) const {
  std::scoped_lock lock(platform_message_handlers_mutex_);
  auto it = platform_message_handlers_.find(root_isolate_token);
  return it == platform_message_handlers_.end()
             ? std::weak_ptr<PlatformMessageHandler>()
             : it->second;
}

void DartIsolateGroupData::AddKernelBuffer(
    const std::shared_ptr<const fml::Mapping>& buffer) {
  kernel_buffers_.push_back(buffer);
}

std::vector<std::shared_ptr<const fml::Mapping>>
DartIsolateGroupData::GetKernelBuffers() const {
  return kernel_buffers_;
}

}  // namespace flutter
