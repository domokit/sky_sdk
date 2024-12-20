// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_dart_native_resolver.h"

#include <mutex>
#include <vector>

#include "flutter/fml/logging.h"
#include "third_party/tonic/logging/dart_error.h"
#include "tonic/converter/dart_converter.h"

namespace flutter::testing {

TestDartNativeResolver::TestDartNativeResolver() = default;

TestDartNativeResolver::~TestDartNativeResolver() = default;

void TestDartNativeResolver::AddNativeCallback(const std::string& name,
                                               Dart_NativeFunction callback) {
  native_callbacks_[name] = callback;
}
void TestDartNativeResolver::AddFfiNativeCallback(const std::string& name,
                                                  void* callback_ptr) {
  ffi_native_callbacks_[name] = callback_ptr;
}

Dart_NativeFunction TestDartNativeResolver::ResolveCallback(
    const std::string& name) const {
  auto found = native_callbacks_.find(name);
  if (found == native_callbacks_.end()) {
    return nullptr;
  }

  return found->second;
}

void* TestDartNativeResolver::ResolveFfiCallback(
    const std::string& name) const {
  auto found = ffi_native_callbacks_.find(name);
  if (found == ffi_native_callbacks_.end()) {
    return nullptr;
  }
  return found->second;
}

static std::mutex gIsolateResolversMutex;
static std::map<Dart_Isolate, std::weak_ptr<TestDartNativeResolver>>
    gIsolateResolvers;

Dart_NativeFunction TestDartNativeResolver::DartNativeEntryResolverCallback(
    Dart_Handle dart_name,
    int num_of_arguments,
    bool* auto_setup_scope) {
  auto name = tonic::StdStringFromDart(dart_name);

  std::scoped_lock lock(gIsolateResolversMutex);
  auto found = gIsolateResolvers.find(Dart_CurrentIsolate());
  if (found == gIsolateResolvers.end()) {
    FML_LOG(ERROR) << "Could not resolve native method for :" << name;
    return nullptr;
  }

  if (auto resolver = found->second.lock()) {
    return resolver->ResolveCallback(name);
  } else {
    gIsolateResolvers.erase(found);
  }

  FML_LOG(ERROR) << "Could not resolve native method for :" << name;
  return nullptr;
}

static const uint8_t* DartNativeEntrySymbolCallback(
    Dart_NativeFunction function) {
  return reinterpret_cast<const uint8_t*>("¯\\_(ツ)_/¯");
}

void* TestDartNativeResolver::FfiNativeResolver(const char* name,
                                                uintptr_t args_n) {
  std::scoped_lock lock(gIsolateResolversMutex);
  auto found = gIsolateResolvers.find(Dart_CurrentIsolate());
  if (found == gIsolateResolvers.end()) {
    FML_LOG(ERROR) << "Could not resolve native method for :" << name;
    return nullptr;
  }

  if (auto resolver = found->second.lock()) {
    return resolver->ResolveFfiCallback(name);
  } else {
    gIsolateResolvers.erase(found);
  }

  FML_LOG(ERROR) << "Could not resolve native method for :" << name;
  return nullptr;
}

void TestDartNativeResolver::SetNativeResolverForIsolate() {
  FML_CHECK(!Dart_IsError(Dart_RootLibrary()));
  auto result = Dart_SetNativeResolver(Dart_RootLibrary(),
                                       DartNativeEntryResolverCallback,
                                       DartNativeEntrySymbolCallback);
  FML_CHECK(!tonic::CheckAndHandleError(result))
      << "Could not set native resolver in test.";

  result = Dart_SetFfiNativeResolver(Dart_RootLibrary(), &FfiNativeResolver);
  FML_CHECK(!tonic::CheckAndHandleError(result))
      << "Could not set FFI native resolver in test.";

  std::scoped_lock lock(gIsolateResolversMutex);
  gIsolateResolvers[Dart_CurrentIsolate()] = shared_from_this();

  std::vector<Dart_Isolate> isolates_with_dead_resolvers;
  for (const auto& entry : gIsolateResolvers) {
    if (!entry.second.lock()) {
      isolates_with_dead_resolvers.push_back(entry.first);
    }
  }

  for (const auto& dead_isolate : isolates_with_dead_resolvers) {
    gIsolateResolvers.erase(dead_isolate);
  }
}

}  // namespace flutter::testing
