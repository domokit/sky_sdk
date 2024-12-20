// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_class_library.h"

#include "tonic/common/macros.h"
#include "tonic/dart_wrapper_info.h"

namespace tonic {

DartClassLibrary::DartClassLibrary() {}

DartClassLibrary::~DartClassLibrary() {
  // Note that we don't need to delete these persistent handles because this
  // object lives as long as the isolate. The handles will get deleted when the
  // isolate dies.
}

Dart_PersistentHandle DartClassLibrary::GetClass(const DartWrapperInfo& info) {
  const auto& result = info_cache_.insert(std::make_pair(&info, nullptr));
  if (!result.second) {
    // Already present, return value.
    return result.first->second;
  }
  return GetAndCacheClass(info.library_name, info.interface_name,
                          &result.first->second);
}

Dart_PersistentHandle DartClassLibrary::GetClass(
    const std::string& library_name,
    const std::string& interface_name) {
  auto key = std::make_pair(library_name, interface_name);
  const auto& result = name_cache_.insert(std::make_pair(key, nullptr));
  if (!result.second) {
    // Already present, return value.
    return result.first->second;
  }
  return GetAndCacheClass(library_name.c_str(), interface_name.c_str(),
                          &result.first->second);
}

Dart_PersistentHandle DartClassLibrary::GetAndCacheClass(
    const char* library_name,
    const char* interface_name,
    Dart_PersistentHandle* cache_slot) {
  auto it = providers_.find(library_name);
  TONIC_DCHECK(it != providers_.end());

  Dart_Handle class_handle = it->second->GetClassByName(interface_name);
  *cache_slot = Dart_NewPersistentHandle(class_handle);
  return *cache_slot;
}

}  // namespace tonic
