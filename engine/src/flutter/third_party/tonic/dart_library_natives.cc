// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/dart_library_natives.h"

#include "tonic/converter/dart_converter.h"

namespace tonic {

DartLibraryNatives::DartLibraryNatives() {}

DartLibraryNatives::~DartLibraryNatives() {}

void DartLibraryNatives::Register(std::initializer_list<Entry> entries) {
  for (const Entry& entry : entries) {
    symbols_.emplace(entry.native_function, entry.symbol);
    entries_.emplace(entry.symbol, entry);
  }
}

Dart_NativeFunction DartLibraryNatives::GetNativeFunction(
    Dart_Handle name,
    int argument_count,
    bool* auto_setup_scope) {
  std::string name_string = StdStringFromDart(name);
  auto it = entries_.find(name_string);
  if (it == entries_.end())
    return nullptr;
  const Entry& entry = it->second;
  if (entry.argument_count != argument_count)
    return nullptr;
  *auto_setup_scope = entry.auto_setup_scope;
  return entry.native_function;
}

const uint8_t* DartLibraryNatives::GetSymbol(
    Dart_NativeFunction native_function) {
  auto it = symbols_.find(native_function);
  if (it == symbols_.end())
    return nullptr;
  return reinterpret_cast<const uint8_t*>(it->second);
}

}  // namespace tonic
