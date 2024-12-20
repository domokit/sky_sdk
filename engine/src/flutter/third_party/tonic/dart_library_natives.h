// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_LIBRARY_NATIVES_H_
#define LIB_TONIC_DART_LIBRARY_NATIVES_H_

#include <initializer_list>
#include <string>
#include <unordered_map>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"

namespace tonic {

class DartLibraryNatives {
 public:
  DartLibraryNatives();
  ~DartLibraryNatives();

  struct Entry {
    const char* symbol;
    Dart_NativeFunction native_function;
    int argument_count;
    bool auto_setup_scope;
  };

  void Register(std::initializer_list<Entry> entries);

  Dart_NativeFunction GetNativeFunction(Dart_Handle name,
                                        int argument_count,
                                        bool* auto_setup_scope);
  const uint8_t* GetSymbol(Dart_NativeFunction native_function);

 private:
  std::unordered_map<std::string, Entry> entries_;
  std::unordered_map<Dart_NativeFunction, const char*> symbols_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartLibraryNatives);
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_LIBRARY_NATIVES_H_
