// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_VM_INITIALIZER_H_
#define FLUTTER_RUNTIME_DART_VM_INITIALIZER_H_

#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"

class DartVMInitializer {
 public:
  static void Initialize(Dart_InitializeParams* params,
                         bool enable_timeline_event_handler,
                         bool trace_systrace);
  static void Cleanup();

 private:
  static void LogDartTimelineEvent(const char* label,
                                   int64_t timestamp0,
                                   int64_t timestamp1_or_async_id,
                                   intptr_t flow_id_count,
                                   const int64_t* flow_ids,
                                   Dart_Timeline_Event_Type type,
                                   intptr_t argument_count,
                                   const char** argument_names,
                                   const char** argument_values);
};

#endif  // FLUTTER_RUNTIME_DART_VM_INITIALIZER_H_
