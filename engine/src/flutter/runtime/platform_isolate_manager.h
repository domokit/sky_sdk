// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_PLATFORM_ISOLATE_MANAGER_H_
#define FLUTTER_RUNTIME_PLATFORM_ISOLATE_MANAGER_H_

#include <atomic>
#include <mutex>
#include <unordered_set>

#include "third_party/dart/runtime/include/dart_api.h"

namespace flutter {

/// Maintains a list of registered platform isolates, so that they can be
/// proactively shutdown as a group during shell shutdown.
class PlatformIsolateManager {
 public:
  /// Returns whether the PlatformIsolateManager is shutdown. New isolates
  /// cannot be registered after the manager is shutdown. Must be called on the
  /// platform thread.
  bool HasShutdown();

  /// Returns whether the PlatformIsolateManager is shutdown. New isolates
  /// cannot be registered after the manager is shutdown. Callable from any
  /// thread. The result may be obsolete immediately after the call.
  bool HasShutdownMaybeFalseNegative();

  /// Register an isolate in the list of platform isolates. Callable from any
  /// thread.
  bool RegisterPlatformIsolate(Dart_Isolate isolate);

  /// Remove an isolate from the list of platform isolates. Must be called from
  /// the platform thread.
  void RemovePlatformIsolate(Dart_Isolate isolate);

  /// Shuts down all registered isolates, and the manager itself. Must be called
  /// from the platform thread.
  void ShutdownPlatformIsolates();

  /// Returns whether an isolate is registered. For testing only. Callable from
  /// any thread.
  bool IsRegisteredForTestingOnly(Dart_Isolate isolate);

 private:
  // This lock must be recursive because ShutdownPlatformIsolates indirectly
  // calls RemovePlatformIsolate.
  std::recursive_mutex lock_;
  std::unordered_set<Dart_Isolate> platform_isolates_;
  bool is_shutdown_ = false;
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_PLATFORM_ISOLATE_MANAGER_H_
