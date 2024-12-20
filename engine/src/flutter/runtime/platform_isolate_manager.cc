// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/platform_isolate_manager.h"

#include "flutter/runtime/dart_isolate.h"

namespace flutter {

bool PlatformIsolateManager::HasShutdown() {
  // TODO(flutter/flutter#136314): Assert that we're on the platform thread.
  std::scoped_lock lock(lock_);
  return is_shutdown_;
}

bool PlatformIsolateManager::HasShutdownMaybeFalseNegative() {
  std::scoped_lock lock(lock_);
  return is_shutdown_;
}

bool PlatformIsolateManager::RegisterPlatformIsolate(Dart_Isolate isolate) {
  std::scoped_lock lock(lock_);
  if (is_shutdown_) {
    // It's possible shutdown occured while we were trying to aquire the lock.
    return false;
  }
  FML_DCHECK(platform_isolates_.find(isolate) == platform_isolates_.end());
  platform_isolates_.insert(isolate);
  return true;
}

void PlatformIsolateManager::RemovePlatformIsolate(Dart_Isolate isolate) {
  // This method is only called by DartIsolate::OnShutdownCallback() during
  // isolate shutdown. This can happen either during the ordinary platform
  // isolate shutdown, or during ShutdownPlatformIsolates(). In either case
  // we're on the platform thread.
  // TODO(flutter/flutter#136314): Assert that we're on the platform thread.
  // Need a method that works for ShutdownPlatformIsolates() too.
  std::scoped_lock lock(lock_);
  if (is_shutdown_) {
    // Removal during ShutdownPlatformIsolates. Ignore, to avoid modifying
    // platform_isolates_ during iteration.
    FML_DCHECK(platform_isolates_.empty());
    return;
  }
  FML_DCHECK(platform_isolates_.find(isolate) != platform_isolates_.end());
  platform_isolates_.erase(isolate);
}

void PlatformIsolateManager::ShutdownPlatformIsolates() {
  // TODO(flutter/flutter#136314): Assert that we're on the platform thread.
  // There's no current UIDartState here, so platform_isolate.cc's method won't
  // work.
  std::scoped_lock lock(lock_);
  is_shutdown_ = true;
  std::unordered_set<Dart_Isolate> platform_isolates;
  std::swap(platform_isolates_, platform_isolates);
  for (Dart_Isolate isolate : platform_isolates) {
    Dart_EnterIsolate(isolate);
    Dart_ShutdownIsolate();
  }
}

bool PlatformIsolateManager::IsRegisteredForTestingOnly(Dart_Isolate isolate) {
  std::scoped_lock lock(lock_);
  return platform_isolates_.find(isolate) != platform_isolates_.end();
}

}  // namespace flutter
