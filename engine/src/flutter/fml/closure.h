// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_CLOSURE_H_
#define FLUTTER_FML_CLOSURE_H_

#include <functional>

#include "flutter/fml/macros.h"

namespace fml {

using closure = std::function<void()>;

//------------------------------------------------------------------------------
/// @brief      Wraps a closure that is invoked in the destructor unless
///             released by the caller.
///
///             This is especially useful in dealing with APIs that return a
///             resource by accepting ownership of a sub-resource and a closure
///             that releases that resource. When such APIs are chained, each
///             link in the chain must check that the next member in the chain
///             has accepted the resource. If not, it must invoke the closure
///             eagerly. Not doing this results in a resource leak in the
///             erroneous case. Using this wrapper, the closure can be released
///             once the next call in the chain has successfully accepted
///             ownership of the resource. If not, the closure gets invoked
///             automatically at the end of the scope. This covers the cases
///             where there are early returns as well.
///
class ScopedCleanupClosure final {
 public:
  ScopedCleanupClosure() = default;

  ScopedCleanupClosure(ScopedCleanupClosure&& other) {
    closure_ = other.Release();
  }

  ScopedCleanupClosure& operator=(ScopedCleanupClosure&& other) {
    closure_ = other.Release();
    return *this;
  }

  explicit ScopedCleanupClosure(const fml::closure& closure)
      : closure_(closure) {}

  ~ScopedCleanupClosure() { Reset(); }

  fml::closure SetClosure(const fml::closure& closure) {
    auto old_closure = closure_;
    closure_ = closure;
    return old_closure;
  }

  fml::closure Release() {
    fml::closure closure = closure_;
    closure_ = nullptr;
    return closure;
  }

  void Reset() {
    if (closure_) {
      closure_();
      closure_ = nullptr;
    }
  }

 private:
  fml::closure closure_;

  FML_DISALLOW_COPY_AND_ASSIGN(ScopedCleanupClosure);
};

}  // namespace fml

#endif  // FLUTTER_FML_CLOSURE_H_
