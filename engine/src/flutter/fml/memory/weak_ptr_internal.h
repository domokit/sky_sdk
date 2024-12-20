// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MEMORY_WEAK_PTR_INTERNAL_H_
#define FLUTTER_FML_MEMORY_WEAK_PTR_INTERNAL_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"

namespace fml {
namespace internal {

// |WeakPtr<T>|s have a reference to a |WeakPtrFlag| to determine whether they
// are valid (non-null) or not. We do not store a |T*| in this object since
// there may also be |WeakPtr<U>|s to the same object, where |U| is a superclass
// of |T|.
//
// This class in not thread-safe, though references may be released on any
// thread (allowing weak pointers to be destroyed/reset/reassigned on any
// thread).
class WeakPtrFlag : public fml::RefCountedThreadSafe<WeakPtrFlag> {
 public:
  WeakPtrFlag();

  ~WeakPtrFlag();

  bool is_valid() const { return is_valid_; }

  void Invalidate();

 private:
  bool is_valid_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(WeakPtrFlag);
};

}  // namespace internal
}  // namespace fml

#endif  // FLUTTER_FML_MEMORY_WEAK_PTR_INTERNAL_H_
