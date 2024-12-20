// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MAKE_COPYABLE_H_
#define FLUTTER_FML_MAKE_COPYABLE_H_

#include <utility>

#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/ref_ptr.h"

namespace fml {
namespace internal {

template <typename T>
class CopyableLambda {
 public:
  explicit CopyableLambda(T func)
      : impl_(MakeRefCounted<Impl>(std::move(func))) {}

  template <typename... ArgType>
  auto operator()(ArgType&&... args) const {
    return impl_->func_(std::forward<ArgType>(args)...);
  }

 private:
  class Impl : public RefCountedThreadSafe<Impl> {
   public:
    explicit Impl(T func) : func_(std::move(func)) {}
    T func_;
  };

  RefPtr<Impl> impl_;
};

}  // namespace internal

// Provides a wrapper for a move-only lambda that is implictly convertable to an
// std::function.
//
// std::function is copyable, but if a lambda captures an argument with a
// move-only type, the lambda itself is not copyable. In order to use the lambda
// in places that accept std::functions, we provide a copyable object that wraps
// the lambda and is implicitly convertable to an std::function.
//
// EXAMPLE:
//
// std::unique_ptr<Foo> foo = ...
// std::function<int()> func =
//     fml::MakeCopyable([bar = std::move(foo)]() { return bar->count(); });
//
// Notice that the return type of MakeCopyable is rarely used directly. Instead,
// callers typically erase the type by implicitly converting the return value
// to an std::function.
template <typename T>
internal::CopyableLambda<T> MakeCopyable(T lambda) {
  return internal::CopyableLambda<T>(std::move(lambda));
}

}  // namespace fml

#endif  // FLUTTER_FML_MAKE_COPYABLE_H_
