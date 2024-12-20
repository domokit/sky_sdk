// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_PROMISE_H_
#define FLUTTER_IMPELLER_BASE_PROMISE_H_

#include <future>

namespace impeller {

template <class T>
std::future<T> RealizedFuture(T t) {
  std::promise<T> promise;
  auto future = promise.get_future();
  promise.set_value(std::move(t));
  return future;
}

// Wraps a std::promise and completes the promise with a value during
// destruction if the promise does not already have a value.
//
// By default the std::promise destructor will complete an empty promise with an
// exception. This will fail because Flutter is built without exception support.
template <typename T>
class NoExceptionPromise {
 public:
  NoExceptionPromise() = default;

  ~NoExceptionPromise() {
    if (!value_set_) {
      promise_.set_value({});
    }
  }

  std::future<T> get_future() { return promise_.get_future(); }

  void set_value(const T& value) {
    promise_.set_value(value);
    value_set_ = true;
  }

 private:
  std::promise<T> promise_;
  bool value_set_ = false;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_PROMISE_H_
