// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_STATUS_OR_H_
#define FLUTTER_FML_STATUS_OR_H_

#include <optional>

#include "flutter/fml/logging.h"
#include "flutter/fml/status.h"

namespace fml {

// TODO(https://github.com/flutter/flutter/issues/134741): Replace with
// absl::StatusOr.
/// Represents a union type of an object of type `T` and an fml::Status.
///
/// This is often used as a replacement for C++ exceptions where a function that
/// could fail may return an error or a result. These are typically used for
/// errors that are meant to be recovered from. If there is no recovery
/// available `FML_CHECK` is more appropriate.
///
/// Example:
/// StatusOr<int> div(int n, int d) {
///   if (d == 0) {
///     return Status(StatusCode::kFailedPrecondition, "div by zero");
///   }
///   return n / d;
/// }
template <typename T>
class StatusOr {
 public:
  // These constructors are intended be compatible with absl::status_or.
  // NOLINTNEXTLINE(google-explicit-constructor)
  StatusOr(const T& value) : status_(), value_(value) {}

  // These constructors are intended be compatible with absl::status_or.
  // NOLINTNEXTLINE(google-explicit-constructor)
  StatusOr(T&& value) : status_(), value_(std::move(value)) {}

  // These constructors are intended be compatible with absl::status_or.
  // NOLINTNEXTLINE(google-explicit-constructor)
  StatusOr(const Status& status) : status_(status), value_() {
    // It's not valid to construct a StatusOr with an OK status and no value.
    FML_CHECK(!status_.ok());
  }

  StatusOr(const StatusOr&) = default;
  StatusOr(StatusOr&&) = default;

  StatusOr& operator=(const StatusOr&) = default;
  StatusOr& operator=(StatusOr&&) = default;

  StatusOr& operator=(const T& value) {
    status_ = Status();
    value_ = value;
    return *this;
  }

  StatusOr& operator=(const T&& value) {
    status_ = Status();
    value_ = std::move(value);
    return *this;
  }

  StatusOr& operator=(const Status& value) {
    status_ = value;
    value_ = std::nullopt;
    return *this;
  }

  const Status& status() const { return status_; }

  bool ok() const { return status_.ok(); }

  const T& value() const {
    if (value_.has_value()) {
      FML_DCHECK(status_.ok());
      return value_.value();
    }
    FML_LOG(FATAL) << "StatusOr::value() called on error Status";
    FML_UNREACHABLE();
  }

  T& value() {
    if (value_.has_value()) {
      FML_DCHECK(status_.ok());
      return value_.value();
    }
    FML_LOG(FATAL) << "StatusOr::value() called on error Status";
    FML_UNREACHABLE();
  }

 private:
  Status status_;
  std::optional<T> value_;
};

}  // namespace fml

#endif  // FLUTTER_FML_STATUS_OR_H_
