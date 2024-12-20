// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_RESULT_FUNCTIONS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_RESULT_FUNCTIONS_H_

#include <functional>
#include <string>
#include <utility>

#include "method_result.h"

namespace flutter {

class EncodableValue;

// Handler types for each of the MethodResult outcomes.
template <typename T>
using ResultHandlerSuccess = std::function<void(const T* result)>;
template <typename T>
using ResultHandlerError = std::function<void(const std::string& error_code,
                                              const std::string& error_message,
                                              const T* error_details)>;
template <typename T>
using ResultHandlerNotImplemented = std::function<void()>;

// An implementation of MethodResult that pass calls through to provided
// function objects, for ease of constructing one-off result handlers.
template <typename T = EncodableValue>
class MethodResultFunctions : public MethodResult<T> {
 public:
  // Creates a result object that calls the provided functions for the
  // corresponding MethodResult outcomes.
  MethodResultFunctions(ResultHandlerSuccess<T> on_success,
                        ResultHandlerError<T> on_error,
                        ResultHandlerNotImplemented<T> on_not_implemented)
      : on_success_(on_success),
        on_error_(on_error),
        on_not_implemented_(std::move(on_not_implemented)) {}

  virtual ~MethodResultFunctions() = default;

  // Prevent copying.
  MethodResultFunctions(MethodResultFunctions const&) = delete;
  MethodResultFunctions& operator=(MethodResultFunctions const&) = delete;

 protected:
  // |flutter::MethodResult|
  void SuccessInternal(const T* result) override {
    if (on_success_) {
      on_success_(result);
    }
  }

  // |flutter::MethodResult|
  void ErrorInternal(const std::string& error_code,
                     const std::string& error_message,
                     const T* error_details) override {
    if (on_error_) {
      on_error_(error_code, error_message, error_details);
    }
  }

  // |flutter::MethodResult|
  void NotImplementedInternal() override {
    if (on_not_implemented_) {
      on_not_implemented_();
    }
  }

 private:
  ResultHandlerSuccess<T> on_success_;
  ResultHandlerError<T> on_error_;
  ResultHandlerNotImplemented<T> on_not_implemented_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_RESULT_FUNCTIONS_H_
