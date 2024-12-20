// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_HANDLE_EXCEPTION_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_HANDLE_EXCEPTION_H_

#include <lib/sys/cpp/service_directory.h>

#include <memory>
#include <string>

#include "third_party/dart/runtime/include/dart_api.h"

namespace dart_utils {

// If |result| is a Dart Exception, passes the exception message and stack trace
// to the crash analyzer service for further handling.
void HandleIfException(std::shared_ptr<::sys::ServiceDirectory> services,
                       const std::string& component_url,
                       Dart_Handle result);

// Passes the exception message and stack trace to the crash analyzer service
// for further handling.
void HandleException(std::shared_ptr<::sys::ServiceDirectory> services,
                     const std::string& component_url,
                     const std::string& error,
                     const std::string& stack_trace);

}  // namespace dart_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_RUNTIME_DART_UTILS_HANDLE_EXCEPTION_H_
