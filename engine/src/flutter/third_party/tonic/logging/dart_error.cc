// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/logging/dart_error.h"

#include <atomic>

#include "tonic/common/macros.h"
#include "tonic/converter/dart_converter.h"

namespace tonic {
namespace DartError {
const char kInvalidArgument[] = "Invalid argument.";
}  // namespace DartError

namespace {
void DefaultLogUnhandledException(Dart_Handle, Dart_Handle) {}
std::atomic<DartError::UnhandledExceptionReporter> log_unhandled_exception =
    DefaultLogUnhandledException;

void ReportUnhandledException(Dart_Handle exception_handle,
                              Dart_Handle stack_trace_handle) {
  log_unhandled_exception.load()(exception_handle, stack_trace_handle);
}
}  // namespace

void SetUnhandledExceptionReporter(
    DartError::UnhandledExceptionReporter reporter) {
  log_unhandled_exception.store(reporter);
}

bool CheckAndHandleError(Dart_Handle handle) {
  // Specifically handle UnhandledExceptionErrors first. These exclude fatal
  // errors that are shutting down the vm and compilation errors in source code.
  if (Dart_IsUnhandledExceptionError(handle)) {
    Dart_Handle exception_handle = Dart_ErrorGetException(handle);
    Dart_Handle stack_trace_handle = Dart_ErrorGetStackTrace(handle);

    ReportUnhandledException(exception_handle, stack_trace_handle);
    return true;
  } else if (Dart_IsFatalError(handle)) {
    // An UnwindError designed to shutdown isolates. This is thrown by
    // Isolate.exit. This is ordinary API usage, not actually an error, so
    // silently shut down the isolate. The actual isolate shutdown happens in
    // DartMessageHandler::UnhandledError.
    return true;
  } else if (Dart_IsError(handle)) {
    tonic::Log("Dart Error: %s", Dart_GetError(handle));
    return true;
  } else {
    return false;
  }
}

DartErrorHandleType GetErrorHandleType(Dart_Handle handle) {
  if (Dart_IsCompilationError(handle)) {
    return kCompilationErrorType;
  } else if (Dart_IsApiError(handle)) {
    return kApiErrorType;
  } else if (Dart_IsError(handle)) {
    return kUnknownErrorType;
  } else {
    return kNoError;
  }
}

int GetErrorExitCode(Dart_Handle handle) {
  if (Dart_IsCompilationError(handle)) {
    return 254;  // dart::bin::kCompilationErrorExitCode
  } else if (Dart_IsApiError(handle)) {
    return 253;  // dart::bin::kApiErrorExitCode
  } else if (Dart_IsError(handle)) {
    return 255;  // dart::bin::kErrorExitCode
  } else {
    return 0;
  }
}

}  // namespace tonic
