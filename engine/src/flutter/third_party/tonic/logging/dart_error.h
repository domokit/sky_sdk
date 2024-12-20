// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_LOGGING_DART_ERROR_H_
#define LIB_TONIC_LOGGING_DART_ERROR_H_

#include "third_party/dart/runtime/include/dart_api.h"

#include "tonic/dart_persistent_value.h"

namespace tonic {

namespace DartError {
using UnhandledExceptionReporter = void (*)(Dart_Handle, Dart_Handle);

extern const char kInvalidArgument[];
}  // namespace DartError

/// Check if a Dart_Handle is an error or exception.
///
/// If it is an error or exception, this method will return true.
///
/// If it is an unhandled error or exception, the closure in
/// |SetUnhandledExceptionReporter| is called. The DartVMInitializer provides
/// that closure, which checks with UIDartState::Current() if it is available
/// and falls back to simply printing the exception and stack to an error log if
/// the settings callback is not provided.
///
/// If UIDartState::Current() is available, it can provide an onError callback
/// that forwards to `PlatformConfiguration.instance.onError`. If that callback
/// is not set, the callback from `Settings.unhandled_exception_callback` is
/// invoked. If that callback is not set, a simple error log is
/// printed.
///
/// If the PlatformDispatcher callback throws an exception, the at least two
/// separate exceptions and stacktraces will be handled by either the
/// Settings.unhandled_exception_callback or the error printer: one for the
/// original exception, and one for the exception thrown in the callback. If the
/// callback returns false, the original exception and stacktrace are logged. If
/// it returns true, no additional logging is done.
///
/// Leaving the PlatformDispatcher.instance.onError callback unset or returning
/// false from it matches the behavior of Flutter applications before the
/// introduction of PlatformDispatcher.onError, which is to print to the error
/// log.
///
/// Dart has errors that are not considered unhandled exceptions, such as
/// Dart_* API usage errors. In these cases, `Dart_IsUnhandledException` returns
/// false but `Dart_IsError` returns true. Such errors are logged to stderr or
/// some similar mechanism provided by the platform such as logcat on Android.
/// Depending on which type of error occurs, the process may crash and the Dart
/// isolate may be unusable. Errors that fall into this category include
/// compilation errors, Dart API errors, and unwind errors that will terminate
/// the Dart VM.
///
/// Historically known as LogIfError.
bool CheckAndHandleError(Dart_Handle handle);

/// The fallback mechanism to log errors if the platform configuration error
/// handler returns false.
///
/// Normally, UIDartState registers with this method in its constructor.
void SetUnhandledExceptionReporter(
    DartError::UnhandledExceptionReporter reporter);

enum DartErrorHandleType {
  kNoError,
  kUnknownErrorType,
  kApiErrorType,
  kCompilationErrorType,
};

DartErrorHandleType GetErrorHandleType(Dart_Handle handle);

int GetErrorExitCode(Dart_Handle handle);

}  // namespace tonic

#endif  // LIB_TONIC_DART_ERROR_H_
