// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_PTRACE_CHECK_H_
#define FLUTTER_RUNTIME_PTRACE_CHECK_H_

#include "flutter/common/settings.h"
#include "flutter/fml/build_config.h"

namespace flutter {

#define TRACING_CHECKS_NECESSARY        \
  FML_OS_IOS && !TARGET_OS_SIMULATOR && \
      (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG)

enum class TracingResult {
  kNotAttempted,
  kEnabled,
  kNotNecessary = kEnabled,
  kDisabled,
};

#if TRACING_CHECKS_NECESSARY
bool EnableTracingIfNecessaryImpl(const Settings& vm_settings);
TracingResult GetTracingResultImpl();
#endif  // TRACING_CHECKS_NECESSARY

//------------------------------------------------------------------------------
/// @brief      Enables tracing in the process so that JIT mode VMs may be
///             launched. Explicitly enabling tracing is not required on all
///             platforms. On platforms where it is not required, calling this
///             method will return true. If tracing is required but cannot be
///             enabled, it is the responsibility of the caller to display the
///             appropriate error message to the user as subsequent attempts to
///             launch the VM in JIT mode will cause process termination.
///
///             This method may be called multiple times and will return the
///             same result. There are no threading restrictions.
///
/// @param[in]  vm_settings  The settings used to launch the VM.
///
/// @return     If tracing was enabled.
///
inline bool EnableTracingIfNecessary(const Settings& vm_settings) {
#if TRACING_CHECKS_NECESSARY
  return EnableTracingIfNecessaryImpl(vm_settings);
#else   // TRACING_CHECKS_NECESSARY
  return true;
#endif  // TRACING_CHECKS_NECESSARY
}

//------------------------------------------------------------------------------
/// @brief      Returns if a tracing check has been performed and its result. To
///             enable tracing, the Settings object used to launch the VM is
///             required. Components may want to display messages based on the
///             result of a previous tracing check without actually having the
///             settings object. This accessor can be used instead.
///
/// @return     The tracing result.
///
inline TracingResult GetTracingResult() {
#if TRACING_CHECKS_NECESSARY
  return GetTracingResultImpl();
#else   // TRACING_CHECKS_NECESSARY
  return TracingResult::kNotNecessary;
#endif  // TRACING_CHECKS_NECESSARY
}

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_PTRACE_CHECK_H_
