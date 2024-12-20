// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/debugger_detection.h"

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"

#if FML_OS_MACOSX
#include <assert.h>
#include <stdbool.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>
#endif  // FML_OS_MACOSX

#if FML_OS_WIN
#include <windows.h>
#endif  // FML_OS_WIN

namespace flutter::testing {

DebuggerStatus GetDebuggerStatus() {
#if FML_OS_MACOSX
  // From Technical Q&A QA1361 Detecting the Debugger
  // https://developer.apple.com/library/archive/qa/qa1361/_index.html
  int management_info_base[4];
  struct kinfo_proc info;
  size_t size;

  // Initialize the flags so that, if sysctl fails for some bizarre
  // reason, we get a predictable result.
  info.kp_proc.p_flag = 0;

  // Initialize management_info_base, which tells sysctl the info we want, in
  // this case we're looking for information about a specific process ID.
  management_info_base[0] = CTL_KERN;
  management_info_base[1] = KERN_PROC;
  management_info_base[2] = KERN_PROC_PID;
  management_info_base[3] = getpid();

  // Call sysctl.

  size = sizeof(info);
  auto status =
      ::sysctl(management_info_base,
               sizeof(management_info_base) / sizeof(*management_info_base),
               &info, &size, NULL, 0);
  FML_CHECK(status == 0);

  // We're being debugged if the P_TRACED flag is set.
  return ((info.kp_proc.p_flag & P_TRACED) != 0) ? DebuggerStatus::kAttached
                                                 : DebuggerStatus::kDontKnow;

#elif FML_OS_WIN
  return ::IsDebuggerPresent() ? DebuggerStatus::kAttached
                               : DebuggerStatus::kDontKnow;

#else
  return DebuggerStatus::kDontKnow;
#endif
}

}  // namespace flutter::testing
