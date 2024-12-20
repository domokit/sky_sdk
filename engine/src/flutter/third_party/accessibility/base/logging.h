// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ACCESSIBILITY_BASE_LOGGING_H_
#define ACCESSIBILITY_BASE_LOGGING_H_

#include <sstream>

#include "macros.h"

namespace base {

class LogMessageVoidify {
 public:
  void operator&(std::ostream&) {}
};

class LogMessage {
 public:
  LogMessage(const char* file,
             int line,
             const char* condition,
             bool killProcess);
  ~LogMessage();

  std::ostream& stream() { return stream_; }

 private:
  std::ostringstream stream_;
  const char* file_;
  const int line_;
  const bool killProcess_;

  BASE_DISALLOW_COPY_AND_ASSIGN(LogMessage);
};

[[noreturn]] void KillProcess();

}  // namespace base

#define BASE_LOG_STREAM() \
  ::base::LogMessage(__FILE__, __LINE__, nullptr, false).stream()

#define BASE_LAZY_STREAM(stream, condition) \
  !(condition) ? (void)0 : ::base::LogMessageVoidify() & (stream)

#define BASE_EAT_STREAM_PARAMETERS(ignored) \
  true || (ignored)                         \
      ? (void)0                             \
      : ::base::LogMessageVoidify() &       \
            ::base::LogMessage(0, 0, nullptr, !(ignored)).stream()

#define BASE_LOG() BASE_LAZY_STREAM(BASE_LOG_STREAM(), true)

#define BASE_CHECK(condition)                                            \
  BASE_LAZY_STREAM(                                                      \
      ::base::LogMessage(__FILE__, __LINE__, #condition, true).stream(), \
      !(condition))

#ifndef NDEBUG
#define BASE_DLOG() BASE_LOG()
#define BASE_DCHECK(condition) BASE_CHECK(condition)
#else
#define BASE_DLOG() BASE_EAT_STREAM_PARAMETERS(true)
#define BASE_DCHECK(condition) BASE_EAT_STREAM_PARAMETERS(condition)
#endif

#define BASE_UNREACHABLE()                     \
  {                                            \
    BASE_LOG() << "Reached unreachable code."; \
    ::base::KillProcess();                     \
  }

#endif  // ACCESSIBILITY_BASE_LOGGING_H_
