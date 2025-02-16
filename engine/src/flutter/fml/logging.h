// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_LOGGING_H_
#define FLUTTER_FML_LOGGING_H_

#include <sstream>

#include "flutter/fml/log_level.h"
#include "flutter/fml/macros.h"

namespace fml {

namespace testing {
struct LogCapture {
  LogCapture();
  ~LogCapture();

  std::string str() const;

 private:
  std::ostringstream stream_;
};
}  // namespace testing

class LogMessageVoidify {
 public:
  void operator&(std::ostream&) {}
};

class LogMessage {
 public:
  LogMessage(LogSeverity severity,
             const char* file,
             int line,
             const char* condition);
  ~LogMessage();

  std::ostream& stream() { return stream_; }

  static void CaptureNextLog(std::ostringstream* stream);

 private:
  // This is a raw pointer so that we avoid having a non-trivially-destructible
  // static. It is only ever for use in unit tests.
  static thread_local std::ostringstream* capture_next_log_stream_;
  std::ostringstream stream_;
  const LogSeverity severity_;
  const char* file_;
  const int line_;

  FML_DISALLOW_COPY_AND_ASSIGN(LogMessage);
};

// Gets the FML_VLOG default verbosity level.
int GetVlogVerbosity();

// Returns true if |severity| is at or above the current minimum log level.
// kLogFatal and above is always true.
bool ShouldCreateLogMessage(LogSeverity severity);

constexpr bool ShouldCreateLogMessageConstexpr(LogSeverity severity,
                                               bool true_arg) {
  if (true_arg) {
    return ShouldCreateLogMessage(severity);
  }
  return false;
}

[[noreturn]] void KillProcess();

[[noreturn]] constexpr void KillProcessConstexpr(bool true_arg) {
  if (true_arg) {
    KillProcess();
  }
#if defined(_MSC_VER) && !defined(__clang__)
  __assume(false);
#else   // defined(_MSC_VER) && !defined(__clang__)
  __builtin_unreachable();
#endif  // defined(_MSC_VER) && !defined(__clang__)
}

}  // namespace fml

#define FML_LOG_STREAM(severity) \
  ::fml::LogMessage(::fml::LOG_##severity, __FILE__, __LINE__, nullptr).stream()

#define FML_LAZY_STREAM(stream, condition) \
  !(condition) ? (void)0 : ::fml::LogMessageVoidify() & (stream)

#define FML_EAT_STREAM_PARAMETERS(ignored) \
  true || (ignored)                        \
      ? (void)0                            \
      : ::fml::LogMessageVoidify() &       \
            ::fml::LogMessage(::fml::kLogFatal, 0, 0, nullptr).stream()

#define FML_LOG_IS_ON(severity) \
  (::fml::ShouldCreateLogMessageConstexpr(::fml::LOG_##severity, true))

#define FML_LOG(severity) \
  FML_LAZY_STREAM(FML_LOG_STREAM(severity), FML_LOG_IS_ON(severity))

#define FML_CHECK(condition)                                              \
  FML_LAZY_STREAM(                                                        \
      ::fml::LogMessage(::fml::kLogFatal, __FILE__, __LINE__, #condition) \
          .stream(),                                                      \
      !(condition))

#define FML_VLOG_IS_ON(verbose_level) \
  ((verbose_level) <= ::fml::GetVlogVerbosity())

// The VLOG macros log with negative verbosities.
#define FML_VLOG_STREAM(verbose_level) \
  ::fml::LogMessage(-verbose_level, __FILE__, __LINE__, nullptr).stream()

#define FML_VLOG(verbose_level) \
  FML_LAZY_STREAM(FML_VLOG_STREAM(verbose_level), FML_VLOG_IS_ON(verbose_level))

#ifndef NDEBUG
#define FML_DLOG(severity) FML_LOG(severity)
#define FML_DCHECK(condition) FML_CHECK(condition)
#else
#define FML_DLOG(severity) FML_EAT_STREAM_PARAMETERS(true)
#define FML_DCHECK(condition) FML_EAT_STREAM_PARAMETERS(condition)
#endif

#define FML_UNREACHABLE()                          \
  {                                                \
    FML_LOG(ERROR) << "Reached unreachable code."; \
    ::fml::KillProcessConstexpr(true);             \
  }

#endif  // FLUTTER_FML_LOGGING_H_
