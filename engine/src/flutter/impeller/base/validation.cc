// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/base/validation.h"

#include <atomic>

#include "flutter/fml/logging.h"

namespace impeller {

static std::atomic_int32_t sValidationLogsDisabledCount = 0;
static std::atomic_int32_t sValidationLogsAreFatal = 0;
static ValidationFailureCallback sValidationFailureCallback;

void ImpellerValidationErrorsSetFatal(bool fatal) {
  sValidationLogsAreFatal = fatal;
}

void ImpellerValidationErrorsSetCallback(ValidationFailureCallback callback) {
  sValidationFailureCallback = std::move(callback);
}

ScopedValidationDisable::ScopedValidationDisable() {
  sValidationLogsDisabledCount++;
}

ScopedValidationDisable::~ScopedValidationDisable() {
  sValidationLogsDisabledCount--;
}

ScopedValidationFatal::ScopedValidationFatal() {
  sValidationLogsAreFatal++;
}

ScopedValidationFatal::~ScopedValidationFatal() {
  sValidationLogsAreFatal--;
}

ValidationLog::ValidationLog(const char* file, int line)
    : file_(file), line_(line) {}

ValidationLog::~ValidationLog() {
  if (sValidationLogsDisabledCount <= 0) {
    ImpellerValidationBreak(stream_.str().c_str(), file_, line_);
  }
}

std::ostream& ValidationLog::GetStream() {
  return stream_;
}

void ImpellerValidationBreak(const char* message, const char* file, int line) {
  if (sValidationFailureCallback &&
      sValidationFailureCallback(message, file, line)) {
    return;
  }
  const auto severity =
      ImpellerValidationErrorsAreFatal() ? fml::LOG_FATAL : fml::LOG_ERROR;
  auto fml_log = fml::LogMessage{severity, file, line, nullptr};
  fml_log.stream() <<
#if FLUTTER_RELEASE
      "Impeller validation: " << message;
#else   // FLUTTER_RELEASE
      "Break on '" << __FUNCTION__
                   << "' to inspect point of failure: " << message;
#endif  // FLUTTER_RELEASE
}

bool ImpellerValidationErrorsAreFatal() {
  return sValidationLogsAreFatal;
}

}  // namespace impeller
