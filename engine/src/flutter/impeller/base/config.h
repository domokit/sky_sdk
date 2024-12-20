// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_CONFIG_H_
#define FLUTTER_IMPELLER_BASE_CONFIG_H_

#include <cstdlib>

#include "flutter/fml/logging.h"

#if defined(__GNUC__) || defined(__clang__)
#define IMPELLER_COMPILER_CLANG 1
#else  // defined(__GNUC__) || defined(__clang__)
#define IMPELLER_COMPILER_CLANG 0
#endif  // defined(__GNUC__) || defined(__clang__)

#if IMPELLER_COMPILER_CLANG
#define IMPELLER_PRINTF_FORMAT(format_number, args_number) \
  __attribute__((format(printf, format_number, args_number)))
#else  // IMPELLER_COMPILER_CLANG
#define IMPELLER_PRINTF_FORMAT(format_number, args_number)
#endif  // IMPELLER_COMPILER_CLANG

#define IMPELLER_UNIMPLEMENTED \
  impeller::ImpellerUnimplemented(__FUNCTION__, __FILE__, __LINE__);

namespace impeller {

[[noreturn]] inline void ImpellerUnimplemented(const char* method,
                                               const char* file,
                                               int line) {
  FML_CHECK(false) << "Unimplemented: " << method << " in " << file << ":"
                   << line;
  std::abort();
}

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_CONFIG_H_
