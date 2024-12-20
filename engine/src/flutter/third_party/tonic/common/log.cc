// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/common/log.h"

#include <cstdarg>
#include <cstdio>
#include <memory>

namespace tonic {

namespace {

std::function<void(const char*)> log_handler;

}  // namespace

void Log(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  int result = vsnprintf(nullptr, 0, format, ap);
  va_end(ap);

  if (result < 0) {
    return;
  }

  int size = result + 1;
  std::unique_ptr<char[]> message = std::make_unique<char[]>(size);
  va_start(ap, format);
  result = vsnprintf(message.get(), size, format, ap);
  va_end(ap);

  if (result < 0) {
    return;
  }

  if (log_handler) {
    log_handler(message.get());
  } else {
    printf("%s\n", message.get());
  }
}

void SetLogHandler(std::function<void(const char*)> handler) {
  log_handler = handler;
}

}  // namespace tonic
