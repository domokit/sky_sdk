// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FILESYSTEM_EINTR_WRAPPER_H_
#define FILESYSTEM_EINTR_WRAPPER_H_

#include <cerrno>

#include "tonic/common/build_config.h"

#if defined(OS_WIN)

// Windows has no concept of EINTR.
#define HANDLE_EINTR(x) (x)
#define IGNORE_EINTR(x) (x)

#else

#if defined(NDEBUG)

#define HANDLE_EINTR(x)                                     \
  ({                                                        \
    decltype(x) eintr_wrapper_result;                       \
    do {                                                    \
      eintr_wrapper_result = (x);                           \
    } while (eintr_wrapper_result == -1 && errno == EINTR); \
    eintr_wrapper_result;                                   \
  })

#else

#define HANDLE_EINTR(x)                                      \
  ({                                                         \
    int eintr_wrapper_counter = 0;                           \
    decltype(x) eintr_wrapper_result;                        \
    do {                                                     \
      eintr_wrapper_result = (x);                            \
    } while (eintr_wrapper_result == -1 && errno == EINTR && \
             eintr_wrapper_counter++ < 100);                 \
    eintr_wrapper_result;                                    \
  })

#endif  // NDEBUG

#define IGNORE_EINTR(x)                                   \
  ({                                                      \
    decltype(x) eintr_wrapper_result;                     \
    do {                                                  \
      eintr_wrapper_result = (x);                         \
      if (eintr_wrapper_result == -1 && errno == EINTR) { \
        eintr_wrapper_result = 0;                         \
      }                                                   \
    } while (0);                                          \
    eintr_wrapper_result;                                 \
  })

#endif  // defined(OS_WIN)

#endif  // FILESYSTEM_EINTR_WRAPPER_H_
