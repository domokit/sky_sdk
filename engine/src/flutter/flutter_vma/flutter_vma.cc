// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifdef VMA_STATIC_VULKAN_FUNCTIONS
#undef VMA_STATIC_VULKAN_FUNCTIONS
#endif  // VMA_STATIC_VULKAN_FUNCTIONS

#ifdef VMA_DYNAMIC_VULKAN_FUNCTIONS
#undef VMA_DYNAMIC_VULKAN_FUNCTIONS
#endif  // VMA_DYNAMIC_VULKAN_FUNCTIONS

// We use our own functions pointers
#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0

#define VMA_IMPLEMENTATION

// Enable this to dump a list of all pending allocations to the log. This comes
// in handy if you are tracking a leak of a resource after context shutdown.
#if 0
#include "flutter/fml/logging.h"  // nogncheck
#define VMA_DEBUG_LOG VMADebugPrint
void VMADebugPrint(const char* message, ...) {
  va_list args;
  va_start(args, message);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer) - 1, message, args);
  va_end(args);
  FML_DLOG(INFO) << buffer;
}
#endif

#include "flutter/fml/logging.h"

#define VMA_ASSERT(expr) \
  FML_DCHECK((expr)) << "Vulkan Memory Allocator Failure!"
#define VMA_HEAVY_ASSERT(expr) \
  FML_DCHECK((expr)) << "Vulkan Memory Allocator Failure!"

#include "flutter/flutter_vma/flutter_vma.h"
