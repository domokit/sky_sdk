// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_SERVICE_ISOLATE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_SERVICE_ISOLATE_H_

#include "third_party/dart/runtime/include/dart_api.h"

namespace dart_runner {

Dart_Isolate CreateServiceIsolate(const char* uri,
                                  Dart_IsolateFlags* flags,
                                  char** error);

Dart_Handle GetVMServiceAssetsArchiveCallback();

}  // namespace dart_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_SERVICE_ISOLATE_H_
