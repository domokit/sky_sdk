// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_BUILTIN_LIBRARIES_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_BUILTIN_LIBRARIES_H_

#include <lib/fdio/namespace.h>
#include <lib/zx/channel.h>

#include <memory>
#include <string>

namespace dart_runner {

void InitBuiltinLibrariesForIsolate(const std::string& script_uri,
                                    fdio_ns_t* namespc,
                                    int stdoutfd,
                                    int stderrfd,
                                    zx::channel directory_request,
                                    bool service_isolate);

}  // namespace dart_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_RUNNER_BUILTIN_LIBRARIES_H_
