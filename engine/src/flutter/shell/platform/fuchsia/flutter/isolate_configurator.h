// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ISOLATE_CONFIGURATOR_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ISOLATE_CONFIGURATOR_H_

#include <lib/zx/channel.h>
#include <lib/zx/eventpair.h>
#include "flutter/fml/macros.h"
#include "unique_fdio_ns.h"

namespace flutter_runner {

// Contains all the information necessary to configure a new root isolate. This
// is a single use item. The lifetime of this object must extend past that of
// the root isolate.
class IsolateConfigurator final {
 public:
  IsolateConfigurator(UniqueFDIONS fdio_ns,
                      zx::channel directory_request,
                      zx::eventpair view_ref);

  ~IsolateConfigurator();

  // Can be used only once and only on the UI thread with the newly created
  // isolate already current.
  bool ConfigureCurrentIsolate();

 private:
  bool used_ = false;
  UniqueFDIONS fdio_ns_;
  zx::channel directory_request_;
  zx::eventpair view_ref_;

  void BindFuchsia();

  void BindZircon();

  void BindDartIO();

  FML_DISALLOW_COPY_AND_ASSIGN(IsolateConfigurator);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_ISOLATE_CONFIGURATOR_H_
