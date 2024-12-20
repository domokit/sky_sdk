// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_IO_MANAGER_H_
#define FLUTTER_LIB_UI_IO_MANAGER_H_

#include "flutter/flow/skia_gpu_object.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace impeller {
class Context;
}  // namespace impeller

namespace flutter {
// Interface for methods that manage access to the resource GrDirectContext and
// Skia unref queue.  Meant to be implemented by the owner of the resource
// GrDirectContext, i.e. the shell's IOManager.
class IOManager {
 public:
  virtual ~IOManager() = default;

  virtual fml::WeakPtr<IOManager> GetWeakIOManager() const = 0;

  virtual fml::WeakPtr<GrDirectContext> GetResourceContext() const = 0;

  virtual fml::RefPtr<flutter::SkiaUnrefQueue> GetSkiaUnrefQueue() const = 0;

  virtual std::shared_ptr<const fml::SyncSwitch>
  GetIsGpuDisabledSyncSwitch() = 0;

  virtual std::shared_ptr<impeller::Context> GetImpellerContext() const;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_IO_MANAGER_H_
