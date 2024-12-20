// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_SHELL_IO_MANAGER_H_
#define FLUTTER_SHELL_COMMON_SHELL_IO_MANAGER_H_

#include <memory>

#include "flutter/flow/skia_gpu_object.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/io_manager.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/GrTypes.h"

struct GrGLInterface;

namespace flutter {

class ShellIOManager final : public IOManager {
 public:
  // Convenience methods for platforms to create a GrDirectContext used to
  // supply to the IOManager. The platforms may create the context themselves if
  // they so desire.
  static sk_sp<GrDirectContext> CreateCompatibleResourceLoadingContext(
      GrBackendApi backend,
      const sk_sp<const GrGLInterface>& gl_interface);

  ShellIOManager(
      sk_sp<GrDirectContext> resource_context,
      std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch,
      fml::RefPtr<fml::TaskRunner> unref_queue_task_runner,
      std::shared_ptr<impeller::Context> impeller_context,
      fml::TimeDelta unref_queue_drain_delay =
          fml::TimeDelta::FromMilliseconds(8));

  ~ShellIOManager() override;

  // This method should be called when a resource_context first becomes
  // available. It is safe to call multiple times, and will only update
  // the held resource context if it has not already been set.
  void NotifyResourceContextAvailable(sk_sp<GrDirectContext> resource_context);

  // This method should be called if you want to force the IOManager to
  // update its resource context reference. It should not be called
  // if there are any Dart objects that have a reference to the old
  // resource context, but may be called if the Dart VM is restarted.
  void UpdateResourceContext(sk_sp<GrDirectContext> resource_context);

  fml::WeakPtr<ShellIOManager> GetWeakPtr();

  // |IOManager|
  fml::WeakPtr<IOManager> GetWeakIOManager() const override;

  // |IOManager|
  fml::WeakPtr<GrDirectContext> GetResourceContext() const override;

  // |IOManager|
  fml::RefPtr<flutter::SkiaUnrefQueue> GetSkiaUnrefQueue() const override;

  // |IOManager|
  std::shared_ptr<const fml::SyncSwitch> GetIsGpuDisabledSyncSwitch() override;

  // |IOManager|
  std::shared_ptr<impeller::Context> GetImpellerContext() const override;

 private:
  // Resource context management.
  sk_sp<GrDirectContext> resource_context_;
  std::unique_ptr<fml::WeakPtrFactory<GrDirectContext>>
      resource_context_weak_factory_;
  // Unref queue management.
  fml::RefPtr<flutter::SkiaUnrefQueue> unref_queue_;
  std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch_;
  std::shared_ptr<impeller::Context> impeller_context_;
  fml::WeakPtrFactory<ShellIOManager> weak_factory_;

  FML_DISALLOW_COPY_AND_ASSIGN(ShellIOManager);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SHELL_IO_MANAGER_H_
