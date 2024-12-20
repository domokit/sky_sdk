// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/shell_io_manager.h"

#include <utility>

#include "flutter/fml/message_loop.h"
#include "flutter/shell/common/context_options.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLInterface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLTypes.h"

namespace flutter {

sk_sp<GrDirectContext> ShellIOManager::CreateCompatibleResourceLoadingContext(
    GrBackendApi backend,
    const sk_sp<const GrGLInterface>& gl_interface) {
#if SK_GL
  if (backend != GrBackendApi::kOpenGL) {
    return nullptr;
  }

  const auto options = MakeDefaultContextOptions(ContextType::kResource);

  if (auto context = GrDirectContexts::MakeGL(gl_interface, options)) {
    // Do not cache textures created by the image decoder.  These textures
    // should be deleted when they are no longer referenced by an SkImage.
    context->setResourceCacheLimit(0);
    return context;
  }
#endif  // SK_GL

  return nullptr;
}

ShellIOManager::ShellIOManager(
    sk_sp<GrDirectContext> resource_context,
    std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch,
    fml::RefPtr<fml::TaskRunner> unref_queue_task_runner,
    std::shared_ptr<impeller::Context> impeller_context,
    fml::TimeDelta unref_queue_drain_delay)
    : resource_context_(std::move(resource_context)),
      resource_context_weak_factory_(
          resource_context_
              ? std::make_unique<fml::WeakPtrFactory<GrDirectContext>>(
                    resource_context_.get())
              : nullptr),
      unref_queue_(fml::MakeRefCounted<flutter::SkiaUnrefQueue>(
          std::move(unref_queue_task_runner),
          unref_queue_drain_delay,
          resource_context_,
          /*drain_immediate=*/!!impeller_context)),
      is_gpu_disabled_sync_switch_(std::move(is_gpu_disabled_sync_switch)),
      impeller_context_(std::move(impeller_context)),
      weak_factory_(this) {
  if (!resource_context_) {
#ifndef OS_FUCHSIA
    FML_DLOG(WARNING) << "The IO manager was initialized without a resource "
                         "context. Async texture uploads will be disabled. "
                         "Expect performance degradation.";
#endif  // OS_FUCHSIA
  }
}

ShellIOManager::~ShellIOManager() {
  // Last chance to drain the IO queue as the platform side reference to the
  // underlying OpenGL context may be going away.
  is_gpu_disabled_sync_switch_->Execute(
      fml::SyncSwitch::Handlers().SetIfFalse([&] { unref_queue_->Drain(); }));
}

void ShellIOManager::NotifyResourceContextAvailable(
    sk_sp<GrDirectContext> resource_context) {
  // The resource context needs to survive as long as we have Dart objects
  // referencing. We shouldn't ever need to replace it if we have one - unless
  // we've somehow shut down the Dart VM and started a new one fresh.
  if (!resource_context_) {
    UpdateResourceContext(std::move(resource_context));
  }
}

void ShellIOManager::UpdateResourceContext(
    sk_sp<GrDirectContext> resource_context) {
  resource_context_ = std::move(resource_context);
  resource_context_weak_factory_ =
      resource_context_
          ? std::make_unique<fml::WeakPtrFactory<GrDirectContext>>(
                resource_context_.get())
          : nullptr;
  unref_queue_->UpdateResourceContext(resource_context_);
}

fml::WeakPtr<ShellIOManager> ShellIOManager::GetWeakPtr() {
  return weak_factory_.GetWeakPtr();
}

// |IOManager|
fml::WeakPtr<GrDirectContext> ShellIOManager::GetResourceContext() const {
  return resource_context_weak_factory_
             ? resource_context_weak_factory_->GetWeakPtr()
             : fml::WeakPtr<GrDirectContext>();
}

// |IOManager|
fml::RefPtr<flutter::SkiaUnrefQueue> ShellIOManager::GetSkiaUnrefQueue() const {
  return unref_queue_;
}

// |IOManager|
fml::WeakPtr<IOManager> ShellIOManager::GetWeakIOManager() const {
  return weak_factory_.GetWeakPtr();
}

// |IOManager|
std::shared_ptr<const fml::SyncSwitch>
ShellIOManager::GetIsGpuDisabledSyncSwitch() {
  return is_gpu_disabled_sync_switch_;
}

// |IOManager|
std::shared_ptr<impeller::Context> ShellIOManager::GetImpellerContext() const {
  return impeller_context_;
}

}  // namespace flutter
