// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VULKAN_SURFACE_PRODUCER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VULKAN_SURFACE_PRODUCER_H_

#include <lib/async/cpp/time.h>
#include <lib/async/default.h>

#include "flutter/flutter_vma/flutter_skia_vma.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/vulkan/procs/vulkan_proc_table.h"
#include "flutter/vulkan/vulkan_application.h"
#include "flutter/vulkan/vulkan_device.h"
#include "flutter/vulkan/vulkan_provider.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

#include "logging.h"
#include "vulkan_surface.h"
#include "vulkan_surface_pool.h"

namespace flutter_runner {

class VulkanSurfaceProducer final : public SurfaceProducer,
                                    public vulkan::VulkanProvider {
 public:
  explicit VulkanSurfaceProducer();
  ~VulkanSurfaceProducer() override;

  bool IsValid() const { return valid_; }

  // |SurfaceProducer|
  GrDirectContext* gr_context() const override { return context_.get(); }

  // |SurfaceProducer|
  std::unique_ptr<SurfaceProducerSurface> ProduceOffscreenSurface(
      const SkISize& size) override;

  // |SurfaceProducer|
  std::unique_ptr<SurfaceProducerSurface> ProduceSurface(
      const SkISize& size) override;

  // |SurfaceProducer|
  void SubmitSurfaces(
      std::vector<std::unique_ptr<SurfaceProducerSurface>> surfaces) override;

 private:
  // VulkanProvider
  const vulkan::VulkanProcTable& vk() override { return *vk_.get(); }
  const vulkan::VulkanHandle<VkDevice>& vk_device() override {
    return logical_device_->GetHandle();
  }

  bool Initialize();

  void SubmitSurface(std::unique_ptr<SurfaceProducerSurface> surface);
  bool TransitionSurfacesToExternal(
      const std::vector<std::unique_ptr<SurfaceProducerSurface>>& surfaces);

  // Keep track of the last time we produced a surface.  This is used to
  // determine whether it is safe to shrink |surface_pool_| or not.
  zx::time last_produce_time_ = async::Now(async_get_default_dispatcher());

  // Disallow copy and assignment.
  VulkanSurfaceProducer(const VulkanSurfaceProducer&) = delete;
  VulkanSurfaceProducer& operator=(const VulkanSurfaceProducer&) = delete;

  // Note: the order here is very important. The proctable must be destroyed
  // last because it contains the function pointers for VkDestroyDevice and
  // VkDestroyInstance.
  fml::RefPtr<vulkan::VulkanProcTable> vk_;
  std::unique_ptr<vulkan::VulkanApplication> application_;
  std::unique_ptr<vulkan::VulkanDevice> logical_device_;
  sk_sp<GrDirectContext> context_;
  std::unique_ptr<VulkanSurfacePool> surface_pool_;
  sk_sp<skgpu::VulkanMemoryAllocator> memory_allocator_;
  bool valid_ = false;

  // WeakPtrFactory must be the last member.
  fml::WeakPtrFactory<VulkanSurfaceProducer> weak_factory_{this};
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VULKAN_SURFACE_PRODUCER_H_
