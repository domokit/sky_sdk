// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vulkan_surface_pool.h"

#include <lib/fdio/directory.h>
#include <lib/zx/process.h>

#include <algorithm>
#include <string>

#include "flutter/fml/trace_event.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter_runner {

static std::string GetCurrentProcessName() {
  char name[ZX_MAX_NAME_LEN];
  zx_status_t status =
      zx::process::self()->get_property(ZX_PROP_NAME, name, sizeof(name));
  return status == ZX_OK ? std::string(name) : std::string();
}

static zx_koid_t GetCurrentProcessId() {
  zx_info_handle_basic_t info;
  zx_status_t status = zx::process::self()->get_info(
      ZX_INFO_HANDLE_BASIC, &info, sizeof(info), nullptr, nullptr);
  return status == ZX_OK ? info.koid : ZX_KOID_INVALID;
}

VulkanSurfacePool::VulkanSurfacePool(vulkan::VulkanProvider& vulkan_provider,
                                     sk_sp<GrDirectContext> context)
    : vulkan_provider_(vulkan_provider), context_(std::move(context)) {
  FML_CHECK(context_ != nullptr);

  zx_status_t status = fdio_service_connect(
      "/svc/fuchsia.sysmem2.Allocator",
      sysmem_allocator_.NewRequest().TakeChannel().release());
  sysmem_allocator_->SetDebugClientInfo(
      std::move(fuchsia::sysmem2::AllocatorSetDebugClientInfoRequest{}
                    .set_name(GetCurrentProcessName())
                    .set_id(GetCurrentProcessId())));
  FML_DCHECK(status == ZX_OK);

  status = fdio_service_connect(
      "/svc/fuchsia.ui.composition.Allocator",
      flatland_allocator_.NewRequest().TakeChannel().release());
  FML_DCHECK(status == ZX_OK);
}

VulkanSurfacePool::~VulkanSurfacePool() {}

std::unique_ptr<VulkanSurface> VulkanSurfacePool::AcquireSurface(
    const SkISize& size) {
  auto surface = GetCachedOrCreateSurface(size);

  if (surface == nullptr) {
    FML_LOG(ERROR) << "VulkanSurfaceProducer: Could not acquire surface";
    return nullptr;
  }

  if (!surface->FlushSessionAcquireAndReleaseEvents()) {
    FML_LOG(ERROR) << "VulkanSurfaceProducer: Could not flush acquire/release "
                      "events for buffer.";
    return nullptr;
  }

  return surface;
}

std::unique_ptr<VulkanSurface> VulkanSurfacePool::GetCachedOrCreateSurface(
    const SkISize& size) {
  TRACE_EVENT2("flutter", "VulkanSurfacePool::GetCachedOrCreateSurface",
               "width", size.width(), "height", size.height());
  // First try to find a surface that exactly matches |size|.
  {
    auto exact_match_it =
        std::find_if(available_surfaces_.begin(), available_surfaces_.end(),
                     [&size](const auto& surface) {
                       return surface->IsValid() && surface->GetSize() == size;
                     });
    if (exact_match_it != available_surfaces_.end()) {
      auto acquired_surface = std::move(*exact_match_it);
      available_surfaces_.erase(exact_match_it);
      TRACE_EVENT_INSTANT0("flutter", "Exact match found");
      return acquired_surface;
    }
  }

  return CreateSurface(size);
}

void VulkanSurfacePool::SubmitSurface(
    std::unique_ptr<SurfaceProducerSurface> p_surface) {
  TRACE_EVENT0("flutter", "VulkanSurfacePool::SubmitSurface");

  // This cast is safe because |VulkanSurface| is the only implementation of
  // |SurfaceProducerSurface| for Flutter on Fuchsia.  Additionally, it is
  // required, because we need to access |VulkanSurface| specific information
  // of the surface (such as the amount of VkDeviceMemory it contains).
  auto vulkan_surface = std::unique_ptr<VulkanSurface>(
      static_cast<VulkanSurface*>(p_surface.release()));
  if (!vulkan_surface) {
    return;
  }

  uintptr_t surface_key = reinterpret_cast<uintptr_t>(vulkan_surface.get());
  auto insert_iterator = pending_surfaces_.insert(std::make_pair(
      surface_key,               // key
      std::move(vulkan_surface)  // value
      ));
  if (insert_iterator.second) {
    insert_iterator.first->second->SignalWritesFinished(std::bind(
        &VulkanSurfacePool::RecyclePendingSurface, this, surface_key));
  }
}

std::unique_ptr<VulkanSurface> VulkanSurfacePool::CreateSurface(
    const SkISize& size) {
  TRACE_EVENT2("flutter", "VulkanSurfacePool::CreateSurface", "width",
               size.width(), "height", size.height());
  auto surface = std::make_unique<VulkanSurface>(
      vulkan_provider_, sysmem_allocator_, flatland_allocator_, context_, size);
  if (!surface->IsValid()) {
    FML_LOG(ERROR) << "VulkanSurfaceProducer: Created surface is invalid";
    return nullptr;
  }
  trace_surfaces_created_++;
  return surface;
}

void VulkanSurfacePool::RecyclePendingSurface(uintptr_t surface_key) {
  // Before we do anything, we must clear the surface from the collection of
  // pending surfaces.
  auto found_in_pending = pending_surfaces_.find(surface_key);
  if (found_in_pending == pending_surfaces_.end()) {
    return;
  }

  // Grab a hold of the surface to recycle and clear the entry in the pending
  // surfaces collection.
  auto surface_to_recycle = std::move(found_in_pending->second);
  pending_surfaces_.erase(found_in_pending);

  RecycleSurface(std::move(surface_to_recycle));
}

void VulkanSurfacePool::RecycleSurface(std::unique_ptr<VulkanSurface> surface) {
  // The surface may have become invalid (for example it the fences could
  // not be reset).
  if (!surface->IsValid()) {
    return;
  }

  TRACE_EVENT0("flutter", "VulkanSurfacePool::RecycleSurface");
  // Recycle the buffer by putting it in the list of available surfaces if we
  // have not reached the maximum amount of cached surfaces.
  if (available_surfaces_.size() < kMaxSurfaces) {
    available_surfaces_.push_back(std::move(surface));
  } else {
    TRACE_EVENT_INSTANT0("flutter", "Too many surfaces in pool, dropping");
  }
  TraceStats();
}

void VulkanSurfacePool::AgeAndCollectOldBuffers() {
  TRACE_EVENT0("flutter", "VulkanSurfacePool::AgeAndCollectOldBuffers");

  // Remove all surfaces that are no longer valid or are too old.
  size_t size_before = available_surfaces_.size();
  available_surfaces_.erase(
      std::remove_if(available_surfaces_.begin(), available_surfaces_.end(),
                     [&](auto& surface) {
                       return !surface->IsValid() ||
                              surface->AdvanceAndGetAge() >= kMaxSurfaceAge;
                     }),
      available_surfaces_.end());
  TRACE_EVENT1("flutter", "AgeAndCollect", "aged surfaces",
               (size_before - available_surfaces_.size()));

  // Look for a surface that has both a larger |VkDeviceMemory| allocation
  // than is necessary for its |VkImage|, and has a stable size history.
  auto surface_to_remove_it = std::find_if(
      available_surfaces_.begin(), available_surfaces_.end(),
      [](const auto& surface) {
        return surface->IsOversized() && surface->HasStableSizeHistory();
      });
  // If we found such a surface, then destroy it and cache a new one that only
  // uses a necessary amount of memory.
  if (surface_to_remove_it != available_surfaces_.end()) {
    TRACE_EVENT_INSTANT0("flutter", "replacing surface with smaller one");
    auto size = (*surface_to_remove_it)->GetSize();
    available_surfaces_.erase(surface_to_remove_it);
    auto new_surface = CreateSurface(size);
    if (new_surface != nullptr) {
      available_surfaces_.push_back(std::move(new_surface));
    } else {
      FML_LOG(ERROR)
          << "VulkanSurfaceProducer: Failed to create a new shrunk surface";
    }
  }

  TraceStats();
}

void VulkanSurfacePool::ShrinkToFit() {
  TRACE_EVENT0("flutter", "VulkanSurfacePool::ShrinkToFit");
  // Reset all oversized surfaces in |available_surfaces_| so that the old
  // surfaces and new surfaces don't exist at the same time at any point,
  // reducing our peak memory footprint.
  std::vector<SkISize> sizes_to_recreate;
  for (auto& surface : available_surfaces_) {
    if (surface->IsOversized()) {
      sizes_to_recreate.push_back(surface->GetSize());
      surface.reset();
    }
  }
  available_surfaces_.erase(std::remove(available_surfaces_.begin(),
                                        available_surfaces_.end(), nullptr),
                            available_surfaces_.end());
  for (const auto& size : sizes_to_recreate) {
    auto surface = CreateSurface(size);
    if (surface != nullptr) {
      available_surfaces_.push_back(std::move(surface));
    } else {
      FML_LOG(ERROR)
          << "VulkanSurfaceProducer: Failed to create resized surface";
    }
  }

  TraceStats();
}

void VulkanSurfacePool::TraceStats() {
  // Resources held in cached buffers.
  size_t cached_surfaces_bytes = 0;
  for (const auto& surface : available_surfaces_) {
    cached_surfaces_bytes += surface->GetAllocationSize();
  }

  // Resources held by Skia.
  int skia_resources = 0;
  size_t skia_bytes = 0;
  context_->getResourceCacheUsage(&skia_resources, &skia_bytes);
  const size_t skia_cache_purgeable =
      context_->getResourceCachePurgeableBytes();

  TRACE_COUNTER("flutter", "SurfacePoolCounts", 0u, "CachedCount",
                available_surfaces_.size(),                       //
                "Created", trace_surfaces_created_,               //
                "Reused", trace_surfaces_reused_,                 //
                "PendingInCompositor", pending_surfaces_.size(),  //
                "Retained", 0,                                    //
                "SkiaCacheResources", skia_resources              //
  );

  TRACE_COUNTER("flutter", "SurfacePoolBytes", 0u,          //
                "CachedBytes", cached_surfaces_bytes,       //
                "RetainedBytes", 0,                         //
                "SkiaCacheBytes", skia_bytes,               //
                "SkiaCachePurgeable", skia_cache_purgeable  //
  );

  // Reset per present/frame stats.
  trace_surfaces_created_ = 0;
  trace_surfaces_reused_ = 0;
}

}  // namespace flutter_runner
