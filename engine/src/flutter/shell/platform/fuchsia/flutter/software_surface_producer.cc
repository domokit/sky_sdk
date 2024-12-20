// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_surface_producer.h"

#include <lib/fdio/directory.h>
#include <lib/zx/process.h>

#include <algorithm>  // For std::remove_if
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"

namespace flutter_runner {

namespace {

std::string GetCurrentProcessName() {
  char name[ZX_MAX_NAME_LEN];
  zx_status_t status =
      zx::process::self()->get_property(ZX_PROP_NAME, name, sizeof(name));
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to get process name for sysmem; using \"\".";
    return std::string();
  }

  return std::string(name);
}

zx_koid_t GetCurrentProcessId() {
  zx_info_handle_basic_t info;
  zx_status_t status = zx::process::self()->get_info(
      ZX_INFO_HANDLE_BASIC, &info, sizeof(info), /*actual_count*/ nullptr,
      /*avail_count*/ nullptr);
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to get process ID for sysmem; using 0.";
    return ZX_KOID_INVALID;
  }

  return info.koid;
}

}  // namespace

SoftwareSurfaceProducer::SoftwareSurfaceProducer() {
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

  valid_ = true;
}

SoftwareSurfaceProducer::~SoftwareSurfaceProducer() = default;

std::unique_ptr<SurfaceProducerSurface>
SoftwareSurfaceProducer::ProduceOffscreenSurface(const SkISize& size) {
  FML_CHECK(valid_);

  return CreateSurface(size);
}

std::unique_ptr<SurfaceProducerSurface> SoftwareSurfaceProducer::ProduceSurface(
    const SkISize& size) {
  TRACE_EVENT2("flutter", "SoftwareSurfacePool::ProduceSurface", "width",
               size.width(), "height", size.height());
  FML_CHECK(valid_);

  std::unique_ptr<SurfaceProducerSurface> surface;
  auto exact_match_it =
      std::find_if(available_surfaces_.begin(), available_surfaces_.end(),
                   [&size](const auto& surface) {
                     return surface->IsValid() && surface->GetSize() == size;
                   });
  if (exact_match_it != available_surfaces_.end()) {
    TRACE_EVENT_INSTANT0("flutter", "Exact match found");
    surface = std::move(*exact_match_it);
    available_surfaces_.erase(exact_match_it);
  } else {
    surface = CreateSurface(size);
  }

  if (surface == nullptr) {
    FML_LOG(ERROR) << "Could not acquire surface.";
    return nullptr;
  }

  if (!surface->FlushSessionAcquireAndReleaseEvents()) {
    FML_LOG(ERROR) << "Could not flush acquire/release events for buffer.";
    return nullptr;
  }

  return surface;
}

void SoftwareSurfaceProducer::SubmitSurfaces(
    std::vector<std::unique_ptr<SurfaceProducerSurface>> surfaces) {
  TRACE_EVENT0("flutter", "SoftwareSurfaceProducer::SubmitSurfaces");

  // Submit surface
  for (auto& surface : surfaces) {
    SubmitSurface(std::move(surface));
  }

  // Buffer management.
  AgeAndCollectOldBuffers();
}

void SoftwareSurfaceProducer::SubmitSurface(
    std::unique_ptr<SurfaceProducerSurface> surface) {
  TRACE_EVENT0("flutter", "SoftwareSurfacePool::SubmitSurface");
  FML_CHECK(valid_);

  // This cast is safe because |SoftwareSurface| is the only implementation of
  // |SurfaceProducerSurface| for Flutter on Fuchsia.  Additionally, it is
  // required, because we need to access |SoftwareSurface| specific information
  // of the surface (such as the amount of memory it contains).
  auto software_surface = std::unique_ptr<SoftwareSurface>(
      static_cast<SoftwareSurface*>(surface.release()));
  if (!software_surface) {
    return;
  }

  uintptr_t surface_key = reinterpret_cast<uintptr_t>(software_surface.get());
  auto insert_iterator = pending_surfaces_.insert(std::make_pair(
      surface_key,                 // key
      std::move(software_surface)  // value
      ));
  if (insert_iterator.second) {
    insert_iterator.first->second->SignalWritesFinished(std::bind(
        &SoftwareSurfaceProducer::RecyclePendingSurface, this, surface_key));
  }
}

std::unique_ptr<SoftwareSurface> SoftwareSurfaceProducer::CreateSurface(
    const SkISize& size) {
  TRACE_EVENT2("flutter", "SoftwareSurfacePool::CreateSurface", "width",
               size.width(), "height", size.height());
  auto surface = std::make_unique<SoftwareSurface>(sysmem_allocator_,
                                                   flatland_allocator_, size);
  if (!surface->IsValid()) {
    FML_LOG(ERROR) << "Created surface is invalid.";
    return nullptr;
  }
  trace_surfaces_created_++;
  return surface;
}

void SoftwareSurfaceProducer::RecycleSurface(
    std::unique_ptr<SoftwareSurface> surface) {
  // The surface may have become invalid (for example if the fences could
  // not be reset).
  if (!surface->IsValid()) {
    FML_LOG(ERROR) << "Attempted to recycle invalid surface.";
    return;
  }

  TRACE_EVENT0("flutter", "SoftwareSurfacePool::RecycleSurface");
  // Recycle the buffer by putting it in the list of available surfaces if we
  // have not reached the maximum amount of cached surfaces.
  if (available_surfaces_.size() < kMaxSurfaces) {
    available_surfaces_.push_back(std::move(surface));
  } else {
    TRACE_EVENT_INSTANT0("flutter", "Too many surfaces in pool, dropping");
  }
  TraceStats();
}

void SoftwareSurfaceProducer::RecyclePendingSurface(uintptr_t surface_key) {
  // Before we do anything, we must clear the surface from the collection of
  // pending surfaces.
  auto found_in_pending = pending_surfaces_.find(surface_key);
  if (found_in_pending == pending_surfaces_.end()) {
    FML_LOG(ERROR) << "Attempted to recycle a surface that wasn't pending.";
    return;
  }

  // Grab a hold of the surface to recycle and clear the entry in the pending
  // surfaces collection.
  auto surface_to_recycle = std::move(found_in_pending->second);
  pending_surfaces_.erase(found_in_pending);

  RecycleSurface(std::move(surface_to_recycle));
}

void SoftwareSurfaceProducer::AgeAndCollectOldBuffers() {
  TRACE_EVENT0("flutter", "SoftwareSurfacePool::AgeAndCollectOldBuffers");

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

  TraceStats();
}

void SoftwareSurfaceProducer::TraceStats() {
  // Resources held in cached buffers.
  size_t cached_surfaces_bytes = 0;
  for (const auto& surface : available_surfaces_) {
    cached_surfaces_bytes += surface->GetAllocationSize();
  }

  TRACE_COUNTER("flutter", "SurfacePoolCounts", 0u, "CachedCount",
                available_surfaces_.size(),                       //
                "Created", trace_surfaces_created_,               //
                "Reused", trace_surfaces_reused_,                 //
                "PendingInCompositor", pending_surfaces_.size(),  //
                "Retained", 0                                     //
  );

  TRACE_COUNTER("flutter", "SurfacePoolBytes", 0u,     //
                "CachedBytes", cached_surfaces_bytes,  //
                "RetainedBytes", 0                     //
  );

  // Reset per present/frame stats.
  trace_surfaces_created_ = 0;
  trace_surfaces_reused_ = 0;
}

}  // namespace flutter_runner
