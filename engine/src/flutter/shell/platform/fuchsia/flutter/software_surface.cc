// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_surface.h"

#include <lib/async/default.h>
#include <zircon/rights.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <cmath>

#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "fuchsia/sysmem/cpp/fidl.h"
#include "include/core/SkImageInfo.h"

#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkSurface.h"

#include "../runtime/dart/utils/inlines.h"

namespace flutter_runner {

namespace {

constexpr SkColorType kSkiaColorType = kRGBA_8888_SkColorType;

uint32_t BytesPerRow(const fuchsia::sysmem2::SingleBufferSettings& settings,
                     uint32_t bytes_per_pixel,
                     uint32_t image_width) {
  const uint32_t bytes_per_row_divisor =
      settings.image_format_constraints().bytes_per_row_divisor();
  const uint32_t min_bytes_per_row =
      settings.image_format_constraints().min_bytes_per_row();
  const uint32_t unrounded_bytes_per_row =
      std::max(image_width * bytes_per_pixel, min_bytes_per_row);
  const uint32_t roundup_bytes =
      unrounded_bytes_per_row % bytes_per_row_divisor;

  return unrounded_bytes_per_row + roundup_bytes;
}

}  // namespace

SoftwareSurface::SoftwareSurface(
    fuchsia::sysmem2::AllocatorSyncPtr& sysmem_allocator,
    fuchsia::ui::composition::AllocatorPtr& flatland_allocator,
    const SkISize& size)
    : wait_for_surface_read_finished_(this) {
  FML_CHECK(flatland_allocator.is_bound());

  if (!SetupSkiaSurface(sysmem_allocator, flatland_allocator, size)) {
    FML_LOG(ERROR) << "Could not create render surface.";
    return;
  }

  if (!CreateFences()) {
    FML_LOG(ERROR) << "Could not create signal fences.";
    return;
  }

  wait_for_surface_read_finished_.set_object(release_event_.get());
  wait_for_surface_read_finished_.set_trigger(ZX_EVENT_SIGNALED);
  Reset();

  valid_ = true;
}

SoftwareSurface::~SoftwareSurface() {
  release_image_callback_();
  wait_for_surface_read_finished_.Cancel();
  wait_for_surface_read_finished_.set_object(ZX_HANDLE_INVALID);
}

bool SoftwareSurface::IsValid() const {
  return valid_;
}

SkISize SoftwareSurface::GetSize() const {
  if (!valid_) {
    return SkISize::Make(0, 0);
  }

  return SkISize::Make(sk_surface_->width(), sk_surface_->height());
}

bool SoftwareSurface::CreateFences() {
  if (zx::event::create(0, &acquire_event_) != ZX_OK) {
    FML_LOG(ERROR) << "Failed to create acquire event.";
    return false;
  }

  if (zx::event::create(0, &release_event_) != ZX_OK) {
    FML_LOG(ERROR) << "Failed to create release event.";
    return false;
  }

  return true;
}

bool SoftwareSurface::SetupSkiaSurface(
    fuchsia::sysmem2::AllocatorSyncPtr& sysmem_allocator,
    fuchsia::ui::composition::AllocatorPtr& flatland_allocator,
    const SkISize& size) {
  if (size.isEmpty()) {
    FML_LOG(ERROR) << "Failed to allocate surface, size is empty.";
    return false;
  }

  // Allocate a "local" sysmem token to represent flutter's handle to the
  // sysmem buffer.
  fuchsia::sysmem2::BufferCollectionTokenSyncPtr local_token;
  zx_status_t allocate_status = sysmem_allocator->AllocateSharedCollection(
      std::move(fuchsia::sysmem2::AllocatorAllocateSharedCollectionRequest{}
                    .set_token_request(local_token.NewRequest())));
  if (allocate_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to allocate collection: "
                   << zx_status_get_string(allocate_status);
    return false;
  }

  // Create a single Duplicate of the token and Sync it; the single duplicate
  // token represents scenic's handle to the sysmem buffer.
  fuchsia::sysmem2::BufferCollectionToken_DuplicateSync_Result duplicate_result;
  zx_status_t duplicate_status = local_token->DuplicateSync(
      std::move(fuchsia::sysmem2::BufferCollectionTokenDuplicateSyncRequest{}
                    .set_rights_attenuation_masks(
                        std::vector<zx_rights_t>{ZX_RIGHT_SAME_RIGHTS})),
      &duplicate_result);
  if (duplicate_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to duplicate collection token: "
                   << zx_status_get_string(duplicate_status);
    return false;
  }
  auto duplicate_tokens =
      std::move(*duplicate_result.response().mutable_tokens());
  if (duplicate_tokens.size() != 1u) {
    FML_LOG(ERROR) << "Failed to duplicate collection token: Incorrect number "
                      "of tokens returned.";
    return false;
  }
  auto scenic_token = std::move(duplicate_tokens[0]);

  // Register the sysmem token with flatland.
  //
  // This binds the sysmem token to a composition token, which is used later
  // to associate the rendering surface with a specific flatland Image.
  fuchsia::ui::composition::BufferCollectionExportToken export_token;
  zx_status_t token_create_status =
      zx::eventpair::create(0, &export_token.value, &import_token_.value);
  if (token_create_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to create flatland export token: "
                   << zx_status_get_string(token_create_status);
    return false;
  }

  fuchsia::ui::composition::RegisterBufferCollectionArgs args;
  args.set_export_token(std::move(export_token));
  args.set_buffer_collection_token(
      fuchsia::sysmem::BufferCollectionTokenHandle(scenic_token.TakeChannel()));
  args.set_usage(
      fuchsia::ui::composition::RegisterBufferCollectionUsage::DEFAULT);
  flatland_allocator->RegisterBufferCollection(
      std::move(args),
      [](fuchsia::ui::composition::Allocator_RegisterBufferCollection_Result
             result) {
        if (result.is_err()) {
          FML_LOG(ERROR)
              << "RegisterBufferCollection call to Scenic Allocator failed.";
        }
      });

  // Acquire flutter's local handle to the sysmem buffer.
  fuchsia::sysmem2::BufferCollectionSyncPtr buffer_collection;
  zx_status_t bind_status = sysmem_allocator->BindSharedCollection(std::move(
      fuchsia::sysmem2::AllocatorBindSharedCollectionRequest{}
          .set_token(std::move(local_token))
          .set_buffer_collection_request(buffer_collection.NewRequest())));
  if (bind_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to bind collection token: "
                   << zx_status_get_string(bind_status);
    return false;
  }

  // Set flutter's constraints on the sysmem buffer.  Software rendering only
  // requires CPU access to the surface and a basic R8G8B8A8 pixel format.
  fuchsia::sysmem2::BufferCollectionConstraints constraints;
  constraints.set_min_buffer_count(1);
  constraints.mutable_usage()->set_cpu(fuchsia::sysmem2::CPU_USAGE_WRITE |
                                       fuchsia::sysmem2::CPU_USAGE_WRITE_OFTEN);
  auto& bmc = *constraints.mutable_buffer_memory_constraints();
  bmc.set_physically_contiguous_required(false);
  bmc.set_secure_required(false);
  bmc.set_ram_domain_supported(true);
  bmc.set_cpu_domain_supported(true);
  bmc.set_inaccessible_domain_supported(false);
  auto& ifc = constraints.mutable_image_format_constraints()->emplace_back();
  ifc.set_min_size(fuchsia::math::SizeU{static_cast<uint32_t>(size.fWidth),
                                        static_cast<uint32_t>(size.fHeight)});
  ifc.set_min_bytes_per_row(static_cast<uint32_t>(size.fWidth) * 4);
  ifc.set_pixel_format(fuchsia::images2::PixelFormat::R8G8B8A8);
  ifc.mutable_color_spaces()->emplace_back(fuchsia::images2::ColorSpace::SRGB);
  ifc.set_pixel_format_modifier(fuchsia::images2::PixelFormatModifier::LINEAR);
  zx_status_t set_constraints_status = buffer_collection->SetConstraints(
      std::move(fuchsia::sysmem2::BufferCollectionSetConstraintsRequest{}
                    .set_constraints(std::move(constraints))));
  if (set_constraints_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to set constraints: "
                   << zx_status_get_string(set_constraints_status);
    return false;
  }

  // Wait for sysmem to allocate, now that constraints are set.
  fuchsia::sysmem2::BufferCollection_WaitForAllBuffersAllocated_Result
      wait_result;
  zx_status_t wait_for_allocated_status =
      buffer_collection->WaitForAllBuffersAllocated(&wait_result);
  if (wait_for_allocated_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to wait for allocate: "
                   << zx_status_get_string(wait_for_allocated_status);
    return false;
  }
  if (!wait_result.is_response()) {
    if (wait_result.is_framework_err()) {
      FML_LOG(ERROR) << "Failed to allocate (framework_err): "
                     << fidl::ToUnderlying(wait_result.framework_err());
    } else {
      FML_DCHECK(wait_result.is_err());
      FML_LOG(ERROR) << "Failed to allocate (err): "
                     << static_cast<uint32_t>(wait_result.err());
    }
    return false;
  }
  auto buffer_collection_info =
      std::move(*wait_result.response().mutable_buffer_collection_info());

  // Cache the allocated surface VMO and metadata.
  FML_CHECK(buffer_collection_info.settings().buffer_settings().size_bytes() !=
            0);
  FML_CHECK(buffer_collection_info.buffers()[0].vmo().is_valid());
  surface_vmo_ =
      std::move(*buffer_collection_info.mutable_buffers()->at(0).mutable_vmo());
  surface_size_bytes_ =
      buffer_collection_info.settings().buffer_settings().size_bytes();
  if (buffer_collection_info.settings().buffer_settings().coherency_domain() ==
      fuchsia::sysmem2::CoherencyDomain::RAM) {
    // RAM coherency domain requires a cache clean when writes are finished.
    needs_cache_clean_ = true;
  }

  // Map the allocated buffer to the CPU.
  uint8_t* vmo_base = nullptr;
  zx_status_t buffer_map_status = zx::vmar::root_self()->map(
      ZX_VM_PERM_WRITE | ZX_VM_PERM_READ, 0, surface_vmo_, 0,
      surface_size_bytes_, reinterpret_cast<uintptr_t*>(&vmo_base));
  if (buffer_map_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to map buffer memory: "
                   << zx_status_get_string(buffer_map_status);
    return false;
  }

  // Now that the buffer is CPU-readable, it's safe to discard flutter's
  // connection to sysmem.
  zx_status_t close_status = buffer_collection->Release();
  if (close_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to close buffer: "
                   << zx_status_get_string(close_status);
    return false;
  }

  // Wrap the buffer in a software-rendered Skia surface.
  const uint64_t vmo_offset =
      buffer_collection_info.buffers()[0].vmo_usable_start();
  const size_t vmo_stride =
      BytesPerRow(buffer_collection_info.settings(), 4u, size.width());
  SkSurfaceProps sk_surface_props(0, kUnknown_SkPixelGeometry);
  sk_surface_ = SkSurfaces::WrapPixels(
      SkImageInfo::Make(size, kSkiaColorType, kPremul_SkAlphaType,
                        SkColorSpace::MakeSRGB()),
      vmo_base + vmo_offset, vmo_stride, &sk_surface_props);
  if (!sk_surface_ || sk_surface_->getCanvas() == nullptr) {
    FML_LOG(ERROR) << "SkSurfaces::WrapPixels failed.";
    return false;
  }

  return true;
}

void SoftwareSurface::SetImageId(uint32_t image_id) {
  FML_CHECK(image_id_ == 0);
  image_id_ = image_id;
}

uint32_t SoftwareSurface::GetImageId() {
  return image_id_;
}

sk_sp<SkSurface> SoftwareSurface::GetSkiaSurface() const {
  return valid_ ? sk_surface_ : nullptr;
}

fuchsia::ui::composition::BufferCollectionImportToken
SoftwareSurface::GetBufferCollectionImportToken() {
  fuchsia::ui::composition::BufferCollectionImportToken import_dup;
  import_token_.value.duplicate(ZX_RIGHT_SAME_RIGHTS, &import_dup.value);
  return import_dup;
}

zx::event SoftwareSurface::GetAcquireFence() {
  zx::event fence;
  acquire_event_.duplicate(ZX_RIGHT_SAME_RIGHTS, &fence);
  return fence;
}

zx::event SoftwareSurface::GetReleaseFence() {
  zx::event fence;
  release_event_.duplicate(ZX_RIGHT_SAME_RIGHTS, &fence);
  return fence;
}
void SoftwareSurface::SetReleaseImageCallback(
    ReleaseImageCallback release_image_callback) {
  release_image_callback_ = release_image_callback;
}

size_t SoftwareSurface::AdvanceAndGetAge() {
  return ++age_;
}

bool SoftwareSurface::FlushSessionAcquireAndReleaseEvents() {
  age_ = 0;
  return true;
}

void SoftwareSurface::SignalWritesFinished(
    const std::function<void(void)>& on_surface_read_finished) {
  FML_CHECK(on_surface_read_finished);

  if (!valid_) {
    on_surface_read_finished();
    return;
  }

  FML_CHECK(surface_read_finished_callback_ == nullptr)
      << "Attempted to signal a write on the surface when the "
         "previous write has not yet been acknowledged by the "
         "compositor.";
  surface_read_finished_callback_ = on_surface_read_finished;

  // Sysmem *may* require the cache to be cleared after writes to the surface
  // are complete.
  if (needs_cache_clean_) {
    surface_vmo_.op_range(ZX_VMO_OP_CACHE_CLEAN, 0, surface_size_bytes_,
                          /*buffer*/ nullptr,
                          /*buffer_size*/ 0);
  }

  // Inform scenic that flutter is finished writing to the surface.
  zx_status_t signal_status = acquire_event_.signal(0u, ZX_EVENT_SIGNALED);
  if (signal_status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to signal acquire event; "
                   << zx_status_get_string(signal_status);
  }
}

void SoftwareSurface::Reset() {
  if (acquire_event_.signal(ZX_EVENT_SIGNALED, 0u) != ZX_OK ||
      release_event_.signal(ZX_EVENT_SIGNALED, 0u) != ZX_OK) {
    valid_ = false;
    FML_LOG(ERROR) << "Could not reset fences. The surface is no longer valid.";
  }

  wait_for_surface_read_finished_.Begin(async_get_default_dispatcher());

  // It is safe for the caller to collect the surface in the callback.
  auto callback = surface_read_finished_callback_;
  surface_read_finished_callback_ = nullptr;
  if (callback) {
    callback();
  }
}

void SoftwareSurface::OnSurfaceReadFinished(async_dispatcher_t* dispatcher,
                                            async::WaitBase* wait,
                                            zx_status_t status,
                                            const zx_packet_signal_t* signal) {
  if (status != ZX_OK) {
    return;
  }
  FML_DCHECK(signal->observed & ZX_EVENT_SIGNALED);

  Reset();
}

}  // namespace flutter_runner
