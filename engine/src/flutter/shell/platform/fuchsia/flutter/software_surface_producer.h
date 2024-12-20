// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_SOFTWARE_SURFACE_PRODUCER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_SOFTWARE_SURFACE_PRODUCER_H_

#include <fuchsia/sysmem2/cpp/fidl.h>
#include <fuchsia/ui/composition/cpp/fidl.h>

#include <unordered_map>

#include "flutter/fml/macros.h"

#include "software_surface.h"

namespace flutter_runner {

class SoftwareSurfaceProducer final : public SurfaceProducer {
 public:
  // Only keep 12 surfaces at a time.
  static constexpr int kMaxSurfaces = 12;
  // If a surface doesn't get used for 3 or more generations, we discard it.
  static constexpr int kMaxSurfaceAge = 3;

  explicit SoftwareSurfaceProducer();
  ~SoftwareSurfaceProducer() override;

  bool IsValid() const { return valid_; }

  // |SurfaceProducer|
  GrDirectContext* gr_context() const override { return nullptr; }

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
  void SubmitSurface(std::unique_ptr<SurfaceProducerSurface> surface);
  std::unique_ptr<SoftwareSurface> CreateSurface(const SkISize& size);
  void RecycleSurface(std::unique_ptr<SoftwareSurface> surface);

  void RecyclePendingSurface(uintptr_t surface_key);

  void AgeAndCollectOldBuffers();

  void TraceStats();

  fuchsia::sysmem2::AllocatorSyncPtr sysmem_allocator_;
  fuchsia::ui::composition::AllocatorPtr flatland_allocator_;

  // These surfaces are available for re-use.
  std::vector<std::unique_ptr<SoftwareSurface>> available_surfaces_;
  // These surfaces have been written to, but scenic is not finished reading
  // from them yet.
  std::unordered_map<uintptr_t, std::unique_ptr<SoftwareSurface>>
      pending_surfaces_;

  size_t trace_surfaces_created_ = 0;
  size_t trace_surfaces_reused_ = 0;

  bool valid_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(SoftwareSurfaceProducer);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_SOFTWARE_SURFACE_PRODUCER_H_
