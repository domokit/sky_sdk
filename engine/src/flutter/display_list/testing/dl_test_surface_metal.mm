// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/testing/dl_test_surface_metal.h"
#include "flutter/impeller/display_list/dl_dispatcher.h"
#include "flutter/impeller/display_list/dl_image_impeller.h"
#include "flutter/impeller/typographer/backends/skia/typographer_context_skia.h"

#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {
namespace testing {

class DlMetalSurfaceInstance : public DlSurfaceInstance {
 public:
  explicit DlMetalSurfaceInstance(std::unique_ptr<TestMetalSurface> metal_surface)
      : metal_surface_(std::move(metal_surface)) {}
  ~DlMetalSurfaceInstance() = default;

  sk_sp<SkSurface> sk_surface() const override { return metal_surface_->GetSurface(); }

 private:
  std::unique_ptr<TestMetalSurface> metal_surface_;
};

bool DlMetalSurfaceProvider::InitializeSurface(size_t width, size_t height, PixelFormat format) {
  if (format != kN32PremulPixelFormat) {
    return false;
  }
  metal_context_ = std::make_unique<TestMetalContext>();
  metal_surface_ = MakeOffscreenSurface(width, height, format);
  return true;
}

std::shared_ptr<DlSurfaceInstance> DlMetalSurfaceProvider::GetPrimarySurface() const {
  if (!metal_surface_) {
    return nullptr;
  }
  return metal_surface_;
}

std::shared_ptr<DlSurfaceInstance> DlMetalSurfaceProvider::MakeOffscreenSurface(
    size_t width,
    size_t height,
    PixelFormat format) const {
  auto surface = TestMetalSurface::Create(*metal_context_, SkISize::Make(width, height));
  surface->GetSurface()->getCanvas()->clear(SK_ColorTRANSPARENT);
  return std::make_shared<DlMetalSurfaceInstance>(std::move(surface));
}

class DlMetalPixelData : public DlPixelData {
 public:
  explicit DlMetalPixelData(std::unique_ptr<impeller::testing::Screenshot> screenshot)
      : screenshot_(std::move(screenshot)),
        addr_(reinterpret_cast<const uint32_t*>(screenshot_->GetBytes())),
        ints_per_row_(screenshot_->GetBytesPerRow() / 4) {
    FML_DCHECK(screenshot_->GetBytesPerRow() == ints_per_row_ * 4);
  }
  ~DlMetalPixelData() override = default;

  const uint32_t* addr32(int x, int y) const override { return addr_ + (y * ints_per_row_) + x; }
  size_t width() const override { return screenshot_->GetWidth(); }
  size_t height() const override { return screenshot_->GetHeight(); }
  void write(const std::string& path) const override { screenshot_->WriteToPNG(path); }

 private:
  std::unique_ptr<impeller::testing::Screenshot> screenshot_;
  const uint32_t* addr_;
  const uint32_t ints_per_row_;
};

sk_sp<DlPixelData> DlMetalSurfaceProvider::ImpellerSnapshot(const sk_sp<DisplayList>& list,
                                                            int width,
                                                            int height) const {
  auto texture = DisplayListToTexture(list, {width, height}, *aiks_context_);
  return sk_make_sp<DlMetalPixelData>(snapshotter_->MakeScreenshot(*aiks_context_, texture));
}

sk_sp<DlImage> DlMetalSurfaceProvider::MakeImpellerImage(const sk_sp<DisplayList>& list,
                                                         int width,
                                                         int height) const {
  InitScreenShotter();
  return impeller::DlImageImpeller::Make(
      DisplayListToTexture(list, {width, height}, *aiks_context_));
}

void DlMetalSurfaceProvider::InitScreenShotter() const {
  if (!snapshotter_) {
    snapshotter_.reset(new MetalScreenshotter(/*enable_wide_gamut=*/false));
    auto typographer = impeller::TypographerContextSkia::Make();
    aiks_context_.reset(
        new impeller::AiksContext(snapshotter_->GetPlayground().GetContext(), typographer));
  }
}

}  // namespace testing
}  // namespace flutter
