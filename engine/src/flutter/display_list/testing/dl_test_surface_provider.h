// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_SURFACE_PROVIDER_H_
#define FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_SURFACE_PROVIDER_H_

#include <utility>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/fml/mapping.h"
#include "flutter/testing/testing.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {
namespace testing {

class DlPixelData : public SkRefCnt {
 public:
  virtual ~DlPixelData() = default;

  virtual const uint32_t* addr32(int x, int y) const = 0;
  virtual size_t width() const = 0;
  virtual size_t height() const = 0;
  virtual void write(const std::string& path) const = 0;
};

class DlSurfaceInstance {
 public:
  virtual ~DlSurfaceInstance() = default;

  virtual sk_sp<SkSurface> sk_surface() const = 0;
  virtual DlCanvas* GetCanvas() = 0;
  void FlushSubmitCpuSync();

  int width() const { return sk_surface()->width(); }
  int height() const { return sk_surface()->height(); }
};

class DlSurfaceInstanceBase : public DlSurfaceInstance {
 public:
  explicit DlSurfaceInstanceBase(sk_sp<SkSurface> surface)
      : surface_(std::move(surface)), adapter_(surface_->getCanvas()) {}
  ~DlSurfaceInstanceBase() = default;

  sk_sp<SkSurface> sk_surface() const override { return surface_; }
  DlCanvas* GetCanvas() override { return &adapter_; }

 private:
  sk_sp<SkSurface> surface_;
  DlSkCanvasAdapter adapter_;
};

class DlSurfaceProvider {
 public:
  typedef enum { kN32PremulPixelFormat, k565PixelFormat } PixelFormat;
  typedef enum { kSoftwareBackend, kOpenGlBackend, kMetalBackend } BackendType;

  static SkImageInfo MakeInfo(PixelFormat format, int w, int h) {
    switch (format) {
      case kN32PremulPixelFormat:
        return SkImageInfo::MakeN32Premul(w, h);
      case k565PixelFormat:
        return SkImageInfo::Make(SkISize::Make(w, h), kRGB_565_SkColorType,
                                 kOpaque_SkAlphaType);
    }
    FML_DCHECK(false);
  }

  static std::string BackendName(BackendType type);
  static std::unique_ptr<DlSurfaceProvider> Create(BackendType backend_type);

  virtual ~DlSurfaceProvider() = default;
  virtual const std::string backend_name() const = 0;
  virtual BackendType backend_type() const = 0;
  virtual bool supports(PixelFormat format) const = 0;
  virtual bool supports_impeller() const { return false; }
  virtual bool InitializeSurface(
      size_t width,
      size_t height,
      PixelFormat format = kN32PremulPixelFormat) = 0;
  virtual std::shared_ptr<DlSurfaceInstance> GetPrimarySurface() const = 0;
  virtual std::shared_ptr<DlSurfaceInstance> MakeOffscreenSurface(
      size_t width,
      size_t height,
      PixelFormat format = kN32PremulPixelFormat) const = 0;

  virtual bool Snapshot(std::string& filename) const;
  virtual sk_sp<DlPixelData> ImpellerSnapshot(const sk_sp<DisplayList>& list,
                                              int width,
                                              int height) const {
    return nullptr;
  }
  virtual sk_sp<DlImage> MakeImpellerImage(const sk_sp<DisplayList>& list,
                                           int width,
                                           int height) const {
    return nullptr;
  }

 protected:
  DlSurfaceProvider() = default;

 private:
  static std::unique_ptr<DlSurfaceProvider> CreateSoftware();
  static std::unique_ptr<DlSurfaceProvider> CreateMetal();
  static std::unique_ptr<DlSurfaceProvider> CreateOpenGL();
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_SURFACE_PROVIDER_H_
