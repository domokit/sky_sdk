// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_surface_software.h"

#include <memory>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/platform/android/jni_weak_ref.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"
#include "flutter/fml/trace_event.h"
#include "flutter/shell/platform/android/android_shell_holder.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace flutter {

namespace {

bool GetSkColorType(int32_t buffer_format,
                    SkColorType* color_type,
                    SkAlphaType* alpha_type) {
  switch (buffer_format) {
    case WINDOW_FORMAT_RGB_565:
      *color_type = kRGB_565_SkColorType;
      *alpha_type = kOpaque_SkAlphaType;
      return true;
    case WINDOW_FORMAT_RGBA_8888:
      *color_type = kRGBA_8888_SkColorType;
      *alpha_type = kPremul_SkAlphaType;
      return true;
    default:
      return false;
  }
}

}  // anonymous namespace

AndroidSurfaceSoftware::AndroidSurfaceSoftware() {
  GetSkColorType(WINDOW_FORMAT_RGBA_8888, &target_color_type_,
                 &target_alpha_type_);
}

AndroidSurfaceSoftware::~AndroidSurfaceSoftware() = default;

bool AndroidSurfaceSoftware::IsValid() const {
  return true;
}

bool AndroidSurfaceSoftware::ResourceContextMakeCurrent() {
  // Resource Context always not available on software backend.
  return false;
}

bool AndroidSurfaceSoftware::ResourceContextClearCurrent() {
  return false;
}

std::unique_ptr<Surface> AndroidSurfaceSoftware::CreateGPUSurface(
    // The software AndroidSurface neither uses any passed in Skia context
    // nor does it interact with the AndroidContext's raster Skia context.
    GrDirectContext* gr_context) {
  if (!IsValid()) {
    return nullptr;
  }

  auto surface =
      std::make_unique<GPUSurfaceSoftware>(this, true /* render to surface */);

  if (!surface->IsValid()) {
    return nullptr;
  }

  return surface;
}

sk_sp<SkSurface> AndroidSurfaceSoftware::AcquireBackingStore(
    const SkISize& size) {
  TRACE_EVENT0("flutter", "AndroidSurfaceSoftware::AcquireBackingStore");
  if (!IsValid()) {
    return nullptr;
  }

  if (sk_surface_ != nullptr &&
      SkISize::Make(sk_surface_->width(), sk_surface_->height()) == size) {
    // The old and new surface sizes are the same. Nothing to do here.
    return sk_surface_;
  }

  SkImageInfo image_info =
      SkImageInfo::Make(size.fWidth, size.fHeight, target_color_type_,
                        target_alpha_type_, SkColorSpace::MakeSRGB());

  sk_surface_ = SkSurfaces::Raster(image_info);

  return sk_surface_;
}

bool AndroidSurfaceSoftware::PresentBackingStore(
    sk_sp<SkSurface> backing_store) {
  TRACE_EVENT0("flutter", "AndroidSurfaceSoftware::PresentBackingStore");
  if (!IsValid() || backing_store == nullptr) {
    return false;
  }

  SkPixmap pixmap;
  if (!backing_store->peekPixels(&pixmap)) {
    return false;
  }

  ANativeWindow_Buffer native_buffer;
  if (ANativeWindow_lock(native_window_->handle(), &native_buffer, nullptr)) {
    return false;
  }

  SkColorType color_type;
  SkAlphaType alpha_type;
  if (GetSkColorType(native_buffer.format, &color_type, &alpha_type)) {
    SkImageInfo native_image_info = SkImageInfo::Make(
        native_buffer.width, native_buffer.height, color_type, alpha_type);

    std::unique_ptr<SkCanvas> canvas = SkCanvas::MakeRasterDirect(
        native_image_info, native_buffer.bits,
        native_buffer.stride * SkColorTypeBytesPerPixel(color_type));

    if (canvas) {
      SkBitmap bitmap;
      if (bitmap.installPixels(pixmap)) {
        canvas->drawImageRect(
            bitmap.asImage(),
            SkRect::MakeIWH(native_buffer.width, native_buffer.height),
            SkSamplingOptions());
      }
    }
  }

  ANativeWindow_unlockAndPost(native_window_->handle());

  return true;
}

void AndroidSurfaceSoftware::TeardownOnScreenContext() {}

bool AndroidSurfaceSoftware::OnScreenSurfaceResize(const SkISize& size) {
  return true;
}

bool AndroidSurfaceSoftware::SetNativeWindow(
    fml::RefPtr<AndroidNativeWindow> window) {
  native_window_ = std::move(window);
  if (!(native_window_ && native_window_->IsValid())) {
    return false;
  }
  int32_t window_format = ANativeWindow_getFormat(native_window_->handle());
  if (window_format < 0) {
    return false;
  }
  if (!GetSkColorType(window_format, &target_color_type_,
                      &target_alpha_type_)) {
    return false;
  }
  return true;
}

}  // namespace flutter
