// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_color_source.h"

#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/fml/logging.h"

namespace flutter {

static void DlGradientDeleter(void* p) {
  // Some of our target environments would prefer a sized delete,
  // but other target environments do not have that operator.
  // Use an unsized delete until we get better agreement in the
  // environments.
  // See https://github.com/flutter/flutter/issues/100327
  ::operator delete(p);
}

std::shared_ptr<DlColorSource> DlColorSource::MakeImage(
    const sk_sp<const DlImage>& image,
    DlTileMode horizontal_tile_mode,
    DlTileMode vertical_tile_mode,
    DlImageSampling sampling,
    const DlMatrix* matrix) {
  return std::make_shared<DlImageColorSource>(
      image, horizontal_tile_mode, vertical_tile_mode, sampling, matrix);
}

namespace {
template <typename GradientColorSource>
size_t CalculateGradientSize(uint32_t stop_count) {
  return sizeof(GradientColorSource) +
         (stop_count * (sizeof(DlColor) + sizeof(float)));
}
}  // namespace

std::shared_ptr<DlColorSource> DlColorSource::MakeLinear(
    const DlPoint start_point,
    const DlPoint end_point,
    uint32_t stop_count,
    const DlColor* colors,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlLinearGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlLinearGradientColorSource> ret;
  ret.reset(new (storage)
                DlLinearGradientColorSource(start_point, end_point, stop_count,
                                            colors, stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeLinear(
    const DlPoint start_point,
    const DlPoint end_point,
    uint32_t stop_count,
    const DlScalar* colors_argb,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlLinearGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlLinearGradientColorSource> ret;
  ret.reset(new (storage) DlLinearGradientColorSource(start_point, end_point,
                                                      stop_count, colors_argb,
                                                      stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeRadial(
    DlPoint center,
    DlScalar radius,
    uint32_t stop_count,
    const DlColor* colors,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlRadialGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlRadialGradientColorSource> ret;
  ret.reset(new (storage) DlRadialGradientColorSource(
                center, radius, stop_count, colors, stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeRadial(
    DlPoint center,
    DlScalar radius,
    uint32_t stop_count,
    const DlScalar* colors_argb,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlRadialGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlRadialGradientColorSource> ret;
  ret.reset(
      new (storage) DlRadialGradientColorSource(
          center, radius, stop_count, colors_argb, stops, tile_mode, matrix),
      DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeConical(
    DlPoint start_center,
    DlScalar start_radius,
    DlPoint end_center,
    DlScalar end_radius,
    uint32_t stop_count,
    const DlColor* colors,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlConicalGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlConicalGradientColorSource> ret;
  ret.reset(new (storage) DlConicalGradientColorSource(
                start_center, start_radius, end_center, end_radius, stop_count,
                colors, stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeConical(
    DlPoint start_center,
    DlScalar start_radius,
    DlPoint end_center,
    DlScalar end_radius,
    uint32_t stop_count,
    const DlScalar* colors_argb,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed =
      CalculateGradientSize<DlConicalGradientColorSource>(stop_count);

  void* storage = ::operator new(needed);

  std::shared_ptr<DlConicalGradientColorSource> ret;
  ret.reset(new (storage) DlConicalGradientColorSource(
                start_center, start_radius, end_center, end_radius, stop_count,
                colors_argb, stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeSweep(
    DlPoint center,
    DlScalar start,
    DlScalar end,
    uint32_t stop_count,
    const DlColor* colors,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed = CalculateGradientSize<DlSweepGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlSweepGradientColorSource> ret;
  ret.reset(new (storage)
                DlSweepGradientColorSource(center, start, end, stop_count,
                                           colors, stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeSweep(
    DlPoint center,
    DlScalar start,
    DlScalar end,
    uint32_t stop_count,
    const DlScalar* colors_argb,
    const float* stops,
    DlTileMode tile_mode,
    const DlMatrix* matrix) {
  size_t needed = CalculateGradientSize<DlSweepGradientColorSource>(stop_count);
  void* storage = ::operator new(needed);

  std::shared_ptr<DlSweepGradientColorSource> ret;
  ret.reset(new (storage) DlSweepGradientColorSource(center, start, end,
                                                     stop_count, colors_argb,
                                                     stops, tile_mode, matrix),
            DlGradientDeleter);
  return ret;
}

std::shared_ptr<DlColorSource> DlColorSource::MakeRuntimeEffect(
    sk_sp<DlRuntimeEffect> runtime_effect,
    std::vector<std::shared_ptr<DlColorSource>> samplers,
    std::shared_ptr<std::vector<uint8_t>> uniform_data) {
  FML_DCHECK(uniform_data != nullptr);
  return std::make_shared<DlRuntimeEffectColorSource>(
      std::move(runtime_effect), std::move(samplers), std::move(uniform_data));
}

DlGradientColorSourceBase::DlGradientColorSourceBase(uint32_t stop_count,
                                                     DlTileMode tile_mode,
                                                     const DlMatrix* matrix)
    : DlMatrixColorSourceBase(matrix),
      mode_(tile_mode),
      stop_count_(stop_count) {}

bool DlGradientColorSourceBase::is_opaque() const {
  if (mode_ == DlTileMode::kDecal) {
    return false;
  }
  const DlColor* my_colors = colors();
  for (uint32_t i = 0; i < stop_count_; i++) {
    if (my_colors[i].getAlpha() < 255) {
      return false;
    }
  }
  return true;
}

bool DlGradientColorSourceBase::base_equals_(
    DlGradientColorSourceBase const* other_base) const {
  if (mode_ != other_base->mode_ || matrix() != other_base->matrix() ||
      stop_count_ != other_base->stop_count_) {
    return false;
  }
  return (memcmp(colors(), other_base->colors(),
                 stop_count_ * sizeof(colors()[0])) == 0 &&
          memcmp(stops(), other_base->stops(),
                 stop_count_ * sizeof(stops()[0])) == 0);
}

namespace {
template <typename DlColorIt>
void do_store_color_stops(void* pod,
                          DlColorIt color_data_argb_begin,
                          DlColorIt color_data_argb_end,
                          const float* stop_data) {
  DlColor* color_storage = reinterpret_cast<DlColor*>(pod);
  uint32_t stop_count = 0;
  while (color_data_argb_begin < color_data_argb_end) {
    *color_storage++ = *color_data_argb_begin++;
    stop_count += 1;
  }
  float* stop_storage = reinterpret_cast<float*>(color_storage);
  if (stop_data) {
    memcpy(stop_storage, stop_data, stop_count * sizeof(*stop_data));
  } else {
    float div = stop_count - 1;
    if (div <= 0) {
      div = 1;
    }
    for (uint32_t i = 0; i < stop_count; i++) {
      stop_storage[i] = i / div;
    }
  }
}

class DlScalerToDlColorIterator {
 public:
  explicit DlScalerToDlColorIterator(const DlScalar* ptr) : ptr_(ptr) {}
  DlScalerToDlColorIterator(DlScalerToDlColorIterator&&) = default;
  DlScalerToDlColorIterator(const DlScalerToDlColorIterator&) = delete;
  DlScalerToDlColorIterator& operator=(const DlScalerToDlColorIterator&) =
      delete;

  DlColor operator*() {
    return DlColor(ptr_[0], ptr_[1], ptr_[2], ptr_[3],
                   DlColorSpace::kExtendedSRGB);
  }
  DlScalerToDlColorIterator operator++(int) {
    auto result = DlScalerToDlColorIterator(ptr_);
    ptr_ += 4;
    return result;
  }
  bool operator<(const DlScalerToDlColorIterator& that) const {
    return ptr_ < that.ptr_;
  }

 private:
  const DlScalar* ptr_;
};

}  // namespace

void DlGradientColorSourceBase::store_color_stops(void* pod,
                                                  const DlColor* color_data,
                                                  const float* stop_data) {
  do_store_color_stops(pod, color_data, color_data + stop_count_, stop_data);
}

void DlGradientColorSourceBase::store_color_stops(
    void* pod,
    const DlScalar* color_data_argb,
    const float* stop_data) {
  do_store_color_stops(
      pod, DlScalerToDlColorIterator(color_data_argb),
      DlScalerToDlColorIterator(color_data_argb + stop_count_ * 4), stop_data);
}

}  // namespace flutter
