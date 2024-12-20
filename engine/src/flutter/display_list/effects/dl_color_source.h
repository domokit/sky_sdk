// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_SOURCE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_SOURCE_H_

#include "flutter/display_list/dl_attributes.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/fml/logging.h"

namespace flutter {

class DlImageColorSource;
class DlLinearGradientColorSource;
class DlRadialGradientColorSource;
class DlConicalGradientColorSource;
class DlSweepGradientColorSource;
class DlRuntimeEffectColorSource;

// The DisplayList ColorSource class. This class implements all of the
// facilities and adheres to the design goals of the |DlAttribute| base
// class.
//
// The role of the DlColorSource is to provide color information for
// the pixels of a rendering operation. The object is essentially the
// origin of all color being rendered, though its output may be
// modified or transformed by geometric coverage data, the filter
// attributes, and the final blend with the pixels in the destination.

enum class DlColorSourceType {
  kImage,
  kLinearGradient,
  kRadialGradient,
  kConicalGradient,
  kSweepGradient,
  kRuntimeEffect,
};

class DlColorSource : public DlAttribute<DlColorSource, DlColorSourceType> {
 public:
  static std::shared_ptr<DlColorSource> MakeImage(
      const sk_sp<const DlImage>& image,
      DlTileMode horizontal_tile_mode,
      DlTileMode vertical_tile_mode,
      DlImageSampling sampling = DlImageSampling::kLinear,
      const DlMatrix* matrix = nullptr);

  static std::shared_ptr<DlColorSource> MakeLinear(
      const DlPoint start_point,
      const DlPoint end_point,
      uint32_t stop_count,
      const DlColor* colors,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  /// @brief Make a linear gradient.
  /// @param colors_argb Array of DlScalars that represents colors in the ARGB
  /// format, in the extended srgb colorspace.
  static std::shared_ptr<DlColorSource> MakeLinear(
      const DlPoint start_point,
      const DlPoint end_point,
      uint32_t stop_count,
      const DlScalar* colors_argb,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  static std::shared_ptr<DlColorSource> MakeRadial(
      DlPoint center,
      DlScalar radius,
      uint32_t stop_count,
      const DlColor* colors,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  /// @brief Make a radial gradient.
  /// @param colors_argb Array of DlScalars that represents colors in the ARGB
  /// format, in the extended srgb colorspace.
  static std::shared_ptr<DlColorSource> MakeRadial(
      DlPoint center,
      DlScalar radius,
      uint32_t stop_count,
      const DlScalar* colors_argb,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  static std::shared_ptr<DlColorSource> MakeConical(
      DlPoint start_center,
      DlScalar start_radius,
      DlPoint end_center,
      DlScalar end_radius,
      uint32_t stop_count,
      const DlColor* colors,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  /// @brief Make a conical gradient.
  /// @param colors_argb colors_argb Array of DlScalars that represents colors
  /// in the ARGB format, in the extended srgb colorspace.
  static std::shared_ptr<DlColorSource> MakeConical(
      DlPoint start_center,
      DlScalar start_radius,
      DlPoint end_center,
      DlScalar end_radius,
      uint32_t stop_count,
      const DlScalar* colors_argb,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  static std::shared_ptr<DlColorSource> MakeSweep(
      DlPoint center,
      DlScalar start,
      DlScalar end,
      uint32_t stop_count,
      const DlColor* colors,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  /// @brief Make a sweep gradient.
  /// @param colors_argb Array of DlScalars that represents colors in the ARGB
  /// format, in the extended srgb colorspace.
  static std::shared_ptr<DlColorSource> MakeSweep(
      DlPoint center,
      DlScalar start,
      DlScalar end,
      uint32_t stop_count,
      const DlScalar* colors_argb,
      const float* stops,
      DlTileMode tile_mode,
      const DlMatrix* matrix = nullptr);

  static std::shared_ptr<DlColorSource> MakeRuntimeEffect(
      sk_sp<DlRuntimeEffect> runtime_effect,
      std::vector<std::shared_ptr<DlColorSource>> samplers,
      std::shared_ptr<std::vector<uint8_t>> uniform_data);

  virtual bool is_opaque() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      If the underlying platform data held by this object is
  ///             held in a way that it can be stored and potentially
  ///             released from the UI thread, this method returns true.
  ///
  /// @return     True if the class has no GPU related resources or if any
  ///             that it holds are held in a thread-safe manner.
  ///
  virtual bool isUIThreadSafe() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      If the underlying platform data represents a gradient.
  ///
  ///             TODO(matanl): Remove this flag when the Skia backend is
  ///             removed, https://github.com/flutter/flutter/issues/112498.
  ///
  /// @return     True if the class represents the output of a gradient.
  ///
  virtual bool isGradient() const { return false; }

  // Return a DlImageColorSource pointer to this object iff it is an Image
  // type of ColorSource, otherwise return nullptr.
  virtual const DlImageColorSource* asImage() const { return nullptr; }

  // Return a DlLinearGradientColorSource pointer to this object iff it is a
  // Linear Gradient type of ColorSource, otherwise return nullptr.
  virtual const DlLinearGradientColorSource* asLinearGradient() const {
    return nullptr;
  }

  // Return a DlRadialGradientColorSource pointer to this object iff it is a
  // Radial Gradient type of ColorSource, otherwise return nullptr.
  virtual const DlRadialGradientColorSource* asRadialGradient() const {
    return nullptr;
  }

  // Return a DlConicalGradientColorSource pointer to this object iff it is a
  // Conical Gradient type of ColorSource, otherwise return nullptr.
  virtual const DlConicalGradientColorSource* asConicalGradient() const {
    return nullptr;
  }

  // Return a DlSweepGradientColorSource pointer to this object iff it is a
  // Sweep Gradient type of ColorSource, otherwise return nullptr.
  virtual const DlSweepGradientColorSource* asSweepGradient() const {
    return nullptr;
  }

  virtual const DlRuntimeEffectColorSource* asRuntimeEffect() const {
    return nullptr;
  }

 protected:
  DlColorSource() = default;

 private:
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(DlColorSource);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_SOURCE_H_
