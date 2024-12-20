// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_DL_IMAGE_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_DL_IMAGE_FILTER_H_

#include "flutter/display_list/dl_attributes.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"

namespace flutter {

// The DisplayList ImageFilter class. This class implements all of the
// facilities and adheres to the design goals of the |DlAttribute| base
// class.
//
// The objects here define operations that can take a location and one or
// more input pixels and produce a color for that output pixel

// An enumerated type for the supported ImageFilter operations.
enum class DlImageFilterType {
  kBlur,
  kDilate,
  kErode,
  kMatrix,
  kRuntimeEffect,
  kColorFilter,
  kCompose,
  kLocalMatrix,
};

class DlBlurImageFilter;
class DlDilateImageFilter;
class DlErodeImageFilter;
class DlMatrixImageFilter;
class DlRuntimeEffectImageFilter;
class DlColorFilterImageFilter;
class DlComposeImageFilter;
class DlLocalMatrixImageFilter;

class DlImageFilter : public DlAttribute<DlImageFilter, DlImageFilterType> {
 public:
  enum class MatrixCapability {
    kTranslate,
    kScaleTranslate,
    kComplex,
  };

  static std::shared_ptr<DlImageFilter> MakeBlur(DlScalar sigma_x,
                                                 DlScalar sigma_y,
                                                 DlTileMode tile_mode);

  static std::shared_ptr<DlImageFilter> MakeDilate(DlScalar radius_x,
                                                   DlScalar radius_y);

  static std::shared_ptr<DlImageFilter> MakeErode(DlScalar radius_x,
                                                  DlScalar radius_y);

  static std::shared_ptr<DlImageFilter> MakeMatrix(const DlMatrix& matrix,
                                                   DlImageSampling sampling);

  static std::shared_ptr<DlImageFilter> MakeRuntimeEffect(
      sk_sp<DlRuntimeEffect> runtime_effect,
      std::vector<std::shared_ptr<DlColorSource>> samplers,
      std::shared_ptr<std::vector<uint8_t>> uniform_data);

  static std::shared_ptr<DlImageFilter> MakeColorFilter(
      const std::shared_ptr<const DlColorFilter>& filter);

  static std::shared_ptr<DlImageFilter> MakeCompose(
      const std::shared_ptr<DlImageFilter>& outer,
      const std::shared_ptr<DlImageFilter>& inner);

  // Return a DlBlurImageFilter pointer to this object iff it is a Blur
  // type of ImageFilter, otherwise return nullptr.
  virtual const DlBlurImageFilter* asBlur() const { return nullptr; }

  // Return a DlDilateImageFilter pointer to this object iff it is a Dilate
  // type of ImageFilter, otherwise return nullptr.
  virtual const DlDilateImageFilter* asDilate() const { return nullptr; }

  // Return a DlErodeImageFilter pointer to this object iff it is an Erode
  // type of ImageFilter, otherwise return nullptr.
  virtual const DlErodeImageFilter* asErode() const { return nullptr; }

  // Return a DlMatrixImageFilter pointer to this object iff it is a Matrix
  // type of ImageFilter, otherwise return nullptr.
  virtual const DlMatrixImageFilter* asMatrix() const { return nullptr; }

  virtual const DlLocalMatrixImageFilter* asLocalMatrix() const {
    return nullptr;
  }

  virtual std::shared_ptr<DlImageFilter> makeWithLocalMatrix(
      const DlMatrix& matrix) const;

  // Return a DlComposeImageFilter pointer to this object iff it is a Compose
  // type of ImageFilter, otherwise return nullptr.
  virtual const DlComposeImageFilter* asCompose() const { return nullptr; }

  // Return a DlColorFilterImageFilter pointer to this object iff it is a
  // ColorFilter type of ImageFilter, otherwise return nullptr.
  virtual const DlColorFilterImageFilter* asColorFilter() const {
    return nullptr;
  }

  // Return a DlRuntimeEffectImageFilter pointer to this object iff it is a
  // DlRuntimeEffectImageFilter type of ImageFilter, otherwise return nullptr.
  virtual const DlRuntimeEffectImageFilter* asRuntimeEffectFilter() const {
    return nullptr;
  }

  // Return a boolean indicating whether the image filtering operation will
  // modify transparent black. This is typically used to determine if applying
  // the ImageFilter to a temporary saveLayer buffer will turn the surrounding
  // pixels non-transparent and therefore expand the bounds.
  virtual bool modifies_transparent_black() const = 0;

  // Return the bounds of the output for this image filtering operation
  // based on the supplied input bounds where both are measured in the local
  // (untransformed) coordinate space.
  //
  // The method will return a pointer to the output_bounds parameter if it
  // can successfully compute the output bounds of the filter, otherwise the
  // method will return a nullptr and the output_bounds will be filled with
  // a best guess for the answer, even if just a copy of the input_bounds.
  virtual DlRect* map_local_bounds(const DlRect& input_bounds,
                                   DlRect& output_bounds) const = 0;

  // Return the device bounds of the output for this image filtering operation
  // based on the supplied input device bounds where both are measured in the
  // pixel coordinate space and relative to the given rendering ctm. The
  // transform matrix is used to adjust the filter parameters for when it
  // is used in a rendering operation (for example, the blur radius of a
  // Blur filter will expand based on the ctm).
  //
  // The method will return a pointer to the output_bounds parameter if it
  // can successfully compute the output bounds of the filter, otherwise the
  // method will return a nullptr and the output_bounds will be filled with
  // a best guess for the answer, even if just a copy of the input_bounds.
  virtual DlIRect* map_device_bounds(const DlIRect& input_bounds,
                                     const DlMatrix& ctm,
                                     DlIRect& output_bounds) const = 0;

  // Return the input bounds that will be needed in order for the filter to
  // properly fill the indicated output_bounds under the specified
  // transformation matrix. Both output_bounds and input_bounds are taken to
  // be relative to the transformed coordinate space of the provided |ctm|.
  //
  // The method will return a pointer to the input_bounds parameter if it
  // can successfully compute the required input bounds, otherwise the
  // method will return a nullptr and the input_bounds will be filled with
  // a best guess for the answer, even if just a copy of the output_bounds.
  virtual DlIRect* get_input_device_bounds(const DlIRect& output_bounds,
                                           const DlMatrix& ctm,
                                           DlIRect& input_bounds) const = 0;

  virtual MatrixCapability matrix_capability() const {
    return MatrixCapability::kScaleTranslate;
  }

 protected:
  static DlVector2 map_vectors_affine(const DlMatrix& ctm,
                                      DlScalar x,
                                      DlScalar y);

  static DlIRect* inset_device_bounds(const DlIRect& input_bounds,
                                      DlScalar radius_x,
                                      DlScalar radius_y,
                                      const DlMatrix& ctm,
                                      DlIRect& output_bounds);

  static DlIRect* outset_device_bounds(const DlIRect& input_bounds,
                                       DlScalar radius_x,
                                       DlScalar radius_y,
                                       const DlMatrix& ctm,
                                       DlIRect& output_bounds);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_DL_IMAGE_FILTER_H_
