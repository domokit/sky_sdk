// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_FILTER_H_

#include "flutter/display_list/dl_attributes.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/fml/logging.h"

namespace flutter {

class DlBlendColorFilter;
class DlMatrixColorFilter;

// An enumerated type for the supported ColorFilter operations.
enum class DlColorFilterType {
  kBlend,
  kMatrix,
  kSrgbToLinearGamma,
  kLinearToSrgbGamma,
};

/// The DisplayList ColorFilter base class. This class implements all of the
/// facilities and adheres to the design goals of the |DlAttribute| base
/// class.
class DlColorFilter : public DlAttribute<DlColorFilter, DlColorFilterType> {
 public:
  /// Return a shared pointer to a DlColorFilter that acts as if blending
  /// the specified color over the rendered colors using the specified
  /// blend mode, or a nullptr if the operation would be a NOP.
  ///
  /// The blend mode takes the color from the filter as the source color and
  /// the rendered color as the destination color.
  static std::shared_ptr<const DlColorFilter> MakeBlend(DlColor color,
                                                        DlBlendMode mode);

  /// Return a shared pointer to a DlColorFilter which transforms each
  /// rendered color using a per-component equation specified by the
  /// contents of the specified 5 column by 4 row matrix specified in
  /// row major order, or a null pointer if the operation would be a NOP.
  ///
  /// The filter runs every pixel drawn by the rendering operation
  /// [iR,iG,iB,iA] through a vector/matrix  multiplication, as in:
  ///
  ///  [ oR ]   [ m[ 0] m[ 1] m[ 2] m[ 3] m[ 4] ]   [ iR ]
  ///  [ oG ]   [ m[ 5] m[ 6] m[ 7] m[ 8] m[ 9] ]   [ iG ]
  ///  [ oB ] = [ m[10] m[11] m[12] m[13] m[14] ] x [ iB ]
  ///  [ oA ]   [ m[15] m[16] m[17] m[18] m[19] ]   [ iA ]
  ///                                               [  1 ]
  ///
  /// The resulting color [oR,oG,oB,oA] is then clamped to the range of
  /// valid pixel components before storing in the output.
  ///
  /// The incoming and outgoing [iR,iG,iB,iA] and [oR,oG,oB,oA] are
  /// considered to be non-premultiplied. When working on premultiplied
  /// pixel data, the necessary pre<->non-pre conversions must be performed.
  static std::shared_ptr<const DlColorFilter> MakeMatrix(
      const float matrix[20]);

  /// Return a shared pointer to a singleton DlColorFilter that transforms
  /// each rendered pixel from Srgb to Linear gamma space.
  static std::shared_ptr<const DlColorFilter> MakeSrgbToLinearGamma();

  /// Return a shared pointer to a singleton DlColorFilter that transforms
  /// each rendered pixel from Linear to Srgb gamma space.
  static std::shared_ptr<const DlColorFilter> MakeLinearToSrgbGamma();

  // Return a boolean indicating whether the color filtering operation will
  // modify transparent black. This is typically used to determine if applying
  // the ColorFilter to a temporary saveLayer buffer will turn the surrounding
  // pixels non-transparent and therefore expand the bounds.
  virtual bool modifies_transparent_black() const = 0;

  // Return a boolean indicating whether the color filtering operation can
  // be applied either before or after modulating the pixels with an opacity
  // value without changing the operation.
  virtual bool can_commute_with_opacity() const { return false; }

  // Return a DlBlendColorFilter pointer to this object iff it is a Blend
  // type of ColorFilter, otherwise return nullptr.
  virtual const DlBlendColorFilter* asBlend() const { return nullptr; }

  // Return a DlMatrixColorFilter pointer to this object iff it is a Matrix
  // type of ColorFilter, otherwise return nullptr.
  virtual const DlMatrixColorFilter* asMatrix() const { return nullptr; }

  // asSrgb<->Linear are not needed because it has no properties to query.
  // Its type fully specifies its operation.
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_DL_COLOR_FILTER_H_
