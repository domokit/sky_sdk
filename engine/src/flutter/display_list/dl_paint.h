// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_PAINT_H_
#define FLUTTER_DISPLAY_LIST_DL_PAINT_H_

#include <memory>
#include <utility>
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/display_list/effects/dl_mask_filter.h"

namespace flutter {

enum class DlDrawStyle {
  kFill,           //!< fills interior of shapes
  kStroke,         //!< strokes boundary of shapes
  kStrokeAndFill,  //!< both strokes and fills shapes

  kLastStyle = kStrokeAndFill,
  kDefaultStyle = kFill,
};

enum class DlStrokeCap {
  kButt,    //!< no stroke extension
  kRound,   //!< adds circle
  kSquare,  //!< adds square

  kLastCap = kSquare,
  kDefaultCap = kButt,
};

enum class DlStrokeJoin {
  kMiter,  //!< extends to miter limit
  kRound,  //!< adds circle
  kBevel,  //!< connects outside edges

  kLastJoin = kBevel,
  kDefaultJoin = kMiter,
};

class DlPaint {
 public:
  static constexpr DlColor kDefaultColor = DlColor::kBlack();
  static constexpr float kDefaultWidth = 0.0;
  static constexpr float kDefaultMiter = 4.0;

  static const DlPaint kDefault;

  DlPaint() : DlPaint(DlColor::kBlack()) {}
  explicit DlPaint(DlColor color);

  bool isAntiAlias() const { return is_anti_alias_; }
  DlPaint& setAntiAlias(bool isAntiAlias) {
    is_anti_alias_ = isAntiAlias;
    return *this;
  }

  bool isInvertColors() const { return is_invert_colors_; }
  DlPaint& setInvertColors(bool isInvertColors) {
    is_invert_colors_ = isInvertColors;
    return *this;
  }

  DlColor getColor() const { return color_; }
  DlPaint& setColor(DlColor color) {
    color_ = color;
    return *this;
  }

  uint8_t getAlpha() const { return color_.argb() >> 24; }
  DlPaint& setAlpha(uint8_t alpha) { return setColor(color_.withAlpha(alpha)); }
  DlScalar getOpacity() const { return color_.getAlphaF(); }
  DlPaint& setOpacity(DlScalar opacity) {
    setAlpha(SkScalarRoundToInt(opacity * 0xff));
    return *this;
  }

  DlBlendMode getBlendMode() const {
    return static_cast<DlBlendMode>(blend_mode_);
  }
  DlPaint& setBlendMode(DlBlendMode mode) {
    blend_mode_ = static_cast<unsigned>(mode);
    return *this;
  }

  DlDrawStyle getDrawStyle() const {
    return static_cast<DlDrawStyle>(draw_style_);
  }
  DlPaint& setDrawStyle(DlDrawStyle style) {
    draw_style_ = static_cast<unsigned>(style);
    return *this;
  }

  DlStrokeCap getStrokeCap() const {
    return static_cast<DlStrokeCap>(stroke_cap_);
  }
  DlPaint& setStrokeCap(DlStrokeCap cap) {
    stroke_cap_ = static_cast<unsigned>(cap);
    return *this;
  }

  DlStrokeJoin getStrokeJoin() const {
    return static_cast<DlStrokeJoin>(stroke_join_);
  }
  DlPaint& setStrokeJoin(DlStrokeJoin join) {
    stroke_join_ = static_cast<unsigned>(join);
    return *this;
  }

  float getStrokeWidth() const { return stroke_width_; }
  DlPaint& setStrokeWidth(float width) {
    stroke_width_ = width;
    return *this;
  }

  float getStrokeMiter() const { return stroke_miter_; }
  DlPaint& setStrokeMiter(float miter) {
    stroke_miter_ = miter;
    return *this;
  }

  std::shared_ptr<const DlColorSource> getColorSource() const {
    return color_source_;
  }
  const DlColorSource* getColorSourcePtr() const { return color_source_.get(); }
  DlPaint& setColorSource(std::shared_ptr<const DlColorSource> source) {
    color_source_ = std::move(source);
    return *this;
  }
  DlPaint& setColorSource(const DlColorSource* source) {
    color_source_ = source ? source->shared() : nullptr;
    return *this;
  }

  std::shared_ptr<const DlColorFilter> getColorFilter() const {
    return color_filter_;
  }
  const DlColorFilter* getColorFilterPtr() const { return color_filter_.get(); }
  DlPaint& setColorFilter(const std::shared_ptr<const DlColorFilter>& filter) {
    color_filter_ = filter;
    return *this;
  }
  DlPaint& setColorFilter(const DlColorFilter* filter) {
    color_filter_ = filter ? filter->shared() : nullptr;
    return *this;
  }

  std::shared_ptr<DlImageFilter> getImageFilter() const {
    return image_filter_;
  }
  const DlImageFilter* getImageFilterPtr() const { return image_filter_.get(); }
  DlPaint& setImageFilter(const std::shared_ptr<DlImageFilter>& filter) {
    image_filter_ = filter;
    return *this;
  }
  DlPaint& setImageFilter(const DlImageFilter* filter) {
    image_filter_ = filter ? filter->shared() : nullptr;
    return *this;
  }

  std::shared_ptr<const DlMaskFilter> getMaskFilter() const {
    return mask_filter_;
  }
  const DlMaskFilter* getMaskFilterPtr() const { return mask_filter_.get(); }
  DlPaint& setMaskFilter(const std::shared_ptr<DlMaskFilter>& filter) {
    mask_filter_ = filter;
    return *this;
  }
  DlPaint& setMaskFilter(const DlMaskFilter* filter) {
    mask_filter_ = filter ? filter->shared() : nullptr;
    return *this;
  }

  bool isDefault() const { return *this == kDefault; }

  bool usesRuntimeEffect() const {
    return ((color_source_ && color_source_->asRuntimeEffect()) ||
            (image_filter_ && image_filter_->asRuntimeEffectFilter()));
  }

  bool operator==(DlPaint const& other) const;
  bool operator!=(DlPaint const& other) const { return !(*this == other); }

 private:
#define ASSERT_ENUM_FITS(last_enum, num_bits)                    \
  static_assert(static_cast<int>(last_enum) < (1 << num_bits) && \
                static_cast<int>(last_enum) * 2 >= (1 << num_bits))

  static constexpr int kBlendModeBits = 5;
  static constexpr int kDrawStyleBits = 2;
  static constexpr int kStrokeCapBits = 2;
  static constexpr int kStrokeJoinBits = 2;
  ASSERT_ENUM_FITS(DlBlendMode::kLastMode, kBlendModeBits);
  ASSERT_ENUM_FITS(DlDrawStyle::kLastStyle, kDrawStyleBits);
  ASSERT_ENUM_FITS(DlStrokeCap::kLastCap, kStrokeCapBits);
  ASSERT_ENUM_FITS(DlStrokeJoin::kLastJoin, kStrokeJoinBits);

  union {
    struct {
      unsigned blend_mode_ : kBlendModeBits;
      unsigned draw_style_ : kDrawStyleBits;
      unsigned stroke_cap_ : kStrokeCapBits;
      unsigned stroke_join_ : kStrokeJoinBits;
      unsigned is_anti_alias_ : 1;
      unsigned is_invert_colors_ : 1;
    };
  };

  DlColor color_;
  float stroke_width_;
  float stroke_miter_;

  std::shared_ptr<const DlColorSource> color_source_;
  std::shared_ptr<const DlColorFilter> color_filter_;
  std::shared_ptr<DlImageFilter> image_filter_;
  std::shared_ptr<const DlMaskFilter> mask_filter_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_PAINT_H_
