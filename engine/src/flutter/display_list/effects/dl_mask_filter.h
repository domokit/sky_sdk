// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_DL_MASK_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_DL_MASK_FILTER_H_

#include "flutter/display_list/dl_attributes.h"
#include "flutter/fml/logging.h"

#include "third_party/skia/include/core/SkScalar.h"

namespace flutter {

class DlBlurMaskFilter;

// The DisplayList MaskFilter class. This class implements all of the
// facilities and adheres to the design goals of the |DlAttribute| base
// class.

// An enumerated type for the supported MaskFilter operations.
enum class DlMaskFilterType { kBlur };

enum class DlBlurStyle {
  kNormal,  //!< fuzzy inside and outside
  kSolid,   //!< solid inside, fuzzy outside
  kOuter,   //!< nothing inside, fuzzy outside
  kInner,   //!< fuzzy inside, nothing outside
};

class DlMaskFilter : public DlAttribute<DlMaskFilter, DlMaskFilterType> {
 public:
  // Return a DlBlurMaskFilter pointer to this object iff it is a Blur
  // type of MaskFilter, otherwise return nullptr.
  virtual const DlBlurMaskFilter* asBlur() const { return nullptr; }
};

// The Blur type of MaskFilter which specifies modifying the
// colors as if the color specified in the Blur filter is the
// source color and the color drawn by the rendering operation
// is the destination color. The mode parameter of the Blur
// filter is then used to combine those colors.
class DlBlurMaskFilter final : public DlMaskFilter {
 public:
  DlBlurMaskFilter(DlBlurStyle style, SkScalar sigma, bool respect_ctm = true)
      : style_(style), sigma_(sigma), respect_ctm_(respect_ctm) {}
  DlBlurMaskFilter(const DlBlurMaskFilter& filter)
      : DlBlurMaskFilter(filter.style_, filter.sigma_, filter.respect_ctm_) {}
  explicit DlBlurMaskFilter(const DlBlurMaskFilter* filter)
      : DlBlurMaskFilter(filter->style_, filter->sigma_, filter->respect_ctm_) {
  }

  static std::shared_ptr<DlMaskFilter> Make(DlBlurStyle style,
                                            SkScalar sigma,
                                            bool respect_ctm = true) {
    if (std::isfinite(sigma) && sigma > 0) {
      return std::make_shared<DlBlurMaskFilter>(style, sigma, respect_ctm);
    }
    return nullptr;
  }

  DlMaskFilterType type() const override { return DlMaskFilterType::kBlur; }
  size_t size() const override { return sizeof(*this); }

  std::shared_ptr<DlMaskFilter> shared() const override {
    return std::make_shared<DlBlurMaskFilter>(this);
  }

  const DlBlurMaskFilter* asBlur() const override { return this; }

  DlBlurStyle style() const { return style_; }
  SkScalar sigma() const { return sigma_; }
  bool respectCTM() const { return respect_ctm_; }

 protected:
  bool equals_(DlMaskFilter const& other) const override {
    FML_DCHECK(other.type() == DlMaskFilterType::kBlur);
    auto that = static_cast<DlBlurMaskFilter const*>(&other);
    return style_ == that->style_ && sigma_ == that->sigma_ &&
           respect_ctm_ == that->respect_ctm_;
  }

 private:
  DlBlurStyle style_;
  SkScalar sigma_;
  // Added for backward compatibility with Flutter text shadow rendering which
  // uses Skia blur filters with this flag set to false.
  bool respect_ctm_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_DL_MASK_FILTER_H_
