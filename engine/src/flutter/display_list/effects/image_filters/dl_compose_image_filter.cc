// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/image_filters/dl_compose_image_filter.h"

#include "flutter/display_list/utils/dl_comparable.h"

namespace flutter {

std::shared_ptr<DlImageFilter> DlComposeImageFilter::Make(
    const std::shared_ptr<DlImageFilter>& outer,
    const std::shared_ptr<DlImageFilter>& inner) {
  if (!outer) {
    return inner;
  }
  if (!inner) {
    return outer;
  }
  return std::make_shared<DlComposeImageFilter>(outer, inner);
}

bool DlComposeImageFilter::modifies_transparent_black() const {
  if (inner_ && inner_->modifies_transparent_black()) {
    return true;
  }
  if (outer_ && outer_->modifies_transparent_black()) {
    return true;
  }
  return false;
}

DlRect* DlComposeImageFilter::map_local_bounds(const DlRect& input_bounds,
                                               DlRect& output_bounds) const {
  DlRect cur_bounds = input_bounds;
  DlRect* ret = &output_bounds;
  // We set this result in case neither filter is present.
  output_bounds = input_bounds;
  if (inner_) {
    if (!inner_->map_local_bounds(cur_bounds, output_bounds)) {
      ret = nullptr;
    }
    cur_bounds = output_bounds;
  }
  if (outer_) {
    if (!outer_->map_local_bounds(cur_bounds, output_bounds)) {
      ret = nullptr;
    }
  }
  return ret;
}

DlIRect* DlComposeImageFilter::map_device_bounds(const DlIRect& input_bounds,
                                                 const DlMatrix& ctm,
                                                 DlIRect& output_bounds) const {
  DlIRect cur_bounds = input_bounds;
  DlIRect* ret = &output_bounds;
  // We set this result in case neither filter is present.
  output_bounds = input_bounds;
  if (inner_) {
    if (!inner_->map_device_bounds(cur_bounds, ctm, output_bounds)) {
      ret = nullptr;
    }
    cur_bounds = output_bounds;
  }
  if (outer_) {
    if (!outer_->map_device_bounds(cur_bounds, ctm, output_bounds)) {
      ret = nullptr;
    }
  }
  return ret;
}

DlIRect* DlComposeImageFilter::get_input_device_bounds(
    const DlIRect& output_bounds,
    const DlMatrix& ctm,
    DlIRect& input_bounds) const {
  DlIRect cur_bounds = output_bounds;
  DlIRect* ret = &input_bounds;
  // We set this result in case neither filter is present.
  input_bounds = output_bounds;
  if (outer_) {
    if (!outer_->get_input_device_bounds(cur_bounds, ctm, input_bounds)) {
      ret = nullptr;
    }
    cur_bounds = input_bounds;
  }
  if (inner_) {
    if (!inner_->get_input_device_bounds(cur_bounds, ctm, input_bounds)) {
      ret = nullptr;
    }
  }
  return ret;
}

DlImageFilter::MatrixCapability DlComposeImageFilter::matrix_capability()
    const {
  return std::min(outer_->matrix_capability(), inner_->matrix_capability());
}

bool DlComposeImageFilter::equals_(const DlImageFilter& other) const {
  FML_DCHECK(other.type() == DlImageFilterType::kCompose);
  auto that = static_cast<const DlComposeImageFilter*>(&other);
  return (Equals(outer_, that->outer_) && Equals(inner_, that->inner_));
}

}  // namespace flutter
