// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_DISPLAY_LIST_RASTER_CACHE_ITEM_H_
#define FLUTTER_FLOW_LAYERS_DISPLAY_LIST_RASTER_CACHE_ITEM_H_

#if !SLIMPELLER

#include <memory>
#include <optional>

#include "flutter/display_list/display_list.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/raster_cache_item.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkPoint.h"

namespace flutter {

class DisplayListRasterCacheItem : public RasterCacheItem {
 public:
  DisplayListRasterCacheItem(const sk_sp<DisplayList>& display_list,
                             const SkPoint& offset,
                             bool is_complex = true,
                             bool will_change = false);

  static std::unique_ptr<DisplayListRasterCacheItem> Make(
      const sk_sp<DisplayList>&,
      const SkPoint& offset,
      bool is_complex,
      bool will_change);

  void PrerollSetup(PrerollContext* context, const DlMatrix& matrix) override;

  void PrerollFinalize(PrerollContext* context,
                       const DlMatrix& matrix) override;

  bool Draw(const PaintContext& context, const DlPaint* paint) const override;

  bool Draw(const PaintContext& context,
            DlCanvas* canvas,
            const DlPaint* paint) const override;

  bool TryToPrepareRasterCache(const PaintContext& context,
                               bool parent_cached = false) const override;

  void ModifyMatrix(SkPoint offset) const {
    matrix_ = matrix_.preTranslate(offset.x(), offset.y());
  }

  const DisplayList* display_list() const { return display_list_.get(); }

 private:
  SkMatrix transformation_matrix_;
  sk_sp<DisplayList> display_list_;
  SkPoint offset_;
  bool is_complex_;
  bool will_change_;
};

}  // namespace flutter

#else  // !SLIMPELLER

class DisplayListRasterCacheItem;

#endif  // !SLIMPELLER

#endif  // FLUTTER_FLOW_LAYERS_DISPLAY_LIST_RASTER_CACHE_ITEM_H_
