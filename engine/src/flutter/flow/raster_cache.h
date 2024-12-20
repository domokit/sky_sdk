// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_RASTER_CACHE_H_
#define FLUTTER_FLOW_RASTER_CACHE_H_

#if !SLIMPELLER

#include <memory>
#include <unordered_map>

#include "flutter/display_list/dl_canvas.h"
#include "flutter/flow/raster_cache_key.h"
#include "flutter/flow/raster_cache_util.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/trace_event.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkRect.h"

class GrDirectContext;
class SkColorSpace;

namespace flutter {

enum class RasterCacheLayerStrategy { kLayer, kLayerChildren };

class RasterCacheResult {
 public:
  RasterCacheResult(sk_sp<DlImage> image,
                    const SkRect& logical_rect,
                    const char* type,
                    sk_sp<const DlRTree> rtree = nullptr);

  virtual ~RasterCacheResult() = default;

  virtual void draw(DlCanvas& canvas,
                    const DlPaint* paint,
                    bool preserve_rtree) const;

  virtual SkISize image_dimensions() const {
    return image_ ? image_->dimensions() : SkISize::Make(0, 0);
  };

  virtual int64_t image_bytes() const {
    return image_ ? image_->GetApproximateByteSize() : 0;
  };

 private:
  sk_sp<DlImage> image_;
  SkRect logical_rect_;
  fml::tracing::TraceFlow flow_;
  sk_sp<const DlRTree> rtree_;
};

class Layer;
class RasterCacheItem;
struct PrerollContext;
struct PaintContext;

struct RasterCacheMetrics {
  /**
   * The number of cache entries with images evicted in this frame.
   */
  size_t eviction_count = 0;

  /**
   * The size of all of the images evicted in this frame.
   */
  size_t eviction_bytes = 0;

  /**
   * The number of cache entries with images used in this frame.
   */
  size_t in_use_count = 0;

  /**
   * The size of all of the images used in this frame.
   */
  size_t in_use_bytes = 0;

  /**
   * The total cache entries that had images during this frame.
   */
  size_t total_count() const { return in_use_count; }

  /**
   * The size of all of the cached images during this frame.
   */
  size_t total_bytes() const { return in_use_bytes; }
};

/**
 * RasterCache is used to cache rasterized layers or display lists to improve
 * performance.
 *
 * Life cycle of RasterCache methods:
 * - Preroll stage
 *   - LayerTree::Preroll - for each Layer in the tree:
 *     - RasterCacheItem::PrerollSetup
 *         At the start of each layer's preroll, add cache items to
 *         `PrerollContext::raster_cached_entries`.
 *     - RasterCacheItem::PrerollFinalize
 *         At the end of each layer's preroll, may mark cache entries as
 *         encountered by the current frame.
 * - Paint stage
 *   - RasterCache::EvictUnusedCacheEntries
 *       Evict cached images that are no longer used.
 *   - LayerTree::TryToPrepareRasterCache
 *       Create cache image for each cache entry if it does not exist.
 *   - LayerTree::Paint - for each layer in the tree:
 *       If layers or display lists are cached as cached images, the method
 *       `RasterCache::Draw` will be used to draw those cache images.
 *   - RasterCache::EndFrame:
 *       Computes used counts and memory then reports cache metrics.
 */
class RasterCache {
 public:
  struct Context {
    GrDirectContext* gr_context;
    const sk_sp<SkColorSpace> dst_color_space;
    const SkMatrix& matrix;
    const SkRect& logical_rect;
    const char* flow_type;
  };
  struct CacheInfo {
    const size_t accesses_since_visible;
    const bool has_image;
  };

  std::unique_ptr<RasterCacheResult> Rasterize(
      const RasterCache::Context& context,
      sk_sp<const DlRTree> rtree,
      const std::function<void(DlCanvas*)>& draw_function,
      const std::function<void(DlCanvas*, const SkRect& rect)>&
          draw_checkerboard) const;

  explicit RasterCache(
      size_t access_threshold = 3,
      size_t picture_and_display_list_cache_limit_per_frame =
          RasterCacheUtil::kDefaultPictureAndDisplayListCacheLimitPerFrame);

  virtual ~RasterCache() = default;

  // Draws this item if it should be rendered from the cache and returns
  // true iff it was successfully drawn. Typically this should only fail
  // if the item was disabled due to conditions discovered during |Preroll|
  // or if the attempt to populate the entry failed due to bounds overflow
  // conditions.
  // If |preserve_rtree| is true, the raster cache will preserve the original
  // RTree of cached content by blitting individual rectangles from the cached
  // image to the canvas according to the original layer R-Tree (if present).
  // This is to ensure that the target surface R-Tree will not be clobbered with
  // one large blit as it can affect platform view overlays and hit testing.
  bool Draw(const RasterCacheKeyID& id,
            DlCanvas& canvas,
            const DlPaint* paint,
            bool preserve_rtree = false) const;

  bool HasEntry(const RasterCacheKeyID& id, const SkMatrix&) const;

  void BeginFrame();

  void EvictUnusedCacheEntries();

  void EndFrame();

  void Clear();

  const RasterCacheMetrics& picture_metrics() const { return picture_metrics_; }
  const RasterCacheMetrics& layer_metrics() const { return layer_metrics_; }

  size_t GetCachedEntriesCount() const;

  /**
   * Return the number of map entries in the layer cache regardless of whether
   * the entries have been populated with an image.
   */
  size_t GetLayerCachedEntriesCount() const;

  /**
   * Return the number of map entries in the picture (DisplayList) cache
   * regardless of whether the entries have been populated with an image.
   */
  size_t GetPictureCachedEntriesCount() const;

  /**
   * @brief Estimate how much memory is used by picture raster cache entries in
   * bytes.
   *
   * Only SkImage's memory usage is counted as other objects are often much
   * smaller compared to SkImage. SkImageInfo::computeMinByteSize is used to
   * estimate the SkImage memory usage.
   */
  size_t EstimatePictureCacheByteSize() const;

  /**
   * @brief Estimate how much memory is used by layer raster cache entries in
   * bytes.
   *
   * Only SkImage's memory usage is counted as other objects are often much
   * smaller compared to SkImage. SkImageInfo::computeMinByteSize is used to
   * estimate the SkImage memory usage.
   */
  size_t EstimateLayerCacheByteSize() const;

  /**
   * @brief Return the number of frames that a picture must be prepared
   * before it will be cached. If the number is 0, then no picture will
   * ever be cached.
   *
   * If the number is one, then it must be prepared and drawn on 1 frame
   * and it will then be cached on the next frame if it is prepared.
   */
  size_t access_threshold() const { return access_threshold_; }

  bool GenerateNewCacheInThisFrame() const {
    // Disabling caching when access_threshold is zero is historic behavior.
    return access_threshold_ != 0 && display_list_cached_this_frame_ <
                                         display_list_cache_limit_per_frame_;
  }

  /**
   * @brief The entry whose RasterCacheKey is generated by RasterCacheKeyID
   * and matrix is marked as encountered by the current frame. The entry
   * will be created if it does not exist. Optionally the entry will be marked
   * as visible in the current frame if the caller determines that it
   * intersects the cull rect. The access_count of the entry will be
   * increased if it is visible, or if it was ever visible.
   * @return the number of times the entry has been hit since it was created.
   * For a new entry that will be 1 if it is visible, or zero if non-visible.
   */
  CacheInfo MarkSeen(const RasterCacheKeyID& id,
                     const SkMatrix& matrix,
                     bool visible) const;

  /**
   * Returns the access count (i.e. accesses_since_visible) for the given
   * entry in the cache, or -1 if no such entry exists.
   */
  int GetAccessCount(const RasterCacheKeyID& id, const SkMatrix& matrix) const;

  bool UpdateCacheEntry(const RasterCacheKeyID& id,
                        const Context& raster_cache_context,
                        const std::function<void(DlCanvas*)>& render_function,
                        sk_sp<const DlRTree> rtree = nullptr) const;

 private:
  struct Entry {
    bool encountered_this_frame = false;
    bool visible_this_frame = false;
    size_t accesses_since_visible = 0;
    std::unique_ptr<RasterCacheResult> image;
  };

  void UpdateMetrics();

  RasterCacheMetrics& GetMetricsForKind(RasterCacheKeyKind kind);

  const size_t access_threshold_;
  const size_t display_list_cache_limit_per_frame_;
  mutable size_t display_list_cached_this_frame_ = 0;
  RasterCacheMetrics layer_metrics_;
  RasterCacheMetrics picture_metrics_;
  mutable RasterCacheKey::Map<Entry> cache_;
  bool checkerboard_images_ = false;

  void TraceStatsToTimeline() const;

  friend class RasterCacheItem;
  friend class LayerRasterCacheItem;

  FML_DISALLOW_COPY_AND_ASSIGN(RasterCache);
};

}  // namespace flutter

#else  //  !SLIMPELLER

class RasterCache;

#endif  //  !SLIMPELLER

#endif  // FLUTTER_FLOW_RASTER_CACHE_H_
