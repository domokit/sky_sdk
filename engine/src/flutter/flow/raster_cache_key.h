// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_RASTER_CACHE_KEY_H_
#define FLUTTER_FLOW_RASTER_CACHE_KEY_H_

#if !SLIMPELLER

#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/fml/hash_combine.h"
#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkMatrix.h"

namespace flutter {

class Layer;

enum class RasterCacheKeyType { kLayer, kDisplayList, kLayerChildren };

class RasterCacheKeyID {
 public:
  static constexpr uint64_t kDefaultUniqueID = 0;

  RasterCacheKeyID(uint64_t unique_id, RasterCacheKeyType type)
      : unique_id_(unique_id), type_(type) {}

  RasterCacheKeyID(std::vector<RasterCacheKeyID> child_ids,
                   RasterCacheKeyType type)
      : unique_id_(kDefaultUniqueID),
        type_(type),
        child_ids_(std::move(child_ids)) {}

  uint64_t unique_id() const { return unique_id_; }

  RasterCacheKeyType type() const { return type_; }

  const std::vector<RasterCacheKeyID>& child_ids() const { return child_ids_; }

  static std::optional<std::vector<RasterCacheKeyID>> LayerChildrenIds(
      const Layer* layer);

  std::size_t GetHash() const {
    if (cached_hash_) {
      return cached_hash_.value();
    }
    std::size_t seed = fml::HashCombine();
    fml::HashCombineSeed(seed, unique_id_);
    fml::HashCombineSeed(seed, type_);
    for (auto& child_id : child_ids_) {
      fml::HashCombineSeed(seed, child_id.GetHash());
    }
    cached_hash_ = seed;
    return seed;
  }

  bool operator==(const RasterCacheKeyID& other) const {
    return unique_id_ == other.unique_id_ && type_ == other.type_ &&
           GetHash() == other.GetHash() && child_ids_ == other.child_ids_;
  }

  bool operator!=(const RasterCacheKeyID& other) const {
    return !operator==(other);
  }

 private:
  const uint64_t unique_id_;
  const RasterCacheKeyType type_;
  const std::vector<RasterCacheKeyID> child_ids_;
  mutable std::optional<std::size_t> cached_hash_;
};

enum class RasterCacheKeyKind { kLayerMetrics, kDisplayListMetrics };

class RasterCacheKey {
 public:
  RasterCacheKey(uint64_t unique_id,
                 RasterCacheKeyType type,
                 const SkMatrix& ctm)
      : RasterCacheKey(RasterCacheKeyID(unique_id, type), ctm) {}

  RasterCacheKey(RasterCacheKeyID id, const SkMatrix& ctm)
      : id_(std::move(id)), matrix_(ctm) {
    matrix_[SkMatrix::kMTransX] = 0;
    matrix_[SkMatrix::kMTransY] = 0;
  }

  const RasterCacheKeyID& id() const { return id_; }
  const SkMatrix& matrix() const { return matrix_; }

  RasterCacheKeyKind kind() const {
    switch (id_.type()) {
      case RasterCacheKeyType::kDisplayList:
        return RasterCacheKeyKind::kDisplayListMetrics;
      case RasterCacheKeyType::kLayer:
      case RasterCacheKeyType::kLayerChildren:
        return RasterCacheKeyKind::kLayerMetrics;
    }
  }

  struct Hash {
    std::size_t operator()(RasterCacheKey const& key) const {
      return key.id_.GetHash();
    }
  };

  struct Equal {
    constexpr bool operator()(const RasterCacheKey& lhs,
                              const RasterCacheKey& rhs) const {
      return lhs.id_ == rhs.id_ && lhs.matrix_ == rhs.matrix_;
    }
  };

  template <class Value>
  using Map = std::unordered_map<RasterCacheKey, Value, Hash, Equal>;

 private:
  RasterCacheKeyID id_;

  // ctm where only fractional (0-1) translations are preserved:
  //   matrix_ = ctm;
  //   matrix_[SkMatrix::kMTransX] = SkScalarFraction(ctm.getTranslateX());
  //   matrix_[SkMatrix::kMTransY] = SkScalarFraction(ctm.getTranslateY());
  SkMatrix matrix_;
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_FLOW_RASTER_CACHE_KEY_H_
