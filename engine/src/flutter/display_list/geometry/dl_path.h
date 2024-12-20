// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_GEOMETRY_DL_PATH_H_
#define FLUTTER_DISPLAY_LIST_GEOMETRY_DL_PATH_H_

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/impeller/geometry/path.h"
#include "flutter/third_party/skia/include/core/SkPath.h"

namespace flutter {

class DlPath {
 public:
  static constexpr uint32_t kMaxVolatileUses = 2;

  static DlPath MakeRect(DlRect rect);
  static DlPath MakeRectLTRB(DlScalar left,
                             DlScalar top,
                             DlScalar right,
                             DlScalar bottom);
  static DlPath MakeRectXYWH(DlScalar x,
                             DlScalar y,
                             DlScalar width,
                             DlScalar height);

  static DlPath MakeOval(DlRect bounds);
  static DlPath MakeOvalLTRB(DlScalar left,
                             DlScalar top,
                             DlScalar right,
                             DlScalar bottom);

  DlPath() : data_(std::make_shared<Data>(SkPath())) {}
  explicit DlPath(const SkPath& path) : data_(std::make_shared<Data>(path)) {}

  DlPath(const DlPath& path) = default;
  DlPath(DlPath&& path) = default;
  DlPath& operator=(const DlPath&) = default;

  const SkPath& GetSkPath() const;
  impeller::Path GetPath() const;

  /// Intent to render an SkPath multiple times will make the path
  /// non-volatile to enable caching in Skia. Calling this method
  /// before every rendering call that uses the SkPath will count
  /// down the uses and eventually reset the volatile flag.
  ///
  /// @see |kMaxVolatileUses|
  void WillRenderSkPath() const;

  bool IsInverseFillType() const;

  bool IsRect(DlRect* rect, bool* is_closed = nullptr) const;
  bool IsOval(DlRect* bounds) const;

  bool IsSkRect(SkRect* rect, bool* is_closed = nullptr) const;
  bool IsSkOval(SkRect* bounds) const;
  bool IsSkRRect(SkRRect* rrect) const;

  SkRect GetSkBounds() const;
  DlRect GetBounds() const;

  bool operator==(const DlPath& other) const;
  bool operator!=(const DlPath& other) const { return !(*this == other); }

  bool IsConverted() const;
  bool IsVolatile() const;

  DlPath operator+(const DlPath& other) const;

 private:
  struct Data {
    explicit Data(const SkPath& path) : sk_path(path) {}

    SkPath sk_path;
    std::optional<impeller::Path> path;
    uint32_t render_count = 0u;
  };

  std::shared_ptr<Data> data_;

  static impeller::Path ConvertToImpellerPath(const SkPath& path,
                                              const DlPoint& shift = DlPoint());
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_GEOMETRY_DL_PATH_H_
