// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_TILE_MODE_H_
#define FLUTTER_DISPLAY_LIST_DL_TILE_MODE_H_

namespace flutter {

// An enum to define how to repeat, fold, or omit colors outside of the
// typically defined range of the source of the colors (such as the
// bounds of an image or the defining geoetry of a gradient).
enum class DlTileMode {
  // Replicate the edge color if the |DlColorSource| draws outside of the
  // defined bounds.
  kClamp,

  // Repeat the |DlColorSource|'s defined colors both horizontally and
  // vertically (or both along and perpendicular to a gradient's geometry).
  kRepeat,

  // Repeat the |DlColorSource|'s colors horizontally and vertically,
  // alternating mirror images so that adjacent images always seam.
  kMirror,

  // Only draw within the original domain, return transparent-black everywhere
  // else.
  kDecal,
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_TILE_MODE_H_
