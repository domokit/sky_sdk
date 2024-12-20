// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_RECTANGLE_PACKER_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_RECTANGLE_PACKER_H_

#include "flutter/fml/logging.h"
#include "impeller/geometry/scalar.h"

#include <cstdint>
#include <memory>

namespace impeller {

struct IPoint16 {
  int16_t x() const { return x_; }
  int16_t y() const { return y_; }

  int16_t x_;
  int16_t y_;
};

//------------------------------------------------------------------------------
/// @brief      Packs rectangles into a specified area without rotating them.
///
class RectanglePacker {
 public:
  //----------------------------------------------------------------------------
  /// @brief     Return an empty packer with area specified by width and height.
  ///
  static std::shared_ptr<RectanglePacker> Factory(int width, int height);

  virtual ~RectanglePacker() {}

  //----------------------------------------------------------------------------
  /// @brief     Attempt to add a rect without moving already placed rectangles.
  ///
  /// @param[in]   width   The width of the rectangle to add.
  /// @param[in]   height  The height of the rectangle to add.
  /// @param[out]  loc     If successful, will be set to the position of the
  ///                      upper-left corner of the rectangle.
  ///
  /// @return     Return true on success; false on failure.
  ///
  virtual bool AddRect(int width, int height, IPoint16* loc) = 0;

  //----------------------------------------------------------------------------
  /// @brief     Returns how much area has been filled with rectangles.
  ///
  /// @return    Percentage as a decimal between 0.0 and 1.0
  ///
  virtual Scalar PercentFull() const = 0;

  //----------------------------------------------------------------------------
  /// @brief     Empty out all previously added rectangles.
  ///
  virtual void Reset() = 0;

 protected:
  RectanglePacker(int width, int height) : width_(width), height_(height) {
    FML_DCHECK(width >= 0);
    FML_DCHECK(height >= 0);
  }

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  const int width_;
  const int height_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_RECTANGLE_PACKER_H_
