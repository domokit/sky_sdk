// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_TYPEFACE_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_TYPEFACE_H_

#include "impeller/base/comparable.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A typeface, usually obtained from a font-file, on disk describes
///             the intrinsic properties of the font. Typefaces are rarely used
///             directly. Instead, font refer to typefaces along with any
///             modifications applied to its intrinsic properties.
///
class Typeface : public Comparable<Typeface> {
 public:
  Typeface();

  virtual ~Typeface();

  virtual bool IsValid() const = 0;

 private:
  Typeface(const Typeface&) = delete;

  Typeface& operator=(const Typeface&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_TYPEFACE_H_
