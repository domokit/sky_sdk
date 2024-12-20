// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_UNIQUE_ID_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_UNIQUE_ID_H_

#include <cstdint>

#include "ax/ax_export.h"
#include "base/macros.h"

namespace ui {

// AXUniqueID provides IDs for accessibility objects that are guaranteed to be
// unique for the entire Chrome instance. Instantiating the class is all that
// is required to generate the ID, and the ID is freed when the AXUniqueID is
// destroyed.
//
// The  unique id that's guaranteed to be a positive number. Because some
// platforms want to negate it, we ensure the range is below the signed int max.
//
// These ids must not be conflated with the int id, that comes with web node
// data, which are only unique within their source frame.
class AX_EXPORT AXUniqueId {
 public:
  AXUniqueId();
  virtual ~AXUniqueId();

  int32_t Get() const { return id_; }
  operator int32_t() const { return id_; }

  bool operator==(const AXUniqueId& other) const;
  bool operator!=(const AXUniqueId& other) const;

 protected:
  // Passing the max id is necessary for testing.
  explicit AXUniqueId(const int32_t max_id);

 private:
  int32_t GetNextAXUniqueId(const int32_t max_id);

  bool IsAssigned(int32_t) const;

  int32_t id_;

  BASE_DISALLOW_COPY_AND_ASSIGN(AXUniqueId);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_UNIQUE_ID_H_
