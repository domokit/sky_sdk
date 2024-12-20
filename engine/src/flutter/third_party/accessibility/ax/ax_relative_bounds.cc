// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_relative_bounds.h"

#include <cmath>

#include "ax_enum_util.h"
#include "base/string_utils.h"

using base::NumberToString;

namespace ui {

AXRelativeBounds::AXRelativeBounds() : offset_container_id(-1) {}

AXRelativeBounds::~AXRelativeBounds() {}

AXRelativeBounds::AXRelativeBounds(const AXRelativeBounds& other) {
  offset_container_id = other.offset_container_id;
  bounds = other.bounds;
  if (other.transform)
    transform = std::make_unique<gfx::Transform>(*other.transform);
}

AXRelativeBounds& AXRelativeBounds::operator=(AXRelativeBounds other) {
  offset_container_id = other.offset_container_id;
  bounds = other.bounds;
  if (other.transform)
    transform = std::make_unique<gfx::Transform>(*other.transform);
  else
    transform.reset(nullptr);
  return *this;
}

bool AXRelativeBounds::operator==(const AXRelativeBounds& other) const {
  if (offset_container_id != other.offset_container_id)
    return false;
  if (bounds != other.bounds)
    return false;
  if (!transform && !other.transform)
    return true;
  if ((transform && !other.transform) || (!transform && other.transform))
    return false;
  return *transform == *other.transform;
}

bool AXRelativeBounds::operator!=(const AXRelativeBounds& other) const {
  return !operator==(other);
}

std::string AXRelativeBounds::ToString() const {
  std::string result;

  if (offset_container_id != -1)
    result += "offset_container_id=" +
              NumberToString(static_cast<int>(round(offset_container_id))) +
              " ";

  result += "(" + NumberToString(static_cast<int>(round(bounds.x()))) + ", " +
            NumberToString(static_cast<int>(round(bounds.y()))) + ")-(" +
            NumberToString(static_cast<int>(round(bounds.width()))) + ", " +
            NumberToString(static_cast<int>(round(bounds.height()))) + ")";

  if (transform && !transform->IsIdentity())
    result += " transform=" + transform->ToString();

  return result;
}

std::ostream& operator<<(std::ostream& stream, const AXRelativeBounds& bounds) {
  return stream << bounds.ToString();
}

}  // namespace ui
