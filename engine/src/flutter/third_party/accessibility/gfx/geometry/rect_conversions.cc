// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "rect_conversions.h"

#include <algorithm>
#include <cmath>

#include "base/logging.h"
#include "base/numerics/safe_conversions.h"

namespace gfx {

namespace {

int FloorIgnoringError(float f, float error) {
  int rounded = base::ClampRound(f);
  return std::abs(rounded - f) < error ? rounded : base::ClampFloor(f);
}

int CeilIgnoringError(float f, float error) {
  int rounded = base::ClampRound(f);
  return std::abs(rounded - f) < error ? rounded : base::ClampCeil(f);
}

}  // anonymous namespace

Rect ToEnclosingRect(const RectF& r) {
  int left = base::ClampFloor(r.x());
  int right = r.width() ? base::ClampCeil(r.right()) : left;
  int top = base::ClampFloor(r.y());
  int bottom = r.height() ? base::ClampCeil(r.bottom()) : top;

  Rect result;
  result.SetByBounds(left, top, right, bottom);
  return result;
}

Rect ToEnclosingRectIgnoringError(const RectF& r, float error) {
  int left = FloorIgnoringError(r.x(), error);
  int right = r.width() ? CeilIgnoringError(r.right(), error) : left;
  int top = FloorIgnoringError(r.y(), error);
  int bottom = r.height() ? CeilIgnoringError(r.bottom(), error) : top;

  Rect result;
  result.SetByBounds(left, top, right, bottom);
  return result;
}

Rect ToEnclosedRect(const RectF& rect) {
  Rect result;
  result.SetByBounds(base::ClampCeil(rect.x()), base::ClampCeil(rect.y()),
                     base::ClampFloor(rect.right()),
                     base::ClampFloor(rect.bottom()));
  return result;
}

Rect ToEnclosedRectIgnoringError(const RectF& r, float error) {
  int left = CeilIgnoringError(r.x(), error);
  int right = r.width() ? FloorIgnoringError(r.right(), error) : left;
  int top = CeilIgnoringError(r.y(), error);
  int bottom = r.height() ? FloorIgnoringError(r.bottom(), error) : top;

  Rect result;
  result.SetByBounds(left, top, right, bottom);
  return result;
}

Rect ToNearestRect(const RectF& rect) {
  float float_min_x = rect.x();
  float float_min_y = rect.y();
  float float_max_x = rect.right();
  float float_max_y = rect.bottom();

  int min_x = base::ClampRound(float_min_x);
  int min_y = base::ClampRound(float_min_y);
  int max_x = base::ClampRound(float_max_x);
  int max_y = base::ClampRound(float_max_y);

  // If these DCHECKs fail, you're using the wrong method, consider using
  // ToEnclosingRect or ToEnclosedRect instead.
  BASE_DCHECK(std::abs(min_x - float_min_x) < 0.01f);
  BASE_DCHECK(std::abs(min_y - float_min_y) < 0.01f);
  BASE_DCHECK(std::abs(max_x - float_max_x) < 0.01f);
  BASE_DCHECK(std::abs(max_y - float_max_y) < 0.01f);

  Rect result;
  result.SetByBounds(min_x, min_y, max_x, max_y);

  return result;
}

bool IsNearestRectWithinDistance(const gfx::RectF& rect, float distance) {
  float float_min_x = rect.x();
  float float_min_y = rect.y();
  float float_max_x = rect.right();
  float float_max_y = rect.bottom();

  int min_x = base::ClampRound(float_min_x);
  int min_y = base::ClampRound(float_min_y);
  int max_x = base::ClampRound(float_max_x);
  int max_y = base::ClampRound(float_max_y);

  return (std::abs(min_x - float_min_x) < distance) &&
         (std::abs(min_y - float_min_y) < distance) &&
         (std::abs(max_x - float_max_x) < distance) &&
         (std::abs(max_y - float_max_y) < distance);
}

gfx::Rect ToRoundedRect(const gfx::RectF& rect) {
  int left = base::ClampRound(rect.x());
  int top = base::ClampRound(rect.y());
  int right = base::ClampRound(rect.right());
  int bottom = base::ClampRound(rect.bottom());
  gfx::Rect result;
  result.SetByBounds(left, top, right, bottom);
  return result;
}

Rect ToFlooredRectDeprecated(const RectF& rect) {
  return Rect(base::ClampFloor(rect.x()), base::ClampFloor(rect.y()),
              base::ClampFloor(rect.width()), base::ClampFloor(rect.height()));
}

}  // namespace gfx
