// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_RANGE_RANGE_H_
#define UI_GFX_RANGE_RANGE_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>

#include "ax_build/build_config.h"
#include "gfx_range_export.h"

#if defined(OS_APPLE)
#if __OBJC__
#import <Foundation/Foundation.h>
#else
typedef struct _NSRange NSRange;
#endif
#endif  // defined(OS_APPLE)

namespace gfx {

// This class represents either a forward range [min, max) or a reverse range
// (max, min]. |start_| is always the first of these and |end_| the second; as a
// result, the range is forward if (start_ <= end_).  The zero-width range
// [val, val) is legal, contains and intersects itself, and is contained by and
// intersects any nonempty range [min, max) where min <= val < max.
class GFX_RANGE_EXPORT Range {
 public:
  // Creates an empty range {0,0}.
  constexpr Range() : Range(0) {}

  // Initializes the range with a start and end.
  constexpr Range(uint32_t start, uint32_t end) : start_(start), end_(end) {}

  // Initializes the range with the same start and end positions.
  constexpr explicit Range(uint32_t position) : Range(position, position) {}

  // Platform constructors.
#if defined(OS_APPLE)
  explicit Range(const NSRange& range);
#endif

  // Returns a range that is invalid, which is {UINT32_MAX,UINT32_MAX}.
  static constexpr Range InvalidRange() { return Range(std::numeric_limits<uint32_t>::max()); }

  // Checks if the range is valid through comparison to InvalidRange().
  constexpr bool IsValid() const { return *this != InvalidRange(); }

  // Getters and setters.
  constexpr uint32_t start() const { return start_; }
  void set_start(uint32_t start) { start_ = start; }

  constexpr uint32_t end() const { return end_; }
  void set_end(uint32_t end) { end_ = end; }

  // Returns the absolute value of the length.
  constexpr uint32_t length() const { return GetMax() - GetMin(); }

  constexpr bool is_reversed() const { return start() > end(); }
  constexpr bool is_empty() const { return start() == end(); }

  // Returns the minimum and maximum values.
  constexpr uint32_t GetMin() const { return start() < end() ? start() : end(); }
  constexpr uint32_t GetMax() const { return start() > end() ? start() : end(); }

  constexpr bool operator==(const Range& other) const {
    return start() == other.start() && end() == other.end();
  }
  constexpr bool operator!=(const Range& other) const { return !(*this == other); }
  constexpr bool EqualsIgnoringDirection(const Range& other) const {
    return GetMin() == other.GetMin() && GetMax() == other.GetMax();
  }

  // Returns true if this range intersects the specified |range|.
  constexpr bool Intersects(const Range& range) const { return Intersect(range).IsValid(); }

  // Returns true if this range contains the specified |range|.
  constexpr bool Contains(const Range& range) const {
    return range.IsBoundedBy(*this) &&
           // A non-empty range doesn't contain the range [max, max).
           (range.GetMax() != GetMax() || range.is_empty() == is_empty());
  }

  // Returns true if this range is contained by the specified |range| or it is
  // an empty range and ending the range |range|.
  constexpr bool IsBoundedBy(const Range& range) const {
    return IsValid() && range.IsValid() && GetMin() >= range.GetMin() && GetMax() <= range.GetMax();
  }

  // Computes the intersection of this range with the given |range|.
  // If they don't intersect, it returns an InvalidRange().
  // The returned range is always empty or forward (never reversed).
  constexpr Range Intersect(const Range& range) const {
    const uint32_t min = std::max(GetMin(), range.GetMin());
    const uint32_t max = std::min(GetMax(), range.GetMax());
    return (min < max || Contains(range) || range.Contains(*this)) ? Range(min, max)
                                                                   : InvalidRange();
  }

#if defined(OS_APPLE)
  Range& operator=(const NSRange& range);

  // NSRange does not store the directionality of a range, so if this
  // is_reversed(), the range will get flipped when converted to an NSRange.
  NSRange ToNSRange() const;
#endif
  // GTK+ has no concept of a range.

  std::string ToString() const;

 private:
  // Note: we use uint32_t instead of size_t because this struct is sent over
  // IPC which could span 32 & 64 bit processes. This is fine since text spans
  // shouldn't exceed UINT32_MAX even on 64 bit builds.
  uint32_t start_;
  uint32_t end_;
};

GFX_RANGE_EXPORT std::ostream& operator<<(std::ostream& os, const Range& range);

}  // namespace gfx

#endif  // UI_GFX_RANGE_RANGE_H_
