// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_GEOMETRY_DL_REGION_H_
#define FLUTTER_DISPLAY_LIST_GEOMETRY_DL_REGION_H_

#include <memory>
#include <vector>

#include "flutter/display_list/geometry/dl_geometry_types.h"

namespace flutter {

/// Represents a region as a collection of non-overlapping rectangles.
/// Implements a subset of SkRegion functionality optimized for quickly
/// converting set of overlapping rectangles to non-overlapping rectangles.
class DlRegion {
 public:
  /// Creates an empty region.
  DlRegion() = default;

  /// Creates region by bulk adding the rectangles.
  /// Matches SkRegion::op(rect, SkRegion::kUnion_Op) behavior.
  explicit DlRegion(const std::vector<DlIRect>& rects);

  /// Creates region covering area of a rectangle.
  explicit DlRegion(const DlIRect& rect);

  DlRegion(const DlRegion&) = default;
  DlRegion(DlRegion&&) = default;

  DlRegion& operator=(const DlRegion&) = default;
  DlRegion& operator=(DlRegion&&) = default;

  /// Creates union region of region a and b.
  /// Matches SkRegion a; a.op(b, SkRegion::kUnion_Op) behavior.
  static DlRegion MakeUnion(const DlRegion& a, const DlRegion& b);

  /// Creates intersection region of region a and b.
  /// Matches SkRegion a; a.op(b, SkRegion::kIntersect_Op) behavior.
  static DlRegion MakeIntersection(const DlRegion& a, const DlRegion& b);

  /// Returns list of non-overlapping rectangles that cover current region.
  /// If |deband| is false, each span line will result in separate rectangles,
  /// closely matching SkRegion::Iterator behavior.
  /// If |deband| is true, matching rectangles from adjacent span lines will be
  /// merged into single rectangle.
  std::vector<DlIRect> getRects(bool deband = true) const;

  /// Returns maximum and minimum axis values of rectangles in this region.
  /// If region is empty returns SKIRect::MakeEmpty().
  const DlIRect& bounds() const { return bounds_; }

  /// Returns whether this region intersects with a rectangle.
  bool intersects(const DlIRect& rect) const;
  bool intersects(const SkIRect& rect) const {
    return intersects(ToDlIRect(rect));
  }

  /// Returns whether this region intersects with another region.
  bool intersects(const DlRegion& region) const;

  /// Returns true if region is empty (contains no rectangles).
  bool isEmpty() const { return lines_.empty(); }

  /// Returns true if region is not empty and contains more than one rectangle.
  bool isComplex() const;

  /// Returns true if region can be represented by single rectangle or is
  /// empty.
  bool isSimple() const { return !isComplex(); }

 private:
  typedef std::uint32_t SpanChunkHandle;

  struct Span {
    int32_t left;
    int32_t right;

    Span() = default;
    Span(int32_t left, int32_t right) : left(left), right(right) {}
  };

  /// Holds spans for the region. Having custom allocated memory that doesn't
  /// do zero initialization every time the buffer gets resized improves
  /// performance measurably.
  class SpanBuffer {
   public:
    SpanBuffer() = default;
    SpanBuffer(const SpanBuffer&);
    SpanBuffer(SpanBuffer&& m);
    SpanBuffer& operator=(const SpanBuffer&);
    SpanBuffer& operator=(SpanBuffer&& m);

    void reserve(size_t capacity);
    size_t capacity() const { return capacity_; }

    SpanChunkHandle storeChunk(const Span* begin, const Span* end);
    size_t getChunkSize(SpanChunkHandle handle) const;
    void getSpans(SpanChunkHandle handle,
                  const DlRegion::Span*& begin,
                  const DlRegion::Span*& end) const;

    ~SpanBuffer();

   private:
    void setChunkSize(SpanChunkHandle handle, size_t size);

    size_t capacity_ = 0;
    size_t size_ = 0;

    // Spans for the region chunks. First span in each chunk contains the
    // chunk size.
    Span* spans_ = nullptr;
  };

  struct SpanLine {
    int32_t top;
    int32_t bottom;
    SpanChunkHandle chunk_handle;
  };

  void setRects(const std::vector<DlIRect>& rects);

  void appendLine(int32_t top,
                  int32_t bottom,
                  const Span* begin,
                  const Span* end);
  void appendLine(int32_t top,
                  int32_t bottom,
                  const SpanBuffer& buffer,
                  SpanChunkHandle handle) {
    const Span *begin, *end;
    buffer.getSpans(handle, begin, end);
    appendLine(top, bottom, begin, end);
  }

  typedef std::vector<Span> SpanVec;
  SpanLine makeLine(int32_t top, int32_t bottom, const SpanVec&);
  SpanLine makeLine(int32_t top,
                    int32_t bottom,
                    const Span* begin,
                    const Span* end);
  static size_t unionLineSpans(std::vector<Span>& res,
                               const SpanBuffer& a_buffer,
                               SpanChunkHandle a_handle,
                               const SpanBuffer& b_buffer,
                               SpanChunkHandle b_handle);
  static size_t intersectLineSpans(std::vector<Span>& res,
                                   const SpanBuffer& a_buffer,
                                   SpanChunkHandle a_handle,
                                   const SpanBuffer& b_buffer,
                                   SpanChunkHandle b_handle);

  bool spansEqual(SpanLine& line, const Span* begin, const Span* end) const;

  static bool spansIntersect(const Span* begin1,
                             const Span* end1,
                             const Span* begin2,
                             const Span* end2);

  static void getIntersectionIterators(
      const std::vector<SpanLine>& a_lines,
      const std::vector<SpanLine>& b_lines,
      std::vector<SpanLine>::const_iterator& a_it,
      std::vector<SpanLine>::const_iterator& b_it);

  std::vector<SpanLine> lines_;
  DlIRect bounds_;
  SpanBuffer span_buffer_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_GEOMETRY_DL_REGION_H_
