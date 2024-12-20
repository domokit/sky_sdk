// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_region.h"

#include "flutter/fml/logging.h"

namespace flutter {

// Threshold for switching from linear search through span lines to binary
// search.
const int kBinarySearchThreshold = 10;

DlRegion::SpanBuffer::SpanBuffer(DlRegion::SpanBuffer&& m)
    : capacity_(m.capacity_), size_(m.size_), spans_(m.spans_) {
  m.size_ = 0;
  m.capacity_ = 0;
  m.spans_ = nullptr;
};

DlRegion::SpanBuffer::SpanBuffer(const DlRegion::SpanBuffer& m)
    : capacity_(m.capacity_), size_(m.size_) {
  if (m.spans_ == nullptr) {
    spans_ = nullptr;
  } else {
    spans_ = static_cast<Span*>(std::malloc(capacity_ * sizeof(Span)));
    memcpy(spans_, m.spans_, size_ * sizeof(Span));
  }
};

DlRegion::SpanBuffer& DlRegion::SpanBuffer::operator=(
    const DlRegion::SpanBuffer& buffer) {
  SpanBuffer copy(buffer);
  std::swap(*this, copy);
  return *this;
}

DlRegion::SpanBuffer& DlRegion::SpanBuffer::operator=(
    DlRegion::SpanBuffer&& buffer) {
  std::swap(capacity_, buffer.capacity_);
  std::swap(size_, buffer.size_);
  std::swap(spans_, buffer.spans_);
  return *this;
}

DlRegion::SpanBuffer::~SpanBuffer() {
  free(spans_);
}

void DlRegion::SpanBuffer::reserve(size_t capacity) {
  if (capacity_ < capacity) {
    spans_ = static_cast<Span*>(std::realloc(spans_, capacity * sizeof(Span)));
    capacity_ = capacity;
  }
}

DlRegion::SpanChunkHandle DlRegion::SpanBuffer::storeChunk(const Span* begin,
                                                           const Span* end) {
  size_t chunk_size = end - begin;
  size_t min_capacity = size_ + chunk_size + 1;
  if (capacity_ < min_capacity) {
    size_t new_capacity = std::max(min_capacity, capacity_ * 2);
    new_capacity = std::max(new_capacity, static_cast<size_t>(512));
    reserve(new_capacity);
  }
  SpanChunkHandle res = size_;
  size_ += chunk_size + 1;
  setChunkSize(res, chunk_size);

  auto* dst = spans_ + res + 1;
  memmove(dst, begin, chunk_size * sizeof(Span));

  return res;
}

size_t DlRegion::SpanBuffer::getChunkSize(SpanChunkHandle handle) const {
  FML_DCHECK(handle < size_);
  return spans_[handle].left;
}

void DlRegion::SpanBuffer::setChunkSize(SpanChunkHandle handle, size_t size) {
  FML_DCHECK(handle < size_);
  FML_DCHECK(spans_ != nullptr);
  // NOLINTNEXTLINE(clang-analyzer-core.NullDereference)
  spans_[handle].left = size;
}

void DlRegion::SpanBuffer::getSpans(SpanChunkHandle handle,
                                    const DlRegion::Span*& begin,
                                    const DlRegion::Span*& end) const {
  FML_DCHECK(handle < size_);
  begin = spans_ + handle + 1;
  end = begin + getChunkSize(handle);
}

DlRegion::DlRegion(const std::vector<DlIRect>& rects) {
  setRects(rects);
}

DlRegion::DlRegion(const DlIRect& rect) : bounds_(rect) {
  Span span{rect.GetLeft(), rect.GetRight()};
  lines_.push_back(makeLine(rect.GetTop(), rect.GetBottom(), &span, &span + 1));
}

bool DlRegion::spansEqual(SpanLine& line,
                          const Span* begin,
                          const Span* end) const {
  const Span *our_begin, *our_end;
  span_buffer_.getSpans(line.chunk_handle, our_begin, our_end);
  size_t our_size = our_end - our_begin;
  size_t their_size = end - begin;
  if (our_size != their_size) {
    return false;
  }

  return memcmp(our_begin, begin, our_size * sizeof(Span)) == 0;
}

DlRegion::SpanLine DlRegion::makeLine(int32_t top,
                                      int32_t bottom,
                                      const SpanVec& v) {
  return makeLine(top, bottom, v.data(), v.data() + v.size());
}

DlRegion::SpanLine DlRegion::makeLine(int32_t top,
                                      int32_t bottom,
                                      const Span* begin,
                                      const Span* end) {
  auto handle = span_buffer_.storeChunk(begin, end);
  return {top, bottom, handle};
}

// Returns number of valid spans in res. For performance reasons res is never
// downsized.
size_t DlRegion::unionLineSpans(std::vector<Span>& res,
                                const SpanBuffer& a_buffer,
                                SpanChunkHandle a_handle,
                                const SpanBuffer& b_buffer,
                                SpanChunkHandle b_handle) {
  class OrderedSpanAccumulator {
   public:
    explicit OrderedSpanAccumulator(std::vector<Span>& res) : res(res) {}

    void accumulate(const Span& span) {
      if (span.left > last_ || len == 0) {
        res[len++] = span;
        last_ = span.right;
      } else if (span.right > last_) {
        FML_DCHECK(len > 0);
        res[len - 1].right = span.right;
        last_ = span.right;
      }
    }

    size_t len = 0;
    std::vector<Span>& res;

   private:
    int32_t last_ = std::numeric_limits<int32_t>::min();
  };

  const Span *begin1, *end1;
  a_buffer.getSpans(a_handle, begin1, end1);

  const Span *begin2, *end2;
  b_buffer.getSpans(b_handle, begin2, end2);

  size_t min_size = (end1 - begin1) + (end2 - begin2);
  if (res.size() < min_size) {
    res.resize(min_size);
  }

  OrderedSpanAccumulator accumulator(res);

  while (true) {
    if (begin1->left < begin2->left) {
      accumulator.accumulate(*begin1++);
      if (begin1 == end1) {
        break;
      }
    } else {
      // Either 2 is first, or they are equal, in which case add 2 now
      // and we might combine 1 with it next time around
      accumulator.accumulate(*begin2++);
      if (begin2 == end2) {
        break;
      }
    }
  }

  FML_DCHECK(begin1 == end1 || begin2 == end2);

  while (begin1 < end1) {
    accumulator.accumulate(*begin1++);
  }
  while (begin2 < end2) {
    accumulator.accumulate(*begin2++);
  }

  FML_DCHECK(begin1 == end1 && begin2 == end2);

  return accumulator.len;
}

size_t DlRegion::intersectLineSpans(std::vector<Span>& res,
                                    const SpanBuffer& a_buffer,
                                    SpanChunkHandle a_handle,
                                    const SpanBuffer& b_buffer,
                                    SpanChunkHandle b_handle) {
  const Span *begin1, *end1;
  a_buffer.getSpans(a_handle, begin1, end1);

  const Span *begin2, *end2;
  b_buffer.getSpans(b_handle, begin2, end2);

  // Worst case scenario, interleaved overlapping spans
  //   AAAA  BBBB  CCCC
  // XXX  YYYY  XXXX
  size_t min_size = (end1 - begin1) + (end2 - begin2) - 1;
  if (res.size() < min_size) {
    res.resize(min_size);
  }

  // Pointer to the next span to be written.
  Span* new_span = res.data();

  while (begin1 != end1 && begin2 != end2) {
    if (begin1->right <= begin2->left) {
      ++begin1;
    } else if (begin2->right <= begin1->left) {
      ++begin2;
    } else {
      int32_t left = std::max(begin1->left, begin2->left);
      int32_t right = std::min(begin1->right, begin2->right);
      FML_DCHECK(left < right);
      FML_DCHECK(new_span < res.data() + res.size());
      *new_span++ = {left, right};
      if (begin1->right == right) {
        ++begin1;
      }
      if (begin2->right == right) {
        ++begin2;
      }
    }
  }

  return new_span - res.data();
}

void DlRegion::setRects(const std::vector<DlIRect>& unsorted_rects) {
  // setRects can only be called on empty regions.
  FML_DCHECK(lines_.empty());

  size_t count = unsorted_rects.size();
  std::vector<const DlIRect*> rects(count);
  for (size_t i = 0; i < count; i++) {
    rects[i] = &unsorted_rects[i];
    bounds_ = bounds_.Union(unsorted_rects[i]);
  }
  std::sort(rects.begin(), rects.end(), [](const DlIRect* a, const DlIRect* b) {
    if (a->GetTop() < b->GetTop()) {
      return true;
    }
    if (a->GetTop() > b->GetTop()) {
      return false;
    }
    return a->GetLeft() < b->GetLeft();
  });

  size_t active_end = 0;
  size_t next_rect = 0;
  int32_t cur_y = std::numeric_limits<int32_t>::min();
  SpanVec working_spans;

#ifdef DlRegion_DO_STATS
  size_t active_rect_count = 0;
  size_t span_count = 0;
  int pass_count = 0;
  int line_count = 0;
#endif

  while (next_rect < count || active_end > 0) {
    // First prune passed rects out of the active list
    size_t preserve_end = 0;
    for (size_t i = 0; i < active_end; i++) {
      const DlIRect* r = rects[i];
      if (r->GetBottom() > cur_y) {
        rects[preserve_end++] = r;
      }
    }
    active_end = preserve_end;

    // If we have no active rects any more, jump to the top of the
    // next available input rect.
    if (active_end == 0) {
      if (next_rect >= count) {
        // No active rects and no more rects to bring in. We are done.
        break;
      }
      cur_y = rects[next_rect]->GetTop();
    }

    // Next, insert any new rects we've reached into the active list
    while (next_rect < count) {
      const DlIRect* r = rects[next_rect];
      if (r->IsEmpty()) {
        continue;
      }
      if (r->GetTop() > cur_y) {
        break;
      }
      // We now know that we will be inserting this rect into active list
      next_rect++;
      size_t insert_at = active_end++;
      while (insert_at > 0) {
        const DlIRect* ir = rects[insert_at - 1];
        if (ir->GetLeft() <= r->GetLeft()) {
          break;
        }
        rects[insert_at--] = ir;
      }
      rects[insert_at] = r;
    }

    // We either preserved some rects in the active list or added more from
    // the remaining input rects, or we would have exited the loop above.
    FML_DCHECK(active_end != 0);
    working_spans.clear();
    FML_DCHECK(working_spans.empty());

#ifdef DlRegion_DO_STATS
    active_rect_count += active_end;
    pass_count++;
#endif

    // [start_x, end_x) always represents a valid span to be inserted
    // [cur_y, end_y) is the intersecting range over which all spans are valid
    int32_t start_x = rects[0]->GetLeft();
    int32_t end_x = rects[0]->GetRight();
    int32_t end_y = rects[0]->GetBottom();
    for (size_t i = 1; i < active_end; i++) {
      const DlIRect* r = rects[i];
      if (r->GetLeft() > end_x) {
        working_spans.emplace_back(start_x, end_x);
        start_x = r->GetLeft();
        end_x = r->GetRight();
      } else if (end_x < r->GetRight()) {
        end_x = r->GetRight();
      }
      if (end_y > r->GetBottom()) {
        end_y = r->GetBottom();
      }
    }
    working_spans.emplace_back(start_x, end_x);

    // end_y must not pass by the top of the next input rect
    if (next_rect < count && end_y > rects[next_rect]->GetTop()) {
      end_y = rects[next_rect]->GetTop();
    }

    // If all of the rules above work out, we should never collapse the
    // current range of Y coordinates to empty
    FML_DCHECK(end_y > cur_y);

    if (!lines_.empty() && lines_.back().bottom == cur_y &&
        spansEqual(lines_.back(), working_spans.data(),
                   working_spans.data() + working_spans.size())) {
      lines_.back().bottom = end_y;
    } else {
#ifdef DlRegion_DO_STATS
      span_count += working_spans.size();
      line_count++;
#endif
      lines_.push_back(makeLine(cur_y, end_y, working_spans));
    }
    cur_y = end_y;
  }

#ifdef DlRegion_DO_STATS
  double span_avg = ((double)span_count) / line_count;
  double active_avg = ((double)active_rect_count) / pass_count;
  FML_LOG(ERROR) << lines_.size() << " lines for " << count
                 << " input rects, avg " << span_avg
                 << " spans per line and avg " << active_avg
                 << " active rects per loop";
#endif
}

void DlRegion::appendLine(int32_t top,
                          int32_t bottom,
                          const Span* begin,
                          const Span* end) {
  if (lines_.empty()) {
    lines_.push_back(makeLine(top, bottom, begin, end));
  } else {
    if (lines_.back().bottom == top && spansEqual(lines_.back(), begin, end)) {
      lines_.back().bottom = bottom;
    } else {
      lines_.push_back(makeLine(top, bottom, begin, end));
    }
  }
}

DlRegion DlRegion::MakeUnion(const DlRegion& a, const DlRegion& b) {
  if (a.isEmpty()) {
    return b;
  } else if (b.isEmpty()) {
    return a;
  } else if (a.isSimple() && a.bounds_.Contains(b.bounds_)) {
    return a;
  } else if (b.isSimple() && b.bounds_.Contains(a.bounds_)) {
    return b;
  }

  DlRegion res;
  res.bounds_ = a.bounds_.Union(b.bounds_);
  res.span_buffer_.reserve(a.span_buffer_.capacity() +
                           b.span_buffer_.capacity());

  auto& lines = res.lines_;
  lines.reserve(a.lines_.size() + b.lines_.size());

  auto a_it = a.lines_.begin();
  auto b_it = b.lines_.begin();
  auto a_end = a.lines_.end();
  auto b_end = b.lines_.end();

  FML_DCHECK(a_it != a_end && b_it != b_end);

  auto& a_buffer = a.span_buffer_;
  auto& b_buffer = b.span_buffer_;

  std::vector<Span> tmp;

  int32_t cur_top = std::numeric_limits<int32_t>::min();

  while (a_it != a_end && b_it != b_end) {
    auto a_top = std::max(cur_top, a_it->top);
    auto b_top = std::max(cur_top, b_it->top);
    if (a_it->bottom <= b_top) {
      res.appendLine(a_top, a_it->bottom, a_buffer, a_it->chunk_handle);
      ++a_it;
    } else if (b_it->bottom <= a_top) {
      res.appendLine(b_top, b_it->bottom, b_buffer, b_it->chunk_handle);
      ++b_it;
    } else {
      if (a_top < b_top) {
        res.appendLine(a_top, b_top, a_buffer, a_it->chunk_handle);
        cur_top = b_top;
        if (cur_top == a_it->bottom) {
          ++a_it;
        }
      } else if (b_top < a_top) {
        res.appendLine(b_top, a_top, b_buffer, b_it->chunk_handle);
        cur_top = a_top;
        if (cur_top == b_it->bottom) {
          ++b_it;
        }
      } else {
        auto new_bottom = std::min(a_it->bottom, b_it->bottom);
        FML_DCHECK(a_top == b_top);
        FML_DCHECK(new_bottom > a_top);
        FML_DCHECK(new_bottom > b_top);
        auto size = unionLineSpans(tmp, a_buffer, a_it->chunk_handle, b_buffer,
                                   b_it->chunk_handle);
        res.appendLine(a_top, new_bottom, tmp.data(), tmp.data() + size);
        cur_top = new_bottom;
        if (cur_top == a_it->bottom) {
          ++a_it;
        }
        if (cur_top == b_it->bottom) {
          ++b_it;
        }
      }
    }
  }

  FML_DCHECK(a_it == a_end || b_it == b_end);

  while (a_it != a_end) {
    auto a_top = std::max(cur_top, a_it->top);
    res.appendLine(a_top, a_it->bottom, a_buffer, a_it->chunk_handle);
    ++a_it;
  }

  while (b_it != b_end) {
    auto b_top = std::max(cur_top, b_it->top);
    res.appendLine(b_top, b_it->bottom, b_buffer, b_it->chunk_handle);
    ++b_it;
  }

  return res;
}

DlRegion DlRegion::MakeIntersection(const DlRegion& a, const DlRegion& b) {
  if (!a.bounds_.IntersectsWithRect(b.bounds_)) {
    return DlRegion();
  } else if (a.isSimple() && b.isSimple()) {
    auto res = a.bounds_.Intersection(b.bounds_);
    FML_DCHECK(res.has_value());
    return res.has_value() ? DlRegion(res.value()) : DlRegion();
  } else if (a.isSimple() && a.bounds_.Contains(b.bounds_)) {
    return b;
  } else if (b.isSimple() && b.bounds_.Contains(a.bounds_)) {
    return a;
  }

  DlRegion res;
  res.span_buffer_.reserve(
      std::max(a.span_buffer_.capacity(), b.span_buffer_.capacity()));

  auto& lines = res.lines_;
  lines.reserve(std::min(a.lines_.size(), b.lines_.size()));

  std::vector<SpanLine>::const_iterator a_it, b_it;
  getIntersectionIterators(a.lines_, b.lines_, a_it, b_it);

  auto a_end = a.lines_.end();
  auto b_end = b.lines_.end();

  auto& a_buffer = a.span_buffer_;
  auto& b_buffer = b.span_buffer_;

  std::vector<Span> tmp;

  int32_t cur_top = std::numeric_limits<int32_t>::min();

  while (a_it != a_end && b_it != b_end) {
    auto a_top = std::max(cur_top, a_it->top);
    auto b_top = std::max(cur_top, b_it->top);
    if (a_it->bottom <= b_top) {
      ++a_it;
    } else if (b_it->bottom <= a_top) {
      ++b_it;
    } else {
      auto top = std::max(a_top, b_top);
      auto bottom = std::min(a_it->bottom, b_it->bottom);
      FML_DCHECK(top < bottom);
      auto size = intersectLineSpans(tmp, a_buffer, a_it->chunk_handle,
                                     b_buffer, b_it->chunk_handle);
      if (size > 0) {
        res.appendLine(top, bottom, tmp.data(), tmp.data() + size);
        res.bounds_ = res.bounds_.Union(DlIRect::MakeLTRB(
            tmp.data()->left, top, (tmp.data() + size - 1)->right, bottom));
      }
      cur_top = bottom;
      if (cur_top == a_it->bottom) {
        ++a_it;
      }
      if (cur_top == b_it->bottom) {
        ++b_it;
      }
    }
  }
  FML_DCHECK(a_it == a_end || b_it == b_end);
  return res;
}

std::vector<DlIRect> DlRegion::getRects(bool deband) const {
  std::vector<DlIRect> rects;
  if (isEmpty()) {
    return rects;
  } else if (isSimple()) {
    rects.push_back(bounds_);
    return rects;
  }

  size_t rect_count = 0;
  size_t previous_span_end = 0;
  for (const auto& line : lines_) {
    rect_count += span_buffer_.getChunkSize(line.chunk_handle);
  }
  rects.reserve(rect_count);

  for (const auto& line : lines_) {
    const Span *span_begin, *span_end;
    span_buffer_.getSpans(line.chunk_handle, span_begin, span_end);
    for (const auto* span = span_begin; span < span_end; ++span) {
      int32_t top = line.top;
      if (deband) {
        auto iter = rects.begin() + previous_span_end;
        // If there is rectangle previously in rects on which this one is a
        // vertical continuation, remove the previous rectangle and expand
        // this one vertically to cover the area.
        while (iter != rects.begin()) {
          --iter;
          if (iter->GetBottom() < top) {
            // Went all the way to previous span line.
            break;
          } else if (iter->GetLeft() == span->left &&
                     iter->GetRight() == span->right) {
            FML_DCHECK(iter->GetBottom() == top);
            top = iter->GetTop();
            rects.erase(iter);
            --previous_span_end;
            break;
          }
        }
      }
      rects.push_back(
          DlIRect::MakeLTRB(span->left, top, span->right, line.bottom));
    }
    previous_span_end = rects.size();
  }
  return rects;
}

bool DlRegion::isComplex() const {
  return lines_.size() > 1 ||
         (lines_.size() == 1 &&
          span_buffer_.getChunkSize(lines_.front().chunk_handle) > 1);
}

bool DlRegion::intersects(const DlIRect& rect) const {
  if (isEmpty()) {
    return false;
  }

  auto bounds_intersect = bounds_.IntersectsWithRect(rect);

  if (isSimple()) {
    return bounds_intersect;
  }

  if (!bounds_intersect) {
    return false;
  }

  auto it = lines_.begin();
  auto end = lines_.end();
  if (lines_.size() > kBinarySearchThreshold &&
      it[kBinarySearchThreshold].bottom <= rect.GetTop()) {
    it = std::lower_bound(
        lines_.begin() + kBinarySearchThreshold + 1, lines_.end(),
        rect.GetTop(),
        [](const SpanLine& line, int32_t top) { return line.bottom <= top; });
  } else {
    while (it != end && it->bottom <= rect.GetTop()) {
      ++it;
      continue;
    }
  }
  while (it != end && it->top < rect.GetBottom()) {
    FML_DCHECK(rect.GetTop() < it->bottom && it->top < rect.GetBottom());
    const Span *begin, *end;
    span_buffer_.getSpans(it->chunk_handle, begin, end);
    while (begin != end && begin->left < rect.GetRight()) {
      if (begin->right > rect.GetLeft()) {
        return true;
      }
      ++begin;
    }
    ++it;
  }

  return false;
}

bool DlRegion::spansIntersect(const Span* begin1,
                              const Span* end1,
                              const Span* begin2,
                              const Span* end2) {
  while (begin1 != end1 && begin2 != end2) {
    if (begin1->right <= begin2->left) {
      ++begin1;
    } else if (begin2->right <= begin1->left) {
      ++begin2;
    } else {
      return true;
    }
  }
  return false;
}

void DlRegion::getIntersectionIterators(
    const std::vector<SpanLine>& a_lines,
    const std::vector<SpanLine>& b_lines,
    std::vector<SpanLine>::const_iterator& a_it,
    std::vector<SpanLine>::const_iterator& b_it) {
  a_it = a_lines.begin();
  auto a_end = a_lines.end();
  b_it = b_lines.begin();
  auto b_end = b_lines.end();

  FML_DCHECK(a_it != a_end && b_it != b_end);

  auto a_len = a_end - a_it;
  auto b_len = b_end - b_it;

  if (a_len > kBinarySearchThreshold &&
      a_it[kBinarySearchThreshold].bottom <= b_it->top) {
    a_it = std::lower_bound(
        a_lines.begin() + kBinarySearchThreshold + 1, a_lines.end(), b_it->top,
        [](const SpanLine& line, int32_t top) { return line.bottom <= top; });
  } else if (b_len > kBinarySearchThreshold &&
             b_it[kBinarySearchThreshold].bottom <= a_it->top) {
    b_it = std::lower_bound(
        b_lines.begin() + kBinarySearchThreshold + 1, b_lines.end(), a_it->top,
        [](const SpanLine& line, int32_t top) { return line.bottom <= top; });
  }
}

bool DlRegion::intersects(const DlRegion& region) const {
  if (isEmpty() || region.isEmpty()) {
    return false;
  }

  auto our_complex = isComplex();
  auto their_complex = region.isComplex();
  auto bounds_intersect = bounds_.IntersectsWithRect(region.bounds_);

  if (!our_complex && !their_complex) {
    return bounds_intersect;
  }

  if (!bounds_intersect) {
    return false;
  }

  if (!our_complex) {
    return region.intersects(bounds_);
  }

  if (!their_complex) {
    return intersects(region.bounds_);
  }

  std::vector<SpanLine>::const_iterator ours, theirs;
  getIntersectionIterators(lines_, region.lines_, ours, theirs);
  auto ours_end = lines_.end();
  auto theirs_end = region.lines_.end();

  while (ours != ours_end && theirs != theirs_end) {
    if (ours->bottom <= theirs->top) {
      ++ours;
    } else if (theirs->bottom <= ours->top) {
      ++theirs;
    } else {
      FML_DCHECK(ours->top < theirs->bottom && theirs->top < ours->bottom);
      const Span *ours_begin, *ours_end;
      span_buffer_.getSpans(ours->chunk_handle, ours_begin, ours_end);
      const Span *theirs_begin, *theirs_end;
      region.span_buffer_.getSpans(theirs->chunk_handle, theirs_begin,
                                   theirs_end);
      if (spansIntersect(ours_begin, ours_end, theirs_begin, theirs_end)) {
        return true;
      }
      if (ours->bottom < theirs->bottom) {
        ++ours;
      } else {
        ++theirs;
      }
    }
  }
  return false;
}

}  // namespace flutter
