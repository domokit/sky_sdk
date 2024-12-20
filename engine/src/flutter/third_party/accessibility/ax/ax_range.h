// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_RANGE_H_
#define UI_ACCESSIBILITY_AX_RANGE_H_

#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "ax_clipping_behavior.h"
#include "ax_enums.h"
#include "ax_offscreen_result.h"
#include "ax_role_properties.h"
#include "ax_tree_manager_map.h"
#include "base/string_utils.h"

namespace ui {

// Specifies how AXRange::GetText treats line breaks introduced by layout.
// For example, consider the following HTML snippet: "A<div>B</div>C".
enum class AXTextConcatenationBehavior {
  // Preserve any introduced line breaks, e.g. GetText = "A\nB\nC".
  kAsInnerText,
  // Ignore any introduced line breaks, e.g. GetText = "ABC".
  kAsTextContent
};

class AXRangeRectDelegate {
 public:
  virtual gfx::Rect GetInnerTextRangeBoundsRect(
      AXTreeID tree_id,
      AXNode::AXID node_id,
      int start_offset,
      int end_offset,
      ui::AXClippingBehavior clipping_behavior,
      AXOffscreenResult* offscreen_result) = 0;
  virtual gfx::Rect GetBoundsRect(AXTreeID tree_id,
                                  AXNode::AXID node_id,
                                  AXOffscreenResult* offscreen_result) = 0;
};

// A range delimited by two positions in the AXTree.
//
// In order to avoid any confusion regarding whether a deep or a shallow copy is
// being performed, this class can be moved, but not copied.
template <class AXPositionType>
class AXRange {
 public:
  using AXPositionInstance = std::unique_ptr<AXPositionType>;

  AXRange()
      : anchor_(AXPositionType::CreateNullPosition()),
        focus_(AXPositionType::CreateNullPosition()) {}

  AXRange(AXPositionInstance anchor, AXPositionInstance focus) {
    anchor_ = anchor ? std::move(anchor) : AXPositionType::CreateNullPosition();
    focus_ = focus ? std::move(focus) : AXPositionType::CreateNullPosition();
  }

  AXRange(const AXRange& other) = delete;

  AXRange(AXRange&& other) : AXRange() {
    anchor_.swap(other.anchor_);
    focus_.swap(other.focus_);
  }

  virtual ~AXRange() = default;

  AXPositionType* anchor() const {
    BASE_DCHECK(anchor_);
    return anchor_.get();
  }

  AXPositionType* focus() const {
    BASE_DCHECK(focus_);
    return focus_.get();
  }

  AXRange& operator=(const AXRange& other) = delete;

  AXRange& operator=(AXRange&& other) {
    if (this != &other) {
      anchor_ = AXPositionType::CreateNullPosition();
      focus_ = AXPositionType::CreateNullPosition();
      anchor_.swap(other.anchor_);
      focus_.swap(other.focus_);
    }
    return *this;
  }

  bool operator==(const AXRange& other) const {
    if (IsNull())
      return other.IsNull();
    return !other.IsNull() && *anchor_ == *other.anchor() &&
           *focus_ == *other.focus();
  }

  bool operator!=(const AXRange& other) const { return !(*this == other); }

  // Given a pair of AXPosition, determines how the first compares with the
  // second, relative to the order they would be iterated over by using
  // AXRange::Iterator to traverse all leaf text ranges in a tree.
  //
  // Notice that this method is different from using AXPosition::CompareTo since
  // the following logic takes into account BOTH tree pre-order traversal and
  // text offsets when both positions are located within the same anchor.
  //
  // Returns:
  //         0 - If both positions are equivalent.
  //        <0 - If the first position would come BEFORE the second.
  //        >0 - If the first position would come AFTER the second.
  //   nullopt - If positions are not comparable (see AXPosition::CompareTo).
  static std::optional<int> CompareEndpoints(const AXPositionType* first,
                                             const AXPositionType* second) {
    std::optional<int> tree_position_comparison =
        first->AsTreePosition()->CompareTo(*second->AsTreePosition());

    // When the tree comparison is nullopt, using value_or(1) forces a default
    // value of 1, making the following statement return nullopt as well.
    return (tree_position_comparison.value_or(1) != 0)
               ? tree_position_comparison
               : first->CompareTo(*second);
  }

  AXRange AsForwardRange() const {
    return (CompareEndpoints(anchor(), focus()).value_or(0) > 0)
               ? AXRange(focus_->Clone(), anchor_->Clone())
               : AXRange(anchor_->Clone(), focus_->Clone());
  }

  AXRange AsBackwardRange() const {
    return (CompareEndpoints(anchor(), focus()).value_or(0) < 0)
               ? AXRange(focus_->Clone(), anchor_->Clone())
               : AXRange(anchor_->Clone(), focus_->Clone());
  }

  bool IsCollapsed() const { return !IsNull() && *anchor_ == *focus_; }

  // We define a "leaf text range" as an AXRange whose endpoints are leaf text
  // positions located within the same anchor of the AXTree.
  bool IsLeafTextRange() const {
    return !IsNull() && anchor_->GetAnchor() == focus_->GetAnchor() &&
           anchor_->IsLeafTextPosition() && focus_->IsLeafTextPosition();
  }

  bool IsNull() const {
    BASE_DCHECK(anchor_ && focus_);
    return anchor_->IsNullPosition() || focus_->IsNullPosition();
  }

  std::string ToString() const {
    return "Range\nAnchor:" + anchor_->ToString() +
           "\nFocus:" + focus_->ToString();
  }

  // We can decompose any given AXRange into multiple "leaf text ranges".
  // As an example, consider the following HTML code:
  //
  //   <p>line with text<br><input type="checkbox">line with checkbox</p>
  //
  // It will produce the following AXTree; notice that the leaf text nodes
  // (enclosed in parenthesis) compose its text representation:
  //
  //   paragraph
  //     staticText name='line with text'
  //       (inlineTextBox name='line with text')
  //     lineBreak name='<newline>'
  //       (inlineTextBox name='<newline>')
  //     (checkBox)
  //     staticText name='line with checkbox'
  //       (inlineTextBox name='line with checkbox')
  //
  // Suppose we have an AXRange containing all elements from the example above.
  // The text representation of such range, with AXRange's endpoints marked by
  // opening and closing brackets, will look like the following:
  //
  //   "[line with text\n{checkBox}line with checkbox]"
  //
  // Note that in the text representation {checkBox} is not visible, but it is
  // effectively a "leaf text range", so we include it in the example above only
  // to visualize how the iterator should work.
  //
  // Decomposing the AXRange above into its "leaf text ranges" would result in:
  //
  //   "[line with text][\n][{checkBox}][line with checkbox]"
  //
  // This class allows AXRange to be iterated through all "leaf text ranges"
  // contained between its endpoints, composing the entire range.
  class Iterator : public std::iterator<std::input_iterator_tag, AXRange> {
   public:
    Iterator()
        : current_start_(AXPositionType::CreateNullPosition()),
          iterator_end_(AXPositionType::CreateNullPosition()) {}

    Iterator(AXPositionInstance start, AXPositionInstance end) {
      if (end && !end->IsNullPosition()) {
        current_start_ = !start ? AXPositionType::CreateNullPosition()
                                : start->AsLeafTextPosition();
        iterator_end_ = end->AsLeafTextPosition();
      } else {
        current_start_ = AXPositionType::CreateNullPosition();
        iterator_end_ = AXPositionType::CreateNullPosition();
      }
    }

    Iterator(const Iterator& other) = delete;

    Iterator(Iterator&& other)
        : current_start_(std::move(other.current_start_)),
          iterator_end_(std::move(other.iterator_end_)) {}

    ~Iterator() = default;

    bool operator==(const Iterator& other) const {
      return current_start_->GetAnchor() == other.current_start_->GetAnchor() &&
             iterator_end_->GetAnchor() == other.iterator_end_->GetAnchor() &&
             *current_start_ == *other.current_start_ &&
             *iterator_end_ == *other.iterator_end_;
    }

    bool operator!=(const Iterator& other) const { return !(*this == other); }

    // Only forward iteration is supported, so operator-- is not implemented.
    Iterator& operator++() {
      BASE_DCHECK(!current_start_->IsNullPosition());
      if (current_start_->GetAnchor() == iterator_end_->GetAnchor()) {
        current_start_ = AXPositionType::CreateNullPosition();
      } else {
        current_start_ = current_start_->CreateNextLeafTreePosition();
        BASE_DCHECK(*current_start_ <= *iterator_end_);
      }
      return *this;
    }

    AXRange operator*() const {
      BASE_DCHECK(!current_start_->IsNullPosition());
      AXPositionInstance current_end =
          (current_start_->GetAnchor() != iterator_end_->GetAnchor())
              ? current_start_->CreatePositionAtEndOfAnchor()
              : iterator_end_->Clone();
      BASE_DCHECK(*current_end <= *iterator_end_);

      AXRange current_leaf_text_range(current_start_->AsTextPosition(),
                                      current_end->AsTextPosition());
      BASE_DCHECK(current_leaf_text_range.IsLeafTextRange());
      return std::move(current_leaf_text_range);
    }

   private:
    AXPositionInstance current_start_;
    AXPositionInstance iterator_end_;
  };

  Iterator begin() const {
    if (IsNull())
      return Iterator(nullptr, nullptr);
    AXRange forward_range = AsForwardRange();
    return Iterator(std::move(forward_range.anchor_),
                    std::move(forward_range.focus_));
  }

  Iterator end() const {
    if (IsNull())
      return Iterator(nullptr, nullptr);
    AXRange forward_range = AsForwardRange();
    return Iterator(nullptr, std::move(forward_range.focus_));
  }

  // Returns the concatenation of the accessible names of all text nodes
  // contained between this AXRange's endpoints.
  // Pass a |max_count| of -1 to retrieve all text in the AXRange.
  // Note that if this AXRange has its anchor or focus located at an ignored
  // position, we shrink the range to the closest unignored positions.
  std::u16string GetText(AXTextConcatenationBehavior concatenation_behavior =
                             AXTextConcatenationBehavior::kAsTextContent,
                         int max_count = -1,
                         bool include_ignored = false,
                         size_t* appended_newlines_count = nullptr) const {
    if (max_count == 0 || IsNull())
      return std::u16string();

    std::optional<int> endpoint_comparison =
        CompareEndpoints(anchor(), focus());
    if (!endpoint_comparison)
      return std::u16string();

    AXPositionInstance start = (endpoint_comparison.value() < 0)
                                   ? anchor_->AsLeafTextPosition()
                                   : focus_->AsLeafTextPosition();
    AXPositionInstance end = (endpoint_comparison.value() < 0)
                                 ? focus_->AsLeafTextPosition()
                                 : anchor_->AsLeafTextPosition();

    std::u16string range_text;
    size_t computed_newlines_count = 0;
    bool is_first_non_whitespace_leaf = true;
    bool crossed_paragraph_boundary = false;
    bool is_first_unignored_leaf = true;
    bool found_trailing_newline = false;

    while (!start->IsNullPosition()) {
      BASE_DCHECK(start->IsLeafTextPosition());
      BASE_DCHECK(start->text_offset() >= 0);

      if (include_ignored || !start->IsIgnored()) {
        if (concatenation_behavior ==
                AXTextConcatenationBehavior::kAsInnerText &&
            !start->IsInWhiteSpace()) {
          if (is_first_non_whitespace_leaf) {
            // The first non-whitespace leaf in the range could be preceded by
            // whitespace spanning even before the start of this range, we need
            // to check such positions in order to correctly determine if this
            // is a paragraph's start (see |AXPosition::AtStartOfParagraph|).
            crossed_paragraph_boundary =
                !is_first_unignored_leaf && start->AtStartOfParagraph();
          }

          // When preserving layout line breaks, don't append `\n` next if the
          // previous leaf position was a <br> (already ending with a newline).
          if (crossed_paragraph_boundary && !found_trailing_newline) {
            range_text += base::ASCIIToUTF16("\n");
            computed_newlines_count++;
          }

          is_first_non_whitespace_leaf = false;
          crossed_paragraph_boundary = false;
        }

        int current_end_offset = (start->GetAnchor() != end->GetAnchor())
                                     ? start->MaxTextOffset()
                                     : end->text_offset();

        if (current_end_offset > start->text_offset()) {
          int characters_to_append =
              (max_count > 0)
                  ? std::min(max_count - static_cast<int>(range_text.length()),
                             current_end_offset - start->text_offset())
                  : current_end_offset - start->text_offset();

          range_text += start->GetText().substr(start->text_offset(),
                                                characters_to_append);

          // Collapse all whitespace following any line break.
          found_trailing_newline =
              start->IsInLineBreak() ||
              (found_trailing_newline && start->IsInWhiteSpace());
        }

        BASE_DCHECK(max_count < 0 ||
                    static_cast<int>(range_text.length()) <= max_count);
        is_first_unignored_leaf = false;
      }

      if (start->GetAnchor() == end->GetAnchor() ||
          static_cast<int>(range_text.length()) == max_count) {
        break;
      } else if (concatenation_behavior ==
                     AXTextConcatenationBehavior::kAsInnerText &&
                 !crossed_paragraph_boundary && !is_first_non_whitespace_leaf) {
        start = start->CreateNextLeafTextPosition(&crossed_paragraph_boundary);
      } else {
        start = start->CreateNextLeafTextPosition();
      }
    }

    if (appended_newlines_count)
      *appended_newlines_count = computed_newlines_count;
    return range_text;
  }

  // Appends rects of all anchor nodes that span between anchor_ and focus_.
  // Rects outside of the viewport are skipped.
  // Coordinate system is determined by the passed-in delegate.
  std::vector<gfx::Rect> GetRects(AXRangeRectDelegate* delegate) const {
    std::vector<gfx::Rect> rects;

    for (const AXRange& leaf_text_range : *this) {
      BASE_DCHECK(leaf_text_range.IsLeafTextRange());
      AXPositionType* current_line_start = leaf_text_range.anchor();
      AXPositionType* current_line_end = leaf_text_range.focus();

      // For text anchors, we retrieve the bounding rectangles of its text
      // content. For non-text anchors (such as checkboxes, images, etc.), we
      // want to directly retrieve their bounding rectangles.
      AXOffscreenResult offscreen_result;
      gfx::Rect current_rect =
          (current_line_start->IsInLineBreak() ||
           current_line_start->IsInTextObject())
              ? delegate->GetInnerTextRangeBoundsRect(
                    current_line_start->tree_id(),
                    current_line_start->anchor_id(),
                    current_line_start->text_offset(),
                    current_line_end->text_offset(),
                    ui::AXClippingBehavior::kUnclipped, &offscreen_result)
              : delegate->GetBoundsRect(current_line_start->tree_id(),
                                        current_line_start->anchor_id(),
                                        &offscreen_result);

      // If the bounding box of the current range is clipped because it lies
      // outside an ancestor’s bounds, then the bounding box is pushed to the
      // nearest edge of such ancestor's bounds, with its width and height
      // forced to be 1, and the node will be marked as "offscreen".
      //
      // Only add rectangles that are not empty and not marked as "offscreen".
      //
      // See the documentation for how bounding boxes are calculated in AXTree:
      // https://chromium.googlesource.com/chromium/src/+/HEAD/docs/accessibility/offscreen.md
      if (!current_rect.IsEmpty() &&
          offscreen_result == AXOffscreenResult::kOnscreen)
        rects.push_back(current_rect);
    }
    return rects;
  }

 private:
  AXPositionInstance anchor_;
  AXPositionInstance focus_;
};

template <class AXPositionType>
std::ostream& operator<<(std::ostream& stream,
                         const AXRange<AXPositionType>& range) {
  return stream << range.ToString();
}

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_RANGE_H_
