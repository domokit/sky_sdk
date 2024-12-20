// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_POSITION_H_
#define UI_ACCESSIBILITY_AX_POSITION_H_

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <ostream>
#include <stack>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "ax_enum_util.h"
#include "ax_enums.h"
#include "ax_node.h"
#include "ax_node_text_styles.h"
#include "ax_role_properties.h"
#include "ax_tree_id.h"
#include "base/container_utils.h"
#include "base/logging.h"
#include "base/string_utils.h"

namespace ui {

// Defines the type of position in the accessibility tree.
// A tree position is used when referring to a specific child of a node in the
// accessibility tree.
// A text position is used when referring to a specific character of text inside
// a particular node.
// A null position is used to signify that the provided data is invalid or that
// a boundary has been reached.
enum class AXPositionKind { NULL_POSITION, TREE_POSITION, TEXT_POSITION };

// Defines how creating the next or previous position should behave whenever we
// are at or are crossing a boundary, such as at the start of an anchor, a word
// or a line.
enum class AXBoundaryBehavior {
  CrossBoundary,
  StopAtAnchorBoundary,
  StopIfAlreadyAtBoundary,
  StopAtLastAnchorBoundary
};

// Describes in further detail what type of boundary a current position is on.
// For complex boundaries such as format boundaries, it can be useful to know
// why a particular boundary was chosen.
enum class AXBoundaryType {
  // Not at a unit boundary.
  kNone,
  // At a unit boundary (e.g. a format boundary).
  kUnitBoundary,
  // At the start of a document.
  kDocumentStart,
  // At the end of a document.
  kDocumentEnd
};

// When converting to an unignored position, determines how to adjust the new
// position in order to make it valid, either moving backward or forward in
// the accessibility tree.
enum class AXPositionAdjustmentBehavior { kMoveBackward, kMoveForward };

// Specifies how AXPosition::ExpandToEnclosingTextBoundary behaves.
//
// As an example, imagine we have the text "hello world" and a position before
// the space character. We want to expand to the surrounding word boundary.
// Since we are right at the end of the first word, we could either expand to
// the left first, find the start of the first word and then use that to find
// the corresponding word end, resulting in the word "Hello". Another
// possibility is to expand to the right first, find the end of the next word
// and use that as our starting point to find the previous word start, resulting
// in the word "world".
enum class AXRangeExpandBehavior {
  // Expands to the left boundary first and then uses that position as the
  // starting point to find the boundary to the right.
  kLeftFirst,
  // Expands to the right boundary first and then uses that position as the
  // starting point to find the boundary to the left.
  kRightFirst
};

// Some platforms require empty objects to be represented by a replacement
// character in order for text navigation to work correctly. This enum controls
// whether a replacement character will be exposed for such objects.
//
// When an embedded object is replaced by a real character, the expectations
// are the same with this character as with other ordinary characters.
// For example, with UIA on Windows, we need to be able to navigate inside and
// outside of this character as if it was an ordinary character, using the
// AXPlatformNodeTextRangeProvider methods. Since an embedded object character
// is the only character in a node, we also treat this character as a word.
enum class AXEmbeddedObjectBehavior {
  kExposeCharacter,
  kSuppressCharacter,
};

// Controls whether embedded objects are represented by a replacement
// character. This is initialized to a per-platform default but can be
// overridden for testing.
AX_EXPORT extern AXEmbeddedObjectBehavior g_ax_embedded_object_behavior;

class AX_EXPORT ScopedAXEmbeddedObjectBehaviorSetter {
 public:
  explicit ScopedAXEmbeddedObjectBehaviorSetter(
      AXEmbeddedObjectBehavior behavior);
  ~ScopedAXEmbeddedObjectBehaviorSetter();

 private:
  AXEmbeddedObjectBehavior prev_behavior_;
};

// Forward declarations.
template <class AXPositionType, class AXNodeType>
class AXPosition;
template <class AXPositionType>
class AXRange;
template <class AXPositionType, class AXNodeType>
bool operator==(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second);
template <class AXPositionType, class AXNodeType>
bool operator!=(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second);

// A position in the accessibility tree.
//
// This class could either represent a tree position or a text position.
// Tree positions point to either a child of a specific node or at the end of a
// node (i.e. an "after children" position).
// Text positions point to either a character offset in the text inside a
// particular node including text from all its children, or to the end of the
// node's text, (i.e. an "after text" position).
// On tree positions that have a leaf node as their anchor, we also need to
// distinguish between "before text" and "after text" positions. To do this, if
// the child index is 0 and the anchor is a leaf node, then it's an "after text"
// position. If the child index is |BEFORE_TEXT| and the anchor is a leaf node,
// then this is a "before text" position.
// It doesn't make sense to have a "before text" position on a text position,
// because it is identical to setting its offset to the first character.
//
// To avoid re-computing either the text offset or the child index when
// converting between the two types of positions, both values are saved after
// the first conversion.
//
// This class template uses static polymorphism in order to allow sub-classes to
// be created from the base class without the base class knowing the type of the
// sub-class in advance.
// The template argument |AXPositionType| should always be set to the type of
// any class that inherits from this template, making this a
// "curiously recursive template".
//
// This class can be copied using the |Clone| method. It is designed to be
// immutable.
template <class AXPositionType, class AXNodeType>
class AXPosition {
 public:
  using AXPositionInstance =
      std::unique_ptr<AXPosition<AXPositionType, AXNodeType>>;

  using AXRangeType = AXRange<AXPosition<AXPositionType, AXNodeType>>;

  typedef bool BoundaryConditionPredicate(const AXPositionInstance&);

  typedef std::vector<int32_t> BoundaryTextOffsetsFunc(
      const AXPositionInstance&);

  static const int BEFORE_TEXT = -1;
  static const int INVALID_INDEX = -2;
  static const int INVALID_OFFSET = -1;

  // Replacement character used to represent an empty object. See
  // AXEmbeddedObjectBehavior for more information.
  //
  // Duplicate of AXPlatformNodeBase::kEmbeddedCharacter because we don't want
  // to include platform specific code in here.
  static constexpr char16_t kEmbeddedCharacter = L'\xfffc';

  static AXPositionInstance CreateNullPosition() {
    AXPositionInstance new_position(new AXPositionType());
    new_position->Initialize(
        AXPositionKind::NULL_POSITION, AXTreeIDUnknown(), AXNode::kInvalidAXID,
        INVALID_INDEX, INVALID_OFFSET, ax::mojom::TextAffinity::kDownstream);
    return new_position;
  }

  static AXPositionInstance CreateTreePosition(AXTreeID tree_id,
                                               AXNode::AXID anchor_id,
                                               int child_index) {
    AXPositionInstance new_position(new AXPositionType());
    new_position->Initialize(AXPositionKind::TREE_POSITION, tree_id, anchor_id,
                             child_index, INVALID_OFFSET,
                             ax::mojom::TextAffinity::kDownstream);
    return new_position;
  }

  static AXPositionInstance CreateTextPosition(
      AXTreeID tree_id,
      AXNode::AXID anchor_id,
      int text_offset,
      ax::mojom::TextAffinity affinity) {
    AXPositionInstance new_position(new AXPositionType());
    new_position->Initialize(AXPositionKind::TEXT_POSITION, tree_id, anchor_id,
                             INVALID_INDEX, text_offset, affinity);
    return new_position;
  }

  virtual ~AXPosition() = default;

  // Implemented based on the copy and swap idiom.
  AXPosition& operator=(const AXPosition& other) {
    AXPositionInstance clone = other.Clone();
    swap(*clone);
    return *this;
  }

  virtual AXPositionInstance Clone() const = 0;

  // A serialization of a position as POD. Not for sharing on disk or sharing
  // across thread or process boundaries, just for passing a position to an
  // API that works with positions as opaque objects.
  struct SerializedPosition {
    AXPositionKind kind;
    AXNode::AXID anchor_id;
    int child_index;
    int text_offset;
    ax::mojom::TextAffinity affinity;
    char tree_id[33];
  };

  static_assert(std::is_trivially_copyable<SerializedPosition>::value,
                "SerializedPosition must be POD");

  SerializedPosition Serialize() {
    SerializedPosition result;
    result.kind = kind_;

    // A tree ID can be serialized as a 32-byte string.
    std::string tree_id_string = tree_id_.ToString();
    BASE_DCHECK(tree_id_string.size() <= 32U);
    strncpy(result.tree_id, tree_id_string.c_str(), 32);
    result.tree_id[32] = 0;

    result.anchor_id = anchor_id_;
    result.child_index = child_index_;
    result.text_offset = text_offset_;
    result.affinity = affinity_;
    return result;
  }

  static AXPositionInstance Unserialize(
      const SerializedPosition& serialization) {
    AXPositionInstance new_position(new AXPositionType());
    new_position->Initialize(serialization.kind,
                             ui::AXTreeID::FromString(serialization.tree_id),
                             serialization.anchor_id, serialization.child_index,
                             serialization.text_offset, serialization.affinity);
    return new_position;
  }

  std::string ToString() const {
    std::string str;
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return "NullPosition";
      case AXPositionKind::TREE_POSITION: {
        std::string str_child_index;
        if (child_index_ == BEFORE_TEXT) {
          str_child_index = "before_text";
        } else if (child_index_ == INVALID_INDEX) {
          str_child_index = "invalid";
        } else {
          str_child_index = base::NumberToString(child_index_);
        }
        str = "TreePosition tree_id=" + tree_id_.ToString() +
              " anchor_id=" + base::NumberToString(anchor_id_) +
              " child_index=" + str_child_index;
        break;
      }
      case AXPositionKind::TEXT_POSITION: {
        std::string str_text_offset;
        if (text_offset_ == INVALID_OFFSET) {
          str_text_offset = "invalid";
        } else {
          str_text_offset = base::NumberToString(text_offset_);
        }
        str = "TextPosition anchor_id=" + base::NumberToString(anchor_id_) +
              " text_offset=" + str_text_offset + " affinity=" +
              ui::ToString(static_cast<ax::mojom::TextAffinity>(affinity_));
        break;
      }
    }

    if (!IsTextPosition() || text_offset_ > MaxTextOffset())
      return str;

    std::u16string text = GetText();
    BASE_DCHECK(text_offset_ >= 0);
    int max_text_offset = MaxTextOffset();
    BASE_DCHECK(text_offset_ <= max_text_offset);
    std::u16string annotated_text;
    if (text_offset_ == max_text_offset) {
      annotated_text = text + u"<>";
    } else {
      annotated_text = text.substr(0, text_offset_) + u"<" +
                       text[text_offset_] + u">" +
                       text.substr(text_offset_ + 1);
    }
    return str + " annotated_text=" + base::UTF16ToUTF8(annotated_text);
  }

  AXTreeID tree_id() const { return tree_id_; }
  AXNode::AXID anchor_id() const { return anchor_id_; }

  AXNodeType* GetAnchor() const {
    if (tree_id_ == AXTreeIDUnknown() || anchor_id_ == AXNode::kInvalidAXID)
      return nullptr;
    return GetNodeInTree(tree_id_, anchor_id_);
  }

  AXPositionKind kind() const { return kind_; }
  int child_index() const { return child_index_; }
  int text_offset() const { return text_offset_; }
  ax::mojom::TextAffinity affinity() const { return affinity_; }

  bool IsIgnored() const {
    if (IsNullPosition())
      return false;

    BASE_DCHECK(GetAnchor());
    // If this position is anchored to an ignored node, then consider this
    // position to be ignored.
    if (GetAnchor()->IsIgnored()) {
      return true;
    }

    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TREE_POSITION: {
        // If this is a "before text" or an "after text" tree position, it's
        // pointing to the anchor itself, which we've determined to be
        // unignored.
        BASE_DCHECK(!IsLeaf() || child_index_ == BEFORE_TEXT ||
                    child_index_ == 0)
            << "\"Before text\" and \"after text\" tree positions are only "
               "valid on leaf nodes.";
        if (child_index_ == BEFORE_TEXT || IsLeaf())
          return false;

        // If this position is an "after children" position, consider the
        // position to be ignored if the last child is ignored. This is because
        // the last child will not be visible in the unignored tree. If the
        // position is not adjusted, the resulting position would erroneously
        // point before the second child in the unignored subtree rooted at the
        // last child.
        //
        // 1 kRootWebArea
        // ++2 kGenericContainer ignored
        // ++++3 kStaticText "Line 1."
        // ++++4 kStaticText "Line 2."
        //
        // Tree position anchor=kGenericContainer, child_index=1.
        //
        // Alternatively, if there is a node at the position pointed to by
        // "child_index_", i.e. this position is neither a leaf position nor an
        // "after children" position, consider this tree position to be ignored
        // if the child node is ignored.
        int adjusted_child_index = child_index_ != AnchorChildCount()
                                       ? child_index_
                                       : child_index_ - 1;
        AXPositionInstance child_position =
            CreateChildPositionAt(adjusted_child_index);
        BASE_DCHECK(child_position && !child_position->IsNullPosition());
        return child_position->GetAnchor()->IsIgnored();
      }
      case AXPositionKind::TEXT_POSITION:
        // If the corresponding leaf position is ignored, the current text
        // offset will point to ignored text. Therefore, consider this position
        // to be ignored.
        if (!IsLeaf()) {
          return AsLeafTreePosition()->IsIgnored();
        }
        return false;
    }
  }

  bool IsNullPosition() const {
    return kind_ == AXPositionKind::NULL_POSITION || !GetAnchor();
  }

  bool IsTreePosition() const {
    return GetAnchor() && kind_ == AXPositionKind::TREE_POSITION;
  }

  bool IsLeafTreePosition() const { return IsTreePosition() && IsLeaf(); }

  bool IsTextPosition() const {
    return GetAnchor() && kind_ == AXPositionKind::TEXT_POSITION;
  }

  bool IsLeafTextPosition() const { return IsTextPosition() && IsLeaf(); }

  bool IsLeaf() const {
    if (IsNullPosition())
      return false;

    return !AnchorChildCount() || IsEmptyObjectReplacedByCharacter();
  }

  // Returns true if this is a valid position, e.g. the child_index_ or
  // text_offset_ is within a valid range.
  bool IsValid() const {
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return tree_id_ == AXTreeIDUnknown() &&
               anchor_id_ == AXNode::kInvalidAXID &&
               child_index_ == INVALID_INDEX &&
               text_offset_ == INVALID_OFFSET &&
               affinity_ == ax::mojom::TextAffinity::kDownstream;
      case AXPositionKind::TREE_POSITION:
        return GetAnchor() &&
               (child_index_ == BEFORE_TEXT ||
                (child_index_ >= 0 && child_index_ <= AnchorChildCount())) &&
               !IsInDescendantOfEmptyObject();
      case AXPositionKind::TEXT_POSITION:
        if (!GetAnchor() || IsInDescendantOfEmptyObject()) {
          return false;
        }

        // For performance reasons we skip any validation of the text offset
        // that involves retrieving the anchor's text, if the offset is set to
        // 0, because 0 is frequently used and always valid regardless of the
        // actual text.
        return text_offset_ == 0 ||
               (text_offset_ > 0 && text_offset_ <= MaxTextOffset());
    }
  }

  // TODO(nektar): Update logic of AtStartOfAnchor() for text_offset_ == 0 and
  // fix related bug.
  bool AtStartOfAnchor() const {
    if (!GetAnchor())
      return false;
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        if (text_offset_ > 0)
          return false;
        if (!IsLeaf() || text_offset_ == 0)
          return child_index_ == 0;
        return child_index_ == BEFORE_TEXT;
      case AXPositionKind::TEXT_POSITION:
        return text_offset_ == 0;
    }
  }

  bool AtEndOfAnchor() const {
    if (!GetAnchor())
      return false;
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        return child_index_ == AnchorChildCount();
      case AXPositionKind::TEXT_POSITION:
        return text_offset_ == MaxTextOffset();
    }
  }

  bool AtStartOfWord() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        const std::vector<int32_t> word_starts =
            text_position->GetWordStartOffsets();
        return base::Contains(word_starts,
                              int32_t{text_position->text_offset_});
      }
    }
  }

  bool AtEndOfWord() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        const std::vector<int32_t> word_ends =
            text_position->GetWordEndOffsets();
        return base::Contains(word_ends, int32_t{text_position->text_offset_});
      }
    }
  }

  bool AtStartOfLine() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION:
        // We treat a position after some white space that is not connected to
        // any node after it via "next on line ID", to be equivalent to a
        // position before the next line, and therefore as being at start of
        // line.
        //
        // We assume that white space, including but not limited to hard line
        // breaks, might be used to separate lines. For example, an inline text
        // box with just a single space character inside it can be used to
        // represent a soft line break. If an inline text box containing white
        // space separates two lines, it should always be connected to the first
        // line via "kPreviousOnLineId". This is guaranteed by the renderer. If
        // there are multiple line breaks separating the two lines, then only
        // the first line break is connected to the first line via
        // "kPreviousOnLineId".
        //
        // Sometimes there might be an inline text box with a single space in it
        // at the end of a text field. We should not mark positions that are at
        // the end of text fields, or in general at the end of their anchor, as
        // being at the start of line, except when that anchor is an inline text
        // box that is in the middle of a text span. Note that in most but not
        // all cases, the parent of an inline text box is a static text object,
        // whose end signifies the end of the text span. One exception is line
        // breaks.
        if (text_position->AtEndOfAnchor() &&
            !text_position->AtEndOfTextSpan() &&
            text_position->IsInWhiteSpace() &&
            GetNextOnLineID(text_position->anchor_id_) ==
                AXNode::kInvalidAXID) {
          return true;
        }

        return GetPreviousOnLineID(text_position->anchor_id_) ==
                   AXNode::kInvalidAXID &&
               text_position->AtStartOfAnchor();
    }
  }

  bool AtEndOfLine() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION:
        // Text positions on objects with no text should not be considered at
        // end of line because the empty position may share a text offset with
        // a non-empty text position in which case the end of line iterators
        // must move to the line end of the non-empty content. Specified next
        // line IDs are ignored.
        if (!text_position->MaxTextOffset())
          return false;

        // If affinity has been used to specify whether the caret is at the end
        // of a line or at the start of the next one, this should have been
        // reflected in the leaf text position we got via "AsLeafTextPosition".
        // If affinity had been set to upstream, the leaf text position should
        // be pointing to the end of the inline text box that ends the first
        // line. If it had been set to downstream, the leaf text position should
        // be pointing to the start of the inline text box that starts the
        // second line.
        //
        // In other cases, we assume that white space, including but not limited
        // to hard line breaks, might be used to separate lines. For example, an
        // inline text box with just a single space character inside it can be
        // used to represent a soft line break. If an inline text box containing
        // white space separates two lines, it should always be connected to the
        // first line via "kPreviousOnLineId". This is guaranteed by the
        // renderer. If there are multiple line breaks separating the two lines,
        // then only the first line break is connected to the first line via
        // "kPreviousOnLineId".
        //
        // We don't treat a position that is at the start of white space that is
        // on a line by itself as being at the end of the line. This is in order
        // to enable screen readers to recognize and announce blank lines
        // correctly. However, we do treat positions at the start of white space
        // that end a line of text as being at the end of that line. We also
        // treat positions at the end of white space that is on a line by
        // itself, i.e. on a blank line, as being at the end of that line.
        //
        // Sometimes there might be an inline text box with a single space in it
        // at the end of a text field. We should mark positions that are at the
        // end of text fields, or in general at the end of an anchor with no
        // "kNextOnLineId", as being at end of line, except when that anchor is
        // an inline text box that is in the middle of a text span. Note that
        // in most but not all cases, the parent of an inline text box is a
        // static text object, whose end signifies the end of the text span. One
        // exception is line breaks.
        if (GetNextOnLineID(text_position->anchor_id_) ==
            AXNode::kInvalidAXID) {
          return (!text_position->AtEndOfTextSpan() &&
                  text_position->IsInWhiteSpace() &&
                  GetPreviousOnLineID(text_position->anchor_id_) !=
                      AXNode::kInvalidAXID)
                     ? text_position->AtStartOfAnchor()
                     : text_position->AtEndOfAnchor();
        }

        // The current anchor might be followed by a soft line break.
        return text_position->AtEndOfAnchor() &&
               text_position->CreateNextLeafTextPosition()->AtEndOfLine();
    }
  }

  // |AtStartOfParagraph| is asymmetric from |AtEndOfParagraph| because of
  // trailing whitespace collapse rules.
  // The start of a paragraph should be a leaf text position (or equivalent),
  // either at the start of the document, or at the start of the next leaf text
  // position from the one representing the end of the previous paragraph.
  // A position |AsLeafTextPosition| is the start of a paragraph if all of the
  // following are true :
  // 1. The current leaf text position must be an unignored position at
  //    the start of an anchor.
  // 2. The current position is not whitespace only, unless it is also
  //    the first leaf text position within the document.
  // 3. Either (a) the current leaf text position is the first leaf text
  //    position in the document, or (b) there are no line breaking
  //    objects between it and the previous non-whitespace leaf text
  //    position.
  bool AtStartOfParagraph() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        // 1. The current leaf text position must be an unignored position at
        //    the start of an anchor.
        if (text_position->IsIgnored() || !text_position->AtStartOfAnchor())
          return false;

        // 2. The current position is not whitespace only, unless it is also
        //    the first leaf text position within the document.
        if (text_position->IsInWhiteSpace()) {
          return text_position->CreatePreviousLeafTextPosition()
              ->IsNullPosition();
        }

        // 3. Either (a) the current leaf text position is the first leaf text
        //    position in the document, or (b) there are no line breaking
        //    objects between it and the previous non-whitespace leaf text
        //    position.
        //
        // Search for the previous text position within the current paragraph,
        // using the paragraph boundary abort predicate.
        // If a valid position was found, then this position cannot be
        // the start of a paragraph.
        // This will return a null position when an anchor movement would
        // cross a paragraph boundary, or the start of document was reached.
        bool crossed_line_breaking_object_token = false;
        auto abort_move_predicate =
            [&crossed_line_breaking_object_token](
                const AXPosition& move_from, const AXPosition& move_to,
                const AXMoveType type, const AXMoveDirection direction) {
              return AbortMoveAtParagraphBoundary(
                  crossed_line_breaking_object_token, move_from, move_to, type,
                  direction);
            };

        AXPositionInstance previous_text_position = text_position->Clone();
        do {
          previous_text_position =
              previous_text_position->CreatePreviousTextAnchorPosition(
                  abort_move_predicate);
          // If the previous position is whitespace, then continue searching
          // until a non-whitespace leaf text position is found within the
          // current paragraph because whitespace is supposed to be collapsed.
          // There's a chance that |CreatePreviousTextAnchorPosition| will
          // return whitespace that should be appended to a previous paragraph
          // rather than separating two pieces of the current paragraph.
        } while (previous_text_position->IsInWhiteSpace() ||
                 previous_text_position->IsIgnored());
        return previous_text_position->IsNullPosition();
      }
    }
  }

  // |AtEndOfParagraph| is asymmetric from |AtStartOfParagraph| because of
  // trailing whitespace collapse rules.
  // The end of a paragraph should be a leaf text position (or equivalent),
  // either at the end of the document, or at the end of the previous leaf text
  // position from the one representing the start of the next paragraph.
  // A position |AsLeafTextPosition| is the end of a paragraph if all of the
  // following are true :
  // 1. The current leaf text position must be an unignored position at
  //    the end of an anchor.
  // 2. Either (a) the current leaf text position is the last leaf text
  //    position in the document, or (b) there are no line breaking
  //    objects between it and the next leaf text position except when
  //    the next leaf text position is whitespace only since whitespace
  //    must be collapsed.
  // 3. If there is a next leaf text position then it must not be
  //    whitespace only.
  // 4. If there is a next leaf text position and it is not whitespace
  //    only, it must also be the start of a paragraph for the current
  //    position to be the end of a paragraph.
  bool AtEndOfParagraph() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        // 1. The current leaf text position must be an unignored position at
        //    the end of an anchor.
        if (text_position->IsIgnored() || !text_position->AtEndOfAnchor())
          return false;

        // 2. Either (a) the current leaf text position is the last leaf text
        //    position in the document, or (b) there are no line breaking
        //    objects between it and the next leaf text position except when
        //    the next leaf text position is whitespace only since whitespace
        //    must be collapsed.
        //
        // Search for the next text position within the current paragraph,
        // using the paragraph boundary abort predicate.
        // If a null position was found, then this position must be the end of
        // a paragraph.
        // |CreateNextTextAnchorPosition| + |AbortMoveAtParagraphBoundary|
        // will return a null position when an anchor movement would
        // cross a paragraph boundary and there is no doubt that it is the end
        // of a paragraph, or the end of document was reached.
        // There are some fringe cases related to whitespace collapse that
        // cannot be handled easily with only |AbortMoveAtParagraphBoundary|.
        bool crossed_line_breaking_object_token = false;
        auto abort_move_predicate =
            [&crossed_line_breaking_object_token](
                const AXPosition& move_from, const AXPosition& move_to,
                const AXMoveType type, const AXMoveDirection direction) {
              return AbortMoveAtParagraphBoundary(
                  crossed_line_breaking_object_token, move_from, move_to, type,
                  direction);
            };

        AXPositionInstance next_text_position = text_position->Clone();
        do {
          next_text_position = next_text_position->CreateNextTextAnchorPosition(
              abort_move_predicate);
        } while (next_text_position->IsIgnored());
        if (next_text_position->IsNullPosition())
          return true;

        // 3. If there is a next leaf text position then it must not be
        //    whitespace only.
        if (next_text_position->IsInWhiteSpace())
          return false;

        // 4. If there is a next leaf text position and it is not whitespace
        //    only, it must also be the start of a paragraph for the current
        //    position to be the end of a paragraph.
        //
        // Consider the following example :
        // ++{1} kStaticText "First Paragraph"
        // ++++{2} kInlineTextBox "First Paragraph"
        // ++{3} kStaticText "\n Second Paragraph"
        // ++++{4} kInlineTextBox "\n" kIsLineBreakingObject
        // ++++{5} kInlineTextBox " "
        // ++++{6} kInlineTextBox "Second Paragraph"
        // A position at the end of {5} is the end of a paragraph, because
        // the first paragraph must collapse trailing whitespace and contain
        // leaf text anchors {2, 4, 5}. The second paragraph is only {6}.
        return next_text_position->CreatePositionAtStartOfAnchor()
            ->AtStartOfParagraph();
      }
    }
  }

  bool AtStartOfPage() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        if (!text_position->AtStartOfAnchor())
          return false;

        // Search for the previous text position within the current page,
        // using the page boundary abort predicate.
        // If a valid position was found, then this position cannot be
        // the start of a page.
        // This will return a null position when an anchor movement would
        // cross a page boundary, or the start of document was reached.
        AXPositionInstance previous_text_position =
            text_position->CreatePreviousTextAnchorPosition(
                AbortMoveAtPageBoundary);
        return previous_text_position->IsNullPosition();
      }
    }
  }

  bool AtEndOfPage() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        if (!text_position->AtEndOfAnchor())
          return false;

        // Search for the next text position within the current page,
        // using the page boundary abort predicate.
        // If a valid position was found, then this position cannot be
        // the end of a page.
        // This will return a null position when an anchor movement would
        // cross a page boundary, or the end of document was reached.
        AXPositionInstance next_text_position =
            text_position->CreateNextTextAnchorPosition(
                AbortMoveAtPageBoundary);
        return next_text_position->IsNullPosition();
      }
    }
  }

  bool AtStartOfAXTree() const {
    if (IsNullPosition())
      return false;

    if (AtStartOfAnchor()) {
      AXPositionInstance previous_anchor = CreatePreviousAnchorPosition();

      // Consider the start of the document as the start of an AXTree.
      if (previous_anchor->IsNullPosition())
        return true;
      else
        return previous_anchor->tree_id() != tree_id();
    }
    return false;
  }

  bool AtEndOfAXTree() const {
    if (IsNullPosition())
      return false;

    if (AtEndOfAnchor()) {
      AXPositionInstance next_anchor = CreateNextAnchorPosition();

      // Consider the end of the document as the end of an AXTree.
      if (next_anchor->IsNullPosition())
        return true;
      else
        return next_anchor->tree_id() != tree_id();
    }
    return false;
  }

  AXBoundaryType GetFormatStartBoundaryType() const {
    // Since formats are stored on text anchors, the start of a format boundary
    // must be at the start of an anchor.
    if (IsNullPosition() || !AtStartOfAnchor())
      return AXBoundaryType::kNone;

    // Treat the first iterable node as a format boundary.
    if (CreatePreviousLeafTreePosition()->IsNullPosition())
      return AXBoundaryType::kDocumentStart;

    // Ignored positions cannot be format boundaries.
    if (IsIgnored())
      return AXBoundaryType::kNone;

    // Iterate over anchors until a format boundary is found. This will return a
    // null position upon crossing a boundary. Make sure the previous position
    // is not on an ignored node.
    AXPositionInstance previous_position = Clone();
    do {
      previous_position = previous_position->CreatePreviousLeafTreePosition(
          AbortMoveAtFormatBoundary);
    } while (previous_position->IsIgnored());

    if (previous_position->IsNullPosition())
      return AXBoundaryType::kUnitBoundary;

    return AXBoundaryType::kNone;
  }

  bool AtStartOfFormat() const {
    return GetFormatStartBoundaryType() != AXBoundaryType::kNone;
  }

  AXBoundaryType GetFormatEndBoundaryType() const {
    // Since formats are stored on text anchors, the end of a format break must
    // be at the end of an anchor.
    if (IsNullPosition() || !AtEndOfAnchor())
      return AXBoundaryType::kNone;

    // Treat the last iterable node as a format boundary
    if (CreateNextLeafTreePosition()->IsNullPosition())
      return AXBoundaryType::kDocumentEnd;

    // Ignored positions cannot be format boundaries.
    if (IsIgnored())
      return AXBoundaryType::kNone;

    // Iterate over anchors until a format boundary is found. This will return a
    // null position upon crossing a boundary. Make sure the next position is
    // not on an ignored node.
    AXPositionInstance next_position = Clone();
    do {
      next_position =
          next_position->CreateNextLeafTreePosition(AbortMoveAtFormatBoundary);
    } while (next_position->IsIgnored());

    if (next_position->IsNullPosition())
      return AXBoundaryType::kUnitBoundary;

    return AXBoundaryType::kNone;
  }

  bool AtEndOfFormat() const {
    return GetFormatEndBoundaryType() != AXBoundaryType::kNone;
  }

  bool AtStartOfInlineBlock() const {
    AXPositionInstance text_position = AsLeafTextPosition();
    switch (text_position->kind_) {
      case AXPositionKind::NULL_POSITION:
        return false;
      case AXPositionKind::TREE_POSITION:
        BASE_UNREACHABLE();
        return false;
      case AXPositionKind::TEXT_POSITION: {
        if (text_position->AtStartOfAnchor()) {
          AXPositionInstance previous_position =
              text_position->CreatePreviousLeafTreePosition();

          // Check that this position is not the start of the first anchor.
          if (!previous_position->IsNullPosition()) {
            previous_position = text_position->CreatePreviousLeafTreePosition(
                &AbortMoveAtStartOfInlineBlock);

            // If we get a null position here it means we have crossed an inline
            // block's start, thus this position is located at such start.
            if (previous_position->IsNullPosition())
              return true;
          }
        }
        if (text_position->AtEndOfAnchor()) {
          AXPositionInstance next_position =
              text_position->CreateNextLeafTreePosition();

          // Check that this position is not the end of the last anchor.
          if (!next_position->IsNullPosition()) {
            next_position = text_position->CreateNextLeafTreePosition(
                &AbortMoveAtStartOfInlineBlock);

            // If we get a null position here it means we have crossed an inline
            // block's start, thus this position is located at such start.
            if (next_position->IsNullPosition())
              return true;
          }
        }
        return false;
      }
    }
  }

  bool AtStartOfDocument() const {
    if (IsNullPosition())
      return false;
    return IsDocument(GetAnchorRole()) && AtStartOfAnchor();
  }

  bool AtEndOfDocument() const {
    if (IsNullPosition())
      return false;
    return AtLastNodeInTree() && AtEndOfAnchor();
  }

  bool AtLastNodeInTree() const {
    if (IsNullPosition())
      return false;

    // Avoid a potentionally expensive MaxTextOffset call by only using tree
    // positions. The only thing that matters is whether our anchor_id_ is at
    // the last anchor of the document, so we're free to ignore text_offset_.
    AXPositionInstance tree_position =
        CreateTreePosition(tree_id_, anchor_id_, 0);
    return tree_position->CreateNextAnchorPosition()->IsNullPosition();
  }

  // This method finds the lowest common AXNodeType of |this| and |second|.
  AXNodeType* LowestCommonAnchor(const AXPosition& second) const {
    if (IsNullPosition() || second.IsNullPosition())
      return nullptr;
    if (GetAnchor() == second.GetAnchor())
      return GetAnchor();

    std::stack<AXNodeType*> our_ancestors = GetAncestorAnchors();
    std::stack<AXNodeType*> other_ancestors = second.GetAncestorAnchors();

    AXNodeType* common_anchor = nullptr;
    while (!our_ancestors.empty() && !other_ancestors.empty() &&
           our_ancestors.top() == other_ancestors.top()) {
      common_anchor = our_ancestors.top();
      our_ancestors.pop();
      other_ancestors.pop();
    }
    return common_anchor;
  }

  // This method returns a position instead of a node because this allows us to
  // return the corresponding text offset or child index in the ancestor that
  // relates to the current position.
  // Also, this method uses position instead of tree logic to traverse the tree,
  // because positions can handle moving across multiple trees, while trees
  // cannot.
  AXPositionInstance LowestCommonAncestor(const AXPosition& second) const {
    return CreateAncestorPosition(LowestCommonAnchor(second));
  }

  // See "CreateParentPosition" for an explanation of the use of
  // |move_direction|.
  AXPositionInstance CreateAncestorPosition(
      const AXNodeType* ancestor_anchor,
      ax::mojom::MoveDirection move_direction =
          ax::mojom::MoveDirection::kForward) const {
    if (!ancestor_anchor) {
      return CreateNullPosition();
    }

    AXPositionInstance ancestor_position = Clone();
    while (!ancestor_position->IsNullPosition() &&
           ancestor_position->GetAnchor() != ancestor_anchor) {
      ancestor_position =
          ancestor_position->CreateParentPosition(move_direction);
    }
    return ancestor_position;
  }

  // If the position is not valid, we return a new valid position that is
  // closest to the original position if possible, or a null position otherwise.
  AXPositionInstance AsValidPosition() const {
    AXPositionInstance position = Clone();
    switch (position->kind_) {
      case AXPositionKind::NULL_POSITION:
        // We avoid cloning to ensure that all fields will be valid.
        return CreateNullPosition();
      case AXPositionKind::TREE_POSITION: {
        if (!position->GetAnchor())
          return CreateNullPosition();

        if (AXNodeType* empty_object_node = GetEmptyObjectAncestorNode()) {
          // In this class and on certain platforms, we define the empty object
          // as one that doesn't expose its underlying content. Its content is
          // replaced by the empty object character (string of length 1). A
          // position on a descendant of an empty object is invalid. To make it
          // valid we move the position from the descendant to the empty object
          // node itself.
          return CreateTreePosition(
              position->tree_id(), GetAnchorID(empty_object_node),
              position->child_index() == BEFORE_TEXT ? BEFORE_TEXT : 0);
        }

        if (position->child_index_ == BEFORE_TEXT)
          return position;

        if (position->child_index_ < 0)
          position->child_index_ = 0;
        else if (position->child_index_ > position->AnchorChildCount())
          position->child_index_ = position->AnchorChildCount();
        break;
      }
      case AXPositionKind::TEXT_POSITION: {
        if (!position->GetAnchor())
          return CreateNullPosition();

        if (AXNodeType* empty_object_node = GetEmptyObjectAncestorNode()) {
          // This is needed because an empty object as defined in this class and
          // on certain platforms can have descendants that should not be
          // exposed. See comment above in similar implementation for
          // AXPositionKind::TREE_POSITION.
          //
          // We set the |text_offset_| to either 0 or 1 here because the
          // MaxTextOffset of an empty object is 1 (the empty object character,
          // a string of length 1). If the invalid position was already at the
          // start of the node, we set it to 0.
          return CreateTextPosition(position->tree_id(),
                                    GetAnchorID(empty_object_node),
                                    position->text_offset() > 0 ? 1 : 0,
                                    ax::mojom::TextAffinity::kDownstream);
        }

        if (position->text_offset_ <= 0) {
          // 0 is always a valid offset, so skip calling MaxTextOffset in that
          // case.
          position->text_offset_ = 0;
          position->affinity_ = ax::mojom::TextAffinity::kDownstream;
        } else {
          int max_text_offset = position->MaxTextOffset();
          if (position->text_offset_ > max_text_offset) {
            position->text_offset_ = max_text_offset;
            position->affinity_ = ax::mojom::TextAffinity::kDownstream;
          }
        }
        break;
      }
    }
    BASE_DCHECK(position->IsValid());
    return position;
  }

  AXPositionInstance AsTreePosition() const {
    if (IsNullPosition() || IsTreePosition())
      return Clone();

    AXPositionInstance copy = Clone();
    BASE_DCHECK(copy);
    BASE_DCHECK(copy->text_offset_ >= 0);
    if (copy->IsLeaf()) {
      const int max_text_offset = copy->MaxTextOffset();
      copy->child_index_ =
          (max_text_offset != 0 && copy->text_offset_ != max_text_offset)
              ? BEFORE_TEXT
              : 0;
      copy->kind_ = AXPositionKind::TREE_POSITION;
      return copy;
    }

    // We stop at the last child that we can reach with the current text offset
    // and ignore any remaining children. This is for defensive programming
    // purposes, in case "MaxTextOffset" doesn't match the total length of all
    // our children. This may happen if, for example, there is a bug in the
    // internal accessibility tree we get from the renderer. In contrast, the
    // current offset could not be greater than the length of all our children
    // because the position would have been invalid.
    int current_offset = 0;
    int child_index = 0;
    for (; child_index < copy->AnchorChildCount(); ++child_index) {
      AXPositionInstance child = copy->CreateChildPositionAt(child_index);
      BASE_DCHECK(child);
      int child_length = child->MaxTextOffsetInParent();
      // If the text offset falls on the boundary between two adjacent children,
      // we look at the affinity to decide whether to place the tree position on
      // the first child vs. the second child. Upstream affinity would always
      // choose the first child, whilst downstream affinity the second. This
      // also has implications when converting the resulting tree position back
      // to a text position. In that case, maintaining an upstream affinity
      // would place the text position at the end of the first child, whilst
      // maintaining a downstream affinity will place the text position at the
      // beginning of the second child.
      //
      // This is vital for text positions on soft line breaks, as well as text
      // positions before and after character, to work properly.
      //
      // See also `CreateLeafTextPositionBeforeCharacter` and
      // `CreateLeafTextPositionAfterCharacter`.
      if (copy->text_offset_ >= current_offset &&
          (copy->text_offset_ < (current_offset + child_length) ||
           (copy->affinity_ == ax::mojom::TextAffinity::kUpstream &&
            copy->text_offset_ == (current_offset + child_length)))) {
        break;
      }

      current_offset += child_length;
    }

    copy->child_index_ = child_index;
    copy->kind_ = AXPositionKind::TREE_POSITION;
    return copy;
  }

  // This is an optimization over "AsLeafTextPosition", in cases when computing
  // the corresponding text offset on the leaf node is not needed. If this
  // method is called on a text position, it will conservatively fall back to
  // the non-optimized "AsLeafTextPosition", if the current text offset is
  // greater than 0, or the affinity is upstream, since converting to a tree
  // position at any point before reaching the leaf node could potentially lose
  // information.
  AXPositionInstance AsLeafTreePosition() const {
    if (IsNullPosition() || IsLeaf())
      return AsTreePosition();

    // If our text offset is greater than 0, or if our affinity is set to
    // upstream, we need to ensure that text offset and affinity will be taken
    // into consideration during our descend to the leaves. Switching to a tree
    // position early in this case will potentially lose information, so we
    // descend using a text position instead.
    //
    // We purposely don't check whether this position is a text position, to
    // allow for the possibility that this position has recently been converted
    // from a text to a tree position and text offset or affinity information
    // has been left intact.
    if (text_offset_ > 0 || affinity_ == ax::mojom::TextAffinity::kUpstream)
      return AsLeafTextPosition()->AsTreePosition();

    AXPositionInstance tree_position = AsTreePosition();
    do {
      if (tree_position->child_index_ == tree_position->AnchorChildCount()) {
        tree_position =
            tree_position
                ->CreateChildPositionAt(tree_position->child_index_ - 1)
                ->CreatePositionAtEndOfAnchor();
      } else {
        tree_position =
            tree_position->CreateChildPositionAt(tree_position->child_index_);
      }
      BASE_DCHECK(tree_position && !tree_position->IsNullPosition());
    } while (!tree_position->IsLeaf());

    BASE_DCHECK(tree_position && tree_position->IsLeafTreePosition());
    return tree_position;
  }

  AXPositionInstance AsTextPosition() const {
    if (IsNullPosition() || IsTextPosition())
      return Clone();

    AXPositionInstance copy = Clone();
    BASE_DCHECK(copy);
    // Check if it is a "before text" position.
    if (copy->child_index_ == BEFORE_TEXT) {
      // "Before text" positions can only appear on leaf nodes.
      BASE_DCHECK(copy->IsLeaf());
      // If the current text offset is valid, we don't touch it to potentially
      // allow converting from a text position to a tree position and back
      // without losing information.
      //
      // We test for INVALID_OFFSET first, due to the possible performance
      // implications of calling MaxTextOffset().
      BASE_DCHECK(copy->text_offset_ >= INVALID_OFFSET);
      if (copy->text_offset_ == INVALID_OFFSET ||
          (copy->text_offset_ > 0 &&
           copy->text_offset_ >= copy->MaxTextOffset())) {
        copy->text_offset_ = 0;
      }
    } else if (copy->child_index_ == copy->AnchorChildCount()) {
      copy->text_offset_ = copy->MaxTextOffset();
    } else {
      BASE_DCHECK(copy->child_index_ >= 0);
      BASE_DCHECK(copy->child_index_ < copy->AnchorChildCount());
      int new_offset = 0;
      for (int i = 0; i <= child_index_; ++i) {
        AXPositionInstance child = copy->CreateChildPositionAt(i);
        BASE_DCHECK(child);
        // If the current text offset is valid, we don't touch it to
        // potentially allow converting from a text position to a tree
        // position and back without losing information. Otherwise, if the
        // text_offset is invalid, equals to 0 or is smaller than
        // |new_offset|, we reset it to the beginning of the current child
        // node.
        if (i == child_index_ && copy->text_offset_ <= new_offset) {
          copy->text_offset_ = new_offset;
          break;
        }

        int child_length = child->MaxTextOffsetInParent();
        // Same comment as above: we don't touch the text offset if it's
        // already valid.
        if (i == child_index_ &&
            (copy->text_offset_ > (new_offset + child_length) ||
             // When the text offset is equal to the text's length but this is
             // not an "after text" position.
             (!copy->AtEndOfAnchor() &&
              copy->text_offset_ == (new_offset + child_length)))) {
          copy->text_offset_ = new_offset;
          break;
        }

        new_offset += child_length;
      }
    }

    // Affinity should always be left as downstream. The only case when the
    // resulting text position is at the end of the line is when we get an
    // "after text" leaf position, but even in this case downstream is
    // appropriate because there is no ambiguity whetehr the position is at the
    // end of the current line vs. the start of the next line. It would always
    // be the former.
    copy->kind_ = AXPositionKind::TEXT_POSITION;
    return copy;
  }

  AXPositionInstance AsLeafTextPosition() const {
    if (IsNullPosition() || IsLeaf()) {
      return AsTextPosition();
    }

    // Adjust the text offset.
    // No need to check for "before text" positions here because they are only
    // present on leaf anchor nodes.
    AXPositionInstance text_position = AsTextPosition();
    int adjusted_offset = text_position->text_offset_;
    do {
      AXPositionInstance child_position =
          text_position->CreateChildPositionAt(0);
      BASE_DCHECK(child_position);

      // If the text offset corresponds to multiple child positions because some
      // of the children have empty text, the condition "adjusted_offset > 0"
      // below ensures that the first child will be chosen.
      for (int i = 1;
           i < text_position->AnchorChildCount() && adjusted_offset > 0; ++i) {
        const int max_text_offset_in_parent =
            child_position->MaxTextOffsetInParent();
        if (adjusted_offset < max_text_offset_in_parent) {
          break;
        }
        if (affinity_ == ax::mojom::TextAffinity::kUpstream &&
            adjusted_offset == max_text_offset_in_parent) {
          // Maintain upstream affinity so that we'll be able to choose the
          // correct leaf anchor if the text offset is right on the boundary
          // between two leaves.
          child_position->affinity_ = ax::mojom::TextAffinity::kUpstream;
          break;
        }
        child_position = std::move(text_position->CreateChildPositionAt(i));
        adjusted_offset -= max_text_offset_in_parent;
      }

      text_position = std::move(child_position);
    } while (!text_position->IsLeaf());

    BASE_DCHECK(text_position);
    BASE_DCHECK(text_position->IsLeafTextPosition());
    text_position->text_offset_ = adjusted_offset;
    // A leaf Text position is always downstream since there is no ambiguity as
    // to whether it refers to the end of the current or the start of the next
    // line.
    text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
    return text_position;
  }

  // We deploy three strategies in order to find the best match for an ignored
  // position in the accessibility tree:
  //
  // 1. In the case of a text position, we move up the parent positions until we
  // find the next unignored equivalent parent position. We don't do this for
  // tree positions because, unlike text positions which maintain the
  // corresponding text offset in the inner text of the parent node, tree
  // positions would lose some information every time a parent position is
  // computed. In other words, the parent position of a tree position is, in
  // most cases, non-equivalent to the child position.
  // 2. If no equivalent and unignored parent position can be computed, we try
  // computing the leaf equivalent position. If this is unignored, we return it.
  // This can happen both for tree and text positions, provided that the leaf
  // node and its inner text is visible to platform APIs, i.e. it's unignored.
  // 3. As a last resort, we move either to the next or previous unignored
  // position in the accessibility tree, based on the "adjustment_behavior".
  AXPositionInstance AsUnignoredPosition(
      AXPositionAdjustmentBehavior adjustment_behavior) const {
    if (IsNullPosition() || !IsIgnored())
      return Clone();

    AXPositionInstance leaf_tree_position = AsLeafTreePosition();

    // If this is a text position, first try moving up to a parent equivalent
    // position and check if the resulting position is still ignored. This
    // won't result in the loss of any information. We can't do that in the
    // case of tree positions, because we would be better off to move to the
    // next or previous position within the same anchor, as this would lose
    // less information than moving to a parent equivalent position.
    //
    // Text positions are considered ignored if either the current anchor is
    // ignored, or if the equivalent leaf tree position is ignored.
    // If this position is a leaf text position, or the equivalent leaf tree
    // position is ignored, then it's not possible to create an ancestor text
    // position that is unignored.
    if (IsTextPosition() && !IsLeafTextPosition() &&
        !leaf_tree_position->IsIgnored()) {
      AXPositionInstance unignored_position = CreateParentPosition();
      while (!unignored_position->IsNullPosition()) {
        // Since the equivalent leaf tree position is unignored, search for the
        // first unignored ancestor anchor and return that text position.
        if (!unignored_position->GetAnchor()->IsIgnored()) {
          BASE_DCHECK(!unignored_position->IsIgnored());
          return unignored_position;
        }
        unignored_position = unignored_position->CreateParentPosition();
      }
    }

    // There is a possibility that the position became unignored by moving to a
    // leaf equivalent position. Otherwise, we have no choice but to move to the
    // next or previous position and lose some information in the process.
    while (leaf_tree_position->IsIgnored()) {
      switch (adjustment_behavior) {
        case AXPositionAdjustmentBehavior::kMoveForward:
          leaf_tree_position = leaf_tree_position->CreateNextLeafTreePosition();
          break;
        case AXPositionAdjustmentBehavior::kMoveBackward:
          leaf_tree_position =
              leaf_tree_position->CreatePreviousLeafTreePosition();
          // in case the unignored leaf node contains some text, ensure that the
          // resulting position is an "after text" position, as such a position
          // would be the closest to the ignored one, given the fact that we are
          // moving backwards through the tree.
          leaf_tree_position =
              leaf_tree_position->CreatePositionAtEndOfAnchor();
          break;
      }
    }

    if (IsTextPosition())
      return leaf_tree_position->AsTextPosition();
    return leaf_tree_position;
  }

  // Searches backward and forward from this position until it finds the given
  // text boundary, and creates an AXRange that spans from the former to the
  // latter. The resulting AXRange is always a forward range: its anchor always
  // comes before its focus in document order. The resulting AXRange is bounded
  // by the anchor of this position, i.e. the AXBoundaryBehavior is set to
  // StopAtAnchorBoundary. The exception is ax::mojom::TextBoundary::kWebPage,
  // where this behavior won't make sense. This behavior is based on current
  // platform needs and might be relaxed if necessary in the future.
  //
  // Please note that |expand_behavior| should have no effect for
  // ax::mojom::TextBoundary::kObject and ax::mojom::TextBoundary::kWebPage
  // because the range should be the same regardless if we first move left or
  // right.
  AXRangeType ExpandToEnclosingTextBoundary(
      ax::mojom::TextBoundary boundary,
      AXRangeExpandBehavior expand_behavior) const {
    AXBoundaryBehavior boundary_behavior =
        AXBoundaryBehavior::StopAtAnchorBoundary;
    if (boundary == ax::mojom::TextBoundary::kWebPage)
      boundary_behavior = AXBoundaryBehavior::CrossBoundary;

    switch (expand_behavior) {
      case AXRangeExpandBehavior::kLeftFirst: {
        AXPositionInstance left_position = CreatePositionAtTextBoundary(
            boundary, ax::mojom::MoveDirection::kBackward, boundary_behavior);
        AXPositionInstance right_position =
            left_position->CreatePositionAtTextBoundary(
                boundary, ax::mojom::MoveDirection::kForward,
                boundary_behavior);
        return AXRangeType(std::move(left_position), std::move(right_position));
      }
      case AXRangeExpandBehavior::kRightFirst: {
        AXPositionInstance right_position = CreatePositionAtTextBoundary(
            boundary, ax::mojom::MoveDirection::kForward, boundary_behavior);
        AXPositionInstance left_position =
            right_position->CreatePositionAtTextBoundary(
                boundary, ax::mojom::MoveDirection::kBackward,
                boundary_behavior);
        return AXRangeType(std::move(left_position), std::move(right_position));
      }
    }
  }

  // Starting from this position, moves in the given direction until it finds
  // the given text boundary, and creates a new position at that location.
  //
  // When a boundary has the "StartOrEnd" suffix, it means that this method will
  // find the start boundary when moving in the backward direction, and the end
  // boundary when moving in the forward direction.
  AXPositionInstance CreatePositionAtTextBoundary(
      ax::mojom::TextBoundary boundary,
      ax::mojom::MoveDirection direction,
      AXBoundaryBehavior boundary_behavior) const {
    AXPositionInstance resulting_position = CreateNullPosition();
    switch (boundary) {
      case ax::mojom::TextBoundary::kCharacter:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousCharacterPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextCharacterPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kFormat:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousFormatStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextFormatEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kLineEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousLineEndPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextLineEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kLineStart:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousLineStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextLineStartPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kLineStartOrEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousLineStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextLineEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kObject:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position = CreatePositionAtStartOfAnchor();
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreatePositionAtEndOfAnchor();
            break;
        }
        break;

      case ax::mojom::TextBoundary::kPageEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousPageEndPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextPageEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kPageStart:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousPageStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextPageStartPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kPageStartOrEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousPageStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextPageEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kParagraphEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousParagraphEndPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position =
                CreateNextParagraphEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kParagraphStart:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousParagraphStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position =
                CreateNextParagraphStartPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kParagraphStartOrEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousParagraphStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position =
                CreateNextParagraphEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kSentenceEnd:
        BASE_LOG() << "Sentence boundaries are not yet supported.";
        BASE_UNREACHABLE();
        return CreateNullPosition();

      case ax::mojom::TextBoundary::kSentenceStart:
        BASE_LOG() << "Sentence boundaries are not yet supported.";
        BASE_UNREACHABLE();
        return CreateNullPosition();

      case ax::mojom::TextBoundary::kSentenceStartOrEnd:
        BASE_LOG() << "Sentence boundaries are not yet supported.";
        BASE_UNREACHABLE();
        return CreateNullPosition();

      case ax::mojom::TextBoundary::kWebPage:
        if (boundary_behavior != AXBoundaryBehavior::CrossBoundary) {
          BASE_LOG() << "We can't reach the start of the document if we "
                        "are disallowed "
                        "from crossing boundaries.";
          BASE_UNREACHABLE();
        }
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position = CreatePositionAtStartOfDocument();
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreatePositionAtEndOfDocument();
            break;
        }
        break;

      case ax::mojom::TextBoundary::kWordEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousWordEndPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextWordEndPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kWordStart:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousWordStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextWordStartPosition(boundary_behavior);
            break;
        }
        break;

      case ax::mojom::TextBoundary::kWordStartOrEnd:
        switch (direction) {
          case ax::mojom::MoveDirection::kBackward:
            resulting_position =
                CreatePreviousWordStartPosition(boundary_behavior);
            break;
          case ax::mojom::MoveDirection::kForward:
            resulting_position = CreateNextWordEndPosition(boundary_behavior);
            break;
        }
        break;
    }
    return resulting_position;
  }

  AXPositionInstance CreatePositionAtStartOfAnchor() const {
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return CreateNullPosition();
      case AXPositionKind::TREE_POSITION:
        if (IsLeaf())
          return CreateTreePosition(tree_id_, anchor_id_, BEFORE_TEXT);
        return CreateTreePosition(tree_id_, anchor_id_, 0 /* child_index */);
      case AXPositionKind::TEXT_POSITION:
        return CreateTextPosition(tree_id_, anchor_id_, 0 /* text_offset */,
                                  ax::mojom::TextAffinity::kDownstream);
    }
    return CreateNullPosition();
  }

  AXPositionInstance CreatePositionAtEndOfAnchor() const {
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        return CreateNullPosition();
      case AXPositionKind::TREE_POSITION:
        return CreateTreePosition(
            tree_id_, anchor_id_,
            IsEmptyObjectReplacedByCharacter() ? 0 : AnchorChildCount());
      case AXPositionKind::TEXT_POSITION:
        return CreateTextPosition(tree_id_, anchor_id_, MaxTextOffset(),
                                  ax::mojom::TextAffinity::kDownstream);
    }
    return CreateNullPosition();
  }

  AXPositionInstance CreatePositionAtStartOfAXTree() const {
    if (IsNullPosition() || AtStartOfAXTree())
      return Clone();

    // First check for positions on nodes which are AXTree boundaries, but where
    // the text position on that node is not at the start of the anchor.
    if (CreatePositionAtStartOfAnchor()->AtStartOfAXTree())
      return CreatePositionAtStartOfAnchor();

    // Iterate over tree positions until a boundary is reached.
    AXPositionInstance previous_position = AsTreePosition();
    do {
      previous_position = previous_position->CreatePreviousAnchorPosition();
    } while (!previous_position->AtStartOfAXTree());

    // This method should not cross tree boundaries.
    BASE_DCHECK(previous_position->tree_id() == tree_id());

    if (IsTextPosition())
      previous_position = previous_position->AsTextPosition();
    return previous_position;
  }

  AXPositionInstance CreatePositionAtEndOfAXTree() const {
    if (IsNullPosition() || AtEndOfAXTree())
      return Clone();

    // First check for positions on nodes which are AXTree boundaries, but where
    // the text position on that node is not at the end of the anchor.
    if (CreatePositionAtEndOfAnchor()->AtEndOfAXTree())
      return CreatePositionAtEndOfAnchor();

    // Iterate over tree positions until a boundary is reached.
    AXPositionInstance next_position = AsTreePosition();
    do {
      next_position = next_position->CreateNextAnchorPosition()
                          ->CreatePositionAtEndOfAnchor();
    } while (!next_position->AtEndOfAXTree());

    // This method should not cross tree boundaries.
    BASE_DCHECK(next_position->tree_id() == tree_id());

    if (IsTextPosition())
      next_position = next_position->AsTextPosition();
    return next_position->CreatePositionAtEndOfAnchor();
  }

  // "document" is defined here as a single, top-level, navigatable unit from
  //  a user's perspective. This means that all iframes are part of a single
  // "document" that contains the top-level navigatable page. So this method
  // will break out of an iframe and return a position at the start of the
  // top-level document.
  //
  // Note that this definition is different than HTML's definition of
  // "document", where each iframe has its own document object. For a similar
  // method that stops at iframe boundaries, see
  // CreatePositionAtStartOfAXTree().
  AXPositionInstance CreatePositionAtStartOfDocument() const {
    AXPositionInstance position =
        AsTreePosition()->CreateDocumentAncestorPosition();
    if (!position->IsNullPosition()) {
      position = position->CreatePositionAtStartOfAnchor();
      if (IsTextPosition())
        position = position->AsTextPosition();
    }
    return position;
  }

  // "document" is defined here as a single, top-level, navigatable unit from
  //  a user's perspective. This means that all iframes are part of a single
  // "document" that contains the top-level navigatable page. So this method
  // will break out of an iframe and return a position at the end of the
  // top-level document.
  //
  // Note that this definition is different than HTML's definition of
  // "document", where each iframe has its own document object. For a similar
  // method that stops at iframe boundaries, see CreatePositionAtEndOfAXTree().
  AXPositionInstance CreatePositionAtEndOfDocument() const {
    AXPositionInstance position =
        AsTreePosition()->CreateDocumentAncestorPosition();
    if (!position->IsNullPosition()) {
      while (!position->IsLeaf()) {
        position =
            position->CreateChildPositionAt(position->AnchorChildCount() - 1);
      }
      position = position->CreatePositionAtEndOfAnchor();
      if (IsTextPosition())
        position = position->AsTextPosition();
    }
    return position;
  }

  AXPositionInstance CreateChildPositionAt(int child_index) const {
    if (IsNullPosition() || IsLeaf())
      return CreateNullPosition();

    if (child_index < 0 || child_index >= AnchorChildCount())
      return CreateNullPosition();

    AXTreeID tree_id = AXTreeIDUnknown();
    AXNode::AXID child_id = AXNode::kInvalidAXID;
    AnchorChild(child_index, &tree_id, &child_id);
    BASE_DCHECK(tree_id != AXTreeIDUnknown());
    BASE_DCHECK(child_id != AXNode::kInvalidAXID);
    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        BASE_UNREACHABLE();
        return CreateNullPosition();
      case AXPositionKind::TREE_POSITION: {
        AXPositionInstance child_position =
            CreateTreePosition(tree_id, child_id, 0 /* child_index */);
        // If the child's anchor is a leaf node, make this a "before text"
        // position.
        if (child_position->IsLeaf())
          child_position->child_index_ = BEFORE_TEXT;
        return child_position;
      }
      case AXPositionKind::TEXT_POSITION:
        return CreateTextPosition(tree_id, child_id, 0 /* text_offset */,
                                  ax::mojom::TextAffinity::kDownstream);
    }

    return CreateNullPosition();
  }

  // Creates a parent equivalent position.
  //
  // "move_direction" is used only in the case of a text position, when in
  // the process of searching for a text boundary, and on platforms where child
  // nodes are represented by embedded object characters. On such platforms, the
  // "IsEmbeddedObjectInParent" method returns true. We need to decide whether
  // to create a parent equivalent position that is before or after the child
  // node, since moving to a parent position would always cause us to lose some
  // information. We can't simply re-use the text offset of the child position
  // because by definition the parent node doesn't include all the text of the
  // child node, but only a single embedded object character.
  //
  // staticText name='Line one' IA2-hypertext='<embedded_object>'
  // ++inlineTextBox name='Line one'
  //
  // If we are given a text position pointing to somewhere inside the
  // inlineTextBox, and we move to the parent equivalent position, we need to
  // decide whether the parent position would be set to point to before the
  // embedded object character or after it. Both are valid, depending on the
  // direction on motion, e.g. if we are trying to find the start of the line
  // vs. the end of the line.
  AXPositionInstance CreateParentPosition(
      ax::mojom::MoveDirection move_direction =
          ax::mojom::MoveDirection::kForward) const {
    if (IsNullPosition())
      return CreateNullPosition();

    AXTreeID tree_id = AXTreeIDUnknown();
    AXNode::AXID parent_id = AXNode::kInvalidAXID;
    AnchorParent(&tree_id, &parent_id);
    if (tree_id == AXTreeIDUnknown() || parent_id == AXNode::kInvalidAXID)
      return CreateNullPosition();

    switch (kind_) {
      case AXPositionKind::NULL_POSITION:
        BASE_UNREACHABLE();
        return CreateNullPosition();
      case AXPositionKind::TREE_POSITION:
        return CreateTreePosition(tree_id, parent_id, AnchorIndexInParent());
      case AXPositionKind::TEXT_POSITION: {
        // On some platforms, such as Android, Mac and Chrome OS, the inner text
        // of a node is made up by concatenating the text of child nodes. On
        // other platforms, such as Windows IA2 and Linux ATK, child nodes are
        // represented by a single embedded object character.
        //
        // If our parent's inner text is a concatenation of all its children's
        // text, we need to maintain the affinity and compute the corresponding
        // text offset. Otherwise, we have no choice but to return a position
        // that is either before or after this child, losing some information in
        // the process. Regardless to whether our parent contains all our text,
        // we always recompute the affinity when the position is after the
        // child.
        //
        // Recomputing the affinity in the latter situation is important because
        // even though a text position might unambiguously be at the end of a
        // line, its parent position might be the same as the parent position of
        // a position that represents the start of the next line. For example:
        //
        // staticText name='Line oneLine two'
        // ++inlineTextBox name='Line one'
        // ++inlineTextBox name='Line two'
        //
        // If the original position is at the end of the inline text box for
        // "Line one", then the resulting parent equivalent position would be
        // the same as the one that would have been computed if the original
        // position were at the start of the inline text box for "Line two".
        const int max_text_offset = MaxTextOffset();
        int max_text_offset_in_parent =
            IsEmbeddedObjectInParent() ? 1 : max_text_offset;
        int parent_offset = AnchorTextOffsetInParent();
        ax::mojom::TextAffinity parent_affinity = affinity_;
        if (max_text_offset == max_text_offset_in_parent) {
          // Our parent contains all our text. No information would be lost when
          // moving to a parent equivalent position.
          parent_offset += text_offset_;
        } else if (text_offset_ > 0) {
          // If "text_offset_" == 0, then the child position is clearly before
          // any embedded object character. No information would be lost when
          // moving to a parent equivalent position, including affinity
          // information. Otherwise, we should decide whether to set the parent
          // position to be before or after the child, based on the direction of
          // motion, and also reset the affinity.
          switch (move_direction) {
            case ax::mojom::MoveDirection::kBackward:
              // Keep the offset to be right before the embedded object
              // character.
              break;
            case ax::mojom::MoveDirection::kForward:
              // Set the offset to be after the embedded object character.
              parent_offset += max_text_offset_in_parent;
              break;
          }

          // The original affinity doesn't apply any more. In most cases, it
          // should be downstream, unless there is an ambiguity as to whether
          // the parent position is between the end of one line and the start of
          // the next. We perform this check below.
          parent_affinity = ax::mojom::TextAffinity::kDownstream;
        }

        // This dummy position serves to retrieve the max text offset of the
        // anchor-node in which we want to create the parent position.
        AXPositionInstance dummy_position =
            CreateTextPosition(tree_id, parent_id, 0, parent_affinity);
        max_text_offset_in_parent = dummy_position->MaxTextOffset();
        if (parent_offset > max_text_offset_in_parent) {
          parent_offset = max_text_offset_in_parent;
        }
        AXPositionInstance parent_position = CreateTextPosition(
            tree_id, parent_id, parent_offset, parent_affinity);

        // If the current position is pointing at the end of its anchor, we need
        // to check if the parent position has introduced ambiguity as to
        // whether it refers to the end of a line or the start of the next.
        // Ambiguity is only present when the parent position points to a text
        // offset that is neither at the start nor at the end of its anchor. We
        // check for ambiguity by creating the parent position and testing if it
        // is erroneously at the start of the next line. Given that the current
        // position, by the nature of being at the end of its anchor, could only
        // be at end of line, the fact that the parent position is also
        // determined to be at start of line demonstrates the presence of
        // ambiguity which is resolved by setting its affinity to upstream.
        //
        // We could not have
        // checked if the child was at the end of the line, because our
        // "AtEndOfLine" predicate takes into account trailing line breaks,
        // which would create false positives.
        if (text_offset_ == max_text_offset &&
            !parent_position->AtEndOfAnchor() &&
            parent_position->AtStartOfLine()) {
          parent_position->affinity_ = ax::mojom::TextAffinity::kUpstream;
        }
        return parent_position;
      }
    }

    return CreateNullPosition();
  }

  // Creates a tree position using the next text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreateNextLeafTreePosition() const {
    return CreateNextLeafTreePosition(&DefaultAbortMovePredicate);
  }

  // Creates a tree position using the previous text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreatePreviousLeafTreePosition() const {
    return CreatePreviousLeafTreePosition(&DefaultAbortMovePredicate);
  }

  // Creates the next text position anchored at a leaf node of the AXTree.
  //
  // If a pointer |crossed_line_breaking_object| is provided, it'll be set to
  // |true| if any line breaking object boundary was crossed by moving from this
  // leaf text position to the next (if it exists), |false| otherwise.
  AXPositionInstance CreateNextLeafTextPosition(
      bool* crossed_line_breaking_object = nullptr) const {
    if (crossed_line_breaking_object)
      *crossed_line_breaking_object = false;

    // If this is an ancestor text position, resolve to its leaf text position.
    if (IsTextPosition() && !IsLeaf())
      return AsLeafTextPosition();
    std::function<AbortMovePredicate> abort_move_predicate;
    if (crossed_line_breaking_object) {
      abort_move_predicate = [crossed_line_breaking_object](
                                 const AXPosition& move_from,
                                 const AXPosition& move_to,
                                 const AXMoveType type,
                                 const AXMoveDirection direction) {
        return UpdateCrossedLineBreakingObjectToken(
            *crossed_line_breaking_object, move_from, move_to, type, direction);
      };
    } else {
      abort_move_predicate =
          [](const AXPosition& move_from, const AXPosition& move_to,
             const AXMoveType type, const AXMoveDirection direction) {
            return AXPosition::DefaultAbortMovePredicate(move_from, move_to,
                                                         type, direction);
          };
    }
    return CreateNextLeafTreePosition(abort_move_predicate)->AsTextPosition();
  }

  // Creates a text position using the previous text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreatePreviousLeafTextPosition() const {
    return CreatePreviousTextAnchorPosition(DefaultAbortMovePredicate);
  }

  // Returns a text position located right before the next character (from this
  // position) in the tree's text representation, following these conditions:
  //
  //   - If this position is at the end of its anchor, normalize it to the start
  //   of the next text anchor, regardless of the position's affinity.
  //   Both text positions are equal when compared, but we consider the start of
  //   an anchor to be a position BEFORE its first character and the end of the
  //   previous to be AFTER its last character.
  //
  //   - Skip any empty text anchors; they're "invisible" to the text
  //   representation and the next character could be ahead.
  //
  //   - Return a null position if there is no next character forward.
  //
  // If possible, return a position anchored at the current position's anchor;
  // this is necessary because we don't want to return any position that might
  // be located in the shadow DOM or in a position anchored at a node that is
  // not visible to a specific platform's APIs.
  //
  // Also, |text_offset| is adjusted to point to a valid character offset, i.e.
  // it cannot be pointing to a low surrogate pair or to the middle of a
  // grapheme cluster.
  AXPositionInstance AsLeafTextPositionBeforeCharacter() const {
    if (IsNullPosition())
      return Clone();

    AXPositionInstance text_position = AsTextPosition();
    // In case the input affinity is upstream, reset it to downstream.
    //
    // This is to ensure that when we find the equivalent leaf text position, it
    // will be at the start of anchor if the original position is anchored to a
    // node higher up in the tree and pointing to a text offset that falls on
    // the boundary between two leaf nodes. In other words, the returned
    // position will always be "before character".
    text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
    text_position = text_position->AsLeafTextPosition();
    BASE_DCHECK(!text_position->IsNullPosition())
        << "Adjusting to a leaf position should never turn a non-null position "
           "into a null one.";
    if (!text_position->IsIgnored() && !text_position->AtEndOfAnchor()) {
      BASE_DCHECK(text_position->text_offset_ >= 0);
      return text_position;
    }

    text_position = text_position->CreateNextLeafTextPosition();
    while (!text_position->IsNullPosition() &&
           (text_position->IsIgnored() || !text_position->MaxTextOffset())) {
      text_position = text_position->CreateNextLeafTextPosition();
    }
    return text_position;
  }

  // Returns a text position located right after the previous character (from
  // this position) in the tree's text representation.
  //
  // See `AsLeafTextPositionBeforeCharacter`, as this is its "reversed" version.
  AXPositionInstance AsLeafTextPositionAfterCharacter() const {
    if (IsNullPosition())
      return Clone();

    AXPositionInstance text_position = AsTextPosition();
    // Temporarily set the affinity to upstream.
    //
    // This is to ensure that when we find the equivalent leaf text position, it
    // will be at the end of anchor if the original position is anchored to a
    // node higher up in the tree and pointing to a text offset that falls on
    // the boundary between two leaf nodes. In other words, the returned
    // position will always be "after character".
    text_position->affinity_ = ax::mojom::TextAffinity::kUpstream;
    text_position = text_position->AsLeafTextPosition();
    BASE_DCHECK(!text_position->IsNullPosition())
        << "Adjusting to a leaf position should never turn a non-null position "
           "into a null one.";
    if (!text_position->IsIgnored() && !text_position->AtStartOfAnchor()) {
      // The following situation should not be possible but there are existing
      // crashes in the field.
      //
      // TODO(nektar): Remove this workaround as soon as the source of the bug
      // is identified.
      BASE_DCHECK(text_position->text_offset_ >= 0);
      // TODO(chunhtai): handles grapheme.

      // Reset the affinity to downstream, because an upstream affinity doesn't
      // make sense on a leaf anchor.
      text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
      return text_position;
    }
    text_position = text_position->CreatePreviousLeafTextPosition();
    while (!text_position->IsNullPosition() &&
           (text_position->IsIgnored() || !text_position->MaxTextOffset())) {
      text_position = text_position->CreatePreviousLeafTextPosition();
    }
    return text_position->CreatePositionAtEndOfAnchor();
  }

  // Creates a position pointing to before the next character, which is defined
  // as the start of the next grapheme cluster. Also, ensures that the created
  // position will not point to a low surrogate pair.
  //
  // A grapheme cluster is what an end-user would consider a character and it
  // could include a letter with additional diacritics. It could be more than
  // one Unicode code unit in length.
  //
  // See also http://www.unicode.org/reports/tr29/#Grapheme_Cluster_Boundaries
  AXPositionInstance CreateNextCharacterPosition(
      AXBoundaryBehavior boundary_behavior) const {
    if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary &&
        AtEndOfAnchor()) {
      return Clone();
    }

    // There is no next character position.
    AXPositionInstance text_position = AsLeafTextPositionBeforeCharacter();
    if (text_position->IsNullPosition()) {
      if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary ||
          boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary) {
        text_position = Clone();
      }
      return text_position;
    }

    if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary &&
        *text_position == *this) {
      return Clone();
    }
    BASE_DCHECK(text_position->text_offset_ < text_position->MaxTextOffset());
    // TODO(chunhtai): Need to consider grapheme cluster.
    ++text_position->text_offset_;
    BASE_DCHECK(text_position->text_offset_ > 0);
    BASE_DCHECK(text_position->text_offset_ <= text_position->MaxTextOffset());
    // If the character boundary is in the same subtree, return a position
    // rooted at this position's anchor. This is necessary because we don't want
    // to return a position that might be in the shadow DOM when this position
    // is not.
    const AXNodeType* common_anchor = text_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      text_position = text_position->CreateAncestorPosition(
          common_anchor, ax::mojom::MoveDirection::kForward);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      // If the next character position crosses the current anchor boundary
      // with StopAtAnchorBoundary, snap to the end of the current anchor.
      return CreatePositionAtEndOfAnchor();
    }
    // Even if the resulting position is right on a soft line break, affinity is
    // defaulted to downstream so that this method will always produce the same
    // result regardless of the direction of motion or the input affinity.
    text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
    if (IsTreePosition())
      return text_position->AsTreePosition();
    return text_position;
  }

  // Creates a position pointing to before the previous character, which is
  // defined as the start of the previous grapheme cluster. Also, ensures that
  // the created position will not point to a low surrogate pair.
  //
  // See the comment above `CreateNextCharacterPosition` for the definition of a
  // grapheme cluster.
  AXPositionInstance CreatePreviousCharacterPosition(
      AXBoundaryBehavior boundary_behavior) const {
    if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary &&
        AtStartOfAnchor()) {
      return Clone();
    }

    // There is no previous character position.
    AXPositionInstance text_position = AsLeafTextPositionAfterCharacter();
    if (text_position->IsNullPosition()) {
      if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary ||
          boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary) {
        text_position = Clone();
      }
      return text_position;
    }

    if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary &&
        *text_position == *this) {
      return Clone();
    }
    BASE_DCHECK(text_position->text_offset_ > 0);
    // TODO(chunhtai): Need to consider grapheme cluster.
    --text_position->text_offset_;
    BASE_DCHECK(text_position->text_offset_ >= 0);
    BASE_DCHECK(text_position->text_offset_ < text_position->MaxTextOffset());
    // The character boundary should be in the same subtree. Return a position
    // rooted at this position's anchor. This is necessary because we don't want
    // to return a position that might be in the shadow DOM when this position
    // is not.
    const AXNodeType* common_anchor = text_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      text_position = text_position->CreateAncestorPosition(
          common_anchor, ax::mojom::MoveDirection::kBackward);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      // If the previous character position crosses the current anchor boundary
      // with StopAtAnchorBoundary, snap to the start of the current anchor.
      return CreatePositionAtStartOfAnchor();
    }
    // Even if the resulting position is right on a soft line break, affinity is
    // defaulted to downstream so that this method will always produce the same
    // result regardless of the direction of motion or the input affinity.
    text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
    if (IsTreePosition())
      return text_position->AsTreePosition();
    return text_position;
  }

  AXPositionInstance CreateNextWordStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfWordPredicate, &AtEndOfWordPredicate,
        &GetWordStartOffsetsFunc);
  }

  AXPositionInstance CreatePreviousWordStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfWordPredicate, &AtEndOfWordPredicate,
        &GetWordStartOffsetsFunc);
  }

  // Word end positions are one past the last character of the word.
  AXPositionInstance CreateNextWordEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfWordPredicate, &AtEndOfWordPredicate, &GetWordEndOffsetsFunc);
  }

  // Word end positions are one past the last character of the word.
  AXPositionInstance CreatePreviousWordEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfWordPredicate, &AtEndOfWordPredicate, &GetWordEndOffsetsFunc);
  }

  AXPositionInstance CreateNextLineStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfLinePredicate, &AtEndOfLinePredicate);
  }

  AXPositionInstance CreatePreviousLineStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfLinePredicate, &AtEndOfLinePredicate);
  }

  // Line end positions are one past the last character of the line, excluding
  // any white space or newline characters that separate the lines.
  AXPositionInstance CreateNextLineEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfLinePredicate, &AtEndOfLinePredicate);
  }

  // Line end positions are one past the last character of the line, excluding
  // any white space or newline characters separating the lines.
  AXPositionInstance CreatePreviousLineEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfLinePredicate, &AtEndOfLinePredicate);
  }

  AXPositionInstance CreatePreviousFormatStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    if (IsNullPosition())
      return Clone();

    AXBoundaryType boundary_type = GetFormatStartBoundaryType();
    if (boundary_type != AXBoundaryType::kNone) {
      if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary ||
          (boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary &&
           boundary_type == AXBoundaryType::kDocumentStart)) {
        AXPositionInstance clone = Clone();
        // In order to make equality checks simpler, affinity should be reset so
        // that we would get consistent output from this function regardless of
        // input affinity.
        clone->affinity_ = ax::mojom::TextAffinity::kDownstream;
        return clone;
      } else if (boundary_behavior == AXBoundaryBehavior::CrossBoundary &&
                 boundary_type == AXBoundaryType::kDocumentStart) {
        // If we're at a format boundary and there are no more text positions
        // to traverse, return a null position for cross-boundary moves.
        return CreateNullPosition();
      }
    }

    AXPositionInstance tree_position =
        AsTreePosition()->CreatePositionAtStartOfAnchor();
    AXPositionInstance previous_tree_position =
        tree_position->CreatePreviousLeafTreePosition();

    // If moving to the start of the current anchor hasn't changed our position
    // from the original position, we need to test the previous leaf tree
    // position.
    if (AtStartOfAnchor() &&
        boundary_behavior != AXBoundaryBehavior::StopIfAlreadyAtBoundary) {
      tree_position = std::move(previous_tree_position);
      previous_tree_position = tree_position->CreatePreviousLeafTreePosition();
    }

    // The first position in the document is also a format start boundary, so we
    // should not return NullPosition unless we started from that location.
    while (boundary_type != AXBoundaryType::kDocumentStart &&
           !previous_tree_position->IsNullPosition() &&
           !tree_position->AtStartOfFormat()) {
      tree_position = std::move(previous_tree_position);
      previous_tree_position = tree_position->CreatePreviousLeafTreePosition();
    }

    // If the format boundary is in the same subtree, return a position rooted
    // at the current position.
    // This is necessary because we don't want to return any position that might
    // be in the shadow DOM if the original position was not.
    const AXNodeType* common_anchor = tree_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      tree_position = tree_position->CreateAncestorPosition(
          common_anchor, ax::mojom::MoveDirection::kBackward);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      return CreatePositionAtStartOfAnchor();
    }

    if (IsTextPosition())
      return tree_position->AsTextPosition();
    return tree_position;
  }

  AXPositionInstance CreateNextFormatEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    if (IsNullPosition())
      return Clone();

    AXBoundaryType boundary_type = GetFormatEndBoundaryType();
    if (boundary_type != AXBoundaryType::kNone) {
      if (boundary_behavior == AXBoundaryBehavior::StopIfAlreadyAtBoundary ||
          (boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary &&
           boundary_type == AXBoundaryType::kDocumentEnd)) {
        AXPositionInstance clone = Clone();
        // In order to make equality checks simpler, affinity should be reset so
        // that we would get consistent output from this function regardless of
        // input affinity.
        clone->affinity_ = ax::mojom::TextAffinity::kDownstream;
        return clone;
      } else if (boundary_behavior == AXBoundaryBehavior::CrossBoundary &&
                 boundary_type == AXBoundaryType::kDocumentEnd) {
        // If we're at a format boundary and there are no more text positions
        // to traverse, return a null position for cross-boundary moves.
        return CreateNullPosition();
      }
    }

    AXPositionInstance tree_position =
        AsTreePosition()->CreatePositionAtEndOfAnchor();
    AXPositionInstance next_tree_position =
        tree_position->CreateNextLeafTreePosition()
            ->CreatePositionAtEndOfAnchor();

    // If moving to the end of the current anchor hasn't changed our original
    // position, we need to test the next leaf tree position.
    if (AtEndOfAnchor() &&
        boundary_behavior != AXBoundaryBehavior::StopIfAlreadyAtBoundary) {
      tree_position = std::move(next_tree_position);
      next_tree_position = tree_position->CreateNextLeafTreePosition()
                               ->CreatePositionAtEndOfAnchor();
    }

    // The last position in the document is also a format end boundary, so we
    // should not return NullPosition unless we started from that location.
    while (boundary_type != AXBoundaryType::kDocumentEnd &&
           !next_tree_position->IsNullPosition() &&
           !tree_position->AtEndOfFormat()) {
      tree_position = std::move(next_tree_position);
      next_tree_position = tree_position->CreateNextLeafTreePosition()
                               ->CreatePositionAtEndOfAnchor();
    }

    // If the format boundary is in the same subtree, return a position
    // rooted at the current position.
    // This is necessary because we don't want to return any position that might
    // be in the shadow DOM if the original position was not.
    const AXNodeType* common_anchor = tree_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      tree_position = tree_position->CreateAncestorPosition(
          common_anchor, ax::mojom::MoveDirection::kForward);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      return CreatePositionAtEndOfAnchor();
    }

    if (IsTextPosition())
      return tree_position->AsTextPosition();
    return tree_position;
  }

  AXPositionInstance CreateNextParagraphStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfParagraphPredicate, &AtEndOfParagraphPredicate);
  }

  AXPositionInstance CreatePreviousParagraphStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfParagraphPredicate, &AtEndOfParagraphPredicate);
  }

  AXPositionInstance CreateNextParagraphEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfParagraphPredicate, &AtEndOfParagraphPredicate);
  }

  AXPositionInstance CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    AXPositionInstance previous_position = CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfParagraphPredicate, &AtEndOfParagraphPredicate);
    if (boundary_behavior == AXBoundaryBehavior::CrossBoundary ||
        boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary) {
      // This is asymmetric with CreateNextParagraphEndPosition due to
      // asymmetries in text anchor movement. Consider:
      //
      // ++1 rootWebArea
      // ++++2 staticText name="FIRST"
      // ++++3 genericContainer isLineBreakingObject=true
      // ++++++4 genericContainer isLineBreakingObject=true
      // ++++++5 staticText name="SECOND"
      //
      // Node 2 offset 5 FIRST<> is a paragraph end since node 3 is a line-
      // breaking object that's not collapsible (since it's not a leaf). When
      // looking for the next text anchor position from there, we advance to
      // sibling node 3, then since that node has descendants, we convert to a
      // tree position to find the leaf node that maps to "node 3 offset 0".
      // Since node 4 has no text, we skip it and land on node 5. We end up at
      // node 5 offset 6 SECOND<> as our next paragraph end.
      //
      // The set of paragraph ends should be consistent when moving in the
      // reverse direction. But starting from node 5 offset 6, the previous text
      // anchor position is previous sibling node 4. We'll consider that a
      // paragraph end since it's a leaf line-breaking object and stop.
      //
      // Essentially, we have two consecutive line-breaking objects, each of
      // which stops movement in the "outward" direction, for different reasons.
      //
      // We handle this by looking back one more step after finding a candidate
      // for previous paragraph end, then testing a forward step from the look-
      // back position. That will land us on the candidate position if it's a
      // valid paragraph boundary.
      //
      while (!previous_position->IsNullPosition()) {
        AXPositionInstance look_back_position =
            previous_position->AsLeafTextPosition()
                ->CreatePreviousLeafTextPosition()
                ->CreatePositionAtEndOfAnchor();
        if (look_back_position->IsNullPosition()) {
          // Nowhere to look back to, so our candidate must be a valid paragraph
          // boundary.
          break;
        }
        AXPositionInstance forward_step_position =
            look_back_position->CreateNextLeafTextPosition()
                ->CreatePositionAtEndOfAnchor();
        if (*forward_step_position == *previous_position)
          break;

        previous_position = previous_position->CreateBoundaryEndPosition(
            boundary_behavior, ax::mojom::MoveDirection::kBackward,
            &AtStartOfParagraphPredicate, &AtEndOfParagraphPredicate);
      }
    }

    return previous_position;
  }

  AXPositionInstance CreateNextPageStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfPagePredicate, &AtEndOfPagePredicate);
  }

  AXPositionInstance CreatePreviousPageStartPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryStartPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfPagePredicate, &AtEndOfPagePredicate);
  }

  AXPositionInstance CreateNextPageEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kForward,
        &AtStartOfPagePredicate, &AtEndOfPagePredicate);
  }

  AXPositionInstance CreatePreviousPageEndPosition(
      AXBoundaryBehavior boundary_behavior) const {
    return CreateBoundaryEndPosition(
        boundary_behavior, ax::mojom::MoveDirection::kBackward,
        &AtStartOfPagePredicate, &AtEndOfPagePredicate);
  }

  AXPositionInstance CreateBoundaryStartPosition(
      AXBoundaryBehavior boundary_behavior,
      ax::mojom::MoveDirection move_direction,
      BoundaryConditionPredicate at_start_condition,
      BoundaryConditionPredicate at_end_condition,
      BoundaryTextOffsetsFunc get_start_offsets = {}) const {
    AXPositionInstance text_position = AsLeafTextPosition();
    if (text_position->IsNullPosition())
      return text_position;

    if (boundary_behavior != AXBoundaryBehavior::StopIfAlreadyAtBoundary) {
      text_position =
          text_position->CreateAdjacentLeafTextPosition(move_direction);
      if (text_position->IsNullPosition()) {
        // There is no adjacent position to move to; in such case, CrossBoundary
        // behavior shall return a null position, while any other behavior shall
        // fallback to return the initial position.
        if (boundary_behavior == AXBoundaryBehavior::CrossBoundary)
          return text_position;
        return Clone();
      }
    }

    if (!at_start_condition(text_position)) {
      text_position = text_position->CreatePositionAtNextOffsetBoundary(
          move_direction, get_start_offsets);

      while (!at_start_condition(text_position)) {
        AXPositionInstance next_position;
        if (move_direction == ax::mojom::MoveDirection::kForward) {
          next_position = text_position->CreateNextLeafTextPosition();
        } else {
          if (text_position->AtStartOfAnchor()) {
            next_position = text_position->CreatePreviousLeafTextPosition();
          } else {
            text_position = text_position->CreatePositionAtStartOfAnchor();
            BASE_DCHECK(!text_position->IsNullPosition());
            continue;
          }
        }

        if (next_position->IsNullPosition()) {
          if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
            switch (move_direction) {
              case ax::mojom::MoveDirection::kForward:
                return CreatePositionAtEndOfAnchor()->AsUnignoredPosition(
                    AXPositionAdjustmentBehavior::kMoveForward);
              case ax::mojom::MoveDirection::kBackward:
                return CreatePositionAtStartOfAnchor()->AsUnignoredPosition(
                    AXPositionAdjustmentBehavior::kMoveBackward);
            }
          }

          if (boundary_behavior ==
              AXBoundaryBehavior::StopAtLastAnchorBoundary) {
            // We can't simply return the following position; break and after
            // this loop we'll try to do some adjustments to text_position.
            switch (move_direction) {
              case ax::mojom::MoveDirection::kForward:
                text_position = text_position->CreatePositionAtEndOfAnchor();
                break;
              case ax::mojom::MoveDirection::kBackward:
                text_position = text_position->CreatePositionAtStartOfAnchor();
                break;
            }

            break;
          }

          return next_position->AsUnignoredPosition(
              AdjustmentBehaviorFromBoundaryDirection(move_direction));
        }

        // Continue searching for the next boundary start in the specified
        // direction until the next logical text position is reached.
        text_position = next_position->CreatePositionAtFirstOffsetBoundary(
            move_direction, get_start_offsets);
      }
    }

    // If the boundary is in the same subtree, return a position rooted at this
    // position's anchor. This is necessary because we don't want to return a
    // position that might be in the shadow DOM when this position is not.
    const AXNodeType* common_anchor = text_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      text_position =
          text_position->CreateAncestorPosition(common_anchor, move_direction);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      switch (move_direction) {
        case ax::mojom::MoveDirection::kForward:
          return CreatePositionAtEndOfAnchor()->AsUnignoredPosition(
              AXPositionAdjustmentBehavior::kMoveForward);
        case ax::mojom::MoveDirection::kBackward:
          return CreatePositionAtStartOfAnchor()->AsUnignoredPosition(
              AXPositionAdjustmentBehavior::kMoveBackward);
      }
    }

    // Affinity is only upstream at the end of a line, and so a start boundary
    // will never have an upstream affinity.
    text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
    if (IsTreePosition())
      text_position = text_position->AsTreePosition();
    AXPositionInstance unignored_position = text_position->AsUnignoredPosition(
        AdjustmentBehaviorFromBoundaryDirection(move_direction));
    // If there are no unignored positions in |move_direction| then
    // |text_position| is anchored in ignored content at the start or end
    // of the document.
    // For StopAtLastAnchorBoundary, try to adjust in the opposite direction
    // to return a position within the document just before crossing into
    // the ignored content. This will be the last unignored anchor boundary.
    if (unignored_position->IsNullPosition() &&
        boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary) {
      unignored_position =
          text_position->AsUnignoredPosition(OppositeAdjustmentBehavior(
              AdjustmentBehaviorFromBoundaryDirection(move_direction)));
    }
    return unignored_position;
  }

  AXPositionInstance CreateBoundaryEndPosition(
      AXBoundaryBehavior boundary_behavior,
      ax::mojom::MoveDirection move_direction,
      BoundaryConditionPredicate at_start_condition,
      BoundaryConditionPredicate at_end_condition,
      BoundaryTextOffsetsFunc get_end_offsets = {}) const {
    AXPositionInstance text_position = AsLeafTextPosition();
    if (text_position->IsNullPosition())
      return text_position;

    if (boundary_behavior != AXBoundaryBehavior::StopIfAlreadyAtBoundary) {
      text_position =
          text_position->CreateAdjacentLeafTextPosition(move_direction);
      if (text_position->IsNullPosition()) {
        // There is no adjacent position to move to; in such case, CrossBoundary
        // behavior shall return a null position, while any other behavior shall
        // fallback to return the initial position.
        if (boundary_behavior == AXBoundaryBehavior::CrossBoundary)
          return text_position;
        return Clone();
      }
    }

    if (!at_end_condition(text_position)) {
      text_position = text_position->CreatePositionAtNextOffsetBoundary(
          move_direction, get_end_offsets);

      while (!at_end_condition(text_position)) {
        AXPositionInstance next_position;
        if (move_direction == ax::mojom::MoveDirection::kForward) {
          if (text_position->AtEndOfAnchor()) {
            next_position = text_position->CreateNextLeafTextPosition();
          } else {
            text_position = text_position->CreatePositionAtEndOfAnchor();
            BASE_DCHECK(!text_position->IsNullPosition());
            continue;
          }
        } else {
          next_position = text_position->CreatePreviousLeafTextPosition()
                              ->CreatePositionAtEndOfAnchor();
        }

        if (next_position->IsNullPosition()) {
          if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
            switch (move_direction) {
              case ax::mojom::MoveDirection::kForward:
                return CreatePositionAtEndOfAnchor()->AsUnignoredPosition(
                    AXPositionAdjustmentBehavior::kMoveForward);
              case ax::mojom::MoveDirection::kBackward:
                return CreatePositionAtStartOfAnchor()->AsUnignoredPosition(
                    AXPositionAdjustmentBehavior::kMoveBackward);
            }
          }

          if (boundary_behavior ==
              AXBoundaryBehavior::StopAtLastAnchorBoundary) {
            // We can't simply return the following position; break and after
            // this loop we'll try to do some adjustments to text_position.
            switch (move_direction) {
              case ax::mojom::MoveDirection::kForward:
                text_position = text_position->CreatePositionAtEndOfAnchor();
                break;
              case ax::mojom::MoveDirection::kBackward:
                text_position = text_position->CreatePositionAtStartOfAnchor();
                break;
            }

            break;
          }

          return next_position->AsUnignoredPosition(
              AdjustmentBehaviorFromBoundaryDirection(move_direction));
        }

        // Continue searching for the next boundary end in the specified
        // direction until the next logical text position is reached.
        text_position = next_position->CreatePositionAtFirstOffsetBoundary(
            move_direction, get_end_offsets);
      }
    }

    // If the boundary is in the same subtree, return a position rooted at this
    // position's anchor. This is necessary because we don't want to return a
    // position that might be in the shadow DOM when this position is not.
    const AXNodeType* common_anchor = text_position->LowestCommonAnchor(*this);
    if (GetAnchor() == common_anchor) {
      text_position =
          text_position->CreateAncestorPosition(common_anchor, move_direction);
    } else if (boundary_behavior == AXBoundaryBehavior::StopAtAnchorBoundary) {
      switch (move_direction) {
        case ax::mojom::MoveDirection::kForward:
          return CreatePositionAtEndOfAnchor()->AsUnignoredPosition(
              AXPositionAdjustmentBehavior::kMoveForward);
        case ax::mojom::MoveDirection::kBackward:
          return CreatePositionAtStartOfAnchor()->AsUnignoredPosition(
              AXPositionAdjustmentBehavior::kMoveBackward);
      }
    }

    // If there is no ambiguity as to whether the position is at the end of
    // the current boundary or the start of the next boundary, an upstream
    // affinity should be reset to downstream in order to get consistent output
    // from this method, regardless of input affinity.
    //
    // Note that there could be no ambiguity if the boundary is either at the
    // start or the end of the current anchor, so we should always reset to
    // downstream affinity in those cases.
    if (text_position->affinity_ == ax::mojom::TextAffinity::kUpstream) {
      AXPositionInstance downstream_position = text_position->Clone();
      downstream_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
      if (downstream_position->AtStartOfAnchor() ||
          downstream_position->AtEndOfAnchor() ||
          !at_start_condition(downstream_position)) {
        text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
      }
    }

    if (IsTreePosition())
      text_position = text_position->AsTreePosition();
    AXPositionInstance unignored_position = text_position->AsUnignoredPosition(
        AdjustmentBehaviorFromBoundaryDirection(move_direction));
    // If there are no unignored positions in |move_direction| then
    // |text_position| is anchored in ignored content at the start or end
    // of the document.
    // For StopAtLastAnchorBoundary, try to adjust in the opposite direction
    // to return a position within the document just before crossing into
    // the ignored content. This will be the last unignored anchor boundary.
    if (unignored_position->IsNullPosition() &&
        boundary_behavior == AXBoundaryBehavior::StopAtLastAnchorBoundary) {
      unignored_position =
          text_position->AsUnignoredPosition(OppositeAdjustmentBehavior(
              AdjustmentBehaviorFromBoundaryDirection(move_direction)));
    }
    return unignored_position;
  }

  // TODO(nektar): Add sentence navigation methods.

  // Uses depth-first pre-order traversal.
  AXPositionInstance CreateNextAnchorPosition() const {
    return CreateNextAnchorPosition(&DefaultAbortMovePredicate);
  }

  // Uses depth-first pre-order traversal.
  AXPositionInstance CreatePreviousAnchorPosition() const {
    return CreatePreviousAnchorPosition(&DefaultAbortMovePredicate);
  }

  // Returns an optional integer indicating the logical order of this position
  // compared to another position or returns an empty optional if the positions
  // are not comparable. Any text position at the same character location is
  // logically equivalent although they may be on different anchors or have
  // different text offsets. Positions are not comparable when one position is
  // null and the other is not or if the positions do not have any common
  // ancestor.
  //    0: if this position is logically equivalent to the other position
  //   <0: if this position is logically less than the other position
  //   >0: if this position is logically greater than the other position
  std::optional<int> CompareTo(const AXPosition& other) const {
    if (this->IsNullPosition() && other.IsNullPosition())
      return std::optional<int>(0);
    if (this->IsNullPosition() || other.IsNullPosition())
      return std::optional<int>(std::nullopt);

    // If both positions share an anchor and are of the same type, we can do a
    // straight compare of text offsets or child indices.
    if (GetAnchor() == other.GetAnchor()) {
      if (IsTextPosition() && other.IsTextPosition())
        return text_offset() - other.text_offset();
      if (IsTreePosition() && other.IsTreePosition())
        return child_index() - other.child_index();
    }

    // Ancestor positions are expensive to compute. If possible, we will avoid
    // doing so by computing the ancestor chain of the two positions' anchors.
    // If the lowest common ancestor is neither position's anchor, we can use
    // the order of the first uncommon ancestors as a proxy for the order of the
    // positions.
    //
    // In order to do that, we need to normalize text positions at the end of an
    // anchor to equivalent positions at the start of the next anchor. Ignored
    // positions are a special case in that they need to be shifted to the
    // nearest unignored position in order to be normalized. That shifting can
    // change the comparison result, so if we have an ignored position, we must
    // use the slow path.
    if (IsIgnored() || other.IsIgnored())
      return SlowCompareTo(other);

    // Normalize any text positions at the end of an anchor to equivalent
    // positions at the start of the next anchor.
    AXPositionInstance normalized_this_position = Clone();
    if (normalized_this_position->IsTextPosition()) {
      normalized_this_position =
          normalized_this_position->AsLeafTextPositionBeforeCharacter();
    }

    AXPositionInstance normalized_other_position = other.Clone();
    if (normalized_other_position->IsTextPosition()) {
      normalized_other_position =
          normalized_other_position->AsLeafTextPositionBeforeCharacter();
    }

    if (normalized_this_position->IsNullPosition()) {
      if (normalized_other_position->IsNullPosition()) {
        // Both positions normalized to a position past the end of the document.
        BASE_DCHECK(SlowCompareTo(other).value() == 0);
        return 0;
      }
      // |this| normalized to a position past the end of the document.
      BASE_DCHECK(SlowCompareTo(other).value() > 0);
      return 1;
    } else if (normalized_other_position->IsNullPosition()) {
      // |other| normalized to a position past the end of the document.
      BASE_DCHECK(SlowCompareTo(other).value() < 0);
      return -1;
    }

    // Compute the ancestor stacks of both positions and walk them ourselves
    // rather than calling LowestCommonAnchor(). That way, we can discover the
    // first uncommon ancestors.
    const AXNodeType* common_anchor = nullptr;
    std::stack<AXNodeType*> our_ancestors =
        normalized_this_position->GetAncestorAnchors();
    std::stack<AXNodeType*> other_ancestors =
        normalized_other_position->GetAncestorAnchors();
    while (!our_ancestors.empty() && !other_ancestors.empty() &&
           our_ancestors.top() == other_ancestors.top()) {
      common_anchor = our_ancestors.top();
      our_ancestors.pop();
      other_ancestors.pop();
    }

    if (!common_anchor)
      return std::optional<int>(std::nullopt);

    // If each position has an uncommon ancestor node, we can compare those
    // instead of needing to compute ancestor positions.
    if (!our_ancestors.empty() && !other_ancestors.empty()) {
      AXPositionInstance this_uncommon_tree_position = CreateTreePosition(
          GetTreeID(our_ancestors.top()), GetAnchorID(our_ancestors.top()),
          0 /*child_index*/);
      int this_uncommon_ancestor_index =
          this_uncommon_tree_position->AnchorIndexInParent();
      AXPositionInstance other_uncommon_tree_position = CreateTreePosition(
          GetTreeID(other_ancestors.top()), GetAnchorID(other_ancestors.top()),
          0 /*child_index*/);
      int other_uncommon_ancestor_index =
          other_uncommon_tree_position->AnchorIndexInParent();
      BASE_DCHECK(this_uncommon_ancestor_index !=
                  other_uncommon_ancestor_index);
      int result = this_uncommon_ancestor_index - other_uncommon_ancestor_index;

      // On platforms that support embedded objects, if a text position is
      // within an embedded object and if it is not at the start of that object,
      // the resulting ancestor position should be adjusted to point after the
      // embedded object. Otherwise, assistive software will not be able to get
      // out of the embedded object if its text is not editable when navigating
      // by character.
      //
      // For example, look at the following accessibility tree and the two
      // example text positions together with their equivalent ancestor
      // positions.
      // ++1 kRootWebArea
      // ++++2 kTextField "Before<embedded_object>after"
      // ++++++3 kStaticText "Before"
      // ++++++++4 kInlineTextBox "Before"
      // ++++++5 kImage "Test image"
      // ++++++6 kStaticText "after"
      // ++++++++7 kInlineTextBox "after"
      //
      // Note that the alt text of an image cannot be navigated with cursor
      // left/right, even when the rest of the contents are in a
      // contenteditable.
      //
      // Ancestor position should not be adjusted:
      // TextPosition anchor_id=kImage text_offset=0 affinity=downstream
      // annotated_text=<T>est image AncestorTextPosition anchor_id=kTextField
      // text_offset=6 affinity=downstream
      // annotated_text=Before<embedded_object>after
      //
      // Ancestor position should be adjusted:
      // TextPosition anchor_id=kImage text_offset=1 affinity=downstream
      // annotated_text=T<e>st image AncestorTextPosition anchor_id=kTextField
      // text_offset=7 affinity=downstream
      // annotated_text=Beforeembedded_object<a>fter
      //
      // Note that since the adjustment to the distance between the ancestor
      // positions could at most be by one, we skip doing this check if the
      // ancestor positions have a distance of more than one since it can never
      // change the outcome of the comparison. Note too that if both ancestor
      // positions need to be adjusted, the adjustments will cancel out.
      if (abs(result) == 1) {
        if (!normalized_this_position->AtStartOfAnchor() &&
            this_uncommon_tree_position->IsEmbeddedObjectInParent()) {
          result += 1;
        }
        if (!normalized_other_position->AtStartOfAnchor() &&
            other_uncommon_tree_position->IsEmbeddedObjectInParent()) {
          result -= 1;
        }
      }

#ifndef NDEBUG
      // Validate the optimization.
      int slow_result = SlowCompareTo(other).value();
      BASE_DCHECK((result == 0 && slow_result == 0) ||
                  (result < 0 && slow_result < 0) ||
                  (result > 0 && slow_result > 0));
#endif

      return result;
    }

    return SlowCompareTo(other);
  }

  std::optional<int> SlowCompareTo(const AXPosition& other) const {
    // It is potentially costly to compute the parent position of a text
    // position, whilst computing the parent position of a tree position is
    // really inexpensive. In order to find the lowest common ancestor,
    // especially if that ancestor is all the way up to the root of the tree,
    // this will need to be done repeatedly. We avoid the performance hit by
    // converting both positions to tree positions and only falling back to text
    // positions if both are text positions and the lowest common ancestor is
    // not one of their anchors. Essentially, the question we need to answer is:
    // "When are two non equivalent positions going to have the same lowest
    // common ancestor position when converted to tree positions?" The answer is
    // when they are both text positions and they either have the same anchor,
    // or one is the ancestor of the other.
    const AXNodeType* common_anchor = this->LowestCommonAnchor(other);
    if (!common_anchor)
      return std::optional<int>(std::nullopt);

    // Attempt to avoid recomputing the lowest common ancestor because we may
    // already have its anchor in which case just find the text offset.
    if (this->IsTextPosition() && other.IsTextPosition()) {
      // This text position's anchor is the common ancestor of the other text
      // position's anchor.
      if (this->GetAnchor() == common_anchor) {
        AXPositionInstance other_text_position =
            other.CreateAncestorPosition(common_anchor);
        return std::optional<int>(this->text_offset_ -
                                  other_text_position->text_offset_);
      }

      // The other text position's anchor is the common ancestor of this text
      // position's anchor.
      if (other.GetAnchor() == common_anchor) {
        AXPositionInstance this_text_position =
            this->CreateAncestorPosition(common_anchor);
        return std::optional<int>(this_text_position->text_offset_ -
                                  other.text_offset_);
      }

      // All optimizations failed. Fall back to comparing text positions with
      // the common text position ancestor.
      AXPositionInstance this_text_position_ancestor =
          this->CreateAncestorPosition(common_anchor);
      AXPositionInstance other_text_position_ancestor =
          other.CreateAncestorPosition(common_anchor);
      BASE_DCHECK(this_text_position_ancestor->IsTextPosition());
      BASE_DCHECK(other_text_position_ancestor->IsTextPosition());
      BASE_DCHECK(common_anchor == this_text_position_ancestor->GetAnchor());
      BASE_DCHECK(common_anchor == other_text_position_ancestor->GetAnchor());

      // TODO - This does not take into account |affinity_|, so we may return
      // a false positive when comparing at the end of a line.
      // For example :
      // ++1 kRootWebArea
      // ++++2 kTextField "Line 1\nLine 2"
      // ++++++3 kStaticText "Line 1"
      // ++++++++4 kInlineTextBox "Line 1"
      // ++++++5 kLineBreak "\n"
      // ++++++6 kStaticText "Line 2"
      // ++++++++7 kInlineTextBox "Line 2"
      //
      // TextPosition anchor_id=5 text_offset=1
      // affinity=downstream annotated_text=\n<>
      //
      // TextPosition anchor_id=7 text_offset=0
      // affinity=downstream annotated_text=<L>ine 2
      //
      // |LowestCommonAncestor| for both will be :
      // TextPosition anchor_id=2 text_offset=7
      // ... except anchor_id=5 creates a kUpstream position, while
      // anchor_id=7 creates a kDownstream position.
      return std::optional<int>(this_text_position_ancestor->text_offset_ -
                                other_text_position_ancestor->text_offset_);
    }

    // All optimizations failed. Fall back to comparing child index with
    // the common tree position ancestor.
    AXPositionInstance this_tree_position_ancestor =
        this->AsTreePosition()->CreateAncestorPosition(common_anchor);
    AXPositionInstance other_tree_position_ancestor =
        other.AsTreePosition()->CreateAncestorPosition(common_anchor);
    BASE_DCHECK(this_tree_position_ancestor->IsTreePosition());
    BASE_DCHECK(other_tree_position_ancestor->IsTreePosition());
    BASE_DCHECK(common_anchor == this_tree_position_ancestor->GetAnchor());
    BASE_DCHECK(common_anchor == other_tree_position_ancestor->GetAnchor());

    return std::optional<int>(this_tree_position_ancestor->child_index() -
                              other_tree_position_ancestor->child_index());
  }

  // A valid position can become invalid if the underlying tree structure
  // changes. This is expected behavior, but it is sometimes necessary to
  // maintain valid positions. This method modifies an invalid position that is
  // beyond MaxTextOffset to snap to MaxTextOffset.
  void SnapToMaxTextOffsetIfBeyond() {
    int max_text_offset = MaxTextOffset();
    if (text_offset_ > max_text_offset)
      text_offset_ = max_text_offset;
  }

  // Returns true if this position is on an empty object node that needs to
  // be represented by an empty object replacement character. It does when the
  // node is a collapsed menu list popup button or has no unignored child and is
  // not a text object. This feature is only enabled on some platforms.
  bool IsEmptyObjectReplacedByCharacter() const {
    if (g_ax_embedded_object_behavior ==
            AXEmbeddedObjectBehavior::kSuppressCharacter ||
        IsNullPosition()) {
      return false;
    }

    // A collapsed popup button that contains a menu list popup (i.e, the exact
    // subtree representation we get from a collapsed <select> element on
    // Windows) should not expose its children even though they are not ignored.
    if (GetAnchor()->IsCollapsedMenuListPopUpButton())
      return true;

    // All other elements that have unignored descendants should not be treated
    // as empty objects.
    if (AnchorUnignoredChildCount())
      return false;

    // All unignored leaf nodes in the AXTree except document and text
    // nodes should be replaced by the embedded object character. Also, nodes
    // that only have ignored children (e.g., a button that contains only an
    // empty div) need to be treated as leaf nodes.
    //
    // Calling AXPosition::IsIgnored here is not possible as it would create an
    // infinite loop. However, GetAnchor()->IsIgnored() is sufficient here
    // because we know that the anchor at this position doesn't have an
    // unignored child, making this a leaf tree or text position.
    return !GetAnchor()->IsIgnored() && !IsDocument(GetAnchorRole()) &&
           !IsInTextObject() && !IsIframe(GetAnchorRole());
  }

  bool IsInDescendantOfEmptyObject() const {
    if (g_ax_embedded_object_behavior ==
            AXEmbeddedObjectBehavior::kSuppressCharacter ||
        IsNullPosition()) {
      return false;
    }

    // Empty objects are only possible on a collapsed popup button parent of a
    // menu list popup or a node that only has ignored descendants. If it has no
    // empty object ancestor, it can't be inside of an empty object.
    return GetEmptyObjectAncestorNode();
  }

  AXNodeType* GetEmptyObjectAncestorNode() const {
    if (g_ax_embedded_object_behavior ==
            AXEmbeddedObjectBehavior::kSuppressCharacter ||
        !GetAnchor()) {
      return nullptr;
    }

    if (!GetAnchor()->IsIgnored()) {
      // The only case where a descendant of an empty object can be unignored is
      // when we are inside of a collapsed popup button parent of a menu list
      // popup.
      if (AXNodeType* popup_button =
              GetAnchor()->GetCollapsedMenuListPopUpButtonAncestor()) {
        return popup_button;
      }
      return nullptr;
    }

    // The first unignored ancestor is necessarily the empty object if this node
    // is the descendant of an empty object.
    AXNodeType* ancestor_node = GetLowestUnignoredAncestor();
    if (!ancestor_node)
      return nullptr;

    AXPositionInstance position = CreateTextPosition(
        tree_id_, GetAnchorID(ancestor_node), 0 /* text_offset */,
        ax::mojom::TextAffinity::kDownstream);
    if (position && position->IsEmptyObjectReplacedByCharacter())
      return ancestor_node;

    return nullptr;
  }

  void swap(AXPosition& other) {
    std::swap(kind_, other.kind_);
    std::swap(tree_id_, other.tree_id_);
    std::swap(anchor_id_, other.anchor_id_);
    std::swap(child_index_, other.child_index_);
    std::swap(text_offset_, other.text_offset_);
    std::swap(affinity_, other.affinity_);
  }

  // Abstract methods.

  // Returns the text that is present inside the anchor node, including any text
  // found in descendant text nodes, based on the platform's text
  // representation. Some platforms use an embedded object replacement character
  // that replaces the text coming from each child node.
  virtual std::u16string GetText() const = 0;

  // Determines if the anchor containing this position is a <br> or a text
  // object whose parent's anchor is an enclosing <br>.
  virtual bool IsInLineBreak() const = 0;

  // Determines if the anchor containing this position is a text object.
  virtual bool IsInTextObject() const = 0;

  // Determines if the text representation of this position's anchor contains
  // only whitespace characters; <br> objects span a single '\n' character, so
  // positions inside line breaks are also considered "in whitespace".
  virtual bool IsInWhiteSpace() const = 0;

  // Returns the length of the text that is present inside the anchor node,
  // including any text found in descendant text nodes. This is based on the
  // platform's text representation. Some platforms use an embedded object
  // character that replaces the text coming from each child node.
  //
  // Similar to "text_offset_", the length of the text is in UTF16 code units,
  // not in grapheme clusters.
  virtual int MaxTextOffset() const {
    if (IsNullPosition())
      return INVALID_OFFSET;
    return static_cast<int>(GetText().length());
  }

 protected:
  AXPosition()
      : kind_(AXPositionKind::NULL_POSITION),
        tree_id_(AXTreeIDUnknown()),
        anchor_id_(AXNode::kInvalidAXID),
        child_index_(INVALID_INDEX),
        text_offset_(INVALID_OFFSET),
        affinity_(ax::mojom::TextAffinity::kDownstream) {}

  // We explicitly don't copy any cached members.
  AXPosition(const AXPosition& other)
      : kind_(other.kind_),
        tree_id_(other.tree_id_),
        anchor_id_(other.anchor_id_),
        child_index_(other.child_index_),
        text_offset_(other.text_offset_),
        affinity_(other.affinity_) {}

  // Returns the character offset inside our anchor's parent at which our text
  // starts.
  int AnchorTextOffsetInParent() const {
    if (IsNullPosition())
      return INVALID_OFFSET;

    // Calculate how much text there is to the left of this anchor.
    AXPositionInstance tree_position = AsTreePosition();
    BASE_DCHECK(tree_position);
    AXPositionInstance parent_position = tree_position->CreateParentPosition();
    BASE_DCHECK(parent_position);
    if (parent_position->IsNullPosition())
      return 0;

    int offset_in_parent = 0;
    for (int i = 0; i < parent_position->child_index(); ++i) {
      AXPositionInstance child = parent_position->CreateChildPositionAt(i);
      BASE_DCHECK(child);
      offset_in_parent += child->MaxTextOffsetInParent();
    }
    return offset_in_parent;
  }

  void Initialize(AXPositionKind kind,
                  AXTreeID tree_id,
                  int32_t anchor_id,
                  int child_index,
                  int text_offset,
                  ax::mojom::TextAffinity affinity) {
    kind_ = kind;
    tree_id_ = tree_id;
    anchor_id_ = anchor_id;
    child_index_ = child_index;
    text_offset_ = text_offset;
    affinity_ = affinity;

    if (!IsValid()) {
      // Reset to the null position.
      kind_ = AXPositionKind::NULL_POSITION;
      tree_id_ = AXTreeIDUnknown();
      anchor_id_ = AXNode::kInvalidAXID;
      child_index_ = INVALID_INDEX;
      text_offset_ = INVALID_OFFSET;
      affinity_ = ax::mojom::TextAffinity::kDownstream;
    }
  }

  // Abstract methods.
  virtual void AnchorChild(int child_index,
                           AXTreeID* tree_id,
                           int32_t* child_id) const = 0;
  virtual int AnchorChildCount() const = 0;
  // When a child is ignored, it looks for unignored nodes of that child's
  // children until there are no more descendants.
  //
  // E.g.
  // ++TextField
  // ++++GenericContainer ignored
  // ++++++StaticText "Hello"
  // When we call the following method on TextField, it would return 1.
  virtual int AnchorUnignoredChildCount() const = 0;
  virtual int AnchorIndexInParent() const = 0;
  virtual int AnchorSiblingCount() const = 0;
  virtual std::stack<AXNodeType*> GetAncestorAnchors() const = 0;
  virtual AXNodeType* GetLowestUnignoredAncestor() const = 0;
  virtual void AnchorParent(AXTreeID* tree_id, int32_t* parent_id) const = 0;
  virtual AXNodeType* GetNodeInTree(AXTreeID tree_id,
                                    int32_t node_id) const = 0;
  virtual int32_t GetAnchorID(AXNodeType* node) const = 0;
  virtual AXTreeID GetTreeID(AXNodeType* node) const = 0;

  // Returns the length of text that this anchor node takes up in its parent.
  // On some platforms, embedded objects are represented in their parent with a
  // single embedded object character.
  int MaxTextOffsetInParent() const {
    return IsEmbeddedObjectInParent() ? 1 : MaxTextOffset();
  }

  // Returns whether or not this anchor is represented in their parent with a
  // single embedded object character.
  virtual bool IsEmbeddedObjectInParent() const = 0;

  // Determines if the anchor containing this position produces a hard line
  // break in the text representation, e.g. a block level element or a <br>.
  virtual bool IsInLineBreakingObject() const = 0;

  virtual ax::mojom::Role GetAnchorRole() const = 0;
  virtual ax::mojom::Role GetRole(AXNodeType* node) const = 0;
  virtual AXNodeTextStyles GetTextStyles() const = 0;
  virtual std::vector<int32_t> GetWordStartOffsets() const = 0;
  virtual std::vector<int32_t> GetWordEndOffsets() const = 0;
  virtual int32_t GetNextOnLineID(int32_t node_id) const = 0;
  virtual int32_t GetPreviousOnLineID(int32_t node_id) const = 0;

 private:
  // Defines the relationship between positions during traversal.
  // For example, moving from a descendant to an ancestor, is a kAncestor move.
  enum class AXMoveType {
    kAncestor,
    kDescendant,
    kSibling,
  };

  // Defines the direction of position movement, either next / previous in tree.
  enum class AXMoveDirection {
    kNextInTree,
    kPreviousInTree,
  };

  // Type of predicate function called during anchor navigation.
  // When the predicate returns |true|, the navigation stops and returns a
  // null position object.
  typedef bool AbortMovePredicate(const AXPosition& move_from,
                                  const AXPosition& move_to,
                                  const AXMoveType type,
                                  const AXMoveDirection direction);

  // A text span is defined by a series of inline text boxes that make up a
  // single static text object.
  bool AtEndOfTextSpan() const {
    if (GetAnchorRole() != ax::mojom::Role::kInlineTextBox || !AtEndOfAnchor())
      return false;

    // We are at the end of text span if |this| position has
    // role::kInlineTextBox, the parent of |this| has role::kStaticText, and the
    // anchor node of |this| is the last child of parent's children.
    const bool is_last_child =
        AnchorIndexInParent() == (AnchorSiblingCount() - 1);

    return is_last_child && GetRole(GetLowestUnignoredAncestor()) ==
                                ax::mojom::Role::kStaticText;
  }

  // Uses depth-first pre-order traversal.
  AXPositionInstance CreateNextAnchorPosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    if (IsNullPosition())
      return Clone();

    AXPositionInstance current_position = AsTreePosition();
    BASE_DCHECK(!current_position->IsNullPosition());

    if (!IsLeaf()) {
      const int child_index = current_position->child_index_;
      if (child_index < current_position->AnchorChildCount()) {
        AXPositionInstance child_position =
            current_position->CreateChildPositionAt(child_index);

        if (abort_predicate(*current_position, *child_position,
                            AXMoveType::kDescendant,
                            AXMoveDirection::kNextInTree)) {
          return CreateNullPosition();
        }
        return child_position;
      }
    }

    AXPositionInstance parent_position =
        current_position->CreateParentPosition();

    // Get the next sibling if it exists, otherwise move up the AXTree to the
    // lowest next sibling of this position's ancestors.
    while (!parent_position->IsNullPosition()) {
      const int index_in_parent = current_position->AnchorIndexInParent();
      if (index_in_parent + 1 < parent_position->AnchorChildCount()) {
        AXPositionInstance next_sibling =
            parent_position->CreateChildPositionAt(index_in_parent + 1);
        BASE_DCHECK(!next_sibling->IsNullPosition());

        if (abort_predicate(*current_position, *next_sibling,
                            AXMoveType::kSibling,
                            AXMoveDirection::kNextInTree)) {
          return CreateNullPosition();
        }
        return next_sibling;
      }

      if (abort_predicate(*current_position, *parent_position,
                          AXMoveType::kAncestor,
                          AXMoveDirection::kNextInTree)) {
        return CreateNullPosition();
      }

      current_position = std::move(parent_position);
      parent_position = current_position->CreateParentPosition();
    }
    return CreateNullPosition();
  }

  // Uses depth-first pre-order traversal.
  AXPositionInstance CreatePreviousAnchorPosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    if (IsNullPosition())
      return Clone();

    AXPositionInstance current_position = AsTreePosition();
    BASE_DCHECK(!current_position->IsNullPosition());

    AXPositionInstance parent_position =
        current_position->CreateParentPosition();
    if (parent_position->IsNullPosition())
      return parent_position;

    // If there is no previous sibling, move up to the parent.
    const int index_in_parent = current_position->AnchorIndexInParent();
    if (index_in_parent <= 0) {
      if (abort_predicate(*current_position, *parent_position,
                          AXMoveType::kAncestor,
                          AXMoveDirection::kPreviousInTree)) {
        return CreateNullPosition();
      }
      return parent_position;
    }

    // Get the previous sibling's deepest last child.
    AXPositionInstance rightmost_leaf =
        parent_position->CreateChildPositionAt(index_in_parent - 1);
    BASE_DCHECK(!rightmost_leaf->IsNullPosition());

    if (abort_predicate(*current_position, *rightmost_leaf,
                        AXMoveType::kSibling,
                        AXMoveDirection::kPreviousInTree)) {
      return CreateNullPosition();
    }

    while (!rightmost_leaf->IsLeaf()) {
      parent_position = std::move(rightmost_leaf);
      rightmost_leaf = parent_position->CreateChildPositionAt(
          parent_position->AnchorChildCount() - 1);
      BASE_DCHECK(!rightmost_leaf->IsNullPosition());

      if (abort_predicate(*parent_position, *rightmost_leaf,
                          AXMoveType::kDescendant,
                          AXMoveDirection::kPreviousInTree)) {
        return CreateNullPosition();
      }
    }
    return rightmost_leaf;
  }

  // Creates a position using the next text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreateNextTextAnchorPosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    // If this is an ancestor text position, resolve to its leaf text position.
    if (IsTextPosition() && !IsLeaf())
      return AsLeafTextPosition();

    AXPositionInstance next_leaf = CreateNextAnchorPosition(abort_predicate);
    while (!next_leaf->IsNullPosition() && !next_leaf->IsLeaf())
      next_leaf = next_leaf->CreateNextAnchorPosition(abort_predicate);

    BASE_DCHECK(next_leaf);
    return next_leaf->AsLeafTextPosition();
  }

  // Creates a position using the previous text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreatePreviousTextAnchorPosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    // If this is an ancestor text position, resolve to its leaf text position.
    if (IsTextPosition() && !IsLeaf())
      return AsLeafTextPosition();

    AXPositionInstance previous_leaf =
        CreatePreviousAnchorPosition(abort_predicate);
    while (!previous_leaf->IsNullPosition() && !previous_leaf->IsLeaf()) {
      previous_leaf =
          previous_leaf->CreatePreviousAnchorPosition(abort_predicate);
    }

    BASE_DCHECK(previous_leaf);
    return previous_leaf->AsLeafTextPosition();
  }

  // Creates a tree position using the next text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreateNextLeafTreePosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    AXPositionInstance next_leaf =
        AsTreePosition()->CreateNextAnchorPosition(abort_predicate);
    while (!next_leaf->IsNullPosition() && !next_leaf->IsLeaf())
      next_leaf = next_leaf->CreateNextAnchorPosition(abort_predicate);

    BASE_DCHECK(next_leaf);
    return next_leaf;
  }

  // Creates a tree position using the previous text-only node as its anchor.
  // Assumes that text-only nodes are leaf nodes.
  AXPositionInstance CreatePreviousLeafTreePosition(
      std::function<AbortMovePredicate> abort_predicate) const {
    AXPositionInstance previous_leaf =
        AsTreePosition()->CreatePreviousAnchorPosition(abort_predicate);
    while (!previous_leaf->IsNullPosition() && !previous_leaf->IsLeaf()) {
      previous_leaf =
          previous_leaf->CreatePreviousAnchorPosition(abort_predicate);
    }

    BASE_DCHECK(previous_leaf);
    return previous_leaf;
  }

  //
  // Static helpers for lambda usage.
  //

  static bool AtStartOfPagePredicate(const AXPositionInstance& position) {
    // If a page boundary is ignored, then it should not be exposed to assistive
    // software.
    return !position->IsIgnored() && position->AtStartOfPage();
  }

  static bool AtEndOfPagePredicate(const AXPositionInstance& position) {
    // If a page boundary is ignored, then it should not be exposed to assistive
    // software.
    return !position->IsIgnored() && position->AtEndOfPage();
  }

  static bool AtStartOfParagraphPredicate(const AXPositionInstance& position) {
    // The "AtStartOfParagraph" method already excludes ignored nodes.
    return position->AtStartOfParagraph();
  }

  static bool AtEndOfParagraphPredicate(const AXPositionInstance& position) {
    // The "AtEndOfParagraph" method already excludes ignored nodes.
    return position->AtEndOfParagraph();
  }

  static bool AtStartOfLinePredicate(const AXPositionInstance& position) {
    // Sometimes, nodes that are used to signify line boundaries are ignored.
    return position->AtStartOfLine();
  }

  static bool AtEndOfLinePredicate(const AXPositionInstance& position) {
    // Sometimes, nodes that are used to signify line boundaries are ignored.
    return position->AtEndOfLine();
  }

  static bool AtStartOfWordPredicate(const AXPositionInstance& position) {
    // Word boundaries should be at specific text offsets that are "visible" to
    // assistive software, hence not ignored. Ignored nodes are often used for
    // additional layout information, such as line and paragraph boundaries.
    // Their text is not currently processed.
    return !position->IsIgnored() && position->AtStartOfWord();
  }

  static bool AtEndOfWordPredicate(const AXPositionInstance& position) {
    // Word boundaries should be at specific text offsets that are "visible" to
    // assistive software, hence not ignored. Ignored nodes are often used for
    // additional layout information, such as line and paragraph boundaries.
    // Their text is not currently processed.
    return !position->IsIgnored() && position->AtEndOfWord();
  }

  static bool DefaultAbortMovePredicate(const AXPosition& move_from,
                                        const AXPosition& move_to,
                                        const AXMoveType move_type,
                                        const AXMoveDirection direction) {
    // Default behavior is to never abort.
    return false;
  }

  // AbortMovePredicate function used to detect format boundaries.
  static bool AbortMoveAtFormatBoundary(const AXPosition& move_from,
                                        const AXPosition& move_to,
                                        const AXMoveType move_type,
                                        const AXMoveDirection direction) {
    if (move_from.IsNullPosition() || move_to.IsNullPosition() ||
        move_from.IsEmptyObjectReplacedByCharacter() ||
        move_to.IsEmptyObjectReplacedByCharacter()) {
      return true;
    }

    // Treat moving into or out of nodes with certain roles as a format break.
    ax::mojom::Role from_role = move_from.GetAnchorRole();
    ax::mojom::Role to_role = move_to.GetAnchorRole();
    if (from_role != to_role) {
      if (IsFormatBoundary(from_role) || IsFormatBoundary(to_role))
        return true;
    }

    // Stop moving when text styles differ.
    return move_from.AsLeafTreePosition()->GetTextStyles() !=
           move_to.AsLeafTreePosition()->GetTextStyles();
  }

  static bool MoveCrossesLineBreakingObject(const AXPosition& move_from,
                                            const AXPosition& move_to,
                                            const AXMoveType move_type,
                                            const AXMoveDirection direction) {
    const bool move_from_break = move_from.IsInLineBreakingObject();
    const bool move_to_break = move_to.IsInLineBreakingObject();

    switch (move_type) {
      case AXMoveType::kAncestor:
        // For Ancestor moves, only abort when exiting a block descendant.
        // We don't care if the ancestor is a block or not, since the
        // descendant is contained by it.
        return move_from_break;
      case AXMoveType::kDescendant:
        // For Descendant moves, only abort when entering a block descendant.
        // We don't care if the ancestor is a block or not, since the
        // descendant is contained by it.
        return move_to_break;
      case AXMoveType::kSibling:
        // For Sibling moves, abort if at least one of the siblings are a block,
        // because that would mean exiting and/or entering a block.
        return move_from_break || move_to_break;
    }
    BASE_UNREACHABLE();
    return false;
  }

  // AbortMovePredicate function used to detect paragraph boundaries.
  // We don't want to abort immediately after crossing a line breaking object
  // boundary if the anchor we're moving to is not a leaf, this is necessary to
  // avoid aborting if the next leaf position is whitespace-only; update
  // |crossed_line_breaking_object_token| and wait until a leaf anchor is
  // reached in order to correctly determine paragraph boundaries.
  static bool AbortMoveAtParagraphBoundary(
      bool& crossed_line_breaking_object_token,
      const AXPosition& move_from,
      const AXPosition& move_to,
      const AXMoveType move_type,
      const AXMoveDirection direction) {
    if (move_from.IsNullPosition() || move_to.IsNullPosition() ||
        move_from.IsEmptyObjectReplacedByCharacter() ||
        move_to.IsEmptyObjectReplacedByCharacter()) {
      return true;
    }

    if (!crossed_line_breaking_object_token) {
      crossed_line_breaking_object_token = MoveCrossesLineBreakingObject(
          move_from, move_to, move_type, direction);
    }

    if (crossed_line_breaking_object_token && move_to.IsLeaf()) {
      // If there's a sequence of whitespace-only anchors, collapse so only the
      // last whitespace-only anchor is considered a paragraph boundary.
      return direction != AXMoveDirection::kNextInTree ||
             !move_to.IsInWhiteSpace();
    }
    return false;
  }

  // This AbortMovePredicate never aborts, but detects whether a sequence of
  // consecutive moves cross any line breaking object boundary.
  static bool UpdateCrossedLineBreakingObjectToken(
      bool& crossed_line_breaking_object_token,
      const AXPosition& move_from,
      const AXPosition& move_to,
      const AXMoveType move_type,
      const AXMoveDirection direction) {
    if (!crossed_line_breaking_object_token) {
      crossed_line_breaking_object_token = MoveCrossesLineBreakingObject(
          move_from, move_to, move_type, direction);
    }
    return false;
  }

  // AbortMovePredicate function used to detect page boundaries.
  static bool AbortMoveAtPageBoundary(const AXPosition& move_from,
                                      const AXPosition& move_to,
                                      const AXMoveType move_type,
                                      const AXMoveDirection direction) {
    if (move_from.IsNullPosition() || move_to.IsNullPosition())
      return true;

    const bool move_from_break = move_from.GetAnchor()->GetBoolAttribute(
        ax::mojom::BoolAttribute::kIsPageBreakingObject);
    const bool move_to_break = move_to.GetAnchor()->GetBoolAttribute(
        ax::mojom::BoolAttribute::kIsPageBreakingObject);

    switch (move_type) {
      case AXMoveType::kAncestor:
        // For Ancestor moves, only abort when exiting a page break.
        // We don't care if the ancestor is a page break or not, since the
        // descendant is contained by it.
        return move_from_break;
      case AXMoveType::kDescendant:
        // For Descendant moves, only abort when entering a page break
        // descendant. We don't care if the ancestor is a page break  or not,
        // since the descendant is contained by it.
        return move_to_break;
      case AXMoveType::kSibling:
        // For Sibling moves, abort if at both of the siblings are a page
        // break, because that would mean exiting and/or entering a page break.
        return move_from_break && move_to_break;
    }
    BASE_UNREACHABLE();
    return false;
  }

  static bool AbortMoveAtStartOfInlineBlock(const AXPosition& move_from,
                                            const AXPosition& move_to,
                                            const AXMoveType move_type,
                                            const AXMoveDirection direction) {
    if (move_from.IsNullPosition() || move_to.IsNullPosition())
      return true;

    // These will only be available if AXMode has kHTML set.
    const bool move_from_is_inline_block =
        move_from.GetAnchor()->GetStringAttribute(
            ax::mojom::StringAttribute::kDisplay) == "inline-block";
    const bool move_to_is_inline_block =
        move_to.GetAnchor()->GetStringAttribute(
            ax::mojom::StringAttribute::kDisplay) == "inline-block";

    switch (direction) {
      case AXMoveDirection::kNextInTree:
        // When moving forward, break if we enter an inline block.
        return move_to_is_inline_block &&
               (move_type == AXMoveType::kDescendant ||
                move_type == AXMoveType::kSibling);
      case AXMoveDirection::kPreviousInTree:
        // When moving backward, break if we exit an inline block.
        return move_from_is_inline_block &&
               (move_type == AXMoveType::kAncestor ||
                move_type == AXMoveType::kSibling);
    }
    BASE_UNREACHABLE();
    return false;
  }

  static AXPositionAdjustmentBehavior AdjustmentBehaviorFromBoundaryDirection(
      ax::mojom::MoveDirection move_direction) {
    switch (move_direction) {
      case ax::mojom::MoveDirection::kForward:
        return AXPositionAdjustmentBehavior::kMoveForward;
      case ax::mojom::MoveDirection::kBackward:
        return AXPositionAdjustmentBehavior::kMoveBackward;
    }
  }

  static AXPositionAdjustmentBehavior OppositeAdjustmentBehavior(
      AXPositionAdjustmentBehavior adjustment_behavior) {
    switch (adjustment_behavior) {
      case AXPositionAdjustmentBehavior::kMoveForward:
        return AXPositionAdjustmentBehavior::kMoveBackward;
      case AXPositionAdjustmentBehavior::kMoveBackward:
        return AXPositionAdjustmentBehavior::kMoveForward;
    }
  }

  static std::vector<int32_t> GetWordStartOffsetsFunc(
      const AXPositionInstance& position) {
    return position->GetWordStartOffsets();
  }

  static std::vector<int32_t> GetWordEndOffsetsFunc(
      const AXPositionInstance& position) {
    return position->GetWordEndOffsets();
  }

  AXPositionInstance CreateDocumentAncestorPosition() const {
    AXPositionInstance iterator = Clone();
    while (!iterator->IsNullPosition()) {
      if (IsDocument(iterator->GetAnchorRole()) &&
          iterator->CreateParentPosition()->IsNullPosition()) {
        break;
      }
      iterator = iterator->CreateParentPosition();
    }
    return iterator;
  }

  // Creates a text position that is in the same anchor as the current
  // position, but starting from the current text offset, adjusts to the next
  // or the previous boundary offset depending on the boundary direction. If
  // there is no next / previous offset, the current text offset is unchanged.
  AXPositionInstance CreatePositionAtNextOffsetBoundary(
      ax::mojom::MoveDirection move_direction,
      BoundaryTextOffsetsFunc get_offsets) const {
    if (IsNullPosition() || !get_offsets)
      return Clone();

    AXPositionInstance text_position = AsTextPosition();
    const std::vector<int32_t> boundary_offsets = get_offsets(text_position);
    if (boundary_offsets.empty())
      return text_position;

    switch (move_direction) {
      case ax::mojom::MoveDirection::kForward: {
        const auto offsets_iterator =
            std::upper_bound(boundary_offsets.begin(), boundary_offsets.end(),
                             int32_t{text_position->text_offset_});
        // If there is no next offset, the current offset should be unchanged.
        if (offsets_iterator < boundary_offsets.end()) {
          text_position->text_offset_ = static_cast<int>(*offsets_iterator);
          text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
        }
        break;
      }
      case ax::mojom::MoveDirection::kBackward: {
        auto offsets_iterator =
            std::lower_bound(boundary_offsets.begin(), boundary_offsets.end(),
                             int32_t{text_position->text_offset_});
        // If there is no previous offset, the current offset should be
        // unchanged.
        if (offsets_iterator > boundary_offsets.begin()) {
          // Since we already checked if "boundary_offsets" are non-empty, we
          // can safely move the iterator one position back, even if it's
          // currently at the vector's end.
          --offsets_iterator;
          text_position->text_offset_ = static_cast<int>(*offsets_iterator);
          text_position->affinity_ = ax::mojom::TextAffinity::kDownstream;
        }
        break;
      }
    }

    return text_position;
  }

  // Creates a text position that is in the same anchor as the current
  // position, but adjusts its text offset to be either at the first or last
  // offset boundary, based on the boundary direction. When moving forward,
  // the text position is adjusted to point to the first offset boundary, or
  // to the end of its anchor if there are no offset boundaries. When moving
  // backward, it is adjusted to point to the last offset boundary, or to the
  // start of its anchor if there are no offset boundaries.
  AXPositionInstance CreatePositionAtFirstOffsetBoundary(
      ax::mojom::MoveDirection move_direction,
      BoundaryTextOffsetsFunc get_offsets) const {
    if (IsNullPosition() || !get_offsets)
      return Clone();

    AXPositionInstance text_position = AsTextPosition();
    const std::vector<int32_t> boundary_offsets = get_offsets(text_position);
    switch (move_direction) {
      case ax::mojom::MoveDirection::kForward:
        if (boundary_offsets.empty()) {
          return text_position->CreatePositionAtEndOfAnchor();
        } else {
          text_position->text_offset_ = static_cast<int>(boundary_offsets[0]);
          return text_position;
        }
        break;
      case ax::mojom::MoveDirection::kBackward:
        if (boundary_offsets.empty()) {
          return text_position->CreatePositionAtStartOfAnchor();
        } else {
          text_position->text_offset_ =
              static_cast<int>(boundary_offsets[boundary_offsets.size() - 1]);
          return text_position;
        }
        break;
    }
  }

  // Returns the next leaf text position in the specified direction ensuring
  // that *AsLeafTextPosition() != *CreateAdjacentLeafTextPosition() is true;
  // returns a null position if no adjacent position exists.
  //
  // This method is the first step for CreateBoundary[Start|End]Position to
  // guarantee that the resulting position when using a boundary behavior other
  // than StopIfAlreadyAtBoundary is not equivalent to the initial position.
  //
  // Note that using CompareTo with text positions does not take into account
  // position affinity or tree pre-order, two text positions are considered
  // equivalent if their offsets in the text representation of the entire AXTree
  // are the same. As such, using Create[Next|Previous]LeafTextPosition is not
  // enough to create adjacent positions, e.g. the end of an anchor and the
  // start of the next one are equivalent; furthermore, there could be nodes
  // with no text representation between them, all of them being equivalent too.
  //
  // IMPORTANT: This method basically moves the given position one character
  // forward/backward, but it could end up at the middle of a grapheme cluster,
  // so it shouldn't be used to move by ax::mojom::TextBoundary::kCharacter (for
  // such purpose use Create[Next|Previous]CharacterPosition instead).
  AXPositionInstance CreateAdjacentLeafTextPosition(
      ax::mojom::MoveDirection move_direction) const {
    AXPositionInstance text_position = AsLeafTextPosition();

    switch (move_direction) {
      case ax::mojom::MoveDirection::kForward:
        // If we are at a text offset less than MaxTextOffset, we will simply
        // increase the offset by one; otherwise, create a position at the start
        // of the next leaf node with non-empty text and increase its offset.
        //
        // Note that a position located at offset 0 of an empty text node is
        // considered both, at the start and at the end of its anchor, so the
        // following loop skips over empty text leaf nodes, which is expected
        // since those positions are equivalent to both, the previous non-empty
        // leaf node's end and the next non-empty leaf node's start.
        while (text_position->AtEndOfAnchor()) {
          text_position = text_position->CreateNextLeafTextPosition();
        }
        if (!text_position->IsNullPosition())
          ++text_position->text_offset_;
        break;
      case ax::mojom::MoveDirection::kBackward:
        // If we are at a text offset greater than 0, we will simply decrease
        // the offset by one; otherwise, create a position at the end of the
        // previous leaf node with non-empty text and decrease its offset.
        //
        // Same as the comment above, using AtStartOfAnchor is enough to skip
        // empty text nodes that are equivalent to the initial position.
        while (text_position->AtStartOfAnchor()) {
          text_position = text_position->CreatePreviousLeafTextPosition()
                              ->CreatePositionAtEndOfAnchor();
        }
        if (!text_position->IsNullPosition())
          --text_position->text_offset_;
        break;
    }

    BASE_DCHECK(text_position->IsValid());
    return text_position;
  }

  AXPositionKind kind_;
  AXTreeID tree_id_;
  AXNode::AXID anchor_id_;

  // For text positions, |child_index_| is initially set to |-1| and only
  // computed on demand. The same with tree positions and |text_offset_|.
  int child_index_;
  // "text_offset_" represents the number of UTF16 code units before this
  // position. It doesn't count grapheme clusters.
  int text_offset_;

  // Affinity is used to distinguish between two text positions that point to
  // the same text offset, but which happens to fall on a soft line break. A
  // soft line break doesn't insert any white space in the accessibility tree,
  // so without affinity there would be no way to determine whether a text
  // position is before or after the soft line break. An upstream affinity
  // means that the position is before the soft line break, whilst a
  // downstream affinity means that the position is after the soft line break.
  //
  // Please note that affinity could only be set to upstream for positions
  // that are anchored to non-leaf nodes. When on a leaf node, there could
  // never be an ambiguity as to which line a position points to because Blink
  // creates separate inline text boxes for each line of text. Therefore, a
  // leaf text position before the soft line break would be pointing to the
  // end of its anchor node, whilst a leaf text position after the soft line
  // break would be pointing to the start of the next node.
  ax::mojom::TextAffinity affinity_;
};

template <class AXPositionType, class AXNodeType>
const int AXPosition<AXPositionType, AXNodeType>::BEFORE_TEXT;
template <class AXPositionType, class AXNodeType>
const int AXPosition<AXPositionType, AXNodeType>::INVALID_INDEX;
template <class AXPositionType, class AXNodeType>
const int AXPosition<AXPositionType, AXNodeType>::INVALID_OFFSET;

template <class AXPositionType, class AXNodeType>
bool operator==(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() == 0;
}

template <class AXPositionType, class AXNodeType>
bool operator!=(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() != 0;
}

template <class AXPositionType, class AXNodeType>
bool operator<(const AXPosition<AXPositionType, AXNodeType>& first,
               const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() < 0;
}

template <class AXPositionType, class AXNodeType>
bool operator<=(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() <= 0;
}

template <class AXPositionType, class AXNodeType>
bool operator>(const AXPosition<AXPositionType, AXNodeType>& first,
               const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() > 0;
}

template <class AXPositionType, class AXNodeType>
bool operator>=(const AXPosition<AXPositionType, AXNodeType>& first,
                const AXPosition<AXPositionType, AXNodeType>& second) {
  const std::optional<int> compare_to_optional = first.CompareTo(second);
  return compare_to_optional.has_value() && compare_to_optional.value() >= 0;
}

template <class AXPositionType, class AXNodeType>
void swap(AXPosition<AXPositionType, AXNodeType>& first,
          AXPosition<AXPositionType, AXNodeType>& second) {
  first.swap(second);
}

template <class AXPositionType, class AXNodeType>
std::ostream& operator<<(
    std::ostream& stream,
    const AXPosition<AXPositionType, AXNodeType>& position) {
  return stream << position.ToString();
}

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_POSITION_H_
