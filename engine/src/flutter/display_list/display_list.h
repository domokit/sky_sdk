// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DISPLAY_LIST_H_
#define FLUTTER_DISPLAY_LIST_DISPLAY_LIST_H_

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_storage.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/display_list/geometry/dl_rtree.h"

// The Flutter DisplayList mechanism encapsulates a persistent sequence of
// rendering operations.
//
// This file contains the definitions for:
// DisplayList: the base class that holds the information about the
//              sequence of operations and can dispatch them to a DlOpReceiver
// DlOpReceiver: a pure virtual interface which can be implemented to field
//               the requests for purposes such as sending them to an SkCanvas
//               or detecting various rendering optimization scenarios
// DisplayListBuilder: a class for constructing a DisplayList from DlCanvas
//                     method calls and which can act as a DlOpReceiver as well
//
// Other files include various class definitions for dealing with display
// lists, such as:
// skia/dl_sk_*.h: classes to interact between SkCanvas and DisplayList
//                 (SkCanvas->DisplayList adapter and vice versa)
//
// display_list_utils.h: various utility classes to ease implementing
//                       a DlOpReceiver, including NOP implementations of
//                       the attribute, clip, and transform methods,
//                       classes to track attributes, clips, and transforms
//                       and a class to compute the bounds of a DisplayList
//                       Any class implementing DlOpReceiver can inherit from
//                       these utility classes to simplify its creation
//
// The Flutter DisplayList mechanism is used in a similar manner to the Skia
// SkPicture mechanism.
//
// A DisplayList must be created using a DisplayListBuilder using its stateless
// methods inherited from DlCanvas.
//
// A DisplayList can be read back by implementing the DlOpReceiver virtual
// methods (with help from some of the classes in the utils file) and
// passing an instance to the Dispatch() method, or it can be rendered
// to Skia using a DlSkCanvasDispatcher.
//
// The mechanism is inspired by the SkLiteDL class that is not directly
// supported by Skia, but has been recommended as a basis for custom
// display lists for a number of their customers.

namespace flutter {

#define FOR_EACH_DISPLAY_LIST_OP(V) \
  V(SetAntiAlias)                   \
  V(SetInvertColors)                \
                                    \
  V(SetStrokeCap)                   \
  V(SetStrokeJoin)                  \
                                    \
  V(SetStyle)                       \
  V(SetStrokeWidth)                 \
  V(SetStrokeMiter)                 \
                                    \
  V(SetColor)                       \
  V(SetBlendMode)                   \
                                    \
  V(ClearColorFilter)               \
  V(SetPodColorFilter)              \
                                    \
  V(ClearColorSource)               \
  V(SetPodColorSource)              \
  V(SetImageColorSource)            \
  V(SetRuntimeEffectColorSource)    \
                                    \
  V(ClearImageFilter)               \
  V(SetPodImageFilter)              \
  V(SetSharedImageFilter)           \
                                    \
  V(ClearMaskFilter)                \
  V(SetPodMaskFilter)               \
                                    \
  V(Save)                           \
  V(SaveLayer)                      \
  V(SaveLayerBackdrop)              \
  V(Restore)                        \
                                    \
  V(Translate)                      \
  V(Scale)                          \
  V(Rotate)                         \
  V(Skew)                           \
  V(Transform2DAffine)              \
  V(TransformFullPerspective)       \
  V(TransformReset)                 \
                                    \
  V(ClipIntersectRect)              \
  V(ClipIntersectOval)              \
  V(ClipIntersectRoundRect)         \
  V(ClipIntersectPath)              \
  V(ClipDifferenceRect)             \
  V(ClipDifferenceOval)             \
  V(ClipDifferenceRoundRect)        \
  V(ClipDifferencePath)             \
                                    \
  V(DrawPaint)                      \
  V(DrawColor)                      \
                                    \
  V(DrawLine)                       \
  V(DrawDashedLine)                 \
  V(DrawRect)                       \
  V(DrawOval)                       \
  V(DrawCircle)                     \
  V(DrawRoundRect)                  \
  V(DrawDiffRoundRect)              \
  V(DrawArc)                        \
  V(DrawPath)                       \
                                    \
  V(DrawPoints)                     \
  V(DrawLines)                      \
  V(DrawPolygon)                    \
  V(DrawVertices)                   \
                                    \
  V(DrawImage)                      \
  V(DrawImageWithAttr)              \
  V(DrawImageRect)                  \
  V(DrawImageNine)                  \
  V(DrawImageNineWithAttr)          \
  V(DrawAtlas)                      \
  V(DrawAtlasCulled)                \
                                    \
  V(DrawDisplayList)                \
  V(DrawTextBlob)                   \
  V(DrawTextFrame)                  \
                                    \
  V(DrawShadow)                     \
  V(DrawShadowTransparentOccluder)

#define DL_OP_TO_ENUM_VALUE(name) k##name,
enum class DisplayListOpType {
  FOR_EACH_DISPLAY_LIST_OP(DL_OP_TO_ENUM_VALUE)

  // empty comment to make formatter happy
  kInvalidOp,
  kMaxOp = kInvalidOp,
};
#undef DL_OP_TO_ENUM_VALUE

enum class DisplayListOpCategory {
  kAttribute,
  kTransform,
  kClip,
  kSave,
  kSaveLayer,
  kRestore,
  kRendering,
  kSubDisplayList,
  kInvalidCategory,
  kMaxCategory = kInvalidCategory,
};

class DlOpReceiver;
class DisplayListBuilder;

class SaveLayerOptions {
 public:
  static const SaveLayerOptions kWithAttributes;
  static const SaveLayerOptions kNoAttributes;

  SaveLayerOptions() : flags_(0) {}
  SaveLayerOptions(const SaveLayerOptions& options) : flags_(options.flags_) {}
  explicit SaveLayerOptions(const SaveLayerOptions* options)
      : flags_(options->flags_) {}

  SaveLayerOptions without_optimizations() const {
    SaveLayerOptions options;
    options.fRendersWithAttributes = fRendersWithAttributes;
    options.fBoundsFromCaller = fBoundsFromCaller;
    return options;
  }

  bool renders_with_attributes() const { return fRendersWithAttributes; }
  SaveLayerOptions with_renders_with_attributes() const {
    SaveLayerOptions options(this);
    options.fRendersWithAttributes = true;
    return options;
  }

  bool can_distribute_opacity() const { return fCanDistributeOpacity; }
  SaveLayerOptions with_can_distribute_opacity() const {
    SaveLayerOptions options(this);
    options.fCanDistributeOpacity = true;
    return options;
  }

  // Returns true iff the bounds for the saveLayer operation were provided
  // by the caller, otherwise the bounds will have been computed by the
  // DisplayListBuilder and provided for reference.
  bool bounds_from_caller() const { return fBoundsFromCaller; }
  SaveLayerOptions with_bounds_from_caller() const {
    SaveLayerOptions options(this);
    options.fBoundsFromCaller = true;
    return options;
  }
  SaveLayerOptions without_bounds_from_caller() const {
    SaveLayerOptions options(this);
    options.fBoundsFromCaller = false;
    return options;
  }
  bool bounds_were_calculated() const { return !fBoundsFromCaller; }

  // Returns true iff the bounds for the saveLayer do not fully cover the
  // contained rendering operations. This will only occur if the original
  // caller supplied bounds and those bounds were not a strict superset
  // of the content bounds computed by the DisplayListBuilder.
  bool content_is_clipped() const { return fContentIsClipped; }
  SaveLayerOptions with_content_is_clipped() const {
    SaveLayerOptions options(this);
    options.fContentIsClipped = true;
    return options;
  }

  bool contains_backdrop_filter() const { return fHasBackdropFilter; }
  SaveLayerOptions with_contains_backdrop_filter() const {
    SaveLayerOptions options(this);
    options.fHasBackdropFilter = true;
    return options;
  }

  bool content_is_unbounded() const { return fContentIsUnbounded; }
  SaveLayerOptions with_content_is_unbounded() const {
    SaveLayerOptions options(this);
    options.fContentIsUnbounded = true;
    return options;
  }

  SaveLayerOptions& operator=(const SaveLayerOptions& other) {
    flags_ = other.flags_;
    return *this;
  }
  bool operator==(const SaveLayerOptions& other) const {
    return flags_ == other.flags_;
  }
  bool operator!=(const SaveLayerOptions& other) const {
    return flags_ != other.flags_;
  }

 private:
  union {
    struct {
      unsigned fRendersWithAttributes : 1;
      unsigned fCanDistributeOpacity : 1;
      unsigned fBoundsFromCaller : 1;
      unsigned fContentIsClipped : 1;
      unsigned fHasBackdropFilter : 1;
      unsigned fContentIsUnbounded : 1;
    };
    uint32_t flags_;
  };
};

using DlIndex = uint32_t;

// The base class that contains a sequence of rendering operations
// for dispatch to a DlOpReceiver. These objects must be instantiated
// through an instance of DisplayListBuilder::build().
class DisplayList : public SkRefCnt {
 public:
  DisplayList();

  ~DisplayList();

  void Dispatch(DlOpReceiver& ctx) const;
  void Dispatch(DlOpReceiver& ctx, const SkRect& cull_rect) const {
    Dispatch(ctx, ToDlRect(cull_rect));
  }
  void Dispatch(DlOpReceiver& ctx, const SkIRect& cull_rect) const {
    Dispatch(ctx, ToDlIRect(cull_rect));
  }
  void Dispatch(DlOpReceiver& ctx, const DlRect& cull_rect) const;
  void Dispatch(DlOpReceiver& ctx, const DlIRect& cull_rect) const;

  // From historical behavior, SkPicture always included nested bytes,
  // but nested ops are only included if requested. The defaults used
  // here for these accessors follow that pattern.
  size_t bytes(bool nested = true) const {
    return sizeof(DisplayList) + storage_.size() +
           (nested ? nested_byte_count_ : 0);
  }

  uint32_t op_count(bool nested = false) const {
    return op_count_ + (nested ? nested_op_count_ : 0);
  }

  uint32_t total_depth() const { return total_depth_; }

  uint32_t unique_id() const { return unique_id_; }

  const SkRect& bounds() const { return ToSkRect(bounds_); }
  const DlRect& GetBounds() const { return bounds_; }

  bool has_rtree() const { return rtree_ != nullptr; }
  sk_sp<const DlRTree> rtree() const { return rtree_; }

  bool Equals(const DisplayList* other) const;
  bool Equals(const DisplayList& other) const { return Equals(&other); }
  bool Equals(const sk_sp<const DisplayList>& other) const {
    return Equals(other.get());
  }

  bool can_apply_group_opacity() const { return can_apply_group_opacity_; }
  bool isUIThreadSafe() const { return is_ui_thread_safe_; }

  /// @brief     Indicates if there are any rendering operations in this
  ///            DisplayList that will modify a surface of transparent black
  ///            pixels.
  ///
  /// This condition can be used to determine whether to create a cleared
  /// surface, render a DisplayList into it, and then composite the
  /// result into a scene. It is not uncommon for code in the engine to
  /// come across such degenerate DisplayList objects when slicing up a
  /// frame between platform views.
  bool modifies_transparent_black() const {
    return modifies_transparent_black_;
  }

  const DisplayListStorage& GetStorage() const { return storage_; }

  /// @brief    Indicates if there are any saveLayer operations at the root
  ///           surface level of the DisplayList that use a backdrop filter.
  ///
  /// This condition can be used to determine what kind of surface to create
  /// for the root layer into which to render the DisplayList as some GPUs
  /// can support surfaces that do or do not support the readback that would
  /// be required for the backdrop filter to do its work.
  bool root_has_backdrop_filter() const { return root_has_backdrop_filter_; }

  /// @brief    Indicates if a rendering operation at the root level of the
  ///           DisplayList had an unbounded result, not otherwise limited by
  ///           a clip operation.
  ///
  /// This condition can occur in a number of situations. The most common
  /// situation is when there is a drawPaint or drawColor rendering
  /// operation which fills out the entire drawable surface unless it is
  /// bounded by a clip. Other situations include an operation rendered
  /// through an ImageFilter that cannot compute the resulting bounds or
  /// when an unclipped backdrop filter is applied by a save layer.
  bool root_is_unbounded() const { return root_is_unbounded_; }

  /// @brief    Indicates the maximum DlBlendMode used on any rendering op
  ///           in the root surface of the DisplayList.
  ///
  /// This condition can be used to determine what kind of surface to create
  /// for the root layer into which to render the DisplayList as some GPUs
  /// can support surfaces that do or do not support the readback that would
  /// be required for the indicated blend mode to do its work.
  DlBlendMode max_root_blend_mode() const { return max_root_blend_mode_; }

  /// @brief   Iterator utility class used for the |DisplayList::begin|
  ///          and |DisplayList::end| methods. It implements just the
  ///          basic methods to enable iteration-style for loops.
  class Iterator {
   public:
    DlIndex operator*() const { return value_; }
    bool operator!=(const Iterator& other) { return value_ != other.value_; }
    Iterator& operator++() {
      value_++;
      return *this;
    }

   private:
    explicit Iterator(DlIndex value) : value_(value) {}

    DlIndex value_;

    friend class DisplayList;
  };

  /// @brief   Return the number of stored records in the DisplayList.
  ///
  /// Each stored record represents a dispatchable operation that will be
  /// sent to a |DlOpReceiver| by the |Dispatch| method. You can directly
  /// simulate the |Dispatch| method using a simple for loop on the indices:
  ///
  /// {
  ///   for (DlIndex i = 0u; i < display_list->GetRecordCount(); i++) {
  ///     display_list->Dispatch(my_receiver, i);
  ///   }
  /// }
  ///
  /// @see |Dispatch(receiver, index)|
  /// @see |begin|
  /// @see |end|
  /// @see |GetCulledIndices|
  DlIndex GetRecordCount() const { return offsets_.size(); }

  /// @brief   Return an iterator to the start of the stored records,
  ///          enabling the iteration form of a for loop.
  ///
  /// Each stored record represents a dispatchable operation that will be
  /// sent to a |DlOpReceiver| by the |Dispatch| method. You can directly
  /// simulate the |Dispatch| method using a simple for loop on the indices:
  ///
  /// {
  ///   for (DlIndex i : *display_list) {
  ///     display_list->Dispatch(my_receiver, i);
  ///   }
  /// }
  ///
  /// @see |end|
  /// @see |GetCulledIndices|
  Iterator begin() const { return Iterator(0u); }

  /// @brief   Return an iterator to the end of the stored records,
  ///          enabling the iteration form of a for loop.
  ///
  /// Each stored record represents a dispatchable operation that will be
  /// sent to a |DlOpReceiver| by the |Dispatch| method. You can directly
  /// simulate the |Dispatch| method using a simple for loop on the indices:
  ///
  /// {
  ///   for (DlIndex i : *display_list) {
  ///     display_list->Dispatch(my_receiver, i);
  ///   }
  /// }
  ///
  /// @see |begin|
  /// @see |GetCulledIndices|
  Iterator end() const { return Iterator(offsets_.size()); }

  /// @brief   Dispatch a single stored operation by its index.
  ///
  /// Each stored record represents a dispatchable operation that will be
  /// sent to a |DlOpReceiver| by the |Dispatch| method. You can use this
  /// method to dispatch a single operation to your receiver with an index
  /// between |0u| (inclusive) and |GetRecordCount()| (exclusive), as in:
  ///
  /// {
  ///   for (DlIndex i = 0u; i < display_list->GetRecordCount(); i++) {
  ///     display_list->Dispatch(my_receiver, i);
  ///   }
  /// }
  ///
  /// If the index is out of the range of the stored records, this method
  /// will not call any methods on the receiver and return false. You can
  /// check the return value for true if you want to make sure you are
  /// using valid indices.
  ///
  /// @see |GetRecordCount|
  /// @see |begin|
  /// @see |end|
  /// @see |GetCulledIndices|
  bool Dispatch(DlOpReceiver& receiver, DlIndex index) const;

  /// @brief   Return an enum describing the specific op type stored at
  ///          the indicated index.
  ///
  /// The specific types of the records are subject to change without notice
  /// as the DisplayList code is developed and optimized. These values are
  /// useful mostly for debugging purposes and should not be used in
  /// production code.
  ///
  /// @see |GetOpCategory| for a more stable description of the records
  DisplayListOpType GetOpType(DlIndex index) const;

  /// @brief   Return an enum describing the general category of the
  ///          operation record stored at the indicated index.
  ///
  /// The categories are general and stable and can be used fairly safely
  /// in production code to plan how to dispatch or reorder ops during
  /// final rendering.
  ///
  /// @see |GetOpType| for a more detailed description of the records
  ///                  primarily for debugging use
  DisplayListOpCategory GetOpCategory(DlIndex index) const;

  /// @brief   Return an enum describing the general category of the
  ///          operation record with the given type.
  ///
  /// @see |GetOpType| for a more detailed description of the records
  ///                  primarily for debugging use
  static DisplayListOpCategory GetOpCategory(DisplayListOpType type);

  /// @brief   Return a vector of valid indices for records stored in
  ///          the DisplayList that must be dispatched if you are
  ///          restricted to the indicated cull_rect.
  ///
  /// This method can be used along with indexed dispatching to implement
  /// RTree culling while still maintaining control over planning of
  /// operations to be rendered, as in:
  ///
  /// {
  ///   std::vector<DlIndex> indices =
  ///       display_list->GetCulledIndices(cull-rect);
  ///   for (DlIndex i : indices) {
  ///     display_list->Dispatch(my_receiver, i);
  ///   }
  /// }
  ///
  /// The indices returned in the vector will automatically deal with
  /// including or culling related operations such as attributes, clips
  /// and transforms that will provide state for any rendering operations
  /// selected by the culling checks.
  ///
  /// @see |GetOpType| for a more detailed description of the records
  ///                  primarily for debugging use
  ///
  /// @see |Dispatch(receiver, index)|
  std::vector<DlIndex> GetCulledIndices(const DlRect& cull_rect) const;

 private:
  DisplayList(DisplayListStorage&& ptr,
              std::vector<size_t>&& offsets,
              uint32_t op_count,
              size_t nested_byte_count,
              uint32_t nested_op_count,
              uint32_t total_depth,
              const DlRect& bounds,
              bool can_apply_group_opacity,
              bool is_ui_thread_safe,
              bool modifies_transparent_black,
              DlBlendMode max_root_blend_mode,
              bool root_has_backdrop_filter,
              bool root_is_unbounded,
              sk_sp<const DlRTree> rtree);

  static uint32_t next_unique_id();

  static void DisposeOps(const DisplayListStorage& storage,
                         const std::vector<size_t>& offsets);

  const DisplayListStorage storage_;
  const std::vector<size_t> offsets_;

  const uint32_t op_count_;
  const size_t nested_byte_count_;
  const uint32_t nested_op_count_;

  const uint32_t total_depth_;

  const uint32_t unique_id_;
  const DlRect bounds_;

  const bool can_apply_group_opacity_;
  const bool is_ui_thread_safe_;
  const bool modifies_transparent_black_;
  const bool root_has_backdrop_filter_;
  const bool root_is_unbounded_;
  const DlBlendMode max_root_blend_mode_;

  const sk_sp<const DlRTree> rtree_;

  void DispatchOneOp(DlOpReceiver& receiver, const uint8_t* ptr) const;

  void RTreeResultsToIndexVector(std::vector<DlIndex>& indices,
                                 const std::vector<int>& rtree_results) const;

  friend class DisplayListBuilder;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DISPLAY_LIST_H_
