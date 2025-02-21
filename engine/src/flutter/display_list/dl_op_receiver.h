// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_OP_RECEIVER_H_
#define FLUTTER_DISPLAY_LIST_DL_OP_RECEIVER_H_

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_vertices.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/display_list/effects/dl_mask_filter.h"
#include "flutter/display_list/image/dl_image.h"

namespace flutter {

class DisplayList;

//------------------------------------------------------------------------------
/// @brief      Internal API for rendering recorded display lists to backends.
///
/// The |DisplayList| object will play back recorded operations in this format.
/// Most developers should not need to deal with this interface unless they are
/// writing a utility that needs to examine the contents of a display list.
///
/// Similar to |DlCanvas|, this interface carries clip and transform state
/// which are saved and restored by the |save|, |saveLayer|, and |restore|
/// calls.
///
/// Unlike DlCanvas, this interface has attribute state which is global across
/// an entire DisplayList (not affected by save/restore).
///
/// DISPLAYLIST DEPTH TRACKING
///
/// Each rendering call in the DisplayList stream is assumed to have a "depth"
/// value relative to the beginning of its DisplayList. The depth value is
/// implicitly allocated during recording and only reported in 2 places so
/// it is important for a dispatcher to perform the same internal allocations
/// if it is to make sense of the information reported by the save/saveLayer
/// calls. This depth value is maintained as follows:
///
/// - The absolute depth value is never reported, only the total depth
///   size of the entire DisplayList or one of its save/restore pairs
///   is reported. Since the DisplayList might be dispatched recursively
///   due to embedded drawDisplayList calls, these depth size values
///   will often be relative to things like:
///     - the start of a given save/saveLayer group
///     - the start of a DisplayList dispatch or recursion
///   as such, only totals for groups of DisplayList dispatched calls
///   will be reported. These totals will be reported in:
///     - the `DisplayList::total_depth()` method reporting the total
///       depth accumulated for every operation in the DisplayList
///     - the save/saveLayer dispatch calls will report the total
///       depth accumulated for every call until their corresponding
///       restore call.
/// - The depth value is incremented for every drawing operation, including:
///   - all draw* calls (including drawDisplayList)
///   - drawDisplayList will also accumulate the total_depth() of the
///     DisplayList object it is drawing (in other words it will skip enough
///     depth values for each drawing call in the child).
///     This bump is in addition to the depth value it records for being
///     a rendering operation. Some implementations may need to surround
///     the actual drawDisplayList with a protective saveLayer, but others
///     may not - so the implicit depth value assigned to the drawDisplayList
///     call itself may go unused, but must be accounted for.
///   - a saveLayer call will also increment the depth value just like a
///     rendering call. This is in addition to the depth of its content.
///     It is doing so to reserve a depth for the drawing operation that
///     copies its layer back to the parent.
/// - Each save() or saveLayer() call will report the total depth of all
///   rendering calls within its content (recorded before the corresponding
///   restore) and report this total during dispatch. This information might
///   be needed to assign depths to the clip operations that occur within
///   its content. As there is no enclosing saveLayer/restore pair around
///   the root of a DisplayList, the total depth of the DisplayList can
///   be used to determine the appropriate clip depths for any clip ops
///   appearing before the first save/saveLayer or after the last restore.
///
/// @see        DlSkCanvasDispatcher
/// @see        impeller::DlDispatcher
/// @see        DlOpSpy
class DlOpReceiver {
 public:
  // MaxDrawPointsCount * sizeof(DlPoint) must be less than 1 << 32
  static constexpr int kMaxDrawPointsCount = ((1 << 29) - 1);

  // The following methods are nearly 1:1 with the methods on DlPaint and
  // carry the same meanings. Each method sets a persistent value for the
  // attribute for the rest of the display list or until it is reset by
  // another method that changes the same attribute. The current set of
  // attributes is not affected by |save| and |restore|.
  virtual void setAntiAlias(bool aa) = 0;
  virtual void setDrawStyle(DlDrawStyle style) = 0;
  virtual void setColor(DlColor color) = 0;
  virtual void setStrokeWidth(float width) = 0;
  virtual void setStrokeMiter(float limit) = 0;
  virtual void setStrokeCap(DlStrokeCap cap) = 0;
  virtual void setStrokeJoin(DlStrokeJoin join) = 0;
  virtual void setColorSource(const DlColorSource* source) = 0;
  virtual void setColorFilter(const DlColorFilter* filter) = 0;
  // setInvertColors is a quick way to set a ColorFilter that inverts the
  // rgb values of all rendered colors.
  // It is not reset by |setColorFilter|, but instead composed with that
  // filter so that the color inversion happens after the ColorFilter.
  virtual void setInvertColors(bool invert) = 0;
  virtual void setBlendMode(DlBlendMode mode) = 0;
  virtual void setMaskFilter(const DlMaskFilter* filter) = 0;
  virtual void setImageFilter(const DlImageFilter* filter) = 0;

  // All of the following methods are nearly 1:1 with their counterparts
  // in |SkCanvas| and have the same behavior and output.
  virtual void save() = 0;
  // Optional variant of save() that passes the total depth count of
  // all rendering operations that occur until the next restore() call.
  virtual void save(uint32_t total_content_depth) { save(); }
  // The |options| parameter can specify whether the existing rendering
  // attributes will be applied to the save layer surface while rendering
  // it back to the current surface. If the flag is false then this method
  // is equivalent to |SkCanvas::saveLayer| with a null paint object.
  //
  // The |options| parameter can also specify whether the bounds came from
  // the caller who recorded the operation, or whether they were calculated
  // by the DisplayListBuilder.
  //
  // The |options| parameter may contain other options that indicate some
  // specific optimizations may be made by the underlying implementation
  // to avoid creating a temporary layer, these optimization options will
  // be determined as the |DisplayList| is constructed and should not be
  // specified in calling a |DisplayListBuilder| as they will be ignored.
  // The |backdrop| filter, if not null, is used to initialize the new
  // layer before further rendering happens.
  virtual void saveLayer(const DlRect& bounds,
                         const SaveLayerOptions options,
                         const DlImageFilter* backdrop = nullptr,
                         std::optional<int64_t> backdrop_id = std::nullopt) = 0;
  // Optional variant of saveLayer() that passes the total depth count of
  // all rendering operations that occur until the next restore() call.
  virtual void saveLayer(const DlRect& bounds,
                         const SaveLayerOptions& options,
                         uint32_t total_content_depth,
                         DlBlendMode max_content_blend_mode,
                         const DlImageFilter* backdrop = nullptr,
                         std::optional<int64_t> backdrop_id = std::nullopt) {
    saveLayer(bounds, options, backdrop, backdrop_id);
  }
  virtual void restore() = 0;

  // ---------------------------------------------------------------------
  // Legacy helper method for older callers that use the null-ness of
  // the bounds to indicate if they should be recorded or computed.
  // This method will not be called on a |DlOpReceiver| that is passed
  // to the |DisplayList::Dispatch()| method, so client receivers should
  // ignore it for their implementation purposes.
  //
  // DlOpReceiver methods are generally meant to ONLY be output from a
  // previously recorded DisplayList so this method is really only used
  // from testing methods that bypass the public builder APIs for legacy
  // convenience or for internal white-box testing of the DisplayList
  // internals. Such methods should eventually be converted to using the
  // public DisplayListBuilder/DlCanvas public interfaces where possible,
  // as tracked in:
  // https://github.com/flutter/flutter/issues/144070
  virtual void saveLayer(
      const DlRect* bounds,
      const SaveLayerOptions options,
      const DlImageFilter* backdrop = nullptr,
      std::optional<int64_t> backdrop_id = std::nullopt) final {
    if (bounds) {
      saveLayer(*bounds, options.with_bounds_from_caller(), backdrop,
                backdrop_id);
    } else {
      saveLayer(DlRect(), options.without_bounds_from_caller(), backdrop,
                backdrop_id);
    }
  }
  // ---------------------------------------------------------------------

  virtual void translate(DlScalar tx, DlScalar ty) = 0;
  virtual void scale(DlScalar sx, DlScalar sy) = 0;
  virtual void rotate(DlScalar degrees) = 0;
  virtual void skew(DlScalar sx, DlScalar sy) = 0;

  // The transform methods all assume the following math for transforming
  // an arbitrary 3D homogenous point (x, y, z, w).
  // All coordinates in the rendering methods (and DlPoint and DlRect objects)
  // represent a simplified coordinate (x, y, 0, 1).
  //   x' = x * mxx + y * mxy + z * mxz + w * mxt
  //   y' = x * myx + y * myy + z * myz + w * myt
  //   z' = x * mzx + y * mzy + z * mzz + w * mzt
  //   w' = x * mwx + y * mwy + z * mwz + w * mwt
  // Note that for non-homogenous 2D coordinates, the last column in those
  // equations is multiplied by 1 and is simply adding a translation and
  // so is referred to with the final letter "t" here instead of "w".
  //
  // In 2D coordinates, z=0 and so the 3rd column always evaluates to 0.
  //
  // In non-perspective transforms, the 4th row has identity values
  // and so w` = w. (i.e. w'=1 for 2d points transformed by a matrix
  // with identity values in the last row).
  //
  // In affine 2D transforms, the 3rd and 4th row and 3rd column are all
  // identity values and so z` = z (which is 0 for 2D coordinates) and
  // the x` and y` equations don't see a contribution from a z coordinate
  // and the w' ends up being the same as the w from the source coordinate
  // (which is 1 for a 2D coordinate).
  //
  // Here is the math for transforming a 2D source coordinate and
  // looking for the destination 2D coordinate (for a surface that
  // does not have a Z buffer or track the Z coordinates in any way)
  //  Source coordinate = (x, y, 0, 1)
  //   x' = x * mxx + y * mxy + 0 * mxz + 1 * mxt
  //   y' = x * myx + y * myy + 0 * myz + 1 * myt
  //   z' = x * mzx + y * mzy + 0 * mzz + 1 * mzt
  //   w' = x * mwx + y * mwy + 0 * mwz + 1 * mwt
  //  Destination coordinate does not need z', so this reduces to:
  //   x' = x * mxx + y * mxy + mxt
  //   y' = x * myx + y * myy + myt
  //   w' = x * mwx + y * mwy + mwt
  //  Destination coordinate is (x' / w', y' / w', 0, 1)
  // Note that these are the matrix values in SkMatrix which means that
  // an SkMatrix contains enough data to transform a 2D source coordinate
  // and place it on a 2D surface, but is otherwise not enough to continue
  // concatenating with further matrices as its missing elements will not
  // be able to model the interplay between the rows and columns that
  // happens during a full 4x4 by 4x4 matrix multiplication.
  //
  // If the transform doesn't have any perspective parts (the last
  // row is identity - 0, 0, 0, 1), then this further simplifies to:
  //   x' = x * mxx + y * mxy + mxt
  //   y' = x * myx + y * myy + myt
  //   w' = x * 0 + y * 0 + 1         = 1
  //
  // In short, while the full 4x4 set of matrix entries needs to be
  // maintained for accumulating transform mutations accurately, the
  // actual end work of transforming a single 2D coordinate (or, in
  // the case of bounds transformations, 4 of them) can be accomplished
  // with the 9 values from transform3x3 or SkMatrix.
  //
  // The only need for the w value here is for homogenous coordinates
  // which only come up if the perspective elements (the 4th row) of
  // a transform are non-identity. Otherwise the w always ends up
  // being 1 in all calculations. If the matrix has perspecitve elements
  // then the final transformed coordinates will have a w that is not 1
  // and the actual coordinates are determined by dividing out that w
  // factor resulting in a real-world point expressed as (x, y, z, 1).
  //
  // Because of the predominance of 2D affine transforms the
  // 2x3 subset of the 4x4 transform matrix is special cased with
  // its own dispatch method that omits the last 2 rows and the 3rd
  // column. Even though a 3x3 subset is enough for transforming
  // leaf coordinates as shown above, no method is provided for
  // representing a 3x3 transform in the DisplayList since if there
  // is perspective involved then a full 4x4 matrix should be provided
  // for accurate concatenations. Providing a 3x3 method or record
  // in the stream would encourage developers to prematurely subset
  // a full perspective matrix.

  // clang-format off

  // |transform2DAffine| is equivalent to concatenating the internal
  // 4x4 transform with the following row major transform matrix:
  //   [ mxx  mxy   0   mxt ]
  //   [ myx  myy   0   myt ]
  //   [  0    0    1    0  ]
  //   [  0    0    0    1  ]
  virtual void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                                 DlScalar myx, DlScalar myy, DlScalar myt) = 0;
  // |transformFullPerspective| is equivalent to concatenating the internal
  // 4x4 transform with the following row major transform matrix:
  //   [ mxx  mxy  mxz  mxt ]
  //   [ myx  myy  myz  myt ]
  //   [ mzx  mzy  mzz  mzt ]
  //   [ mwx  mwy  mwz  mwt ]
  virtual void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) = 0;
  // clang-format on

  // Clears the transformation stack.
  virtual void transformReset() = 0;

  virtual void clipRect(const DlRect& rect, DlClipOp clip_op, bool is_aa) = 0;
  virtual void clipOval(const DlRect& bounds, DlClipOp clip_op, bool is_aa) = 0;
  virtual void clipRoundRect(const DlRoundRect& rrect,
                             DlClipOp clip_op,
                             bool is_aa) = 0;
  virtual void clipRoundSuperellipse(const DlRoundSuperellipse& rse,
                                     DlClipOp clip_op,
                                     bool is_aa) = 0;
  virtual void clipPath(const DlPath& path, DlClipOp clip_op, bool is_aa) = 0;

  // The following rendering methods all take their rendering attributes
  // from the last value set by the attribute methods above (regardless
  // of any |save| or |restore| operations which do not affect attributes).
  // In cases where a paint object may have been optional in the SkCanvas
  // method, the methods here will generally offer a boolean parameter
  // which specifies whether to honor the attributes of the display list
  // stream, or assume default attributes.
  virtual void drawColor(DlColor color, DlBlendMode mode) = 0;
  virtual void drawPaint() = 0;
  virtual void drawLine(const DlPoint& p0, const DlPoint& p1) = 0;
  virtual void drawDashedLine(const DlPoint& p0,
                              const DlPoint& p1,
                              DlScalar on_length,
                              DlScalar off_length) = 0;
  virtual void drawRect(const DlRect& rect) = 0;
  virtual void drawOval(const DlRect& bounds) = 0;
  virtual void drawCircle(const DlPoint& center, DlScalar radius) = 0;
  virtual void drawRoundRect(const DlRoundRect& rrect) = 0;
  virtual void drawDiffRoundRect(const DlRoundRect& outer,
                                 const DlRoundRect& inner) = 0;
  virtual void drawRoundSuperellipse(const DlRoundSuperellipse& rse) = 0;
  virtual void drawPath(const DlPath& path) = 0;
  virtual void drawArc(const DlRect& oval_bounds,
                       DlScalar start_degrees,
                       DlScalar sweep_degrees,
                       bool use_center) = 0;
  virtual void drawPoints(DlPointMode mode,
                          uint32_t count,
                          const DlPoint points[]) = 0;
  virtual void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                            DlBlendMode mode) = 0;
  virtual void drawImage(const sk_sp<DlImage> image,
                         const DlPoint& point,
                         DlImageSampling sampling,
                         bool render_with_attributes) = 0;
  virtual void drawImageRect(
      const sk_sp<DlImage> image,
      const DlRect& src,
      const DlRect& dst,
      DlImageSampling sampling,
      bool render_with_attributes,
      DlSrcRectConstraint constraint = DlSrcRectConstraint::kFast) = 0;
  virtual void drawImageNine(const sk_sp<DlImage> image,
                             const DlIRect& center,
                             const DlRect& dst,
                             DlFilterMode filter,
                             bool render_with_attributes) = 0;
  virtual void drawAtlas(const sk_sp<DlImage> atlas,
                         const DlRSTransform xform[],
                         const DlRect tex[],
                         const DlColor colors[],
                         int count,
                         DlBlendMode mode,
                         DlImageSampling sampling,
                         const DlRect* cull_rect,
                         bool render_with_attributes) = 0;
  virtual void drawDisplayList(const sk_sp<DisplayList> display_list,
                               DlScalar opacity = SK_Scalar1) = 0;
  virtual void drawTextBlob(const sk_sp<SkTextBlob> blob,
                            DlScalar x,
                            DlScalar y) = 0;
  virtual void drawTextFrame(
      const std::shared_ptr<impeller::TextFrame>& text_frame,
      DlScalar x,
      DlScalar y) = 0;
  virtual void drawShadow(const DlPath& path,
                          const DlColor color,
                          const DlScalar elevation,
                          bool transparent_occluder,
                          DlScalar dpr) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_OP_RECEIVER_H_
