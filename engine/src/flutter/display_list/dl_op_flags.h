// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_OP_FLAGS_H_
#define FLUTTER_DISPLAY_LIST_DL_OP_FLAGS_H_

#include "flutter/display_list/dl_paint.h"
#include "flutter/fml/logging.h"

namespace flutter {

/// The base class for the classes that maintain a list of
/// attributes that might be important for a number of operations
/// including which rendering attributes need to be set before
/// calling a rendering method (all |drawSomething| calls),
/// or for determining which exceptional conditions may need
/// to be accounted for in bounds calculations.
/// This class contains only protected definitions and helper methods
/// for the public classes |DisplayListAttributeFlags| and
/// |DisplayListSpecialGeometryFlags|.
class DisplayListFlags {
 protected:
  // A drawing operation that is not geometric in nature (but which
  // may still apply a MaskFilter - see |kUsesMaskFilter| below).
  static constexpr int kIsNonGeometric = 0;

  // A geometric operation that is defined as a fill operation
  // regardless of what the current paint Style is set to.
  // This flag will automatically assume |kUsesMaskFilter|.
  static constexpr int kIsFilledGeometry = 1 << 0;

  // A geometric operation that is defined as a stroke operation
  // regardless of what the current paint Style is set to.
  // This flag will automatically assume |kUsesMaskFilter|.
  static constexpr int kIsStrokedGeometry = 1 << 1;

  // A geometric operation that may be a stroke or fill operation
  // depending on the current state of the paint Style attribute.
  // This flag will automatically assume |kUsesMaskFilter|.
  static constexpr int kIsDrawnGeometry = 1 << 2;

  static constexpr int kIsAnyGeometryMask =  //
      kIsFilledGeometry |                    //
      kIsStrokedGeometry |                   //
      kIsDrawnGeometry;

  // A primitive that floods the surface (or clip) with no
  // natural bounds, such as |drawColor| or |drawPaint|.
  static constexpr int kFloodsSurface = 1 << 3;

  static constexpr int kMayHaveCaps = 1 << 4;
  static constexpr int kMayHaveJoins = 1 << 5;
  static constexpr int kButtCapIsSquare = 1 << 6;

  // A geometric operation which has a path that might have
  // end caps that are not rectilinear which means that square
  // end caps might project further than half the stroke width
  // from the geometry bounds.
  // A rectilinear path such as |drawRect| will not have
  // diagonal end caps. |drawLine| might have diagonal end
  // caps depending on the angle of the line, and more likely
  // |drawPath| will often have such end caps.
  static constexpr int kMayHaveDiagonalCaps = 1 << 7;

  // A geometric operation which has joined vertices that are
  // not guaranteed to be smooth (angles of incoming and outgoing)
  // segments at some joins may not have the same angle) or
  // rectilinear (squares have right angles at the corners, but
  // those corners will never extend past the bounding box of
  // the geometry pre-transform).
  // |drawRect|, |drawOval| and |drawRRect| all have well
  // behaved joins, but |drawPath| might have joins that cause
  // mitered extensions outside the pre-transformed bounding box.
  static constexpr int kMayHaveAcuteJoins = 1 << 8;

  static constexpr int kAnySpecialGeometryMask =         //
      kMayHaveCaps | kMayHaveJoins | kButtCapIsSquare |  //
      kMayHaveDiagonalCaps | kMayHaveAcuteJoins;

  // clang-format off
  static constexpr int kUsesAntiAlias       = 1 << 10;
  static constexpr int kUsesAlpha           = 1 << 11;
  static constexpr int kUsesColor           = 1 << 12;
  static constexpr int kUsesBlend           = 1 << 13;
  static constexpr int kUsesShader          = 1 << 14;
  static constexpr int kUsesColorFilter     = 1 << 15;
  static constexpr int kUsesMaskFilter      = 1 << 16;
  static constexpr int kUsesImageFilter     = 1 << 17;

  // Some ops have an optional paint argument. If the version
  // stored in the DisplayList ignores the paint, but there
  // is an option to render the same op with a paint then
  // both of the following flags are set to indicate that
  // a default paint object can be constructed when rendering
  // the op to carry information imposed from outside the
  // DisplayList (for example, the opacity override).
  static constexpr int kIgnoresPaint        = 1 << 30;
  // clang-format on

  static constexpr int kAnyAttributeMask =  //
      kUsesAntiAlias | kUsesAlpha | kUsesColor | kUsesBlend | kUsesShader |
      kUsesColorFilter | kUsesMaskFilter | kUsesImageFilter;
};

class DisplayListFlagsBase : protected DisplayListFlags {
 protected:
  explicit constexpr DisplayListFlagsBase(int flags) : flags_(flags) {}

  const int flags_;

  constexpr bool has_any(int qFlags) const { return (flags_ & qFlags) != 0; }
  constexpr bool has_all(int qFlags) const {
    return (flags_ & qFlags) == qFlags;
  }
  constexpr bool has_none(int qFlags) const { return (flags_ & qFlags) == 0; }
};

/// An attribute class for advertising specific properties of
/// a geometric attribute that can affect the computation of
/// the bounds of the primitive.
class DisplayListSpecialGeometryFlags : DisplayListFlagsBase {
 public:
  /// The geometry may have segments that end without closing the path.
  constexpr bool may_have_end_caps() const { return has_any(kMayHaveCaps); }

  /// The geometry may have segments connect non-continuously.
  constexpr bool may_have_joins() const { return has_any(kMayHaveJoins); }

  /// Mainly for drawPoints(PointMode) where Butt caps are rendered as squares.
  constexpr bool butt_cap_becomes_square() const {
    return has_any(kButtCapIsSquare);
  }

  /// The geometry may have segments that end on a diagonal
  /// such that their end caps extend further than the default
  /// |strokeWidth * 0.5| margin around the geometry.
  constexpr bool may_have_diagonal_caps() const {
    return has_any(kMayHaveDiagonalCaps);
  }

  /// The geometry may have segments that meet at vertices at
  /// an acute angle such that the miter joins will extend
  /// further than the default |strokeWidth * 0.5| margin around
  /// the geometry.
  constexpr bool may_have_acute_joins() const {
    return has_any(kMayHaveAcuteJoins);
  }

 private:
  explicit constexpr DisplayListSpecialGeometryFlags(int flags)
      : DisplayListFlagsBase(flags) {
    FML_DCHECK((flags & kAnySpecialGeometryMask) == flags);
  }

  const DisplayListSpecialGeometryFlags with(int extra) const {
    return extra == 0 ? *this : DisplayListSpecialGeometryFlags(flags_ | extra);
  }

  friend class DisplayListAttributeFlags;
};

class DisplayListAttributeFlags : DisplayListFlagsBase {
 public:
  const DisplayListSpecialGeometryFlags GeometryFlags(bool is_stroked) const {
    return special_flags_;
  }

  constexpr bool ignores_paint() const { return has_any(kIgnoresPaint); }

  constexpr bool applies_anti_alias() const { return has_any(kUsesAntiAlias); }
  constexpr bool applies_color() const { return has_any(kUsesColor); }
  constexpr bool applies_alpha() const { return has_any(kUsesAlpha); }
  constexpr bool applies_alpha_or_color() const {
    return has_any(kUsesAlpha | kUsesColor);
  }

  /// The primitive dynamically determines whether it is a stroke or fill
  /// operation (or both) based on the setting of the |Style| attribute.
  constexpr bool applies_style() const { return has_any(kIsDrawnGeometry); }
  /// The primitive can use any of the stroke attributes, such as
  /// StrokeWidth, StrokeMiter, StrokeCap, or StrokeJoin. This
  /// method will return if the primitive is defined as one that
  /// strokes its geometry (such as |drawLine|) or if it is defined
  /// as one that honors the Style attribute. If the Style attribute
  /// is known then a more accurate answer can be returned from
  /// the |is_stroked| method by supplying the actual setting of
  /// the style.
  // bool applies_stroke_attributes() const { return is_stroked(); }

  constexpr bool applies_shader() const { return has_any(kUsesShader); }
  /// The primitive honors the current DlColorFilter, including
  /// the related attribute InvertColors
  constexpr bool applies_color_filter() const {
    return has_any(kUsesColorFilter);
  }
  /// The primitive honors the DlBlendMode
  constexpr bool applies_blend() const { return has_any(kUsesBlend); }
  /// The primitive honors the DlMaskFilter whether set using the
  /// filter object or using the convenience method |setMaskBlurFilter|
  constexpr bool applies_mask_filter() const {
    return has_any(kUsesMaskFilter);
  }
  constexpr bool applies_image_filter() const {
    return has_any(kUsesImageFilter);
  }

  constexpr bool is_geometric() const { return has_any(kIsAnyGeometryMask); }
  constexpr bool always_stroked() const { return has_any(kIsStrokedGeometry); }
  constexpr bool is_stroked(DlDrawStyle style = DlDrawStyle::kStroke) const {
    return (has_any(kIsStrokedGeometry) ||
            (style != DlDrawStyle::kFill && has_any(kIsDrawnGeometry)));
  }

  constexpr bool is_flood() const { return has_any(kFloodsSurface); }

  constexpr bool operator==(DisplayListAttributeFlags const& other) const {
    return flags_ == other.flags_;
  }

 private:
  explicit constexpr DisplayListAttributeFlags(int flags)
      : DisplayListFlagsBase(flags),
        special_flags_(flags & kAnySpecialGeometryMask) {
    FML_DCHECK((flags & kIsAnyGeometryMask) == kIsNonGeometric ||
               (flags & kIsAnyGeometryMask) == kIsFilledGeometry ||
               (flags & kIsAnyGeometryMask) == kIsStrokedGeometry ||
               (flags & kIsAnyGeometryMask) == kIsDrawnGeometry);
    FML_DCHECK(((flags & kAnyAttributeMask) == 0) !=
               ((flags & kIgnoresPaint) == 0));
    FML_DCHECK((flags & kIsAnyGeometryMask) != 0 ||
               (flags & kAnySpecialGeometryMask) == 0);
  }

  constexpr DisplayListAttributeFlags operator+(int extra) const {
    return extra == 0 ? *this : DisplayListAttributeFlags(flags_ | extra);
  }

  constexpr DisplayListAttributeFlags operator-(int remove) const {
    FML_DCHECK(has_all(remove));
    return DisplayListAttributeFlags(flags_ & ~remove);
  }

  const DisplayListSpecialGeometryFlags special_flags_;

  friend class DisplayListOpFlags;
};

class DisplayListOpFlags : DisplayListFlags {
 private:
  // Flags common to all primitives that apply colors
  static constexpr int kBasePaintFlags = (kUsesColor |        //
                                          kUsesAlpha |        //
                                          kUsesBlend |        //
                                          kUsesShader |       //
                                          kUsesColorFilter |  //
                                          kUsesImageFilter);

  // Flags common to all primitives that stroke or fill
  static constexpr int kBaseStrokeOrFillFlags = (kIsDrawnGeometry |  //
                                                 kUsesAntiAlias |    //
                                                 kUsesMaskFilter);

  // Flags common to primitives that stroke geometry
  static constexpr int kBaseStrokeFlags = (kIsStrokedGeometry |  //
                                           kUsesAntiAlias |      //
                                           kUsesMaskFilter);

  // Flags common to primitives that render an image with paint attributes
  static constexpr int kBaseImageFlags = (kIsNonGeometric |   //
                                          kUsesAlpha |        //
                                          kUsesBlend |        //
                                          kUsesColorFilter |  //
                                          kUsesImageFilter);

 public:
  static constexpr DisplayListAttributeFlags kSaveLayerFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kSaveLayerWithPaintFlags{
      kIsNonGeometric |   //
      kUsesAlpha |        //
      kUsesBlend |        //
      kUsesColorFilter |  //
      kUsesImageFilter    //
  };
  static constexpr DisplayListAttributeFlags kDrawColorFlags{
      kFloodsSurface |  //
      kIgnoresPaint     //
  };
  static constexpr DisplayListAttributeFlags kDrawPaintFlags{
      kBasePaintFlags |  //
      kFloodsSurface     //
  };
  // Special case flags for horizonal and vertical lines
  static constexpr DisplayListAttributeFlags kDrawHVLineFlags{
      kBasePaintFlags |   //
      kBaseStrokeFlags |  //
      kMayHaveCaps        //
  };
  static constexpr DisplayListAttributeFlags kDrawLineFlags{
      kDrawHVLineFlags        //
      + kMayHaveDiagonalCaps  //
  };
  static constexpr DisplayListAttributeFlags kDrawRectFlags{
      kBasePaintFlags |         //
      kBaseStrokeOrFillFlags |  //
      kMayHaveJoins             //
  };
  static constexpr DisplayListAttributeFlags kDrawOvalFlags{
      kBasePaintFlags |       //
      kBaseStrokeOrFillFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawCircleFlags{
      kBasePaintFlags |       //
      kBaseStrokeOrFillFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawRRectFlags{
      kBasePaintFlags |       //
      kBaseStrokeOrFillFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawDRRectFlags{
      kBasePaintFlags |       //
      kBaseStrokeOrFillFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawRSuperellipseFlags{
      kBasePaintFlags |       //
      kBaseStrokeOrFillFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawPathFlags{
      kBasePaintFlags |         //
      kBaseStrokeOrFillFlags |  //
      kMayHaveCaps |            //
      kMayHaveDiagonalCaps |    //
      kMayHaveJoins |           //
      kMayHaveAcuteJoins        //
  };
  static constexpr DisplayListAttributeFlags kDrawArcNoCenterFlags{
      kBasePaintFlags |         //
      kBaseStrokeOrFillFlags |  //
      kMayHaveCaps |            //
      kMayHaveDiagonalCaps      //
  };
  static constexpr DisplayListAttributeFlags kDrawArcWithCenterFlags{
      kBasePaintFlags |         //
      kBaseStrokeOrFillFlags |  //
      kMayHaveJoins |           //
      kMayHaveAcuteJoins        //
  };
  static constexpr DisplayListAttributeFlags kDrawPointsAsPointsFlags{
      kBasePaintFlags |   //
      kBaseStrokeFlags |  //
      kMayHaveCaps |      //
      kButtCapIsSquare    //
  };
  static constexpr DisplayListAttributeFlags kDrawPointsAsLinesFlags{
      kBasePaintFlags |     //
      kBaseStrokeFlags |    //
      kMayHaveCaps |        //
      kMayHaveDiagonalCaps  //
  };
  // Polygon mode just draws (count-1) separate lines, no joins
  static constexpr DisplayListAttributeFlags kDrawPointsAsPolygonFlags{
      kBasePaintFlags |     //
      kBaseStrokeFlags |    //
      kMayHaveCaps |        //
      kMayHaveDiagonalCaps  //
  };
  static constexpr DisplayListAttributeFlags kDrawVerticesFlags{
      kIsNonGeometric |   //
      kUsesAlpha |        //
      kUsesShader |       //
      kUsesBlend |        //
      kUsesColorFilter |  //
      kUsesImageFilter    //
  };
  static constexpr DisplayListAttributeFlags kDrawImageFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kDrawImageWithPaintFlags{
      kBaseImageFlags |  //
      kUsesAntiAlias |   //
      kUsesMaskFilter    //
  };
  static constexpr DisplayListAttributeFlags kDrawImageRectFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kDrawImageRectWithPaintFlags{
      kBaseImageFlags |  //
      kUsesAntiAlias |   //
      kUsesMaskFilter    //
  };
  static constexpr DisplayListAttributeFlags kDrawImageNineFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kDrawImageNineWithPaintFlags{
      kBaseImageFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawAtlasFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kDrawAtlasWithPaintFlags{
      kBaseImageFlags  //
  };
  static constexpr DisplayListAttributeFlags kDrawDisplayListFlags{
      kIgnoresPaint  //
  };
  static constexpr DisplayListAttributeFlags kDrawTextBlobFlags{
      DisplayListAttributeFlags(kBasePaintFlags |         //
                                kBaseStrokeOrFillFlags |  //
                                kMayHaveJoins)            //
      - kUsesAntiAlias                                    //
  };
  static constexpr DisplayListAttributeFlags kDrawShadowFlags{
      kIgnoresPaint  //
  };
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_OP_FLAGS_H_
