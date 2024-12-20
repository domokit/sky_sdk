// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_OP_RECORDS_H_
#define FLUTTER_DISPLAY_LIST_DL_OP_RECORDS_H_

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "flutter/fml/macros.h"

#include "flutter/impeller/geometry/path.h"
#include "flutter/impeller/typographer/text_frame.h"
#include "third_party/skia/include/core/SkRSXform.h"

namespace flutter {

// Most Ops can be bulk compared using memcmp because they contain
// only numeric values or constructs that are constructed from numeric
// values.
//
// Some contain sk_sp<> references which can also be bulk compared
// to see if they are pointing to the same reference. (Note that
// two sk_sp<> that refer to the same object are themselves ==.)
//
// Only a DLOp that wants to do a deep compare needs to override the
// DLOp::equals() method and return a value of kEqual or kNotEqual.
enum class DisplayListCompare {
  // The Op is deferring comparisons to a bulk memcmp performed lazily
  // across all bulk-comparable ops.
  kUseBulkCompare,

  // The Op provided a specific equals method that spotted a difference
  kNotEqual,

  // The Op provided a specific equals method that saw no differences
  kEqual,
};

// "DLOpPackLabel" is just a label for the pack pragma so it can be popped
// later.
#pragma pack(push, DLOpPackLabel, 8)

// Assuming a 64-bit platform (most of our platforms at this time?)
// the following comments are a "worst case" assessment of how well
// these structures pack into memory. They may be packed more tightly
// on some of the 32-bit platforms that we see in older phones.
//
// Struct allocation in the DL memory is aligned to a void* boundary
// which means that the minimum (aligned) struct size will be 8 bytes.
// The DLOp base uses 4 bytes so each Op-specific struct gets 4 bytes
// of data for "free" and works best when it packs well into an 8-byte
// aligned size.
struct DLOp {
  static constexpr uint32_t kDepthInc = 0;
  static constexpr uint32_t kRenderOpInc = 0;

  explicit DLOp(DisplayListOpType type) : type(type) {}

  const DisplayListOpType type;

  DisplayListCompare equals(const DLOp* other) const {
    return DisplayListCompare::kUseBulkCompare;
  }
};

// 4 byte header + 4 byte payload packs into minimum 8 bytes
#define DEFINE_SET_BOOL_OP(name)                                      \
  struct Set##name##Op final : DLOp {                                 \
    static constexpr auto kType = DisplayListOpType::kSet##name;      \
                                                                      \
    explicit Set##name##Op(bool value) : DLOp(kType), value(value) {} \
                                                                      \
    const bool value;                                                 \
                                                                      \
    void dispatch(DlOpReceiver& receiver) const {                     \
      receiver.set##name(value);                                      \
    }                                                                 \
  };
DEFINE_SET_BOOL_OP(AntiAlias)
DEFINE_SET_BOOL_OP(InvertColors)
#undef DEFINE_SET_BOOL_OP

// 4 byte header + 4 byte payload packs into minimum 8 bytes
#define DEFINE_SET_ENUM_OP(name)                                       \
  struct SetStroke##name##Op final : DLOp {                            \
    static constexpr auto kType = DisplayListOpType::kSetStroke##name; \
                                                                       \
    explicit SetStroke##name##Op(DlStroke##name value)                 \
        : DLOp(kType), value(value) {}                                 \
                                                                       \
    const DlStroke##name value;                                        \
                                                                       \
    void dispatch(DlOpReceiver& receiver) const {                      \
      receiver.setStroke##name(value);                                 \
    }                                                                  \
  };
DEFINE_SET_ENUM_OP(Cap)
DEFINE_SET_ENUM_OP(Join)
#undef DEFINE_SET_ENUM_OP

// 4 byte header + 4 byte payload packs into minimum 8 bytes
struct SetStyleOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetStyle;

  explicit SetStyleOp(DlDrawStyle style) : DLOp(kType), style(style) {}

  const DlDrawStyle style;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.setDrawStyle(style);
  }
};
// 4 byte header + 4 byte payload packs into minimum 8 bytes
struct SetStrokeWidthOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetStrokeWidth;

  explicit SetStrokeWidthOp(float width) : DLOp(kType), width(width) {}

  const float width;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.setStrokeWidth(width);
  }
};
// 4 byte header + 4 byte payload packs into minimum 8 bytes
struct SetStrokeMiterOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetStrokeMiter;

  explicit SetStrokeMiterOp(float limit) : DLOp(kType), limit(limit) {}

  const float limit;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.setStrokeMiter(limit);
  }
};

// 4 byte header + 20 byte payload packs into minimum 24 bytes
struct SetColorOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetColor;

  explicit SetColorOp(DlColor color) : DLOp(kType), color(color) {}

  const DlColor color;

  void dispatch(DlOpReceiver& receiver) const { receiver.setColor(color); }
};
// 4 byte header + 4 byte payload packs into minimum 8 bytes
struct SetBlendModeOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetBlendMode;

  explicit SetBlendModeOp(DlBlendMode mode) : DLOp(kType), mode(mode) {}

  const DlBlendMode mode;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.setBlendMode(mode);
  }
};

// Clear: 4 byte header + unused 4 byte payload uses 8 bytes
//        (4 bytes unused)
// Set: 4 byte header + unused 4 byte struct padding + Dl<name>
//      instance copied to the memory following the record
//      yields a size and efficiency that has somewhere between
//      4 and 8 bytes unused
#define DEFINE_SET_CLEAR_DLATTR_OP(name, field)                             \
  struct Clear##name##Op final : DLOp {                                     \
    static constexpr auto kType = DisplayListOpType::kClear##name;          \
                                                                            \
    Clear##name##Op() : DLOp(kType) {}                                      \
                                                                            \
    void dispatch(DlOpReceiver& receiver) const {                           \
      receiver.set##name(nullptr);                                          \
    }                                                                       \
  };                                                                        \
  struct SetPod##name##Op final : DLOp {                                    \
    static constexpr auto kType = DisplayListOpType::kSetPod##name;         \
                                                                            \
    SetPod##name##Op() : DLOp(kType) {}                                     \
                                                                            \
    void dispatch(DlOpReceiver& receiver) const {                           \
      const Dl##name* filter = reinterpret_cast<const Dl##name*>(this + 1); \
      receiver.set##name(filter);                                           \
    }                                                                       \
  };
DEFINE_SET_CLEAR_DLATTR_OP(ColorFilter, filter)
DEFINE_SET_CLEAR_DLATTR_OP(ImageFilter, filter)
DEFINE_SET_CLEAR_DLATTR_OP(MaskFilter, filter)
DEFINE_SET_CLEAR_DLATTR_OP(ColorSource, source)
#undef DEFINE_SET_CLEAR_DLATTR_OP

// 4 byte header + 96 bytes for the embedded DlImageColorSource
// uses 104 total bytes (4 bytes unused)
struct SetImageColorSourceOp : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetImageColorSource;

  explicit SetImageColorSourceOp(const DlImageColorSource* source)
      : DLOp(kType),
        source(source->image(),
               source->horizontal_tile_mode(),
               source->vertical_tile_mode(),
               source->sampling(),
               source->matrix_ptr()) {}

  const DlImageColorSource source;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.setColorSource(&source);
  }
};

// 56 bytes: 4 byte header, 4 byte padding, 8 for vtable, 8 * 2 for sk_sps, 24
// for the std::vector.
struct SetRuntimeEffectColorSourceOp : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetRuntimeEffectColorSource;

  explicit SetRuntimeEffectColorSourceOp(
      const DlRuntimeEffectColorSource* source)
      : DLOp(kType),
        source(source->runtime_effect(),
               source->samplers(),
               source->uniform_data()) {}

  const DlRuntimeEffectColorSource source;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.setColorSource(&source);
  }

  DisplayListCompare equals(const SetRuntimeEffectColorSourceOp* other) const {
    return (source == other->source) ? DisplayListCompare::kEqual
                                     : DisplayListCompare::kNotEqual;
  }
};

// 4 byte header + 16 byte payload uses 24 total bytes (4 bytes unused)
struct SetSharedImageFilterOp : DLOp {
  static constexpr auto kType = DisplayListOpType::kSetSharedImageFilter;

  explicit SetSharedImageFilterOp(const DlImageFilter* filter)
      : DLOp(kType), filter(filter->shared()) {}

  const std::shared_ptr<DlImageFilter> filter;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.setImageFilter(filter.get());
  }

  DisplayListCompare equals(const SetSharedImageFilterOp* other) const {
    return Equals(filter, other->filter) ? DisplayListCompare::kEqual
                                         : DisplayListCompare::kNotEqual;
  }
};

// The base struct for all save() and saveLayer() ops
// 4 byte header + 12 byte payload packs exactly into 16 bytes
struct SaveOpBase : DLOp {
  static constexpr uint32_t kDepthInc = 0;
  static constexpr uint32_t kRenderOpInc = 1;

  explicit SaveOpBase(DisplayListOpType type)
      : DLOp(type), options(), restore_index(0), total_content_depth(0) {}

  SaveOpBase(DisplayListOpType type, const SaveLayerOptions& options)
      : DLOp(type),
        options(options),
        restore_index(0),
        total_content_depth(0) {}

  // options parameter is only used by saveLayer operations, but since
  // it packs neatly into the empty space created by laying out the rest
  // of the data here, it can be stored for free and defaulted to 0 for
  // save operations.
  SaveLayerOptions options;
  DlIndex restore_index;
  uint32_t total_content_depth;
};
// 16 byte SaveOpBase with no additional data (options is unsed here)
struct SaveOp final : SaveOpBase {
  static constexpr auto kType = DisplayListOpType::kSave;

  SaveOp() : SaveOpBase(kType) {}

  void dispatch(DlOpReceiver& receiver) const {
    receiver.save(total_content_depth);
  }
};
// The base struct for all saveLayer() ops
// 16 byte SaveOpBase + 20 byte payload packs into 36 bytes
struct SaveLayerOpBase : SaveOpBase {
  SaveLayerOpBase(DisplayListOpType type,
                  const SaveLayerOptions& options,
                  const DlRect& rect)
      : SaveOpBase(type, options), rect(rect) {}

  DlRect rect;
  DlBlendMode max_blend_mode = DlBlendMode::kClear;
};
// 36 byte SaveLayerOpBase with no additional data packs into 40 bytes
// of buffer storage with 4 bytes unused.
struct SaveLayerOp final : SaveLayerOpBase {
  static constexpr auto kType = DisplayListOpType::kSaveLayer;

  SaveLayerOp(const SaveLayerOptions& options, const DlRect& rect)
      : SaveLayerOpBase(kType, options, rect) {}

  void dispatch(DlOpReceiver& receiver) const {
    receiver.saveLayer(rect, options, total_content_depth, max_blend_mode);
  }
};
// 36 byte SaveLayerOpBase + 4 bytes for alignment + 16 byte payload packs
// into minimum 56 bytes
struct SaveLayerBackdropOp final : SaveLayerOpBase {
  static constexpr auto kType = DisplayListOpType::kSaveLayerBackdrop;

  SaveLayerBackdropOp(const SaveLayerOptions& options,
                      const DlRect& rect,
                      const DlImageFilter* backdrop,
                      std::optional<int64_t> backdrop_id)
      : SaveLayerOpBase(kType, options, rect),
        backdrop(backdrop->shared()),
        backdrop_id_(backdrop_id) {}

  const std::shared_ptr<DlImageFilter> backdrop;
  std::optional<int64_t> backdrop_id_;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.saveLayer(rect, options, total_content_depth, max_blend_mode,
                       backdrop.get(), backdrop_id_);
  }

  DisplayListCompare equals(const SaveLayerBackdropOp* other) const {
    return (options == other->options && rect == other->rect &&
            Equals(backdrop, other->backdrop) &&
            backdrop_id_ == other->backdrop_id_)
               ? DisplayListCompare::kEqual
               : DisplayListCompare::kNotEqual;
  }
};
// 4 byte header + no payload uses minimum 8 bytes (4 bytes unused)
struct RestoreOp final : DLOp {
  static constexpr auto kType = DisplayListOpType::kRestore;
  static constexpr uint32_t kDepthInc = 0;
  static constexpr uint32_t kRenderOpInc = 1;

  RestoreOp() : DLOp(kType) {}

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.restore();
  }
};

struct TransformClipOpBase : DLOp {
  static constexpr uint32_t kDepthInc = 0;
  static constexpr uint32_t kRenderOpInc = 1;

  explicit TransformClipOpBase(DisplayListOpType type) : DLOp(type) {}
};
// 4 byte header + 8 byte payload uses 12 bytes but is rounded up to 16 bytes
// (4 bytes unused)
struct TranslateOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kTranslate;

  TranslateOp(DlScalar tx, DlScalar ty)
      : TransformClipOpBase(kType), tx(tx), ty(ty) {}

  const DlScalar tx;
  const DlScalar ty;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.translate(tx, ty);
  }
};
// 4 byte header + 8 byte payload uses 12 bytes but is rounded up to 16 bytes
// (4 bytes unused)
struct ScaleOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kScale;

  ScaleOp(DlScalar sx, DlScalar sy)
      : TransformClipOpBase(kType), sx(sx), sy(sy) {}

  const DlScalar sx;
  const DlScalar sy;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.scale(sx, sy);
  }
};
// 4 byte header + 4 byte payload packs into minimum 8 bytes
struct RotateOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kRotate;

  explicit RotateOp(DlScalar degrees)
      : TransformClipOpBase(kType), degrees(degrees) {}

  const DlScalar degrees;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.rotate(degrees);
  }
};
// 4 byte header + 8 byte payload uses 12 bytes but is rounded up to 16 bytes
// (4 bytes unused)
struct SkewOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kSkew;

  SkewOp(DlScalar sx, DlScalar sy)
      : TransformClipOpBase(kType), sx(sx), sy(sy) {}

  const DlScalar sx;
  const DlScalar sy;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.skew(sx, sy);
  }
};
// 4 byte header + 24 byte payload uses 28 bytes but is rounded up to 32 bytes
// (4 bytes unused)
struct Transform2DAffineOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kTransform2DAffine;

  // clang-format off
  Transform2DAffineOp(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                      DlScalar myx, DlScalar myy, DlScalar myt)
      : TransformClipOpBase(kType),
        mxx(mxx), mxy(mxy), mxt(mxt),
        myx(myx), myy(myy), myt(myt) {}
  // clang-format on

  const DlScalar mxx, mxy, mxt;
  const DlScalar myx, myy, myt;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.transform2DAffine(mxx, mxy, mxt,  //
                               myx, myy, myt);
  }
};
// 4 byte header + 64 byte payload uses 68 bytes which is rounded up to 72 bytes
// (4 bytes unused)
struct TransformFullPerspectiveOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kTransformFullPerspective;

  // clang-format off
  TransformFullPerspectiveOp(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt)
      : TransformClipOpBase(kType),
        mxx(mxx), mxy(mxy), mxz(mxz), mxt(mxt),
        myx(myx), myy(myy), myz(myz), myt(myt),
        mzx(mzx), mzy(mzy), mzz(mzz), mzt(mzt),
        mwx(mwx), mwy(mwy), mwz(mwz), mwt(mwt) {}
  // clang-format on

  const DlScalar mxx, mxy, mxz, mxt;
  const DlScalar myx, myy, myz, myt;
  const DlScalar mzx, mzy, mzz, mzt;
  const DlScalar mwx, mwy, mwz, mwt;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.transformFullPerspective(mxx, mxy, mxz, mxt,  //
                                      myx, myy, myz, myt,  //
                                      mzx, mzy, mzz, mzt,  //
                                      mwx, mwy, mwz, mwt);
  }
};

// 4 byte header with no payload.
struct TransformResetOp final : TransformClipOpBase {
  static constexpr auto kType = DisplayListOpType::kTransformReset;

  TransformResetOp() : TransformClipOpBase(kType) {}

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.transformReset();
  }
};

// 4 byte header + 4 byte common payload packs into minimum 8 bytes
// DlRect is 16 more bytes, which packs efficiently into 24 bytes total
// DlRoundRect is 48 more bytes, which rounds up to 48 bytes
//         which packs into 56 bytes total
// CacheablePath is 128 more bytes, which packs efficiently into 136 bytes total
//
// We could pack the clip_op and the bool both into the free 4 bytes after
// the header, but the Windows compiler keeps wanting to expand that
// packing into more bytes than needed (even when they are declared as
// packed bit fields!)
#define DEFINE_CLIP_SHAPE_OP(shapename, shapetype, clipop)                     \
  struct Clip##clipop##shapename##Op final : TransformClipOpBase {             \
    static constexpr auto kType = DisplayListOpType::kClip##clipop##shapename; \
                                                                               \
    Clip##clipop##shapename##Op(shapetype shape, bool is_aa)                   \
        : TransformClipOpBase(kType), is_aa(is_aa), shape(shape) {}            \
                                                                               \
    const bool is_aa;                                                          \
    const shapetype shape;                                                     \
                                                                               \
    void dispatch(DlOpReceiver& receiver) const {                              \
      receiver.clip##shapename(shape, DlCanvas::ClipOp::k##clipop, is_aa);     \
    }                                                                          \
  };
DEFINE_CLIP_SHAPE_OP(Rect, DlRect, Intersect)
DEFINE_CLIP_SHAPE_OP(Oval, DlRect, Intersect)
DEFINE_CLIP_SHAPE_OP(RoundRect, DlRoundRect, Intersect)
DEFINE_CLIP_SHAPE_OP(Rect, DlRect, Difference)
DEFINE_CLIP_SHAPE_OP(Oval, DlRect, Difference)
DEFINE_CLIP_SHAPE_OP(RoundRect, DlRoundRect, Difference)
#undef DEFINE_CLIP_SHAPE_OP

// 4 byte header + 20 byte payload packs evenly into 24 bytes
#define DEFINE_CLIP_PATH_OP(clipop)                                       \
  struct Clip##clipop##PathOp final : TransformClipOpBase {               \
    static constexpr auto kType = DisplayListOpType::kClip##clipop##Path; \
                                                                          \
    Clip##clipop##PathOp(const DlPath& path, bool is_aa)                  \
        : TransformClipOpBase(kType), is_aa(is_aa), path(path) {}         \
                                                                          \
    const bool is_aa;                                                     \
    const DlPath path;                                                    \
                                                                          \
    void dispatch(DlOpReceiver& receiver) const {                         \
      receiver.clipPath(path, DlCanvas::ClipOp::k##clipop, is_aa);        \
    }                                                                     \
                                                                          \
    DisplayListCompare equals(const Clip##clipop##PathOp* other) const {  \
      return is_aa == other->is_aa && path == other->path                 \
                 ? DisplayListCompare::kEqual                             \
                 : DisplayListCompare::kNotEqual;                         \
    }                                                                     \
  };
DEFINE_CLIP_PATH_OP(Intersect)
DEFINE_CLIP_PATH_OP(Difference)
#undef DEFINE_CLIP_PATH_OP

struct DrawOpBase : DLOp {
  static constexpr uint32_t kDepthInc = 1;
  static constexpr uint32_t kRenderOpInc = 1;

  explicit DrawOpBase(DisplayListOpType type) : DLOp(type) {}
};

// 4 byte header + no payload uses minimum 8 bytes (4 bytes unused)
struct DrawPaintOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawPaint;

  DrawPaintOp() : DrawOpBase(kType) {}

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.drawPaint();
  }
};
// 4 byte header + 8 byte payload uses 12 bytes but is rounded up to 16 bytes
// (4 bytes unused)
struct DrawColorOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawColor;

  DrawColorOp(DlColor color, DlBlendMode mode)
      : DrawOpBase(kType), color(color), mode(mode) {}

  const DlColor color;
  const DlBlendMode mode;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawColor(color, mode);
  }
};

// The common data is a 4 byte header with an unused 4 bytes
// DlRect is 16 more bytes, using 20 bytes which rounds up to 24 bytes total
//        (4 bytes unused)
// SkOval is same as DlRect
// DlRoundRect is 48 more bytes, using 52 bytes which rounds up to 56 bytes
//        total (4 bytes unused)
#define DEFINE_DRAW_1ARG_OP(op_name, arg_type, arg_name)             \
  struct Draw##op_name##Op final : DrawOpBase {                      \
    static constexpr auto kType = DisplayListOpType::kDraw##op_name; \
                                                                     \
    explicit Draw##op_name##Op(arg_type arg_name)                    \
        : DrawOpBase(kType), arg_name(arg_name) {}                   \
                                                                     \
    const arg_type arg_name;                                         \
                                                                     \
    void dispatch(DlOpReceiver& receiver) const {                    \
      receiver.draw##op_name(arg_name);                              \
    }                                                                \
  };
DEFINE_DRAW_1ARG_OP(Rect, DlRect, rect)
DEFINE_DRAW_1ARG_OP(Oval, DlRect, oval)
DEFINE_DRAW_1ARG_OP(RoundRect, DlRoundRect, rrect)
#undef DEFINE_DRAW_1ARG_OP

// 4 byte header + 16 byte payload uses 20 bytes but is rounded
// up to 24 bytes (4 bytes unused)
struct DrawPathOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawPath;

  explicit DrawPathOp(const DlPath& path) : DrawOpBase(kType), path(path) {}

  const DlPath path;

  void dispatch(DlOpReceiver& receiver) const {  //
    receiver.drawPath(path);
  }

  DisplayListCompare equals(const DrawPathOp* other) const {
    return path == other->path ? DisplayListCompare::kEqual
                               : DisplayListCompare::kNotEqual;
  }
};

// The common data is a 4 byte header with an unused 4 bytes
// 2 x DlPoint is 16 more bytes, using 20 bytes rounding up to 24 bytes total
//             (4 bytes unused)
// DlPoint + DlScalar is 12 more bytes, packing efficiently into 16 bytes total
// 2 x DlRoundRect is 96 more bytes, using 100 and rounding up to 104 bytes
//             total (4 bytes unused)
#define DEFINE_DRAW_2ARG_OP(op_name, type1, name1, type2, name2)     \
  struct Draw##op_name##Op final : DrawOpBase {                      \
    static constexpr auto kType = DisplayListOpType::kDraw##op_name; \
                                                                     \
    Draw##op_name##Op(type1 name1, type2 name2)                      \
        : DrawOpBase(kType), name1(name1), name2(name2) {}           \
                                                                     \
    const type1 name1;                                               \
    const type2 name2;                                               \
                                                                     \
    void dispatch(DlOpReceiver& receiver) const {                    \
      receiver.draw##op_name(name1, name2);                          \
    }                                                                \
  };
DEFINE_DRAW_2ARG_OP(Line, DlPoint, p0, DlPoint, p1)
DEFINE_DRAW_2ARG_OP(Circle, DlPoint, center, DlScalar, radius)
DEFINE_DRAW_2ARG_OP(DiffRoundRect, DlRoundRect, outer, DlRoundRect, inner)
#undef DEFINE_DRAW_2ARG_OP

// 4 byte header + 24 byte payload packs into 32 bytes (4 bytes unused)
struct DrawDashedLineOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawDashedLine;

  DrawDashedLineOp(const DlPoint& p0,
                   const DlPoint& p1,
                   DlScalar on_length,
                   DlScalar off_length)
      : DrawOpBase(kType),
        p0(p0),
        p1(p1),
        on_length(on_length),
        off_length(off_length) {}

  const DlPoint p0;
  const DlPoint p1;
  const DlScalar on_length;
  const DlScalar off_length;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawDashedLine(p0, p1, on_length, off_length);
  }
};

// 4 byte header + 28 byte payload packs efficiently into 32 bytes
struct DrawArcOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawArc;

  DrawArcOp(DlRect bounds, DlScalar start, DlScalar sweep, bool center)
      : DrawOpBase(kType),
        bounds(bounds),
        start(start),
        sweep(sweep),
        center(center) {}

  const DlRect bounds;
  const DlScalar start;
  const DlScalar sweep;
  const bool center;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawArc(bounds, start, sweep, center);
  }
};

// 4 byte header + 4 byte fixed payload packs efficiently into 8 bytes
// But then there is a list of points following the structure which
// is guaranteed to be a multiple of 8 bytes (DlPoint is 8 bytes)
// so this op will always pack efficiently
// The point type is packed into 3 different OpTypes to avoid expanding
// the fixed payload beyond the 8 bytes
#define DEFINE_DRAW_POINTS_OP(name, mode)                              \
  struct Draw##name##Op final : DrawOpBase {                           \
    static constexpr auto kType = DisplayListOpType::kDraw##name;      \
                                                                       \
    explicit Draw##name##Op(uint32_t count)                            \
        : DrawOpBase(kType), count(count) {}                           \
                                                                       \
    const uint32_t count;                                              \
                                                                       \
    void dispatch(DlOpReceiver& receiver) const {                      \
      const DlPoint* pts = reinterpret_cast<const DlPoint*>(this + 1); \
      receiver.drawPoints(DlCanvas::PointMode::mode, count, pts);      \
    }                                                                  \
  };
DEFINE_DRAW_POINTS_OP(Points, kPoints);
DEFINE_DRAW_POINTS_OP(Lines, kLines);
DEFINE_DRAW_POINTS_OP(Polygon, kPolygon);
#undef DEFINE_DRAW_POINTS_OP

// 4 byte header + 20 byte payload packs efficiently into 24 bytes
struct DrawVerticesOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawVertices;

  explicit DrawVerticesOp(const std::shared_ptr<DlVertices>& vertices,
                          DlBlendMode mode)
      : DrawOpBase(kType), mode(mode), vertices(vertices) {}

  const DlBlendMode mode;
  const std::shared_ptr<DlVertices> vertices;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawVertices(vertices, mode);
  }
};

// 4 byte header + 40 byte payload uses 44 bytes but is rounded up to 48 bytes
// (4 bytes unused)
#define DEFINE_DRAW_IMAGE_OP(name, with_attributes)                   \
  struct name##Op final : DrawOpBase {                                \
    static constexpr auto kType = DisplayListOpType::k##name;         \
                                                                      \
    name##Op(const sk_sp<DlImage>& image,                             \
             const DlPoint& point,                                    \
             DlImageSampling sampling)                                \
        : DrawOpBase(kType),                                          \
          point(point),                                               \
          sampling(sampling),                                         \
          image(std::move(image)) {}                                  \
                                                                      \
    const DlPoint point;                                              \
    const DlImageSampling sampling;                                   \
    const sk_sp<DlImage> image;                                       \
                                                                      \
    void dispatch(DlOpReceiver& receiver) const {                     \
      receiver.drawImage(image, point, sampling, with_attributes);    \
    }                                                                 \
                                                                      \
    DisplayListCompare equals(const name##Op* other) const {          \
      return (point == other->point && sampling == other->sampling && \
              image->Equals(other->image))                            \
                 ? DisplayListCompare::kEqual                         \
                 : DisplayListCompare::kNotEqual;                     \
    }                                                                 \
  };
DEFINE_DRAW_IMAGE_OP(DrawImage, false)
DEFINE_DRAW_IMAGE_OP(DrawImageWithAttr, true)
#undef DEFINE_DRAW_IMAGE_OP

// 4 byte header + 72 byte payload uses 76 bytes but is rounded up to 80 bytes
// (4 bytes unused)
struct DrawImageRectOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawImageRect;

  DrawImageRectOp(const sk_sp<DlImage>& image,
                  const DlRect& src,
                  const DlRect& dst,
                  DlImageSampling sampling,
                  bool render_with_attributes,
                  DlCanvas::SrcRectConstraint constraint)
      : DrawOpBase(kType),
        src(src),
        dst(dst),
        sampling(sampling),
        render_with_attributes(render_with_attributes),
        constraint(constraint),
        image(image) {}

  const DlRect src;
  const DlRect dst;
  const DlImageSampling sampling;
  const bool render_with_attributes;
  const DlCanvas::SrcRectConstraint constraint;
  const sk_sp<DlImage> image;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawImageRect(image, src, dst, sampling, render_with_attributes,
                           constraint);
  }

  DisplayListCompare equals(const DrawImageRectOp* other) const {
    return (src == other->src && dst == other->dst &&
            sampling == other->sampling &&
            render_with_attributes == other->render_with_attributes &&
            constraint == other->constraint && image->Equals(other->image))
               ? DisplayListCompare::kEqual
               : DisplayListCompare::kNotEqual;
  }
};

// 4 byte header + 44 byte payload packs efficiently into 48 bytes
#define DEFINE_DRAW_IMAGE_NINE_OP(name, render_with_attributes)   \
  struct name##Op final : DrawOpBase {                            \
    static constexpr auto kType = DisplayListOpType::k##name;     \
    static constexpr uint32_t kDepthInc = 9;                      \
                                                                  \
    name##Op(const sk_sp<DlImage>& image,                         \
             const DlIRect& center,                               \
             const DlRect& dst,                                   \
             DlFilterMode mode)                                   \
        : DrawOpBase(kType),                                      \
          center(center),                                         \
          dst(dst),                                               \
          mode(mode),                                             \
          image(std::move(image)) {}                              \
                                                                  \
    const DlIRect center;                                         \
    const DlRect dst;                                             \
    const DlFilterMode mode;                                      \
    const sk_sp<DlImage> image;                                   \
                                                                  \
    void dispatch(DlOpReceiver& receiver) const {                 \
      receiver.drawImageNine(image, center, dst, mode,            \
                             render_with_attributes);             \
    }                                                             \
                                                                  \
    DisplayListCompare equals(const name##Op* other) const {      \
      return (center == other->center && dst == other->dst &&     \
              mode == other->mode && image->Equals(other->image)) \
                 ? DisplayListCompare::kEqual                     \
                 : DisplayListCompare::kNotEqual;                 \
    }                                                             \
  };
DEFINE_DRAW_IMAGE_NINE_OP(DrawImageNine, false)
DEFINE_DRAW_IMAGE_NINE_OP(DrawImageNineWithAttr, true)
#undef DEFINE_DRAW_IMAGE_NINE_OP

// 4 byte header + 40 byte payload uses 44 bytes but is rounded up to 48 bytes
// (4 bytes unused)
// Each of these is then followed by a number of lists.
// SkRSXform list is a multiple of 16 bytes so it is always packed well
// DlRect list is also a multiple of 16 bytes so it also packs well
// DlColor list only packs well if the count is even, otherwise there
// can be 4 unusued bytes at the end.
struct DrawAtlasBaseOp : DrawOpBase {
  DrawAtlasBaseOp(DisplayListOpType type,
                  const sk_sp<DlImage>& atlas,
                  int count,
                  DlBlendMode mode,
                  DlImageSampling sampling,
                  bool has_colors,
                  bool render_with_attributes)
      : DrawOpBase(type),
        count(count),
        mode_index(static_cast<uint16_t>(mode)),
        has_colors(has_colors),
        render_with_attributes(render_with_attributes),
        sampling(sampling),
        atlas(atlas) {}

  const int count;
  const uint16_t mode_index;
  const uint8_t has_colors;
  const uint8_t render_with_attributes;
  const DlImageSampling sampling;
  const sk_sp<DlImage> atlas;

  bool equals(const DrawAtlasBaseOp* other,
              const void* pod_this,
              const void* pod_other) const {
    bool ret = (count == other->count && mode_index == other->mode_index &&
                has_colors == other->has_colors &&
                render_with_attributes == other->render_with_attributes &&
                sampling == other->sampling && atlas->Equals(other->atlas));
    if (ret) {
      size_t bytes = count * (sizeof(SkRSXform) + sizeof(DlRect));
      if (has_colors) {
        bytes += count * sizeof(DlColor);
      }
      ret = (memcmp(pod_this, pod_other, bytes) == 0);
    }
    return ret;
  }
};

// Packs into 48 bytes as per DrawAtlasBaseOp
// with array data following the struct also as per DrawAtlasBaseOp
struct DrawAtlasOp final : DrawAtlasBaseOp {
  static constexpr auto kType = DisplayListOpType::kDrawAtlas;

  DrawAtlasOp(const sk_sp<DlImage>& atlas,
              int count,
              DlBlendMode mode,
              DlImageSampling sampling,
              bool has_colors,
              bool render_with_attributes)
      : DrawAtlasBaseOp(kType,
                        atlas,
                        count,
                        mode,
                        sampling,
                        has_colors,
                        render_with_attributes) {}

  void dispatch(DlOpReceiver& receiver) const {
    const SkRSXform* xform = reinterpret_cast<const SkRSXform*>(this + 1);
    const DlRect* tex = reinterpret_cast<const DlRect*>(xform + count);
    const DlColor* colors =
        has_colors ? reinterpret_cast<const DlColor*>(tex + count) : nullptr;
    const DlBlendMode mode = static_cast<DlBlendMode>(mode_index);
    receiver.drawAtlas(atlas, xform, tex, colors, count, mode, sampling,
                       nullptr, render_with_attributes);
  }

  DisplayListCompare equals(const DrawAtlasOp* other) const {
    const void* pod_this = reinterpret_cast<const void*>(this + 1);
    const void* pod_other = reinterpret_cast<const void*>(other + 1);
    return (DrawAtlasBaseOp::equals(other, pod_this, pod_other))
               ? DisplayListCompare::kEqual
               : DisplayListCompare::kNotEqual;
  }
};

// Packs into 48 bytes as per DrawAtlasBaseOp plus
// an additional 16 bytes for the cull rect resulting in a total
// of 56 bytes for the Culled drawAtlas.
// Also with array data following the struct as per DrawAtlasBaseOp
struct DrawAtlasCulledOp final : DrawAtlasBaseOp {
  static constexpr auto kType = DisplayListOpType::kDrawAtlasCulled;

  DrawAtlasCulledOp(const sk_sp<DlImage>& atlas,
                    int count,
                    DlBlendMode mode,
                    DlImageSampling sampling,
                    bool has_colors,
                    const DlRect& cull_rect,
                    bool render_with_attributes)
      : DrawAtlasBaseOp(kType,
                        atlas,
                        count,
                        mode,
                        sampling,
                        has_colors,
                        render_with_attributes),
        cull_rect(cull_rect) {}

  const DlRect cull_rect;

  void dispatch(DlOpReceiver& receiver) const {
    const SkRSXform* xform = reinterpret_cast<const SkRSXform*>(this + 1);
    const DlRect* tex = reinterpret_cast<const DlRect*>(xform + count);
    const DlColor* colors =
        has_colors ? reinterpret_cast<const DlColor*>(tex + count) : nullptr;
    const DlBlendMode mode = static_cast<DlBlendMode>(mode_index);
    receiver.drawAtlas(atlas, xform, tex, colors, count, mode, sampling,
                       &cull_rect, render_with_attributes);
  }

  DisplayListCompare equals(const DrawAtlasCulledOp* other) const {
    const void* pod_this = reinterpret_cast<const void*>(this + 1);
    const void* pod_other = reinterpret_cast<const void*>(other + 1);
    return (cull_rect == other->cull_rect &&
            DrawAtlasBaseOp::equals(other, pod_this, pod_other))
               ? DisplayListCompare::kEqual
               : DisplayListCompare::kNotEqual;
  }
};

// 4 byte header + ptr aligned payload uses 12 bytes round up to 16
// (4 bytes unused)
struct DrawDisplayListOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawDisplayList;

  explicit DrawDisplayListOp(const sk_sp<DisplayList>& display_list,
                             DlScalar opacity)
      : DrawOpBase(kType), opacity(opacity), display_list(display_list) {}

  DlScalar opacity;
  const sk_sp<DisplayList> display_list;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawDisplayList(display_list, opacity);
  }

  DisplayListCompare equals(const DrawDisplayListOp* other) const {
    return (opacity == other->opacity &&
            display_list->Equals(other->display_list))
               ? DisplayListCompare::kEqual
               : DisplayListCompare::kNotEqual;
  }
};

// 4 byte header + 8 payload bytes + an aligned pointer take 24 bytes
// (4 unused to align the pointer)
struct DrawTextBlobOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawTextBlob;

  DrawTextBlobOp(const sk_sp<SkTextBlob>& blob, DlScalar x, DlScalar y)
      : DrawOpBase(kType), x(x), y(y), blob(blob) {}

  const DlScalar x;
  const DlScalar y;
  const sk_sp<SkTextBlob> blob;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawTextBlob(blob, x, y);
  }
};

struct DrawTextFrameOp final : DrawOpBase {
  static constexpr auto kType = DisplayListOpType::kDrawTextFrame;

  DrawTextFrameOp(const std::shared_ptr<impeller::TextFrame>& text_frame,
                  DlScalar x,
                  DlScalar y)
      : DrawOpBase(kType), x(x), y(y), text_frame(text_frame) {}

  const DlScalar x;
  const DlScalar y;
  const std::shared_ptr<impeller::TextFrame> text_frame;

  void dispatch(DlOpReceiver& receiver) const {
    receiver.drawTextFrame(text_frame, x, y);
  }
};

// 4 byte header + 44 byte payload packs evenly into 48 bytes
#define DEFINE_DRAW_SHADOW_OP(name, transparent_occluder)                     \
  struct Draw##name##Op final : DrawOpBase {                                  \
    static constexpr auto kType = DisplayListOpType::kDraw##name;             \
                                                                              \
    Draw##name##Op(const DlPath& path,                                        \
                   DlColor color,                                             \
                   DlScalar elevation,                                        \
                   DlScalar dpr)                                              \
        : DrawOpBase(kType),                                                  \
          color(color),                                                       \
          elevation(elevation),                                               \
          dpr(dpr),                                                           \
          path(path) {}                                                       \
                                                                              \
    const DlColor color;                                                      \
    const DlScalar elevation;                                                 \
    const DlScalar dpr;                                                       \
    const DlPath path;                                                        \
                                                                              \
    void dispatch(DlOpReceiver& receiver) const {                             \
      receiver.drawShadow(path, color, elevation, transparent_occluder, dpr); \
    }                                                                         \
                                                                              \
    DisplayListCompare equals(const Draw##name##Op* other) const {            \
      return color == other->color && elevation == other->elevation &&        \
                     dpr == other->dpr && path == other->path                 \
                 ? DisplayListCompare::kEqual                                 \
                 : DisplayListCompare::kNotEqual;                             \
    }                                                                         \
  };
DEFINE_DRAW_SHADOW_OP(Shadow, false)
DEFINE_DRAW_SHADOW_OP(ShadowTransparentOccluder, true)
#undef DEFINE_DRAW_SHADOW_OP

#pragma pack(pop, DLOpPackLabel)

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_OP_RECORDS_H_
