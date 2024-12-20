// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_DISPLAY_LIST_TESTING_H_
#define FLUTTER_TESTING_DISPLAY_LIST_TESTING_H_

#include <ostream>

#include "display_list/effects/dl_image_filter.h"
#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_op_receiver.h"

namespace flutter::testing {

[[nodiscard]] bool DisplayListsEQ_Verbose(const DisplayList* a,
                                          const DisplayList* b);
[[nodiscard]] bool inline DisplayListsEQ_Verbose(const DisplayList& a,
                                                 const DisplayList& b) {
  return DisplayListsEQ_Verbose(&a, &b);
}
[[nodiscard]] bool inline DisplayListsEQ_Verbose(
    const sk_sp<const DisplayList>& a,
    const sk_sp<const DisplayList>& b) {
  return DisplayListsEQ_Verbose(a.get(), b.get());
}
[[nodiscard]] bool DisplayListsNE_Verbose(const DisplayList* a,
                                          const DisplayList* b);
[[nodiscard]] bool inline DisplayListsNE_Verbose(const DisplayList& a,
                                                 const DisplayList& b) {
  return DisplayListsNE_Verbose(&a, &b);
}
[[nodiscard]] bool inline DisplayListsNE_Verbose(
    const sk_sp<const DisplayList>& a,
    const sk_sp<const DisplayList>& b) {
  return DisplayListsNE_Verbose(a.get(), b.get());
}

}  // namespace flutter::testing

namespace std {

extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DisplayList& display_list);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlPaint& paint);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlBlendMode& mode);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlCanvas::ClipOp& op);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlCanvas::PointMode& op);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlCanvas::SrcRectConstraint& op);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlStrokeCap& cap);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlStrokeJoin& join);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlDrawStyle& style);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlBlurStyle& style);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlFilterMode& mode);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlColor& color);
extern std::ostream& operator<<(std::ostream& os,
                                flutter::DlImageSampling sampling);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlVertexMode& mode);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlTileMode& mode);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlImage* image);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::SaveLayerOptions& image);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DisplayListOpType& type);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DisplayListOpCategory& category);
extern std::ostream& operator<<(std::ostream& os, const flutter::DlPath& path);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlImageFilter& type);
extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DlColorFilter& type);

}  // namespace std

namespace flutter::testing {

class DisplayListStreamDispatcher final : public DlOpReceiver {
 public:
  explicit DisplayListStreamDispatcher(std::ostream& os,
                                       int cur_indent = 2,
                                       int indent = 2)
      : os_(os), cur_indent_(cur_indent), indent_(indent) {}

  void setAntiAlias(bool aa) override;
  void setDrawStyle(DlDrawStyle style) override;
  void setColor(DlColor color) override;
  void setStrokeWidth(DlScalar width) override;
  void setStrokeMiter(DlScalar limit) override;
  void setStrokeCap(DlStrokeCap cap) override;
  void setStrokeJoin(DlStrokeJoin join) override;
  void setColorSource(const DlColorSource* source) override;
  void setColorFilter(const DlColorFilter* filter) override;
  void setInvertColors(bool invert) override;
  void setBlendMode(DlBlendMode mode) override;
  void setMaskFilter(const DlMaskFilter* filter) override;
  void setImageFilter(const DlImageFilter* filter) override;

  void save() override;
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;
  void restore() override;

  void translate(DlScalar tx, DlScalar ty) override;
  void scale(DlScalar sx, DlScalar sy) override;
  void rotate(DlScalar degrees) override;
  void skew(DlScalar sx, DlScalar sy) override;
  // clang-format off
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override;
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override;
  // clang-format on
  void transformReset() override;

  void clipRect(const DlRect& rect, ClipOp clip_op, bool is_aa) override;
  void clipOval(const DlRect& bounds, ClipOp clip_op, bool is_aa) override;
  void clipRoundRect(const DlRoundRect& rrect,
                     ClipOp clip_op,
                     bool is_aa) override;
  void clipPath(const DlPath& path, ClipOp clip_op, bool is_aa) override;

  void drawColor(DlColor color, DlBlendMode mode) override;
  void drawPaint() override;
  void drawLine(const DlPoint& p0, const DlPoint& p1) override;
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override;
  void drawRect(const DlRect& rect) override;
  void drawOval(const DlRect& bounds) override;
  void drawCircle(const DlPoint& center, DlScalar radius) override;
  void drawRoundRect(const DlRoundRect& rrect) override;
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override;
  void drawPath(const DlPath& path) override;
  void drawArc(const DlRect& oval_bounds,
               DlScalar start_degrees,
               DlScalar sweep_degrees,
               bool use_center) override;
  void drawPoints(PointMode mode,
                  uint32_t count,
                  const DlPoint points[]) override;
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override;
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override;
  void drawImageRect(const sk_sp<DlImage> image,
                     const DlRect& src,
                     const DlRect& dst,
                     DlImageSampling sampling,
                     bool render_with_attributes,
                     SrcRectConstraint constraint) override;
  void drawImageNine(const sk_sp<DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     bool render_with_attributes) override;
  void drawAtlas(const sk_sp<DlImage> atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cull_rect,
                 bool render_with_attributes) override;
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity) override;
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override;
  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override;
  void drawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override;

  void out(const DlColorFilter& filter);
  void out(const DlColorFilter* filter);
  void out(const DlImageFilter& filter);
  void out(const DlImageFilter* filter);

 private:
  std::ostream& os_;
  int cur_indent_;
  int indent_;

  void indent() { indent(indent_); }
  void outdent() { outdent(indent_); }
  void indent(int spaces) { cur_indent_ += spaces; }
  void outdent(int spaces) { cur_indent_ -= spaces; }

  template <class T>
  std::ostream& out_array(std::string name, int count, const T array[]);

  std::ostream& startl();
};

class DisplayListGeneralReceiver : public DlOpReceiver {
 public:
  DisplayListGeneralReceiver() {
    type_counts_.fill(0u);
    category_counts_.fill(0u);
  }

  void setAntiAlias(bool aa) override {
    RecordByType(DisplayListOpType::kSetAntiAlias);
  }
  void setInvertColors(bool invert) override {
    RecordByType(DisplayListOpType::kSetInvertColors);
  }
  void setStrokeCap(DlStrokeCap cap) override {
    RecordByType(DisplayListOpType::kSetStrokeCap);
  }
  void setStrokeJoin(DlStrokeJoin join) override {
    RecordByType(DisplayListOpType::kSetStrokeJoin);
  }
  void setDrawStyle(DlDrawStyle style) override {
    RecordByType(DisplayListOpType::kSetStyle);
  }
  void setStrokeWidth(float width) override {
    RecordByType(DisplayListOpType::kSetStrokeWidth);
  }
  void setStrokeMiter(float limit) override {
    RecordByType(DisplayListOpType::kSetStrokeMiter);
  }
  void setColor(DlColor color) override {
    RecordByType(DisplayListOpType::kSetColor);
  }
  void setBlendMode(DlBlendMode mode) override {
    RecordByType(DisplayListOpType::kSetBlendMode);
  }
  void setColorSource(const DlColorSource* source) override {
    if (source) {
      switch (source->type()) {
        case DlColorSourceType::kImage:
          RecordByType(DisplayListOpType::kSetImageColorSource);
          break;
        case DlColorSourceType::kRuntimeEffect:
          RecordByType(DisplayListOpType::kSetRuntimeEffectColorSource);
          break;
        case DlColorSourceType::kLinearGradient:
        case DlColorSourceType::kRadialGradient:
        case DlColorSourceType::kConicalGradient:
        case DlColorSourceType::kSweepGradient:
          RecordByType(DisplayListOpType::kSetPodColorSource);
          break;
      }
    } else {
      RecordByType(DisplayListOpType::kClearColorSource);
    }
  }
  void setImageFilter(const DlImageFilter* filter) override {
    if (filter) {
      switch (filter->type()) {
        case DlImageFilterType::kBlur:
        case DlImageFilterType::kDilate:
        case DlImageFilterType::kErode:
        case DlImageFilterType::kMatrix:
          RecordByType(DisplayListOpType::kSetPodImageFilter);
          break;
        case DlImageFilterType::kCompose:
        case DlImageFilterType::kLocalMatrix:
        case DlImageFilterType::kColorFilter:
        case DlImageFilterType::kRuntimeEffect:
          RecordByType(DisplayListOpType::kSetSharedImageFilter);
          break;
      }
    } else {
      RecordByType(DisplayListOpType::kClearImageFilter);
    }
  }
  void setColorFilter(const DlColorFilter* filter) override {
    if (filter) {
      switch (filter->type()) {
        case DlColorFilterType::kBlend:
        case DlColorFilterType::kMatrix:
        case DlColorFilterType::kLinearToSrgbGamma:
        case DlColorFilterType::kSrgbToLinearGamma:
          RecordByType(DisplayListOpType::kSetPodColorFilter);
          break;
      }
    } else {
      RecordByType(DisplayListOpType::kClearColorFilter);
    }
  }
  void setMaskFilter(const DlMaskFilter* filter) override {
    if (filter) {
      switch (filter->type()) {
        case DlMaskFilterType::kBlur:
          RecordByType(DisplayListOpType::kSetPodMaskFilter);
          break;
      }
    } else {
      RecordByType(DisplayListOpType::kClearMaskFilter);
    }
  }

  void translate(DlScalar tx, DlScalar ty) override {
    RecordByType(DisplayListOpType::kTranslate);
  }
  void scale(DlScalar sx, DlScalar sy) override {
    RecordByType(DisplayListOpType::kScale);
  }
  void rotate(DlScalar degrees) override {
    RecordByType(DisplayListOpType::kRotate);
  }
  void skew(DlScalar sx, DlScalar sy) override {
    RecordByType(DisplayListOpType::kSkew);
  }
  // clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override {
    RecordByType(DisplayListOpType::kTransform2DAffine);
  }
  // full 4x4 transform in row major order
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override {
    RecordByType(DisplayListOpType::kTransformFullPerspective);
  }
  // clang-format on
  void transformReset() override {
    RecordByType(DisplayListOpType::kTransformReset);
  }

  void clipRect(const DlRect& rect,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    switch (clip_op) {
      case DlCanvas::ClipOp::kIntersect:
        RecordByType(DisplayListOpType::kClipIntersectRect);
        break;
      case DlCanvas::ClipOp::kDifference:
        RecordByType(DisplayListOpType::kClipDifferenceRect);
        break;
    }
  }
  void clipOval(const DlRect& bounds,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    switch (clip_op) {
      case DlCanvas::ClipOp::kIntersect:
        RecordByType(DisplayListOpType::kClipIntersectOval);
        break;
      case DlCanvas::ClipOp::kDifference:
        RecordByType(DisplayListOpType::kClipDifferenceOval);
        break;
    }
  }
  void clipRoundRect(const DlRoundRect& rrect,
                     DlCanvas::ClipOp clip_op,
                     bool is_aa) override {
    switch (clip_op) {
      case DlCanvas::ClipOp::kIntersect:
        RecordByType(DisplayListOpType::kClipIntersectRoundRect);
        break;
      case DlCanvas::ClipOp::kDifference:
        RecordByType(DisplayListOpType::kClipDifferenceRoundRect);
        break;
    }
  }
  void clipPath(const DlPath& path,
                DlCanvas::ClipOp clip_op,
                bool is_aa) override {
    switch (clip_op) {
      case DlCanvas::ClipOp::kIntersect:
        RecordByType(DisplayListOpType::kClipIntersectPath);
        break;
      case DlCanvas::ClipOp::kDifference:
        RecordByType(DisplayListOpType::kClipDifferencePath);
        break;
    }
  }

  void save() override { RecordByType(DisplayListOpType::kSave); }
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    if (backdrop) {
      RecordByType(DisplayListOpType::kSaveLayerBackdrop);
    } else {
      RecordByType(DisplayListOpType::kSaveLayer);
    }
  }
  void restore() override { RecordByType(DisplayListOpType::kRestore); }

  void drawColor(DlColor color, DlBlendMode mode) override {
    RecordByType(DisplayListOpType::kDrawColor);
  }
  void drawPaint() override { RecordByType(DisplayListOpType::kDrawPaint); }
  void drawLine(const DlPoint& p0, const DlPoint& p1) override {
    RecordByType(DisplayListOpType::kDrawLine);
  }
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override {
    RecordByType(DisplayListOpType::kDrawDashedLine);
  }
  void drawRect(const DlRect& rect) override {
    RecordByType(DisplayListOpType::kDrawRect);
  }
  void drawOval(const DlRect& bounds) override {
    RecordByType(DisplayListOpType::kDrawOval);
  }
  void drawCircle(const DlPoint& center, DlScalar radius) override {
    RecordByType(DisplayListOpType::kDrawCircle);
  }
  void drawRoundRect(const DlRoundRect& rrect) override {
    RecordByType(DisplayListOpType::kDrawRoundRect);
  }
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override {
    RecordByType(DisplayListOpType::kDrawDiffRoundRect);
  }
  void drawPath(const DlPath& path) override {
    RecordByType(DisplayListOpType::kDrawPath);
  }
  void drawArc(const DlRect& oval_bounds,
               DlScalar start_degrees,
               DlScalar sweep_degrees,
               bool use_center) override {
    RecordByType(DisplayListOpType::kDrawArc);
  }
  void drawPoints(DlCanvas::PointMode mode,
                  uint32_t count,
                  const DlPoint points[]) override {
    switch (mode) {
      case DlCanvas::PointMode::kPoints:
        RecordByType(DisplayListOpType::kDrawPoints);
        break;
      case DlCanvas::PointMode::kLines:
        RecordByType(DisplayListOpType::kDrawLines);
        break;
      case DlCanvas::PointMode::kPolygon:
        RecordByType(DisplayListOpType::kDrawPolygon);
        break;
    }
  }
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override {
    RecordByType(DisplayListOpType::kDrawVertices);
  }
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override {
    if (render_with_attributes) {
      RecordByType(DisplayListOpType::kDrawImageWithAttr);
    } else {
      RecordByType(DisplayListOpType::kDrawImage);
    }
  }
  void drawImageRect(const sk_sp<DlImage> image,
                     const DlRect& src,
                     const DlRect& dst,
                     DlImageSampling sampling,
                     bool render_with_attributes,
                     SrcRectConstraint constraint) override {
    RecordByType(DisplayListOpType::kDrawImageRect);
  }
  void drawImageNine(const sk_sp<DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     bool render_with_attributes) override {
    if (render_with_attributes) {
      RecordByType(DisplayListOpType::kDrawImageNineWithAttr);
    } else {
      RecordByType(DisplayListOpType::kDrawImageNine);
    }
  }
  void drawAtlas(const sk_sp<DlImage> atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cull_rect,
                 bool render_with_attributes) override {
    if (cull_rect) {
      RecordByType(DisplayListOpType::kDrawAtlasCulled);
    } else {
      RecordByType(DisplayListOpType::kDrawAtlas);
    }
  }
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity) override {
    RecordByType(DisplayListOpType::kDrawDisplayList);
  }
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override {
    RecordByType(DisplayListOpType::kDrawTextBlob);
  }
  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override {
    RecordByType(DisplayListOpType::kDrawTextFrame);
  }
  void drawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override {
    if (transparent_occluder) {
      RecordByType(DisplayListOpType::kDrawShadowTransparentOccluder);
    } else {
      RecordByType(DisplayListOpType::kDrawShadow);
    }
  }

  uint32_t GetOpsReceived() { return op_count_; }
  uint32_t GetOpsReceived(DisplayListOpCategory category) {
    return category_counts_[static_cast<int>(category)];
  }
  uint32_t GetOpsReceived(DisplayListOpType type) {
    return type_counts_[static_cast<int>(type)];
  }

 protected:
  virtual void RecordByType(DisplayListOpType type) {
    type_counts_[static_cast<int>(type)]++;
    RecordByCategory(DisplayList::GetOpCategory(type));
  }

  virtual void RecordByCategory(DisplayListOpCategory category) {
    category_counts_[static_cast<int>(category)]++;
    switch (category) {
      case DisplayListOpCategory::kAttribute:
        RecordAttribute();
        break;
      case DisplayListOpCategory::kTransform:
        RecordTransform();
        break;
      case DisplayListOpCategory::kClip:
        RecordClip();
        break;
      case DisplayListOpCategory::kSave:
        RecordSave();
        break;
      case DisplayListOpCategory::kSaveLayer:
        RecordSaveLayer();
        break;
      case DisplayListOpCategory::kRestore:
        RecordRestore();
        break;
      case DisplayListOpCategory::kRendering:
        RecordRendering();
        break;
      case DisplayListOpCategory::kSubDisplayList:
        RecordSubDisplayList();
        break;
      case DisplayListOpCategory::kInvalidCategory:
        RecordInvalid();
        break;
    }
  }

  virtual void RecordAttribute() { RecordOp(); }
  virtual void RecordTransform() { RecordOp(); }
  virtual void RecordClip() { RecordOp(); }
  virtual void RecordSave() { RecordOp(); }
  virtual void RecordSaveLayer() { RecordOp(); }
  virtual void RecordRestore() { RecordOp(); }
  virtual void RecordRendering() { RecordOp(); }
  virtual void RecordSubDisplayList() { RecordOp(); }
  virtual void RecordInvalid() { RecordOp(); }

  virtual void RecordOp() { op_count_++; }

  static constexpr size_t kTypeCount =
      static_cast<size_t>(DisplayListOpType::kMaxOp) + 1;
  static constexpr size_t kCategoryCount =
      static_cast<size_t>(DisplayListOpCategory::kMaxCategory) + 1;

  std::array<uint32_t, kTypeCount> type_counts_;
  std::array<uint32_t, kCategoryCount> category_counts_;
  uint32_t op_count_ = 0u;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_DISPLAY_LIST_TESTING_H_
