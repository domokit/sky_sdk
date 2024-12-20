// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/layer_state_stack.h"

#include "flutter/display_list/utils/dl_matrix_clip_tracker.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/paint_utils.h"
#include "flutter/flow/raster_cache_util.h"

namespace flutter {

// ==============================================================
// Delegate subclasses
// ==============================================================

// The DummyDelegate class implements most Delegate methods as NOP
// but throws errors if the caller starts executing query methods
// that require an active delegate to be tracking. It is specifically
// designed to be immutable, lightweight, and a singleton so that it
// can be substituted into the delegate slot in a LayerStateStack
// quickly and cheaply when no externally supplied delegates are present.
class DummyDelegate : public LayerStateStack::Delegate {
 public:
  static const std::shared_ptr<DummyDelegate> kInstance;

  void decommission() override {}

  DlRect local_cull_rect() const override {
    error();
    return {};
  }
  DlRect device_cull_rect() const override {
    error();
    return {};
  }
  DlMatrix matrix() const override {
    error();
    return dummy_matrix_;
  }
  bool content_culled(const DlRect& content_bounds) const override {
    error();
    return true;
  }

  void save() override {}
  void saveLayer(const DlRect& bounds,
                 LayerStateStack::RenderingAttributes& attributes,
                 DlBlendMode blend,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {}
  void restore() override {}

  void translate(DlScalar tx, DlScalar ty) override {}
  void transform(const DlMatrix& matrix) override {}
  void integralTransform() override {}

  void clipRect(const DlRect& rect, ClipOp op, bool is_aa) override {}
  void clipRRect(const DlRoundRect& rrect, ClipOp op, bool is_aa) override {}
  void clipPath(const DlPath& path, ClipOp op, bool is_aa) override {}

 private:
  static void error() {
    FML_DCHECK(false) << "LayerStateStack state queried without a delegate";
  }
  const DlMatrix dummy_matrix_;
};
const std::shared_ptr<DummyDelegate> DummyDelegate::kInstance =
    std::make_shared<DummyDelegate>();

class DlCanvasDelegate : public LayerStateStack::Delegate {
 public:
  explicit DlCanvasDelegate(DlCanvas* canvas)
      : canvas_(canvas), initial_save_level_(canvas->GetSaveCount()) {}

  void decommission() override { canvas_->RestoreToCount(initial_save_level_); }

  DlCanvas* canvas() const override { return canvas_; }

  DlRect local_cull_rect() const override {
    return canvas_->GetLocalClipCoverage();
  }
  DlRect device_cull_rect() const override {
    return canvas_->GetDestinationClipCoverage();
  }
  DlMatrix matrix() const override { return canvas_->GetMatrix(); }
  bool content_culled(const DlRect& content_bounds) const override {
    return canvas_->QuickReject(content_bounds);
  }

  void save() override { canvas_->Save(); }
  void saveLayer(const DlRect& bounds,
                 LayerStateStack::RenderingAttributes& attributes,
                 DlBlendMode blend_mode,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    TRACE_EVENT0("flutter", "Canvas::saveLayer");
    DlPaint paint;
    canvas_->SaveLayer(bounds, attributes.fill(paint, blend_mode), backdrop,
                       backdrop_id);
  }
  void restore() override { canvas_->Restore(); }

  void translate(DlScalar tx, DlScalar ty) override {
    canvas_->Translate(tx, ty);
  }
  void transform(const DlMatrix& matrix) override {
    canvas_->Transform(matrix);
  }
  void integralTransform() override {
    DlMatrix integral;
    if (RasterCacheUtil::ComputeIntegralTransCTM(matrix(), &integral)) {
      canvas_->SetTransform(integral);
    }
  }

  void clipRect(const DlRect& rect, ClipOp op, bool is_aa) override {
    canvas_->ClipRect(rect, op, is_aa);
  }
  void clipRRect(const DlRoundRect& rrect, ClipOp op, bool is_aa) override {
    canvas_->ClipRoundRect(rrect, op, is_aa);
  }
  void clipPath(const DlPath& path, ClipOp op, bool is_aa) override {
    canvas_->ClipPath(path, op, is_aa);
  }

 private:
  DlCanvas* canvas_;
  const int initial_save_level_;
};

class PrerollDelegate : public LayerStateStack::Delegate {
 public:
  PrerollDelegate(const DlRect& cull_rect, const DlMatrix& matrix) {
    save_stack_.emplace_back(cull_rect, matrix);
  }

  void decommission() override {}

  DlMatrix matrix() const override { return state().matrix(); }
  DlRect local_cull_rect() const override {
    return state().GetLocalCullCoverage();
  }
  DlRect device_cull_rect() const override {
    return state().GetDeviceCullCoverage();
  }
  bool content_culled(const DlRect& content_bounds) const override {
    return state().content_culled(content_bounds);
  }

  void save() override { save_stack_.emplace_back(state()); }
  void saveLayer(const DlRect& bounds,
                 LayerStateStack::RenderingAttributes& attributes,
                 DlBlendMode blend,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {
    save_stack_.emplace_back(state());
  }
  void restore() override { save_stack_.pop_back(); }

  void translate(DlScalar tx, DlScalar ty) override {
    state().translate(tx, ty);
  }
  void transform(const DlMatrix& matrix) override { state().transform(matrix); }
  void integralTransform() override {
    DlMatrix integral;
    if (RasterCacheUtil::ComputeIntegralTransCTM(state().matrix(), &integral)) {
      state().setTransform(integral);
    }
  }

  void clipRect(const DlRect& rect, ClipOp op, bool is_aa) override {
    state().clipRect(rect, op, is_aa);
  }
  void clipRRect(const DlRoundRect& rrect, ClipOp op, bool is_aa) override {
    state().clipRRect(rrect, op, is_aa);
  }
  void clipPath(const DlPath& path, ClipOp op, bool is_aa) override {
    state().clipPath(path, op, is_aa);
  }

 private:
  DisplayListMatrixClipState& state() { return save_stack_.back(); }
  const DisplayListMatrixClipState& state() const { return save_stack_.back(); }

  std::vector<DisplayListMatrixClipState> save_stack_;
};

// ==============================================================
// StateEntry subclasses
// ==============================================================

class SaveEntry : public LayerStateStack::StateEntry {
 public:
  SaveEntry() = default;

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->save();
  }
  void restore(LayerStateStack* stack) const override {
    stack->delegate_->restore();
  }

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(SaveEntry);
};

class SaveLayerEntry : public LayerStateStack::StateEntry {
 public:
  SaveLayerEntry(const DlRect& bounds,
                 DlBlendMode blend_mode,
                 const LayerStateStack::RenderingAttributes& prev)
      : bounds_(bounds), blend_mode_(blend_mode), old_attributes_(prev) {}

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->saveLayer(bounds_, stack->outstanding_, blend_mode_,
                                nullptr);
    stack->outstanding_ = {};
  }
  void restore(LayerStateStack* stack) const override {
    stack->delegate_->restore();
    stack->outstanding_ = old_attributes_;
  }

 protected:
  const DlRect bounds_;
  const DlBlendMode blend_mode_;
  const LayerStateStack::RenderingAttributes old_attributes_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(SaveLayerEntry);
};

class OpacityEntry : public LayerStateStack::StateEntry {
 public:
  OpacityEntry(const DlRect& bounds,
               SkScalar opacity,
               const LayerStateStack::RenderingAttributes& prev)
      : bounds_(bounds),
        opacity_(opacity),
        old_opacity_(prev.opacity),
        old_bounds_(prev.save_layer_bounds) {}

  void apply(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = bounds_;
    stack->outstanding_.opacity *= opacity_;
  }
  void restore(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = old_bounds_;
    stack->outstanding_.opacity = old_opacity_;
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushOpacity(DlColor::toAlpha(opacity_));
  }

 private:
  const DlRect bounds_;
  const DlScalar opacity_;
  const DlScalar old_opacity_;
  const DlRect old_bounds_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(OpacityEntry);
};

class ImageFilterEntry : public LayerStateStack::StateEntry {
 public:
  ImageFilterEntry(const DlRect& bounds,
                   const std::shared_ptr<DlImageFilter>& filter,
                   const LayerStateStack::RenderingAttributes& prev)
      : bounds_(bounds),
        filter_(filter),
        old_filter_(prev.image_filter),
        old_bounds_(prev.save_layer_bounds) {}
  ~ImageFilterEntry() override = default;

  void apply(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = bounds_;
    stack->outstanding_.image_filter = filter_;
  }
  void restore(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = old_bounds_;
    stack->outstanding_.image_filter = old_filter_;
  }

  // There is no ImageFilter mutator currently
  // void update_mutators(MutatorsStack* mutators_stack) const override;

 private:
  const DlRect bounds_;
  const std::shared_ptr<DlImageFilter> filter_;
  const std::shared_ptr<DlImageFilter> old_filter_;
  const DlRect old_bounds_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(ImageFilterEntry);
};

class ColorFilterEntry : public LayerStateStack::StateEntry {
 public:
  ColorFilterEntry(const DlRect& bounds,
                   const std::shared_ptr<const DlColorFilter>& filter,
                   const LayerStateStack::RenderingAttributes& prev)
      : bounds_(bounds),
        filter_(filter),
        old_filter_(prev.color_filter),
        old_bounds_(prev.save_layer_bounds) {}
  ~ColorFilterEntry() override = default;

  void apply(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = bounds_;
    stack->outstanding_.color_filter = filter_;
  }
  void restore(LayerStateStack* stack) const override {
    stack->outstanding_.save_layer_bounds = old_bounds_;
    stack->outstanding_.color_filter = old_filter_;
  }

  // There is no ColorFilter mutator currently
  // void update_mutators(MutatorsStack* mutators_stack) const override;

 private:
  const DlRect bounds_;
  const std::shared_ptr<const DlColorFilter> filter_;
  const std::shared_ptr<const DlColorFilter> old_filter_;
  const DlRect old_bounds_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(ColorFilterEntry);
};

class BackdropFilterEntry : public SaveLayerEntry {
 public:
  BackdropFilterEntry(const DlRect& bounds,
                      const std::shared_ptr<DlImageFilter>& filter,
                      DlBlendMode blend_mode,
                      std::optional<int64_t> backdrop_id,
                      const LayerStateStack::RenderingAttributes& prev)
      : SaveLayerEntry(bounds, blend_mode, prev),
        filter_(filter),
        backdrop_id_(backdrop_id) {}
  ~BackdropFilterEntry() override = default;

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->saveLayer(bounds_, stack->outstanding_, blend_mode_,
                                filter_.get(), backdrop_id_);
    stack->outstanding_ = {};
  }

  void reapply(LayerStateStack* stack) const override {
    // On the reapply for subsequent overlay layers, we do not
    // want to reapply the backdrop filter, but we do need to
    // do a saveLayer to encapsulate the contents and match the
    // restore that will be forthcoming. Note that this is not
    // perfect if the BlendMode is not associative as we will be
    // compositing multiple parts of the content in batches.
    // Luckily the most common SrcOver is associative.
    SaveLayerEntry::apply(stack);
  }

 private:
  const std::shared_ptr<DlImageFilter> filter_;
  std::optional<int64_t> backdrop_id_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(BackdropFilterEntry);
};

class TranslateEntry : public LayerStateStack::StateEntry {
 public:
  TranslateEntry(SkScalar tx, SkScalar ty) : tx_(tx), ty_(ty) {}

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->translate(tx_, ty_);
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushTransform(SkMatrix::Translate(tx_, ty_));
  }

 private:
  const SkScalar tx_;
  const SkScalar ty_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TranslateEntry);
};

class TransformMatrixEntry : public LayerStateStack::StateEntry {
 public:
  explicit TransformMatrixEntry(const DlMatrix& matrix) : matrix_(matrix) {}

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->transform(matrix_);
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushTransform(ToSkMatrix(matrix_));
  }

 private:
  const DlMatrix matrix_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TransformMatrixEntry);
};

class IntegralTransformEntry : public LayerStateStack::StateEntry {
 public:
  IntegralTransformEntry() = default;

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->integralTransform();
  }

 private:
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(IntegralTransformEntry);
};

class ClipRectEntry : public LayerStateStack::StateEntry {
 public:
  ClipRectEntry(const DlRect& clip_rect, bool is_aa)
      : clip_rect_(clip_rect), is_aa_(is_aa) {}

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->clipRect(clip_rect_, DlCanvas::ClipOp::kIntersect,
                               is_aa_);
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushClipRect(ToSkRect(clip_rect_));
  }

 private:
  const DlRect clip_rect_;
  const bool is_aa_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(ClipRectEntry);
};

class ClipRRectEntry : public LayerStateStack::StateEntry {
 public:
  ClipRRectEntry(const DlRoundRect& clip_rrect, bool is_aa)
      : clip_rrect_(clip_rrect), is_aa_(is_aa) {}

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->clipRRect(clip_rrect_, DlCanvas::ClipOp::kIntersect,
                                is_aa_);
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushClipRRect(ToSkRRect(clip_rrect_));
  }

 private:
  const DlRoundRect clip_rrect_;
  const bool is_aa_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(ClipRRectEntry);
};

class ClipPathEntry : public LayerStateStack::StateEntry {
 public:
  ClipPathEntry(const DlPath& clip_path, bool is_aa)
      : clip_path_(clip_path), is_aa_(is_aa) {}
  ~ClipPathEntry() override = default;

  void apply(LayerStateStack* stack) const override {
    stack->delegate_->clipPath(clip_path_, DlCanvas::ClipOp::kIntersect,
                               is_aa_);
  }
  void update_mutators(MutatorsStack* mutators_stack) const override {
    mutators_stack->PushClipPath(clip_path_.GetSkPath());
  }

 private:
  const DlPath clip_path_;
  const bool is_aa_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(ClipPathEntry);
};

// ==============================================================
// RenderingAttributes methods
// ==============================================================

DlPaint* LayerStateStack::RenderingAttributes::fill(DlPaint& paint,
                                                    DlBlendMode mode) const {
  DlPaint* ret = nullptr;
  if (opacity < SK_Scalar1) {
    paint.setOpacity(std::max(opacity, 0.0f));
    ret = &paint;
  } else {
    paint.setOpacity(SK_Scalar1);
  }
  paint.setColorFilter(color_filter);
  if (color_filter) {
    ret = &paint;
  }
  paint.setImageFilter(image_filter);
  if (image_filter) {
    ret = &paint;
  }
  paint.setBlendMode(mode);
  if (mode != DlBlendMode::kSrcOver) {
    ret = &paint;
  }
  return ret;
}

// ==============================================================
// MutatorContext methods
// ==============================================================

using MutatorContext = LayerStateStack::MutatorContext;

void MutatorContext::saveLayer(const DlRect& bounds) {
  layer_state_stack_->save_layer(bounds);
}

void MutatorContext::applyOpacity(const DlRect& bounds, DlScalar opacity) {
  if (opacity < SK_Scalar1) {
    layer_state_stack_->push_opacity(bounds, opacity);
  }
}

void MutatorContext::applyImageFilter(
    const DlRect& bounds,
    const std::shared_ptr<DlImageFilter>& filter) {
  if (filter) {
    layer_state_stack_->push_image_filter(bounds, filter);
  }
}

void MutatorContext::applyColorFilter(
    const DlRect& bounds,
    const std::shared_ptr<const DlColorFilter>& filter) {
  if (filter) {
    layer_state_stack_->push_color_filter(bounds, filter);
  }
}

void MutatorContext::applyBackdropFilter(
    const DlRect& bounds,
    const std::shared_ptr<DlImageFilter>& filter,
    DlBlendMode blend_mode,
    std::optional<int64_t> backdrop_id) {
  layer_state_stack_->push_backdrop(bounds, filter, blend_mode, backdrop_id);
}

void MutatorContext::translate(SkScalar tx, SkScalar ty) {
  if (!(tx == 0 && ty == 0)) {
    layer_state_stack_->maybe_save_layer_for_transform(save_needed_);
    save_needed_ = false;
    layer_state_stack_->push_translate(tx, ty);
  }
}

void MutatorContext::transform(const DlMatrix& matrix) {
  if (matrix.IsTranslationOnly()) {
    translate(matrix.m[12], matrix.m[13]);
  } else if (!matrix.IsIdentity()) {
    layer_state_stack_->maybe_save_layer_for_transform(save_needed_);
    save_needed_ = false;
    layer_state_stack_->push_transform(matrix);
  }
}

void MutatorContext::integralTransform() {
  layer_state_stack_->maybe_save_layer_for_transform(save_needed_);
  save_needed_ = false;
  layer_state_stack_->push_integral_transform();
}

void MutatorContext::clipRect(const DlRect& rect, bool is_aa) {
  layer_state_stack_->maybe_save_layer_for_clip(save_needed_);
  save_needed_ = false;
  layer_state_stack_->push_clip_rect(rect, is_aa);
}

void MutatorContext::clipRRect(const DlRoundRect& rrect, bool is_aa) {
  layer_state_stack_->maybe_save_layer_for_clip(save_needed_);
  save_needed_ = false;
  layer_state_stack_->push_clip_rrect(rrect, is_aa);
}

void MutatorContext::clipPath(const DlPath& path, bool is_aa) {
  layer_state_stack_->maybe_save_layer_for_clip(save_needed_);
  save_needed_ = false;
  layer_state_stack_->push_clip_path(path, is_aa);
}

// ==============================================================
// LayerStateStack methods
// ==============================================================

LayerStateStack::LayerStateStack() : delegate_(DummyDelegate::kInstance) {}

void LayerStateStack::clear_delegate() {
  delegate_->decommission();
  delegate_ = DummyDelegate::kInstance;
}

void LayerStateStack::set_delegate(DlCanvas* canvas) {
  if (delegate_) {
    if (canvas == delegate_->canvas()) {
      return;
    }
    clear_delegate();
  }
  if (canvas) {
    delegate_ = std::make_shared<DlCanvasDelegate>(canvas);
    reapply_all();
  }
}

void LayerStateStack::set_preroll_delegate(const DlRect& cull_rect) {
  set_preroll_delegate(cull_rect, DlMatrix());
}
void LayerStateStack::set_preroll_delegate(const DlMatrix& matrix) {
  set_preroll_delegate(kGiantRect, matrix);
}
void LayerStateStack::set_preroll_delegate(const DlRect& cull_rect,
                                           const DlMatrix& matrix) {
  clear_delegate();
  delegate_ = std::make_shared<PrerollDelegate>(cull_rect, matrix);
  reapply_all();
}

void LayerStateStack::reapply_all() {
  // We use a local RenderingAttributes instance so that it can track the
  // necessary state changes independently as they occur in the stack.
  // Reusing |outstanding_| would wreak havoc on the current state of
  // the stack. When we are finished, though, the local attributes
  // contents should match the current outstanding_ values;
  RenderingAttributes attributes = outstanding_;
  outstanding_ = {};
  for (auto& state : state_stack_) {
    state->reapply(this);
  }
  FML_DCHECK(attributes == outstanding_);
}

void LayerStateStack::fill(MutatorsStack* mutators) {
  for (auto& state : state_stack_) {
    state->update_mutators(mutators);
  }
}

void LayerStateStack::restore_to_count(size_t restore_count) {
  while (state_stack_.size() > restore_count) {
    state_stack_.back()->restore(this);
    state_stack_.pop_back();
  }
}

void LayerStateStack::push_opacity(const DlRect& bounds, DlScalar opacity) {
  maybe_save_layer(opacity);
  state_stack_.emplace_back(
      std::make_unique<OpacityEntry>(bounds, opacity, outstanding_));
  apply_last_entry();
}

void LayerStateStack::push_color_filter(
    const DlRect& bounds,
    const std::shared_ptr<const DlColorFilter>& filter) {
  maybe_save_layer(filter);
  state_stack_.emplace_back(
      std::make_unique<ColorFilterEntry>(bounds, filter, outstanding_));
  apply_last_entry();
}

void LayerStateStack::push_image_filter(
    const DlRect& bounds,
    const std::shared_ptr<DlImageFilter>& filter) {
  maybe_save_layer(filter);
  state_stack_.emplace_back(
      std::make_unique<ImageFilterEntry>(bounds, filter, outstanding_));
  apply_last_entry();
}

void LayerStateStack::push_backdrop(
    const DlRect& bounds,
    const std::shared_ptr<DlImageFilter>& filter,
    DlBlendMode blend_mode,
    std::optional<int64_t> backdrop_id) {
  state_stack_.emplace_back(std::make_unique<BackdropFilterEntry>(
      bounds, filter, blend_mode, backdrop_id, outstanding_));
  apply_last_entry();
}

void LayerStateStack::push_translate(SkScalar tx, SkScalar ty) {
  state_stack_.emplace_back(std::make_unique<TranslateEntry>(tx, ty));
  apply_last_entry();
}

void LayerStateStack::push_transform(const DlMatrix& matrix) {
  state_stack_.emplace_back(std::make_unique<TransformMatrixEntry>(matrix));
  apply_last_entry();
}

void LayerStateStack::push_integral_transform() {
  state_stack_.emplace_back(std::make_unique<IntegralTransformEntry>());
  apply_last_entry();
}

void LayerStateStack::push_clip_rect(const DlRect& rect, bool is_aa) {
  state_stack_.emplace_back(std::make_unique<ClipRectEntry>(rect, is_aa));
  apply_last_entry();
}

void LayerStateStack::push_clip_rrect(const DlRoundRect& rrect, bool is_aa) {
  state_stack_.emplace_back(std::make_unique<ClipRRectEntry>(rrect, is_aa));
  apply_last_entry();
}

void LayerStateStack::push_clip_path(const DlPath& path, bool is_aa) {
  state_stack_.emplace_back(std::make_unique<ClipPathEntry>(path, is_aa));
  apply_last_entry();
}

bool LayerStateStack::needs_save_layer(int flags) const {
  if (outstanding_.opacity < SK_Scalar1 &&
      (flags & LayerStateStack::kCallerCanApplyOpacity) == 0) {
    return true;
  }
  if (outstanding_.image_filter &&
      (flags & LayerStateStack::kCallerCanApplyImageFilter) == 0) {
    return true;
  }
  if (outstanding_.color_filter &&
      (flags & LayerStateStack::kCallerCanApplyColorFilter) == 0) {
    return true;
  }
  return false;
}

void LayerStateStack::do_save() {
  state_stack_.emplace_back(std::make_unique<SaveEntry>());
  apply_last_entry();
}

void LayerStateStack::save_layer(const DlRect& bounds) {
  state_stack_.emplace_back(std::make_unique<SaveLayerEntry>(
      bounds, DlBlendMode::kSrcOver, outstanding_));
  apply_last_entry();
}

void LayerStateStack::maybe_save_layer_for_transform(bool save_needed) {
  // Alpha and ColorFilter don't care about transform
  if (outstanding_.image_filter) {
    save_layer(outstanding_.save_layer_bounds);
  } else if (save_needed) {
    do_save();
  }
}

void LayerStateStack::maybe_save_layer_for_clip(bool save_needed) {
  // Alpha and ColorFilter don't care about clipping
  // - Alpha of clipped content == clip of alpha content
  // - Color-filtering of clipped content == clip of color-filtered content
  if (outstanding_.image_filter) {
    save_layer(outstanding_.save_layer_bounds);
  } else if (save_needed) {
    do_save();
  }
}

void LayerStateStack::maybe_save_layer(int apply_flags) {
  if (needs_save_layer(apply_flags)) {
    save_layer(outstanding_.save_layer_bounds);
  }
}

void LayerStateStack::maybe_save_layer(SkScalar opacity) {
  if (outstanding_.image_filter) {
    save_layer(outstanding_.save_layer_bounds);
  }
}

void LayerStateStack::maybe_save_layer(
    const std::shared_ptr<const DlColorFilter>& filter) {
  if (outstanding_.color_filter || outstanding_.image_filter ||
      (outstanding_.opacity < SK_Scalar1 &&
       !filter->can_commute_with_opacity())) {
    // TBD: compose the 2 color filters together.
    save_layer(outstanding_.save_layer_bounds);
  }
}

void LayerStateStack::maybe_save_layer(
    const std::shared_ptr<DlImageFilter>& filter) {
  if (outstanding_.image_filter) {
    // TBD: compose the 2 image filters together.
    save_layer(outstanding_.save_layer_bounds);
  }
}

}  // namespace flutter
