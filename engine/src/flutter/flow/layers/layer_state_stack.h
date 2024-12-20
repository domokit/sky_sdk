// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_LAYER_STATE_STACK_H_
#define FLUTTER_FLOW_LAYERS_LAYER_STATE_STACK_H_

#include "flutter/display_list/dl_canvas.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/paint_utils.h"

namespace flutter {

/// The LayerStateStack manages the inherited state passed down between
/// |Layer| objects in a |LayerTree| during |Preroll| and |Paint|.
///
/// More specifically, it manages the clip and transform state during
/// recursive rendering and will hold and lazily apply opacity, ImageFilter
/// and ColorFilter attributes to recursive content. This is not a truly
/// general state management mechnanism as it makes assumptions that code
/// will be applying the attributes to rendered content that happens in
/// recursive calls. The automatic save/restore mechanisms only work in
/// a context where C++ auto-destruct calls will engage the restore at
/// the end of a code block and that any applied attributes will only
/// be applied to the content rendered inside that block. These restrictions
/// match the organization of the |LayerTree| methods precisely.
///
/// The stack can manage a single state delegate. The delegate will provide
/// tracking of the current transform and clip and will also execute
/// saveLayer calls at the appropriate time if it is a rendering delegate.
/// The delegate can be swapped out on the fly (as is typically done by
/// PlatformViewLayer when recording the state for multiple "overlay"
/// layers that occur between embedded view subtrees. The old delegate
/// will be restored to its original state before it became a delegate
/// and the new delegate will have all of the state recorded by the stack
/// replayed into it to bring it up to speed with the current rendering
/// context.
///
/// The delegate can be any one of:
///   - Preroll delegate: used during Preroll to remember the outstanding
///                       state for embedded platform layers
///   - DlCanvas: used during Paint for rendering output
/// The stack will know which state needs to be conveyed to any of these
/// delegates and when is the best time to convey that state (i.e. lazy
/// saveLayer calls for example).
///
/// The rendering state attributes will be automatically applied to the
/// nested content using a |saveLayer| call at the point at which we
/// encounter rendered content (i.e. various nested layers that exist only
/// to apply new state will not trigger the |saveLayer| and the attributes
/// can accumulate until we reach actual content that is rendered.) Some
/// rendered content can avoid the |saveLayer| if it reports to the object
/// that it is able to apply all of the attributes that happen to be
/// outstanding (accumulated from parent state-modifiers). A |ContainerLayer|
/// can also monitor the attribute rendering capabilities of a list of
/// children and can ask the object to apply a protective |saveLayer| or
/// not based on the negotiated capabilities of the entire group.
///
/// Any code that is planning to modify the clip, transform, or rendering
/// attributes for its child content must start by calling the |save| method
/// which returns a MutatorContext object. The methods that modify such
/// state only exist on the MutatorContext object so it is difficult to get
/// that wrong, but the caller must make sure that the call happens within
/// a C++ code block that will define the "rendering scope" of those
/// state changes as they will be automatically restored on exit from that
/// block. Note that the layer might make similar state calls directly on
/// the canvas or builder during the Paint cycle (via saveLayer, transform,
/// or clip calls), but should avoid doing so if there is any nested content
/// that needs to track or react to those state calls.
///
/// Code that needs to render content can simply inform the parent of their
/// abilities by setting the |PrerollContext::renderable_state_flags| during
/// |Preroll| and then render with those attributes during |Paint| by
/// requesting the outstanding values of those attributes from the state_stack
/// object. Individual leaf layers can ignore this feature as the default
/// behavior during |Preroll| will have their parent |ContainerLayer| assume
/// that they cannot render any outstanding state attributes and will apply
/// the protective saveLayer on their behalf if needed. As such, this object
/// only provides "opt-in" features for leaf layers and no responsibilities
/// otherwise.
/// See |LayerStateStack::fill|
/// See |LayerStateStack::outstanding_opacity|
/// See |LayerStateStack::outstanding_color_filter|
/// See |LayerStateStack::outstanding_image_filter|
///
/// State-modifying layers should contain code similar to this pattern in both
/// their |Preroll| and |Paint| methods.
///
/// void [LayerType]::[Preroll/Paint](context) {
///   auto mutator = context.state_stack.save();
///   mutator.translate(origin.x, origin.y);
///   mutator.applyOpacity(content_bounds, opacity_value);
///   mutator.applyColorFilter(content_bounds, color_filter);
///   // or any of the mutator transform, clip or attribute methods
///
///   // Children will react to the state applied above during their
///   // Preroll/Paint methods or ContainerLayer will protect them
///   // conservatively by default.
///   [Preroll/Paint]Children(context);
///
///   // here the mutator will be auto-destructed and the state accumulated
///   // by it will be restored out of the state_stack and its associated
///   // delegates.
/// }
class LayerStateStack {
 public:
  LayerStateStack();

  // Clears out any old delegate to make room for a new one.
  void clear_delegate();

  // Return the DlCanvas delegate if the state stack has such a delegate.
  // The state stack will only have one delegate at a time holding either
  // a DlCanvas or a preroll accumulator.
  DlCanvas* canvas_delegate() { return delegate_->canvas(); }

  // Clears the old delegate and sets the canvas delegate to the indicated
  // DL canvas (if not nullptr). This ensures that only one delegate - either
  // a DlCanvas or a preroll accumulator - is present at any one time.
  void set_delegate(DlCanvas* canvas);

  // Clears the old delegate and sets the state stack up to accumulate
  // clip and transform information for a Preroll phase. This ensures
  // that only one delegate - either a DlCanvas or a preroll accumulator -
  // is present at any one time.
  void set_preroll_delegate(const DlRect& cull_rect, const DlMatrix& matrix);
  void set_preroll_delegate(const DlRect& cull_rect);
  void set_preroll_delegate(const DlMatrix& matrix);

  // Fills the supplied MatatorsStack object with the mutations recorded
  // by this LayerStateStack in the order encountered.
  void fill(MutatorsStack* mutators);

  class AutoRestore {
   public:
    ~AutoRestore() {
      layer_state_stack_->restore_to_count(stack_restore_count_);
    }

   private:
    AutoRestore(LayerStateStack* stack, const DlRect& bounds, int flags)
        : layer_state_stack_(stack),
          stack_restore_count_(stack->stack_count()) {
      if (stack->needs_save_layer(flags)) {
        stack->save_layer(bounds);
      }
    }
    friend class LayerStateStack;

    LayerStateStack* layer_state_stack_;
    const size_t stack_restore_count_;

    FML_DISALLOW_COPY_ASSIGN_AND_MOVE(AutoRestore);
  };

  class MutatorContext {
   public:
    ~MutatorContext() {
      layer_state_stack_->restore_to_count(stack_restore_count_);
    }

    // Immediately executes a saveLayer with all accumulated state
    // onto the canvas or builder to be applied at the next matching
    // restore. A saveLayer is always executed by this method even if
    // there are no outstanding attributes.
    void saveLayer(const DlRect& bounds);

    // Records the opacity for application at the next call to
    // saveLayer or applyState. A saveLayer may be executed at
    // this time if the opacity cannot be batched with other
    // outstanding attributes.
    void applyOpacity(const DlRect& bounds, DlScalar opacity);

    // Records the image filter for application at the next call to
    // saveLayer or applyState. A saveLayer may be executed at
    // this time if the image filter cannot be batched with other
    // outstanding attributes.
    // (Currently only opacity is recorded for batching)
    void applyImageFilter(const DlRect& bounds,
                          const std::shared_ptr<DlImageFilter>& filter);

    // Records the color filter for application at the next call to
    // saveLayer or applyState. A saveLayer may be executed at
    // this time if the color filter cannot be batched with other
    // outstanding attributes.
    // (Currently only opacity is recorded for batching)
    void applyColorFilter(const DlRect& bounds,
                          const std::shared_ptr<const DlColorFilter>& filter);

    // Saves the state stack and immediately executes a saveLayer
    // with the indicated backdrop filter and any outstanding
    // state attributes. Since the backdrop filter only applies
    // to the pixels alrady on the screen when this call is made,
    // the backdrop filter will only be applied to the canvas or
    // builder installed at the time that this call is made, and
    // subsequent canvas or builder objects that are made delegates
    // will only see a saveLayer with the indicated blend_mode.
    void applyBackdropFilter(const DlRect& bounds,
                             const std::shared_ptr<DlImageFilter>& filter,
                             DlBlendMode blend_mode,
                             std::optional<int64_t> backdrop_id);

    void translate(DlScalar tx, DlScalar ty);
    void translate(const DlPoint& tp) { translate(tp.x, tp.y); }
    void transform(const DlMatrix& matrix);
    void integralTransform();

    void clipRect(const DlRect& rect, bool is_aa);
    void clipRRect(const DlRoundRect& rrect, bool is_aa);
    void clipPath(const DlPath& path, bool is_aa);

   private:
    explicit MutatorContext(LayerStateStack* stack)
        : layer_state_stack_(stack),
          stack_restore_count_(stack->stack_count()) {}
    friend class LayerStateStack;

    LayerStateStack* layer_state_stack_;
    const size_t stack_restore_count_;
    bool save_needed_ = true;

    FML_DISALLOW_COPY_ASSIGN_AND_MOVE(MutatorContext);
  };

  static constexpr int kCallerCanApplyOpacity = 0x1;
  static constexpr int kCallerCanApplyColorFilter = 0x2;
  static constexpr int kCallerCanApplyImageFilter = 0x4;
  static constexpr int kCallerCanApplyAnything =
      (kCallerCanApplyOpacity | kCallerCanApplyColorFilter |
       kCallerCanApplyImageFilter);

  // Apply the outstanding state via saveLayer if necessary,
  // respecting the flags representing which potentially
  // outstanding attributes the calling layer can apply
  // themselves.
  //
  // A saveLayer may or may not be sent to the delegates depending
  // on how the outstanding state intersects with the flags supplied
  // by the caller.
  //
  // An AutoRestore instance will always be returned even if there
  // was no saveLayer applied.
  [[nodiscard]] inline AutoRestore applyState(const DlRect& bounds,
                                              int can_apply_flags) {
    return AutoRestore(this, bounds, can_apply_flags);
  }

  DlScalar outstanding_opacity() const { return outstanding_.opacity; }

  std::shared_ptr<const DlColorFilter> outstanding_color_filter() const {
    return outstanding_.color_filter;
  }

  std::shared_ptr<DlImageFilter> outstanding_image_filter() const {
    return outstanding_.image_filter;
  }

  // The outstanding bounds are the bounds recorded during the last
  // attribute applied to this state stack. The assumption is that
  // the nested calls to the state stack will each supply bounds relative
  // to the content of that single attribute and the bounds of the content
  // of any outstanding attributes will include the output bounds of
  // applying any nested attributes. Thus, only the innermost content
  // bounds received will be sufficient to apply all outstanding attributes.
  DlRect outstanding_bounds() const { return outstanding_.save_layer_bounds; }

  // Fill the provided paint object with any oustanding attributes and
  // return a pointer to it, or return a nullptr if there were no
  // outstanding attributes to paint with.
  DlPaint* fill(DlPaint& paint) const { return outstanding_.fill(paint); }

  // The cull_rect (not the exact clip) relative to the device pixels.
  // This rectangle may be a conservative estimate of the true clip region.
  DlRect device_cull_rect() const { return delegate_->device_cull_rect(); }

  // The cull_rect (not the exact clip) relative to the local coordinates.
  // This rectangle may be a conservative estimate of the true clip region.
  DlRect local_cull_rect() const { return delegate_->local_cull_rect(); }

  // The transform from the local coordinates to the device coordinates
  // in 4x4 DlMatrix representation. This matrix provides all information
  // needed to compute bounds for a 2D rendering primitive, and it will
  // accurately concatenate with other 4x4 matrices without losing information.
  const DlMatrix matrix() const { return delegate_->matrix(); }

  // Tests if painting content with the current outstanding attributes
  // will produce any content. This method does not check the current
  // transform or clip for being singular or empty.
  // See |content_culled|
  bool painting_is_nop() const { return outstanding_.opacity <= 0; }

  // Tests if painting content with the given bounds will produce any output.
  // This method does not check the outstanding attributes to verify that
  // they produce visible results.
  // See |painting_is_nop|
  bool content_culled(const DlRect& content_bounds) const {
    return delegate_->content_culled(content_bounds);
  }

  // Saves the current state of the state stack and returns a
  // MutatorContext which can be used to manipulate the state.
  // The state stack will be restored to its current state
  // when the MutatorContext object goes out of scope.
  [[nodiscard]] inline MutatorContext save() { return MutatorContext(this); }

  // Returns true if the state stack is in, or has returned to,
  // its initial state.
  bool is_empty() const { return state_stack_.empty(); }

 private:
  size_t stack_count() const { return state_stack_.size(); }
  void restore_to_count(size_t restore_count);
  void reapply_all();

  void apply_last_entry() { state_stack_.back()->apply(this); }

  // The push methods simply push an associated StateEntry on the stack
  // and then apply it to the current canvas and builder.
  // ---------------------
  // void push_attributes();
  void push_opacity(const DlRect& rect, DlScalar opacity);
  void push_color_filter(const DlRect& bounds,
                         const std::shared_ptr<const DlColorFilter>& filter);
  void push_image_filter(const DlRect& bounds,
                         const std::shared_ptr<DlImageFilter>& filter);
  void push_backdrop(const DlRect& bounds,
                     const std::shared_ptr<DlImageFilter>& filter,
                     DlBlendMode blend_mode,
                     std::optional<int64_t> backdrop_id);

  void push_translate(DlScalar tx, DlScalar ty);
  void push_transform(const DlMatrix& matrix);
  void push_integral_transform();

  void push_clip_rect(const DlRect& rect, bool is_aa);
  void push_clip_rrect(const DlRoundRect& rrect, bool is_aa);
  void push_clip_path(const DlPath& path, bool is_aa);
  // ---------------------

  // The maybe/needs_save_layer methods will determine if the indicated
  // attribute can be incorporated into the outstanding attributes as is,
  // or if the apply_flags are compatible with the outstanding attributes.
  // If the oustanding attributes are incompatible with the new attribute
  // or the apply flags, then a protective saveLayer will be executed.
  // ---------------------
  bool needs_save_layer(int flags) const;
  void do_save();
  void save_layer(const DlRect& bounds);
  void maybe_save_layer_for_transform(bool needs_save);
  void maybe_save_layer_for_clip(bool needs_save);
  void maybe_save_layer(int apply_flags);
  void maybe_save_layer(DlScalar opacity);
  void maybe_save_layer(const std::shared_ptr<const DlColorFilter>& filter);
  void maybe_save_layer(const std::shared_ptr<DlImageFilter>& filter);
  // ---------------------

  struct RenderingAttributes {
    // We need to record the last bounds we received for the last
    // attribute that we recorded so that we can perform a saveLayer
    // on the proper area. When an attribute is applied that cannot
    // be merged with the existing attributes, it will be submitted
    // with a bounds for its own source content, not the bounds for
    // the content that will be included in the saveLayer that applies
    // the existing outstanding attributes - thus we need to record
    // the bounds that were supplied with the most recent previous
    // attribute to be applied.
    DlRect save_layer_bounds;

    DlScalar opacity = SK_Scalar1;
    std::shared_ptr<const DlColorFilter> color_filter;
    std::shared_ptr<DlImageFilter> image_filter;

    DlPaint* fill(DlPaint& paint,
                  DlBlendMode mode = DlBlendMode::kSrcOver) const;

    bool operator==(const RenderingAttributes& other) const {
      return save_layer_bounds == other.save_layer_bounds &&
             opacity == other.opacity &&
             Equals(color_filter, other.color_filter) &&
             Equals(image_filter, other.image_filter);
    }
  };

  class StateEntry {
   public:
    virtual ~StateEntry() = default;

    virtual void apply(LayerStateStack* stack) const = 0;
    virtual void reapply(LayerStateStack* stack) const { apply(stack); }
    virtual void restore(LayerStateStack* stack) const {}
    virtual void update_mutators(MutatorsStack* mutators_stack) const {}

   protected:
    StateEntry() = default;

    FML_DISALLOW_COPY_ASSIGN_AND_MOVE(StateEntry);
  };
  friend class SaveEntry;
  friend class SaveLayerEntry;
  friend class BackdropFilterEntry;
  friend class OpacityEntry;
  friend class ImageFilterEntry;
  friend class ColorFilterEntry;
  friend class TranslateEntry;
  friend class TransformMatrixEntry;
  friend class TransformM44Entry;
  friend class IntegralTransformEntry;
  friend class ClipEntry;
  friend class ClipRectEntry;
  friend class ClipRRectEntry;
  friend class ClipPathEntry;

  class Delegate {
   protected:
    using ClipOp = DlCanvas::ClipOp;

   public:
    virtual ~Delegate() = default;

    // Mormally when a |Paint| or |Preroll| cycle is completed, the
    // delegate will have been rewound to its initial state by the
    // trailing recursive actions of the paint and preroll methods.
    // When a delegate is swapped out, there may be unresolved state
    // that the delegate received. This method is called when the
    // delegate is cleared or swapped out to inform it to rewind its
    // state and finalize all outstanding save or saveLayer operations.
    virtual void decommission() = 0;

    virtual DlCanvas* canvas() const { return nullptr; }

    virtual DlRect local_cull_rect() const = 0;
    virtual DlRect device_cull_rect() const = 0;
    virtual DlMatrix matrix() const = 0;
    virtual bool content_culled(const DlRect& content_bounds) const = 0;

    virtual void save() = 0;
    virtual void saveLayer(
        const DlRect& bounds,
        RenderingAttributes& attributes,
        DlBlendMode blend,
        const DlImageFilter* backdrop,
        std::optional<int64_t> backdrop_id = std::nullopt) = 0;
    virtual void restore() = 0;

    virtual void translate(DlScalar tx, DlScalar ty) = 0;
    virtual void transform(const DlMatrix& matrix) = 0;
    virtual void integralTransform() = 0;

    virtual void clipRect(const DlRect& rect, ClipOp op, bool is_aa) = 0;
    virtual void clipRRect(const DlRoundRect& rrect, ClipOp op, bool is_aa) = 0;
    virtual void clipPath(const DlPath& path, ClipOp op, bool is_aa) = 0;
  };
  friend class DummyDelegate;
  friend class DlCanvasDelegate;
  friend class PrerollDelegate;

  std::vector<std::unique_ptr<StateEntry>> state_stack_;
  friend class MutatorContext;

  std::shared_ptr<Delegate> delegate_;
  RenderingAttributes outstanding_;

  friend class SaveLayerEntry;
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_LAYER_STATE_STACK_H_
