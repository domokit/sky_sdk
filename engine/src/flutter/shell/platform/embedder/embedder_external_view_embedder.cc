// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_external_view_embedder.h"

#include <cassert>
#include <utility>

#include "flutter/common/constants.h"
#include "flutter/shell/platform/embedder/embedder_layers.h"
#include "flutter/shell/platform/embedder/embedder_render_target.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

static const auto kRootViewIdentifier = EmbedderExternalView::ViewIdentifier{};

EmbedderExternalViewEmbedder::EmbedderExternalViewEmbedder(
    bool avoid_backing_store_cache,
    const CreateRenderTargetCallback& create_render_target_callback,
    const PresentCallback& present_callback)
    : avoid_backing_store_cache_(avoid_backing_store_cache),
      create_render_target_callback_(create_render_target_callback),
      present_callback_(present_callback) {
  FML_DCHECK(create_render_target_callback_);
  FML_DCHECK(present_callback_);
}

EmbedderExternalViewEmbedder::~EmbedderExternalViewEmbedder() = default;

void EmbedderExternalViewEmbedder::CollectView(int64_t view_id) {
  render_target_caches_.erase(view_id);
}

void EmbedderExternalViewEmbedder::SetSurfaceTransformationCallback(
    SurfaceTransformationCallback surface_transformation_callback) {
  surface_transformation_callback_ = std::move(surface_transformation_callback);
}

SkMatrix EmbedderExternalViewEmbedder::GetSurfaceTransformation() const {
  if (!surface_transformation_callback_) {
    return SkMatrix{};
  }

  return surface_transformation_callback_();
}

void EmbedderExternalViewEmbedder::Reset() {
  pending_views_.clear();
  composition_order_.clear();
}

// |ExternalViewEmbedder|
void EmbedderExternalViewEmbedder::CancelFrame() {
  Reset();
}

// |ExternalViewEmbedder|
void EmbedderExternalViewEmbedder::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void EmbedderExternalViewEmbedder::PrepareFlutterView(
    SkISize frame_size,
    double device_pixel_ratio) {
  Reset();

  pending_frame_size_ = frame_size;
  pending_device_pixel_ratio_ = device_pixel_ratio;
  pending_surface_transformation_ = GetSurfaceTransformation();

  pending_views_[kRootViewIdentifier] = std::make_unique<EmbedderExternalView>(
      pending_frame_size_, pending_surface_transformation_);
  composition_order_.push_back(kRootViewIdentifier);
}

// |ExternalViewEmbedder|
void EmbedderExternalViewEmbedder::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<EmbeddedViewParams> params) {
  auto vid = EmbedderExternalView::ViewIdentifier(view_id);
  FML_DCHECK(pending_views_.count(vid) == 0);

  pending_views_[vid] = std::make_unique<EmbedderExternalView>(
      pending_frame_size_,              // frame size
      pending_surface_transformation_,  // surface xformation
      vid,                              // view identifier
      std::move(params)                 // embedded view params
  );
  composition_order_.push_back(vid);
}

// |ExternalViewEmbedder|
DlCanvas* EmbedderExternalViewEmbedder::GetRootCanvas() {
  auto found = pending_views_.find(kRootViewIdentifier);
  if (found == pending_views_.end()) {
    FML_DLOG(WARNING)
        << "No root canvas could be found. This is extremely unlikely and "
           "indicates that the external view embedder did not receive the "
           "notification to begin the frame.";
    return nullptr;
  }
  return found->second->GetCanvas();
}

// |ExternalViewEmbedder|
DlCanvas* EmbedderExternalViewEmbedder::CompositeEmbeddedView(int64_t view_id) {
  auto vid = EmbedderExternalView::ViewIdentifier(view_id);
  auto found = pending_views_.find(vid);
  if (found == pending_views_.end()) {
    FML_DCHECK(false) << "Attempted to composite a view that was not "
                         "pre-rolled.";
    return nullptr;
  }
  return found->second->GetCanvas();
}

static FlutterBackingStoreConfig MakeBackingStoreConfig(
    int64_t view_id,
    const SkISize& backing_store_size) {
  FlutterBackingStoreConfig config = {};

  config.struct_size = sizeof(config);

  config.size.width = backing_store_size.width();
  config.size.height = backing_store_size.height();
  config.view_id = view_id;

  return config;
}

namespace {

struct PlatformView {
  EmbedderExternalView::ViewIdentifier view_identifier;
  const EmbeddedViewParams* params;

  // The frame of the platform view, after clipping, in screen coordinates.
  SkRect clipped_frame;

  explicit PlatformView(const EmbedderExternalView* view) {
    FML_DCHECK(view->HasPlatformView());
    view_identifier = view->GetViewIdentifier();
    params = view->GetEmbeddedViewParams();

    clipped_frame = view->GetEmbeddedViewParams()->finalBoundingRect();
    SkMatrix transform;
    for (auto i = params->mutatorsStack().Begin();
         i != params->mutatorsStack().End(); ++i) {
      const auto& m = *i;
      switch (m->GetType()) {
        case kClipRect: {
          auto rect = transform.mapRect(m->GetRect());
          if (!clipped_frame.intersect(rect)) {
            clipped_frame = SkRect::MakeEmpty();
          }
          break;
        }
        case kClipRRect: {
          auto rect = transform.mapRect(m->GetRRect().getBounds());
          if (!clipped_frame.intersect(rect)) {
            clipped_frame = SkRect::MakeEmpty();
          }
          break;
        }
        case kClipPath: {
          auto rect = transform.mapRect(m->GetPath().getBounds());
          if (!clipped_frame.intersect(rect)) {
            clipped_frame = SkRect::MakeEmpty();
          }
          break;
        }
        case kTransform: {
          transform.preConcat(m->GetMatrix());
          break;
        }
        case kOpacity:
        case kBackdropFilter:
          break;
      }
    }
  }
};

/// Each layer will result in a single physical surface that contains Flutter
/// contents. It may contain multiple platform views and the slices
/// that would be otherwise rendered between these platform views will be
/// collapsed into this layer, as long as they do not intersect any of the
/// platform views.
/// In Z order the Flutter contents of Layer is above the platform views.
class Layer {
 public:
  /// Returns whether the rectangle intersects any of the platform views of
  /// this layer.
  bool IntersectsPlatformView(const SkRect& rect) {
    for (auto& platform_view : platform_views_) {
      if (platform_view.clipped_frame.intersects(rect)) {
        return true;
      }
    }
    return false;
  }

  /// Returns whether the region intersects any of the platform views of this
  /// layer.
  bool IntersectsPlatformView(const DlRegion& region) {
    for (auto& platform_view : platform_views_) {
      if (region.intersects(platform_view.clipped_frame.roundOut())) {
        return true;
      }
    }
    return false;
  }

  /// Returns whether the rectangle intersects any of the Flutter contents of
  /// this layer.
  bool IntersectsFlutterContents(const SkRect& rect) {
    return flutter_contents_region_.intersects(rect.roundOut());
  }

  /// Returns whether the region intersects any of the Flutter contents of this
  /// layer.
  bool IntersectsFlutterContents(const DlRegion& region) {
    return flutter_contents_region_.intersects(region);
  }

  /// Adds a platform view to this layer.
  void AddPlatformView(const PlatformView& platform_view) {
    platform_views_.push_back(platform_view);
  }

  /// Adds Flutter contents to this layer.
  void AddFlutterContents(EmbedderExternalView* contents,
                          const DlRegion& contents_region) {
    flutter_contents_.push_back(contents);
    flutter_contents_region_ =
        DlRegion::MakeUnion(flutter_contents_region_, contents_region);
  }

  bool has_flutter_contents() const { return !flutter_contents_.empty(); }

  void SetRenderTarget(std::unique_ptr<EmbedderRenderTarget> target) {
    FML_DCHECK(render_target_ == nullptr);
    FML_DCHECK(has_flutter_contents());
    render_target_ = std::move(target);
  }

  /// Renders this layer Flutter contents to the render target previously
  /// assigned with SetRenderTarget.
  void RenderFlutterContents() {
    FML_DCHECK(has_flutter_contents());
    if (render_target_) {
      bool clear_surface = true;
      for (auto c : flutter_contents_) {
        c->Render(*render_target_, clear_surface);
        clear_surface = false;
      }
    }
  }

  /// Returns platform views for this layer. In Z-order the platform views are
  /// positioned *below* this layer's Flutter contents.
  const std::vector<PlatformView>& platform_views() const {
    return platform_views_;
  }

  EmbedderRenderTarget* render_target() { return render_target_.get(); }

  std::vector<DlIRect> coverage() {
    return flutter_contents_region_.getRects();
  }

 private:
  std::vector<PlatformView> platform_views_;
  std::vector<EmbedderExternalView*> flutter_contents_;
  DlRegion flutter_contents_region_;
  std::unique_ptr<EmbedderRenderTarget> render_target_;
  friend class LayerBuilder;
};

/// A layout builder is responsible for building an optimized list of Layers
/// from a list of `EmbedderExternalView`s. Single EmbedderExternalView contains
/// at most one platform view and at most one layer of Flutter contents
/// ('slice'). LayerBuilder is responsible for producing as few Layers from the
/// list of EmbedderExternalViews as possible while maintaining identical visual
/// result.
///
/// Implements https://flutter.dev/go/optimized-platform-view-layers
class LayerBuilder {
 public:
  using RenderTargetProvider =
      std::function<std::unique_ptr<EmbedderRenderTarget>(
          const SkISize& frame_size)>;

  explicit LayerBuilder(SkISize frame_size) : frame_size_(frame_size) {
    layers_.push_back(Layer());
  }

  /// Adds the platform view and/or flutter contents from the
  /// EmbedderExternalView instance.
  ///
  /// This will try to add the content and platform view to an existing layer
  /// if possible. If not, a new layer will be created.
  void AddExternalView(EmbedderExternalView* view) {
    if (view->HasPlatformView()) {
      PlatformView platform_view(view);
      AddPlatformView(platform_view);
    }
    if (view->HasEngineRenderedContents()) {
      AddFlutterContents(view);
    }
  }

  /// Prepares the render targets for all layers that have Flutter contents.
  void PrepareBackingStore(const RenderTargetProvider& target_provider) {
    for (auto& layer : layers_) {
      if (layer.has_flutter_contents()) {
        layer.SetRenderTarget(target_provider(frame_size_));
      }
    }
  }

  /// Renders all layers with Flutter contents to their respective render
  /// targets.
  void Render() {
    for (auto& layer : layers_) {
      if (layer.has_flutter_contents()) {
        layer.RenderFlutterContents();
      }
    }
  }

  /// Populates EmbedderLayers from layer builder's layers.
  void PushLayers(EmbedderLayers& layers) {
    for (auto& layer : layers_) {
      for (auto& view : layer.platform_views()) {
        auto platform_view_id = view.view_identifier.platform_view_id;
        if (platform_view_id.has_value()) {
          layers.PushPlatformViewLayer(platform_view_id.value(), *view.params);
        }
      }
      if (layer.render_target() != nullptr) {
        layers.PushBackingStoreLayer(layer.render_target()->GetBackingStore(),
                                     layer.coverage());
      }
    }
  }

  /// Removes the render targets from layers and returns them for collection.
  std::vector<std::unique_ptr<EmbedderRenderTarget>>
  ClearAndCollectRenderTargets() {
    std::vector<std::unique_ptr<EmbedderRenderTarget>> result;
    for (auto& layer : layers_) {
      if (layer.render_target() != nullptr) {
        result.push_back(std::move(layer.render_target_));
      }
    }
    layers_.clear();
    return result;
  }

 private:
  void AddPlatformView(PlatformView view) {
    GetLayerForPlatformView(view).AddPlatformView(view);
  }

  void AddFlutterContents(EmbedderExternalView* contents) {
    FML_DCHECK(contents->HasEngineRenderedContents());

    DlRegion region = contents->GetDlRegion();
    GetLayerForFlutterContentsRegion(region).AddFlutterContents(contents,
                                                                region);
  }

  /// Returns the deepest layer to which the platform view can be added. That
  /// would be (whichever comes first):
  /// - First layer from back that has platform view that intersects with this
  ///   view
  /// - Very last layer from back that has surface that doesn't intersect with
  ///   this. That is because layer content renders on top of the platform view.
  Layer& GetLayerForPlatformView(PlatformView view) {
    for (auto iter = layers_.rbegin(); iter != layers_.rend(); ++iter) {
      // This layer has surface that intersects with this view. That means we
      // went one too far and need the layer before this.
      if (iter->IntersectsFlutterContents(view.clipped_frame)) {
        if (iter == layers_.rbegin()) {
          layers_.emplace_back();
          return layers_.back();
        } else {
          --iter;
          return *iter;
        }
      }
      if (iter->IntersectsPlatformView(view.clipped_frame)) {
        return *iter;
      }
    }
    return layers_.front();
  }

  /// Finds layer to which the Flutter content can be added. That would
  /// be first layer from back that has any intersection with this region.
  Layer& GetLayerForFlutterContentsRegion(const DlRegion& region) {
    for (auto iter = layers_.rbegin(); iter != layers_.rend(); ++iter) {
      if (iter->IntersectsPlatformView(region) ||
          iter->IntersectsFlutterContents(region)) {
        return *iter;
      }
    }
    return layers_.front();
  }

  std::vector<Layer> layers_;
  SkISize frame_size_;
};

};  // namespace

void EmbedderExternalViewEmbedder::SubmitFlutterView(
    int64_t flutter_view_id,
    GrDirectContext* context,
    const std::shared_ptr<impeller::AiksContext>& aiks_context,
    std::unique_ptr<SurfaceFrame> frame) {
  // The unordered_map render_target_cache creates a new entry if the view ID is
  // unrecognized.
  EmbedderRenderTargetCache& render_target_cache =
      render_target_caches_[flutter_view_id];
  SkRect _rect = SkRect::MakeIWH(pending_frame_size_.width(),
                                 pending_frame_size_.height());
  pending_surface_transformation_.mapRect(&_rect);

  LayerBuilder builder(SkISize::Make(_rect.width(), _rect.height()));

  for (auto view_id : composition_order_) {
    auto& view = pending_views_[view_id];
    builder.AddExternalView(view.get());
  }

  builder.PrepareBackingStore([&](const SkISize& frame_size) {
    if (!avoid_backing_store_cache_) {
      std::unique_ptr<EmbedderRenderTarget> target =
          render_target_cache.GetRenderTarget(
              EmbedderExternalView::RenderTargetDescriptor(frame_size));
      if (target != nullptr) {
        return target;
      }
    }
    auto config = MakeBackingStoreConfig(flutter_view_id, frame_size);
    return create_render_target_callback_(context, aiks_context, config);
  });

  // This is where unused render targets will be collected. Control may flow
  // to the embedder. Here, the embedder has the opportunity to trample on the
  // OpenGL context.
  //
  // For optimum performance, we should tell the render target cache to clear
  // its unused entries before allocating new ones. This collection step
  // before allocating new render targets ameliorates peak memory usage within
  // the frame. But, this causes an issue in a known internal embedder. To
  // work around this issue while that embedder migrates, collection of render
  // targets is deferred after the presentation.
  //
  // @warning: Embedder may trample on our OpenGL context here.
  auto deferred_cleanup_render_targets =
      render_target_cache.ClearAllRenderTargetsInCache();

#if !SLIMPELLER
  // The OpenGL context could have been trampled by the embedder at this point
  // as it attempted to collect old render targets and create new ones. Tell
  // Skia to not rely on existing bindings.
  if (context) {
    context->resetContext(kAll_GrBackendState);
  }
#endif  //  !SLIMPELLER

  builder.Render();

#if !SLIMPELLER
  // We are going to be transferring control back over to the embedder there
  // the context may be trampled upon again. Flush all operations to the
  // underlying rendering API.
  //
  // @warning: Embedder may trample on our OpenGL context here.
  if (context) {
    context->flushAndSubmit();
  }
#endif  //  !SLIMPELLER

  {
    auto presentation_time_optional = frame->submit_info().presentation_time;
    uint64_t presentation_time =
        presentation_time_optional.has_value()
            ? presentation_time_optional->ToEpochDelta().ToNanoseconds()
            : 0;

    // Submit the scribbled layer to the embedder for presentation.
    //
    // @warning: Embedder may trample on our OpenGL context here.
    EmbedderLayers presented_layers(
        pending_frame_size_, pending_device_pixel_ratio_,
        pending_surface_transformation_, presentation_time);

    builder.PushLayers(presented_layers);

    presented_layers.InvokePresentCallback(flutter_view_id, present_callback_);
  }

  // See why this is necessary in the comment where this collection in
  // realized.
  //
  // @warning: Embedder may trample on our OpenGL context here.
  deferred_cleanup_render_targets.clear();

  auto render_targets = builder.ClearAndCollectRenderTargets();
  for (auto& render_target : render_targets) {
    if (!avoid_backing_store_cache_) {
      render_target_cache.CacheRenderTarget(std::move(render_target));
    }
  }

  frame->Submit();
}

}  // namespace flutter
