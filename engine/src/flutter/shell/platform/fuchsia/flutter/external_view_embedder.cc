// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "external_view_embedder.h"

#include <algorithm>
#include <cstdint>

#include "flutter/common/constants.h"
#include "flutter/fml/trace_event.h"
#include "third_party/skia/include/core/SkPicture.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/GrRecordingContext.h"

namespace flutter_runner {
namespace {

void AttachClipTransformChild(
    FlatlandConnection* flatland,
    ExternalViewEmbedder::ClipTransform* parent_clip_transform,
    const fuchsia::ui::composition::TransformId& child_transform_id) {
  flatland->flatland()->AddChild(parent_clip_transform->transform_id,
                                 child_transform_id);
  parent_clip_transform->children.push_back(child_transform_id);
}

void DetachClipTransformChildren(
    FlatlandConnection* flatland,
    ExternalViewEmbedder::ClipTransform* clip_transform) {
  for (auto& child : clip_transform->children) {
    flatland->flatland()->RemoveChild(clip_transform->transform_id, child);
  }
  clip_transform->children.clear();
}

}  // namespace

ExternalViewEmbedder::ExternalViewEmbedder(
    fuchsia::ui::views::ViewCreationToken view_creation_token,
    fuchsia::ui::views::ViewIdentityOnCreation view_identity,
    fuchsia::ui::composition::ViewBoundProtocols view_protocols,
    fidl::InterfaceRequest<fuchsia::ui::composition::ParentViewportWatcher>
        parent_viewport_watcher_request,
    std::shared_ptr<FlatlandConnection> flatland,
    std::shared_ptr<SurfaceProducer> surface_producer,
    bool intercept_all_input)
    : flatland_(flatland), surface_producer_(surface_producer) {
  flatland_->flatland()->CreateView2(
      std::move(view_creation_token), std::move(view_identity),
      std::move(view_protocols), std::move(parent_viewport_watcher_request));

  root_transform_id_ = flatland_->NextTransformId();
  flatland_->flatland()->CreateTransform(root_transform_id_);
  flatland_->flatland()->SetRootTransform(root_transform_id_);

  if (intercept_all_input) {
    input_interceptor_transform_ = flatland_->NextTransformId();
    flatland_->flatland()->CreateTransform(*input_interceptor_transform_);
    flatland_->flatland()->SetInfiniteHitRegion(
        *input_interceptor_transform_,
        fuchsia::ui::composition::HitTestInteraction::SEMANTICALLY_INVISIBLE);

    flatland_->flatland()->AddChild(root_transform_id_,
                                    *input_interceptor_transform_);
    child_transforms_.emplace_back(*input_interceptor_transform_);
  }
}

ExternalViewEmbedder::~ExternalViewEmbedder() = default;

flutter::DlCanvas* ExternalViewEmbedder::GetRootCanvas() {
  auto found = frame_layers_.find(kRootLayerId);
  if (found == frame_layers_.end()) {
    FML_LOG(WARNING)
        << "No root canvas could be found. This is extremely unlikely and "
           "indicates that the external view embedder did not receive the "
           "notification to begin the frame.";
    return nullptr;
  }

  return found->second.canvas_spy->GetSpyingCanvas();
}

void ExternalViewEmbedder::PrerollCompositeEmbeddedView(
    int64_t view_id,
    std::unique_ptr<flutter::EmbeddedViewParams> params) {
  zx_handle_t handle = static_cast<zx_handle_t>(view_id);
  FML_CHECK(frame_layers_.count(handle) == 0);

  frame_layers_.emplace(std::make_pair(
      EmbedderLayerId{handle},
      EmbedderLayer(frame_size_, *params, flutter::RTreeFactory())));
  frame_composition_order_.push_back(handle);
}

flutter::DlCanvas* ExternalViewEmbedder::CompositeEmbeddedView(
    int64_t view_id) {
  zx_handle_t handle = static_cast<zx_handle_t>(view_id);
  auto found = frame_layers_.find(handle);
  FML_CHECK(found != frame_layers_.end());

  return found->second.canvas_spy->GetSpyingCanvas();
}

flutter::PostPrerollResult ExternalViewEmbedder::PostPrerollAction(
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  return flutter::PostPrerollResult::kSuccess;
}

void ExternalViewEmbedder::BeginFrame(
    GrDirectContext* context,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {}

// |ExternalViewEmbedder|
void ExternalViewEmbedder::PrepareFlutterView(SkISize frame_size,
                                              double device_pixel_ratio) {
  // Reset for new view.
  Reset();
  frame_size_ = frame_size;
  frame_dpr_ = device_pixel_ratio;

  // Create the root layer.
  frame_layers_.emplace(std::make_pair(
      kRootLayerId,
      EmbedderLayer(frame_size, std::nullopt, flutter::RTreeFactory())));
  frame_composition_order_.push_back(kRootLayerId);
}

void ExternalViewEmbedder::EndFrame(
    bool should_resubmit_frame,
    const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger) {
  TRACE_EVENT0("flutter", "ExternalViewEmbedder::EndFrame");
}

void ExternalViewEmbedder::SubmitFlutterView(
    int64_t flutter_view_id,
    GrDirectContext* context,
    const std::shared_ptr<impeller::AiksContext>& aiks_context,
    std::unique_ptr<flutter::SurfaceFrame> frame) {
  // Fuchsia only supports operating the implicit view for now.
  FML_DCHECK(flutter_view_id == flutter::kFlutterImplicitViewId);

  TRACE_EVENT0("flutter", "ExternalViewEmbedder::SubmitFlutterView");
  std::vector<std::unique_ptr<SurfaceProducerSurface>> frame_surfaces;
  std::unordered_map<EmbedderLayerId, size_t> frame_surface_indices;

  // Create surfaces for the frame and associate them with layer IDs.
  {
    TRACE_EVENT0("flutter", "CreateSurfaces");

    for (const auto& layer : frame_layers_) {
      if (!layer.second.canvas_spy->DidDrawIntoCanvas()) {
        continue;
      }

      auto surface =
          surface_producer_->ProduceSurface(layer.second.surface_size);
      if (!surface) {
        const std::string layer_id_str =
            layer.first.has_value() ? std::to_string(layer.first.value())
                                    : "Background";
        FML_LOG(ERROR) << "Failed to create surface for layer " << layer_id_str
                       << "; size (" << layer.second.surface_size.width()
                       << ", " << layer.second.surface_size.height() << ")";
        FML_DCHECK(false);
        continue;
      }

      // If we receive an unitialized surface, we need to first create flatland
      // resource.
      if (surface->GetImageId() == 0) {
        auto image_id = flatland_->NextContentId().value;
        const auto& size = surface->GetSize();
        fuchsia::ui::composition::ImageProperties image_properties;
        image_properties.set_size({static_cast<uint32_t>(size.width()),
                                   static_cast<uint32_t>(size.height())});
        flatland_->flatland()->CreateImage(
            {image_id}, surface->GetBufferCollectionImportToken(), 0,
            std::move(image_properties));

        surface->SetImageId(image_id);
        surface->SetReleaseImageCallback([flatland = flatland_, image_id]() {
          flatland->flatland()->ReleaseImage({image_id});
        });
      }

      // Enqueue fences for the next present.
      flatland_->EnqueueAcquireFence(surface->GetAcquireFence());
      flatland_->EnqueueReleaseFence(surface->GetReleaseFence());

      frame_surface_indices.emplace(
          std::make_pair(layer.first, frame_surfaces.size()));
      frame_surfaces.emplace_back(std::move(surface));
    }
  }

  // Finish recording SkPictures.
  {
    TRACE_EVENT0("flutter", "FinishRecordingPictures");

    for (const auto& surface_index : frame_surface_indices) {
      const auto& layer = frame_layers_.find(surface_index.first);
      FML_CHECK(layer != frame_layers_.end());
      layer->second.picture =
          layer->second.recorder->finishRecordingAsPicture();
      FML_CHECK(layer->second.picture != nullptr);
    }
  }

  // Submit layers and platform views to Scenic in composition order.
  {
    TRACE_EVENT0("flutter", "SubmitLayers");

    // First re-scale everything according to the DPR.
    const float inv_dpr = 1.0f / frame_dpr_;
    flatland_->flatland()->SetScale(root_transform_id_, {inv_dpr, inv_dpr});

    size_t layer_index = 0;
    for (const auto& layer_id : frame_composition_order_) {
      const auto& layer = frame_layers_.find(layer_id);
      FML_CHECK(layer != frame_layers_.end());

      // Draw the PlatformView associated with each layer first.
      if (layer_id.has_value()) {
        FML_CHECK(layer->second.embedded_view_params.has_value());
        auto& view_params = layer->second.embedded_view_params.value();

        // Get the View structure corresponding to the platform view.
        auto found = views_.find(layer_id.value());
        FML_CHECK(found != views_.end())
            << "No View for layer_id = " << layer_id.value()
            << ". This typically indicates that the Dart code in "
               "Fuchsia's fuchsia_scenic_flutter library failed to create "
               "the platform view, leading to a crash later down the road in "
               "the Flutter Engine code that tries to find that platform view. "
               "Check the Flutter Framework for changes to PlatformView that "
               "might have caused a regression.";
        auto& viewport = found->second;

        // Compute mutators, and size for the platform view.
        const ViewMutators view_mutators =
            ParseMutatorStack(view_params.mutatorsStack());
        const SkSize view_size = view_params.sizePoints();

        // Verify that we're unpacking the mutators' transform matrix correctly.
        // Use built-in get method for SkMatrix to get values, see:
        // https://source.corp.google.com/piper///depot/google3/third_party/skia/HEAD/include/core/SkMatrix.h;l=391
        for (int index = 0; index < 9; index++) {
          const SkScalar mutators_transform_value =
              view_mutators.total_transform.get(index);
          const SkScalar params_transform_value =
              view_params.transformMatrix().get(index);
          if (!SkScalarNearlyEqual(mutators_transform_value,
                                   params_transform_value, 0.0005f)) {
            FML_LOG(ERROR)
                << "Assertion failed: view_mutators.total_transform[" << index
                << "] (" << mutators_transform_value
                << ") != view_params.transformMatrix()[" << index << "] ("
                << params_transform_value
                << "). This likely means there is a bug with the "
                << "logic for parsing embedded views' transform matrices.";
          }
        }

        if (viewport.pending_create_viewport_callback) {
          if (view_size.fWidth && view_size.fHeight) {
            viewport.pending_create_viewport_callback(
                view_size, viewport.pending_occlusion_hint);
            viewport.size = view_size;
            viewport.occlusion_hint = viewport.pending_occlusion_hint;
          } else {
            FML_DLOG(WARNING)
                << "Failed to create viewport because width or height is zero.";
          }
        }

        // Set transform for the viewport.
        if (view_mutators.transform != viewport.mutators.transform) {
          flatland_->flatland()->SetTranslation(
              viewport.transform_id,
              {static_cast<int32_t>(view_mutators.transform.getTranslateX()),
               static_cast<int32_t>(view_mutators.transform.getTranslateY())});
          flatland_->flatland()->SetScale(
              viewport.transform_id, {view_mutators.transform.getScaleX(),
                                      view_mutators.transform.getScaleY()});
          viewport.mutators.transform = view_mutators.transform;
        }

        // Set clip regions.
        if (view_mutators.clips != viewport.mutators.clips) {
          // Expand the clip_transforms array to fit any new transforms.
          while (viewport.clip_transforms.size() < view_mutators.clips.size()) {
            ClipTransform clip_transform;
            clip_transform.transform_id = flatland_->NextTransformId();
            flatland_->flatland()->CreateTransform(clip_transform.transform_id);
            viewport.clip_transforms.emplace_back(std::move(clip_transform));
          }
          FML_CHECK(viewport.clip_transforms.size() >=
                    view_mutators.clips.size());

          // Adjust and re-parent all clip transforms.
          for (auto& clip_transform : viewport.clip_transforms) {
            DetachClipTransformChildren(flatland_.get(), &clip_transform);
          }

          for (size_t c = 0; c < view_mutators.clips.size(); c++) {
            const SkMatrix& clip_matrix = view_mutators.clips[c].transform;
            const SkRect& clip_rect = view_mutators.clips[c].rect;

            flatland_->flatland()->SetTranslation(
                viewport.clip_transforms[c].transform_id,
                {static_cast<int32_t>(clip_matrix.getTranslateX()),
                 static_cast<int32_t>(clip_matrix.getTranslateY())});
            flatland_->flatland()->SetScale(
                viewport.clip_transforms[c].transform_id,
                {clip_matrix.getScaleX(), clip_matrix.getScaleY()});
            fuchsia::math::Rect rect = {
                static_cast<int32_t>(clip_rect.x()),
                static_cast<int32_t>(clip_rect.y()),
                static_cast<int32_t>(clip_rect.width()),
                static_cast<int32_t>(clip_rect.height())};
            flatland_->flatland()->SetClipBoundary(
                viewport.clip_transforms[c].transform_id,
                std::make_unique<fuchsia::math::Rect>(std::move(rect)));

            const auto child_transform_id =
                c != (view_mutators.clips.size() - 1)
                    ? viewport.clip_transforms[c + 1].transform_id
                    : viewport.transform_id;
            AttachClipTransformChild(flatland_.get(),
                                     &(viewport.clip_transforms[c]),
                                     child_transform_id);
          }
          viewport.mutators.clips = view_mutators.clips;
        }

        // Set opacity.
        if (view_mutators.opacity != viewport.mutators.opacity) {
          flatland_->flatland()->SetOpacity(viewport.transform_id,
                                            view_mutators.opacity);
          viewport.mutators.opacity = view_mutators.opacity;
        }

        // Set size and occlusion hint.
        if (view_size != viewport.size ||
            viewport.pending_occlusion_hint != viewport.occlusion_hint) {
          fuchsia::ui::composition::ViewportProperties properties;
          properties.set_logical_size(
              {static_cast<uint32_t>(view_size.fWidth),
               static_cast<uint32_t>(view_size.fHeight)});
          properties.set_inset(
              {static_cast<int32_t>(viewport.pending_occlusion_hint.fTop),
               static_cast<int32_t>(viewport.pending_occlusion_hint.fRight),
               static_cast<int32_t>(viewport.pending_occlusion_hint.fBottom),
               static_cast<int32_t>(viewport.pending_occlusion_hint.fLeft)});
          flatland_->flatland()->SetViewportProperties(viewport.viewport_id,
                                                       std::move(properties));
          viewport.size = view_size;
          viewport.occlusion_hint = viewport.pending_occlusion_hint;
        }

        // Attach the View to the main scene graph.
        const auto main_child_transform =
            viewport.mutators.clips.empty()
                ? viewport.transform_id
                : viewport.clip_transforms[0].transform_id;
        flatland_->flatland()->AddChild(root_transform_id_,
                                        main_child_transform);
        child_transforms_.emplace_back(main_child_transform);
      }

      // Acquire the surface associated with the layer.
      SurfaceProducerSurface* surface_for_layer = nullptr;
      if (layer->second.canvas_spy->DidDrawIntoCanvas()) {
        const auto& surface_index = frame_surface_indices.find(layer_id);
        if (surface_index != frame_surface_indices.end()) {
          FML_CHECK(surface_index->second < frame_surfaces.size());
          surface_for_layer = frame_surfaces[surface_index->second].get();
          FML_CHECK(surface_for_layer != nullptr);
        } else {
          const std::string layer_id_str =
              layer_id.has_value() ? std::to_string(layer_id.value())
                                   : "Background";
          FML_LOG(ERROR) << "Missing surface for layer " << layer_id_str
                         << "; skipping scene graph add of layer.";
          FML_DCHECK(false);
        }
      }

      // Draw the layer if we acquired a surface for it successfully.
      if (surface_for_layer != nullptr) {
        // Create a new layer if needed for the surface.
        FML_CHECK(layer_index <= layers_.size());
        if (layer_index == layers_.size()) {
          Layer new_layer{.transform_id = flatland_->NextTransformId()};
          flatland_->flatland()->CreateTransform(new_layer.transform_id);
          layers_.emplace_back(std::move(new_layer));
        }

        // Update the image content and set size.
        flatland_->flatland()->SetContent(layers_[layer_index].transform_id,
                                          {surface_for_layer->GetImageId()});
        flatland_->flatland()->SetImageDestinationSize(
            {surface_for_layer->GetImageId()},
            {static_cast<uint32_t>(surface_for_layer->GetSize().width()),
             static_cast<uint32_t>(surface_for_layer->GetSize().height())});

        // Flutter Embedder lacks an API to detect if a layer has alpha or not.
        // For now, we assume any layer beyond the first has alpha.
        flatland_->flatland()->SetImageBlendingFunction(
            {surface_for_layer->GetImageId()},
            layer_index == 0 ? fuchsia::ui::composition::BlendMode::SRC
                             : fuchsia::ui::composition::BlendMode::SRC_OVER);

        // Set hit regions for this layer; these hit regions correspond to the
        // portions of the layer on which skia drew content.
        {
          FML_CHECK(layer->second.rtree);
          std::list<SkRect> intersection_rects =
              layer->second.rtree->searchNonOverlappingDrawnRects(
                  SkRect::Make(layer->second.surface_size));

          std::vector<fuchsia::ui::composition::HitRegion> hit_regions;
          for (const SkRect& rect : intersection_rects) {
            hit_regions.emplace_back();
            auto& new_hit_region = hit_regions.back();
            new_hit_region.region.x = rect.x();
            new_hit_region.region.y = rect.y();
            new_hit_region.region.width = rect.width();
            new_hit_region.region.height = rect.height();
            new_hit_region.hit_test =
                fuchsia::ui::composition::HitTestInteraction::DEFAULT;
          }

          flatland_->flatland()->SetHitRegions(
              layers_[layer_index].transform_id, std::move(hit_regions));
        }

        // Attach the Layer to the main scene graph.
        flatland_->flatland()->AddChild(root_transform_id_,
                                        layers_[layer_index].transform_id);
        child_transforms_.emplace_back(layers_[layer_index].transform_id);
      }

      // Reset for the next pass:
      layer_index++;
    }

    // Set up the input interceptor at the top of the scene, if applicable. It
    // will capture all input, and any unwanted input will be reinjected into
    // embedded views.
    if (input_interceptor_transform_.has_value()) {
      flatland_->flatland()->AddChild(root_transform_id_,
                                      *input_interceptor_transform_);
      child_transforms_.emplace_back(*input_interceptor_transform_);
    }
  }

  // Present the session to Scenic, along with surface acquire/release fences.
  {
    TRACE_EVENT0("flutter", "SessionPresent");

    flatland_->Present();
  }

  // Render the recorded SkPictures into the surfaces.
  {
    TRACE_EVENT0("flutter", "RasterizeSurfaces");

    for (const auto& surface_index : frame_surface_indices) {
      TRACE_EVENT0("flutter", "RasterizeSurface");

      FML_CHECK(surface_index.second < frame_surfaces.size());
      SurfaceProducerSurface* surface =
          frame_surfaces[surface_index.second].get();
      FML_CHECK(surface != nullptr);

      sk_sp<SkSurface> sk_surface = surface->GetSkiaSurface();
      FML_CHECK(sk_surface != nullptr);
      FML_CHECK(SkISize::Make(sk_surface->width(), sk_surface->height()) ==
                frame_size_);
      SkCanvas* canvas = sk_surface->getCanvas();
      FML_CHECK(canvas != nullptr);

      const auto& layer = frame_layers_.find(surface_index.first);
      FML_CHECK(layer != frame_layers_.end());

      canvas->setMatrix(SkMatrix::I());
      canvas->clear(SK_ColorTRANSPARENT);
      canvas->drawPicture(layer->second.picture);
      if (GrDirectContext* direct_context =
              GrAsDirectContext(canvas->recordingContext())) {
        direct_context->flushAndSubmit();
      }
    }
  }

  // Flush deferred Skia work and inform Scenic that render targets are ready.
  {
    TRACE_EVENT0("flutter", "PresentSurfaces");

    surface_producer_->SubmitSurfaces(std::move(frame_surfaces));
  }

  // Submit the underlying render-backend-specific frame for processing.
  frame->Submit();
}

void ExternalViewEmbedder::CreateView(int64_t view_id,
                                      ViewCallback on_view_created,
                                      ViewCreatedCallback on_view_bound) {
  FML_CHECK(views_.find(view_id) == views_.end());

  const auto transform_id = flatland_->NextTransformId();
  const auto viewport_id = flatland_->NextContentId();
  View new_view = {.transform_id = transform_id, .viewport_id = viewport_id};
  flatland_->flatland()->CreateTransform(new_view.transform_id);
  fuchsia::ui::composition::ChildViewWatcherHandle child_view_watcher;
  new_view.pending_create_viewport_callback =
      [this, transform_id, viewport_id, view_id,
       child_view_watcher_request = child_view_watcher.NewRequest()](
          const SkSize& size, const SkRect& inset) mutable {
        fuchsia::ui::composition::ViewportProperties properties;
        properties.set_logical_size({static_cast<uint32_t>(size.fWidth),
                                     static_cast<uint32_t>(size.fHeight)});
        properties.set_inset({static_cast<int32_t>(inset.fTop),
                              static_cast<int32_t>(inset.fRight),
                              static_cast<int32_t>(inset.fBottom),
                              static_cast<int32_t>(inset.fLeft)});
        flatland_->flatland()->CreateViewport(
            viewport_id, {zx::channel((zx_handle_t)view_id)},
            std::move(properties), std::move(child_view_watcher_request));
        flatland_->flatland()->SetContent(transform_id, viewport_id);
      };

  on_view_created();
  on_view_bound(new_view.viewport_id, std::move(child_view_watcher));
  views_.emplace(std::make_pair(view_id, std::move(new_view)));
}

void ExternalViewEmbedder::DestroyView(int64_t view_id,
                                       ViewIdCallback on_view_unbound) {
  auto view = views_.find(view_id);
  FML_CHECK(view != views_.end());

  auto viewport_id = view->second.viewport_id;
  auto transform_id = view->second.transform_id;
  auto& clip_transforms = view->second.clip_transforms;
  if (!view->second.pending_create_viewport_callback) {
    flatland_->flatland()->ReleaseViewport(viewport_id, [](auto) {});
  }
  auto itr = std::find_if(
      child_transforms_.begin(), child_transforms_.end(),
      [transform_id,
       &clip_transforms](fuchsia::ui::composition::TransformId id) {
        return id.value == transform_id.value ||
               (!clip_transforms.empty() &&
                (id.value == clip_transforms[0].transform_id.value));
      });
  if (itr != child_transforms_.end()) {
    flatland_->flatland()->RemoveChild(root_transform_id_, *itr);
    child_transforms_.erase(itr);
  }
  for (auto& clip_transform : clip_transforms) {
    DetachClipTransformChildren(flatland_.get(), &clip_transform);
  }
  flatland_->flatland()->ReleaseTransform(transform_id);
  for (auto& clip_transform : clip_transforms) {
    flatland_->flatland()->ReleaseTransform(clip_transform.transform_id);
  }

  views_.erase(view);
  on_view_unbound(viewport_id);
}

void ExternalViewEmbedder::SetViewProperties(int64_t view_id,
                                             const SkRect& occlusion_hint,
                                             bool hit_testable,
                                             bool focusable) {
  auto found = views_.find(view_id);
  FML_CHECK(found != views_.end());

  // Note that pending_create_viewport_callback might not have run at this
  // point.
  auto& viewport = found->second;
  viewport.pending_occlusion_hint = occlusion_hint;
}

void ExternalViewEmbedder::Reset() {
  frame_layers_.clear();
  frame_composition_order_.clear();
  frame_size_ = SkISize::Make(0, 0);
  frame_dpr_ = 1.f;

  // Clear all children from root.
  for (const auto& transform : child_transforms_) {
    flatland_->flatland()->RemoveChild(root_transform_id_, transform);
  }
  child_transforms_.clear();

  // Clear images on all layers so they aren't cached unnecessarily.
  for (const auto& layer : layers_) {
    flatland_->flatland()->SetContent(layer.transform_id, {0});
  }
}

ExternalViewEmbedder::ViewMutators ExternalViewEmbedder::ParseMutatorStack(
    const flutter::MutatorsStack& mutators_stack) {
  ViewMutators mutators;
  SkMatrix total_transform = SkMatrix::I();
  SkMatrix transform_accumulator = SkMatrix::I();

  for (auto i = mutators_stack.Begin(); i != mutators_stack.End(); ++i) {
    const auto& mutator = *i;
    switch (mutator->GetType()) {
      case flutter::MutatorType::kOpacity: {
        mutators.opacity *= std::clamp(mutator->GetAlphaFloat(), 0.f, 1.f);
      } break;
      case flutter::MutatorType::kTransform: {
        total_transform.preConcat(mutator->GetMatrix());
        transform_accumulator.preConcat(mutator->GetMatrix());
      } break;
      case flutter::MutatorType::kClipRect: {
        mutators.clips.emplace_back(TransformedClip{
            .transform = transform_accumulator,
            .rect = mutator->GetRect(),
        });
        transform_accumulator = SkMatrix::I();
      } break;
      case flutter::MutatorType::kClipRRect: {
        mutators.clips.emplace_back(TransformedClip{
            .transform = transform_accumulator,
            .rect = mutator->GetRRect().getBounds(),
        });
        transform_accumulator = SkMatrix::I();
      } break;
      case flutter::MutatorType::kClipPath: {
        mutators.clips.emplace_back(TransformedClip{
            .transform = transform_accumulator,
            .rect = mutator->GetPath().getBounds(),
        });
        transform_accumulator = SkMatrix::I();
      } break;
      default: {
        break;
      }
    }
  }
  mutators.total_transform = total_transform;
  mutators.transform = transform_accumulator;
  mutators.opacity = std::clamp(mutators.opacity, 0.f, 1.f);

  return mutators;
}

}  // namespace flutter_runner
