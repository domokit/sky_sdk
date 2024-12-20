// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterCompositor.h"

#include "flutter/common/constants.h"
#include "flutter/fml/logging.h"

namespace flutter {

namespace {
std::vector<LayerVariant> CopyLayers(const FlutterLayer** layers, size_t layer_count) {
  std::vector<LayerVariant> layers_copy;
  for (size_t i = 0; i < layer_count; i++) {
    const auto& layer = layers[i];
    if (layer->type == kFlutterLayerContentTypePlatformView) {
      layers_copy.push_back(PlatformViewLayer(layer));
    } else if (layer->type == kFlutterLayerContentTypeBackingStore) {
      std::vector<FlutterRect> rects;
      auto present_info = layer->backing_store_present_info;
      if (present_info != nullptr && present_info->paint_region != nullptr) {
        rects.reserve(present_info->paint_region->rects_count);
        std::copy(present_info->paint_region->rects,
                  present_info->paint_region->rects + present_info->paint_region->rects_count,
                  std::back_inserter(rects));
      }
      layers_copy.push_back(BackingStoreLayer{rects});
    }
  }
  return layers_copy;
}
}  // namespace

FlutterCompositor::FlutterCompositor(id<FlutterViewProvider> view_provider,
                                     FlutterTimeConverter* time_converter,
                                     FlutterPlatformViewController* platform_view_controller)
    : view_provider_(view_provider),
      time_converter_(time_converter),
      platform_view_controller_(platform_view_controller) {
  FML_CHECK(view_provider != nullptr) << "view_provider cannot be nullptr";
}

void FlutterCompositor::AddView(FlutterViewId view_id) {
  dispatch_assert_queue(dispatch_get_main_queue());
  presenters_.try_emplace(view_id);
}

void FlutterCompositor::RemoveView(FlutterViewId view_id) {
  dispatch_assert_queue(dispatch_get_main_queue());
  presenters_.erase(view_id);
}

bool FlutterCompositor::CreateBackingStore(const FlutterBackingStoreConfig* config,
                                           FlutterBackingStore* backing_store_out) {
  FlutterView* view = [view_provider_ viewForIdentifier:config->view_id];
  if (!view) {
    return false;
  }

  CGSize size = CGSizeMake(config->size.width, config->size.height);
  FlutterSurface* surface = [view.surfaceManager surfaceForSize:size];
  memset(backing_store_out, 0, sizeof(FlutterBackingStore));
  backing_store_out->struct_size = sizeof(FlutterBackingStore);
  backing_store_out->type = kFlutterBackingStoreTypeMetal;
  backing_store_out->metal.struct_size = sizeof(FlutterMetalBackingStore);
  backing_store_out->metal.texture = surface.asFlutterMetalTexture;
  return true;
}

bool FlutterCompositor::Present(FlutterViewIdentifier view_id,
                                const FlutterLayer** layers,
                                size_t layers_count) {
  FlutterView* view = [view_provider_ viewForIdentifier:view_id];
  if (!view) {
    return false;
  }

  NSMutableArray* surfaces = [NSMutableArray array];
  for (size_t i = 0; i < layers_count; i++) {
    const FlutterLayer* layer = layers[i];
    if (layer->type == kFlutterLayerContentTypeBackingStore) {
      FlutterSurface* surface =
          [FlutterSurface fromFlutterMetalTexture:&layer->backing_store->metal.texture];

      if (surface) {
        FlutterSurfacePresentInfo* info = [[FlutterSurfacePresentInfo alloc] init];
        info.surface = surface;
        info.offset = CGPointMake(layer->offset.x, layer->offset.y);
        info.zIndex = i;
        FlutterBackingStorePresentInfo* present_info = layer->backing_store_present_info;
        if (present_info != nullptr && present_info->paint_region != nullptr) {
          auto paint_region = present_info->paint_region;
          // Safe because the size of FlutterRect is not expected to change.
          info.paintRegion = std::vector<FlutterRect>(
              paint_region->rects, paint_region->rects + paint_region->rects_count);
        }
        [surfaces addObject:info];
      }
    }
  }

  CFTimeInterval presentation_time = 0;

  if (layers_count > 0 && layers[0]->presentation_time != 0) {
    presentation_time = [time_converter_ engineTimeToCAMediaTime:layers[0]->presentation_time];
  }

  // Notify block below may be called asynchronously, hence the need to copy
  // the layer information instead of passing the original pointers from embedder.
  auto layers_copy = std::make_shared<std::vector<LayerVariant>>(CopyLayers(layers, layers_count));

  [view.surfaceManager presentSurfaces:surfaces
                                atTime:presentation_time
                                notify:^{
                                  // Accessing presenters_ here does not need a
                                  // lock to avoid race condition against
                                  // AddView and RemoveView, since all three
                                  // take place on the platform thread. (The
                                  // macOS API requires platform view presenting
                                  // to take place on the platform thread,
                                  // enforced by `FlutterThreadSynchronizer`.)
                                  dispatch_assert_queue(dispatch_get_main_queue());
                                  auto found_presenter = presenters_.find(view_id);
                                  if (found_presenter != presenters_.end()) {
                                    found_presenter->second.PresentPlatformViews(
                                        view, *layers_copy, platform_view_controller_);
                                  }
                                }];

  return true;
}

size_t FlutterCompositor::DebugNumViews() {
  return presenters_.size();
}

FlutterCompositor::ViewPresenter::ViewPresenter()
    : mutator_views_([NSMapTable strongToStrongObjectsMapTable]) {}

void FlutterCompositor::ViewPresenter::PresentPlatformViews(
    FlutterView* default_base_view,
    const std::vector<LayerVariant>& layers,
    const FlutterPlatformViewController* platform_view_controller) {
  FML_DCHECK([[NSThread currentThread] isMainThread])
      << "Must be on the main thread to present platform views";

  // Active mutator views for this frame.
  NSMutableArray<FlutterMutatorView*>* present_mutators = [NSMutableArray array];

  for (size_t i = 0; i < layers.size(); i++) {
    const auto& layer = layers[i];
    if (!std::holds_alternative<PlatformViewLayer>(layer)) {
      continue;
    }
    const auto& platform_view = std::get<PlatformViewLayer>(layer);
    FlutterMutatorView* mutator_view =
        PresentPlatformView(default_base_view, platform_view, i, platform_view_controller);
    [present_mutators addObject:mutator_view];

    // Gather all overlay regions above this mutator view.
    [mutator_view resetHitTestRegion];
    for (size_t j = i + 1; j < layers.size(); j++) {
      const auto& overlay_layer = layers[j];
      if (!std::holds_alternative<BackingStoreLayer>(overlay_layer)) {
        continue;
      }
      const auto& backing_store_layer = std::get<BackingStoreLayer>(overlay_layer);
      for (const auto& flutter_rect : backing_store_layer.paint_region) {
        double scale = default_base_view.layer.contentsScale;
        CGRect rect = CGRectMake(flutter_rect.left / scale, flutter_rect.top / scale,
                                 (flutter_rect.right - flutter_rect.left) / scale,
                                 (flutter_rect.bottom - flutter_rect.top) / scale);
        CGRect intersection = CGRectIntersection(rect, mutator_view.frame);
        if (!CGRectIsNull(intersection)) {
          intersection.origin.x -= mutator_view.frame.origin.x;
          intersection.origin.y -= mutator_view.frame.origin.y;
          [mutator_view addHitTestIgnoreRegion:intersection];
        }
      }
    }
  }

  NSMutableArray<FlutterMutatorView*>* obsolete_mutators =
      [NSMutableArray arrayWithArray:[mutator_views_ objectEnumerator].allObjects];
  [obsolete_mutators removeObjectsInArray:present_mutators];

  for (FlutterMutatorView* mutator in obsolete_mutators) {
    [mutator_views_ removeObjectForKey:mutator.platformView];
    [mutator removeFromSuperview];
  }

  [platform_view_controller disposePlatformViews];
}

FlutterMutatorView* FlutterCompositor::ViewPresenter::PresentPlatformView(
    FlutterView* default_base_view,
    const PlatformViewLayer& layer,
    size_t index,
    const FlutterPlatformViewController* platform_view_controller) {
  FML_DCHECK([[NSThread currentThread] isMainThread])
      << "Must be on the main thread to present platform views";

  int64_t platform_view_id = layer.identifier();
  NSView* platform_view = [platform_view_controller platformViewWithID:platform_view_id];

  FML_DCHECK(platform_view) << "Platform view not found for id: " << platform_view_id;

  if (cursor_coordinator_ == nil) {
    cursor_coordinator_ = [[FlutterCursorCoordinator alloc] initWithFlutterView:default_base_view];
  }

  FlutterMutatorView* container = [mutator_views_ objectForKey:platform_view];

  if (!container) {
    container = [[FlutterMutatorView alloc] initWithPlatformView:platform_view
                                                cursorCoordiator:cursor_coordinator_];
    [mutator_views_ setObject:container forKey:platform_view];
    [default_base_view addSubview:container];
  }

  container.layer.zPosition = index;
  [container applyFlutterLayer:&layer];

  return container;
}

}  // namespace flutter
