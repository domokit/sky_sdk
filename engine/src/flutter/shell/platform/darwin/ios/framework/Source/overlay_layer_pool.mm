// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/darwin/ios/framework/Source/overlay_layer_pool.h"

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterOverlayView.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"

namespace flutter {

OverlayLayer::OverlayLayer(UIView* overlay_view,
                           UIView* overlay_view_wrapper,
                           std::unique_ptr<IOSSurface> ios_surface,
                           std::unique_ptr<Surface> surface)
    : overlay_view(overlay_view),
      overlay_view_wrapper(overlay_view_wrapper),
      ios_surface(std::move(ios_surface)),
      surface(std::move(surface)){};

void OverlayLayer::UpdateViewState(UIView* flutter_view,
                                   SkRect rect,
                                   int64_t view_id,
                                   int64_t overlay_id) {
  auto screenScale = [UIScreen mainScreen].scale;
  // Set the size of the overlay view wrapper.
  // This wrapper view masks the overlay view.
  overlay_view_wrapper.frame = CGRectMake(rect.x() / screenScale, rect.y() / screenScale,
                                          rect.width() / screenScale, rect.height() / screenScale);
  // Set a unique view identifier, so the overlay_view_wrapper can be identified in XCUITests.
  overlay_view_wrapper.accessibilityIdentifier =
      [NSString stringWithFormat:@"platform_view[%lld].overlay[%lld]", view_id, overlay_id];

  // Set the size of the overlay view.
  // This size is equal to the device screen size.
  overlay_view.frame = [flutter_view convertRect:flutter_view.bounds toView:overlay_view_wrapper];
  // Set a unique view identifier, so the overlay_view can be identified in XCUITests.
  overlay_view.accessibilityIdentifier =
      [NSString stringWithFormat:@"platform_view[%lld].overlay_view[%lld]", view_id, overlay_id];
}

// OverlayLayerPool
////////////////////////////////////////////////////////

std::shared_ptr<OverlayLayer> OverlayLayerPool::GetNextLayer() {
  std::shared_ptr<OverlayLayer> result;
  if (available_layer_index_ < layers_.size()) {
    result = layers_[available_layer_index_];
    available_layer_index_++;
  }

  return result;
}

void OverlayLayerPool::CreateLayer(GrDirectContext* gr_context,
                                   const std::shared_ptr<IOSContext>& ios_context,
                                   MTLPixelFormat pixel_format) {
  FML_DCHECK([[NSThread currentThread] isMainThread]);
  std::shared_ptr<OverlayLayer> layer;
  UIView* overlay_view;
  UIView* overlay_view_wrapper;

  bool impeller_enabled = !!ios_context->GetImpellerContext();
  if (!gr_context && !impeller_enabled) {
    overlay_view = [[FlutterOverlayView alloc] init];
    overlay_view_wrapper = [[FlutterOverlayView alloc] init];

    CALayer* ca_layer = overlay_view.layer;
    std::unique_ptr<IOSSurface> ios_surface = IOSSurface::Create(ios_context, ca_layer);
    std::unique_ptr<Surface> surface = ios_surface->CreateGPUSurface();

    layer = std::make_shared<OverlayLayer>(overlay_view, overlay_view_wrapper,
                                           std::move(ios_surface), std::move(surface));
  } else {
    CGFloat screenScale = [UIScreen mainScreen].scale;
    overlay_view = [[FlutterOverlayView alloc] initWithContentsScale:screenScale
                                                         pixelFormat:pixel_format];
    overlay_view_wrapper = [[FlutterOverlayView alloc] initWithContentsScale:screenScale
                                                                 pixelFormat:pixel_format];

    CALayer* ca_layer = overlay_view.layer;
    std::unique_ptr<IOSSurface> ios_surface = IOSSurface::Create(ios_context, ca_layer);
    std::unique_ptr<Surface> surface = ios_surface->CreateGPUSurface(gr_context);

    layer = std::make_shared<OverlayLayer>(overlay_view, overlay_view_wrapper,
                                           std::move(ios_surface), std::move(surface));
    layer->gr_context = gr_context;
  }
  // The overlay view wrapper masks the overlay view.
  // This is required to keep the backing surface size unchanged between frames.
  //
  // Otherwise, changing the size of the overlay would require a new surface,
  // which can be very expensive.
  //
  // This is the case of an animation in which the overlay size is changing in every frame.
  //
  // +------------------------+
  // |   overlay_view         |
  // |    +--------------+    |              +--------------+
  // |    |    wrapper   |    |  == mask =>  | overlay_view |
  // |    +--------------+    |              +--------------+
  // +------------------------+
  layer->overlay_view_wrapper.clipsToBounds = YES;
  [layer->overlay_view_wrapper addSubview:layer->overlay_view];

  layers_.push_back(layer);
}

void OverlayLayerPool::RecycleLayers() {
  available_layer_index_ = 0;
}

std::vector<std::shared_ptr<OverlayLayer>> OverlayLayerPool::RemoveUnusedLayers() {
  std::vector<std::shared_ptr<OverlayLayer>> results;
  for (size_t i = available_layer_index_; i < layers_.size(); i++) {
    results.push_back(layers_[i]);
  }
  // Leave at least one overlay layer, to work around cases where scrolling
  // platform views under an app bar continually adds and removes an
  // overlay layer. This logic could be removed if https://github.com/flutter/flutter/issues/150646
  // is fixed.
  static constexpr size_t kLeakLayerCount = 1;
  size_t erase_offset = std::max(available_layer_index_, kLeakLayerCount);
  if (erase_offset < layers_.size()) {
    layers_.erase(layers_.begin() + erase_offset, layers_.end());
  }
  return results;
}

size_t OverlayLayerPool::size() const {
  return layers_.size();
}

}  // namespace flutter
