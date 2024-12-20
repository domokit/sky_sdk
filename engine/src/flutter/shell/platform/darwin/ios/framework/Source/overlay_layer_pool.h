// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_OVERLAY_LAYER_POOL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_OVERLAY_LAYER_POOL_H_

#include <Metal/Metal.h>
#include <memory>

#import <UIKit/UIKit.h>

#include "flow/surface.h"

#import "flutter/shell/platform/darwin/ios/ios_context.h"

namespace flutter {

class IOSSurface;

/// @brief State holder for a Flutter overlay layer.
struct OverlayLayer {
  OverlayLayer(UIView* overlay_view,
               UIView* overlay_view_wrapper,
               std::unique_ptr<IOSSurface> ios_surface,
               std::unique_ptr<Surface> surface);

  ~OverlayLayer() = default;

  UIView* overlay_view;
  UIView* overlay_view_wrapper;
  std::unique_ptr<IOSSurface> ios_surface;
  std::unique_ptr<Surface> surface;

  // Whether a frame for this layer was submitted.
  bool did_submit_last_frame;

  // The GrContext that is currently used by the overlay surfaces.
  // We track this to know when the GrContext for the Flutter app has changed
  // so we can update the overlay with the new context.
  GrDirectContext* gr_context;

  void UpdateViewState(UIView* flutter_view, SkRect rect, int64_t view_id, int64_t overlay_id);
};

/// @brief Storage for Overlay layers across frames.
///
/// Note: this class does not synchronize access to its layers or any layer removal. As it
/// is currently used, layers must be created on the platform thread but other methods of
/// it are called on the raster thread. This is safe as overlay layers are only ever added
/// while the raster thread is latched.
class OverlayLayerPool {
 public:
  OverlayLayerPool() = default;

  ~OverlayLayerPool() = default;

  /// @brief Gets a layer from the pool if available.
  ///
  /// The layer is marked as used until [RecycleLayers] is called.
  std::shared_ptr<OverlayLayer> GetNextLayer();

  /// @brief Create a new overlay layer.
  ///
  /// This method can only be called on the Platform thread.
  void CreateLayer(GrDirectContext* gr_context,
                   const std::shared_ptr<IOSContext>& ios_context,
                   MTLPixelFormat pixel_format);

  /// @brief Removes unused layers from the pool. Returns the unused layers.
  std::vector<std::shared_ptr<OverlayLayer>> RemoveUnusedLayers();

  /// @brief Marks the layers in the pool as available for reuse.
  void RecycleLayers();

  /// @brief The count of layers currently in the pool.
  size_t size() const;

 private:
  OverlayLayerPool(const OverlayLayerPool&) = delete;
  OverlayLayerPool& operator=(const OverlayLayerPool&) = delete;

  // The index of the entry in the layers_ vector that determines the beginning of the unused
  // layers. For example, consider the following vector:
  //  _____
  //  | 0 |
  /// |---|
  /// | 1 | <-- available_layer_index_
  /// |---|
  /// | 2 |
  /// |---|
  ///
  /// This indicates that entries starting from 1 can be reused meanwhile the entry at position 0
  /// cannot be reused.
  size_t available_layer_index_ = 0;
  std::vector<std::shared_ptr<OverlayLayer>> layers_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_OVERLAY_LAYER_POOL_H_
