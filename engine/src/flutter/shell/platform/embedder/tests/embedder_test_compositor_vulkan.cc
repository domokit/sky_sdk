// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_compositor_vulkan.h"

#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer_vulkan.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

namespace flutter {
namespace testing {

EmbedderTestCompositorVulkan::EmbedderTestCompositorVulkan(
    SkISize surface_size,
    sk_sp<GrDirectContext> context)
    : EmbedderTestCompositor(surface_size, std::move(context)) {}

EmbedderTestCompositorVulkan::~EmbedderTestCompositorVulkan() = default;

void EmbedderTestCompositorVulkan::SetRenderTargetType(
    EmbedderTestBackingStoreProducer::RenderTargetType type,
    FlutterSoftwarePixelFormat software_pixfmt) {
  switch (type) {
    case EmbedderTestBackingStoreProducer::RenderTargetType::kVulkanImage:
      backingstore_producer_ =
          std::make_unique<EmbedderTestBackingStoreProducerVulkan>(context_,
                                                                   type);
      return;
    case EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture:
    case EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer:
    case EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLSurface:
    case EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture:
    case EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer:
    case EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer2:
      FML_LOG(FATAL) << "Unsupported render target type: "
                     << static_cast<int>(type);
      return;
  }
}

bool EmbedderTestCompositorVulkan::UpdateOffscrenComposition(
    const FlutterLayer** layers,
    size_t layers_count) {
  last_composition_ = nullptr;

  const auto image_info = SkImageInfo::MakeN32Premul(surface_size_);

  sk_sp<SkSurface> surface =
      SkSurfaces::RenderTarget(context_.get(),            // context
                               skgpu::Budgeted::kNo,      // budgeted
                               image_info,                // image info
                               1,                         // sample count
                               kTopLeft_GrSurfaceOrigin,  // surface origin
                               nullptr,                   // surface properties
                               false                      // create mipmaps
      );

  if (!surface) {
    FML_LOG(ERROR) << "Could not update the off-screen composition.";
    return false;
  }

  auto canvas = surface->getCanvas();

  // This has to be transparent because we are going to be compositing this
  // sub-hierarchy onto the on-screen surface.
  canvas->clear(SK_ColorTRANSPARENT);

  for (size_t i = 0; i < layers_count; ++i) {
    const auto* layer = layers[i];

    sk_sp<SkImage> platform_rendered_contents;

    sk_sp<SkImage> layer_image;
    SkIPoint canvas_offset = SkIPoint::Make(0, 0);

    switch (layer->type) {
      case kFlutterLayerContentTypeBackingStore:
        layer_image =
            backingstore_producer_->MakeImageSnapshot(layer->backing_store);
        break;
      case kFlutterLayerContentTypePlatformView:
        layer_image =
            platform_view_renderer_callback_
                ? platform_view_renderer_callback_(*layer, context_.get())
                : nullptr;
        canvas_offset = SkIPoint::Make(layer->offset.x, layer->offset.y);
        break;
    };

    // If the layer is not a platform view but the engine did not specify an
    // image for the backing store, it is an error.
    if (!layer_image && layer->type != kFlutterLayerContentTypePlatformView) {
      FML_LOG(ERROR) << "Could not snapshot layer in test compositor: "
                     << *layer;
      return false;
    }

    // The test could have just specified no contents to be rendered in place of
    // a platform view. This is not an error.
    if (layer_image) {
      // The image rendered by Flutter already has the correct offset and
      // transformation applied. The layers offset is meant for the platform.
      canvas->drawImage(layer_image.get(), canvas_offset.x(),
                        canvas_offset.y());
    }
  }

  last_composition_ = surface->makeImageSnapshot();

  if (!last_composition_) {
    FML_LOG(ERROR) << "Could not update the contents of the sub-composition.";
    return false;
  }

  if (next_scene_callback_) {
    auto last_composition_snapshot = last_composition_->makeRasterImage();
    FML_CHECK(last_composition_snapshot);
    auto callback = next_scene_callback_;
    next_scene_callback_ = nullptr;
    callback(std::move(last_composition_snapshot));
  }

  return true;
}

}  // namespace testing
}  // namespace flutter
