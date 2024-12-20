// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_EXTERNAL_VIEW_EMBEDDER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_EXTERNAL_VIEW_EMBEDDER_H_

#include <fuchsia/ui/composition/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>
#include <lib/fit/function.h>

#include <cstdint>  // For uint32_t & uint64_t
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "flutter/flow/embedded_views.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/fuchsia/flutter/canvas_spy.h"
#include "flutter/shell/platform/fuchsia/flutter/rtree.h"
#include "third_party/skia/include/core/SkPictureRecorder.h"
#include "third_party/skia/include/core/SkPoint.h"
#include "third_party/skia/include/core/SkRect.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

#include "flatland_connection.h"
#include "surface_producer.h"

namespace flutter_runner {

using ViewCallback = std::function<void()>;
using ViewCreatedCallback = std::function<void(
    fuchsia::ui::composition::ContentId,
    fuchsia::ui::composition::ChildViewWatcherHandle child_view_watcher)>;
using ViewIdCallback = std::function<void(fuchsia::ui::composition::ContentId)>;

// This class orchestrates interaction with the Scenic's compositor on
// Fuchsia. It ensures that flutter content and platform view content are both
// rendered correctly in a unified scene.
class ExternalViewEmbedder final : public flutter::ExternalViewEmbedder {
 public:
  constexpr static uint32_t kDefaultViewportSize = 32;

  ExternalViewEmbedder(
      fuchsia::ui::views::ViewCreationToken view_creation_token,
      fuchsia::ui::views::ViewIdentityOnCreation view_identity,
      fuchsia::ui::composition::ViewBoundProtocols endpoints,
      fidl::InterfaceRequest<fuchsia::ui::composition::ParentViewportWatcher>
          parent_viewport_watcher_request,
      std::shared_ptr<FlatlandConnection> flatland,
      std::shared_ptr<SurfaceProducer> surface_producer,
      bool intercept_all_input = false);
  ~ExternalViewEmbedder();

  // |ExternalViewEmbedder|
  flutter::DlCanvas* GetRootCanvas() override;

  // |ExternalViewEmbedder|
  void PrerollCompositeEmbeddedView(
      int64_t view_id,
      std::unique_ptr<flutter::EmbeddedViewParams> params) override;

  // |ExternalViewEmbedder|
  flutter::DlCanvas* CompositeEmbeddedView(int64_t view_id) override;

  // |ExternalViewEmbedder|
  flutter::PostPrerollResult PostPrerollAction(
      const fml::RefPtr<fml::RasterThreadMerger>& raster_thread_merger)
      override;

  // |ExternalViewEmbedder|
  void BeginFrame(GrDirectContext* context,
                  const fml::RefPtr<fml::RasterThreadMerger>&
                      raster_thread_merger) override;

  // |ExternalViewEmbedder|
  void PrepareFlutterView(SkISize frame_size,
                          double device_pixel_ratio) override;

  // |ExternalViewEmbedder|
  void EndFrame(bool should_resubmit_frame,
                const fml::RefPtr<fml::RasterThreadMerger>&
                    raster_thread_merger) override;

  // |ExternalViewEmbedder|
  void SubmitFlutterView(
      int64_t flutter_view_id,
      GrDirectContext* context,
      const std::shared_ptr<impeller::AiksContext>& aiks_context,
      std::unique_ptr<flutter::SurfaceFrame> frame) override;

  // |ExternalViewEmbedder|
  void CancelFrame() override { Reset(); }

  // |ExternalViewEmbedder|
  bool SupportsDynamicThreadMerging() override { return false; }

  // View manipulation.
  // |SetViewProperties| doesn't manipulate the view directly -- it sets pending
  // properties for the next |UpdateView| call.
  void CreateView(int64_t view_id,
                  ViewCallback on_view_created,
                  ViewCreatedCallback on_view_bound);
  void DestroyView(int64_t view_id, ViewIdCallback on_view_unbound);
  void SetViewProperties(int64_t view_id,
                         const SkRect& occlusion_hint,
                         bool hit_testable,
                         bool focusable);

  // Holds the clip transform that may be applied on a View.
  struct ClipTransform {
    fuchsia::ui::composition::TransformId transform_id;
    std::vector<fuchsia::ui::composition::TransformId> children;
  };

 private:
  void Reset();  // Reset state for a new frame.

  // This struct represents a transformed clip rect.
  struct TransformedClip {
    SkMatrix transform = SkMatrix::I();
    SkRect rect = SkRect::MakeEmpty();

    bool operator==(const TransformedClip& other) const {
      return transform == other.transform && rect == other.rect;
    }
  };

  // This struct represents all the mutators that can be applied to a
  // PlatformView, unpacked from the `MutatorStack`.
  struct ViewMutators {
    std::vector<TransformedClip> clips;
    SkMatrix total_transform = SkMatrix::I();
    SkMatrix transform = SkMatrix::I();
    SkScalar opacity = 1.f;

    bool operator==(const ViewMutators& other) const {
      return clips == other.clips && total_transform == other.total_transform &&
             transform == other.transform && opacity == other.opacity;
    }
  };

  ViewMutators ParseMutatorStack(const flutter::MutatorsStack& mutators_stack);

  struct EmbedderLayer {
    EmbedderLayer(const SkISize& frame_size,
                  std::optional<flutter::EmbeddedViewParams> view_params,
                  flutter::RTreeFactory rtree_factory)
        : rtree(rtree_factory.getInstance()),
          embedded_view_params(std::move(view_params)),
          recorder(std::make_unique<SkPictureRecorder>()),
          canvas_spy(std::make_unique<flutter::CanvasSpy>(
              recorder->beginRecording(SkRect::Make(frame_size),
                                       &rtree_factory))),
          surface_size(frame_size),
          picture(nullptr) {}

    // Records paint operations applied to this layer's `SkCanvas`.
    // These records are used to determine which portions of this layer
    // contain content. The embedder propagates this information to scenic, so
    // that scenic can accurately decide which portions of this layer may
    // interact with input.
    sk_sp<flutter::RTree> rtree;

    std::optional<flutter::EmbeddedViewParams> embedded_view_params;
    std::unique_ptr<SkPictureRecorder> recorder;
    // TODO(cyanglaz: use DlOpSpy instead.
    // https://github.com/flutter/flutter/issues/123805
    std::unique_ptr<flutter::CanvasSpy> canvas_spy;
    SkISize surface_size;
    sk_sp<SkPicture> picture;
  };
  using EmbedderLayerId = std::optional<uint32_t>;
  constexpr static EmbedderLayerId kRootLayerId = EmbedderLayerId{};

  struct View {
    std::vector<ClipTransform> clip_transforms;
    fuchsia::ui::composition::TransformId transform_id;
    fuchsia::ui::composition::ContentId viewport_id;
    ViewMutators mutators;
    SkSize size = SkSize::MakeEmpty();
    SkRect pending_occlusion_hint = SkRect::MakeEmpty();
    SkRect occlusion_hint = SkRect::MakeEmpty();
    fit::callback<void(const SkSize&, const SkRect&)>
        pending_create_viewport_callback;
  };

  struct Layer {
    // Transform on which Images are set.
    fuchsia::ui::composition::TransformId transform_id;
  };

  std::shared_ptr<FlatlandConnection> flatland_;
  std::shared_ptr<SurfaceProducer> surface_producer_;

  fuchsia::ui::composition::ParentViewportWatcherPtr parent_viewport_watcher_;

  fuchsia::ui::composition::TransformId root_transform_id_;

  std::unordered_map<int64_t, View> views_;
  std::vector<Layer> layers_;

  std::unordered_map<EmbedderLayerId, EmbedderLayer> frame_layers_;
  std::vector<EmbedderLayerId> frame_composition_order_;
  std::vector<fuchsia::ui::composition::TransformId> child_transforms_;
  SkISize frame_size_ = SkISize::Make(0, 0);
  float frame_dpr_ = 1.f;

  // TransformId for the input interceptor node when input shield is turned on,
  // std::nullptr otherwise.
  std::optional<fuchsia::ui::composition::TransformId>
      input_interceptor_transform_;

  FML_DISALLOW_COPY_AND_ASSIGN(ExternalViewEmbedder);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_EXTERNAL_VIEW_EMBEDDER_H_
