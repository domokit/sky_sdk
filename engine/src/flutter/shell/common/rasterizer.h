// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_RASTERIZER_H_
#define FLUTTER_SHELL_COMMON_RASTERIZER_H_

#include <memory>
#include <optional>
#include <unordered_map>

#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/flow/compositor_context.h"
#include "flutter/flow/embedded_views.h"
#include "flutter/flow/frame_timings.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/closure.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/raster_thread_merger.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"
#if IMPELLER_SUPPORTS_RENDERING
#include "impeller/core/formats.h"               // nogncheck
#include "impeller/display_list/aiks_context.h"  // nogncheck
#include "impeller/renderer/context.h"           // nogncheck
#include "impeller/typographer/backends/skia/typographer_context_skia.h"  // nogncheck
#endif  // IMPELLER_SUPPORTS_RENDERING
#include "flutter/lib/ui/snapshot_delegate.h"
#include "flutter/shell/common/pipeline.h"
#include "flutter/shell/common/snapshot_controller.h"
#include "flutter/shell/common/snapshot_surface_producer.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkRect.h"
#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

#if !IMPELLER_SUPPORTS_RENDERING
namespace impeller {
class Context;
class AiksContext;
}  // namespace impeller
#endif  // !IMPELLER_SUPPORTS_RENDERING

namespace flutter {

// The result status of Rasterizer::Draw. This is only used for unit tests.
enum class DrawStatus {
  // The drawing was done without any specified status.
  kDone,
  // Failed to rasterize the frame because the Rasterizer is not set up.
  kNotSetUp,
  // Nothing was done, because the call was not on the raster thread. Yielded to
  // let this frame be serviced on the right thread.
  kYielded,
  // Nothing was done, because the pipeline was empty.
  kPipelineEmpty,
  // Nothing was done, because the GPU was unavailable.
  kGpuUnavailable,
};

// The result status of drawing to a view. This is only used for unit tests.
enum class DrawSurfaceStatus {
  // The layer tree was successfully rasterized.
  kSuccess,
  // The layer tree must be submitted again.
  //
  // This can occur on Android when switching the background surface to
  // FlutterImageView.  On Android, the first frame doesn't make the image
  // available to the ImageReader right away. The second frame does.
  // TODO(egarciad): https://github.com/flutter/flutter/issues/65652
  //
  // This can also occur when the frame is dropped to wait for the thread
  // merger to merge the raster and platform threads.
  kRetry,
  // Failed to rasterize the frame.
  kFailed,
  // Layer tree was discarded because its size does not match the view size.
  // This typically occurs during resizing.
  kDiscarded,
};

// The information to draw to all views of a frame.
struct FrameItem {
  FrameItem(std::vector<std::unique_ptr<LayerTreeTask>> tasks,
            std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder)
      : layer_tree_tasks(std::move(tasks)),
        frame_timings_recorder(std::move(frame_timings_recorder)) {}
  std::vector<std::unique_ptr<LayerTreeTask>> layer_tree_tasks;
  std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder;
};

using FramePipeline = Pipeline<FrameItem>;

//------------------------------------------------------------------------------
/// The rasterizer is a component owned by the shell that resides on the raster
/// task runner. Each shell owns exactly one instance of a rasterizer. The
/// rasterizer may only be created, used and collected on the raster task
/// runner.
///
/// The rasterizer owns the instance of the currently active on-screen render
/// surface. On this surface, it renders the contents of layer trees submitted
/// to it by the `Engine` (which lives on the UI task runner).
///
/// The primary components owned by the rasterizer are the compositor context
/// and the on-screen render surface. The compositor context has all the GPU
/// state necessary to render frames to the render surface.
///
class Rasterizer final : public SnapshotDelegate,
                         public Stopwatch::RefreshRateUpdater,
                         public SnapshotController::Delegate {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Used to forward events from the rasterizer to interested
  ///             subsystems. Currently, the shell sets itself up as the
  ///             rasterizer delegate to listen for frame rasterization events.
  ///             It can then forward these events to the engine.
  ///
  ///             Like all rasterizer operation, the rasterizer delegate call
  ///             are made on the raster task runner. Any delegate must ensure
  ///             that they can handle the threading implications.
  ///
  class Delegate {
   public:
    //--------------------------------------------------------------------------
    /// @brief      Notifies the delegate that a frame has been rendered. The
    ///             rasterizer collects profiling information for each part of
    ///             the frame workload. This profiling information is made
    ///             available to the delegate for forwarding to subsystems
    ///             interested in collecting such profiles. Currently, the shell
    ///             (the delegate) forwards this to the engine where Dart code
    ///             can react to this information.
    ///
    /// @see        `FrameTiming`
    ///
    /// @param[in]  frame_timing  Instrumentation information for each phase of
    ///                           the frame workload.
    ///
    virtual void OnFrameRasterized(const FrameTiming& frame_timing) = 0;

    /// Time limit for a smooth frame.
    ///
    /// See: `DisplayManager::GetMainDisplayRefreshRate`.
    virtual fml::Milliseconds GetFrameBudget() = 0;

    /// Target time for the latest frame. See also `Shell::OnAnimatorBeginFrame`
    /// for when this time gets updated.
    virtual fml::TimePoint GetLatestFrameTargetTime() const = 0;

    /// Task runners used by the shell.
    virtual const TaskRunners& GetTaskRunners() const = 0;

    /// The raster thread merger from parent shell's rasterizer.
    virtual const fml::RefPtr<fml::RasterThreadMerger>
    GetParentRasterThreadMerger() const = 0;

    /// Accessor for the shell's GPU sync switch, which determines whether GPU
    /// operations are allowed on the current thread.
    ///
    /// For example, on some platforms when the application is backgrounded it
    /// is critical that GPU operations are not processed.
    virtual std::shared_ptr<const fml::SyncSwitch> GetIsGpuDisabledSyncSwitch()
        const = 0;

    virtual const Settings& GetSettings() const = 0;

    virtual bool ShouldDiscardLayerTree(int64_t view_id,
                                        const flutter::LayerTree& tree) = 0;
  };

  //----------------------------------------------------------------------------
  /// @brief     How to handle calls to MakeSkiaGpuImage.
  enum class MakeGpuImageBehavior {
    /// MakeSkiaGpuImage returns a GPU resident image, if possible.
    kGpu,
    /// MakeSkiaGpuImage returns a checkerboard bitmap. This is useful in test
    /// contexts where no GPU surface is available.
    kBitmap,
  };

  //----------------------------------------------------------------------------
  /// @brief      Creates a new instance of a rasterizer. Rasterizers may only
  ///             be created on the raster task runner. Rasterizers are
  ///             currently only created by the shell (which also sets itself up
  ///             as the rasterizer delegate).
  ///
  /// @param[in]  delegate                   The rasterizer delegate.
  /// @param[in]  gpu_image_behavior         How to handle calls to
  ///                                        MakeSkiaGpuImage.
  ///
  explicit Rasterizer(
      Delegate& delegate,
      MakeGpuImageBehavior gpu_image_behavior = MakeGpuImageBehavior::kGpu);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the rasterizer. This must happen on the raster task
  ///             runner. All GPU resources are collected before this call
  ///             returns. Any context set up by the embedder to hold these
  ///             resources can be immediately collected as well.
  ///
  ~Rasterizer();

  void SetImpellerContext(std::weak_ptr<impeller::Context> impeller_context);

  //----------------------------------------------------------------------------
  /// @brief      Rasterizers may be created well before an on-screen surface is
  ///             available for rendering. Shells usually create a rasterizer in
  ///             their constructors. Once an on-screen surface is available
  ///             however, one may be provided to the rasterizer using this
  ///             call. No rendering may occur before this call. The surface is
  ///             held till the balancing call to `Rasterizer::Teardown` is
  ///             made. Calling a setup before tearing down the previous surface
  ///             (if this is not the first time the surface has been set up) is
  ///             user error.
  ///
  /// @see        `Rasterizer::Teardown`
  ///
  /// @param[in]  surface  The on-screen render surface.
  ///
  void Setup(std::unique_ptr<Surface> surface);

  //----------------------------------------------------------------------------
  /// @brief      Releases the previously set up on-screen render surface and
  ///             collects associated resources. No more rendering may occur
  ///             till the next call to `Rasterizer::Setup` with a new render
  ///             surface. Calling a teardown without a setup is user error.
  ///             Calling this method multiple times is safe.
  ///
  void Teardown();

  //----------------------------------------------------------------------------
  /// @brief      Releases any resource used by the external view embedder.
  ///             For example, overlay surfaces or Android views.
  ///             On Android, this method post a task to the platform thread,
  ///             and waits until it completes.
  void TeardownExternalViewEmbedder();

  //----------------------------------------------------------------------------
  /// @brief      Notifies the rasterizer that there is a low memory situation
  ///             and it must purge as many unnecessary resources as possible.
  ///             Currently, the Skia context associated with onscreen rendering
  ///             is told to free GPU resources.
  ///
  void NotifyLowMemoryWarning() const;

  //----------------------------------------------------------------------------
  /// @brief      Gets a weak pointer to the rasterizer. The rasterizer may only
  ///             be accessed on the raster task runner.
  ///
  /// @return     The weak pointer to the rasterizer.
  ///
  fml::TaskRunnerAffineWeakPtr<Rasterizer> GetWeakPtr() const;

  fml::TaskRunnerAffineWeakPtr<SnapshotDelegate> GetSnapshotDelegate() const;

  //----------------------------------------------------------------------------
  /// @brief      Deallocate the resources for displaying a view.
  ///
  ///             This method must be called on the raster task runner when a
  ///             view is removed from the engine.
  ///
  ///             When the rasterizer is requested to draw an unrecognized view,
  ///             it implicitly allocates necessary resources. These resources
  ///             must be explicitly deallocated.
  ///
  /// @param[in]  view_id  The ID of the view.
  ///
  void CollectView(int64_t view_id);

  //----------------------------------------------------------------------------
  /// @brief      Returns the last successfully drawn layer tree for the given
  ///             view, or nullptr if there isn't any. This is useful during
  ///             `DrawLastLayerTrees` and computing frame damage.
  ///
  /// @bug        https://github.com/flutter/flutter/issues/33939
  ///
  /// @return     A pointer to the last layer or `nullptr` if this rasterizer
  ///             has never rendered a frame to the given view.
  ///
  flutter::LayerTree* GetLastLayerTree(int64_t view_id);

  //----------------------------------------------------------------------------
  /// @brief      Draws the last layer trees with their last configuration. This
  ///             may seem entirely redundant at first glance. After all, on
  ///             surface loss and re-acquisition, the framework generates a new
  ///             layer tree. Otherwise, why render the same contents to the
  ///             screen again? This is used as an optimization in cases where
  ///             there are external textures (video or camera streams for
  ///             example) in referenced in the layer tree. These textures may
  ///             be updated at a cadence different from that of the Flutter
  ///             application. Flutter can re-render the layer tree with just
  ///             the updated textures instead of waiting for the framework to
  ///             do the work to generate the layer tree describing the same
  ///             contents.
  ///
  ///             Calling this method clears all last layer trees
  ///             (GetLastLayerTree).
  ///
  void DrawLastLayerTrees(
      std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder);

  // |SnapshotDelegate|
  GrDirectContext* GetGrContext() override;

  std::shared_ptr<flutter::TextureRegistry> GetTextureRegistry() override;

  //----------------------------------------------------------------------------
  /// @brief      Takes the next item from the layer tree pipeline and executes
  ///             the raster thread frame workload for that pipeline item to
  ///             render a frame on the on-screen surface.
  ///
  ///             Why does the draw call take a layer tree pipeline and not the
  ///             layer tree directly?
  ///
  ///             The pipeline is the way book-keeping of frame workloads
  ///             distributed across the multiple threads is managed. The
  ///             rasterizer deals with the pipelines directly (instead of layer
  ///             trees which is what it actually renders) because the pipeline
  ///             consumer's workload must be accounted for within the pipeline
  ///             itself. If the rasterizer took the layer tree directly, it
  ///             would have to be taken out of the pipeline. That would signal
  ///             the end of the frame workload and the pipeline would be ready
  ///             for new frames. But the last frame has not been rendered by
  ///             the frame yet! On the other hand, the pipeline must own the
  ///             layer tree it renders because it keeps a reference to the last
  ///             layer tree around till a new frame is rendered. So a simple
  ///             reference wont work either. The `Rasterizer::DoDraw` method
  ///             actually performs the GPU operations within the layer tree
  ///             pipeline.
  ///
  /// @see        `Rasterizer::DoDraw`
  ///
  /// @param[in]  pipeline  The layer tree pipeline to take the next layer tree
  ///                       to render from.
  ///
  DrawStatus Draw(const std::shared_ptr<FramePipeline>& pipeline);

  //----------------------------------------------------------------------------
  /// @brief      The type of the screenshot to obtain of the previously
  ///             rendered layer tree.
  ///
  enum class ScreenshotType {
    // NOLINTBEGIN(readability-identifier-naming)
    //--------------------------------------------------------------------------
    /// A format used to denote a Skia picture. A Skia picture is a serialized
    /// representation of an `SkPicture` that can be used to introspect the
    /// series of commands used to draw that picture.
    ///
    /// Skia pictures are typically stored as files with the .skp extension on
    /// disk. These files may be viewed in an interactive debugger available at
    /// https://debugger.skia.org/
    ///
    SkiaPicture,

    //--------------------------------------------------------------------------
    /// A format used to denote uncompressed image data. For Skia, this format
    /// is 32 bits per pixel, 8 bits per component and
    /// denoted by the `kN32_SkColorType ` Skia color type. For Impeller, its
    /// format is specified in Screenshot::pixel_format.
    ///
    UncompressedImage,

    //--------------------------------------------------------------------------
    /// A format used to denote compressed image data. The PNG compressed
    /// container is used.
    ///
    CompressedImage,

    //--------------------------------------------------------------------------
    /// Reads the data directly from the Rasterizer's surface. The pixel format
    /// is determined from the surface. This is the only way to read wide gamut
    /// color data, but isn't supported everywhere.
    SurfaceData,
    // NOLINTEND(readability-identifier-naming)
  };

  // Specifies the format of pixel data in a Screenshot.
  enum class ScreenshotFormat {
    // Unknown format, or Skia default.
    kUnknown,
    // RGBA 8 bits per channel.
    kR8G8B8A8UNormInt,
    // BGRA 8 bits per channel.
    kB8G8R8A8UNormInt,
    // RGBA 16 bit floating point per channel.
    kR16G16B16A16Float,
  };

  //----------------------------------------------------------------------------
  /// @brief      A POD type used to return the screenshot data along with the
  ///             size of the frame.
  ///
  struct Screenshot {
    //--------------------------------------------------------------------------
    /// The data used to describe the screenshot. The data format depends on the
    /// type of screenshot taken and any further encoding done to the same.
    ///
    /// @see      `ScreenshotType`
    ///
    sk_sp<SkData> data;

    //--------------------------------------------------------------------------
    /// The size of the screenshot in texels.
    ///
    SkISize frame_size = SkISize::MakeEmpty();

    //--------------------------------------------------------------------------
    /// Characterization of the format of the data in `data`.
    ///
    std::string format;

    //--------------------------------------------------------------------------
    /// The pixel format of the data in `data`.
    ///
    /// If the impeller backend is not used, this value is always kUnknown and
    /// the data is in RGBA8888 format.
    ScreenshotFormat pixel_format = ScreenshotFormat::kUnknown;

    //--------------------------------------------------------------------------
    /// @brief      Creates an empty screenshot
    ///
    Screenshot();

    //--------------------------------------------------------------------------
    /// @brief      Creates a screenshot with the specified data and size.
    ///
    /// @param[in]  p_data  The screenshot data
    /// @param[in]  p_size  The screenshot size.
    /// @param[in]  p_format  The screenshot format.
    /// @param[in]  p_pixel_format  The screenshot format.
    ///
    Screenshot(sk_sp<SkData> p_data,
               SkISize p_size,
               const std::string& p_format,
               ScreenshotFormat p_pixel_format);

    //--------------------------------------------------------------------------
    /// @brief      The copy constructor for a screenshot.
    ///
    /// @param[in]  other  The screenshot to copy from.
    ///
    Screenshot(const Screenshot& other);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the screenshot object and releases underlying data.
    ///
    ~Screenshot();
  };

  //----------------------------------------------------------------------------
  /// @brief      Screenshots the last layer tree to one of the supported
  ///             screenshot types and optionally Base 64 encodes that data for
  ///             easier transmission and packaging (usually over the service
  ///             protocol for instrumentation tools running on the host).
  ///
  /// @param[in]  type           The type of the screenshot to gather.
  /// @param[in]  base64_encode  Whether Base 64 encoding must be applied to the
  ///                            data after a screenshot has been captured.
  ///
  /// @return     A non-empty screenshot if one could be captured. A screenshot
  ///             capture may fail if there were no layer trees previously
  ///             rendered by this rasterizer, or, due to an unspecified
  ///             internal error. Internal error will be logged to the console.
  ///
  Screenshot ScreenshotLastLayerTree(ScreenshotType type, bool base64_encode);

  //----------------------------------------------------------------------------
  /// @brief      Sets a callback that will be executed when the next layer tree
  ///             in rendered to the on-screen surface. This is used by
  ///             embedders to listen for one time operations like listening for
  ///             when the first frame is rendered so that they may hide splash
  ///             screens.
  ///
  ///             The callback is only executed once and dropped on the GPU
  ///             thread when executed (lambda captures must be able to deal
  ///             with the threading repercussions of this behavior).
  ///
  /// @param[in]  callback  The callback to execute when the next layer tree is
  ///                       rendered on-screen.
  ///
  void SetNextFrameCallback(const fml::closure& callback);

  //----------------------------------------------------------------------------
  /// @brief Set the External View Embedder. This is done on shell
  ///        initialization. This is non-null on platforms that support
  ///        embedding externally composited views.
  ///
  /// @param[in] view_embedder The external view embedder object.
  ///
  void SetExternalViewEmbedder(
      const std::shared_ptr<ExternalViewEmbedder>& view_embedder);

  //----------------------------------------------------------------------------
  /// @brief Set the snapshot surface producer. This is done on shell
  ///        initialization. This is non-null on platforms that support taking
  ///        GPU accelerated raster snapshots in the background.
  ///
  /// @param[in]  producer  A surface producer for raster snapshotting when the
  ///                       onscreen surface is not available.
  ///
  void SetSnapshotSurfaceProducer(
      std::unique_ptr<SnapshotSurfaceProducer> producer);

  //----------------------------------------------------------------------------
  /// @brief      Returns a pointer to the compositor context used by this
  ///             rasterizer. This pointer will never be `nullptr`.
  ///
  /// @return     The compositor context used by this rasterizer.
  ///
  flutter::CompositorContext* compositor_context() {
    return compositor_context_.get();
  }

  //----------------------------------------------------------------------------
  /// @brief      Returns the raster thread merger used by this rasterizer.
  ///             This may be `nullptr`.
  ///
  /// @return     The raster thread merger used by this rasterizer.
  ///
  fml::RefPtr<fml::RasterThreadMerger> GetRasterThreadMerger();

  //----------------------------------------------------------------------------
  /// @brief      Skia has no notion of time. To work around the performance
  ///             implications of this, it may cache GPU resources to reference
  ///             them from one frame to the next. Using this call, embedders
  ///             may set the maximum bytes cached by Skia in its caches
  ///             dedicated to on-screen rendering.
  ///
  /// @attention  This cache setting will be invalidated when the surface is
  ///             torn down via `Rasterizer::Teardown`. This call must be made
  ///             again with new limits after surface re-acquisition.
  ///
  /// @attention  This cache does not describe the entirety of GPU resources
  ///             that may be cached. The `RasterCache` also holds very large
  ///             GPU resources.
  ///
  /// @see        `RasterCache`
  ///
  /// @param[in]  max_bytes  The maximum byte size of resource that may be
  ///                        cached for GPU rendering.
  /// @param[in]  from_user  Whether this request was from user code, e.g. via
  ///                        the flutter/skia message channel, in which case
  ///                        it should not be overridden by the platform.
  ///
  void SetResourceCacheMaxBytes(size_t max_bytes, bool from_user);

  //----------------------------------------------------------------------------
  /// @brief      The current value of Skia's resource cache size, if a surface
  ///             is present.
  ///
  /// @attention  This cache does not describe the entirety of GPU resources
  ///             that may be cached. The `RasterCache` also holds very large
  ///             GPU resources.
  ///
  /// @see        `RasterCache`
  ///
  /// @return     The size of Skia's resource cache, if available.
  ///
  std::optional<size_t> GetResourceCacheMaxBytes() const;

  //----------------------------------------------------------------------------
  /// @brief      Enables the thread merger if the external view embedder
  ///             supports dynamic thread merging.
  ///
  /// @attention  This method is thread-safe. When the thread merger is enabled,
  ///             the raster task queue can run in the platform thread at any
  ///             time.
  ///
  /// @see        `ExternalViewEmbedder`
  ///
  void EnableThreadMergerIfNeeded();

  //----------------------------------------------------------------------------
  /// @brief      Disables the thread merger if the external view embedder
  ///             supports dynamic thread merging.
  ///
  /// @attention  This method is thread-safe. When the thread merger is
  ///             disabled, the raster task queue will continue to run in the
  ///             same thread until |EnableThreadMergerIfNeeded| is called.
  ///
  /// @see        `ExternalViewEmbedder`
  ///
  void DisableThreadMergerIfNeeded();

  //----------------------------------------------------------------------------
  /// @brief      Returns whether TearDown has been called.
  ///
  ///             This method is used only in unit tests.
  ///
  bool IsTornDown();

  //----------------------------------------------------------------------------
  /// @brief      Returns the last status of drawing the specific view.
  ///
  ///             This method is used only in unit tests.
  ///
  std::optional<DrawSurfaceStatus> GetLastDrawStatus(int64_t view_id);

 private:
  // The result status of DoDraw, DrawToSurfaces, and DrawToSurfacesUnsafe.
  enum class DoDrawStatus {
    // The drawing was done without any specified status.
    kDone,
    // Frame has been successfully rasterized, but there are additional items
    // in the pipeline waiting to be consumed. This is currently only used when
    // thread configuration change occurs.
    kEnqueuePipeline,
    // Failed to rasterize the frame because the Rasterizer is not set up.
    kNotSetUp,
    // Nothing was done, because GPU was unavailable.
    kGpuUnavailable,
  };

  // The result of DoDraw.
  struct DoDrawResult {
    // The overall status of the drawing process.
    //
    // The status of drawing a specific view is available at GetLastDrawStatus.
    DoDrawStatus status = DoDrawStatus::kDone;

    // The frame item that needs to be submitted again.
    //
    // See RasterStatus::kResubmit and kSkipAndRetry for when it happens.
    //
    // If `resubmitted_item` is not null, its `tasks` is guaranteed to be
    // non-empty.
    std::unique_ptr<FrameItem> resubmitted_item;
  };

  struct ViewRecord {
    std::unique_ptr<LayerTreeTask> last_successful_task;
    std::optional<DrawSurfaceStatus> last_draw_status;
  };

  // |SnapshotDelegate|
  std::unique_ptr<GpuImageResult> MakeSkiaGpuImage(
      sk_sp<DisplayList> display_list,
      const SkImageInfo& image_info) override;

  // |SnapshotDelegate|
  void MakeRasterSnapshot(
      sk_sp<DisplayList> display_list,
      SkISize picture_size,
      std::function<void(sk_sp<DlImage>)> callback) override;

  // |SnapshotDelegate|
  sk_sp<DlImage> MakeRasterSnapshotSync(sk_sp<DisplayList> display_list,
                                        SkISize picture_size) override;

  // |SnapshotDelegate|
  sk_sp<SkImage> ConvertToRasterImage(sk_sp<SkImage> image) override;

  // |SnapshotDelegate|
  void CacheRuntimeStage(
      const std::shared_ptr<impeller::RuntimeStage>& runtime_stage) override;

  // |Stopwatch::Delegate|
  /// Time limit for a smooth frame.
  ///
  /// See: `DisplayManager::GetMainDisplayRefreshRate`.
  fml::Milliseconds GetFrameBudget() const override;

  // |SnapshotController::Delegate|
  const std::unique_ptr<Surface>& GetSurface() const override {
    return surface_;
  }

  // |SnapshotController::Delegate|
  std::shared_ptr<impeller::AiksContext> GetAiksContext() const override {
#if IMPELLER_SUPPORTS_RENDERING
    if (surface_) {
      return surface_->GetAiksContext();
    }
    if (auto context = impeller_context_.lock()) {
      return std::make_shared<impeller::AiksContext>(
          context, impeller::TypographerContextSkia::Make());
    }
#endif
    return nullptr;
  }

  // |SnapshotController::Delegate|
  const std::unique_ptr<SnapshotSurfaceProducer>& GetSnapshotSurfaceProducer()
      const override {
    return snapshot_surface_producer_;
  }

  // |SnapshotController::Delegate|
  std::shared_ptr<const fml::SyncSwitch> GetIsGpuDisabledSyncSwitch()
      const override {
    return delegate_.GetIsGpuDisabledSyncSwitch();
  }

  std::pair<sk_sp<SkData>, ScreenshotFormat> ScreenshotLayerTreeAsImage(
      flutter::LayerTree* tree,
      flutter::CompositorContext& compositor_context,
      bool compressed);

  // This method starts with the frame timing recorder at build end. This
  // method might push it to raster end and get the recorded time, or abort in
  // the middle and not get the recorded time.
  DoDrawResult DoDraw(
      std::unique_ptr<FrameTimingsRecorder> frame_timings_recorder,
      std::vector<std::unique_ptr<LayerTreeTask>> tasks);

  // This method pushes the frame timing recorder from build end to raster end.
  DoDrawResult DrawToSurfaces(
      FrameTimingsRecorder& frame_timings_recorder,
      std::vector<std::unique_ptr<LayerTreeTask>> tasks);

  // Draws the specified layer trees to views, assuming we have access to the
  // GPU.
  //
  // If any layer trees need resubmitting, this method returns the frame item to
  // be resubmitted. Otherwise, it returns nullptr.
  //
  // Unsafe because it assumes we have access to the GPU which isn't the case
  // when iOS is backgrounded, for example.
  //
  // This method pushes the frame timing recorder from build end to raster end.
  std::unique_ptr<FrameItem> DrawToSurfacesUnsafe(
      FrameTimingsRecorder& frame_timings_recorder,
      std::vector<std::unique_ptr<LayerTreeTask>> tasks);

  // Draws the layer tree to the specified view, assuming we have access to the
  // GPU.
  //
  // This method is not affiliated with the frame timing recorder, but must be
  // included between the RasterStart and RasterEnd.
  DrawSurfaceStatus DrawToSurfaceUnsafe(
      int64_t view_id,
      flutter::LayerTree& layer_tree,
      float device_pixel_ratio,
      std::optional<fml::TimePoint> presentation_time);

  ViewRecord& EnsureViewRecord(int64_t view_id);

  void FireNextFrameCallbackIfPresent();

  static bool ShouldResubmitFrame(const DoDrawResult& result);
  static DrawStatus ToDrawStatus(DoDrawStatus status);

  bool is_torn_down_ = false;
  Delegate& delegate_;
  [[maybe_unused]] MakeGpuImageBehavior gpu_image_behavior_;
  std::weak_ptr<impeller::Context> impeller_context_;
  std::unique_ptr<Surface> surface_;
  std::unique_ptr<SnapshotSurfaceProducer> snapshot_surface_producer_;
  std::unique_ptr<flutter::CompositorContext> compositor_context_;
  std::unordered_map<int64_t, ViewRecord> view_records_;
  fml::closure next_frame_callback_;
  bool user_override_resource_cache_bytes_ = false;
  std::optional<size_t> max_cache_bytes_;
  fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger_;
  std::shared_ptr<ExternalViewEmbedder> external_view_embedder_;
  std::unique_ptr<SnapshotController> snapshot_controller_;

  // WeakPtrFactory must be the last member.
  fml::TaskRunnerAffineWeakPtrFactory<Rasterizer> weak_factory_;
  FML_DISALLOW_COPY_AND_ASSIGN(Rasterizer);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_RASTERIZER_H_
