// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_CANVAS_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_CANVAS_H_

#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "display_list/effects/dl_image_filter.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/display_list/paint.h"
#include "impeller/entity/contents/atlas_contents.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/entity_pass_clip_stack.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/vertices_geometry.h"
#include "impeller/entity/inline_pass_context.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/path.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/round_rect.h"
#include "impeller/geometry/vector.h"
#include "impeller/renderer/snapshot.h"
#include "impeller/typographer/text_frame.h"

namespace impeller {

struct BackdropData {
  size_t backdrop_count = 0;
  bool all_filters_equal = true;
  std::shared_ptr<Texture> texture_slot;
  // A single snapshot of the backdrop filter that is used when there are
  // multiple backdrops that share an identical filter.
  std::optional<Snapshot> shared_filter_snapshot;
  std::shared_ptr<flutter::DlImageFilter> last_backdrop;
};

struct CanvasStackEntry {
  Matrix transform;
  uint32_t clip_depth = 0u;
  size_t clip_height = 0u;
  // The number of clips tracked for this canvas stack entry.
  size_t num_clips = 0u;
  Scalar distributed_opacity = 1.0f;
  Entity::RenderingMode rendering_mode = Entity::RenderingMode::kDirect;
  // Whether all entities in the current save should be skipped.
  bool skipping = false;
  // Whether subpass coverage was rounded out to pixel coverage, or if false
  // truncated.
  bool did_round_out = false;
};

enum class PointStyle {
  /// @brief Points are drawn as squares.
  kRound,

  /// @brief Points are drawn as circles.
  kSquare,
};

/// Controls the behavior of the source rectangle given to DrawImageRect.
enum class SourceRectConstraint {
  /// @brief Faster, but may sample outside the bounds of the source rectangle.
  kFast,

  /// @brief Sample only within the source rectangle. May be slower.
  kStrict,
};

/// Specifies how much to trust the bounds rectangle provided for a list
/// of contents. Used by both |EntityPass| and |Canvas::SaveLayer|.
enum class ContentBoundsPromise {
  /// @brief The caller makes no claims related to the size of the bounds.
  kUnknown,

  /// @brief The caller claims the bounds are a reasonably tight estimate
  ///        of the coverage of the contents and should contain all of the
  ///        contents.
  kContainsContents,

  /// @brief The caller claims the bounds are a subset of an estimate of
  ///        the reasonably tight bounds but likely clips off some of the
  ///        contents.
  kMayClipContents,
};

struct LazyRenderingConfig {
  std::unique_ptr<EntityPassTarget> entity_pass_target;
  std::unique_ptr<InlinePassContext> inline_pass_context;

  /// Whether or not the clear color texture can still be updated.
  bool IsApplyingClearColor() const { return !inline_pass_context->IsActive(); }

  LazyRenderingConfig(ContentContext& renderer,
                      std::unique_ptr<EntityPassTarget> p_entity_pass_target)
      : entity_pass_target(std::move(p_entity_pass_target)) {
    inline_pass_context =
        std::make_unique<InlinePassContext>(renderer, *entity_pass_target);
  }

  LazyRenderingConfig(ContentContext& renderer,
                      std::unique_ptr<EntityPassTarget> entity_pass_target,
                      std::unique_ptr<InlinePassContext> inline_pass_context)
      : entity_pass_target(std::move(entity_pass_target)),
        inline_pass_context(std::move(inline_pass_context)) {}
};

class Canvas {
 public:
  static constexpr uint32_t kMaxDepth = 1 << 24;

  using BackdropFilterProc = std::function<std::shared_ptr<FilterContents>(
      FilterInput::Ref,
      const Matrix& effect_transform,
      Entity::RenderingMode rendering_mode)>;

  Canvas(ContentContext& renderer,
         const RenderTarget& render_target,
         bool requires_readback);

  explicit Canvas(ContentContext& renderer,
                  const RenderTarget& render_target,
                  bool requires_readback,
                  Rect cull_rect);

  explicit Canvas(ContentContext& renderer,
                  const RenderTarget& render_target,
                  bool requires_readback,
                  IRect cull_rect);

  ~Canvas() = default;

  /// @brief Update the backdrop data used to group together backdrop filters
  ///        within the same layer
  void SetBackdropData(std::unordered_map<int64_t, BackdropData> backdrop_data,
                       size_t backdrop_count);

  /// @brief Return the culling bounds of the current render target, or nullopt
  ///        if there is no coverage.
  std::optional<Rect> GetLocalCoverageLimit() const;

  void Save(uint32_t total_content_depth = kMaxDepth);

  void SaveLayer(
      const Paint& paint,
      std::optional<Rect> bounds = std::nullopt,
      const flutter::DlImageFilter* backdrop_filter = nullptr,
      ContentBoundsPromise bounds_promise = ContentBoundsPromise::kUnknown,
      uint32_t total_content_depth = kMaxDepth,
      bool can_distribute_opacity = false,
      std::optional<int64_t> backdrop_id = std::nullopt);

  bool Restore();

  size_t GetSaveCount() const;

  void RestoreToCount(size_t count);

  const Matrix& GetCurrentTransform() const;

  void ResetTransform();

  void Transform(const Matrix& transform);

  void Concat(const Matrix& transform);

  void PreConcat(const Matrix& transform);

  void Translate(const Vector3& offset);

  void Scale(const Vector2& scale);

  void Scale(const Vector3& scale);

  void Skew(Scalar sx, Scalar sy);

  void Rotate(Radians radians);

  void DrawPath(const Path& path, const Paint& paint);

  void DrawPaint(const Paint& paint);

  void DrawLine(const Point& p0,
                const Point& p1,
                const Paint& paint,
                bool reuse_depth = false);

  void DrawRect(const Rect& rect, const Paint& paint);

  void DrawOval(const Rect& rect, const Paint& paint);

  void DrawRoundRect(const RoundRect& rect, const Paint& paint);

  void DrawCircle(const Point& center, Scalar radius, const Paint& paint);

  void DrawPoints(const Point points[],
                  uint32_t count,
                  Scalar radius,
                  const Paint& paint,
                  PointStyle point_style);

  void DrawImage(const std::shared_ptr<Texture>& image,
                 Point offset,
                 const Paint& paint,
                 const SamplerDescriptor& sampler = {});

  void DrawImageRect(
      const std::shared_ptr<Texture>& image,
      Rect source,
      Rect dest,
      const Paint& paint,
      const SamplerDescriptor& sampler = {},
      SourceRectConstraint src_rect_constraint = SourceRectConstraint::kFast);

  void DrawTextFrame(const std::shared_ptr<TextFrame>& text_frame,
                     Point position,
                     const Paint& paint);

  void DrawVertices(const std::shared_ptr<VerticesGeometry>& vertices,
                    BlendMode blend_mode,
                    const Paint& paint);

  void DrawAtlas(const std::shared_ptr<AtlasContents>& atlas_contents,
                 const Paint& paint);

  void ClipGeometry(const Geometry& geometry,
                    Entity::ClipOperation clip_op,
                    bool is_aa = true);

  void EndReplay();

  uint64_t GetOpDepth() const { return current_depth_; }

  uint64_t GetMaxOpDepth() const { return transform_stack_.back().clip_depth; }

  struct SaveLayerState {
    Paint paint;
    Rect coverage;
  };

  // Visible for testing.
  bool RequiresReadback() const { return requires_readback_; }

 private:
  ContentContext& renderer_;
  RenderTarget render_target_;
  bool requires_readback_;
  EntityPassClipStack clip_coverage_stack_;

  std::deque<CanvasStackEntry> transform_stack_;
  std::optional<Rect> initial_cull_rect_;
  std::vector<LazyRenderingConfig> render_passes_;
  std::vector<SaveLayerState> save_layer_state_;

  /// Backdrop layers identified by an optional backdrop id.
  ///
  /// This is not the same as the [backdrop_count_] below as not
  /// all backdrop filters will have an identified backdrop id. The
  /// backdrop_count_ is also mutated during rendering.
  std::unordered_map<int64_t, BackdropData> backdrop_data_;

  /// The remaining number of backdrop filters.
  ///
  /// This value is decremented while rendering. When it reaches 0, then
  /// the FlipBackdrop can use the onscreen render target instead of
  /// another offscreen.
  ///
  /// This optimization is disabled on devices that do not support framebuffer
  /// fetch (iOS Simulator and certain OpenGLES devices).
  size_t backdrop_count_ = 0u;

  // All geometry objects created for regular draws can be stack allocated,
  // but clip geometries must be cached for record/replay for backdrop filters
  // and so must be kept alive longer.
  std::vector<std::unique_ptr<Geometry>> clip_geometry_;

  uint64_t current_depth_ = 0u;

  Point GetGlobalPassPosition() const;

  // clip depth of the previous save or 0.
  size_t GetClipHeightFloor() const;

  /// @brief Whether all entites should be skipped until a corresponding
  ///        restore.
  bool IsSkipping() const;

  /// @brief Skip all rendering/clipping entities until next restore.
  void SkipUntilMatchingRestore(size_t total_content_depth);

  void SetupRenderPass();

  /// @brief  Ends the current render pass, saving the result as a texture, and
  ///         thenrestart it with the backdrop cleared to the previous contents.
  ///
  /// The returned texture is used as the input for backdrop filters and
  /// emulated advanced blends. Returns nullptr if there was a validation
  /// failure.
  ///
  /// [should_remove_texture] defaults to false. If true, the render target
  /// texture is removed from the entity pass target. This allows the texture to
  /// be cached by the canvas dispatcher for usage in the backdrop filter reuse
  /// mechanism.
  ///
  /// [should_use_onscreen] defaults to false. If true, the results are flipped
  /// to the onscreen render target. This will set requires_readback_ to false.
  /// This action is only safe to perform when there are no more backdrop
  /// filters or advanced blends, or no more backdrop filters and the device
  /// supports framebuffer fetch.
  std::shared_ptr<Texture> FlipBackdrop(Point global_pass_position,
                                        bool should_remove_texture = false,
                                        bool should_use_onscreen = false);

  bool BlitToOnscreen();

  size_t GetClipHeight() const;

  void Initialize(std::optional<Rect> cull_rect);

  void Reset();

  void AddRenderEntityWithFiltersToCurrentPass(Entity& entity,
                                               const Geometry* geometry,
                                               const Paint& paint,
                                               bool reuse_depth = false);

  void AddRenderEntityToCurrentPass(Entity& entity, bool reuse_depth = false);

  bool AttemptDrawBlurredRRect(const Rect& rect,
                               Size corner_radii,
                               const Paint& paint);

  RenderPass& GetCurrentRenderPass() const;

  Canvas(const Canvas&) = delete;

  Canvas& operator=(const Canvas&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_CANVAS_H_
