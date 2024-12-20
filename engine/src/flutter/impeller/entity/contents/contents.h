// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENTS_H_

#include <functional>
#include <memory>

#include "impeller/core/sampler_descriptor.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/rect.h"
#include "impeller/renderer/snapshot.h"
#include "impeller/typographer/lazy_glyph_atlas.h"

namespace impeller {

class ContentContext;
struct ContentContextOptions;
class Entity;
class Surface;
class RenderPass;
class FilterContents;

ContentContextOptions OptionsFromPass(const RenderPass& pass);

ContentContextOptions OptionsFromPassAndEntity(const RenderPass& pass,
                                               const Entity& entity);

class Contents {
 public:
  /// A procedure that filters a given unpremultiplied color to produce a new
  /// unpremultiplied color.
  using ColorFilterProc = std::function<Color(Color)>;

  using RenderProc = std::function<bool(const ContentContext& renderer,
                                        const Entity& entity,
                                        RenderPass& pass)>;
  using CoverageProc = std::function<std::optional<Rect>(const Entity& entity)>;

  static std::shared_ptr<Contents> MakeAnonymous(RenderProc render_proc,
                                                 CoverageProc coverage_proc);

  Contents();

  virtual ~Contents();

  virtual bool Render(const ContentContext& renderer,
                      const Entity& entity,
                      RenderPass& pass) const = 0;

  //----------------------------------------------------------------------------
  /// @brief   Get the area of the render pass that will be affected when this
  ///          contents is rendered.
  ///
  ///          During rendering, coverage coordinates count pixels from the top
  ///          left corner of the framebuffer.
  ///
  /// @return  The coverage rectangle. An `std::nullopt` result means that
  ///          rendering this contents has no effect on the output color.
  ///
  virtual std::optional<Rect> GetCoverage(const Entity& entity) const = 0;

  //----------------------------------------------------------------------------
  /// @brief  Hint that specifies the coverage area of this Contents that will
  ///         actually be used during rendering. This is for optimization
  ///         purposes only and can not be relied on as a clip. May optionally
  ///         affect the result of `GetCoverage()`.
  ///
  void SetCoverageHint(std::optional<Rect> coverage_hint);

  const std::optional<Rect>& GetCoverageHint() const;

  //----------------------------------------------------------------------------
  /// @brief Whether this Contents only emits opaque source colors from the
  ///        fragment stage. This value does not account for any entity
  ///        properties (e.g. the blend mode), clips/visibility culling, or
  ///        inherited opacity.
  ///
  /// @param transform The current transform matrix of the entity that will
  /// render this contents.
  virtual bool IsOpaque(const Matrix& transform) const;

  //----------------------------------------------------------------------------
  /// @brief Render this contents to a snapshot, respecting the entity's
  ///        transform, path, clip depth, and blend mode.
  ///        The result texture size is always the size of
  ///        `GetCoverage(entity)`.
  ///
  virtual std::optional<Snapshot> RenderToSnapshot(
      const ContentContext& renderer,
      const Entity& entity,
      std::optional<Rect> coverage_limit = std::nullopt,
      const std::optional<SamplerDescriptor>& sampler_descriptor = std::nullopt,
      bool msaa_enabled = true,
      int32_t mip_count = 1,
      std::string_view label = "Snapshot") const;

  //----------------------------------------------------------------------------
  /// @brief  Return the color source's intrinsic size, if available.
  ///
  ///         For example, a gradient has a size based on its end and beginning
  ///         points, ignoring any tiling. Solid colors and runtime effects have
  ///         no size.
  ///
  std::optional<Size> GetColorSourceSize() const;

  void SetColorSourceSize(Size size);

  //----------------------------------------------------------------------------
  /// @brief Inherit the provided opacity.
  ///
  ///        Use of this method is invalid if CanAcceptOpacity returns false.
  ///
  virtual void SetInheritedOpacity(Scalar opacity);

  //----------------------------------------------------------------------------
  /// @brief Returns a color if this Contents will flood the given `target_size`
  ///        with a color. This output color is the "Source" color that will be
  ///        used for the Entity's blend operation.
  ///
  ///        This is useful for absorbing full screen solid color draws into
  ///        subpass clear colors.
  ///
  virtual std::optional<Color> AsBackgroundColor(const Entity& entity,
                                                 ISize target_size) const;

  //----------------------------------------------------------------------------
  /// @brief      If possible, applies a color filter to this contents inputs on
  ///             the CPU.
  ///
  ///             This method will either fully apply the color filter or
  ///             perform no action. Partial/incorrect application of the color
  ///             filter will never occur.
  ///
  /// @param[in]  color_filter_proc  A function that filters a given
  ///                                unpremultiplied color to produce a new
  ///                                unpremultiplied color.
  ///
  /// @return     True if the color filter was able to be fully applied to all
  ///             all relevant inputs. Otherwise, this operation is a no-op and
  ///             false is returned.
  ///
  [[nodiscard]] virtual bool ApplyColorFilter(
      const ColorFilterProc& color_filter_proc);

 private:
  std::optional<Rect> coverage_hint_;
  std::optional<Size> color_source_size_;

  Contents(const Contents&) = delete;

  Contents& operator=(const Contents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_CONTENTS_H_
