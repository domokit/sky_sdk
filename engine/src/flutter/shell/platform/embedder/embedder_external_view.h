// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_VIEW_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_VIEW_H_

#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "flutter/flow/embedded_views.h"
#include "flutter/fml/hash_combine.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder_render_target.h"

namespace flutter {

class EmbedderExternalView {
 public:
  using PlatformViewID = int64_t;
  struct ViewIdentifier {
    std::optional<PlatformViewID> platform_view_id;

    ViewIdentifier() {}

    explicit ViewIdentifier(PlatformViewID view_id)
        : platform_view_id(view_id) {}

    struct Hash {
      constexpr std::size_t operator()(const ViewIdentifier& desc) const {
        if (!desc.platform_view_id.has_value()) {
          return fml::HashCombine();
        }

        return fml::HashCombine(desc.platform_view_id.value());
      }
    };

    struct Equal {
      constexpr bool operator()(const ViewIdentifier& lhs,
                                const ViewIdentifier& rhs) const {
        return lhs.platform_view_id == rhs.platform_view_id;
      }
    };
  };

  struct RenderTargetDescriptor {
    SkISize surface_size;

    explicit RenderTargetDescriptor(const SkISize& p_surface_size)
        : surface_size(p_surface_size) {}

    struct Hash {
      constexpr std::size_t operator()(
          const RenderTargetDescriptor& desc) const {
        return fml::HashCombine(desc.surface_size.width(),
                                desc.surface_size.height());
      }
    };

    struct Equal {
      bool operator()(const RenderTargetDescriptor& lhs,
                      const RenderTargetDescriptor& rhs) const {
        return lhs.surface_size == rhs.surface_size;
      }
    };
  };

  using ViewIdentifierSet = std::unordered_set<ViewIdentifier,
                                               ViewIdentifier::Hash,
                                               ViewIdentifier::Equal>;

  using PendingViews = std::unordered_map<ViewIdentifier,
                                          std::unique_ptr<EmbedderExternalView>,
                                          ViewIdentifier::Hash,
                                          ViewIdentifier::Equal>;

  EmbedderExternalView(const SkISize& frame_size,
                       const SkMatrix& surface_transformation);

  EmbedderExternalView(const SkISize& frame_size,
                       const SkMatrix& surface_transformation,
                       ViewIdentifier view_identifier,
                       std::unique_ptr<EmbeddedViewParams> params);

  ~EmbedderExternalView();

  bool IsRootView() const;

  bool HasPlatformView() const;

  bool HasEngineRenderedContents();

  ViewIdentifier GetViewIdentifier() const;

  const EmbeddedViewParams* GetEmbeddedViewParams() const;

  RenderTargetDescriptor CreateRenderTargetDescriptor() const;

  DlCanvas* GetCanvas();

  SkISize GetRenderSurfaceSize() const;

  bool Render(const EmbedderRenderTarget& render_target,
              bool clear_surface = true);

  const DlRegion& GetDlRegion() const;

 private:
  // End the recording of the slice.
  // Noop if the slice's recording has already ended.
  void TryEndRecording() const;

  const SkISize render_surface_size_;
  const SkMatrix surface_transformation_;
  ViewIdentifier view_identifier_;
  std::unique_ptr<EmbeddedViewParams> embedded_view_params_;
  std::unique_ptr<DisplayListEmbedderViewSlice> slice_;
  std::optional<bool> has_engine_rendered_contents_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderExternalView);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_EXTERNAL_VIEW_H_
