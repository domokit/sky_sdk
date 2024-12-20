// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_SURFACE_FRAME_H_
#define FLUTTER_FLOW_SURFACE_FRAME_H_

#include <memory>
#include <optional>

#include "flutter/common/graphics/gl_context_switch.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_point.h"

#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace impeller {
class Surface;
}

namespace flutter {

// This class represents a frame that has been fully configured for the
// underlying client rendering API. A frame may only be submitted once.
class SurfaceFrame {
 public:
  using EncodeCallback =
      std::function<bool(SurfaceFrame& surface_frame, DlCanvas* canvas)>;
  using SubmitCallback = std::function<bool(SurfaceFrame& surface_frame)>;

  // Information about the underlying framebuffer
  struct FramebufferInfo {
    // Indicates whether or not the surface supports pixel readback as used in
    // circumstances such as a BackdropFilter.
    bool supports_readback = false;

    // Indicates that target device supports partial repaint. At very minimum
    // this means that the surface will provide valid existing damage.
    bool supports_partial_repaint = false;

    // For some targets it may be beneficial or even required to snap clip
    // rect to tile grid. I.e. repainting part of a tile may cause performance
    // degradation if the tile needs to be decompressed first.
    int vertical_clip_alignment = 1;
    int horizontal_clip_alignment = 1;

    // This is the area of framebuffer that lags behind the front buffer.
    //
    // Correctly providing exiting_damage is necessary for supporting double and
    // triple buffering. Embedder is responsible for tracking this area for each
    // of the back buffers used. When doing partial redraw, this area will be
    // repainted alongside of dirty area determined by diffing current and
    // last successfully rasterized layer tree;
    //
    // If existing damage is unspecified (nullopt), entire frame will be
    // rasterized (no partial redraw). To signal that there is no existing
    // damage use an empty SkIRect.
    std::optional<SkIRect> existing_damage = std::nullopt;
  };

  SurfaceFrame(sk_sp<SkSurface> surface,
               FramebufferInfo framebuffer_info,
               const EncodeCallback& encode_callback,
               const SubmitCallback& submit_callback,
               SkISize frame_size,
               std::unique_ptr<GLContextResult> context_result = nullptr,
               bool display_list_fallback = false);

  struct SubmitInfo {
    // The frame damage for frame n is the difference between frame n and
    // frame (n-1), and represents the area that a compositor must recompose.
    //
    // Corresponds to EGL_KHR_swap_buffers_with_damage
    std::optional<SkIRect> frame_damage;

    // The buffer damage for a frame is the area changed since that same buffer
    // was last used. If the buffer has not been used before, the buffer damage
    // is the entire area of the buffer.
    //
    // Corresponds to EGL_KHR_partial_update
    std::optional<SkIRect> buffer_damage;

    // Time at which this frame is scheduled to be presented. This is a hint
    // that can be passed to the platform to drop queued frames.
    std::optional<fml::TimePoint> presentation_time;

    // Whether this surface frame represents the last in a group frames that
    // were submitted as part of a platform compositor interop step, such as
    // during iOS platform view compositing.
    //
    // Defaults to true, which is generally a safe value.
    bool frame_boundary = true;

    // Whether this surface presents with a CATransaction on Apple platforms.
    //
    // When there are platform views in the scene, the drawable needs to be
    // presented in the same CATransaction as the one created for platform view
    // mutations.
    //
    // If the drawables are being presented from the raster thread, we cannot
    // use a transaction as it will dirty the UIViews being presented. If there
    // is a non-Flutter UIView active, such as in add2app or a
    // presentViewController page transition, then this will cause CoreAnimation
    // assertion errors and exit the app.
    bool present_with_transaction = false;
  };

  bool Encode();

  bool Submit();

  bool IsSubmitted() const;

  sk_sp<SkSurface> SkiaSurface() const;

  DlCanvas* Canvas();

  const FramebufferInfo& framebuffer_info() const { return framebuffer_info_; }

  void set_submit_info(const SubmitInfo& submit_info) {
    submit_info_ = submit_info;
  }
  const SubmitInfo& submit_info() const { return submit_info_; }

  sk_sp<DisplayList> BuildDisplayList();

  void set_user_data(std::shared_ptr<impeller::Surface> data) {
    user_data_ = std::move(data);
  }

  std::shared_ptr<impeller::Surface> take_user_data() {
    return std::move(user_data_);
  }

 private:
  bool submitted_ = false;
  bool encoded_ = false;

#if !SLIMPELLER
  DlSkCanvasAdapter adapter_;
#endif  //  !SLIMPELLER
  sk_sp<DisplayListBuilder> dl_builder_;
  sk_sp<SkSurface> surface_;
  DlCanvas* canvas_ = nullptr;
  FramebufferInfo framebuffer_info_;
  SubmitInfo submit_info_;
  EncodeCallback encode_callback_;
  SubmitCallback submit_callback_;
  std::shared_ptr<impeller::Surface> user_data_;
  std::unique_ptr<GLContextResult> context_result_;

  bool PerformSubmit();

  bool PerformEncode();

  FML_DISALLOW_COPY_AND_ASSIGN(SurfaceFrame);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_SURFACE_FRAME_H_
