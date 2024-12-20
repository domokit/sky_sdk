// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_RENDER_PASS_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_RENDER_PASS_GLES_H_

#include <memory>

#include "flutter/impeller/renderer/backend/gles/reactor_gles.h"
#include "flutter/impeller/renderer/render_pass.h"

namespace impeller {

class RenderPassGLES final
    : public RenderPass,
      public std::enable_shared_from_this<RenderPassGLES> {
 public:
  // |RenderPass|
  ~RenderPassGLES() override;

  static void ResetGLState(const ProcTableGLES& gl);

 private:
  friend class CommandBufferGLES;

  std::shared_ptr<ReactorGLES> reactor_;
  std::string label_;
  bool is_valid_ = false;

  RenderPassGLES(std::shared_ptr<const Context> context,
                 const RenderTarget& target,
                 std::shared_ptr<ReactorGLES> reactor);

  // |RenderPass|
  bool IsValid() const override;

  // |RenderPass|
  void OnSetLabel(std::string_view label) override;

  // |RenderPass|
  bool OnEncodeCommands(const Context& context) const override;

  RenderPassGLES(const RenderPassGLES&) = delete;

  RenderPassGLES& operator=(const RenderPassGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_RENDER_PASS_GLES_H_
