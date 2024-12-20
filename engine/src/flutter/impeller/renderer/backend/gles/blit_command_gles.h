// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BLIT_COMMAND_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BLIT_COMMAND_GLES_H_

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"
#include "impeller/renderer/blit_command.h"

namespace impeller {

/// Mixin for dispatching GLES commands.
struct BlitEncodeGLES : BackendCast<BlitEncodeGLES, BlitCommand> {
  virtual ~BlitEncodeGLES();

  virtual std::string GetLabel() const = 0;

  [[nodiscard]] virtual bool Encode(const ReactorGLES& reactor) const = 0;
};

struct BlitCopyBufferToTextureCommandGLES
    : public BlitEncodeGLES,
      public BlitCopyBufferToTextureCommand {
  ~BlitCopyBufferToTextureCommandGLES() override;

  std::string GetLabel() const override;

  [[nodiscard]] bool Encode(const ReactorGLES& reactor) const override;
};

struct BlitCopyTextureToTextureCommandGLES
    : public BlitEncodeGLES,
      public BlitCopyTextureToTextureCommand {
  ~BlitCopyTextureToTextureCommandGLES() override;

  std::string GetLabel() const override;

  [[nodiscard]] bool Encode(const ReactorGLES& reactor) const override;
};

struct BlitCopyTextureToBufferCommandGLES
    : public BlitEncodeGLES,
      public BlitCopyTextureToBufferCommand {
  ~BlitCopyTextureToBufferCommandGLES() override;

  std::string GetLabel() const override;

  [[nodiscard]] bool Encode(const ReactorGLES& reactor) const override;
};

struct BlitGenerateMipmapCommandGLES : public BlitEncodeGLES,
                                       public BlitGenerateMipmapCommand {
  ~BlitGenerateMipmapCommandGLES() override;

  std::string GetLabel() const override;

  [[nodiscard]] bool Encode(const ReactorGLES& reactor) const override;
};

struct BlitResizeTextureCommandGLES : public BlitEncodeGLES,
                                      public BlitResizeTextureCommand {
  ~BlitResizeTextureCommandGLES() override;

  std::string GetLabel() const override;

  [[nodiscard]] bool Encode(const ReactorGLES& reactor) const override;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BLIT_COMMAND_GLES_H_
