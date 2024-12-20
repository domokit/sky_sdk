// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_COMMAND_BUFFER_H_
#define FLUTTER_LIB_GPU_COMMAND_BUFFER_H_

#include "flutter/lib/gpu/context.h"
#include "flutter/lib/gpu/export.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/context.h"

namespace flutter {
namespace gpu {

class CommandBuffer : public RefCountedDartWrappable<CommandBuffer> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(CommandBuffer);

 public:
  CommandBuffer(std::shared_ptr<impeller::Context> context,
                std::shared_ptr<impeller::CommandBuffer> command_buffer);

  std::shared_ptr<impeller::CommandBuffer> GetCommandBuffer();

  void AddRenderPass(std::shared_ptr<impeller::RenderPass> render_pass);

  bool Submit();
  bool Submit(
      const impeller::CommandBuffer::CompletionCallback& completion_callback);

  ~CommandBuffer() override;

 private:
  std::shared_ptr<impeller::Context> context_;
  std::shared_ptr<impeller::CommandBuffer> command_buffer_;
  std::vector<std::shared_ptr<impeller::RenderPass>> encodables_;

  FML_DISALLOW_COPY_AND_ASSIGN(CommandBuffer);
};

}  // namespace gpu
}  // namespace flutter

//----------------------------------------------------------------------------
/// Exports
///

extern "C" {

FLUTTER_GPU_EXPORT
extern bool InternalFlutterGpu_CommandBuffer_Initialize(
    Dart_Handle wrapper,
    flutter::gpu::Context* contextWrapper);

FLUTTER_GPU_EXPORT
extern Dart_Handle InternalFlutterGpu_CommandBuffer_Submit(
    flutter::gpu::CommandBuffer* wrapper,
    Dart_Handle completion_callback);

}  // extern "C"

#endif  // FLUTTER_LIB_GPU_COMMAND_BUFFER_H_
