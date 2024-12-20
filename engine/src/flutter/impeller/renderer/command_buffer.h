// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_COMMAND_BUFFER_H_
#define FLUTTER_IMPELLER_RENDERER_COMMAND_BUFFER_H_

#include <functional>
#include <memory>

#include "impeller/renderer/blit_pass.h"
#include "impeller/renderer/compute_pass.h"

namespace impeller {

class ComputePass;
class Context;
class RenderPass;
class RenderTarget;
class CommandQueue;

namespace testing {
class CommandBufferMock;
}

//------------------------------------------------------------------------------
/// @brief      A collection of encoded commands to be submitted to the GPU for
///             execution. A command buffer is obtained from a graphics
///             `Context`.
///
///             To submit commands to the GPU, acquire a `RenderPass` from the
///             command buffer and record `Command`s into that pass. A
///             `RenderPass` describes the configuration of the various
///             attachments when the command is submitted.
///
///             A command buffer is only meant to be used on a single thread. If
///             a frame workload needs to be encoded from multiple threads,
///             set up and record into multiple command buffers. The order of
///             submission of commands encoded in multiple command buffers can
///             be controlled via either the order in which the command buffers
///             were created, or, using the `ReserveSpotInQueue` command which
///             allows for encoding commands for submission in an order that is
///             different from the encoding order.
///
class CommandBuffer {
  friend class testing::CommandBufferMock;

 public:
  enum class Status {
    kPending,
    kError,
    kCompleted,
  };

  using CompletionCallback = std::function<void(Status)>;

  virtual ~CommandBuffer();

  virtual bool IsValid() const = 0;

  virtual void SetLabel(std::string_view label) const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Block the current thread until the GPU has completed execution
  ///             of the commands.
  ///
  void WaitUntilCompleted();

  //----------------------------------------------------------------------------
  /// @brief      Block the current thread until the GPU has completed
  ///             scheduling execution of the commands.
  ///
  void WaitUntilScheduled();

  //----------------------------------------------------------------------------
  /// @brief      Create a render pass to record render commands into.
  ///
  /// @param[in]  render_target  The description of the render target this pass
  ///                            will target.
  ///
  /// @return     A valid render pass or null.
  ///
  std::shared_ptr<RenderPass> CreateRenderPass(
      const RenderTarget& render_target);

  //----------------------------------------------------------------------------
  /// @brief      Create a blit pass to record blit commands into.
  ///
  /// @return     A valid blit pass or null.
  ///
  std::shared_ptr<BlitPass> CreateBlitPass();

  //----------------------------------------------------------------------------
  /// @brief      Create a compute pass to record compute commands into.
  ///
  /// @return     A valid compute pass or null.
  ///
  std::shared_ptr<ComputePass> CreateComputePass();

 protected:
  std::weak_ptr<const Context> context_;

  explicit CommandBuffer(std::weak_ptr<const Context> context);

  virtual std::shared_ptr<RenderPass> OnCreateRenderPass(
      RenderTarget render_target) = 0;

  virtual std::shared_ptr<BlitPass> OnCreateBlitPass() = 0;

  [[nodiscard]] virtual bool OnSubmitCommands(CompletionCallback callback) = 0;

  virtual void OnWaitUntilCompleted() = 0;

  virtual void OnWaitUntilScheduled() = 0;

  virtual std::shared_ptr<ComputePass> OnCreateComputePass() = 0;

 private:
  friend class CommandQueue;

  //----------------------------------------------------------------------------
  /// @brief      Schedule the command encoded by render passes within this
  ///             command buffer on the GPU. The encoding of these commnands is
  ///             performed immediately on the calling thread.
  ///
  ///             A command buffer may only be committed once.
  ///
  /// @param[in]  callback  The completion callback.
  ///
  [[nodiscard]] bool SubmitCommands(const CompletionCallback& callback);

  [[nodiscard]] bool SubmitCommands();

  CommandBuffer(const CommandBuffer&) = delete;

  CommandBuffer& operator=(const CommandBuffer&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_COMMAND_BUFFER_H_
