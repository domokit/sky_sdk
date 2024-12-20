// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_CONTEXT_H_
#define FLUTTER_IMPELLER_RENDERER_CONTEXT_H_

#include <memory>
#include <string>

#include "fml/closure.h"
#include "impeller/core/allocator.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/capabilities.h"
#include "impeller/renderer/command_queue.h"
#include "impeller/renderer/sampler_library.h"

namespace impeller {

class ShaderLibrary;
class CommandBuffer;
class PipelineLibrary;

//------------------------------------------------------------------------------
/// @brief      To do anything rendering related with Impeller, you need a
///             context.
///
///             Contexts are expensive to construct and typically you only need
///             one in the process. The context represents a connection to a
///             graphics or compute accelerator on the device.
///
///             If there are multiple context in a process, it would typically
///             be for separation of concerns (say, use with multiple engines in
///             Flutter), talking to multiple accelerators, or talking to the
///             same accelerator using different client APIs (Metal, Vulkan,
///             OpenGL ES, etc..).
///
///             Contexts are thread-safe. They may be created, used, and
///             collected (though not from a thread used by an internal pool) on
///             any thread. They may also be accessed simultaneously from
///             multiple threads.
///
///             Contexts are abstract and a concrete instance must be created
///             using one of the subclasses of `Context` in
///             `//impeller/renderer/backend`.
class Context {
 public:
  enum class BackendType {
    kMetal,
    kOpenGLES,
    kVulkan,
  };

  /// The maximum number of tasks that should ever be stored for
  /// `StoreTaskForGPU`.
  ///
  /// This number was arbitrarily chosen. The idea is that this is a somewhat
  /// rare situation where tasks happen to get executed in that tiny amount of
  /// time while an app is being backgrounded but still executing.
  static constexpr int32_t kMaxTasksAwaitingGPU = 10;

  //----------------------------------------------------------------------------
  /// @brief      Destroys an Impeller context.
  ///
  virtual ~Context();

  //----------------------------------------------------------------------------
  /// @brief      Get the graphics backend of an Impeller context.
  ///
  ///             This is useful for cases where a renderer needs to track and
  ///             lookup backend-specific resources, like shaders or uniform
  ///             layout information.
  ///
  ///             It's not recommended to use this as a substitute for
  ///             per-backend capability checking. Instead, check for specific
  ///             capabilities via `GetCapabilities()`.
  ///
  /// @return     The graphics backend of the `Context`.
  ///
  virtual BackendType GetBackendType() const = 0;

  // TODO(129920): Refactor and move to capabilities.
  virtual std::string DescribeGpuModel() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Determines if a context is valid. If the caller ever receives
  ///             an invalid context, they must discard it and construct a new
  ///             context. There is no recovery mechanism to repair a bad
  ///             context.
  ///
  ///             It is convention in Impeller to never return an invalid
  ///             context from a call that returns an pointer to a context. The
  ///             call implementation performs validity checks itself and return
  ///             a null context instead of a pointer to an invalid context.
  ///
  ///             How a context goes invalid is backend specific. It could
  ///             happen due to device loss, or any other unrecoverable error.
  ///
  /// @return     If the context is valid.
  ///
  virtual bool IsValid() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Get the capabilities of Impeller context. All optionally
  ///             supported feature of the platform, client-rendering API, and
  ///             device can be queried using the `Capabilities`.
  ///
  /// @return     The capabilities. Can never be `nullptr` for a valid context.
  ///
  virtual const std::shared_ptr<const Capabilities>& GetCapabilities()
      const = 0;

  // TODO(129920): Refactor and move to capabilities.
  virtual bool UpdateOffscreenLayerPixelFormat(PixelFormat format);

  //----------------------------------------------------------------------------
  /// @brief      Returns the allocator used to create textures and buffers on
  ///             the device.
  ///
  /// @return     The resource allocator. Can never be `nullptr` for a valid
  ///             context.
  ///
  virtual std::shared_ptr<Allocator> GetResourceAllocator() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Returns the library of shaders used to specify the
  ///             programmable stages of a pipeline.
  ///
  /// @return     The shader library. Can never be `nullptr` for a valid
  ///             context.
  ///
  virtual std::shared_ptr<ShaderLibrary> GetShaderLibrary() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Returns the library of combined image samplers used in
  ///             shaders.
  ///
  /// @return     The sampler library. Can never be `nullptr` for a valid
  ///             context.
  ///
  virtual std::shared_ptr<SamplerLibrary> GetSamplerLibrary() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Returns the library of pipelines used by render or compute
  ///             commands.
  ///
  /// @return     The pipeline library. Can never be `nullptr` for a valid
  ///             context.
  ///
  virtual std::shared_ptr<PipelineLibrary> GetPipelineLibrary() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Create a new command buffer. Command buffers can be used to
  ///             encode graphics, blit, or compute commands to be submitted to
  ///             the device.
  ///
  ///             A command buffer can only be used on a single thread.
  ///             Multi-threaded render, blit, or compute passes must create a
  ///             new command buffer on each thread.
  ///
  /// @return     A new command buffer.
  ///
  virtual std::shared_ptr<CommandBuffer> CreateCommandBuffer() const = 0;

  /// @brief Return the graphics queue for submitting command buffers.
  virtual std::shared_ptr<CommandQueue> GetCommandQueue() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Force all pending asynchronous work to finish. This is
  ///             achieved by deleting all owned concurrent message loops.
  ///
  virtual void Shutdown() = 0;

  /// Stores a task on the `ContextMTL` that is awaiting access for the GPU.
  ///
  /// The task will be executed in the event that the GPU access has changed to
  /// being available or that the task has been canceled. The task should
  /// operate with the `SyncSwitch` to make sure the GPU is accessible.
  ///
  /// If the queue of pending tasks is cleared without GPU access, then the
  /// failure callback will be invoked and the primary task function will not
  ///
  /// Threadsafe.
  ///
  /// `task` will be executed on the platform thread.
  virtual void StoreTaskForGPU(const fml::closure& task,
                               const fml::closure& failure) {
    FML_CHECK(false && "not supported in this context");
  }

  /// Run backend specific additional setup and create common shader variants.
  ///
  /// This bootstrap is intended to improve the performance of several
  /// first frame benchmarks that are tracked in the flutter device lab.
  /// The workload includes initializing commonly used but not default
  /// shader variants, as well as forcing driver initialization.
  virtual void InitializeCommonlyUsedShadersIfNeeded() const {}

  /// Dispose resources that are cached on behalf of the current thread.
  ///
  /// Some backends such as Vulkan may cache resources that can be reused while
  /// executing a rendering operation.  This API can be called after the
  /// operation completes in order to clear the cache.
  virtual void DisposeThreadLocalCachedResources() {}

  /// @brief Enqueue command_buffer for submission by the end of the frame.
  ///
  /// Certain backends may immediately flush the command buffer if batch
  /// submission is not supported. This functionality is not thread safe
  /// and should only be used via the ContentContext for rendering a
  /// 2D workload.
  ///
  /// Returns true if submission has succeeded. If the buffer is enqueued
  /// then no error may be returned until FlushCommandBuffers is called.
  [[nodiscard]] virtual bool EnqueueCommandBuffer(
      std::shared_ptr<CommandBuffer> command_buffer);

  /// @brief Flush all pending command buffers.
  ///
  /// Returns whether or not submission was successful. This functionality
  /// is not threadsafe and should only be used via the ContentContext for
  /// rendering a 2D workload.
  [[nodiscard]] virtual bool FlushCommandBuffers();

  virtual bool AddTrackingFence(const std::shared_ptr<Texture>& texture) const;

  virtual std::shared_ptr<const IdleWaiter> GetIdleWaiter() const;

  //----------------------------------------------------------------------------
  /// Resets any thread local state that may interfere with embedders.
  ///
  /// Today, only the OpenGL backend can trample on thread local state that the
  /// embedder can access. This call puts the GL state in a sane "clean" state.
  ///
  /// Impeller itself is resilient to a dirty thread local state table.
  ///
  virtual void ResetThreadLocalState() const;

  /// @brief Retrieve the runtime stage for this context type.
  ///
  /// This is used by the engine shell and other subsystems for loading the
  /// correct shader types.
  virtual RuntimeStageBackend GetRuntimeStageBackend() const = 0;

 protected:
  Context();

  std::vector<std::function<void()>> per_frame_task_;

 private:
  Context(const Context&) = delete;

  Context& operator=(const Context&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_CONTEXT_H_
