// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_SKIA_GPU_OBJECT_H_
#define FLUTTER_FLOW_SKIA_GPU_OBJECT_H_

#include <mutex>
#include <queue>

#include "flutter/common/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/trace_event.h"
#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/GrTypes.h"

namespace flutter {

// A queue that holds Skia objects that must be destructed on the given task
// runner.
template <class T>
class UnrefQueue : public fml::RefCountedThreadSafe<UnrefQueue<T>> {
 public:
  using ResourceContext = T;

  void Unref(SkRefCnt* object) {
    if (drain_immediate_) {
      object->unref();
      return;
    }
    std::scoped_lock lock(mutex_);
    objects_.push_back(object);
    if (!drain_pending_) {
      drain_pending_ = true;
      task_runner_->PostDelayedTask(
          [strong = fml::Ref(this)]() { strong->Drain(); }, drain_delay_);
    }
  }

#if !SLIMPELLER
  void DeleteTexture(const GrBackendTexture& texture) {
    // drain_immediate_ should only be used on Impeller.
    FML_DCHECK(!drain_immediate_);
    std::scoped_lock lock(mutex_);
    textures_.push_back(texture);
    if (!drain_pending_) {
      drain_pending_ = true;
      task_runner_->PostDelayedTask(
          [strong = fml::Ref(this)]() { strong->Drain(); }, drain_delay_);
    }
  }
#endif  //  !SLIMPELLER

  // Usually, the drain is called automatically. However, during IO manager
  // shutdown (when the platform side reference to the OpenGL context is about
  // to go away), we may need to pre-emptively drain the unref queue. It is the
  // responsibility of the caller to ensure that no further unrefs are queued
  // after this call.
  void Drain() {
    TRACE_EVENT0("flutter", "SkiaUnrefQueue::Drain");
    std::deque<SkRefCnt*> skia_objects;

    NOT_SLIMPELLER(std::deque<GrBackendTexture> textures);

    {
      std::scoped_lock lock(mutex_);
      objects_.swap(skia_objects);
      NOT_SLIMPELLER(textures_.swap(textures));
      drain_pending_ = false;
    }
    DoDrain(skia_objects,
#if !SLIMPELLER
            textures,
#endif  //  !SLIMPELLER
            context_);
  }

  void UpdateResourceContext(sk_sp<ResourceContext> context) {
    context_ = context;
  }

 private:
  const fml::RefPtr<fml::TaskRunner> task_runner_;
  const fml::TimeDelta drain_delay_;
  std::mutex mutex_;
  std::deque<SkRefCnt*> objects_;
  NOT_SLIMPELLER(std::deque<GrBackendTexture> textures_);
  bool drain_pending_ = false;
  sk_sp<ResourceContext> context_;
  // Enabled when there is an impeller context, which removes the usage of
  // the queue altogether.
  bool drain_immediate_;

  // The `GrDirectContext* context` is only used for signaling Skia to
  // performDeferredCleanup. It can be nullptr when such signaling is not needed
  // (e.g., in unit tests).
  UnrefQueue(fml::RefPtr<fml::TaskRunner> task_runner,
             fml::TimeDelta delay,
             sk_sp<ResourceContext> context = nullptr,
             bool drain_immediate = false)
      : task_runner_(std::move(task_runner)),
        drain_delay_(delay),
        context_(context),
        drain_immediate_(drain_immediate) {}

  ~UnrefQueue() {
    // The ResourceContext must be deleted on the task runner thread.
    // Transfer ownership of the UnrefQueue's ResourceContext reference
    // into a task queued to that thread.
    ResourceContext* raw_context = context_.release();
    fml::TaskRunner::RunNowOrPostTask(
        task_runner_, [objects = std::move(objects_),
#if !SLIMPELLER
                       textures = std::move(textures_),
#endif  //  !SLIMPELLER
                       raw_context]() mutable {
          sk_sp<ResourceContext> context(raw_context);
          DoDrain(objects,
#if !SLIMPELLER
                  textures,
#endif  //  !SLIMPELLER
                  context);
          context.reset();
        });
  }

  // static
  static void DoDrain(const std::deque<SkRefCnt*>& skia_objects,
#if !SLIMPELLER
                      const std::deque<GrBackendTexture>& textures,
#endif  //  !SLIMPELLER
                      sk_sp<ResourceContext> context) {
    for (SkRefCnt* skia_object : skia_objects) {
      skia_object->unref();
    }

#if !SLIMPELLER
    if (context) {
      for (const GrBackendTexture& texture : textures) {
        context->deleteBackendTexture(texture);
      }

      if (!skia_objects.empty()) {
        context->performDeferredCleanup(std::chrono::milliseconds(0));
      }

      context->flushAndSubmit(GrSyncCpu::kYes);
    }
#endif  //  !SLIMPELLER
  }

  FML_FRIEND_REF_COUNTED_THREAD_SAFE(UnrefQueue);
  FML_FRIEND_MAKE_REF_COUNTED(UnrefQueue);
  FML_DISALLOW_COPY_AND_ASSIGN(UnrefQueue);
};

using SkiaUnrefQueue = UnrefQueue<GrDirectContext>;

/// An object whose deallocation needs to be performed on an specific unref
/// queue.
template <class T>
class SkiaGPUObject {
 public:
  using SkiaObjectType = T;

  SkiaGPUObject() = default;
  SkiaGPUObject(sk_sp<SkiaObjectType> object, fml::RefPtr<SkiaUnrefQueue> queue)
      : object_(std::move(object)), queue_(std::move(queue)) {
    FML_DCHECK(object_);
  }
  SkiaGPUObject(SkiaGPUObject&&) = default;
  ~SkiaGPUObject() { reset(); }

  SkiaGPUObject& operator=(SkiaGPUObject&&) = default;

  sk_sp<SkiaObjectType> skia_object() const { return object_; }

  void reset() {
    if (object_ && queue_) {
      queue_->Unref(object_.release());
    }
    queue_ = nullptr;
    FML_DCHECK(object_ == nullptr);
  }

 private:
  sk_sp<SkiaObjectType> object_;
  fml::RefPtr<SkiaUnrefQueue> queue_;

  FML_DISALLOW_COPY_AND_ASSIGN(SkiaGPUObject);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_SKIA_GPU_OBJECT_H_
