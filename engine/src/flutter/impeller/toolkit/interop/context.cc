// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/context.h"

#include <thread>

#include "flutter/fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"

#if IMPELLER_ENABLE_OPENGLES
#include "impeller/entity/gles/entity_shaders_gles.h"
#include "impeller/entity/gles/framebuffer_blend_shaders_gles.h"
#include "impeller/renderer/backend/gles/context_gles.h"
#endif  // IMPELLER_ENABLE_OPENGLES

namespace impeller::interop {

class Context::BackendData {
 public:
  virtual ~BackendData() = default;
};

Context::Context(std::shared_ptr<impeller::Context> context,
                 std::shared_ptr<BackendData> backend_data)
    : context_(std::move(context), TypographerContextSkia::Make()),
      backend_data_(std::move(backend_data)) {}

Context::~Context() = default;

bool Context::IsValid() const {
  return context_.IsValid();
}

std::shared_ptr<impeller::Context> Context::GetContext() const {
  return context_.GetContext();
}

#if IMPELLER_ENABLE_OPENGLES

class ReactorWorker final : public ReactorGLES::Worker,
                            public Context::BackendData {
 public:
  ReactorWorker() : thread_id_(std::this_thread::get_id()) {}

  // |ReactorGLES::Worker|
  ~ReactorWorker() override = default;

  // |ReactorGLES::Worker|
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override {
    return thread_id_ == std::this_thread::get_id();
  }

 private:
  std::thread::id thread_id_;

  FML_DISALLOW_COPY_AND_ASSIGN(ReactorWorker);
};

#endif  // IMPELLER_ENABLE_OPENGLES

ScopedObject<Context> Context::CreateOpenGLES(
    std::function<void*(const char* gl_proc_name)> proc_address_callback) {
#if IMPELLER_ENABLE_OPENGLES
  auto proc_table = std::make_unique<ProcTableGLES>(
      impeller::ProcTableGLES(std::move(proc_address_callback)));
  if (!proc_table || !proc_table->IsValid()) {
    VALIDATION_LOG << "Could not create valid OpenGL ES proc. table.";
    return {};
  }
  std::vector<std::shared_ptr<fml::Mapping>> shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_entity_shaders_gles_data,
          impeller_entity_shaders_gles_length),
      std::make_shared<fml::NonOwnedMapping>(
          impeller_framebuffer_blend_shaders_gles_data,
          impeller_framebuffer_blend_shaders_gles_length),
  };
  auto impeller_context =
      ContextGLES::Create(std::move(proc_table), shader_mappings, false);
  if (!impeller_context) {
    VALIDATION_LOG << "Could not create Impeller context.";
    return {};
  }
  auto reactor_worker = std::make_shared<ReactorWorker>();
  auto worker_id = impeller_context->AddReactorWorker(reactor_worker);
  if (!worker_id.has_value()) {
    VALIDATION_LOG << "Could not add reactor worker.";
    return {};
  }
  auto context =
      Create<Context>(std::move(impeller_context), std::move(reactor_worker));
  if (!context->IsValid()) {
    VALIDATION_LOG << "Could not create valid context.";
    return {};
  }
  return context;
#else   // IMPELLER_ENABLE_OPENGLES
  VALIDATION_LOG << "This build does not support OpenGL ES contexts.";
  return {};
#endif  // IMPELLER_ENABLE_OPENGLES
}

AiksContext& Context::GetAiksContext() {
  return context_;
}

}  // namespace impeller::interop
