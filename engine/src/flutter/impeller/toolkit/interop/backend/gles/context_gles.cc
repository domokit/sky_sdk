// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/backend/gles/context_gles.h"

#include "impeller/base/validation.h"
#include "impeller/entity/gles/entity_shaders_gles.h"
#include "impeller/entity/gles/framebuffer_blend_shaders_gles.h"
#include "impeller/renderer/backend/gles/context_gles.h"

namespace impeller::interop {

ScopedObject<Context> ContextGLES::Create(
    std::function<void*(const char* gl_proc_name)> proc_address_callback) {
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
  auto impeller_context = impeller::ContextGLES::Create(std::move(proc_table),
                                                        shader_mappings, false);
  if (!impeller_context) {
    VALIDATION_LOG << "Could not create Impeller context.";
    return {};
  }
  auto reactor_worker = std::make_shared<ReactorWorkerGLES>();
  auto worker_id = impeller_context->AddReactorWorker(reactor_worker);
  if (!worker_id.has_value()) {
    VALIDATION_LOG << "Could not add reactor worker.";
    return {};
  }
  return Create(std::move(impeller_context), std::move(reactor_worker));
}

ScopedObject<Context> ContextGLES::Create(
    std::shared_ptr<impeller::Context> impeller_context,
    std::shared_ptr<ReactorWorkerGLES> worker) {
  // Can't call Create because of private constructor. Adopt the raw pointer
  // instead.
  auto context = Adopt<Context>(
      new ContextGLES(std::move(impeller_context), std::move(worker)));

  if (!context->IsValid()) {
    VALIDATION_LOG << "Could not create valid context.";
    return {};
  }
  return context;
}

ContextGLES::ContextGLES(std::shared_ptr<impeller::Context> context,
                         std::shared_ptr<ReactorWorkerGLES> worker)
    : Context(std::move(context)), worker_(std::move(worker)) {}

ContextGLES::~ContextGLES() = default;

}  // namespace impeller::interop
