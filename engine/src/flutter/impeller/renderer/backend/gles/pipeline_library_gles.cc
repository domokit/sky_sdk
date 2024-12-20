// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/pipeline_library_gles.h"

#include <sstream>
#include <string>

#include "flutter/fml/container.h"
#include "flutter/fml/trace_event.h"
#include "fml/closure.h"
#include "impeller/base/promise.h"
#include "impeller/renderer/backend/gles/pipeline_gles.h"
#include "impeller/renderer/backend/gles/shader_function_gles.h"

namespace impeller {

PipelineLibraryGLES::PipelineLibraryGLES(std::shared_ptr<ReactorGLES> reactor)
    : reactor_(std::move(reactor)) {}

static std::string GetShaderInfoLog(const ProcTableGLES& gl, GLuint shader) {
  GLint log_length = 0;
  gl.GetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
  if (log_length == 0) {
    return "";
  }
  auto log_buffer =
      reinterpret_cast<char*>(std::calloc(log_length, sizeof(char)));
  gl.GetShaderInfoLog(shader, log_length, &log_length, log_buffer);
  auto log_string = std::string(log_buffer, log_length);
  std::free(log_buffer);
  return log_string;
}

static std::string GetShaderSource(const ProcTableGLES& gl, GLuint shader) {
  // Arbitrarily chosen size that should be larger than most shaders.
  // Since this only fires on compilation errors the performance shouldn't
  // matter.
  auto data = static_cast<char*>(malloc(10240));
  GLsizei length;
  gl.GetShaderSource(shader, 10240, &length, data);

  auto result = std::string{data, static_cast<size_t>(length)};
  free(data);
  return result;
}

static void LogShaderCompilationFailure(const ProcTableGLES& gl,
                                        GLuint shader,
                                        std::string_view name,
                                        const fml::Mapping& source_mapping,
                                        ShaderStage stage) {
  std::stringstream stream;
  stream << "Failed to compile ";
  switch (stage) {
    case ShaderStage::kUnknown:
      stream << "unknown";
      break;
    case ShaderStage::kVertex:
      stream << "vertex";
      break;
    case ShaderStage::kFragment:
      stream << "fragment";
      break;
    case ShaderStage::kCompute:
      stream << "compute";
      break;
  }
  stream << " shader for '" << name << "' with error:" << std::endl;
  stream << GetShaderInfoLog(gl, shader) << std::endl;
  stream << "Shader source was: " << std::endl;
  stream << GetShaderSource(gl, shader) << std::endl;
  VALIDATION_LOG << stream.str();
}

static bool LinkProgram(
    const ReactorGLES& reactor,
    const std::shared_ptr<PipelineGLES>& pipeline,
    const std::shared_ptr<const ShaderFunction>& vert_function,
    const std::shared_ptr<const ShaderFunction>& frag_function) {
  TRACE_EVENT0("impeller", __FUNCTION__);

  const auto& descriptor = pipeline->GetDescriptor();

  auto vert_mapping =
      ShaderFunctionGLES::Cast(*vert_function).GetSourceMapping();
  auto frag_mapping =
      ShaderFunctionGLES::Cast(*frag_function).GetSourceMapping();

  const auto& gl = reactor.GetProcTable();

  auto vert_shader = gl.CreateShader(GL_VERTEX_SHADER);
  auto frag_shader = gl.CreateShader(GL_FRAGMENT_SHADER);

  if (vert_shader == 0 || frag_shader == 0) {
    VALIDATION_LOG << "Could not create shader handles.";
    return false;
  }

  gl.SetDebugLabel(DebugResourceType::kShader, vert_shader,
                   SPrintF("%s Vertex Shader", descriptor.GetLabel().data()));
  gl.SetDebugLabel(DebugResourceType::kShader, frag_shader,
                   SPrintF("%s Fragment Shader", descriptor.GetLabel().data()));

  fml::ScopedCleanupClosure delete_vert_shader(
      [&gl, vert_shader]() { gl.DeleteShader(vert_shader); });
  fml::ScopedCleanupClosure delete_frag_shader(
      [&gl, frag_shader]() { gl.DeleteShader(frag_shader); });

  gl.ShaderSourceMapping(vert_shader, *vert_mapping,
                         descriptor.GetSpecializationConstants());
  gl.ShaderSourceMapping(frag_shader, *frag_mapping,
                         descriptor.GetSpecializationConstants());

  gl.CompileShader(vert_shader);
  gl.CompileShader(frag_shader);

  GLint vert_status = GL_FALSE;
  GLint frag_status = GL_FALSE;

  gl.GetShaderiv(vert_shader, GL_COMPILE_STATUS, &vert_status);
  gl.GetShaderiv(frag_shader, GL_COMPILE_STATUS, &frag_status);

  if (vert_status != GL_TRUE) {
    LogShaderCompilationFailure(gl, vert_shader, descriptor.GetLabel(),
                                *vert_mapping, ShaderStage::kVertex);
    return false;
  }

  if (frag_status != GL_TRUE) {
    LogShaderCompilationFailure(gl, frag_shader, descriptor.GetLabel(),
                                *frag_mapping, ShaderStage::kFragment);
    return false;
  }

  auto program = reactor.GetGLHandle(pipeline->GetProgramHandle());
  if (!program.has_value()) {
    VALIDATION_LOG << "Could not get program handle from reactor.";
    return false;
  }

  gl.AttachShader(*program, vert_shader);
  gl.AttachShader(*program, frag_shader);

  fml::ScopedCleanupClosure detach_vert_shader(
      [&gl, program = *program, vert_shader]() {
        gl.DetachShader(program, vert_shader);
      });
  fml::ScopedCleanupClosure detach_frag_shader(
      [&gl, program = *program, frag_shader]() {
        gl.DetachShader(program, frag_shader);
      });

  for (const auto& stage_input :
       descriptor.GetVertexDescriptor()->GetStageInputs()) {
    gl.BindAttribLocation(*program,                                   //
                          static_cast<GLuint>(stage_input.location),  //
                          stage_input.name                            //
    );
  }

  gl.LinkProgram(*program);

  GLint link_status = GL_FALSE;
  gl.GetProgramiv(*program, GL_LINK_STATUS, &link_status);

  if (link_status != GL_TRUE) {
    VALIDATION_LOG << "Could not link shader program: "
                   << gl.GetProgramInfoLogString(*program);
    return false;
  }
  return true;
}

// |PipelineLibrary|
bool PipelineLibraryGLES::IsValid() const {
  return reactor_ != nullptr;
}

std::shared_ptr<PipelineGLES> PipelineLibraryGLES::CreatePipeline(
    const std::weak_ptr<PipelineLibrary>& weak_library,
    const PipelineDescriptor& desc,
    const std::shared_ptr<const ShaderFunction>& vert_function,
    const std::shared_ptr<const ShaderFunction>& frag_function) {
  auto strong_library = weak_library.lock();

  if (!strong_library) {
    VALIDATION_LOG << "Library was collected before a pending pipeline "
                      "creation could finish.";
    return nullptr;
  }

  auto& library = PipelineLibraryGLES::Cast(*strong_library);

  const auto& reactor = library.GetReactor();

  if (!reactor) {
    return nullptr;
  }

  auto program_key = ProgramKey{vert_function, frag_function,
                                desc.GetSpecializationConstants()};

  auto cached_program = library.GetProgramForKey(program_key);

  const auto has_cached_program = !!cached_program;

  auto pipeline = std::shared_ptr<PipelineGLES>(new PipelineGLES(
      reactor,       //
      weak_library,  //
      desc,          //
      has_cached_program
          ? std::move(cached_program)
          : std::make_shared<UniqueHandleGLES>(UniqueHandleGLES::MakeUntracked(
                reactor, HandleType::kProgram))));

  auto program = reactor->GetGLHandle(pipeline->GetProgramHandle());

  if (!program.has_value()) {
    VALIDATION_LOG << "Could not obtain program handle.";
    return nullptr;
  }

  const auto link_result = !has_cached_program ? LinkProgram(*reactor,       //
                                                             pipeline,       //
                                                             vert_function,  //
                                                             frag_function   //
                                                             )
                                               : true;

  if (!link_result) {
    VALIDATION_LOG << "Could not link pipeline program.";
    return nullptr;
  }

  if (!pipeline->BuildVertexDescriptor(reactor->GetProcTable(),
                                       program.value())) {
    VALIDATION_LOG << "Could not build pipeline vertex descriptors.";
    return nullptr;
  }

  if (!pipeline->IsValid()) {
    VALIDATION_LOG << "Pipeline validation checks failed.";
    return nullptr;
  }

  if (!has_cached_program) {
    library.SetProgramForKey(program_key, pipeline->GetSharedHandle());
  }

  return pipeline;
}

// |PipelineLibrary|
PipelineFuture<PipelineDescriptor> PipelineLibraryGLES::GetPipeline(
    PipelineDescriptor descriptor,
    bool async) {
  if (auto found = pipelines_.find(descriptor); found != pipelines_.end()) {
    return found->second;
  }

  if (!reactor_) {
    return {
        descriptor,
        RealizedFuture<std::shared_ptr<Pipeline<PipelineDescriptor>>>(nullptr)};
  }

  auto vert_function = descriptor.GetEntrypointForStage(ShaderStage::kVertex);
  auto frag_function = descriptor.GetEntrypointForStage(ShaderStage::kFragment);

  if (!vert_function || !frag_function) {
    VALIDATION_LOG
        << "Could not find stage entrypoint functions in pipeline descriptor.";
    return {
        descriptor,
        RealizedFuture<std::shared_ptr<Pipeline<PipelineDescriptor>>>(nullptr)};
  }

  auto promise = std::make_shared<
      std::promise<std::shared_ptr<Pipeline<PipelineDescriptor>>>>();
  auto pipeline_future =
      PipelineFuture<PipelineDescriptor>{descriptor, promise->get_future()};
  pipelines_[descriptor] = pipeline_future;

  const auto result = reactor_->AddOperation([promise,                       //
                                              weak_this = weak_from_this(),  //
                                              descriptor,                    //
                                              vert_function,                 //
                                              frag_function                  //
  ](const ReactorGLES& reactor) {
    promise->set_value(
        CreatePipeline(weak_this, descriptor, vert_function, frag_function));
  });
  FML_CHECK(result);

  return pipeline_future;
}

// |PipelineLibrary|
PipelineFuture<ComputePipelineDescriptor> PipelineLibraryGLES::GetPipeline(
    ComputePipelineDescriptor descriptor,
    bool async) {
  auto promise = std::make_shared<
      std::promise<std::shared_ptr<Pipeline<ComputePipelineDescriptor>>>>();
  promise->set_value(nullptr);
  return {descriptor, promise->get_future()};
}

// |PipelineLibrary|
bool PipelineLibraryGLES::HasPipeline(const PipelineDescriptor& descriptor) {
  return pipelines_.find(descriptor) != pipelines_.end();
}

// |PipelineLibrary|
void PipelineLibraryGLES::RemovePipelinesWithEntryPoint(
    std::shared_ptr<const ShaderFunction> function) {
  fml::erase_if(pipelines_, [&](auto item) {
    return item->first.GetEntrypointForStage(function->GetStage())
        ->IsEqual(*function);
  });
}

// |PipelineLibrary|
PipelineLibraryGLES::~PipelineLibraryGLES() = default;

const std::shared_ptr<ReactorGLES>& PipelineLibraryGLES::GetReactor() const {
  return reactor_;
}

std::shared_ptr<UniqueHandleGLES> PipelineLibraryGLES::GetProgramForKey(
    const ProgramKey& key) {
  Lock lock(programs_mutex_);
  auto found = programs_.find(key);
  if (found != programs_.end()) {
    return found->second;
  }
  return nullptr;
}

void PipelineLibraryGLES::SetProgramForKey(
    const ProgramKey& key,
    std::shared_ptr<UniqueHandleGLES> program) {
  Lock lock(programs_mutex_);
  programs_[key] = std::move(program);
}

}  // namespace impeller
