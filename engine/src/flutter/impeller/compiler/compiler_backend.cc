// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/compiler/compiler_backend.h"

#include "impeller/base/comparable.h"

namespace impeller {
namespace compiler {

CompilerBackend::CompilerBackend(MSLCompiler compiler)
    : CompilerBackend(Type::kMSL, compiler) {}

CompilerBackend::CompilerBackend(GLSLCompiler compiler)
    : CompilerBackend(compiler->get_common_options().vulkan_semantics
                          ? Type::kGLSLVulkan
                          : Type::kGLSL,
                      compiler) {}

CompilerBackend::CompilerBackend(SkSLCompiler compiler)
    : CompilerBackend(Type::kSkSL, compiler) {}

CompilerBackend::CompilerBackend() = default;

CompilerBackend::CompilerBackend(Type type, Compiler compiler)
    : type_(type), compiler_(std::move(compiler)){};

CompilerBackend::~CompilerBackend() = default;

const spirv_cross::Compiler* CompilerBackend::operator->() const {
  return GetCompiler();
}

uint32_t CompilerBackend::GetExtendedMSLResourceBinding(
    ExtendedResourceIndex index,
    spirv_cross::ID id) const {
  if (auto compiler = GetMSLCompiler()) {
    switch (index) {
      case ExtendedResourceIndex::kPrimary:
        return compiler->get_automatic_msl_resource_binding(id);
      case ExtendedResourceIndex::kSecondary:
        return compiler->get_automatic_msl_resource_binding_secondary(id);
        break;
    }
  }
  if (auto compiler = GetGLSLCompiler()) {
    return compiler->get_decoration(id, spv::Decoration::DecorationBinding);
  }
  const auto kOOBIndex = static_cast<uint32_t>(-1);
  return kOOBIndex;
}

const spirv_cross::Compiler* CompilerBackend::GetCompiler() const {
  if (auto compiler = GetGLSLCompiler()) {
    return compiler;
  }

  if (auto compiler = GetMSLCompiler()) {
    return compiler;
  }

  if (auto compiler = GetSkSLCompiler()) {
    return compiler;
  }

  return nullptr;
}

spirv_cross::Compiler* CompilerBackend::GetCompiler() {
  if (auto* msl = std::get_if<MSLCompiler>(&compiler_)) {
    return msl->get();
  }
  if (auto* glsl = std::get_if<GLSLCompiler>(&compiler_)) {
    return glsl->get();
  }
  if (auto* sksl = std::get_if<SkSLCompiler>(&compiler_)) {
    return sksl->get();
  }
  return nullptr;
}

const spirv_cross::CompilerMSL* CompilerBackend::GetMSLCompiler() const {
  if (auto* msl = std::get_if<MSLCompiler>(&compiler_)) {
    return msl->get();
  }
  return nullptr;
}

const spirv_cross::CompilerGLSL* CompilerBackend::GetGLSLCompiler() const {
  if (auto* glsl = std::get_if<GLSLCompiler>(&compiler_)) {
    return glsl->get();
  }
  return nullptr;
}

const CompilerSkSL* CompilerBackend::GetSkSLCompiler() const {
  if (auto* sksl = std::get_if<SkSLCompiler>(&compiler_)) {
    return sksl->get();
  }
  return nullptr;
}

CompilerBackend::operator bool() const {
  return !!GetCompiler();
}

CompilerBackend::Type CompilerBackend::GetType() const {
  return type_;
}

}  // namespace compiler
}  // namespace impeller
