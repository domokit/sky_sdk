// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_COMPILER_BACKEND_H_
#define FLUTTER_IMPELLER_COMPILER_COMPILER_BACKEND_H_

#include <cstdint>
#include <memory>
#include <variant>

#include "spirv_glsl.hpp"
#include "spirv_msl.hpp"
#include "spirv_sksl.h"

namespace impeller {
namespace compiler {

struct CompilerBackend {
  using MSLCompiler = std::shared_ptr<spirv_cross::CompilerMSL>;
  using GLSLCompiler = std::shared_ptr<spirv_cross::CompilerGLSL>;
  using SkSLCompiler = std::shared_ptr<CompilerSkSL>;
  using Compiler = std::variant<MSLCompiler, GLSLCompiler, SkSLCompiler>;

  enum class Type {
    kMSL,
    kGLSL,
    kGLSLVulkan,
    kSkSL,
  };

  explicit CompilerBackend(MSLCompiler compiler);

  explicit CompilerBackend(GLSLCompiler compiler);

  explicit CompilerBackend(SkSLCompiler compiler);

  CompilerBackend(Type type, Compiler compiler);

  CompilerBackend();

  ~CompilerBackend();

  Type GetType() const;

  const spirv_cross::Compiler* operator->() const;

  spirv_cross::Compiler* GetCompiler();

  explicit operator bool() const;

  enum class ExtendedResourceIndex {
    kPrimary,
    kSecondary,
  };
  uint32_t GetExtendedMSLResourceBinding(ExtendedResourceIndex index,
                                         spirv_cross::ID id) const;

  const spirv_cross::Compiler* GetCompiler() const;

 private:
  Type type_ = Type::kMSL;
  Compiler compiler_;

  const spirv_cross::CompilerMSL* GetMSLCompiler() const;

  const spirv_cross::CompilerGLSL* GetGLSLCompiler() const;

  const CompilerSkSL* GetSkSLCompiler() const;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_COMPILER_BACKEND_H_
