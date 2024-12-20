// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_SPIRV_SKSL_H_
#define FLUTTER_IMPELLER_COMPILER_SPIRV_SKSL_H_

#include <cstdint>
#include <memory>
#include <utility>
#include <variant>

#include "flutter/fml/logging.h"
#include "spirv_glsl.hpp"

namespace impeller {
namespace compiler {

class CompilerSkSL : public spirv_cross::CompilerGLSL {
 public:
  explicit CompilerSkSL(std::vector<uint32_t> spirv_)
      : CompilerGLSL(std::move(spirv_)) {}

  CompilerSkSL(const uint32_t* ir_, size_t word_count)
      : CompilerGLSL(ir_, word_count) {}

  explicit CompilerSkSL(const spirv_cross::ParsedIR& ir_)
      : spirv_cross::CompilerGLSL(ir_) {}

  explicit CompilerSkSL(spirv_cross::ParsedIR&& ir_)
      : spirv_cross::CompilerGLSL(std::move(ir_)) {}

  std::string compile() override;

 private:
  std::string output_name_;

  void emit_header() override;

  void emit_uniform(const spirv_cross::SPIRVariable& var) override;

  void fixup_user_functions();

  void detect_unsupported_resources();
  bool emit_constant_resources();
  bool emit_struct_resources();
  bool emit_uniform_resources();
  bool emit_output_resources();
  bool emit_global_variable_resources();
  bool emit_undefined_values();
  void emit_resources();

  void emit_interface_block(const spirv_cross::SPIRVariable& var);

  void emit_function_prototype(
      spirv_cross::SPIRFunction& func,
      const spirv_cross::Bitset& return_flags) override;

  std::string image_type_glsl(const spirv_cross::SPIRType& type,
                              uint32_t id = 0,
                              bool member = false) override;

  std::string builtin_to_glsl(spv::BuiltIn builtin,
                              spv::StorageClass storage) override;

  std::string to_texture_op(
      const spirv_cross::Instruction& i,
      bool sparse,
      bool* forward,
      spirv_cross::SmallVector<uint32_t>& inherited_expressions) override;

  std::string to_function_name(
      const spirv_cross::CompilerGLSL::TextureFunctionNameArguments& args)
      override;

  std::string to_function_args(
      const spirv_cross::CompilerGLSL::TextureFunctionArguments& args,
      bool* p_forward) override;
};

}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_SPIRV_SKSL_H_
