// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/shader_function_mtl.h"

namespace impeller {

ShaderFunctionMTL::ShaderFunctionMTL(UniqueID parent_library_id,
                                     id<MTLFunction> function,
                                     id<MTLLibrary> library,
                                     std::string name,
                                     ShaderStage stage)
    : ShaderFunction(parent_library_id, std::move(name), stage),
      function_(function),
      library_(library) {}

ShaderFunctionMTL::~ShaderFunctionMTL() = default;

void ShaderFunctionMTL::GetMTLFunctionSpecialized(
    const std::vector<Scalar>& constants,
    const CompileCallback& callback) const {
  MTLFunctionConstantValues* constantValues =
      [[MTLFunctionConstantValues alloc] init];
  size_t index = 0;
  for (const auto value : constants) {
    Scalar copied_value = value;
    [constantValues setConstantValue:&copied_value
                                type:MTLDataTypeFloat
                             atIndex:index];
    index++;
  }
  CompileCallback callback_value = callback;
  [library_ newFunctionWithName:@(GetName().data())
                 constantValues:constantValues
              completionHandler:^(id<MTLFunction> _Nullable function,
                                  NSError* _Nullable error) {
                callback_value(function);
              }];
}

id<MTLFunction> ShaderFunctionMTL::GetMTLFunction() const {
  return function_;
}

}  // namespace impeller
