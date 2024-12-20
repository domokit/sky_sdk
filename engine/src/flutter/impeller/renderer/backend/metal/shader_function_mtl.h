// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_FUNCTION_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_FUNCTION_MTL_H_

#include <Metal/Metal.h>

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/shader_function.h"

namespace impeller {

class ShaderFunctionMTL final
    : public ShaderFunction,
      public BackendCast<ShaderFunctionMTL, ShaderFunction> {
 public:
  // |ShaderFunction|
  ~ShaderFunctionMTL() override;

  id<MTLFunction> GetMTLFunction() const;

  using CompileCallback = std::function<void(id<MTLFunction>)>;

  void GetMTLFunctionSpecialized(const std::vector<Scalar>& constants,
                                 const CompileCallback& callback) const;

 private:
  friend class ShaderLibraryMTL;

  id<MTLFunction> function_ = nullptr;
  id<MTLLibrary> library_ = nullptr;

  ShaderFunctionMTL(UniqueID parent_library_id,
                    id<MTLFunction> function,
                    id<MTLLibrary> library,
                    std::string name,
                    ShaderStage stage);

  ShaderFunctionMTL(const ShaderFunctionMTL&) = delete;

  ShaderFunctionMTL& operator=(const ShaderFunctionMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_FUNCTION_MTL_H_
