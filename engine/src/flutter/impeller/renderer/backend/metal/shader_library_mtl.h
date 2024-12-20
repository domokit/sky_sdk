// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_LIBRARY_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_LIBRARY_MTL_H_

#include <Foundation/Foundation.h>
#include <Metal/Metal.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "impeller/base/comparable.h"
#include "impeller/base/thread.h"
#include "impeller/renderer/shader_key.h"
#include "impeller/renderer/shader_library.h"

namespace impeller {

class ShaderLibraryMTL final : public ShaderLibrary {
 public:
  ShaderLibraryMTL();

  // |ShaderLibrary|
  ~ShaderLibraryMTL() override;

  // |ShaderLibrary|
  bool IsValid() const override;

 private:
  friend class ContextMTL;

  UniqueID library_id_;
  mutable RWMutex libraries_mutex_;
  NSMutableArray<id<MTLLibrary>>* libraries_ IPLR_GUARDED_BY(libraries_mutex_) =
      nullptr;
  ShaderFunctionMap functions_;
  bool is_valid_ = false;

  explicit ShaderLibraryMTL(NSArray<id<MTLLibrary>>* libraries);

  // |ShaderLibrary|
  std::shared_ptr<const ShaderFunction> GetFunction(std::string_view name,
                                                    ShaderStage stage) override;

  // |ShaderLibrary|
  void RegisterFunction(std::string name,
                        ShaderStage stage,
                        std::shared_ptr<fml::Mapping> code,
                        RegistrationCallback callback) override;

  // |ShaderLibrary|
  void UnregisterFunction(std::string name, ShaderStage stage) override;

  id<MTLDevice> GetDevice() const;

  void RegisterLibrary(id<MTLLibrary> library);

  ShaderLibraryMTL(const ShaderLibraryMTL&) = delete;

  ShaderLibraryMTL& operator=(const ShaderLibraryMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_SHADER_LIBRARY_MTL_H_
