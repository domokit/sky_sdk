// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SHADER_LIBRARY_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SHADER_LIBRARY_GLES_H_

#include <memory>

#include "flutter/fml/mapping.h"
#include "impeller/base/comparable.h"
#include "impeller/base/thread.h"
#include "impeller/renderer/shader_key.h"
#include "impeller/renderer/shader_library.h"

namespace impeller {

class ShaderLibraryGLES final : public ShaderLibrary {
 public:
  // |ShaderLibrary|
  ~ShaderLibraryGLES() override;

  // |ShaderLibrary|
  bool IsValid() const override;

 private:
  friend class ContextGLES;
  const UniqueID library_id_;
  mutable RWMutex functions_mutex_;
  ShaderFunctionMap functions_ IPLR_GUARDED_BY(functions_mutex_);
  bool is_valid_ = false;

  explicit ShaderLibraryGLES(
      const std::vector<std::shared_ptr<fml::Mapping>>& shader_libraries);

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

  ShaderLibraryGLES(const ShaderLibraryGLES&) = delete;

  ShaderLibraryGLES& operator=(const ShaderLibraryGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_SHADER_LIBRARY_GLES_H_
