// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_ALLOCATOR_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_ALLOCATOR_GLES_H_

#include "impeller/core/allocator.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"

namespace impeller {

class AllocatorGLES final : public Allocator {
 public:
  // |Allocator|
  ~AllocatorGLES() override;

 private:
  friend class ContextGLES;

  std::shared_ptr<ReactorGLES> reactor_;
  bool is_valid_ = false;

  explicit AllocatorGLES(std::shared_ptr<ReactorGLES> reactor);

  // |Allocator|
  bool IsValid() const;

  // |Allocator|
  std::shared_ptr<DeviceBuffer> OnCreateBuffer(
      const DeviceBufferDescriptor& desc) override;

  // |Allocator|
  std::shared_ptr<Texture> OnCreateTexture(
      const TextureDescriptor& desc) override;

  // |Allocator|
  ISize GetMaxTextureSizeSupported() const override;

  AllocatorGLES(const AllocatorGLES&) = delete;

  AllocatorGLES& operator=(const AllocatorGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_ALLOCATOR_GLES_H_
