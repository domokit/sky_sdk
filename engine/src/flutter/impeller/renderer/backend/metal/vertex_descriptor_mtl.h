// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_VERTEX_DESCRIPTOR_MTL_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_VERTEX_DESCRIPTOR_MTL_H_

#include <Metal/Metal.h>

#include <set>

#include "impeller/base/backend_cast.h"
#include "impeller/renderer/vertex_descriptor.h"

namespace impeller {

class VertexDescriptorMTL {
 public:
  VertexDescriptorMTL();

  ~VertexDescriptorMTL();

  bool SetStageInputsAndLayout(
      const std::vector<ShaderStageIOSlot>& inputs,
      const std::vector<ShaderStageBufferLayout>& layouts);

  MTLVertexDescriptor* GetMTLVertexDescriptor() const;

 private:
  MTLVertexDescriptor* descriptor_;

  VertexDescriptorMTL(const VertexDescriptorMTL&) = delete;

  VertexDescriptorMTL& operator=(const VertexDescriptorMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_METAL_VERTEX_DESCRIPTOR_MTL_H_
