// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/vertex_descriptor_mtl.h"

#include "flutter/fml/logging.h"
#include "impeller/base/validation.h"

namespace impeller {

VertexDescriptorMTL::VertexDescriptorMTL() = default;

VertexDescriptorMTL::~VertexDescriptorMTL() = default;

static MTLVertexFormat ReadStageInputFormat(const ShaderStageIOSlot& input) {
  if (input.columns != 1) {
    // All matrix types are unsupported as vertex inputs.
    return MTLVertexFormatInvalid;
  }

  switch (input.type) {
    case ShaderType::kFloat: {
      if (input.bit_width == 8 * sizeof(float)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatFloat;
          case 2:
            return MTLVertexFormatFloat2;
          case 3:
            return MTLVertexFormatFloat3;
          case 4:
            return MTLVertexFormatFloat4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kHalfFloat: {
      if (input.bit_width == 8 * sizeof(float) / 2) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatHalf;
          case 2:
            return MTLVertexFormatHalf2;
          case 3:
            return MTLVertexFormatHalf3;
          case 4:
            return MTLVertexFormatHalf4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kDouble: {
      // Unsupported.
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kBoolean: {
      if (input.bit_width == 8 * sizeof(bool) && input.vec_size == 1) {
        return MTLVertexFormatChar;
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kSignedByte: {
      if (input.bit_width == 8 * sizeof(char)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatChar;
          case 2:
            return MTLVertexFormatChar2;
          case 3:
            return MTLVertexFormatChar3;
          case 4:
            return MTLVertexFormatChar4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kUnsignedByte: {
      if (input.bit_width == 8 * sizeof(char)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatUChar;
          case 2:
            return MTLVertexFormatUChar2;
          case 3:
            return MTLVertexFormatUChar3;
          case 4:
            return MTLVertexFormatUChar4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kSignedShort: {
      if (input.bit_width == 8 * sizeof(short)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatShort;
          case 2:
            return MTLVertexFormatShort2;
          case 3:
            return MTLVertexFormatShort3;
          case 4:
            return MTLVertexFormatShort4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kUnsignedShort: {
      if (input.bit_width == 8 * sizeof(ushort)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatUShort;
          case 2:
            return MTLVertexFormatUShort2;
          case 3:
            return MTLVertexFormatUShort3;
          case 4:
            return MTLVertexFormatUShort4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kSignedInt: {
      if (input.bit_width == 8 * sizeof(int32_t)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatInt;
          case 2:
            return MTLVertexFormatInt2;
          case 3:
            return MTLVertexFormatInt3;
          case 4:
            return MTLVertexFormatInt4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kUnsignedInt: {
      if (input.bit_width == 8 * sizeof(uint32_t)) {
        switch (input.vec_size) {
          case 1:
            return MTLVertexFormatUInt;
          case 2:
            return MTLVertexFormatUInt2;
          case 3:
            return MTLVertexFormatUInt3;
          case 4:
            return MTLVertexFormatUInt4;
        }
      }
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kSignedInt64: {
      // Unsupported.
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kUnsignedInt64: {
      // Unsupported.
      return MTLVertexFormatInvalid;
    }
    case ShaderType::kAtomicCounter:
    case ShaderType::kStruct:
    case ShaderType::kImage:
    case ShaderType::kSampledImage:
    case ShaderType::kUnknown:
    case ShaderType::kVoid:
    case ShaderType::kSampler:
      return MTLVertexFormatInvalid;
  }
}

bool VertexDescriptorMTL::SetStageInputsAndLayout(
    const std::vector<ShaderStageIOSlot>& inputs,
    const std::vector<ShaderStageBufferLayout>& layouts) {
  auto descriptor = descriptor_ = [MTLVertexDescriptor vertexDescriptor];

  // TODO(jonahwilliams): its odd that we offset buffers from the max index on
  // metal but not on GLES or Vulkan. We should probably consistently start
  // these at zero?
  for (size_t i = 0; i < inputs.size(); i++) {
    const auto& input = inputs[i];
    auto vertex_format = ReadStageInputFormat(input);
    if (vertex_format == MTLVertexFormatInvalid) {
      VALIDATION_LOG << "Format for input " << input.name << " not supported.";
      return false;
    }
    auto attrib = descriptor.attributes[input.location];
    attrib.format = vertex_format;
    attrib.offset = input.offset;
    attrib.bufferIndex =
        VertexDescriptor::kReservedVertexBufferIndex - input.binding;
  }

  for (size_t i = 0; i < layouts.size(); i++) {
    const auto& layout = layouts[i];
    auto vertex_layout =
        descriptor.layouts[VertexDescriptor::kReservedVertexBufferIndex -
                           layout.binding];
    vertex_layout.stride = layout.stride;
    vertex_layout.stepRate = 1u;
    vertex_layout.stepFunction = MTLVertexStepFunctionPerVertex;
  }
  return true;
}

MTLVertexDescriptor* VertexDescriptorMTL::GetMTLVertexDescriptor() const {
  return descriptor_;
}

}  // namespace impeller
