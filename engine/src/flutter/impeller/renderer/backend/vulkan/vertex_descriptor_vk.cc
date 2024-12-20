// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/vertex_descriptor_vk.h"

#include <cstdint>

namespace impeller {

vk::Format ToVertexDescriptorFormat(const ShaderStageIOSlot& input) {
  if (input.columns != 1) {
    // All matrix types are unsupported as vertex inputs.
    return vk::Format::eUndefined;
  }

  switch (input.type) {
    case ShaderType::kFloat: {
      if (input.bit_width == 8 * sizeof(float)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR32Sfloat;
          case 2:
            return vk::Format::eR32G32Sfloat;
          case 3:
            return vk::Format::eR32G32B32Sfloat;
          case 4:
            return vk::Format::eR32G32B32A32Sfloat;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kHalfFloat: {
      if (input.bit_width == 8 * sizeof(float) / 2) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR16Sfloat;
          case 2:
            return vk::Format::eR16G16Sfloat;
          case 3:
            return vk::Format::eR16G16B16Sfloat;
          case 4:
            return vk::Format::eR16G16B16A16Sfloat;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kDouble: {
      // Unsupported.
      return vk::Format::eUndefined;
    }
    case ShaderType::kBoolean: {
      if (input.bit_width == 8 * sizeof(bool) && input.vec_size == 1) {
        return vk::Format::eR8Uint;
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kSignedByte: {
      if (input.bit_width == 8 * sizeof(char)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR8Sint;
          case 2:
            return vk::Format::eR8G8Sint;
          case 3:
            return vk::Format::eR8G8B8Sint;
          case 4:
            return vk::Format::eR8G8B8A8Sint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kUnsignedByte: {
      if (input.bit_width == 8 * sizeof(char)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR8Uint;
          case 2:
            return vk::Format::eR8G8Uint;
          case 3:
            return vk::Format::eR8G8B8Uint;
          case 4:
            return vk::Format::eR8G8B8A8Uint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kSignedShort: {
      if (input.bit_width == 8 * sizeof(int16_t)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR16Sint;
          case 2:
            return vk::Format::eR16G16Sint;
          case 3:
            return vk::Format::eR16G16B16Sint;
          case 4:
            return vk::Format::eR16G16B16A16Sint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kUnsignedShort: {
      if (input.bit_width == 8 * sizeof(uint16_t)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR16Uint;
          case 2:
            return vk::Format::eR16G16Uint;
          case 3:
            return vk::Format::eR16G16B16Uint;
          case 4:
            return vk::Format::eR16G16B16A16Uint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kSignedInt: {
      if (input.bit_width == 8 * sizeof(int32_t)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR32Sint;
          case 2:
            return vk::Format::eR32G32Sint;
          case 3:
            return vk::Format::eR32G32B32Sint;
          case 4:
            return vk::Format::eR32G32B32A32Sint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kUnsignedInt: {
      if (input.bit_width == 8 * sizeof(uint32_t)) {
        switch (input.vec_size) {
          case 1:
            return vk::Format::eR32Uint;
          case 2:
            return vk::Format::eR32G32Uint;
          case 3:
            return vk::Format::eR32G32B32Uint;
          case 4:
            return vk::Format::eR32G32B32A32Uint;
        }
      }
      return vk::Format::eUndefined;
    }
    case ShaderType::kSignedInt64: {
      // Unsupported.
      return vk::Format::eUndefined;
    }
    case ShaderType::kUnsignedInt64: {
      // Unsupported.
      return vk::Format::eUndefined;
    }
    case ShaderType::kAtomicCounter:
    case ShaderType::kStruct:
    case ShaderType::kImage:
    case ShaderType::kSampledImage:
    case ShaderType::kUnknown:
    case ShaderType::kVoid:
    case ShaderType::kSampler:
      return vk::Format::eUndefined;
  }
}

}  // namespace impeller
