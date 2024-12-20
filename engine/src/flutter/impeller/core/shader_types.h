// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_SHADER_TYPES_H_
#define FLUTTER_IMPELLER_CORE_SHADER_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "flutter/fml/hash_combine.h"
#include "flutter/fml/logging.h"
#include "impeller/core/runtime_types.h"
#include "impeller/geometry/half.h"
#include "impeller/geometry/matrix.h"

namespace impeller {

enum class ShaderStage {
  kUnknown,
  kVertex,
  kFragment,
  kCompute,
};

constexpr ShaderStage ToShaderStage(RuntimeShaderStage stage) {
  switch (stage) {
    case RuntimeShaderStage::kVertex:
      return ShaderStage::kVertex;
    case RuntimeShaderStage::kFragment:
      return ShaderStage::kFragment;
    case RuntimeShaderStage::kCompute:
      return ShaderStage::kCompute;
  }
  FML_UNREACHABLE();
}

enum class ShaderType {
  kUnknown,
  kVoid,
  kBoolean,
  kSignedByte,
  kUnsignedByte,
  kSignedShort,
  kUnsignedShort,
  kSignedInt,
  kUnsignedInt,
  kSignedInt64,
  kUnsignedInt64,
  kAtomicCounter,
  kHalfFloat,
  kFloat,
  kDouble,
  kStruct,
  kImage,
  kSampledImage,
  kSampler,
};

struct ShaderStructMemberMetadata {
  ShaderType type;
  std::string name;
  size_t offset;
  size_t size;
  size_t byte_length;
  std::optional<size_t> array_elements;
};

struct ShaderMetadata {
  // This must match the uniform name in the shader program.
  std::string name;
  std::vector<ShaderStructMemberMetadata> members;
};

/// @brief Metadata required to bind a buffer.
///
/// OpenGL binding requires the usage of the separate shader metadata struct.
struct ShaderUniformSlot {
  /// @brief The name of the uniform slot.
  const char* name;

  /// @brief `ext_res_0` is the Metal binding value.
  size_t ext_res_0;

  /// @brief The Vulkan descriptor set index.
  size_t set;

  /// @brief The Vulkan binding value.
  size_t binding;
};

/// @brief Metadata required to bind a combined texture and sampler.
///
/// OpenGL binding requires the usage of the separate shader metadata struct.
struct SampledImageSlot {
  /// @brief The name of the uniform slot.
  const char* name;

  /// @brief `ext_res_0` is the Metal binding value.
  size_t texture_index;

  /// @brief The Vulkan descriptor set index.
  size_t set;

  /// @brief The Vulkan binding value.
  size_t binding;
};

struct ShaderStageIOSlot {
  const char* name;
  size_t location;
  size_t set;
  size_t binding;
  ShaderType type;
  size_t bit_width;
  size_t vec_size;
  size_t columns;
  size_t offset;
  bool relaxed_precision;

  constexpr size_t GetHash() const {
    return fml::HashCombine(name, location, set, binding, type, bit_width,
                            vec_size, columns, offset, relaxed_precision);
  }

  constexpr bool operator==(const ShaderStageIOSlot& other) const {
    return name == other.name &&                         //
           location == other.location &&                 //
           set == other.set &&                           //
           binding == other.binding &&                   //
           type == other.type &&                         //
           bit_width == other.bit_width &&               //
           vec_size == other.vec_size &&                 //
           columns == other.columns &&                   //
           offset == other.offset &&                     //
           relaxed_precision == other.relaxed_precision  //
        ;
  }
};

struct ShaderStageBufferLayout {
  size_t stride;
  size_t binding;

  constexpr size_t GetHash() const { return fml::HashCombine(stride, binding); }

  constexpr bool operator==(const ShaderStageBufferLayout& other) const {
    return stride == other.stride &&  //
           binding == other.binding;
  }
};

// These enum values were chosen to match the same values
// in the VK Descriptor Type enum.
enum class DescriptorType {
  kSampler = 0,
  kSampledImage = 1,
  kImage = 2,
  kUniformBuffer = 6,
  kStorageBuffer = 7,
  kInputAttachment = 10,
};

struct DescriptorSetLayout {
  uint32_t binding;
  DescriptorType descriptor_type;
  ShaderStage shader_stage;
};

template <size_t Size>
struct Padding {
 private:
  uint8_t pad_[Size];
};

/// @brief Struct used for padding uniform buffer array elements.
template <typename T,
          size_t Size,
          class = std::enable_if_t<std::is_standard_layout_v<T>>>
struct Padded {
  T value;
  Padding<Size> _PADDING_;

  Padded(T p_value) : value(p_value){};  // NOLINT(google-explicit-constructor)
};

inline constexpr Vector4 ToVector(Color color) {
  return {color.red, color.green, color.blue, color.alpha};
}

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_SHADER_TYPES_H_
