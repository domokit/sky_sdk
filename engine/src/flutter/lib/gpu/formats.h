// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_GPU_FORMATS_H_
#define FLUTTER_LIB_GPU_FORMATS_H_

#include "fml/logging.h"
#include "impeller/core/formats.h"
#include "impeller/core/shader_types.h"

// ATTENTION! ATTENTION! ATTENTION!
// All enums defined in this file must exactly match the contents and order of
// the corresponding enums defined in `gpu/lib/src/formats.dart`.

namespace flutter {
namespace gpu {

enum class FlutterGPUStorageMode {
  kHostVisible,
  kDevicePrivate,
  kDeviceTransient,
};

constexpr impeller::StorageMode ToImpellerStorageMode(
    FlutterGPUStorageMode value) {
  switch (value) {
    case FlutterGPUStorageMode::kHostVisible:
      return impeller::StorageMode::kHostVisible;
    case FlutterGPUStorageMode::kDevicePrivate:
      return impeller::StorageMode::kDevicePrivate;
    case FlutterGPUStorageMode::kDeviceTransient:
      return impeller::StorageMode::kDeviceTransient;
  }
}

constexpr impeller::StorageMode ToImpellerStorageMode(int value) {
  return ToImpellerStorageMode(static_cast<FlutterGPUStorageMode>(value));
}

enum class FlutterGPUPixelFormat {
  kUnknown,
  kA8UNormInt,
  kR8UNormInt,
  kR8G8UNormInt,
  kR8G8B8A8UNormInt,
  kR8G8B8A8UNormIntSRGB,
  kB8G8R8A8UNormInt,
  kB8G8R8A8UNormIntSRGB,
  kR32G32B32A32Float,
  kR16G16B16A16Float,
  kB10G10R10XR,
  kB10G10R10XRSRGB,
  kB10G10R10A10XR,
  kS8UInt,
  kD24UnormS8Uint,
  kD32FloatS8UInt,
};

constexpr impeller::PixelFormat ToImpellerPixelFormat(
    FlutterGPUPixelFormat value) {
  switch (value) {
    case FlutterGPUPixelFormat::kUnknown:
      return impeller::PixelFormat::kUnknown;
    case FlutterGPUPixelFormat::kA8UNormInt:
      return impeller::PixelFormat::kA8UNormInt;
    case FlutterGPUPixelFormat::kR8UNormInt:
      return impeller::PixelFormat::kR8UNormInt;
    case FlutterGPUPixelFormat::kR8G8UNormInt:
      return impeller::PixelFormat::kR8G8UNormInt;
    case FlutterGPUPixelFormat::kR8G8B8A8UNormInt:
      return impeller::PixelFormat::kR8G8B8A8UNormInt;
    case FlutterGPUPixelFormat::kR8G8B8A8UNormIntSRGB:
      return impeller::PixelFormat::kR8G8B8A8UNormIntSRGB;
    case FlutterGPUPixelFormat::kB8G8R8A8UNormInt:
      return impeller::PixelFormat::kB8G8R8A8UNormInt;
    case FlutterGPUPixelFormat::kB8G8R8A8UNormIntSRGB:
      return impeller::PixelFormat::kB8G8R8A8UNormIntSRGB;
    case FlutterGPUPixelFormat::kR32G32B32A32Float:
      return impeller::PixelFormat::kR32G32B32A32Float;
    case FlutterGPUPixelFormat::kR16G16B16A16Float:
      return impeller::PixelFormat::kR16G16B16A16Float;
    case FlutterGPUPixelFormat::kB10G10R10XR:
      return impeller::PixelFormat::kB10G10R10XR;
    case FlutterGPUPixelFormat::kB10G10R10XRSRGB:
      return impeller::PixelFormat::kB10G10R10XRSRGB;
    case FlutterGPUPixelFormat::kB10G10R10A10XR:
      return impeller::PixelFormat::kB10G10R10A10XR;
    case FlutterGPUPixelFormat::kS8UInt:
      return impeller::PixelFormat::kS8UInt;
    case FlutterGPUPixelFormat::kD24UnormS8Uint:
      return impeller::PixelFormat::kD24UnormS8Uint;
    case FlutterGPUPixelFormat::kD32FloatS8UInt:
      return impeller::PixelFormat::kD32FloatS8UInt;
  }
}

constexpr impeller::PixelFormat ToImpellerPixelFormat(int value) {
  return ToImpellerPixelFormat(static_cast<FlutterGPUPixelFormat>(value));
}

constexpr FlutterGPUPixelFormat FromImpellerPixelFormat(
    impeller::PixelFormat value) {
  switch (value) {
    case impeller::PixelFormat::kUnknown:
      return FlutterGPUPixelFormat::kUnknown;
    case impeller::PixelFormat::kA8UNormInt:
      return FlutterGPUPixelFormat::kA8UNormInt;
    case impeller::PixelFormat::kR8UNormInt:
      return FlutterGPUPixelFormat::kR8UNormInt;
    case impeller::PixelFormat::kR8G8UNormInt:
      return FlutterGPUPixelFormat::kR8G8UNormInt;
    case impeller::PixelFormat::kR8G8B8A8UNormInt:
      return FlutterGPUPixelFormat::kR8G8B8A8UNormInt;
    case impeller::PixelFormat::kR8G8B8A8UNormIntSRGB:
      return FlutterGPUPixelFormat::kR8G8B8A8UNormIntSRGB;
    case impeller::PixelFormat::kB8G8R8A8UNormInt:
      return FlutterGPUPixelFormat::kB8G8R8A8UNormInt;
    case impeller::PixelFormat::kB8G8R8A8UNormIntSRGB:
      return FlutterGPUPixelFormat::kB8G8R8A8UNormIntSRGB;
    case impeller::PixelFormat::kR32G32B32A32Float:
      return FlutterGPUPixelFormat::kR32G32B32A32Float;
    case impeller::PixelFormat::kR16G16B16A16Float:
      return FlutterGPUPixelFormat::kR16G16B16A16Float;
    case impeller::PixelFormat::kB10G10R10XR:
      return FlutterGPUPixelFormat::kB10G10R10XR;
    case impeller::PixelFormat::kB10G10R10XRSRGB:
      return FlutterGPUPixelFormat::kB10G10R10XRSRGB;
    case impeller::PixelFormat::kB10G10R10A10XR:
      return FlutterGPUPixelFormat::kB10G10R10A10XR;
    case impeller::PixelFormat::kS8UInt:
      return FlutterGPUPixelFormat::kS8UInt;
    case impeller::PixelFormat::kD24UnormS8Uint:
      return FlutterGPUPixelFormat::kD24UnormS8Uint;
    case impeller::PixelFormat::kD32FloatS8UInt:
      return FlutterGPUPixelFormat::kD32FloatS8UInt;
  }
}

enum class FlutterGPUTextureCoordinateSystem {
  kUploadFromHost,
  kRenderToTexture,
};

constexpr impeller::TextureCoordinateSystem ToImpellerTextureCoordinateSystem(
    FlutterGPUTextureCoordinateSystem value) {
  switch (value) {
    case FlutterGPUTextureCoordinateSystem::kUploadFromHost:
      return impeller::TextureCoordinateSystem::kUploadFromHost;
    case FlutterGPUTextureCoordinateSystem::kRenderToTexture:
      return impeller::TextureCoordinateSystem::kRenderToTexture;
  }
}

constexpr impeller::TextureCoordinateSystem ToImpellerTextureCoordinateSystem(
    int value) {
  return ToImpellerTextureCoordinateSystem(
      static_cast<FlutterGPUTextureCoordinateSystem>(value));
}

enum class FlutterGPUBlendFactor {
  kZero,
  kOne,
  kSourceColor,
  kOneMinusSourceColor,
  kSourceAlpha,
  kOneMinusSourceAlpha,
  kDestinationColor,
  kOneMinusDestinationColor,
  kDestinationAlpha,
  kOneMinusDestinationAlpha,
  kSourceAlphaSaturated,
  kBlendColor,
  kOneMinusBlendColor,
  kBlendAlpha,
  kOneMinusBlendAlpha,
};

constexpr impeller::BlendFactor ToImpellerBlendFactor(
    FlutterGPUBlendFactor value) {
  switch (value) {
    case FlutterGPUBlendFactor::kZero:
      return impeller::BlendFactor::kZero;
    case FlutterGPUBlendFactor::kOne:
      return impeller::BlendFactor::kOne;
    case FlutterGPUBlendFactor::kSourceColor:
      return impeller::BlendFactor::kSourceColor;
    case FlutterGPUBlendFactor::kOneMinusSourceColor:
      return impeller::BlendFactor::kOneMinusSourceColor;
    case FlutterGPUBlendFactor::kSourceAlpha:
      return impeller::BlendFactor::kSourceAlpha;
    case FlutterGPUBlendFactor::kOneMinusSourceAlpha:
      return impeller::BlendFactor::kOneMinusSourceAlpha;
    case FlutterGPUBlendFactor::kDestinationColor:
      return impeller::BlendFactor::kDestinationColor;
    case FlutterGPUBlendFactor::kOneMinusDestinationColor:
      return impeller::BlendFactor::kOneMinusDestinationColor;
    case FlutterGPUBlendFactor::kDestinationAlpha:
      return impeller::BlendFactor::kDestinationAlpha;
    case FlutterGPUBlendFactor::kOneMinusDestinationAlpha:
      return impeller::BlendFactor::kOneMinusDestinationAlpha;
    case FlutterGPUBlendFactor::kSourceAlphaSaturated:
      return impeller::BlendFactor::kSourceAlphaSaturated;
    case FlutterGPUBlendFactor::kBlendColor:
      return impeller::BlendFactor::kBlendColor;
    case FlutterGPUBlendFactor::kOneMinusBlendColor:
      return impeller::BlendFactor::kOneMinusBlendColor;
    case FlutterGPUBlendFactor::kBlendAlpha:
      return impeller::BlendFactor::kBlendAlpha;
    case FlutterGPUBlendFactor::kOneMinusBlendAlpha:
      return impeller::BlendFactor::kOneMinusBlendAlpha;
  }
}

constexpr impeller::BlendFactor ToImpellerBlendFactor(int value) {
  return ToImpellerBlendFactor(static_cast<FlutterGPUBlendFactor>(value));
}

enum class FlutterGPUBlendOperation {
  kAdd,
  kSubtract,
  kReverseSubtract,
};

constexpr impeller::BlendOperation ToImpellerBlendOperation(
    FlutterGPUBlendOperation value) {
  switch (value) {
    case FlutterGPUBlendOperation::kAdd:
      return impeller::BlendOperation::kAdd;
    case FlutterGPUBlendOperation::kSubtract:
      return impeller::BlendOperation::kSubtract;
    case FlutterGPUBlendOperation::kReverseSubtract:
      return impeller::BlendOperation::kReverseSubtract;
  }
}

constexpr impeller::BlendOperation ToImpellerBlendOperation(int value) {
  return ToImpellerBlendOperation(static_cast<FlutterGPUBlendOperation>(value));
}

enum class FlutterGPULoadAction {
  kDontCare,
  kLoad,
  kClear,
};

constexpr impeller::LoadAction ToImpellerLoadAction(
    FlutterGPULoadAction value) {
  switch (value) {
    case FlutterGPULoadAction::kDontCare:
      return impeller::LoadAction::kDontCare;
    case FlutterGPULoadAction::kLoad:
      return impeller::LoadAction::kLoad;
    case FlutterGPULoadAction::kClear:
      return impeller::LoadAction::kClear;
  }
}

constexpr impeller::LoadAction ToImpellerLoadAction(int value) {
  return ToImpellerLoadAction(static_cast<FlutterGPULoadAction>(value));
}

enum class FlutterGPUStoreAction {
  kDontCare,
  kStore,
  kMultisampleResolve,
  kStoreAndMultisampleResolve,
};

constexpr impeller::StoreAction ToImpellerStoreAction(
    FlutterGPUStoreAction value) {
  switch (value) {
    case FlutterGPUStoreAction::kDontCare:
      return impeller::StoreAction::kDontCare;
    case FlutterGPUStoreAction::kStore:
      return impeller::StoreAction::kStore;
    case FlutterGPUStoreAction::kMultisampleResolve:
      return impeller::StoreAction::kMultisampleResolve;
    case FlutterGPUStoreAction::kStoreAndMultisampleResolve:
      return impeller::StoreAction::kStoreAndMultisampleResolve;
  }
}

constexpr impeller::StoreAction ToImpellerStoreAction(int value) {
  return ToImpellerStoreAction(static_cast<FlutterGPUStoreAction>(value));
}

enum class FlutterGPUShaderStage {
  kVertex,
  kFragment,
};

constexpr impeller::ShaderStage ToImpellerShaderStage(
    FlutterGPUShaderStage value) {
  switch (value) {
    case FlutterGPUShaderStage::kVertex:
      return impeller::ShaderStage::kVertex;
    case FlutterGPUShaderStage::kFragment:
      return impeller::ShaderStage::kFragment;
  }
}

constexpr impeller::ShaderStage ToImpellerShaderStage(int value) {
  return ToImpellerShaderStage(static_cast<FlutterGPUShaderStage>(value));
}

constexpr FlutterGPUShaderStage FromImpellerShaderStage(
    impeller::ShaderStage value) {
  switch (value) {
    case impeller::ShaderStage::kVertex:
      return FlutterGPUShaderStage::kVertex;
    case impeller::ShaderStage::kFragment:
      return FlutterGPUShaderStage::kFragment;
    case impeller::ShaderStage::kUnknown:
    case impeller::ShaderStage::kCompute:
      FML_LOG(FATAL) << "Invalid Flutter GPU ShaderStage "
                     << static_cast<size_t>(value);
      FML_UNREACHABLE();
  }
}

enum class FlutterGPUMinMagFilter {
  kNearest,
  kLinear,
};

constexpr impeller::MinMagFilter ToImpellerMinMagFilter(
    FlutterGPUMinMagFilter value) {
  switch (value) {
    case FlutterGPUMinMagFilter::kNearest:
      return impeller::MinMagFilter::kNearest;
    case FlutterGPUMinMagFilter::kLinear:
      return impeller::MinMagFilter::kLinear;
  }
}

constexpr impeller::MinMagFilter ToImpellerMinMagFilter(int value) {
  return ToImpellerMinMagFilter(static_cast<FlutterGPUMinMagFilter>(value));
}

enum class FlutterGPUMipFilter {
  kNearest,
  kLinear,
};

constexpr impeller::MipFilter ToImpellerMipFilter(FlutterGPUMipFilter value) {
  switch (value) {
    case FlutterGPUMipFilter::kNearest:
      return impeller::MipFilter::kNearest;
    case FlutterGPUMipFilter::kLinear:
      return impeller::MipFilter::kLinear;
  }
}

constexpr impeller::MipFilter ToImpellerMipFilter(int value) {
  return ToImpellerMipFilter(static_cast<FlutterGPUMipFilter>(value));
}

enum class FlutterGPUSamplerAddressMode {
  kClampToEdge,
  kRepeat,
  kMirror,
};

constexpr impeller::SamplerAddressMode ToImpellerSamplerAddressMode(
    FlutterGPUSamplerAddressMode value) {
  switch (value) {
    case FlutterGPUSamplerAddressMode::kClampToEdge:
      return impeller::SamplerAddressMode::kClampToEdge;
    case FlutterGPUSamplerAddressMode::kRepeat:
      return impeller::SamplerAddressMode::kRepeat;
    case FlutterGPUSamplerAddressMode::kMirror:
      return impeller::SamplerAddressMode::kMirror;
  }
}

constexpr impeller::SamplerAddressMode ToImpellerSamplerAddressMode(int value) {
  return ToImpellerSamplerAddressMode(
      static_cast<FlutterGPUSamplerAddressMode>(value));
}

enum class FlutterGPUIndexType {
  k16bit,
  k32bit,
};

constexpr impeller::IndexType ToImpellerIndexType(FlutterGPUIndexType value) {
  switch (value) {
    case FlutterGPUIndexType::k16bit:
      return impeller::IndexType::k16bit;
    case FlutterGPUIndexType::k32bit:
      return impeller::IndexType::k32bit;
  }
}

constexpr impeller::IndexType ToImpellerIndexType(int value) {
  return ToImpellerIndexType(static_cast<FlutterGPUIndexType>(value));
}

enum class FlutterGPUPrimitiveType {
  kTriangle,
  kTriangleStrip,
  kLine,
  kLineStrip,
  kPoint,
};

constexpr impeller::PrimitiveType ToImpellerPrimitiveType(
    FlutterGPUPrimitiveType value) {
  switch (value) {
    case FlutterGPUPrimitiveType::kTriangle:
      return impeller::PrimitiveType::kTriangle;
    case FlutterGPUPrimitiveType::kTriangleStrip:
      return impeller::PrimitiveType::kTriangleStrip;
    case FlutterGPUPrimitiveType::kLine:
      return impeller::PrimitiveType::kLine;
    case FlutterGPUPrimitiveType::kLineStrip:
      return impeller::PrimitiveType::kLineStrip;
    case FlutterGPUPrimitiveType::kPoint:
      return impeller::PrimitiveType::kPoint;
  }
}

constexpr impeller::PrimitiveType ToImpellerPrimitiveType(int value) {
  return ToImpellerPrimitiveType(static_cast<FlutterGPUPrimitiveType>(value));
}

enum class FlutterGPUCompareFunction {
  kNever,
  kAlways,
  kLess,
  kEqual,
  kLessEqual,
  kGreater,
  kNotEqual,
  kGreaterEqual,
};

constexpr impeller::CompareFunction ToImpellerCompareFunction(
    FlutterGPUCompareFunction value) {
  switch (value) {
    case FlutterGPUCompareFunction::kNever:
      return impeller::CompareFunction::kNever;
    case FlutterGPUCompareFunction::kAlways:
      return impeller::CompareFunction::kAlways;
    case FlutterGPUCompareFunction::kLess:
      return impeller::CompareFunction::kLess;
    case FlutterGPUCompareFunction::kEqual:
      return impeller::CompareFunction::kEqual;
    case FlutterGPUCompareFunction::kLessEqual:
      return impeller::CompareFunction::kLessEqual;
    case FlutterGPUCompareFunction::kGreater:
      return impeller::CompareFunction::kGreater;
    case FlutterGPUCompareFunction::kNotEqual:
      return impeller::CompareFunction::kNotEqual;
    case FlutterGPUCompareFunction::kGreaterEqual:
      return impeller::CompareFunction::kGreaterEqual;
  }
}

constexpr impeller::CompareFunction ToImpellerCompareFunction(int value) {
  return ToImpellerCompareFunction(
      static_cast<FlutterGPUCompareFunction>(value));
}

enum class FlutterGPUStencilOperation {
  kKeep,
  kZero,
  kSetToReferenceValue,
  kIncrementClamp,
  kDecrementClamp,
  kInvert,
  kIncrementWrap,
  kDecrementWrap,
};

constexpr impeller::StencilOperation ToImpellerStencilOperation(
    FlutterGPUStencilOperation value) {
  switch (value) {
    case FlutterGPUStencilOperation::kKeep:
      return impeller::StencilOperation::kKeep;
    case FlutterGPUStencilOperation::kZero:
      return impeller::StencilOperation::kZero;
    case FlutterGPUStencilOperation::kSetToReferenceValue:
      return impeller::StencilOperation::kSetToReferenceValue;
    case FlutterGPUStencilOperation::kIncrementClamp:
      return impeller::StencilOperation::kIncrementClamp;
    case FlutterGPUStencilOperation::kDecrementClamp:
      return impeller::StencilOperation::kDecrementClamp;
    case FlutterGPUStencilOperation::kInvert:
      return impeller::StencilOperation::kInvert;
    case FlutterGPUStencilOperation::kIncrementWrap:
      return impeller::StencilOperation::kIncrementWrap;
    case FlutterGPUStencilOperation::kDecrementWrap:
      return impeller::StencilOperation::kDecrementWrap;
  }
}

constexpr impeller::StencilOperation ToImpellerStencilOperation(int value) {
  return ToImpellerStencilOperation(
      static_cast<FlutterGPUStencilOperation>(value));
}

enum class FlutterGPUCullMode {
  kNone,
  kFrontFace,
  kBackFace,
};

constexpr impeller::CullMode ToImpellerCullMode(FlutterGPUCullMode value) {
  switch (value) {
    case FlutterGPUCullMode::kNone:
      return impeller::CullMode::kNone;
    case FlutterGPUCullMode::kFrontFace:
      return impeller::CullMode::kFrontFace;
    case FlutterGPUCullMode::kBackFace:
      return impeller::CullMode::kBackFace;
  }
}

constexpr impeller::CullMode ToImpellerCullMode(int value) {
  return ToImpellerCullMode(static_cast<FlutterGPUCullMode>(value));
}

enum class FlutterGPUWindingOrder {
  kClockwise,
  kCounterClockwise,
};

constexpr impeller::WindingOrder ToImpellerWindingOrder(
    FlutterGPUWindingOrder value) {
  switch (value) {
    case FlutterGPUWindingOrder::kClockwise:
      return impeller::WindingOrder::kClockwise;
    case FlutterGPUWindingOrder::kCounterClockwise:
      return impeller::WindingOrder::kCounterClockwise;
  }
}

constexpr impeller::WindingOrder ToImpellerWindingOrder(int value) {
  return ToImpellerWindingOrder(static_cast<FlutterGPUWindingOrder>(value));
}

enum class FlutterGPUPolygonMode {
  kFill,
  kLine,
};

constexpr impeller::PolygonMode ToImpellerPolygonMode(
    FlutterGPUPolygonMode value) {
  switch (value) {
    case FlutterGPUPolygonMode::kFill:
      return impeller::PolygonMode::kFill;
    case FlutterGPUPolygonMode::kLine:
      return impeller::PolygonMode::kLine;
  }
}

constexpr impeller::PolygonMode ToImpellerPolygonMode(int value) {
  return ToImpellerPolygonMode(static_cast<FlutterGPUPolygonMode>(value));
}

}  // namespace gpu
}  // namespace flutter

#endif  // FLUTTER_LIB_GPU_FORMATS_H_
