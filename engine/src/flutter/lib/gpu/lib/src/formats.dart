// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

// ATTENTION! ATTENTION! ATTENTION!
// All enum classes defined in this file must exactly match the contents and
// order of the corresponding enums defined in `gpu/formats.h`.

/// Specifies where an allocation resides and how it may be used.
enum StorageMode {
  /// Allocations can be mapped onto the hosts address space and also be used by
  /// the device.
  hostVisible,

  /// Allocations can only be used by the device. This location is optimal for
  /// use by the device. If the host needs to access these allocations, the
  /// data must first be copied into a host visible allocation.
  devicePrivate,

  /// Used by the device for temporary render targets. These allocations cannot
  /// be copied to or from other allocations. This storage mode is only valid
  /// for Textures.
  ///
  /// These allocations reside in tile memory which has higher bandwidth, lower
  /// latency and lower power consumption. The total device memory usage is
  /// also lower as a separate allocation does not need to be created in
  /// device memory. Prefer using these allocations for intermediates like depth
  /// and stencil buffers.
  deviceTransient,
}

enum PixelFormat {
  unknown,
  a8UNormInt,
  r8UNormInt,
  r8g8UNormInt,
  r8g8b8a8UNormInt,
  r8g8b8a8UNormIntSRGB,
  b8g8r8a8UNormInt,
  b8g8r8a8UNormIntSRGB,
  r32g32b32a32Float,
  r16g16b16a16Float,
  b10g10r10XR,
  b10g10r10XRSRGB,
  b10g10r10a10XR,
  // Depth and stencil formats.
  s8UInt,
  d24UnormS8Uint,
  d32FloatS8UInt,
}

enum TextureCoordinateSystem {
  /// Alternative coordinate system used when uploading texture data from the
  /// host.
  /// (0, 0) is the bottom-left of the image with +Y going up.
  uploadFromHost,

  /// Default coordinate system.
  /// (0, 0) is the top-left of the image with +Y going down.
  renderToTexture,
}

enum BlendFactor {
  zero,
  one,
  sourceColor,
  oneMinusSourceColor,
  sourceAlpha,
  oneMinusSourceAlpha,
  destinationColor,
  oneMinusDestinationColor,
  destinationAlpha,
  oneMinusDestinationAlpha,
  sourceAlphaSaturated,
  blendColor,
  oneMinusBlendColor,
  blendAlpha,
  oneMinusBlendAlpha,
}

enum BlendOperation { add, subtract, reverseSubtract }

enum LoadAction { dontCare, load, clear }

enum StoreAction {
  dontCare,
  store,
  multisampleResolve,
  storeAndMultisampleResolve,
}

enum ShaderStage { vertex, fragment }

enum MinMagFilter { nearest, linear }

enum MipFilter { nearest, linear }

enum SamplerAddressMode { clampToEdge, repeat, mirror }

enum IndexType { int16, int32 }

enum PrimitiveType { triangle, triangleStrip, line, lineStrip, point }

enum CullMode { none, frontFace, backFace }

enum WindingOrder { clockwise, counterClockwise }

enum PolygonMode { fill, line }

enum CompareFunction {
  /// Comparison test never passes.
  never,

  /// Comparison test passes always passes.
  always,

  /// Comparison test passes if new_value < current_value.
  less,

  /// Comparison test passes if new_value == current_value.
  equal,

  /// Comparison test passes if new_value <= current_value.
  lessEqual,

  /// Comparison test passes if new_value > current_value.
  greater,

  /// Comparison test passes if new_value != current_value.
  notEqual,

  /// Comparison test passes if new_value >= current_value.
  greaterEqual,
}

enum StencilOperation {
  /// Don't modify the current stencil value.
  keep,

  /// Reset the stencil value to zero.
  zero,

  /// Reset the stencil value to the reference value.
  setToReferenceValue,

  /// Increment the current stencil value by 1. Clamp it to the maximum.
  incrementClamp,

  /// Decrement the current stencil value by 1. Clamp it to zero.
  decrementClamp,

  /// Perform a logical bitwise invert on the current stencil value.
  invert,

  /// Increment the current stencil value by 1. If at maximum, set to zero.
  incrementWrap,

  /// Decrement the current stencil value by 1. If at zero, set to maximum.
  decrementWrap,
}
