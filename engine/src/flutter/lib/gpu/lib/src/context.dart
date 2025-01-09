// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

/// A handle to a graphics context. Used to create and manage GPU resources.
///
/// To obtain the default graphics context, use [getContext].
base class GpuContext extends NativeFieldWrapperClass1 {
  /// Creates a new graphics context that corresponds to the default Impeller
  /// context.
  GpuContext._createDefault() {
    final String? error = _initializeDefault();
    if (error != null) {
      throw Exception(error);
    }
  }

  /// A supported [PixelFormat] for textures that store 4-channel colors
  /// (red/green/blue/alpha).
  PixelFormat get defaultColorFormat {
    return PixelFormat.values[_getDefaultColorFormat()];
  }

  /// A supported [PixelFormat] for textures that store stencil information.
  /// May include a depth channel if a stencil-only format is not available.
  PixelFormat get defaultStencilFormat {
    return PixelFormat.values[_getDefaultStencilFormat()];
  }

  /// A supported `PixelFormat` for textures that store both a stencil and depth
  /// component. This will never return a depth-only or stencil-only texture.
  ///
  /// May be [PixelFormat.unknown] if no suitable depth+stencil format was
  /// found.
  PixelFormat get defaultDepthStencilFormat {
    return PixelFormat.values[_getDefaultDepthStencilFormat()];
  }

  /// The minimum alignment required when referencing uniform blocks stored in a
  /// `DeviceBuffer`.
  int get minimumUniformByteAlignment {
    return _getMinimumUniformByteAlignment();
  }

  /// Whether the backend supports multisample anti-aliasing for offscreen
  /// color and stencil attachments. A subset of OpenGLES-only devices do not
  /// support this functionality.
  ///
  /// Any texture created via [createTexture] is an offscreen texture.
  /// There is currently no way to render directly against the "onscreen"
  /// texture that the framework renders to, so all Flutter GPU textures are
  /// "offscreen".
  bool get doesSupportOffscreenMSAA {
    return _getSupportsOffscreenMSAA();
  }

  /// Allocates a new region of GPU-resident memory.
  ///
  /// The [storageMode] must be either [StorageMode.hostVisible] or
  /// [StorageMode.devicePrivate], otherwise an exception will be thrown.
  ///
  /// Returns [null] if the [DeviceBuffer] creation failed.
  DeviceBuffer? createDeviceBuffer(StorageMode storageMode, int sizeInBytes) {
    if (storageMode == StorageMode.deviceTransient) {
      throw Exception(
        'DeviceBuffers cannot be set to StorageMode.deviceTransient',
      );
    }
    DeviceBuffer result = DeviceBuffer._initialize(
      this,
      storageMode,
      sizeInBytes,
    );
    return result.isValid ? result : null;
  }

  /// Allocates a new region of host-visible GPU-resident memory, initialized
  /// with the given [data].
  ///
  /// Given that the buffer will be immediately populated with [data] uploaded
  /// from the host, the [StorageMode] of the new [DeviceBuffer] is
  /// automatically set to [StorageMode.hostVisible].
  ///
  /// Returns [null] if the [DeviceBuffer] creation failed.
  DeviceBuffer? createDeviceBufferWithCopy(ByteData data) {
    DeviceBuffer result = DeviceBuffer._initializeWithHostData(this, data);
    return result.isValid ? result : null;
  }

  /// Creates a bump allocator that managed a [DeviceBuffer] block list.
  ///
  /// See also [HostBuffer].
  HostBuffer createHostBuffer({
    int blockLengthInBytes = HostBuffer.kDefaultBlockLengthInBytes,
  }) {
    return HostBuffer._initialize(this, blockLengthInBytes: blockLengthInBytes);
  }

  /// Allocates a new texture in GPU-resident memory.
  ///
  /// Returns [null] if the [Texture] creation failed.
  Texture? createTexture(
    StorageMode storageMode,
    int width,
    int height, {
    PixelFormat format = PixelFormat.r8g8b8a8UNormInt,
    sampleCount = 1,
    TextureCoordinateSystem coordinateSystem =
        TextureCoordinateSystem.renderToTexture,
    bool enableRenderTargetUsage = true,
    bool enableShaderReadUsage = true,
    bool enableShaderWriteUsage = false,
  }) {
    Texture result = Texture._initialize(
      this,
      storageMode,
      format,
      width,
      height,
      sampleCount,
      coordinateSystem,
      enableRenderTargetUsage,
      enableShaderReadUsage,
      enableShaderWriteUsage,
    );
    return result.isValid ? result : null;
  }

  /// Create a new command buffer that can be used to submit GPU commands.
  CommandBuffer createCommandBuffer() {
    return CommandBuffer._(this);
  }

  RenderPipeline createRenderPipeline(
    Shader vertexShader,
    Shader fragmentShader,
  ) {
    return RenderPipeline._(this, vertexShader, fragmentShader);
  }

  /// Associates the default Impeller context with this Context.
  @Native<Handle Function(Handle)>(
    symbol: 'InternalFlutterGpu_Context_InitializeDefault',
  )
  external String? _initializeDefault();

  @Native<Int Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Context_GetDefaultColorFormat',
  )
  external int _getDefaultColorFormat();

  @Native<Int Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Context_GetDefaultStencilFormat',
  )
  external int _getDefaultStencilFormat();

  @Native<Int Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Context_GetDefaultDepthStencilFormat',
  )
  external int _getDefaultDepthStencilFormat();

  @Native<Int Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Context_GetMinimumUniformByteAlignment',
  )
  external int _getMinimumUniformByteAlignment();

  @Native<Bool Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Context_GetSupportsOffscreenMSAA',
  )
  external bool _getSupportsOffscreenMSAA();
}

/// The default graphics context.
final GpuContext gpuContext = GpuContext._createDefault();
