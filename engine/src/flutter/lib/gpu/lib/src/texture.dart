// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

base class Texture extends NativeFieldWrapperClass1 {
  bool _valid = false;
  get isValid {
    return _valid;
  }

  /// Creates a new Texture.
  Texture._initialize(
    GpuContext gpuContext,
    this.storageMode,
    this.format,
    this.width,
    this.height,
    this.sampleCount,
    TextureCoordinateSystem coordinateSystem,
    this.enableRenderTargetUsage,
    this.enableShaderReadUsage,
    this.enableShaderWriteUsage,
  ) : _coordinateSystem = coordinateSystem {
    if (sampleCount != 1 && sampleCount != 4) {
      throw Exception("Only a sample count of 1 or 4 is currently supported");
    }
    _valid = _initialize(
      gpuContext,
      storageMode.index,
      format.index,
      width,
      height,
      sampleCount,
      coordinateSystem.index,
      enableRenderTargetUsage,
      enableShaderReadUsage,
      enableShaderWriteUsage,
    );
  }

  final StorageMode storageMode;
  final PixelFormat format;
  final int width;
  final int height;
  final int sampleCount;

  /// Enable using this texture as a render pass attachment.
  final bool enableRenderTargetUsage;

  /// Enable reading or sampling from this texture in a shader.
  final bool enableShaderReadUsage;

  /// Enable writing to the texture in a shader.
  ///
  /// Note that this is distinct from [enableRenderTargetUsage].
  final bool enableShaderWriteUsage;

  TextureCoordinateSystem _coordinateSystem;
  TextureCoordinateSystem get coordinateSystem {
    return _coordinateSystem;
  }

  set coordinateSystem(TextureCoordinateSystem value) {
    value;
    _setCoordinateSystem(value.index);
  }

  int get bytesPerTexel {
    return _bytesPerTexel();
  }

  int GetBaseMipLevelSizeInBytes() {
    return bytesPerTexel * width * height;
  }

  /// Overwrite the entire base mipmap level of this [Texture].
  ///
  /// This method can only be used if the [Texture] was created with
  /// [StorageMode.hostVisible]. An exception will be thrown otherwise.
  ///
  /// The length of [sourceBytes] must be exactly the size of the base mip
  /// level, otherwise an exception will be thrown. The size of the base mip
  /// level is always `width * height * bytesPerPixel`.
  ///
  /// Returns [true] if the write was successful, or [false] if the write
  /// failed due to an internal error.
  bool overwrite(ByteData sourceBytes) {
    if (storageMode != StorageMode.hostVisible) {
      throw Exception(
        'Texture.overwrite can only be used with Textures that are host visible',
      );
    }
    int baseMipSize = GetBaseMipLevelSizeInBytes();
    if (sourceBytes.lengthInBytes != baseMipSize) {
      throw Exception(
        'The length of sourceBytes (bytes: ${sourceBytes.lengthInBytes}) must exactly match the size of the base mip level (bytes: ${baseMipSize})',
      );
    }
    return _overwrite(sourceBytes);
  }

  ui.Image asImage() {
    if (!enableShaderReadUsage) {
      throw Exception(
        'Only shader readable Flutter GPU textures can be used as UI Images',
      );
    }
    return _asImage();
  }

  /// Wrap with native counterpart.
  @Native<
    Bool Function(
      Handle,
      Pointer<Void>,
      Int,
      Int,
      Int,
      Int,
      Int,
      Int,
      Bool,
      Bool,
      Bool,
    )
  >(symbol: 'InternalFlutterGpu_Texture_Initialize')
  external bool _initialize(
    GpuContext gpuContext,
    int storageMode,
    int format,
    int width,
    int height,
    int sampleCount,
    int coordinateSystem,
    bool enableRenderTargetUsage,
    bool enableShaderReadUsage,
    bool enableShaderWriteUsage,
  );

  @Native<Void Function(Handle, Int)>(
    symbol: 'InternalFlutterGpu_Texture_SetCoordinateSystem',
  )
  external void _setCoordinateSystem(int coordinateSystem);

  @Native<Int Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Texture_BytesPerTexel',
  )
  external int _bytesPerTexel();

  @Native<Bool Function(Pointer<Void>, Handle)>(
    symbol: 'InternalFlutterGpu_Texture_Overwrite',
  )
  external bool _overwrite(ByteData bytes);

  @Native<Handle Function(Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_Texture_AsImage',
  )
  external ui.Image _asImage();
}
