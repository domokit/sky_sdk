// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

/// A reference to a byte range within a GPU-resident [Buffer].
class BufferView {
  /// The device buffer of this view.
  final DeviceBuffer buffer;

  /// The start of the view, in bytes starting from the beginning of the
  /// [buffer].
  final int offsetInBytes;

  /// The length of the view.
  final int lengthInBytes;

  /// Create a new view into a buffer on the GPU.
  const BufferView(
    this.buffer, {
    required this.offsetInBytes,
    required this.lengthInBytes,
  });
}

/// [DeviceBuffer] is a region of memory allocated on the device heap
/// (GPU-resident memory).
base class DeviceBuffer extends NativeFieldWrapperClass1 {
  bool _valid = false;
  get isValid {
    return _valid;
  }

  /// Creates a new DeviceBuffer.
  DeviceBuffer._initialize(
    GpuContext gpuContext,
    StorageMode storageMode,
    int sizeInBytes,
  ) : storageMode = storageMode,
      sizeInBytes = sizeInBytes {
    _valid = _initialize(gpuContext, storageMode.index, sizeInBytes);
  }

  /// Creates a new host visible DeviceBuffer with data copied from the host.
  DeviceBuffer._initializeWithHostData(GpuContext gpuContext, ByteData data)
    : storageMode = StorageMode.hostVisible,
      sizeInBytes = data.lengthInBytes {
    _valid = _initializeWithHostData(gpuContext, data);
  }

  final StorageMode storageMode;
  final int sizeInBytes;

  void _bindAsVertexBuffer(
    RenderPass renderPass,
    int offsetInBytes,
    int lengthInBytes,
    int vertexCount,
  ) {
    renderPass._bindVertexBufferDevice(
      this,
      offsetInBytes,
      lengthInBytes,
      vertexCount,
    );
  }

  void _bindAsIndexBuffer(
    RenderPass renderPass,
    int offsetInBytes,
    int lengthInBytes,
    IndexType indexType,
    int indexCount,
  ) {
    renderPass._bindIndexBufferDevice(
      this,
      offsetInBytes,
      lengthInBytes,
      indexType.index,
      indexCount,
    );
  }

  bool _bindAsUniform(
    RenderPass renderPass,
    UniformSlot slot,
    int offsetInBytes,
    int lengthInBytes,
  ) {
    return renderPass._bindUniformDevice(
      slot.shader,
      slot.uniformName,
      this,
      offsetInBytes,
      lengthInBytes,
    );
  }

  /// Wrap with native counterpart.
  @Native<Bool Function(Handle, Pointer<Void>, Int, Int)>(
    symbol: 'InternalFlutterGpu_DeviceBuffer_Initialize',
  )
  external bool _initialize(
    GpuContext gpuContext,
    int storageMode,
    int sizeInBytes,
  );

  /// Wrap with native counterpart.
  @Native<Bool Function(Handle, Pointer<Void>, Handle)>(
    symbol: 'InternalFlutterGpu_DeviceBuffer_InitializeWithHostData',
  )
  external bool _initializeWithHostData(GpuContext gpuContext, ByteData data);

  /// Overwrite a range of bytes within an existing [DeviceBuffer].
  ///
  /// This method can only be used if the [DeviceBuffer] was created with
  /// [StorageMode.hostVisible]. An exception will be thrown otherwise.
  ///
  /// After new writes have been staged, the [DeviceBuffer.flush] should be
  /// called prior to accessing the data. Otherwise, the updated data will not
  /// be copied to the GPU on devices that don't have host coherent memory.
  ///
  /// The entire length of [sourceBytes] will be copied into the [DeviceBuffer],
  /// starting at byte index [destinationOffsetInBytes] in the [DeviceBuffer].
  /// If performing this copy would result in an out of bounds write to the
  /// buffer, then the write will not be attempted and will fail.
  ///
  /// Returns [true] if the write was successful, or [false] if the write
  /// failed due to an internal error.
  bool overwrite(ByteData sourceBytes, {int destinationOffsetInBytes = 0}) {
    if (storageMode != StorageMode.hostVisible) {
      throw Exception(
        'DeviceBuffer.overwrite can only be used with DeviceBuffers that are host visible',
      );
    }
    if (destinationOffsetInBytes < 0) {
      throw Exception('destinationOffsetInBytes must be positive');
    }
    return _overwrite(sourceBytes, destinationOffsetInBytes);
  }

  @Native<Bool Function(Pointer<Void>, Handle, Int)>(
    symbol: 'InternalFlutterGpu_DeviceBuffer_Overwrite',
  )
  external bool _overwrite(ByteData bytes, int destinationOffsetInBytes);

  /// Flush the contents of the [DeviceBuffer] to the GPU.
  ///
  /// This method can only be used if the [DeviceBuffer] was created with
  /// [StorageMode.hostVisible]. An exception will be thrown otherwise.
  ///
  /// If [lengthInBytes] is set to -1, the entire buffer will be flushed.
  ///
  /// On devices with coherent host memory (memory shared between the CPU and
  /// GPU), this method is a no-op.
  void flush({int offsetInBytes = 0, int lengthInBytes = -1}) {
    if (storageMode != StorageMode.hostVisible) {
      throw Exception(
        'DeviceBuffer.flush can only be used with DeviceBuffers that are host visible',
      );
    }
    if (offsetInBytes < 0 || offsetInBytes >= sizeInBytes) {
      throw Exception('offsetInBytes must be within the bounds of the buffer');
    }
    if (lengthInBytes < -1) {
      throw Exception('lengthInBytes must be either positive or -1');
    }
    if (lengthInBytes != -1 && offsetInBytes + lengthInBytes > sizeInBytes) {
      throw Exception(
        'The provided range must not be too large to fit within the buffer',
      );
    }
    _flush(offsetInBytes, lengthInBytes);
  }

  @Native<Void Function(Pointer<Void>, Int, Int)>(
    symbol: 'InternalFlutterGpu_DeviceBuffer_Flush',
  )
  external void _flush(int offsetInBytes, int lengthInBytes);
}

/// [HostBuffer] is a bump allocator that managed a [DeviceBuffer] block list.
///
/// This is useful for chunking sparse data uploads, especially ephemeral
/// uniform or vertex data that needs to change from frame to frame.
///
/// Different platforms have different data alignment requirements when reading
/// [DeviceBuffer] data for shader uniforms. [HostBuffer] uses
/// [GpuContext.minimumUniformByteAlignment] to align each emplacement
/// automatically, so that they may be used in uniform bindings.
///
/// The length of each [DeviceBuffer] block is determined by
/// [blockLengthInBytes] and cannot be changed after creation of the
/// [HostBuffer]. If [HostBuffer.emplace] is given a [ByteData] that is larger
/// than [blockLengthInBytes], a new [DeviceBuffer] block is created that
/// matches the size of the oversized [ByteData].
base class HostBuffer {
  /// The default length to use for each [DeviceBuffer] block.
  static const int kDefaultBlockLengthInBytes = 1024000; // 1024 Kb

  /// The length to use for each [DeviceBuffer] block.
  final int blockLengthInBytes;

  static const int _kFrameCount = 4;

  /// The number of frames to cycle through before reusing device buffers.
  /// Cycling to the next frame happens when [reset] is called.
  int get frameCount {
    return _kFrameCount;
  }

  final GpuContext _gpuContext;

  /// The current frame. Rotates through [frameCount] frames when [reset] is
  /// called.
  int _frameCursor = 0;

  /// The buffer within the current frame to be used for the next emplacement.
  int _bufferCursor = 0;

  /// The offset within the current block to be used for the next emplacement.
  int _offsetCursor = 0;

  final List<List<DeviceBuffer>> _buffers = [];

  /// Creates a new HostBuffer.
  HostBuffer._initialize(
    this._gpuContext, {
    this.blockLengthInBytes = HostBuffer.kDefaultBlockLengthInBytes,
  }) {
    for (int i = 0; i < frameCount; i++) {
      List<DeviceBuffer> frame = [];
      _buffers.add(frame);
      _buffers[i].add(_allocateNewBlock(blockLengthInBytes));
    }
  }

  DeviceBuffer _allocateNewBlock(length) {
    final buffer = _gpuContext.createDeviceBuffer(
      StorageMode.hostVisible,
      length,
    );
    if (buffer == null) {
      throw Exception('Failed to allocate DeviceBuffer of length $length');
    }
    return buffer;
  }

  /// Prepare a new buffer range to be used for storing the given bytes.
  /// Allocates a new block if necessary.
  BufferView _allocateEmplacement(ByteData bytes) {
    if (bytes.lengthInBytes > blockLengthInBytes) {
      return BufferView(
        _allocateNewBlock(bytes.lengthInBytes),
        offsetInBytes: 0,
        lengthInBytes: bytes.lengthInBytes,
      );
    }

    int padding =
        _gpuContext.minimumUniformByteAlignment -
        (_offsetCursor % _gpuContext.minimumUniformByteAlignment);
    // If the padding is the full alignment size, then we're already aligned.
    // So reset the padding to zero.
    padding %= _gpuContext.minimumUniformByteAlignment;
    if (_offsetCursor + padding >= blockLengthInBytes) {
      DeviceBuffer buffer = _allocateNewBlock(blockLengthInBytes);
      _buffers[_frameCursor].add(buffer);
      _bufferCursor++;
      _offsetCursor = bytes.lengthInBytes;

      return BufferView(
        buffer,
        offsetInBytes: 0,
        lengthInBytes: blockLengthInBytes,
      );
    }

    _offsetCursor += padding;
    final view = BufferView(
      _buffers[_frameCursor][_bufferCursor],
      offsetInBytes: _offsetCursor,
      lengthInBytes: bytes.lengthInBytes,
    );
    _offsetCursor += bytes.lengthInBytes;
    return view;
  }

  /// Append byte data to the end of the [HostBuffer] and produce a [BufferView]
  /// that references the new data in the buffer.
  ///
  /// This method automatically inserts padding in-between emplace calls in the
  /// buffer if necessary to abide by platform-specific uniform alignment
  /// requirements.
  ///
  /// The [DeviceBuffer] referenced in the [BufferView] has already been
  /// flushed, so there is no need to call [DeviceBuffer.flush] before
  /// referencing it in a command.
  BufferView emplace(ByteData bytes) {
    BufferView view = _allocateEmplacement(bytes);
    if (!view.buffer.overwrite(
      bytes,
      destinationOffsetInBytes: view.offsetInBytes,
    )) {
      throw Exception(
        'Failed to write range (offset=${view.offsetInBytes}, length=${view.lengthInBytes}) '
        'to HostBuffer-managed DeviceBuffer (frame=$_frameCursor, buffer=$_bufferCursor, offset=$_offsetCursor).',
      );
    }

    return view;
  }

  /// Resets the bump allocator to the beginning of the first [DeviceBuffer]
  /// block.
  void reset() {
    _frameCursor = (_frameCursor + 1) % frameCount;
    _bufferCursor = 0;
    _offsetCursor = 0;
  }
}
