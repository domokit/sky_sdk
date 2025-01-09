// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

export 'dart:typed_data'
    show ByteData, Endian, Float32List, Float64List, Int32List, Int64List, Uint8List;

/// Write-only buffer for incrementally building a [ByteData] instance.
///
/// A WriteBuffer instance can be used only once. Attempts to reuse will result
/// in [StateError]s being thrown.
///
/// The byte order used is [Endian.host] throughout.
class WriteBuffer {
  /// Creates an interface for incrementally building a [ByteData] instance.
  /// [startCapacity] determines the start size of the [WriteBuffer] in bytes.
  /// The closer that value is to the real size used, the better the
  /// performance.
  factory WriteBuffer({int startCapacity = 8}) {
    assert(startCapacity > 0);
    final ByteData eightBytes = ByteData(8);
    final Uint8List eightBytesAsList = eightBytes.buffer.asUint8List();
    return WriteBuffer._(Uint8List(startCapacity), eightBytes, eightBytesAsList);
  }

  WriteBuffer._(this._buffer, this._eightBytes, this._eightBytesAsList);

  Uint8List _buffer;
  int _currentSize = 0;
  bool _isDone = false;
  final ByteData _eightBytes;
  final Uint8List _eightBytesAsList;
  static final Uint8List _zeroBuffer = Uint8List(8);

  void _add(int byte) {
    if (_currentSize == _buffer.length) {
      _resize();
    }
    _buffer[_currentSize] = byte;
    _currentSize += 1;
  }

  void _append(Uint8List other) {
    final int newSize = _currentSize + other.length;
    if (newSize >= _buffer.length) {
      _resize(newSize);
    }
    _buffer.setRange(_currentSize, newSize, other);
    _currentSize += other.length;
  }

  void _addAll(Uint8List data, [int start = 0, int? end]) {
    final int newEnd = end ?? _eightBytesAsList.length;
    final int newSize = _currentSize + (newEnd - start);
    if (newSize >= _buffer.length) {
      _resize(newSize);
    }
    _buffer.setRange(_currentSize, newSize, data);
    _currentSize = newSize;
  }

  void _resize([int? requiredLength]) {
    final int doubleLength = _buffer.length * 2;
    final int newLength = math.max(requiredLength ?? 0, doubleLength);
    final Uint8List newBuffer = Uint8List(newLength);
    newBuffer.setRange(0, _buffer.length, _buffer);
    _buffer = newBuffer;
  }

  /// Write a Uint8 into the buffer.
  void putUint8(int byte) {
    assert(!_isDone);
    _add(byte);
  }

  /// Write a Uint16 into the buffer.
  void putUint16(int value, {Endian? endian}) {
    assert(!_isDone);
    _eightBytes.setUint16(0, value, endian ?? Endian.host);
    _addAll(_eightBytesAsList, 0, 2);
  }

  /// Write a Uint32 into the buffer.
  void putUint32(int value, {Endian? endian}) {
    assert(!_isDone);
    _eightBytes.setUint32(0, value, endian ?? Endian.host);
    _addAll(_eightBytesAsList, 0, 4);
  }

  /// Write an Int32 into the buffer.
  void putInt32(int value, {Endian? endian}) {
    assert(!_isDone);
    _eightBytes.setInt32(0, value, endian ?? Endian.host);
    _addAll(_eightBytesAsList, 0, 4);
  }

  /// Write an Int64 into the buffer.
  void putInt64(int value, {Endian? endian}) {
    assert(!_isDone);
    _eightBytes.setInt64(0, value, endian ?? Endian.host);
    _addAll(_eightBytesAsList, 0, 8);
  }

  /// Write an Float64 into the buffer.
  void putFloat64(double value, {Endian? endian}) {
    assert(!_isDone);
    _alignTo(8);
    _eightBytes.setFloat64(0, value, endian ?? Endian.host);
    _addAll(_eightBytesAsList);
  }

  /// Write all the values from a [Uint8List] into the buffer.
  void putUint8List(Uint8List list) {
    assert(!_isDone);
    _append(list);
  }

  /// Write all the values from an [Int32List] into the buffer.
  void putInt32List(Int32List list) {
    assert(!_isDone);
    _alignTo(4);
    _append(list.buffer.asUint8List(list.offsetInBytes, 4 * list.length));
  }

  /// Write all the values from an [Int64List] into the buffer.
  void putInt64List(Int64List list) {
    assert(!_isDone);
    _alignTo(8);
    _append(list.buffer.asUint8List(list.offsetInBytes, 8 * list.length));
  }

  /// Write all the values from a [Float32List] into the buffer.
  void putFloat32List(Float32List list) {
    assert(!_isDone);
    _alignTo(4);
    _append(list.buffer.asUint8List(list.offsetInBytes, 4 * list.length));
  }

  /// Write all the values from a [Float64List] into the buffer.
  void putFloat64List(Float64List list) {
    assert(!_isDone);
    _alignTo(8);
    _append(list.buffer.asUint8List(list.offsetInBytes, 8 * list.length));
  }

  void _alignTo(int alignment) {
    assert(!_isDone);
    final int mod = _currentSize % alignment;
    if (mod != 0) {
      _addAll(_zeroBuffer, 0, alignment - mod);
    }
  }

  /// Finalize and return the written [ByteData].
  ByteData done() {
    if (_isDone) {
      throw StateError('done() must not be called more than once on the same $runtimeType.');
    }
    final ByteData result = _buffer.buffer.asByteData(0, _currentSize);
    _buffer = Uint8List(0);
    _isDone = true;
    return result;
  }
}

/// Read-only buffer for reading sequentially from a [ByteData] instance.
///
/// The byte order used is [Endian.host] throughout.
class ReadBuffer {
  /// Creates a [ReadBuffer] for reading from the specified [data].
  ReadBuffer(this.data);

  /// The underlying data being read.
  final ByteData data;

  /// The position to read next.
  int _position = 0;

  /// Whether the buffer has data remaining to read.
  bool get hasRemaining => _position < data.lengthInBytes;

  /// Reads a Uint8 from the buffer.
  int getUint8() {
    return data.getUint8(_position++);
  }

  /// Reads a Uint16 from the buffer.
  int getUint16({Endian? endian}) {
    final int value = data.getUint16(_position, endian ?? Endian.host);
    _position += 2;
    return value;
  }

  /// Reads a Uint32 from the buffer.
  int getUint32({Endian? endian}) {
    final int value = data.getUint32(_position, endian ?? Endian.host);
    _position += 4;
    return value;
  }

  /// Reads an Int32 from the buffer.
  int getInt32({Endian? endian}) {
    final int value = data.getInt32(_position, endian ?? Endian.host);
    _position += 4;
    return value;
  }

  /// Reads an Int64 from the buffer.
  int getInt64({Endian? endian}) {
    final int value = data.getInt64(_position, endian ?? Endian.host);
    _position += 8;
    return value;
  }

  /// Reads a Float64 from the buffer.
  double getFloat64({Endian? endian}) {
    _alignTo(8);
    final double value = data.getFloat64(_position, endian ?? Endian.host);
    _position += 8;
    return value;
  }

  /// Reads the given number of Uint8s from the buffer.
  Uint8List getUint8List(int length) {
    final Uint8List list = data.buffer.asUint8List(data.offsetInBytes + _position, length);
    _position += length;
    return list;
  }

  /// Reads the given number of Int32s from the buffer.
  Int32List getInt32List(int length) {
    _alignTo(4);
    final Int32List list = data.buffer.asInt32List(data.offsetInBytes + _position, length);
    _position += 4 * length;
    return list;
  }

  /// Reads the given number of Int64s from the buffer.
  Int64List getInt64List(int length) {
    _alignTo(8);
    final Int64List list = data.buffer.asInt64List(data.offsetInBytes + _position, length);
    _position += 8 * length;
    return list;
  }

  /// Reads the given number of Float32s from the buffer
  Float32List getFloat32List(int length) {
    _alignTo(4);
    final Float32List list = data.buffer.asFloat32List(data.offsetInBytes + _position, length);
    _position += 4 * length;
    return list;
  }

  /// Reads the given number of Float64s from the buffer.
  Float64List getFloat64List(int length) {
    _alignTo(8);
    final Float64List list = data.buffer.asFloat64List(data.offsetInBytes + _position, length);
    _position += 8 * length;
    return list;
  }

  void _alignTo(int alignment) {
    final int mod = _position % alignment;
    if (mod != 0) {
      _position += alignment - mod;
    }
  }
}
