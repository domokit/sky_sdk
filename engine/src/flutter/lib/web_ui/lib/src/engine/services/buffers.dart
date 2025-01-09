// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';
import 'dart:typed_data';

abstract class _TypedDataBuffer<E> extends ListBase<E> {
  _TypedDataBuffer(List<E> buffer) : _buffer = buffer, _length = buffer.length;

  static const int _initialLength = 8;

  /// The underlying data buffer.
  ///
  /// This is always both a List<E> and a TypedData, which we don't have a type
  /// for here. For example, for a `Uint8Buffer`, this is a `Uint8List`.
  List<E> _buffer;

  /// Returns a view of [_buffer] as a [TypedData].
  TypedData get _typedBuffer => _buffer as TypedData;

  /// The length of the list being built.
  int _length;

  @override
  int get length => _length;

  @override
  E operator [](int index) {
    if (index >= length) {
      throw RangeError.index(index, this);
    }
    return _buffer[index];
  }

  @override
  void operator []=(int index, E value) {
    if (index >= length) {
      throw RangeError.index(index, this);
    }
    _buffer[index] = value;
  }

  @override
  set length(int newLength) {
    if (newLength < _length) {
      final E defaultValue = _defaultValue;
      for (int i = newLength; i < _length; i++) {
        _buffer[i] = defaultValue;
      }
    } else if (newLength > _buffer.length) {
      List<E> newBuffer;
      if (_buffer.isEmpty) {
        newBuffer = _createBuffer(newLength);
      } else {
        newBuffer = _createBiggerBuffer(newLength);
      }
      newBuffer.setRange(0, _length, _buffer);
      _buffer = newBuffer;
    }
    _length = newLength;
  }

  void _add(E value) {
    if (_length == _buffer.length) {
      _grow(_length);
    }
    _buffer[_length++] = value;
  }

  // We override the default implementation of `add` because it grows the list
  // by setting the length in increments of one. We want to grow by doubling
  // capacity in most cases.
  @override
  void add(E value) {
    _add(value);
  }

  /// Appends all objects of [values] to the end of this buffer.
  ///
  /// This adds values from [start] (inclusive) to [end] (exclusive) in
  /// [values]. If [end] is omitted, it defaults to adding all elements of
  /// [values] after [start].
  ///
  /// The [start] value must be non-negative. The [values] iterable must have at
  /// least [start] elements, and if [end] is specified, it must be greater than
  /// or equal to [start] and [values] must have at least [end] elements.
  @override
  void addAll(Iterable<E> values, [int start = 0, int? end]) {
    RangeError.checkNotNegative(start, 'start');
    if (end != null && start > end) {
      throw RangeError.range(end, start, null, 'end');
    }

    _addAll(values, start, end);
  }

  /// Inserts all objects of [values] at position [index] in this list.
  ///
  /// This adds values from [start] (inclusive) to [end] (exclusive) in
  /// [values]. If [end] is omitted, it defaults to adding all elements of
  /// [values] after [start].
  ///
  /// The [start] value must be non-negative. The [values] iterable must have at
  /// least [start] elements, and if [end] is specified, it must be greater than
  /// or equal to [start] and [values] must have at least [end] elements.
  @override
  void insertAll(int index, Iterable<E> values, [int start = 0, int? end]) {
    IndexError.check(index, _length + 1, indexable: this, name: 'index');
    RangeError.checkNotNegative(start, 'start');
    if (end != null) {
      if (start > end) {
        throw RangeError.range(end, start, null, 'end');
      }
      if (start == end) {
        return;
      }
    }

    // If we're adding to the end of the list anyway, use [_addAll]. This lets
    // us avoid converting [values] into a list even if [end] is null, since we
    // can add values iteratively to the end of the list. We can't do so in the
    // center because copying the trailing elements every time is non-linear.
    if (index == _length) {
      _addAll(values, start, end);
      return;
    }

    if (end == null && values is List) {
      end = values.length;
    }
    if (end != null) {
      _insertKnownLength(index, values, start, end);
      return;
    }

    // Add elements at end, growing as appropriate, then put them back at
    // position [index] using flip-by-double-reverse.
    int writeIndex = _length;
    int skipCount = start;
    for (final E value in values) {
      if (skipCount > 0) {
        skipCount--;
        continue;
      }
      if (writeIndex == _buffer.length) {
        _grow(writeIndex);
      }
      _buffer[writeIndex++] = value;
    }

    if (skipCount > 0) {
      throw StateError('Too few elements');
    }
    if (end != null && writeIndex < end) {
      throw RangeError.range(end, start, writeIndex, 'end');
    }

    // Swap [index.._length) and [_length..writeIndex) by double-reversing.
    _reverse(_buffer, index, _length);
    _reverse(_buffer, _length, writeIndex);
    _reverse(_buffer, index, writeIndex);
    _length = writeIndex;
    return;
  }

  // Reverses the range [start..end) of buffer.
  static void _reverse(List<Object?> buffer, int start, int end) {
    end--; // Point to last element, not after last element.
    while (start < end) {
      final Object? first = buffer[start];
      final Object? last = buffer[end];
      buffer[end] = first;
      buffer[start] = last;
      start++;
      end--;
    }
  }

  /// Does the same thing as [addAll].
  ///
  /// This allows [addAll] and [insertAll] to share implementation without a
  /// subclass unexpectedly overriding both when it intended to only override
  /// [addAll].
  void _addAll(Iterable<E> values, [int start = 0, int? end]) {
    if (values is List<E>) {
      end ??= values.length;
    }

    // If we know the length of the segment to add, do so with [addRange]. This
    // way we know how much to grow the buffer in advance, and it may be even
    // more efficient for typed data input.
    if (end != null) {
      _insertKnownLength(_length, values, start, end);
      return;
    }

    // Otherwise, just add values one at a time.
    int i = 0;
    for (final E value in values) {
      if (i >= start) {
        add(value);
      }
      i++;
    }
    if (i < start) {
      throw StateError('Too few elements');
    }
  }

  /// Like [insertAll], but with a guaranteed non-`null` [start] and [end].
  void _insertKnownLength(int index, Iterable<E> values, int start, int end) {
    if (start > values.length || end > values.length) {
      throw StateError('Too few elements');
    }

    final int valuesLength = end - start;
    final int newLength = _length + valuesLength;
    _ensureCapacity(newLength);

    _buffer.setRange(index + valuesLength, _length + valuesLength, _buffer, index);
    _buffer.setRange(index, index + valuesLength, values, start);
    _length = newLength;
  }

  @override
  void insert(int index, E element) {
    if (index < 0 || index > _length) {
      throw RangeError.range(index, 0, _length);
    }
    if (_length < _buffer.length) {
      _buffer.setRange(index + 1, _length + 1, _buffer, index);
      _buffer[index] = element;
      _length++;
      return;
    }
    final List<E> newBuffer = _createBiggerBuffer(null);
    newBuffer.setRange(0, index, _buffer);
    newBuffer.setRange(index + 1, _length + 1, _buffer, index);
    newBuffer[index] = element;
    _length++;
    _buffer = newBuffer;
  }

  /// Ensures that [_buffer] is at least [requiredCapacity] long,
  ///
  /// Grows the buffer if necessary, preserving existing data.
  void _ensureCapacity(int requiredCapacity) {
    if (requiredCapacity <= _buffer.length) {
      return;
    }
    final List<E> newBuffer = _createBiggerBuffer(requiredCapacity);
    newBuffer.setRange(0, _length, _buffer);
    _buffer = newBuffer;
  }

  /// Create a bigger buffer.
  ///
  /// This method determines how much bigger a bigger buffer should
  /// be. If [requiredCapacity] is not null, it will be at least that
  /// size. It will always have at least have double the capacity of
  /// the current buffer.
  List<E> _createBiggerBuffer(int? requiredCapacity) {
    int newLength = _buffer.length * 2;
    if (requiredCapacity != null && newLength < requiredCapacity) {
      newLength = requiredCapacity;
    } else if (newLength < _initialLength) {
      newLength = _initialLength;
    }
    return _createBuffer(newLength);
  }

  /// Grows the buffer.
  ///
  /// This copies the first [length] elements into the new buffer.
  void _grow(int length) {
    _buffer = _createBiggerBuffer(null)..setRange(0, length, _buffer);
  }

  @override
  void setRange(int start, int end, Iterable<E> source, [int skipCount = 0]) {
    if (end > _length) {
      throw RangeError.range(end, 0, _length);
    }
    _setRange(start, end, source, skipCount);
  }

  /// Like [setRange], but with no bounds checking.
  void _setRange(int start, int end, Iterable<E> source, int skipCount) {
    if (source is _TypedDataBuffer<E>) {
      _buffer.setRange(start, end, source._buffer, skipCount);
    } else {
      _buffer.setRange(start, end, source, skipCount);
    }
  }

  // TypedData.

  int get elementSizeInBytes => _typedBuffer.elementSizeInBytes;

  int get lengthInBytes => _length * _typedBuffer.elementSizeInBytes;

  int get offsetInBytes => _typedBuffer.offsetInBytes;

  /// Returns the underlying [ByteBuffer].
  ///
  /// The returned buffer may be replaced by operations that change the [length]
  /// of this list.
  ///
  /// The buffer may be larger than [lengthInBytes] bytes, but never smaller.
  ByteBuffer get buffer => _typedBuffer.buffer;

  // Specialization for the specific type.

  // Return zero for integers, 0.0 for floats, etc.
  // Used to fill buffer when changing length.
  E get _defaultValue;

  // Create a new typed list to use as buffer.
  List<E> _createBuffer(int size);
}

abstract class _IntBuffer extends _TypedDataBuffer<int> {
  _IntBuffer(super.buffer);

  @override
  int get _defaultValue => 0;
}

class Uint8Buffer extends _IntBuffer {
  Uint8Buffer([int initialLength = 0]) : super(Uint8List(initialLength));

  @override
  Uint8List _createBuffer(int size) => Uint8List(size);
}
