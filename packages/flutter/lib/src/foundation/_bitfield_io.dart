// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'bitfield.dart' as bitfield;

/// The dart:io implementation of [bitfield.kMaxUnsignedSMI].
const int kMaxUnsignedSMI = 0x3FFFFFFFFFFFFFFF; // ignore: avoid_js_rounded_ints, (VM-only code)

/// The dart:io implementation of [bitfield.BitField].
class BitField<T extends dynamic> implements bitfield.BitField<T> {
  /// The dart:io implementation of [bitfield.Bitfield()].
  BitField(this._length) : assert(_length <= _smiBits), _bits = _allZeros;

  /// The dart:io implementation of [bitfield.BitField.filled].
  BitField.filled(this._length, bool value)
    : assert(_length <= _smiBits),
      _bits = value ? _allOnes : _allZeros;

  final int _length;
  int _bits;

  static const int _smiBits =
      62; // see https://www.dartlang.org/articles/numeric-computation/#smis-and-mints
  static const int _allZeros = 0;
  static const int _allOnes = kMaxUnsignedSMI; // 2^(_kSMIBits+1)-1

  @override
  bool operator [](T index) {
    final int intIndex = index.index as int;
    assert(intIndex < _length);
    return (_bits & 1 << intIndex) > 0;
  }

  @override
  void operator []=(T index, bool value) {
    final int intIndex = index.index as int;
    assert(intIndex < _length);
    if (value) {
      _bits = _bits | (1 << intIndex);
    } else {
      _bits = _bits & ~(1 << intIndex);
    }
  }

  @override
  void reset([bool value = false]) {
    _bits = value ? _allOnes : _allZeros;
  }
}
