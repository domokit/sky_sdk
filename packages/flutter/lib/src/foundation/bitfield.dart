// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '_bitfield_io.dart' if (dart.library.js_util) '_bitfield_web.dart' as bitfield;

/// The largest SMI value.
///
/// See <https://www.dartlang.org/articles/numeric-computation/#smis-and-mints>
///
/// When compiling to JavaScript, this value is not supported since it is
/// larger than the maximum safe 32bit integer.
const int kMaxUnsignedSMI = bitfield.kMaxUnsignedSMI;

/// A BitField over an enum (or other class whose values implement "index").
/// Only the first 62 values of the enum can be used as indices.
///
/// When compiling to JavaScript, this class is not supported.
abstract class BitField<T extends dynamic> {
  /// Creates a bit field of all zeros.
  ///
  /// The given length must be at most 62.
  factory BitField(int length) = bitfield.BitField<T>;

  /// Creates a bit field filled with a particular value.
  ///
  /// If the value argument is true, the bits are filled with ones. Otherwise,
  /// the bits are filled with zeros.
  ///
  /// The given length must be at most 62.
  factory BitField.filled(int length, bool value) = bitfield.BitField<T>.filled;

  /// Returns whether the bit with the given index is set to one.
  bool operator [](T index);

  /// Sets the bit with the given index to the given value.
  ///
  /// If value is true, the bit with the given index is set to one. Otherwise,
  /// the bit is set to zero.
  void operator []=(T index, bool value);

  /// Sets all the bits to the given value.
  ///
  /// If the value is true, the bits are all set to one. Otherwise, the bits are
  /// all set to zero. Defaults to setting all the bits to zero.
  void reset([bool value = false]);
}
