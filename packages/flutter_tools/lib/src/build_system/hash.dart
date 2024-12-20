// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

/// Data from a non-linear mathematical function that functions as
/// reproducible noise.
final Uint32List _noise = Uint32List.fromList(<int>[
  0xd76aa478,
  0xe8c7b756,
  0x242070db,
  0xc1bdceee,
  0xf57c0faf,
  0x4787c62a,
  0xa8304613,
  0xfd469501,
  0x698098d8,
  0x8b44f7af,
  0xffff5bb1,
  0x895cd7be,
  0x6b901122,
  0xfd987193,
  0xa679438e,
  0x49b40821,
  0xf61e2562,
  0xc040b340,
  0x265e5a51,
  0xe9b6c7aa,
  0xd62f105d,
  0x02441453,
  0xd8a1e681,
  0xe7d3fbc8,
  0x21e1cde6,
  0xc33707d6,
  0xf4d50d87,
  0x455a14ed,
  0xa9e3e905,
  0xfcefa3f8,
  0x676f02d9,
  0x8d2a4c8a,
  0xfffa3942,
  0x8771f681,
  0x6d9d6122,
  0xfde5380c,
  0xa4beea44,
  0x4bdecfa9,
  0xf6bb4b60,
  0xbebfbc70,
  0x289b7ec6,
  0xeaa127fa,
  0xd4ef3085,
  0x04881d05,
  0xd9d4d039,
  0xe6db99e5,
  0x1fa27cf8,
  0xc4ac5665,
  0xf4292244,
  0x432aff97,
  0xab9423a7,
  0xfc93a039,
  0x655b59c3,
  0x8f0ccc92,
  0xffeff47d,
  0x85845dd1,
  0x6fa87e4f,
  0xfe2ce6e0,
  0xa3014314,
  0x4e0811a1,
  0xf7537e82,
  0xbd3af235,
  0x2ad7d2bb,
  0xeb86d391,
]);

/// Per-round shift amounts.
const List<int> _shiftAmounts = <int>[
  07,
  12,
  17,
  22,
  07,
  12,
  17,
  22,
  07,
  12,
  17,
  22,
  07,
  12,
  17,
  22,
  05,
  09,
  14,
  20,
  05,
  09,
  14,
  20,
  05,
  09,
  14,
  20,
  05,
  09,
  14,
  20,
  04,
  11,
  16,
  23,
  04,
  11,
  16,
  23,
  04,
  11,
  16,
  23,
  04,
  11,
  16,
  23,
  06,
  10,
  15,
  21,
  06,
  10,
  15,
  21,
  06,
  10,
  15,
  21,
  06,
  10,
  15,
  21,
];

/// A bitmask that limits an integer to 32 bits.
const int _mask32 = 0xFFFFFFFF;

/// An incremental hash computation of md5.
class Md5Hash {
  Md5Hash() {
    _digest[0] = 0x67452301;
    _digest[1] = 0xefcdab89;
    _digest[2] = 0x98badcfe;
    _digest[3] = 0x10325476;
  }

  // 64 bytes is 512 bits.
  static const int _kChunkSize = 64;

  /// The current hash digest.
  final Uint32List _digest = Uint32List(4);
  final Uint8List _scratchSpace = Uint8List(_kChunkSize);
  int _remainingLength = 0;
  int _contentLength = 0;

  void addChunk(Uint8List data, [int? stop]) {
    assert(_remainingLength == 0);
    stop ??= data.length;
    int i = 0;
    for (; i <= stop - _kChunkSize; i += _kChunkSize) {
      final Uint32List view = Uint32List.view(data.buffer, i, 16);
      _writeChunk(view);
    }
    if (i != stop) {
      // The data must be copied so that the provided buffer can be reused.
      int j = 0;
      for (; i < stop; i += 1) {
        _scratchSpace[j] = data[i];
        j += 1;
      }
      _remainingLength = j;
    }
    _contentLength += stop;
  }

  void _writeChunk(Uint32List chunk) {
    // help dart remove bounds checks
    // ignore: unnecessary_statements
    chunk[15];
    // ignore: unnecessary_statements
    _shiftAmounts[63];
    // ignore: unnecessary_statements
    _noise[63];

    int d = _digest[3];
    int c = _digest[2];
    int b = _digest[1];
    int a = _digest[0];
    int e = 0;
    int f = 0;
    int i = 0;
    for (; i < 16; i += 1) {
      e = (b & c) | ((~b & _mask32) & d);
      f = i;
      final int temp = d;
      d = c;
      c = b;
      b = _add32(b, _rotl32(_add32(_add32(a, e), _add32(_noise[i], chunk[f])), _shiftAmounts[i]));
      a = temp;
    }
    for (; i < 32; i += 1) {
      e = (d & b) | ((~d & _mask32) & c);
      f = ((5 * i) + 1) % 16;
      final int temp = d;
      d = c;
      c = b;
      b = _add32(b, _rotl32(_add32(_add32(a, e), _add32(_noise[i], chunk[f])), _shiftAmounts[i]));
      a = temp;
    }
    for (; i < 48; i += 1) {
      e = b ^ c ^ d;
      f = ((3 * i) + 5) % 16;
      final int temp = d;
      d = c;
      c = b;
      b = _add32(b, _rotl32(_add32(_add32(a, e), _add32(_noise[i], chunk[f])), _shiftAmounts[i]));
      a = temp;
    }
    for (; i < 64; i += 1) {
      e = c ^ (b | (~d & _mask32));
      f = (7 * i) % 16;
      final int temp = d;
      d = c;
      c = b;
      b = _add32(b, _rotl32(_add32(_add32(a, e), _add32(_noise[i], chunk[f])), _shiftAmounts[i]));
      a = temp;
    }

    _digest[0] += a;
    _digest[1] += b;
    _digest[2] += c;
    _digest[3] += d;
  }

  Uint32List finalize() {
    // help dart remove bounds checks
    // ignore: unnecessary_statements
    _scratchSpace[63];
    _scratchSpace[_remainingLength] = 0x80;
    _remainingLength += 1;

    final int zeroes = 56 - _remainingLength;
    for (int i = _remainingLength; i < zeroes; i += 1) {
      _scratchSpace[i] = 0;
    }
    final int bitLength = _contentLength * 8;
    _scratchSpace.buffer.asByteData().setUint64(56, bitLength, Endian.little);
    _writeChunk(Uint32List.view(_scratchSpace.buffer, 0, 16));
    return _digest;
  }

  /// Adds [x] and [y] with 32-bit overflow semantics.
  int _add32(int x, int y) => (x + y) & _mask32;

  /// Bitwise rotates [val] to the left by [shift], obeying 32-bit overflow
  /// semantics.
  int _rotl32(int val, int shift) {
    final int modShift = shift & 31;
    return ((val << modShift) & _mask32) | ((val & _mask32) >> (32 - modShift));
  }
}
