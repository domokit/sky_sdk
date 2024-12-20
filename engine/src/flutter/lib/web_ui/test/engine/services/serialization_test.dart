// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

// Some tests here cannot run in JS mode because of limited numerics. This
// should be fixed by dart2wasm.
const bool _kSupportsDartNumerics = !identical(0, 0.0);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('Write and read buffer round-trip', () {
    test('of single byte', () {
      final WriteBuffer write = WriteBuffer();
      write.putUint8(201);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(1));
      final ReadBuffer read = ReadBuffer(written);
      expect(read.getUint8(), equals(201));
    });
    test('of 32-bit integer', () {
      final WriteBuffer write = WriteBuffer();
      write.putInt32(-9);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(4));
      final ReadBuffer read = ReadBuffer(written);
      expect(read.getInt32(), equals(-9));
    });
    test('of 64-bit integer', () {
      final WriteBuffer write = WriteBuffer();
      write.putInt64(-9000000000000);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(8));
      final ReadBuffer read = ReadBuffer(written);
      expect(read.getInt64(), equals(-9000000000000));
    }, skip: !_kSupportsDartNumerics);
    test('of double', () {
      final WriteBuffer write = WriteBuffer();
      write.putFloat64(3.14);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(8));
      final ReadBuffer read = ReadBuffer(written);
      expect(read.getFloat64(), equals(3.14));
    });
    test('of 32-bit int list when unaligned', () {
      final Int32List integers = Int32List.fromList(<int>[-99, 2, 99]);
      final WriteBuffer write = WriteBuffer();
      write.putUint8(9);
      write.putInt32List(integers);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(16));
      final ReadBuffer read = ReadBuffer(written);
      read.getUint8();
      expect(read.getInt32List(3), equals(integers));
    });
    test('of 64-bit int list when unaligned', () {
      final Int64List integers = Int64List.fromList(<int>[-99, 2, 99]);
      final WriteBuffer write = WriteBuffer();
      write.putUint8(9);
      write.putInt64List(integers);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(32));
      final ReadBuffer read = ReadBuffer(written);
      read.getUint8();
      expect(read.getInt64List(3), equals(integers));
    }, skip: !_kSupportsDartNumerics);
    test('of double list when unaligned', () {
      final Float64List doubles = Float64List.fromList(<double>[3.14, double.nan]);
      final WriteBuffer write = WriteBuffer();
      write.putUint8(9);
      write.putFloat64List(doubles);
      final ByteData written = write.done();
      expect(written.lengthInBytes, equals(24));
      final ReadBuffer read = ReadBuffer(written);
      read.getUint8();
      final Float64List readDoubles = read.getFloat64List(2);
      expect(readDoubles[0], equals(3.14));
      expect(readDoubles[1], isNaN);
    });
  });
}
