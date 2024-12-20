// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';

import 'package:convert/convert.dart';
import 'package:flutter_tools/src/build_system/hash.dart';

import '../../src/common.dart';

void main() {
  // Examples taken from https://en.wikipedia.org/wiki/MD5
  testWithoutContext('md5 control test zero length string', () {
    final Md5Hash hash = Md5Hash();

    expect(hex.encode(hash.finalize().buffer.asUint8List()), 'd41d8cd98f00b204e9800998ecf8427e');
  });

  testWithoutContext('md5 control test fox test', () {
    final Md5Hash hash = Md5Hash();
    hash.addChunk(ascii.encode('The quick brown fox jumps over the lazy dog'));

    expect(hex.encode(hash.finalize().buffer.asUint8List()), '9e107d9d372bb6826bd81d3542a419d6');
  });

  testWithoutContext('md5 control test fox test with period', () {
    final Md5Hash hash = Md5Hash();
    hash.addChunk(ascii.encode('The quick brown fox jumps over the lazy dog.'));

    expect(hex.encode(hash.finalize().buffer.asUint8List()), 'e4d909c290d0fb1ca068ffaddf22cbd0');
  });

  testWithoutContext('Can hash bytes less than 64 length', () {
    final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5, 6]);
    final Md5Hash hashA = Md5Hash();
    hashA.addChunk(bytes);

    expect(hashA.finalize(), <int>[1810219370, 268668871, 3900423769, 1277973076]);

    final Md5Hash hashB = Md5Hash();
    hashB.addChunk(bytes);

    expect(hashB.finalize(), <int>[1810219370, 268668871, 3900423769, 1277973076]);
  });

  testWithoutContext('Can hash bytes exactly 64 length', () {
    final Uint8List bytes = Uint8List.fromList(List<int>.filled(64, 2));
    final Md5Hash hashA = Md5Hash();
    hashA.addChunk(bytes);

    expect(hashA.finalize(), <int>[260592333, 2557619848, 2729912077, 812879060]);

    final Md5Hash hashB = Md5Hash();
    hashB.addChunk(bytes);

    expect(hashB.finalize(), <int>[260592333, 2557619848, 2729912077, 812879060]);
  });

  testWithoutContext('Can hash bytes more than 64 length', () {
    final Uint8List bytes = Uint8List.fromList(List<int>.filled(514, 2));
    final Md5Hash hashA = Md5Hash();
    hashA.addChunk(bytes);

    expect(hashA.finalize(), <int>[387658779, 2003142991, 243395797, 1487291259]);

    final Md5Hash hashB = Md5Hash();
    hashB.addChunk(bytes);

    expect(hashB.finalize(), <int>[387658779, 2003142991, 243395797, 1487291259]);
  });
}
