// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/convert.dart';

import '../src/common.dart';

void main() {
  late String passedString;
  late String nonpassString;

  const Utf8Decoder decoder = Utf8Decoder();

  setUp(() {
    passedString = 'normal string';
    nonpassString = 'malformed string => �';
  });

  testWithoutContext('Decode a normal string', () async {
    expect(decoder.convert(passedString.codeUnits), passedString);
  });

  testWithoutContext('Decode a malformed string', () async {
    expect(
      () => decoder.convert(nonpassString.codeUnits),
      throwsA(
        isA<ToolExit>().having(
          (ToolExit error) => error.message,
          'message',
          contains('(U+FFFD; REPLACEMENT CHARACTER)'), // Added paragraph
        ),
      ),
    );
  });
}
