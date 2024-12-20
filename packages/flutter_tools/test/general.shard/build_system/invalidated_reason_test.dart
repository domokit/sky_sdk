// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/build_system/build_system.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('InvalidatedReason formats message per invalidation kind', () {
    final InvalidatedReason inputChanged = InvalidatedReason(InvalidatedReasonKind.inputChanged)
      ..data.add('a.dart');
    final InvalidatedReason outputChanged = InvalidatedReason(InvalidatedReasonKind.outputChanged)
      ..data.add('b.dart');
    final InvalidatedReason inputMissing = InvalidatedReason(InvalidatedReasonKind.inputMissing)
      ..data.add('c.dart');
    final InvalidatedReason outputMissing = InvalidatedReason(InvalidatedReasonKind.outputMissing)
      ..data.add('d.dart');
    final InvalidatedReason outputSetChanged = InvalidatedReason(
      InvalidatedReasonKind.outputSetChanged,
    )..data.add('e.dart');

    expect(inputChanged.toString(), 'The following inputs have updated contents: a.dart');
    expect(outputChanged.toString(), 'The following outputs have updated contents: b.dart');
    expect(inputMissing.toString(), 'The following inputs were missing: c.dart');
    expect(outputMissing.toString(), 'The following outputs were missing: d.dart');
    expect(
      outputSetChanged.toString(),
      'The following outputs were removed from the output set: e.dart',
    );
  });
}
