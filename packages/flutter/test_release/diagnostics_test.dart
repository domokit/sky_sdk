// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('TextTreeRenderer returns an empty string in release mode', () {
    final TextTreeRenderer renderer = TextTreeRenderer();
    final TestDiagnosticsNode node = TestDiagnosticsNode();

    expect(renderer.render(node), '');
  });
}

class TestDiagnosticsNode extends DiagnosticsNode {
  TestDiagnosticsNode() : super(name: 'test', style: DiagnosticsTreeStyle.singleLine);

  @override
  List<DiagnosticsNode> getChildren() {
    return <DiagnosticsNode>[];
  }

  @override
  List<DiagnosticsNode> getProperties() {
    return <DiagnosticsNode>[];
  }

  @override
  String toDescription({TextTreeConfiguration? parentConfiguration}) {
    return 'Test Description';
  }

  @override
  final Object? value = Object();
}
