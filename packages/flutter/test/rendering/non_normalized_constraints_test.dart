// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// THIS TEST IS SENSITIVE TO LINE NUMBERS AT THE TOP OF THIS FILE
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class RenderFoo extends RenderShiftedBox {
  RenderFoo({RenderBox? child}) : super(child);

  @override
  void performLayout() {
    child?.layout(
      const BoxConstraints(
        // THIS MUST BE LINE 17
        minWidth: 100.0,
        maxWidth: 50.0,
      ),
    );
  }
}

class Foo extends SingleChildRenderObjectWidget {
  const Foo({super.key, super.child});

  @override
  RenderFoo createRenderObject(BuildContext context) {
    return RenderFoo();
  }
}

// END OF SENSITIVE SECTION

void main() {
  testWidgets('Stack parsing in non-normalized constraints error', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Foo(child: Placeholder()),
      duration: Duration.zero,
      phase: EnginePhase.layout,
    );
    final Object? exception = tester.takeException();
    final String text = exception.toString();
    expect(text, contains('BoxConstraints has non-normalized width constraints.'));
    expect(
      text,
      contains(
        'which probably computed the invalid constraints in question:\n  RenderFoo.performLayout (',
      ),
    );
    expect(text, contains('non_normalized_constraints_test.dart:'));
  }, skip: kIsWeb); // [intended] stack traces on web are insufficiently predictable
}
