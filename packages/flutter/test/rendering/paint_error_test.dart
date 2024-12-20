// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  // This test has to be kept separate from object_test.dart because the way
  // the rendering_test.dart dependency of this test uses the bindings in not
  // compatible with existing tests in object_test.dart.
  test('reentrant paint error', () {
    late FlutterErrorDetails errorDetails;
    final RenderBox root = TestReentrantPaintingErrorRenderBox();
    layout(
      root,
      onErrors: () {
        errorDetails = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails()!;
      },
    );
    pumpFrame(phase: EnginePhase.paint);

    expect(errorDetails, isNotNull);
    expect(errorDetails.stack, isNotNull);
    // Check the ErrorDetails without the stack trace
    final List<String> lines = errorDetails.toString().split('\n');
    // The lines in the middle of the error message contain the stack trace
    // which will change depending on where the test is run.
    expect(lines.length, greaterThan(12));
    expect(
      lines.take(12).join('\n'),
      equalsIgnoringHashCodes(
        '══╡ EXCEPTION CAUGHT BY RENDERING LIBRARY ╞══════════════════════\n'
        'The following assertion was thrown during paint():\n'
        'Tried to paint a RenderObject reentrantly.\n'
        'The following RenderObject was already being painted when it was painted again:\n'
        '  TestReentrantPaintingErrorRenderBox#00000:\n'
        '  parentData: <none>\n'
        '  constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '  size: Size(100.0, 100.0)\n'
        'Since this typically indicates an infinite recursion, it is\n'
        'disallowed.\n'
        '\n'
        'When the exception was thrown, this was the stack:',
      ),
    );

    expect(
      lines.getRange(lines.length - 8, lines.length).join('\n'),
      equalsIgnoringHashCodes(
        'The following RenderObject was being processed when the exception was fired:\n'
        '  TestReentrantPaintingErrorRenderBox#00000:\n'
        '  parentData: <none>\n'
        '  constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '  size: Size(100.0, 100.0)\n'
        'This RenderObject has no descendants.\n'
        '═════════════════════════════════════════════════════════════════\n',
      ),
    );
  });

  test('needsCompositingBitsUpdate paint error', () {
    late FlutterError flutterError;
    final RenderBox root = RenderRepaintBoundary(child: RenderSizedBox(const Size(100, 100)));
    try {
      layout(root);
      PaintingContext.repaintCompositedChild(root, debugAlsoPaintedParent: true);
    } on FlutterError catch (exception) {
      flutterError = exception;
    }

    expect(flutterError, isNotNull);
    // The lines in the middle of the error message contain the stack trace
    // which will change depending on where the test is run.
    expect(
      flutterError.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   Tried to paint a RenderObject before its compositing bits were\n'
        '   updated.\n'
        '   The following RenderObject was marked as having dirty compositing bits at the time that it was painted:\n'
        '     RenderRepaintBoundary#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE:\n'
        '     needs compositing\n'
        '     parentData: <none>\n'
        '     constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     layer: OffsetLayer#00000 DETACHED\n'
        '     size: Size(800.0, 600.0)\n'
        '     metrics: 0.0% useful (1 bad vs 0 good)\n'
        '     diagnosis: insufficient data to draw conclusion (less than five\n'
        '       repaints)\n'
        '   A RenderObject that still has dirty compositing bits cannot be\n'
        '   painted because this indicates that the tree has not yet been\n'
        '   properly configured for creating the layer tree.\n'
        '   This usually indicates an error in the Flutter framework itself.\n',
      ),
    );
    expect(
      flutterError.diagnostics
          .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
          .toString(),
      'This usually indicates an error in the Flutter framework itself.',
    );
  });
}

class TestReentrantPaintingErrorRenderBox extends RenderBox {
  @override
  void paint(PaintingContext context, Offset offset) {
    // Cause a reentrant painting bug that would show up as a stack overflow if
    // it was not for debugging checks in RenderObject.
    context.paintChild(this, offset);
  }

  @override
  void performLayout() {
    size = const Size(100, 100);
  }
}
