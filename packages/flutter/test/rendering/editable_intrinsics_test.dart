// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TextSelectionDelegate delegate = _FakeEditableTextState();

  test('editable intrinsics', () {
    final RenderEditable editable = RenderEditable(
      text: const TextSpan(style: TextStyle(height: 1.0, fontSize: 10.0), text: '12345'),
      startHandleLayerLink: LayerLink(),
      endHandleLayerLink: LayerLink(),
      textDirection: TextDirection.ltr,
      locale: const Locale('ja', 'JP'),
      offset: ViewportOffset.zero(),
      textSelectionDelegate: delegate,
    );
    expect(editable.getMinIntrinsicWidth(double.infinity), 50.0);
    // The width includes the width of the cursor (1.0).
    expect(editable.getMaxIntrinsicWidth(double.infinity), 52.0);
    expect(editable.getMinIntrinsicHeight(double.infinity), 10.0);
    expect(editable.getMaxIntrinsicHeight(double.infinity), 10.0);

    expect(
      editable.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderEditable#00000 NEEDS-LAYOUT NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE DETACHED\n'
        ' │ parentData: MISSING\n'
        ' │ constraints: MISSING\n'
        ' │ size: MISSING\n'
        ' │ cursorColor: null\n'
        ' │ showCursor: ValueNotifier<bool>#00000(false)\n'
        ' │ maxLines: 1\n'
        ' │ minLines: null\n'
        ' │ selectionColor: null\n'
        ' │ locale: ja_JP\n'
        ' │ selection: null\n'
        ' │ offset: _FixedViewportOffset#00000(offset: 0.0)\n'
        ' ╘═╦══ text ═══\n'
        '   ║ TextSpan:\n'
        '   ║   inherit: true\n'
        '   ║   size: 10.0\n'
        '   ║   height: 1.0x\n'
        '   ║   "12345"\n'
        '   ╚═══════════\n',
      ),
    );
  });

  test('textScaler affects intrinsics', () {
    final RenderEditable editable = RenderEditable(
      text: const TextSpan(style: TextStyle(fontSize: 10), text: 'Hello World'),
      textDirection: TextDirection.ltr,
      startHandleLayerLink: LayerLink(),
      endHandleLayerLink: LayerLink(),
      offset: ViewportOffset.zero(),
      textSelectionDelegate: delegate,
    );

    expect(editable.getMaxIntrinsicWidth(double.infinity), 110 + 2);

    editable.textScaler = const TextScaler.linear(2);
    expect(editable.getMaxIntrinsicWidth(double.infinity), 220 + 2);
  });

  test('maxLines affects intrinsics', () {
    final RenderEditable editable = RenderEditable(
      text: TextSpan(
        style: const TextStyle(fontSize: 10),
        text: List<String>.filled(5, 'A').join('\n'),
      ),
      textDirection: TextDirection.ltr,
      startHandleLayerLink: LayerLink(),
      endHandleLayerLink: LayerLink(),
      offset: ViewportOffset.zero(),
      textSelectionDelegate: delegate,
      maxLines: null,
    );

    expect(editable.getMaxIntrinsicHeight(double.infinity), 50);

    editable.maxLines = 1;
    expect(editable.getMaxIntrinsicHeight(double.infinity), 10);
  });

  test(
    'strutStyle affects intrinsics',
    () {
      final RenderEditable editable = RenderEditable(
        text: const TextSpan(style: TextStyle(fontSize: 10), text: 'Hello World'),
        textDirection: TextDirection.ltr,
        startHandleLayerLink: LayerLink(),
        endHandleLayerLink: LayerLink(),
        offset: ViewportOffset.zero(),
        textSelectionDelegate: delegate,
      );

      expect(editable.getMaxIntrinsicHeight(double.infinity), 10);

      editable.strutStyle = const StrutStyle(fontSize: 100, forceStrutHeight: true);
      expect(editable.getMaxIntrinsicHeight(double.infinity), 100);
    },
    // [intended] strut support for HTML renderer https://github.com/flutter/flutter/issues/32243.
    skip: kIsWeb && !isSkiaWeb,
  );
}

class _FakeEditableTextState with TextSelectionDelegate {
  @override
  TextEditingValue textEditingValue = TextEditingValue.empty;

  TextSelection? selection;

  @override
  void hideToolbar([bool hideHandles = true]) {}

  @override
  void userUpdateTextEditingValue(TextEditingValue value, SelectionChangedCause cause) {
    selection = value.selection;
  }

  @override
  void bringIntoView(TextPosition position) {}

  @override
  void cutSelection(SelectionChangedCause cause) {}

  @override
  Future<void> pasteText(SelectionChangedCause cause) {
    return Future<void>.value();
  }

  @override
  void selectAll(SelectionChangedCause cause) {}

  @override
  void copySelection(SelectionChangedCause cause) {}
}
