// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(LongCatIsLooong): Remove this file once textScaleFactor is removed.
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('TextStyle', () {
    test('getTextStyle is backward compatible', () {
      expect(
        const TextStyle(fontSize: 14).getTextStyle(textScaleFactor: 2.0).toString(),
        contains('fontSize: 28'),
      );
    }, skip: kIsWeb); // [intended] CkTextStyle doesn't have a custom toString implementation.
  });
  group('TextPainter', () {
    test('textScaleFactor translates to textScaler', () {
      final TextPainter textPainter = TextPainter(
        text: const TextSpan(text: 'text'),
        textDirection: TextDirection.ltr,
        textScaleFactor: 42,
      );

      expect(textPainter.textScaler, const TextScaler.linear(42.0));

      // Linear TextScaler translates to textScaleFactor.
      textPainter.textScaler = const TextScaler.linear(12.0);
      expect(textPainter.textScaleFactor, 12.0);

      textPainter.textScaleFactor = 10;
      expect(textPainter.textScaler, const TextScaler.linear(10));
    });
  });

  group('MediaQuery', () {
    test('specifying both textScaler and textScalingFactor asserts', () {
      expect(
        () => MediaQueryData(textScaleFactor: 2, textScaler: const TextScaler.linear(2.0)),
        throwsAssertionError,
      );
    });

    test('copyWith is backward compatible', () {
      const MediaQueryData data = MediaQueryData(textScaler: TextScaler.linear(2.0));

      final MediaQueryData data1 = data.copyWith(textScaleFactor: 42);
      expect(data1.textScaler, const TextScaler.linear(42));
      expect(data1.textScaleFactor, 42);

      final MediaQueryData data2 = data.copyWith(textScaler: TextScaler.noScaling);
      expect(data2.textScaler, TextScaler.noScaling);
      expect(data2.textScaleFactor, 1.0);
    });

    test('copyWith specifying both textScaler and textScalingFactor asserts', () {
      const MediaQueryData data = MediaQueryData();
      expect(
        () => data.copyWith(textScaleFactor: 2, textScaler: const TextScaler.linear(2.0)),
        throwsAssertionError,
      );
    });

    testWidgets('MediaQuery.textScaleFactorOf overriding compatibility', (
      WidgetTester tester,
    ) async {
      late final double outsideTextScaleFactor;
      late final TextScaler outsideTextScaler;
      late final double insideTextScaleFactor;
      late final TextScaler insideTextScaler;

      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            outsideTextScaleFactor = MediaQuery.textScaleFactorOf(context);
            outsideTextScaler = MediaQuery.textScalerOf(context);
            return MediaQuery(
              data: const MediaQueryData(textScaleFactor: 4.0),
              child: Builder(
                builder: (BuildContext context) {
                  insideTextScaleFactor = MediaQuery.textScaleFactorOf(context);
                  insideTextScaler = MediaQuery.textScalerOf(context);
                  return Container();
                },
              ),
            );
          },
        ),
      );

      // Overriding textScaleFactor should work for unmigrated widgets that are
      // still using MediaQuery.textScaleFactorOf. Also if a unmigrated widget
      // overrides MediaQuery.textScaleFactor, migrated widgets in the subtree
      // should get the correct TextScaler.
      expect(outsideTextScaleFactor, 1.0);
      expect(outsideTextScaler.textScaleFactor, 1.0);
      expect(outsideTextScaler, TextScaler.noScaling);
      expect(insideTextScaleFactor, 4.0);
      expect(insideTextScaler.textScaleFactor, 4.0);
      expect(insideTextScaler, const TextScaler.linear(4.0));
    });

    testWidgets('textScaleFactor overriding backward compatibility', (WidgetTester tester) async {
      late final double outsideTextScaleFactor;
      late final TextScaler outsideTextScaler;
      late final double insideTextScaleFactor;
      late final TextScaler insideTextScaler;

      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            outsideTextScaleFactor = MediaQuery.textScaleFactorOf(context);
            outsideTextScaler = MediaQuery.textScalerOf(context);
            return MediaQuery(
              data: const MediaQueryData(textScaler: TextScaler.linear(4.0)),
              child: Builder(
                builder: (BuildContext context) {
                  insideTextScaleFactor = MediaQuery.textScaleFactorOf(context);
                  insideTextScaler = MediaQuery.textScalerOf(context);
                  return Container();
                },
              ),
            );
          },
        ),
      );

      expect(outsideTextScaleFactor, 1.0);
      expect(outsideTextScaler.textScaleFactor, 1.0);
      expect(outsideTextScaler, TextScaler.noScaling);
      expect(insideTextScaleFactor, 4.0);
      expect(insideTextScaler.textScaleFactor, 4.0);
      expect(insideTextScaler, const TextScaler.linear(4.0));
    });
  });

  group('RenderObjects backward compatibility', () {
    test('RenderEditable', () {
      final RenderEditable renderObject = RenderEditable(
        backgroundCursorColor: const Color.fromARGB(0xFF, 0xFF, 0x00, 0x00),
        textDirection: TextDirection.ltr,
        cursorColor: const Color.fromARGB(0xFF, 0xFF, 0x00, 0x00),
        offset: ViewportOffset.zero(),
        textSelectionDelegate: _FakeEditableTextState(),
        text: const TextSpan(text: 'test', style: TextStyle(height: 1.0, fontSize: 10.0)),
        startHandleLayerLink: LayerLink(),
        endHandleLayerLink: LayerLink(),
        selection: const TextSelection.collapsed(offset: 0),
      );
      expect(renderObject.textScaleFactor, 1.0);

      renderObject.textScaleFactor = 3.0;
      expect(renderObject.textScaleFactor, 3.0);
      expect(renderObject.textScaler, const TextScaler.linear(3.0));

      renderObject.textScaler = const TextScaler.linear(4.0);
      expect(renderObject.textScaleFactor, 4.0);
    });

    test('RenderParagraph', () {
      final RenderParagraph renderObject = RenderParagraph(
        const TextSpan(text: 'test', style: TextStyle(height: 1.0, fontSize: 10.0)),
        textDirection: TextDirection.ltr,
      );
      expect(renderObject.textScaleFactor, 1.0);

      renderObject.textScaleFactor = 3.0;
      expect(renderObject.textScaleFactor, 3.0);
      expect(renderObject.textScaler, const TextScaler.linear(3.0));

      renderObject.textScaler = const TextScaler.linear(4.0);
      expect(renderObject.textScaleFactor, 4.0);
    });
  });

  group('Widgets backward compatibility', () {
    testWidgets('RichText', (WidgetTester tester) async {
      await tester.pumpWidget(
        RichText(textDirection: TextDirection.ltr, text: const TextSpan(), textScaleFactor: 2.0),
      );

      expect(
        tester.renderObject<RenderParagraph>(find.byType(RichText)).textScaler,
        const TextScaler.linear(2.0),
      );
      expect(tester.renderObject<RenderParagraph>(find.byType(RichText)).textScaleFactor, 2.0);
    });

    testWidgets('Text', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Text('text', textDirection: TextDirection.ltr, textScaleFactor: 2.0),
      );

      expect(
        tester.renderObject<RenderParagraph>(find.text('text')).textScaler,
        const TextScaler.linear(2.0),
      );
    });

    testWidgets('EditableText', (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode(debugLabel: 'EditableText Node');
      addTearDown(focusNode.dispose);
      const TextStyle textStyle = TextStyle();
      const Color cursorColor = Color.fromARGB(0xFF, 0xFF, 0x00, 0x00);
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: EditableText(
              backgroundCursorColor: cursorColor,
              controller: controller,
              focusNode: focusNode,
              style: textStyle,
              cursorColor: cursorColor,
              textScaleFactor: 2.0,
            ),
          ),
        ),
      );

      final RenderEditable renderEditable =
          tester.allRenderObjects.whereType<RenderEditable>().first;
      expect(renderEditable.textScaler, const TextScaler.linear(2.0));
    });
  });
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
