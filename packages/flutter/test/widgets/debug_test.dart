// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('debugChildrenHaveDuplicateKeys control test', () {
    const Key key = Key('key');
    final List<Widget> children = <Widget>[Container(key: key), Container(key: key)];
    final Widget widget = Flex(direction: Axis.vertical, children: children);
    late FlutterError error;
    try {
      debugChildrenHaveDuplicateKeys(widget, children);
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   Duplicate keys found.\n'
          '   If multiple keyed widgets exist as children of another widget,\n'
          '   they must have unique keys.\n'
          '   Flex(direction: vertical, mainAxisAlignment: start,\n'
          '   crossAxisAlignment: center) has multiple children with key\n'
          "   [<'key'>].\n",
        ),
      );
    }
  });

  test('debugItemsHaveDuplicateKeys control test', () {
    const Key key = Key('key');
    final List<Widget> items = <Widget>[Container(key: key), Container(key: key)];
    late FlutterError error;
    try {
      debugItemsHaveDuplicateKeys(items);
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          "   Duplicate key found: [<'key'>].\n",
        ),
      );
    }
  });

  testWidgets('debugCheckHasTable control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          late FlutterError error;
          try {
            debugCheckHasTable(context);
          } on FlutterError catch (e) {
            error = e;
          } finally {
            expect(error, isNotNull);
            expect(error.diagnostics.length, 4);
            expect(error.diagnostics[2], isA<DiagnosticsProperty<Element>>());
            expect(
              error.toStringDeep(),
              startsWith(
                'FlutterError\n'
                '   No Table widget found.\n'
                '   Builder widgets require a Table widget ancestor.\n'
                '   The specific widget that could not find a Table ancestor was:\n'
                '     Builder\n'
                '   The ownership chain for the affected widget is: "Builder ←', // End of ownership chain omitted, not relevant for test.
              ),
            );
          }
          return Container();
        },
      ),
    );
  });

  testWidgets('debugCheckHasMediaQuery control test', (WidgetTester tester) async {
    // Cannot use tester.pumpWidget here because it wraps the widget in a View,
    // which introduces a MediaQuery ancestor.
    await tester.pumpWidget(
      wrapWithView: false,
      Builder(
        builder: (BuildContext context) {
          late FlutterError error;
          try {
            debugCheckHasMediaQuery(context);
          } on FlutterError catch (e) {
            error = e;
          } finally {
            expect(error, isNotNull);
            expect(error.diagnostics.length, 5);
            expect(error.diagnostics[2], isA<DiagnosticsProperty<Element>>());
            expect(error.diagnostics.last.level, DiagnosticLevel.hint);
            expect(
              error.diagnostics.last.toStringDeep(),
              equalsIgnoringHashCodes(
                'No MediaQuery ancestor could be found starting from the context\n'
                'that was passed to MediaQuery.of(). This can happen because the\n'
                'context used is not a descendant of a View widget, which\n'
                'introduces a MediaQuery.\n',
              ),
            );
            expect(
              error.toStringDeep(),
              startsWith(
                'FlutterError\n'
                '   No MediaQuery widget ancestor found.\n'
                '   Builder widgets require a MediaQuery widget ancestor.\n'
                '   The specific widget that could not find a MediaQuery ancestor\n'
                '   was:\n'
                '     Builder\n'
                '   The ownership chain for the affected widget is: "Builder ←', // Full chain omitted, not relevant for test.
              ),
            );
            expect(
              error.toStringDeep(),
              endsWith(
                '[root]"\n' // End of ownership chain.
                '   No MediaQuery ancestor could be found starting from the context\n'
                '   that was passed to MediaQuery.of(). This can happen because the\n'
                '   context used is not a descendant of a View widget, which\n'
                '   introduces a MediaQuery.\n',
              ),
            );
          }
          return View(view: tester.view, child: const SizedBox());
        },
      ),
    );
  });

  test('debugWidgetBuilderValue control test', () {
    final Widget widget = Container();
    FlutterError? error;
    try {
      debugWidgetBuilderValue(widget, null);
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(error!.diagnostics.length, 4);
      expect(error.diagnostics[1], isA<DiagnosticsProperty<Widget>>());
      expect(error.diagnostics[1].style, DiagnosticsTreeStyle.errorProperty);
      expect(
        error.diagnostics[1].toStringDeep(),
        equalsIgnoringHashCodes(
          'The offending widget is:\n'
          '  Container\n',
        ),
      );
      expect(error.diagnostics[2].level, DiagnosticLevel.info);
      expect(error.diagnostics[3].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[3].toStringDeep(),
        equalsIgnoringHashCodes(
          'To return an empty space that causes the building widget to fill\n'
          'available room, return "Container()". To return an empty space\n'
          'that takes as little room as possible, return "Container(width:\n'
          '0.0, height: 0.0)".\n',
        ),
      );
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   A build function returned null.\n'
          '   The offending widget is:\n'
          '     Container\n'
          '   Build functions must never return null.\n'
          '   To return an empty space that causes the building widget to fill\n'
          '   available room, return "Container()". To return an empty space\n'
          '   that takes as little room as possible, return "Container(width:\n'
          '   0.0, height: 0.0)".\n',
        ),
      );
      error = null;
    }
    try {
      debugWidgetBuilderValue(widget, widget);
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(error!.diagnostics.length, 3);
      expect(error.diagnostics[1], isA<DiagnosticsProperty<Widget>>());
      expect(error.diagnostics[1].style, DiagnosticsTreeStyle.errorProperty);
      expect(
        error.diagnostics[1].toStringDeep(),
        equalsIgnoringHashCodes(
          'The offending widget is:\n'
          '  Container\n',
        ),
      );
      expect(
        error.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   A build function returned context.widget.\n'
          '   The offending widget is:\n'
          '     Container\n'
          "   Build functions must never return their BuildContext parameter's\n"
          '   widget or a child that contains "context.widget". Doing so\n'
          '   introduces a loop in the widget tree that can cause the app to\n'
          '   crash.\n',
        ),
      );
    }
  });

  testWidgets('debugCheckHasWidgetsLocalizations throws', (WidgetTester tester) async {
    final GlobalKey noLocalizationsAvailable = GlobalKey();
    final GlobalKey localizationsAvailable = GlobalKey();

    await tester.pumpWidget(
      Container(
        key: noLocalizationsAvailable,
        child: WidgetsApp(
          builder: (BuildContext context, Widget? child) {
            return Container(key: localizationsAvailable);
          },
          color: const Color(0xFF4CAF50),
        ),
      ),
    );

    expect(
      () => debugCheckHasWidgetsLocalizations(noLocalizationsAvailable.currentContext!),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.message,
          'message',
          contains('No WidgetsLocalizations found'),
        ),
      ),
    );

    expect(debugCheckHasWidgetsLocalizations(localizationsAvailable.currentContext!), isTrue);
  });

  test('debugAssertAllWidgetVarsUnset', () {
    debugHighlightDeprecatedWidgets = true;
    late FlutterError error;
    try {
      debugAssertAllWidgetVarsUnset(
        'The value of a widget debug variable was changed by the test.',
      );
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(error.diagnostics.length, 1);
      expect(
        error.toStringDeep(),
        'FlutterError\n'
        '   The value of a widget debug variable was changed by the test.\n',
      );
    }
    debugHighlightDeprecatedWidgets = false;
  });

  testWidgets('debugCreator of layers should not be null', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Stack(
              children: <Widget>[
                const ColorFiltered(
                  colorFilter: ColorFilter.mode(Color(0xFFFF0000), BlendMode.color),
                  child: Placeholder(),
                ),
                const Opacity(opacity: 0.9, child: Placeholder()),
                ImageFiltered(
                  imageFilter: ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
                  child: const Placeholder(),
                ),
                BackdropFilter(
                  filter: ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
                  child: const Placeholder(),
                ),
                ShaderMask(
                  shaderCallback:
                      (Rect bounds) => const RadialGradient(
                        radius: 0.05,
                        colors: <Color>[Color(0xFFFF0000), Color(0xFF00FF00)],
                        tileMode: TileMode.mirror,
                      ).createShader(bounds),
                  child: const Placeholder(),
                ),
                CompositedTransformFollower(link: LayerLink()),
              ],
            ),
          ),
        ),
      ),
    );

    RenderObject renderObject;

    renderObject = tester.firstRenderObject(find.byType(Opacity));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);

    renderObject = tester.firstRenderObject(find.byType(ColorFiltered));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);

    renderObject = tester.firstRenderObject(find.byType(ImageFiltered));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);

    renderObject = tester.firstRenderObject(find.byType(BackdropFilter));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);

    renderObject = tester.firstRenderObject(find.byType(ShaderMask));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);

    renderObject = tester.firstRenderObject(find.byType(CompositedTransformFollower));
    expect(renderObject.debugLayer?.debugCreator, isNotNull);
  });
}
