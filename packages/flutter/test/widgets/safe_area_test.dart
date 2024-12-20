// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('SafeArea', () {
    testWidgets('SafeArea - basic', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.all(20.0)),
          child: SafeArea(left: false, child: Placeholder()),
        ),
      );
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(0.0, 20.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(780.0, 580.0));
    });

    testWidgets('SafeArea - with minimums', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.all(20.0)),
          child: SafeArea(
            top: false,
            minimum: EdgeInsets.fromLTRB(0.0, 10.0, 20.0, 30.0),
            child: Placeholder(),
          ),
        ),
      );
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(20.0, 10.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(780.0, 570.0));
    });

    testWidgets('SafeArea - nested', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.all(20.0)),
          child: SafeArea(top: false, child: SafeArea(right: false, child: Placeholder())),
        ),
      );
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(20.0, 20.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(780.0, 580.0));
    });

    testWidgets('SafeArea - changing', (WidgetTester tester) async {
      const Widget child = SafeArea(
        bottom: false,
        child: SafeArea(left: false, bottom: false, child: Placeholder()),
      );
      await tester.pumpWidget(
        const MediaQuery(data: MediaQueryData(padding: EdgeInsets.all(20.0)), child: child),
      );
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(20.0, 20.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(780.0, 600.0));
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.only(left: 100.0, top: 30.0, bottom: 40.0)),
          child: child,
        ),
      );
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(100.0, 30.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(800.0, 600.0));
    });

    testWidgets('SafeArea - properties', (WidgetTester tester) async {
      final SafeArea child = SafeArea(right: false, bottom: false, child: Container());
      final DiagnosticPropertiesBuilder properties = DiagnosticPropertiesBuilder();
      child.debugFillProperties(properties);

      expect(
        properties.properties.any(
          (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid left padding',
        ),
        true,
      );
      expect(
        properties.properties.any(
          (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid right padding',
        ),
        false,
      );
      expect(
        properties.properties.any(
          (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid top padding',
        ),
        true,
      );
      expect(
        properties.properties.any(
          (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid bottom padding',
        ),
        false,
      );
    });

    group('SafeArea maintains bottom viewPadding when specified for consumed bottom padding', () {
      Widget boilerplate(Widget child) {
        return Localizations(
          locale: const Locale('en', 'us'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultWidgetsLocalizations.delegate,
            DefaultMaterialLocalizations.delegate,
          ],
          child: Directionality(textDirection: TextDirection.ltr, child: child),
        );
      }

      testWidgets('SafeArea alone.', (WidgetTester tester) async {
        final Widget child = boilerplate(
          const SafeArea(
            maintainBottomViewPadding: true,
            child: Column(children: <Widget>[Expanded(child: Placeholder())]),
          ),
        );

        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.symmetric(vertical: 20.0),
              viewPadding: EdgeInsets.only(bottom: 20.0),
            ),
            child: child,
          ),
        );
        final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
        // Consume bottom padding - as if by the keyboard opening
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(top: 20.0),
              viewPadding: EdgeInsets.only(bottom: 20.0),
              viewInsets: EdgeInsets.only(bottom: 300.0),
            ),
            child: child,
          ),
        );
        final Offset finalPoint = tester.getCenter(find.byType(Placeholder));
        expect(initialPoint, finalPoint);
      });

      testWidgets('SafeArea alone - partial ViewInsets consume Padding', (
        WidgetTester tester,
      ) async {
        final Widget child = boilerplate(
          const SafeArea(
            maintainBottomViewPadding: true,
            child: Column(children: <Widget>[Expanded(child: Placeholder())]),
          ),
        );

        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 20.0)),
            child: child,
          ),
        );
        final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
        // Consume bottom padding - as if by the keyboard opening
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              viewPadding: EdgeInsets.only(bottom: 20.0),
              viewInsets: EdgeInsets.only(bottom: 10.0),
            ),
            child: child,
          ),
        );
        final Offset finalPoint = tester.getCenter(find.byType(Placeholder));
        expect(initialPoint, finalPoint);
      });

      testWidgets('SafeArea with nested Scaffold', (WidgetTester tester) async {
        final Widget child = boilerplate(
          const SafeArea(
            maintainBottomViewPadding: true,
            child: Scaffold(
              resizeToAvoidBottomInset: false,
              body: Column(children: <Widget>[Expanded(child: Placeholder())]),
            ),
          ),
        );

        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.symmetric(vertical: 20.0),
              viewPadding: EdgeInsets.only(bottom: 20.0),
            ),
            child: child,
          ),
        );
        final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
        // Consume bottom padding - as if by the keyboard opening
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(top: 20.0),
              viewPadding: EdgeInsets.only(bottom: 20.0),
              viewInsets: EdgeInsets.only(bottom: 300.0),
            ),
            child: child,
          ),
        );
        final Offset finalPoint = tester.getCenter(find.byType(Placeholder));
        expect(initialPoint, finalPoint);
      });

      testWidgets('SafeArea with nested Scaffold  - partial ViewInsets consume Padding', (
        WidgetTester tester,
      ) async {
        final Widget child = boilerplate(
          const SafeArea(
            maintainBottomViewPadding: true,
            child: Scaffold(
              resizeToAvoidBottomInset: false,
              body: Column(children: <Widget>[Expanded(child: Placeholder())]),
            ),
          ),
        );

        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 20.0)),
            child: child,
          ),
        );
        final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
        // Consume bottom padding - as if by the keyboard opening
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(
              viewPadding: EdgeInsets.only(bottom: 20.0),
              viewInsets: EdgeInsets.only(bottom: 10.0),
            ),
            child: child,
          ),
        );
        final Offset finalPoint = tester.getCenter(find.byType(Placeholder));
        expect(initialPoint, finalPoint);
      });
    });
  });

  group('SliverSafeArea', () {
    Widget buildWidget(EdgeInsets mediaPadding, Widget sliver) {
      late final ViewportOffset offset;
      addTearDown(() => offset.dispose());

      return MediaQuery(
        data: MediaQueryData(padding: mediaPadding),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Viewport(
            offset: offset = ViewportOffset.fixed(0.0),
            slivers: <Widget>[
              const SliverToBoxAdapter(
                child: SizedBox(width: 800.0, height: 100.0, child: Text('before')),
              ),
              sliver,
              const SliverToBoxAdapter(
                child: SizedBox(width: 800.0, height: 100.0, child: Text('after')),
              ),
            ],
          ),
        ),
      );
    }

    void verify(WidgetTester tester, List<Rect> expectedRects) {
      final List<Rect> testAnswers =
          tester.renderObjectList<RenderBox>(find.byType(SizedBox)).map<Rect>((RenderBox target) {
            final Offset topLeft = target.localToGlobal(Offset.zero);
            final Offset bottomRight = target.localToGlobal(target.size.bottomRight(Offset.zero));
            return Rect.fromPoints(topLeft, bottomRight);
          }).toList();
      expect(testAnswers, equals(expectedRects));
    }

    testWidgets('SliverSafeArea - basic', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildWidget(
          const EdgeInsets.all(20.0),
          const SliverSafeArea(
            left: false,
            sliver: SliverToBoxAdapter(
              child: SizedBox(width: 800.0, height: 100.0, child: Text('padded')),
            ),
          ),
        ),
      );
      verify(tester, <Rect>[
        const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0),
        const Rect.fromLTWH(0.0, 120.0, 780.0, 100.0),
        const Rect.fromLTWH(0.0, 240.0, 800.0, 100.0),
      ]);
    });

    testWidgets('SliverSafeArea - basic', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildWidget(
          const EdgeInsets.all(20.0),
          const SliverSafeArea(
            top: false,
            minimum: EdgeInsets.fromLTRB(0.0, 10.0, 20.0, 30.0),
            sliver: SliverToBoxAdapter(
              child: SizedBox(width: 800.0, height: 100.0, child: Text('padded')),
            ),
          ),
        ),
      );
      verify(tester, <Rect>[
        const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0),
        const Rect.fromLTWH(20.0, 110.0, 760.0, 100.0),
        const Rect.fromLTWH(0.0, 240.0, 800.0, 100.0),
      ]);
    });

    testWidgets('SliverSafeArea - nested', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildWidget(
          const EdgeInsets.all(20.0),
          const SliverSafeArea(
            top: false,
            sliver: SliverSafeArea(
              right: false,
              sliver: SliverToBoxAdapter(
                child: SizedBox(width: 800.0, height: 100.0, child: Text('padded')),
              ),
            ),
          ),
        ),
      );
      verify(tester, <Rect>[
        const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0),
        const Rect.fromLTWH(20.0, 120.0, 760.0, 100.0),
        const Rect.fromLTWH(0.0, 240.0, 800.0, 100.0),
      ]);
    });

    testWidgets('SliverSafeArea - changing', (WidgetTester tester) async {
      const Widget sliver = SliverSafeArea(
        bottom: false,
        sliver: SliverSafeArea(
          left: false,
          bottom: false,
          sliver: SliverToBoxAdapter(
            child: SizedBox(width: 800.0, height: 100.0, child: Text('padded')),
          ),
        ),
      );
      await tester.pumpWidget(buildWidget(const EdgeInsets.all(20.0), sliver));
      verify(tester, <Rect>[
        const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0),
        const Rect.fromLTWH(20.0, 120.0, 760.0, 100.0),
        const Rect.fromLTWH(0.0, 220.0, 800.0, 100.0),
      ]);

      await tester.pumpWidget(
        buildWidget(const EdgeInsets.only(left: 100.0, top: 30.0, bottom: 40.0), sliver),
      );
      verify(tester, <Rect>[
        const Rect.fromLTWH(0.0, 0.0, 800.0, 100.0),
        const Rect.fromLTWH(100.0, 130.0, 700.0, 100.0),
        const Rect.fromLTWH(0.0, 230.0, 800.0, 100.0),
      ]);
    });
  });

  testWidgets('SliverSafeArea - properties', (WidgetTester tester) async {
    const SliverSafeArea child = SliverSafeArea(
      right: false,
      bottom: false,
      sliver: SliverToBoxAdapter(
        child: SizedBox(width: 800.0, height: 100.0, child: Text('padded')),
      ),
    );
    final DiagnosticPropertiesBuilder properties = DiagnosticPropertiesBuilder();
    child.debugFillProperties(properties);

    expect(
      properties.properties.any(
        (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid left padding',
      ),
      true,
    );
    expect(
      properties.properties.any(
        (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid right padding',
      ),
      false,
    );
    expect(
      properties.properties.any(
        (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid top padding',
      ),
      true,
    );
    expect(
      properties.properties.any(
        (DiagnosticsNode n) => n is FlagProperty && n.toString() == 'avoid bottom padding',
      ),
      false,
    );
  });
}
