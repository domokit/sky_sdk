// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome')
library;

import 'dart:async';
import 'dart:ui_web' as ui_web;

import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/widgets/_html_element_view_web.dart'
    show debugOverridePlatformViewRegistry;
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:web/web.dart' as web;

import 'web_platform_view_registry_utils.dart';

final Object _mockHtmlElement = Object();
Object _mockViewFactory(int id, {Object? params}) {
  return _mockHtmlElement;
}

void main() {
  late FakePlatformViewRegistry fakePlatformViewRegistry;

  setUp(() {
    fakePlatformViewRegistry = FakePlatformViewRegistry();

    // Simulate the engine registering default factories.
    fakePlatformViewRegistry.registerViewFactory(
      ui_web.PlatformViewRegistry.defaultVisibleViewType,
      (int viewId, {Object? params}) {
        params!;
        params as Map<Object?, Object?>;
        return web.document.createElement(params['tagName']! as String);
      },
    );
    fakePlatformViewRegistry.registerViewFactory(
      ui_web.PlatformViewRegistry.defaultInvisibleViewType,
      (int viewId, {Object? params}) {
        params!;
        params as Map<Object?, Object?>;
        return web.document.createElement(params['tagName']! as String);
      },
    );
  });

  group('HtmlElementView', () {
    testWidgets('Create HTML view', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);

      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 200.0, height: 100.0, child: HtmlElementView(viewType: 'webview')),
        ),
      );

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (id: currentViewId + 1, viewType: 'webview', params: null, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('Create HTML view with PlatformViewCreatedCallback', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);

      bool hasPlatformViewCreated = false;
      void onPlatformViewCreatedCallBack(int id) {
        hasPlatformViewCreated = true;
      }

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView(
              viewType: 'webview',
              onPlatformViewCreated: onPlatformViewCreatedCallBack,
            ),
          ),
        ),
      );

      // Check the onPlatformViewCreatedCallBack has been called.
      expect(hasPlatformViewCreated, true);

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (id: currentViewId + 1, viewType: 'webview', params: null, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('Create HTML view with creation params', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);
      await tester.pumpWidget(
        const Column(
          children: <Widget>[
            SizedBox(
              width: 200.0,
              height: 100.0,
              child: HtmlElementView(viewType: 'webview', creationParams: 'foobar'),
            ),
            SizedBox(
              width: 200.0,
              height: 100.0,
              child: HtmlElementView(viewType: 'webview', creationParams: 123),
            ),
          ],
        ),
      );

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (
            id: currentViewId + 1,
            viewType: 'webview',
            params: 'foobar',
            htmlElement: _mockHtmlElement,
          ),
          (id: currentViewId + 2, viewType: 'webview', params: 123, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('Resize HTML view', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);
      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 200.0, height: 100.0, child: HtmlElementView(viewType: 'webview')),
        ),
      );

      final Completer<void> resizeCompleter = Completer<void>();

      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 100.0, height: 50.0, child: HtmlElementView(viewType: 'webview')),
        ),
      );

      resizeCompleter.complete();
      await tester.pump();

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (id: currentViewId + 1, viewType: 'webview', params: null, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('Change HTML view type', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);
      fakePlatformViewRegistry.registerViewFactory('maps', _mockViewFactory);
      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 200.0, height: 100.0, child: HtmlElementView(viewType: 'webview')),
        ),
      );

      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 200.0, height: 100.0, child: HtmlElementView(viewType: 'maps')),
        ),
      );

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (id: currentViewId + 2, viewType: 'maps', params: null, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('Dispose HTML view', (WidgetTester tester) async {
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);
      await tester.pumpWidget(
        const Center(
          child: SizedBox(width: 200.0, height: 100.0, child: HtmlElementView(viewType: 'webview')),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      expect(fakePlatformViewRegistry.views, isEmpty);
    });

    testWidgets('HTML view survives widget tree change', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView(viewType: 'webview', key: key),
          ),
        ),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView(viewType: 'webview', key: key),
          ),
        ),
      );

      expect(
        fakePlatformViewRegistry.views,
        unorderedEquals(<FakePlatformView>[
          (id: currentViewId + 1, viewType: 'webview', params: null, htmlElement: _mockHtmlElement),
        ]),
      );
    });

    testWidgets('HtmlElementView has correct semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      expect(currentViewId, greaterThanOrEqualTo(0));
      fakePlatformViewRegistry.registerViewFactory('webview', _mockViewFactory);

      await tester.pumpWidget(
        Semantics(
          container: true,
          child: const Align(
            alignment: Alignment.bottomRight,
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: HtmlElementView(viewType: 'webview'),
            ),
          ),
        ),
      );
      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      // The platform view ID is set on the child of the HtmlElementView render object.
      final SemanticsNode semantics = tester.getSemantics(find.byType(PlatformViewSurface));

      expect(semantics.platformViewId, currentViewId + 1);
      expect(semantics.rect, const Rect.fromLTWH(0, 0, 200, 100));
      // A 200x100 rect positioned at bottom right of a 800x600 box.
      expect(semantics.transform, Matrix4.translationValues(600, 500, 0));
      expect(semantics.childrenCount, 0);

      handle.dispose();
    });
  });

  group('HtmlElementView.fromTagName', () {
    setUp(() {
      debugOverridePlatformViewRegistry = fakePlatformViewRegistry;
    });

    tearDown(() {
      debugOverridePlatformViewRegistry = null;
    });

    testWidgets('Create platform view from tagName', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView.fromTagName(tagName: 'div'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(fakePlatformViewRegistry.views, hasLength(1));
      final FakePlatformView fakePlatformView = fakePlatformViewRegistry.views.single;
      expect(fakePlatformView.id, currentViewId + 1);
      expect(fakePlatformView.viewType, ui_web.PlatformViewRegistry.defaultVisibleViewType);
      expect(fakePlatformView.params, <dynamic, dynamic>{'tagName': 'div'});

      // The HTML element should be a div.
      final web.HTMLElement htmlElement = fakePlatformView.htmlElement as web.HTMLElement;
      expect(htmlElement.tagName, equalsIgnoringCase('div'));
    });

    testWidgets('Create invisible platform view', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView.fromTagName(tagName: 'script', isVisible: false),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(fakePlatformViewRegistry.views, hasLength(1));
      final FakePlatformView fakePlatformView = fakePlatformViewRegistry.views.single;
      expect(fakePlatformView.id, currentViewId + 1);
      // The view should be invisible.
      expect(fakePlatformView.viewType, ui_web.PlatformViewRegistry.defaultInvisibleViewType);
      expect(fakePlatformView.params, <dynamic, dynamic>{'tagName': 'script'});

      // The HTML element should be a script.
      final web.HTMLElement htmlElement = fakePlatformView.htmlElement as web.HTMLElement;
      expect(htmlElement.tagName, equalsIgnoringCase('script'));
    });

    testWidgets('onElementCreated', (WidgetTester tester) async {
      final List<Object> createdElements = <Object>[];
      void onElementCreated(Object element) {
        createdElements.add(element);
      }

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: HtmlElementView.fromTagName(
              tagName: 'table',
              onElementCreated: onElementCreated,
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(fakePlatformViewRegistry.views, hasLength(1));
      final FakePlatformView fakePlatformView = fakePlatformViewRegistry.views.single;

      expect(createdElements, hasLength(1));
      final Object createdElement = createdElements.single;

      expect(createdElement, fakePlatformView.htmlElement);
    });

    group('hitTestBehavior', () {
      testWidgets('opaque by default', (WidgetTester tester) async {
        final Key containerKey = UniqueKey();
        int taps = 0;

        await tester.pumpWidget(
          GestureDetector(
            onTap: () => taps++,
            child: Container(
              key: containerKey,
              width: 200,
              height: 200,
              // Add a color to make it a visible container. This ensures that
              // GestureDetector's default hit test behavior works.
              color: const Color(0xFF00FF00),
              child: HtmlElementView.fromTagName(tagName: 'div'),
            ),
          ),
        );
        await tester.pumpAndSettle();

        expect(taps, isZero);

        await tester.tap(find.byKey(containerKey), warnIfMissed: false);

        // Taps are still zero on the container because the HtmlElementView is
        // opaque and prevents widgets behind it from receiving pointer events.
        expect(taps, isZero);
      });

      testWidgets('can be set to transparent', (WidgetTester tester) async {
        final Key containerKey = UniqueKey();
        int taps = 0;

        await tester.pumpWidget(
          GestureDetector(
            onTap: () => taps++,
            child: Container(
              key: containerKey,
              width: 200,
              height: 200,
              // Add a color to make it a visible container. This ensures that
              // GestureDetector's default hit test behavior works.
              color: const Color(0xFF00FF00),
              child: HtmlElementView.fromTagName(
                tagName: 'div',
                hitTestBehavior: PlatformViewHitTestBehavior.transparent,
              ),
            ),
          ),
        );
        await tester.pumpAndSettle();

        expect(taps, isZero);

        await tester.tap(find.byKey(containerKey), warnIfMissed: false);

        // The container can receive taps because the HtmlElementView is
        // transparent from a hit testing perspective.
        expect(taps, 1);
      });
    });
  });
}
