// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpImplicitView();

  group('PlatformViewManager', () {
    const String viewType = 'forTest';
    const int viewId = 6;

    late PlatformViewManager contentManager;

    setUp(() {
      contentManager = PlatformViewManager();
    });

    group('knowsViewType', () {
      test('recognizes viewTypes after registering them', () async {
        expect(contentManager.knowsViewType(viewType), isFalse);

        contentManager.registerFactory(viewType, (int id) => createDomHTMLDivElement());

        expect(contentManager.knowsViewType(viewType), isTrue);
      });
    });

    group('knowsViewId', () {
      test('recognizes viewIds after *rendering* them', () async {
        expect(contentManager.knowsViewId(viewId), isFalse);

        contentManager.registerFactory(viewType, (int id) => createDomHTMLDivElement());

        expect(contentManager.knowsViewId(viewId), isFalse);

        contentManager.renderContent(viewType, viewId, null);

        expect(contentManager.knowsViewId(viewId), isTrue);
      });

      test('forgets viewIds after clearing them', () {
        contentManager.registerFactory(viewType, (int id) => createDomHTMLDivElement());
        final DomElement view = contentManager.renderContent(viewType, viewId, null);

        expect(contentManager.knowsViewId(viewId), isTrue);

        contentManager.clearPlatformView(viewId);

        expect(contentManager.knowsViewId(viewId), isFalse);
        expect(view.parentNode, isNull);
      });
    });

    group('registerFactory', () {
      test('does NOT re-register factories', () async {
        contentManager.registerFactory(
          viewType,
          (int id) => createDomHTMLDivElement()..id = 'pass',
        );
        // this should be rejected
        contentManager.registerFactory(
          viewType,
          (int id) => createDomHTMLSpanElement()..id = 'fail',
        );

        final DomElement contents = contentManager.renderContent(viewType, viewId, null);

        expect(contents.querySelector('#pass'), isNotNull);
        expect(
          contents.querySelector('#fail'),
          isNull,
          reason: 'Factories cannot be overridden once registered',
        );
      });
    });

    group('renderContent', () {
      const String unregisteredViewType = 'unregisteredForTest';
      const String anotherViewType = 'anotherViewType';

      setUp(() {
        contentManager.registerFactory(viewType, (int id) {
          return createDomHTMLDivElement()..setAttribute('data-viewId', '$id');
        });

        contentManager.registerFactory(anotherViewType, (int id) {
          return createDomHTMLDivElement()
            ..setAttribute('data-viewId', '$id')
            ..style.height = 'auto'
            ..style.width = '55%';
        });
      });

      test('refuse to render views for unregistered factories', () async {
        expect(
          () => contentManager.renderContent(unregisteredViewType, viewId, null),
          throwsA(
            const TypeMatcher<AssertionError>().having(
              (AssertionError error) => error.message,
              'assertion message',
              contains(unregisteredViewType),
            ),
          ),
        );
      });

      test('rendered markup contains required attributes', () async {
        final DomElement content = contentManager.renderContent(viewType, viewId, null);
        expect(content.getAttribute('slot'), getPlatformViewSlotName(viewId));
        expect(content.getAttribute('id'), getPlatformViewDomId(viewId));

        final DomElement userContent = content.querySelector('div')!;
        expect(userContent.style.height, '100%');
        expect(userContent.style.width, '100%');
      });

      test('slot property has the same value as createPlatformViewSlot', () async {
        final DomElement content = contentManager.renderContent(viewType, viewId, null);
        final DomElement slot = createPlatformViewSlot(viewId);
        final DomElement innerSlot = slot.querySelector('slot')!;

        expect(
          content.getAttribute('slot'),
          innerSlot.getAttribute('name'),
          reason:
              'The slot attribute of the rendered content must match the name attribute of the SLOT of a given viewId',
        );
      });

      test(
        'do not modify style.height / style.width if passed by the user (anotherViewType)',
        () async {
          final DomElement content = contentManager.renderContent(anotherViewType, viewId, null);
          final DomElement userContent = content.querySelector('div')!;
          expect(userContent.style.height, 'auto');
          expect(userContent.style.width, '55%');
        },
      );

      test('returns cached instances of already-rendered content', () async {
        final DomElement firstRender = contentManager.renderContent(viewType, viewId, null);
        final DomElement anotherRender = contentManager.renderContent(viewType, viewId, null);

        expect(firstRender, same(anotherRender));
      });
    });

    group('getViewById', () {
      test('finds created views', () async {
        final Map<int, DomElement> views1 = <int, DomElement>{
          1: createDomHTMLDivElement(),
          2: createDomHTMLDivElement(),
          5: createDomHTMLDivElement(),
        };
        final Map<int, DomElement> views2 = <int, DomElement>{
          3: createDomHTMLDivElement(),
          4: createDomHTMLDivElement(),
        };

        contentManager.registerFactory('forTest1', (int id) => views1[id]!);
        contentManager.registerFactory('forTest2', (int id) => views2[id]!);

        // Render all 5 views.
        for (final int id in views1.keys) {
          contentManager.renderContent('forTest1', id, null);
        }
        for (final int id in views2.keys) {
          contentManager.renderContent('forTest2', id, null);
        }

        // Check all 5 views.
        for (final int id in views1.keys) {
          expect(contentManager.getViewById(id), views1[id]);
        }
        for (final int id in views2.keys) {
          expect(contentManager.getViewById(id), views2[id]);
        }

        // Throws for unknown viewId.
        expect(() {
          contentManager.getViewById(99);
        }, throwsA(isA<AssertionError>()));
      });

      test('throws if view has been cleared', () {
        final DomHTMLDivElement view = createDomHTMLDivElement();
        contentManager.registerFactory(viewType, (int id) => view);

        // Throws before viewId is rendered.
        expect(() {
          contentManager.getViewById(viewId);
        }, throwsA(isA<AssertionError>()));

        contentManager.renderContent(viewType, viewId, null);
        // Succeeds after viewId is rendered.
        expect(contentManager.getViewById(viewId), view);

        contentManager.clearPlatformView(viewId);
        // Throws after viewId is cleared.
        expect(() {
          contentManager.getViewById(viewId);
        }, throwsA(isA<AssertionError>()));
      });
    });

    test('default factories', () {
      final DomElement content0 = contentManager.renderContent(
        ui_web.PlatformViewRegistry.defaultVisibleViewType,
        viewId,
        <dynamic, dynamic>{'tagName': 'table'},
      );
      expect(contentManager.getViewById(viewId), content0.querySelector('table'));
      expect(contentManager.isVisible(viewId), isTrue);
      expect(contentManager.isInvisible(viewId), isFalse);

      final DomElement content1 = contentManager.renderContent(
        ui_web.PlatformViewRegistry.defaultInvisibleViewType,
        viewId + 1,
        <dynamic, dynamic>{'tagName': 'script'},
      );
      expect(contentManager.getViewById(viewId + 1), content1.querySelector('script'));
      expect(contentManager.isVisible(viewId + 1), isFalse);
      expect(contentManager.isInvisible(viewId + 1), isTrue);

      final DomElement content2 = contentManager.renderContent(
        ui_web.PlatformViewRegistry.defaultVisibleViewType,
        viewId + 2,
        <dynamic, dynamic>{'tagName': 'p'},
      );
      expect(contentManager.getViewById(viewId + 2), content2.querySelector('p'));
      expect(contentManager.isVisible(viewId + 2), isTrue);
      expect(contentManager.isInvisible(viewId + 2), isFalse);
    });
  });
}
