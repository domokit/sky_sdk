// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'common.dart';
import 'test_data.dart';

EngineFlutterWindow get implicitView => EnginePlatformDispatcher.instance.implicitView!;

DomElement get platformViewsHost => implicitView.dom.platformViewsHost;
DomElement get sceneHost => implicitView.dom.sceneHost;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$HtmlViewEmbedder', () {
    setUpCanvasKitTest(withImplicitView: true);

    setUp(() {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1);
    });

    tearDown(() {
      PlatformViewManager.instance.debugClear();
      CanvasKitRenderer.instance.debugClear();
      Instrumentation.enabled = false;
    });

    test('embeds interactive platform views', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      // The platform view is now split in two parts. The contents live
      // as a child of the glassPane, and the slot lives in the glassPane
      // shadow root. The slot is the one that has pointer events auto.
      final DomElement contents = platformViewsHost.querySelector('#view-0')!;
      final DomElement slot = sceneHost.querySelector('slot')!;
      final DomElement contentsHost = contents.parent!;
      final DomElement slotHost = slot.parent!;

      expect(contents, isNotNull, reason: 'The view from the factory is injected in the DOM.');

      expect(contentsHost.tagName, equalsIgnoringCase('flt-platform-view'));
      expect(slotHost.tagName, equalsIgnoringCase('flt-platform-view-slot'));

      expect(slotHost.style.pointerEvents, 'auto', reason: 'The slot reenables pointer events.');
      expect(
        contentsHost.getAttribute('slot'),
        slot.getAttribute('name'),
        reason: 'The contents and slot are correctly related.',
      );
    });

    test('clips platform views with RRects', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.pushClipRRect(ui.RRect.fromLTRBR(0, 0, 10, 10, const ui.Radius.circular(3)));
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      expect(sceneHost.querySelectorAll('#sk_path_defs').single, isNotNull);
      expect(
        sceneHost.querySelectorAll('#sk_path_defs').single.querySelectorAll('clipPath').single,
        isNotNull,
      );
      expect(sceneHost.querySelectorAll('flt-clip').single.style.clipPath, 'url("#svgClip1")');
      expect(sceneHost.querySelectorAll('flt-clip').single.style.width, '100%');
      expect(sceneHost.querySelectorAll('flt-clip').single.style.height, '100%');
    });

    test('correctly transforms platform views', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      final Matrix4 scaleMatrix =
          Matrix4.identity()
            ..scale(5, 5)
            ..translate(100, 100);
      sb.pushTransform(scaleMatrix.toFloat64());
      sb.pushOffset(3, 3);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      // Transformations happen on the slot element.
      final DomElement slotHost = sceneHost.querySelector('flt-platform-view-slot')!;

      expect(
        slotHost.style.transform,
        // We should apply the scale matrix first, then the offset matrix.
        // So the translate should be 515 (5 * 100 + 5 * 3), and not
        // 503 (5 * 100 + 3).
        'matrix3d(5, 0, 0, 0, 0, 5, 0, 0, 0, 0, 5, 0, 515, 515, 0, 1)',
      );
    });

    test('correctly offsets platform views', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.addPlatformView(0, offset: const ui.Offset(3, 4), width: 5, height: 6);
      await renderScene(sb.build());

      final DomElement slotHost = sceneHost.querySelector('flt-platform-view-slot')!;
      final DomCSSStyleDeclaration style = slotHost.style;

      expect(style.transform, 'matrix(1, 0, 0, 1, 3, 4)');
      expect(style.width, '5px');
      expect(style.height, '6px');

      final DomRect slotRect = slotHost.getBoundingClientRect();
      expect(slotRect.left, 3);
      expect(slotRect.top, 4);
      expect(slotRect.right, 8);
      expect(slotRect.bottom, 10);
    });

    // Returns the list of CSS transforms applied to the ancestor chain of
    // elements starting from `viewHost`, up until and excluding
    // <flt-scene-host>.
    List<String> getTransformChain(DomElement viewHost) {
      final List<String> chain = <String>[];
      DomElement? element = viewHost;
      while (element != null && element.tagName.toLowerCase() != 'flt-scene-host') {
        chain.add(element.style.transform);
        element = element.parent;
      }
      return chain;
    }

    test('correctly offsets when clip chain length is changed', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(3, 3);
      sb.pushClipRect(ui.Rect.largest);
      sb.pushOffset(6, 6);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());

      // Transformations happen on the slot element.
      DomElement slotHost = sceneHost.querySelector('flt-platform-view-slot')!;

      expect(getTransformChain(slotHost), <String>[
        'matrix(1, 0, 0, 1, 6, 6)',
        'matrix(1, 0, 0, 1, 3, 3)',
      ]);

      sb = LayerSceneBuilder();
      sb.pushOffset(3, 3);
      sb.pushClipRect(ui.Rect.largest);
      sb.pushOffset(6, 6);
      sb.pushClipRect(ui.Rect.largest);
      sb.pushOffset(9, 9);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      // Transformations happen on the slot element.
      slotHost = sceneHost.querySelector('flt-platform-view-slot')!;

      expect(getTransformChain(slotHost), <String>[
        'matrix(1, 0, 0, 1, 9, 9)',
        'matrix(1, 0, 0, 1, 6, 6)',
        'matrix(1, 0, 0, 1, 3, 3)',
      ]);
    });

    test('converts device pixels to logical pixels (no clips)', () async {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(4);
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(1, 1);
      sb.pushOffset(2, 2);
      sb.pushOffset(3, 3);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      // Transformations happen on the slot element.
      final DomElement slotHost = sceneHost.querySelector('flt-platform-view-slot')!;

      expect(getTransformChain(slotHost), <String>['matrix(0.25, 0, 0, 0.25, 1.5, 1.5)']);
    });

    test('converts device pixels to logical pixels (with clips)', () async {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(4);
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(3, 3);
      sb.pushClipRect(ui.Rect.largest);
      sb.pushOffset(6, 6);
      sb.pushClipRect(ui.Rect.largest);
      sb.pushOffset(9, 9);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());

      // Transformations happen on the slot element.
      final DomElement slotHost = sceneHost.querySelector('flt-platform-view-slot')!;

      expect(getTransformChain(slotHost), <String>[
        'matrix(1, 0, 0, 1, 9, 9)',
        'matrix(1, 0, 0, 1, 6, 6)',
        'matrix(0.25, 0, 0, 0.25, 0.75, 0.75)',
      ]);
    });

    test('renders overlays on top of platform views', () async {
      final CkPicture testPicture = paintPicture(const ui.Rect.fromLTRB(0, 0, 10, 10), (
        CkCanvas canvas,
      ) {
        canvas.drawCircle(const ui.Offset(5, 5), 5, CkPaint());
      });

      // Initialize all platform views to be used in the test.
      final List<int> platformViewIds = <int>[];
      for (int i = 0; i < 16; i++) {
        ui_web.platformViewRegistry.registerViewFactory(
          'test-platform-view',
          (int viewId) => createDomHTMLDivElement()..id = 'view-$i',
        );
        await createPlatformView(i, 'test-platform-view');
        platformViewIds.add(i);
      }

      Future<void> renderTestScene({required int viewCount}) async {
        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        for (int i = 0; i < viewCount; i++) {
          sb.addPicture(ui.Offset.zero, testPicture);
          sb.addPlatformView(i, width: 10, height: 10);
        }
        await renderScene(sb.build());
      }

      // Frame 1:
      //   Render: up to cache size platform views.
      //   Expect: main canvas plus platform view overlays.
      await renderTestScene(viewCount: 8);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 2:
      //   Render: zero platform views.
      //   Expect: main canvas, no overlays.
      await renderTestScene(viewCount: 0);
      _expectSceneMatches(<_EmbeddedViewMarker>[]);

      // Frame 3:
      //   Render: less than cache size platform views.
      //   Expect: overlays reused.
      await renderTestScene(viewCount: 6);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 4:
      //   Render: more platform views than max overlay count.
      //   Expect: main canvas, backup overlay, maximum overlays.
      await renderTestScene(viewCount: 16);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 5:
      //   Render: zero platform views.
      //   Expect: main canvas, no overlays.
      await renderTestScene(viewCount: 0);
      _expectSceneMatches(<_EmbeddedViewMarker>[]);

      // Frame 6:
      //   Render: deleted platform views.
      //   Expect: error.
      for (final int id in platformViewIds) {
        const StandardMethodCodec codec = StandardMethodCodec();
        final Completer<void> completer = Completer<void>();
        ui.PlatformDispatcher.instance.sendPlatformMessage(
          'flutter/platform_views',
          codec.encodeMethodCall(MethodCall('dispose', id)),
          completer.complete,
        );
        await completer.future;
      }

      try {
        await renderTestScene(viewCount: platformViewIds.length);
        fail('Expected to throw');
      } on AssertionError catch (error) {
        expect(
          error.toString(),
          contains(
            'Cannot render platform views: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15. These views have not been created, or they have been deleted.',
          ),
        );
      }

      // Frame 7:
      //   Render: a platform view after error.
      //   Expect: success. Just checking the system is not left in a corrupted state.
      await createPlatformView(0, 'test-platform-view');
      await renderTestScene(viewCount: 0);
      _expectSceneMatches(<_EmbeddedViewMarker>[]);
    });

    test('correctly reuses overlays', () async {
      final CkPicture testPicture = paintPicture(const ui.Rect.fromLTRB(0, 0, 10, 10), (
        CkCanvas canvas,
      ) {
        canvas.drawCircle(const ui.Offset(5, 5), 5, CkPaint());
      });

      // Initialize all platform views to be used in the test.
      final List<int> platformViewIds = <int>[];
      for (int i = 0; i < 20; i++) {
        ui_web.platformViewRegistry.registerViewFactory(
          'test-platform-view',
          (int viewId) => createDomHTMLDivElement()..id = 'view-$i',
        );
        await createPlatformView(i, 'test-platform-view');
        platformViewIds.add(i);
      }

      Future<void> renderTestScene(List<int> views) async {
        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        for (final int view in views) {
          sb.addPicture(ui.Offset.zero, testPicture);
          sb.addPlatformView(view, width: 10, height: 10);
        }
        await renderScene(sb.build());
      }

      // Frame 1:
      //   Render: Views 1-10
      //   Expect: main canvas plus platform view overlays; empty cache.
      await renderTestScene(<int>[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 2:
      //   Render: Views 2-11
      //   Expect: main canvas plus platform view overlays; empty cache.
      await renderTestScene(<int>[2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 3:
      //   Render: Views 3-12
      //   Expect: main canvas plus platform view overlays; empty cache.
      await renderTestScene(<int>[3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Frame 4:
      //   Render: Views 3-12 again (same as last frame)
      //   Expect: main canvas plus platform view overlays; empty cache.
      await renderTestScene(<int>[3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);
    });

    test('embeds and disposes of a platform view', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[_platformView]);

      expect(platformViewsHost.querySelector('flt-platform-view'), isNotNull);

      await disposePlatformView(0);

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      await renderScene(sb.build());

      _expectSceneMatches(<_EmbeddedViewMarker>[]);

      expect(platformViewsHost.querySelector('flt-platform-view'), isNull);
    });

    test(
      'does not crash when resizing the window after textures have been registered',
      () async {
        ui_web.platformViewRegistry.registerViewFactory(
          'test-platform-view',
          (int viewId) => createDomHTMLDivElement()..id = 'view-0',
        );
        await createPlatformView(0, 'test-platform-view');

        final CkBrowserImageDecoder image = await CkBrowserImageDecoder.create(
          contentType: 'image/gif',
          data: kAnimatedGif,
          debugSource: 'test',
        );
        final ui.FrameInfo frame = await image.getNextFrame();
        final CkImage ckImage = frame.image as CkImage;

        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        final CkPictureRecorder recorder = CkPictureRecorder();
        final CkCanvas canvas = recorder.beginRecording(ui.Rect.largest);
        canvas.drawImage(ckImage, ui.Offset.zero, CkPaint());
        final CkPicture picture = recorder.endRecording();
        sb.addPicture(ui.Offset.zero, picture);
        sb.addPlatformView(0, width: 10, height: 10);

        implicitView.debugPhysicalSizeOverride = const ui.Size(100, 100);
        implicitView.debugForceResize();
        await renderScene(sb.build());
        _expectSceneMatches(<_EmbeddedViewMarker>[_overlay, _platformView]);

        implicitView.debugPhysicalSizeOverride = const ui.Size(200, 200);
        implicitView.debugForceResize();
        await renderScene(sb.build());
        _expectSceneMatches(<_EmbeddedViewMarker>[_overlay, _platformView]);

        implicitView.debugPhysicalSizeOverride = null;
        implicitView.debugForceResize();

        // ImageDecoder is not supported in Safari or Firefox.
      },
      skip: isSafari || isFirefox,
    );

    test('removes the DOM node of an unrendered platform view', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPlatformView(0, width: 10, height: 10);
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[_platformView]);

      expect(platformViewsHost.querySelector('flt-platform-view'), isNotNull);

      // Render a frame with a different platform view.
      await createPlatformView(1, 'test-platform-view');
      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPlatformView(1, width: 10, height: 10);
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[_platformView]);

      expect(platformViewsHost.querySelectorAll('flt-platform-view'), hasLength(2));

      // Render a frame without a platform view, but also without disposing of
      // the platform view.
      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[]);

      // The actual contents of the platform view are kept in the dom, until
      // it's actually disposed of!
      expect(platformViewsHost.querySelectorAll('flt-platform-view'), hasLength(2));
    });

    test('removes old SVG clip definitions from the DOM when the view is recomposited', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'test-view',
      );
      await createPlatformView(0, 'test-platform-view');

      Future<void> renderTestScene() async {
        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        sb.pushClipRRect(ui.RRect.fromLTRBR(0, 0, 10, 10, const ui.Radius.circular(3)));
        sb.addPlatformView(0, width: 10, height: 10);
        await renderScene(sb.build());
      }

      final DomNode skPathDefs = sceneHost.querySelector('#sk_path_defs')!;

      expect(skPathDefs.childNodes, hasLength(0));

      await renderTestScene();
      expect(skPathDefs.childNodes, hasLength(1));

      await renderTestScene();
      expect(skPathDefs.childNodes, hasLength(1));

      await renderTestScene();
      expect(skPathDefs.childNodes, hasLength(1));
    });

    test('does not crash when a prerolled platform view is not composited', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-platform-view',
        (int viewId) => createDomHTMLDivElement()..id = 'view-0',
      );
      await createPlatformView(0, 'test-platform-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.pushClipRect(ui.Rect.zero);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.pop();
      // The below line should not throw an error.
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[]);
    });

    test('does not create overlays for invisible platform views', () async {
      final CkPicture testPicture = paintPicture(const ui.Rect.fromLTRB(0, 0, 10, 10), (
        CkCanvas canvas,
      ) {
        canvas.drawCircle(const ui.Offset(5, 5), 5, CkPaint());
      });
      ui_web.platformViewRegistry.registerViewFactory(
        'test-visible-view',
        (int viewId) => createDomHTMLDivElement()..className = 'visible-platform-view',
      );
      ui_web.platformViewRegistry.registerViewFactory(
        'test-invisible-view',
        (int viewId) => createDomHTMLDivElement()..className = 'invisible-platform-view',
        isVisible: false,
      );
      await createPlatformView(0, 'test-visible-view');
      await createPlatformView(1, 'test-invisible-view');
      await createPlatformView(2, 'test-visible-view');
      await createPlatformView(3, 'test-invisible-view');
      await createPlatformView(4, 'test-invisible-view');
      await createPlatformView(5, 'test-invisible-view');
      await createPlatformView(6, 'test-invisible-view');

      expect(PlatformViewManager.instance.isInvisible(0), isFalse);
      expect(PlatformViewManager.instance.isInvisible(1), isTrue);

      LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _overlay,
      ], reason: 'Invisible view renders, followed by an overlay.');

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
      ], reason: 'Overlay created after a group containing a visible view.');

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ], reason: 'Overlays created after each group containing a visible view.');

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _overlay,
      ], reason: 'Invisible views grouped in with visible views.');

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(5, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(5, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(6, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(5, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(6, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ], reason: 'Many invisible views can be rendered on top of the base overlay.');

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(4, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(3, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(2, width: 10, height: 10);
      sb.addPicture(ui.Offset.zero, testPicture);
      sb.addPlatformView(1, width: 10, height: 10);
      sb.pop();
      await renderScene(sb.build());
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _overlay,
      ]);
    });

    test('can dispose without crashing', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
        isVisible: false,
      );

      await createPlatformView(0, 'test-view');
      await createPlatformView(1, 'test-view');
      await createPlatformView(2, 'test-view');

      final LayerSceneBuilder sb =
          LayerSceneBuilder()
            ..pushOffset(0, 0)
            ..addPlatformView(0, width: 10, height: 10)
            ..addPlatformView(1, width: 10, height: 10)
            ..addPlatformView(2, width: 10, height: 10)
            ..pop();

      await renderScene(sb.build());

      _expectSceneMatches(<_EmbeddedViewMarker>[_platformView, _platformView, _platformView]);

      expect(() {
        final HtmlViewEmbedder embedder =
            (renderer as CanvasKitRenderer).debugGetRasterizerForView(implicitView)!.viewEmbedder;
        // The following line used to cause a "Concurrent modification during iteration"
        embedder.dispose();
      }, returnsNormally);
    });

    test('optimizes overlays when pictures and platform views do not overlap', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
      );

      CkPicture rectPicture(ui.Rect rect) {
        return paintPicture(rect, (CkCanvas canvas) {
          canvas.drawRect(rect, CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0));
        });
      }

      await createPlatformView(0, 'test-view');
      await createPlatformView(1, 'test-view');
      await createPlatformView(2, 'test-view');

      expect(PlatformViewManager.instance.isVisible(0), isTrue);
      expect(PlatformViewManager.instance.isVisible(1), isTrue);
      expect(PlatformViewManager.instance.isVisible(2), isTrue);

      // Scene 1: Pictures just overlap with the most recently painted platform
      // view. Analogous to third-party images with subtitles overlaid. Should
      // only need one overlay at the end of the scene.
      final LayerSceneBuilder sb1 = LayerSceneBuilder();
      sb1.pushOffset(0, 0);
      sb1.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb1.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(12, 12, 10, 10)));
      sb1.addPlatformView(1, offset: const ui.Offset(70, 10), width: 50, height: 50);
      sb1.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(72, 12, 10, 10)));
      sb1.addPlatformView(2, offset: const ui.Offset(130, 10), width: 50, height: 50);
      sb1.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(132, 12, 10, 10)));
      final LayerScene scene1 = sb1.build();
      await renderScene(scene1);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      // Scene 2: Same as scene 1 but with a background painted first. Should only
      // need a canvas for the background and one more for the rest of the
      // pictures.
      final LayerSceneBuilder sb2 = LayerSceneBuilder();
      sb2.pushOffset(0, 0);
      sb2.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb2.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb2.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(12, 12, 10, 10)));
      sb2.addPlatformView(1, offset: const ui.Offset(70, 10), width: 50, height: 50);
      sb2.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(72, 12, 10, 10)));
      sb2.addPlatformView(2, offset: const ui.Offset(130, 10), width: 50, height: 50);
      sb2.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(132, 12, 10, 10)));
      final LayerScene scene2 = sb2.build();
      await renderScene(scene2);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);

      // Scene 3: Paints a full-screen picture between each platform view. This
      // is the worst case scenario. There should be an overlay between each
      // platform view.
      final LayerSceneBuilder sb3 = LayerSceneBuilder();
      sb3.pushOffset(0, 0);
      sb3.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb3.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb3.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb3.addPlatformView(1, offset: const ui.Offset(70, 10), width: 50, height: 50);
      sb3.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb3.addPlatformView(2, offset: const ui.Offset(130, 10), width: 50, height: 50);
      sb3.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      final LayerScene scene3 = sb3.build();
      await renderScene(scene3);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
      ]);

      // Scene 4: Same as scene 1 but with a placeholder rectangle painted
      // under each platform view. This is closer to how the real Flutter
      // framework would render a grid of platform views. Interestingly, in this
      // case every drawing can go in a base canvas.
      final LayerSceneBuilder sb4 = LayerSceneBuilder();
      sb4.pushOffset(0, 0);
      sb4.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb4.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(10, 10, 50, 50)));
      sb4.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb4.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(70, 10, 50, 50)));
      sb4.addPlatformView(1, offset: const ui.Offset(70, 10), width: 50, height: 50);
      sb4.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(130, 10, 50, 50)));
      sb4.addPlatformView(2, offset: const ui.Offset(130, 10), width: 50, height: 50);
      final LayerScene scene4 = sb4.build();
      await renderScene(scene4);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _platformView,
      ]);

      // Scene 5: A combination of scene 1 and scene 4, where a subtitle is
      // painted over each platform view and a placeholder is painted under each
      // one.
      final LayerSceneBuilder sb5 = LayerSceneBuilder();
      sb5.pushOffset(0, 0);
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 300, 300)));
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(10, 10, 50, 50)));
      sb5.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(12, 12, 10, 10)));
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(70, 10, 50, 50)));
      sb5.addPlatformView(1, offset: const ui.Offset(70, 10), width: 50, height: 50);
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(72, 12, 10, 10)));
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(130, 10, 50, 50)));
      sb5.addPlatformView(2, offset: const ui.Offset(130, 10), width: 50, height: 50);
      sb5.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(132, 12, 10, 10)));
      final LayerScene scene5 = sb5.build();
      await renderScene(scene5);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
      ]);
    });

    test('correctly places pictures in case where next '
        'picture intersects multiple elements', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
      );
      ui_web.platformViewRegistry.registerViewFactory(
        'invisible-view',
        (int viewId) => createDomHTMLDivElement()..className = 'invisible-platform-view',
        isVisible: false,
      );

      CkPicture rectPicture(ui.Rect rect) {
        return paintPicture(rect, (CkCanvas canvas) {
          canvas.drawRect(rect, CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0));
        });
      }

      await createPlatformView(0, 'test-view');
      await createPlatformView(1, 'invisible-view');

      expect(PlatformViewManager.instance.isVisible(0), isTrue);
      expect(PlatformViewManager.instance.isVisible(1), isFalse);

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      sb.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 100, 100)));
      sb.addPlatformView(0, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 100, 100)));
      sb.addPlatformView(1, offset: const ui.Offset(10, 10), width: 50, height: 50);
      sb.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 5, 5)));
      final LayerScene scene = sb.build();
      await renderScene(scene);
      _expectSceneMatches(<_EmbeddedViewMarker>[_overlay, _platformView, _platformView, _overlay]);

      final Rendering rendering =
          CanvasKitRenderer.instance
              .debugGetRasterizerForView(implicitView)!
              .viewEmbedder
              .debugActiveRendering;
      final List<int> picturesPerCanvas =
          rendering.canvases.map((RenderingRenderCanvas canvas) => canvas.pictures.length).toList();
      expect(picturesPerCanvas, <int>[1, 2]);
    });

    test('sinks platform view under the canvas if it does not overlap with the picture', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
      );

      CkPicture rectPicture(double l, double t, double w, double h) {
        final ui.Rect rect = ui.Rect.fromLTWH(l, t, w, h);
        return paintPicture(rect, (CkCanvas canvas) {
          canvas.drawRect(rect, CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0));
        });
      }

      await createPlatformView(0, 'test-view');
      await createPlatformView(1, 'test-view');

      expect(PlatformViewManager.instance.isVisible(0), isTrue);
      expect(PlatformViewManager.instance.isVisible(1), isTrue);

      final LayerSceneBuilder sb = LayerSceneBuilder();

      // First picture-view-picture stack.
      {
        sb.pushOffset(0, 0);
        sb.addPicture(ui.Offset.zero, rectPicture(0, 0, 10, 10));
        sb.addPlatformView(0, width: 10, height: 10);
        sb.addPicture(ui.Offset.zero, rectPicture(2, 2, 5, 5));
        sb.pop();
      }

      // Second picture-view-picture stack that does not overlap with the first one.
      {
        sb.pushOffset(20, 0);
        sb.addPicture(ui.Offset.zero, rectPicture(0, 0, 10, 10));
        sb.addPlatformView(1, width: 10, height: 10);
        sb.addPicture(ui.Offset.zero, rectPicture(2, 2, 5, 5));
        sb.pop();
      }

      final LayerScene scene1 = sb.build();
      await renderScene(scene1);
      _expectSceneMatches(<_EmbeddedViewMarker>[_overlay, _platformView, _platformView, _overlay]);
    });

    test('optimizes overlays correctly with transforms and clips', () async {
      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
      );

      CkPicture rectPicture(ui.Rect rect) {
        return paintPicture(rect, (CkCanvas canvas) {
          canvas.drawRect(rect, CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0));
        });
      }

      await createPlatformView(0, 'test-view');

      expect(PlatformViewManager.instance.isVisible(0), isTrue);

      // Test optimization correctly computes bounds with transforms and clips.
      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushOffset(0, 0);
      final Matrix4 scaleMatrix = Matrix4.identity()..scale(3, 3, 1);
      sb.pushTransform(scaleMatrix.toFloat64());
      sb.pushClipRect(const ui.Rect.fromLTWH(10, 10, 10, 10));
      sb.addPicture(ui.Offset.zero, rectPicture(const ui.Rect.fromLTWH(0, 0, 20, 20)));
      sb.addPlatformView(0, width: 20, height: 20);
      final LayerScene scene = sb.build();
      await renderScene(scene);
      _expectSceneMatches(<_EmbeddedViewMarker>[_overlay, _platformView]);
    });

    test('can customize amount of overlays', () async {
      final CkPicture testPicture = paintPicture(const ui.Rect.fromLTRB(0, 0, 10, 10), (
        CkCanvas canvas,
      ) {
        canvas.drawCircle(const ui.Offset(5, 5), 5, CkPaint());
      });

      // Initialize all platform views to be used in the test.
      final List<int> platformViewIds = <int>[];
      for (int i = 0; i < 16; i++) {
        ui_web.platformViewRegistry.registerViewFactory(
          'test-platform-view',
          (int viewId) => createDomHTMLDivElement()..id = 'view-$i',
        );
        await createPlatformView(i, 'test-platform-view');
        platformViewIds.add(i);
      }

      Future<void> renderTestScene({required int viewCount}) async {
        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        for (int i = 0; i < viewCount; i++) {
          sb.addPicture(ui.Offset.zero, testPicture);
          sb.addPlatformView(i, width: 10, height: 10);
        }
        await renderScene(sb.build());
      }

      // Set maximum overlays to 4.
      debugOverrideJsConfiguration(
        <String, Object?>{'canvasKitMaximumSurfaces': 4}.jsify() as JsFlutterConfiguration?,
      );

      await renderTestScene(viewCount: 8);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // Set maximum overlays to -1. Should default to 1.
      debugOverrideJsConfiguration(
        <String, Object?>{'canvasKitMaximumSurfaces': -1}.jsify() as JsFlutterConfiguration?,
      );

      await renderTestScene(viewCount: 8);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      debugOverrideJsConfiguration(null);
    });

    test('correctly rearranges pictures to second-to-last canvas '
        'when hitting canvas limit', () async {
      final CkPicture testPicture = paintPicture(const ui.Rect.fromLTRB(0, 0, 10, 10), (
        CkCanvas canvas,
      ) {
        canvas.drawCircle(const ui.Offset(5, 5), 5, CkPaint());
      });

      // Initialize all platform views to be used in the test.
      final List<int> platformViewIds = <int>[];
      for (int i = 0; i < 20; i++) {
        ui_web.platformViewRegistry.registerViewFactory(
          'test-platform-view',
          (int viewId) => createDomHTMLDivElement()..id = 'view-$i',
        );
        await createPlatformView(i, 'test-platform-view');
        platformViewIds.add(i);
      }

      Future<void> renderTestScene(List<int> views) async {
        final LayerSceneBuilder sb = LayerSceneBuilder();
        sb.pushOffset(0, 0);
        for (final int view in views) {
          sb.addPicture(ui.Offset.zero, testPicture);
          sb.addPlatformView(view, width: 10, height: 10);
        }
        await renderScene(sb.build());
      }

      // Render scene with 20 pictures. Check that the second-to-last canvas
      // contains the pictures from the canvases that were deleted.
      await renderTestScene(<int>[
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
      ]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _overlay,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // The second-to-last canvas should have all the extra pictures.
      final Rendering rendering =
          CanvasKitRenderer.instance
              .debugGetRasterizerForView(implicitView)!
              .viewEmbedder
              .debugActiveRendering;
      final List<int> numPicturesPerCanvas =
          rendering.canvases.map((RenderingRenderCanvas canvas) => canvas.pictures.length).toList();
      expect(numPicturesPerCanvas, <int>[1, 1, 1, 1, 1, 1, 12, 1]);

      // It should also work when the maximum canvases is just one.
      debugOverrideJsConfiguration(
        <String, Object?>{'canvasKitMaximumSurfaces': 1}.jsify() as JsFlutterConfiguration?,
      );

      // Render scene with 20 pictures. Check that the last canvas contains the
      // pictures from the canvases that were deleted.
      await renderTestScene(<int>[
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
      ]);
      _expectSceneMatches(<_EmbeddedViewMarker>[
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _platformView,
        _overlay,
        _platformView,
      ]);

      // The last canvas should have all the pictures.
      final Rendering secondRendering =
          CanvasKitRenderer.instance
              .debugGetRasterizerForView(implicitView)!
              .viewEmbedder
              .debugActiveRendering;
      final List<int> picturesPerCanvasInSecondRendering =
          secondRendering.canvases
              .map((RenderingRenderCanvas canvas) => canvas.pictures.length)
              .toList();
      expect(picturesPerCanvasInSecondRendering, <int>[19]);
      debugOverrideJsConfiguration(null);
    });

    test('disposes render pictures', () async {
      Instrumentation.enabled = true;
      Instrumentation.instance.debugCounters.clear();

      ui_web.platformViewRegistry.registerViewFactory(
        'test-view',
        (int viewId) => createDomHTMLDivElement()..className = 'platform-view',
      );

      CkPicture rectPicture(ui.Rect rect) {
        return paintPicture(rect, (CkCanvas canvas) {
          canvas.drawRect(rect, CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0));
        });
      }

      final CkPicture picture = rectPicture(const ui.Rect.fromLTWH(0, 0, 20, 20));

      await createPlatformView(0, 'test-view');

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.addPicture(ui.Offset.zero, picture);
      sb.addPlatformView(0, width: 20, height: 20);
      final LayerScene scene = sb.build();
      await renderScene(scene);

      picture.dispose();
      final Map<String, int> debugCounters = Instrumentation.instance.debugCounters;
      expect(debugCounters['Picture Created'], debugCounters['Picture Deleted']);
    });
  });
}

// Used to test that the platform views and overlays are in the correct order in
// the scene.
enum _EmbeddedViewMarker { overlay, platformView }

_EmbeddedViewMarker get _overlay => _EmbeddedViewMarker.overlay;
_EmbeddedViewMarker get _platformView => _EmbeddedViewMarker.platformView;

const Map<String, _EmbeddedViewMarker> _tagToViewMarker = <String, _EmbeddedViewMarker>{
  'flt-canvas-container': _EmbeddedViewMarker.overlay,
  'flt-platform-view-slot': _EmbeddedViewMarker.platformView,
  'flt-clip': _EmbeddedViewMarker.platformView,
};

void _expectSceneMatches(List<_EmbeddedViewMarker> expectedMarkers, {String? reason}) {
  // Convert the scene elements to its corresponding array of _EmbeddedViewMarker
  final List<_EmbeddedViewMarker> sceneElements =
      sceneHost.children
          .where((DomElement element) => element.tagName != 'svg')
          .map((DomElement element) => _tagToViewMarker[element.tagName.toLowerCase()]!)
          .toList();

  expect(sceneElements, expectedMarkers, reason: reason);
}
