// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../services/fake_platform_views.dart';

void main() {
  group('AndroidView', () {
    testWidgets('Create Android view', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'webview',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Create Android view with params', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(
              viewType: 'webview',
              layoutDirection: TextDirection.ltr,
              creationParams: 'creation parameters',
              creationParamsCodec: StringCodec(),
            ),
          ),
        ),
      );

      final FakeAndroidPlatformView fakeView = viewsController.views.first;
      final Uint8List rawCreationParams = fakeView.creationParams!;
      final ByteData byteData = ByteData.view(
        rawCreationParams.buffer,
        rawCreationParams.offsetInBytes,
        rawCreationParams.lengthInBytes,
      );
      final dynamic actualParams = const StringCodec().decodeMessage(byteData);

      expect(actualParams, 'creation parameters');
      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'webview',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
            creationParams: fakeView.creationParams,
          ),
        ]),
      );
    });

    testWidgets('Zero sized Android view is not created', (WidgetTester tester) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox.shrink(
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(viewsController.views, isEmpty);
    });

    testWidgets('Resize Android view', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      viewsController.resizeCompleter = Completer<void>();

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 100.0,
            height: 50.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      final Layer textureParentLayer = tester.layers[tester.layers.length - 2];
      expect(textureParentLayer, isA<ClipRectLayer>());
      final ClipRectLayer clipRect = textureParentLayer as ClipRectLayer;
      expect(clipRect.clipRect, const Rect.fromLTWH(0.0, 0.0, 100.0, 50.0));
      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'webview',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );

      viewsController.resizeCompleter!.complete();
      await tester.pump();

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'webview',
            const Size(100.0, 50.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Change Android view type', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      viewsController.registerViewType('maps');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'maps', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 2,
            'maps',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Dispose Android view', (WidgetTester tester) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      expect(viewsController.views, isEmpty);
    });

    testWidgets('Android view survives widget tree change', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'webview',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Android view gets touch events', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view transparent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AndroidView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.transparent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      expect(viewsController.motionEvents[currentViewId + 1], isNull);
      expect(numPointerDownsOnParent, 1);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets('Android view translucent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AndroidView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.translucent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
      expect(numPointerDownsOnParent, 1);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets('Android view opaque hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
                ),
              ),
            ],
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
      expect(numPointerDownsOnParent, 0);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets("Android view touch events are in virtual display's coordinate system", (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: Container(
            margin: const EdgeInsets.all(10.0),
            child: const SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(40.0, 40.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(40.0, 40.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view directionality', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('maps');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'maps', layoutDirection: TextDirection.rtl),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'maps',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionRtl,
          ),
        ]),
      );

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'maps', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'maps',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Android view ambient directionality', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('maps');
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.rtl,
          child: Center(
            child: SizedBox(width: 200.0, height: 100.0, child: AndroidView(viewType: 'maps')),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'maps',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionRtl,
          ),
        ]),
      );

      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(width: 200.0, height: 100.0, child: AndroidView(viewType: 'maps')),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAndroidPlatformView>[
          FakeAndroidPlatformView(
            currentViewId + 1,
            'maps',
            const Size(200.0, 100.0),
            AndroidViewController.kAndroidLayoutDirectionLtr,
          ),
        ]),
      );
    });

    testWidgets('Android view can lose gesture arenas', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: Container(
            margin: const EdgeInsets.all(10.0),
            child: GestureDetector(
              onVerticalDragStart: (DragStartDetails d) {
                verticalDragAcceptedByParent = true;
              },
              child: const SizedBox(
                width: 200.0,
                height: 100.0,
                child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, true);
      expect(viewsController.motionEvents[currentViewId + 1], isNull);
    });

    testWidgets('Android view drag gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<VerticalDragGestureRecognizer>(() {
                    final VerticalDragGestureRecognizer recognizer =
                        VerticalDragGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);
      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionMove,
            <int>[0],
            <Offset>[Offset(50.0, 150.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 150.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view long press gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      bool longPressAccessedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onLongPress: () {
              longPressAccessedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<LongPressGestureRecognizer>(() {
                    final LongPressGestureRecognizer recognizer = LongPressGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      await tester.longPressAt(const Offset(50.0, 50.0));

      expect(longPressAccessedByParent, false);
      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view tap gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      bool tapAccessedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onTap: () {
              tapAccessedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<TapGestureRecognizer>(() {
                    final TapGestureRecognizer recognizer = TapGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      await tester.tapAt(const Offset(50.0, 50.0));

      expect(tapAccessedByParent, false);
      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view can claim gesture after all pointers are up', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      // The long press recognizer rejects the gesture after the AndroidView gets the pointer up event.
      // This test makes sure that the Android view can win the gesture after it got the pointer up event.
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            onLongPress: () {},
            child: const SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);
      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view rebuilt during gesture', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));

      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await gesture.up();

      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionMove,
            <int>[0],
            <Offset>[Offset(50.0, 150.0)],
          ),
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionUp,
            <int>[0],
            <Offset>[Offset(50.0, 150.0)],
          ),
        ]),
      );
    });

    testWidgets('Android view with eager gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {},
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<OneSequenceGestureRecognizer>(() {
                    final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      // Normally (without the eager gesture recognizer) after just the pointer down event
      // no gesture arena member will claim the arena (so no motion events will be dispatched to
      // the Android view). Here we assert that with the eager recognizer in the gesture team the
      // pointer down event is immediately dispatched.
      expect(
        viewsController.motionEvents[currentViewId + 1],
        orderedEquals(<FakeAndroidMotionEvent>[
          const FakeAndroidMotionEvent(
            AndroidViewController.kActionDown,
            <int>[0],
            <Offset>[Offset(50.0, 50.0)],
          ),
        ]),
      );

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    // This test makes sure it doesn't crash.
    // https://github.com/flutter/flutter/issues/21514
    testWidgets('RenderAndroidView reconstructed with same gestureRecognizers does not crash', (
      WidgetTester tester,
    ) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      final AndroidView androidView = AndroidView(
        viewType: 'webview',
        gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
          Factory<EagerGestureRecognizer>(() {
            final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
            addTearDown(recognizer.dispose);
            return recognizer;
          }),
        },
        layoutDirection: TextDirection.ltr,
      );

      await tester.pumpWidget(androidView);
      await tester.pumpWidget(const SizedBox.shrink());
      await tester.pumpWidget(androidView);
    });

    testWidgets('AndroidView rebuilt with same gestureRecognizers', (WidgetTester tester) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      int factoryInvocationCount = 0;
      EagerGestureRecognizer constructRecognizer() {
        factoryInvocationCount += 1;
        final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
        addTearDown(recognizer.dispose);
        return recognizer;
      }

      await tester.pumpWidget(
        AndroidView(
          viewType: 'webview',
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      await tester.pumpWidget(
        AndroidView(
          viewType: 'webview',
          hitTestBehavior: PlatformViewHitTestBehavior.translucent,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      expect(factoryInvocationCount, 1);
    });

    testWidgets('AndroidView has correct semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      expect(currentViewId, greaterThanOrEqualTo(0));
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      viewsController.createCompleter = Completer<void>();

      await tester.pumpWidget(
        Semantics(
          container: true,
          child: const Align(
            alignment: Alignment.bottomRight,
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );

      // Find the first _AndroidPlatformView widget inside of the AndroidView so
      // that it finds the right RenderObject when looking for semantics.
      final Finder semanticsFinder = find.byWidgetPredicate((Widget widget) {
        return widget.runtimeType.toString() == '_AndroidPlatformView';
      }, description: '_AndroidPlatformView widget inside AndroidView');
      final SemanticsNode semantics = tester.getSemantics(semanticsFinder.first);

      // Platform view has not been created yet, no platformViewId.
      expect(semantics.platformViewId, null);
      expect(semantics.rect, const Rect.fromLTWH(0, 0, 200, 100));
      // A 200x100 rect positioned at bottom right of a 800x600 box.
      expect(semantics.transform, Matrix4.translationValues(600, 500, 0));
      expect(semantics.childrenCount, 0);

      viewsController.createCompleter!.complete();
      await tester.pumpAndSettle();

      expect(semantics.platformViewId, currentViewId + 1);
      expect(semantics.rect, const Rect.fromLTWH(0, 0, 200, 100));
      // A 200x100 rect positioned at bottom right of a 800x600 box.
      expect(semantics.transform, Matrix4.translationValues(600, 500, 0));
      expect(semantics.childrenCount, 0);

      handle.dispose();
    });

    testWidgets('AndroidView can take input focus', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      viewsController.createCompleter = Completer<void>();

      final GlobalKey containerKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              const SizedBox(
                width: 200.0,
                height: 100.0,
                child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );

      final Focus androidViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(AndroidView), matching: find.byType(Focus)),
      );
      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode androidViewFocusNode = androidViewFocusWidget.focusNode!;
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();

      viewsController.createCompleter!.complete();
      await tester.pump();

      expect(containerFocusNode.hasFocus, isTrue);
      expect(androidViewFocusNode.hasFocus, isFalse);

      viewsController.invokeViewFocused(currentViewId + 1);

      await tester.pump();

      expect(containerFocusNode.hasFocus, isFalse);
      expect(androidViewFocusNode.hasFocus, isTrue);
    });

    testWidgets('AndroidView sets a platform view text input client when focused', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      viewsController.createCompleter = Completer<void>();

      final GlobalKey containerKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              const SizedBox(
                width: 200.0,
                height: 100.0,
                child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );

      viewsController.createCompleter!.complete();

      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();
      await tester.pump();

      late Map<String, dynamic> lastPlatformViewTextClient;
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall call,
      ) {
        if (call.method == 'TextInput.setPlatformViewClient') {
          lastPlatformViewTextClient = call.arguments as Map<String, dynamic>;
        }
        return null;
      });

      viewsController.invokeViewFocused(currentViewId + 1);
      await tester.pump();

      expect(lastPlatformViewTextClient.containsKey('platformViewId'), true);
      expect(lastPlatformViewTextClient['platformViewId'], currentViewId + 1);
    });

    testWidgets('AndroidView clears platform focus when unfocused', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      viewsController.createCompleter = Completer<void>();

      final GlobalKey containerKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              const SizedBox(
                width: 200.0,
                height: 100.0,
                child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );

      viewsController.createCompleter!.complete();

      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();
      await tester.pump();

      viewsController.invokeViewFocused(currentViewId + 1);
      await tester.pump();

      viewsController.lastClearedFocusViewId = null;

      containerFocusNode.requestFocus();
      await tester.pump();

      expect(viewsController.lastClearedFocusViewId, currentViewId + 1);
    });

    testWidgets('can set and update clipBehavior', (WidgetTester tester) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      // By default, clipBehavior should be Clip.hardEdge
      final RenderAndroidView renderObject = tester.renderObject(
        find.descendant(
          of: find.byType(AndroidView),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget.runtimeType.toString() == '_AndroidPlatformView',
          ),
        ),
      );
      expect(renderObject.clipBehavior, equals(Clip.hardEdge));

      for (final Clip clip in Clip.values) {
        await tester.pumpWidget(
          Center(
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AndroidView(
                viewType: 'webview',
                layoutDirection: TextDirection.ltr,
                clipBehavior: clip,
              ),
            ),
          ),
        );
        expect(renderObject.clipBehavior, clip);
      }
    });

    testWidgets('clip is handled correctly during resizing', (WidgetTester tester) async {
      // Regressing test for https://github.com/flutter/flutter/issues/67343

      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      Widget buildView(double width, double height, Clip clipBehavior) {
        return Center(
          child: SizedBox(
            width: width,
            height: height,
            child: AndroidView(
              viewType: 'webview',
              layoutDirection: TextDirection.ltr,
              clipBehavior: clipBehavior,
            ),
          ),
        );
      }

      await tester.pumpWidget(buildView(200.0, 200.0, Clip.none));
      // Resize the view.
      await tester.pumpWidget(buildView(100.0, 100.0, Clip.none));
      // No clip happen when the clip behavior is `Clip.none` .
      expect(tester.layers.whereType<ClipRectLayer>(), hasLength(0));

      // No clip when only the clip behavior changes while the size remains the same.
      await tester.pumpWidget(buildView(100.0, 100.0, Clip.hardEdge));
      expect(tester.layers.whereType<ClipRectLayer>(), hasLength(0));

      // Resize trigger clip when the clip behavior is not `Clip.none` .
      await tester.pumpWidget(buildView(50.0, 100.0, Clip.hardEdge));
      expect(tester.layers.whereType<ClipRectLayer>(), hasLength(1));
      ClipRectLayer clipRectLayer = tester.layers.whereType<ClipRectLayer>().first;
      expect(clipRectLayer.clipRect, const Rect.fromLTWH(0.0, 0.0, 50.0, 100.0));

      await tester.pumpWidget(buildView(50.0, 50.0, Clip.hardEdge));
      expect(tester.layers.whereType<ClipRectLayer>(), hasLength(1));
      clipRectLayer = tester.layers.whereType<ClipRectLayer>().first;
      expect(clipRectLayer.clipRect, const Rect.fromLTWH(0.0, 0.0, 50.0, 50.0));
    });

    testWidgets('offset is sent to the platform', (WidgetTester tester) async {
      final FakeAndroidPlatformViewsController viewsController =
          FakeAndroidPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Padding(
          padding: EdgeInsets.fromLTRB(10, 20, 0, 0),
          child: AndroidView(viewType: 'webview', layoutDirection: TextDirection.ltr),
        ),
      );

      await tester.pump();
      expect(viewsController.offsets.values, equals(<Offset>[const Offset(10, 20)]));
    });
  });

  group('AndroidViewSurface', () {
    late FakeAndroidViewController controller;

    setUp(() {
      controller = FakeAndroidViewController(0);
    });

    testWidgets('AndroidViewSurface sets pointTransformer of view controller', (
      WidgetTester tester,
    ) async {
      final AndroidViewSurface surface = AndroidViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
      );
      await tester.pumpWidget(surface);
      expect(controller.pointTransformer, isNotNull);
    });

    testWidgets('AndroidViewSurface defaults to texture-based rendering', (
      WidgetTester tester,
    ) async {
      final AndroidViewSurface surface = AndroidViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
      );
      await tester.pumpWidget(surface);

      expect(
        find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == '_TextureBasedAndroidViewSurface',
        ),
        findsOneWidget,
      );
    });

    testWidgets('AndroidViewSurface uses view-based rendering when initially required', (
      WidgetTester tester,
    ) async {
      controller.requiresViewComposition = true;
      final AndroidViewSurface surface = AndroidViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
      );
      await tester.pumpWidget(surface);

      expect(
        find.byWidgetPredicate(
          (Widget widget) =>
              widget.runtimeType.toString() == '_PlatformLayerBasedAndroidViewSurface',
        ),
        findsOneWidget,
      );
    });

    testWidgets('AndroidViewSurface can switch to view-based rendering after creation', (
      WidgetTester tester,
    ) async {
      final AndroidViewSurface surface = AndroidViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
      );
      await tester.pumpWidget(surface);

      expect(
        find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == '_TextureBasedAndroidViewSurface',
        ),
        findsOneWidget,
      );
      expect(
        find.byWidgetPredicate(
          (Widget widget) =>
              widget.runtimeType.toString() == '_PlatformLayerBasedAndroidViewSurface',
        ),
        findsNothing,
      );

      // Simulate a creation-time switch to view composition.
      controller.requiresViewComposition = true;
      for (final PlatformViewCreatedCallback callback in controller.createdCallbacks) {
        callback(controller.viewId);
      }
      await tester.pumpWidget(surface);

      expect(
        find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == '_TextureBasedAndroidViewSurface',
        ),
        findsNothing,
      );
      expect(
        find.byWidgetPredicate(
          (Widget widget) =>
              widget.runtimeType.toString() == '_PlatformLayerBasedAndroidViewSurface',
        ),
        findsOneWidget,
      );
    });
  });

  group('UiKitView', () {
    testWidgets('Create UIView', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeUiKitView>[FakeUiKitView(currentViewId + 1, 'webview')]),
      );
    });

    testWidgets('Change UIView view type', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      viewsController.registerViewType('maps');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'maps', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeUiKitView>[FakeUiKitView(currentViewId + 2, 'maps')]),
      );
    });

    testWidgets('Dispose UIView ', (WidgetTester tester) async {
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      expect(viewsController.views, isEmpty);
    });

    testWidgets('Dispose UIView before creation completed ', (WidgetTester tester) async {
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      viewsController.creationDelay = Completer<void>();
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      viewsController.creationDelay!.complete();

      expect(viewsController.views, isEmpty);
    });

    testWidgets('UIView survives widget tree change', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeUiKitView>[FakeUiKitView(currentViewId + 1, 'webview')]),
      );
    });

    testWidgets('Create UIView with params', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(
              viewType: 'webview',
              layoutDirection: TextDirection.ltr,
              creationParams: 'creation parameters',
              creationParamsCodec: StringCodec(),
            ),
          ),
        ),
      );

      final FakeUiKitView fakeView = viewsController.views.first;
      final Uint8List rawCreationParams = fakeView.creationParams!;
      final ByteData byteData = ByteData.view(
        rawCreationParams.buffer,
        rawCreationParams.offsetInBytes,
        rawCreationParams.lengthInBytes,
      );
      final dynamic actualParams = const StringCodec().decodeMessage(byteData);

      expect(actualParams, 'creation parameters');
      expect(
        viewsController.views,
        unorderedEquals(<FakeUiKitView>[
          FakeUiKitView(currentViewId + 1, 'webview', fakeView.creationParams),
        ]),
      );
    });

    testWidgets('UiKitView accepts gestures', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
    });

    testWidgets('UiKitView transparent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: UiKitView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.transparent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);

      expect(numPointerDownsOnParent, 1);
    });

    testWidgets('UiKitView translucent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: UiKitView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.translucent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);

      expect(numPointerDownsOnParent, 1);
    });

    testWidgets('UiKitView opaque hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(numPointerDownsOnParent, 0);
    });

    testWidgets('UiKitView can lose gesture arenas', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: Container(
            margin: const EdgeInsets.all(10.0),
            child: GestureDetector(
              onVerticalDragStart: (DragStartDetails d) {
                verticalDragAcceptedByParent = true;
              },
              child: const SizedBox(
                width: 200.0,
                height: 100.0,
                child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, true);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
      expect(viewsController.gesturesRejected[currentViewId + 1], 1);
    });

    testWidgets('UiKitView tap gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool gestureAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              gestureAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<VerticalDragGestureRecognizer>(() {
                    final VerticalDragGestureRecognizer recognizer =
                        VerticalDragGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(gestureAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView long press gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool gestureAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onLongPress: () {
              gestureAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<LongPressGestureRecognizer>(() {
                    final LongPressGestureRecognizer recognizer = LongPressGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      await tester.longPressAt(const Offset(50.0, 50.0));

      expect(gestureAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView drag gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<TapGestureRecognizer>(() {
                    final TapGestureRecognizer recognizer = TapGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      await tester.tapAt(const Offset(50.0, 50.0));

      expect(verticalDragAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView can claim gesture after all pointers are up', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      // The long press recognizer rejects the gesture after the AndroidView gets the pointer up event.
      // This test makes sure that the Android view can win the gesture after it got the pointer up event.
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            onLongPress: () {},
            child: const SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView rebuilt during gesture', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));

      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView with eager gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {},
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<OneSequenceGestureRecognizer>(() {
                    final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      // Normally (without the eager gesture recognizer) after just the pointer down event
      // no gesture arena member will claim the arena (so no motion events will be dispatched to
      // the Android view). Here we assert that with the eager recognizer in the gesture team the
      // pointer down event is immediately dispatched.
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets('UiKitView rejects gestures absorbed by siblings', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        Stack(
          alignment: Alignment.topLeft,
          children: <Widget>[
            const UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            Container(color: const Color.fromARGB(255, 255, 255, 255), width: 100, height: 100),
          ],
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesRejected[currentViewId + 1], 1);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
    });

    testWidgets(
      'UiKitView rejects gestures absorbed by siblings if the touch is outside of the platform view bounds but inside platform view frame',
      (WidgetTester tester) async {
        // UiKitView is positioned at (left=0, top=100, right=300, bottom=600).
        // Opaque container is on top of the UiKitView positioned at (left=0, top=500, right=300, bottom=600).
        // Touch on (550, 150) is expected to be absorbed by the container.
        final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
        final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
        viewsController.registerViewType('webview');

        await tester.pumpWidget(
          SizedBox(
            width: 300,
            height: 600,
            child: Stack(
              alignment: Alignment.topLeft,
              children: <Widget>[
                Transform.translate(
                  offset: const Offset(0, 100),
                  child: const SizedBox(
                    width: 300,
                    height: 500,
                    child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
                  ),
                ),
                Transform.translate(
                  offset: const Offset(0, 500),
                  child: Container(
                    color: const Color.fromARGB(255, 255, 255, 255),
                    width: 300,
                    height: 100,
                  ),
                ),
              ],
            ),
          ),
        );

        // First frame is before the platform view was created so the render object
        // is not yet in the tree.
        await tester.pump();

        final TestGesture gesture = await tester.startGesture(const Offset(150, 550));
        await gesture.up();

        expect(viewsController.gesturesRejected[currentViewId + 1], 1);
        expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
      },
    );

    testWidgets('UiKitView rebuilt with same gestureRecognizers', (WidgetTester tester) async {
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      int factoryInvocationCount = 0;
      EagerGestureRecognizer constructRecognizer() {
        factoryInvocationCount += 1;
        final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
        addTearDown(recognizer.dispose);
        return recognizer;
      }

      await tester.pumpWidget(
        UiKitView(
          viewType: 'webview',
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      await tester.pumpWidget(
        UiKitView(
          viewType: 'webview',
          hitTestBehavior: PlatformViewHitTestBehavior.translucent,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      expect(factoryInvocationCount, 1);
    });

    testWidgets('UiKitView can take input focus', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      final GlobalKey containerKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              const SizedBox(
                width: 200.0,
                height: 100.0,
                child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final Focus uiKitViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(UiKitView), matching: find.byType(Focus)),
      );
      final FocusNode uiKitViewFocusNode = uiKitViewFocusWidget.focusNode!;
      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();

      await tester.pump();

      expect(containerFocusNode.hasFocus, isTrue);
      expect(uiKitViewFocusNode.hasFocus, isFalse);

      viewsController.invokeViewFocused(currentViewId + 1);

      await tester.pump();

      expect(containerFocusNode.hasFocus, isFalse);
      expect(uiKitViewFocusNode.hasFocus, isTrue);
    });

    testWidgets('UiKitView sends TextInput.setPlatformViewClient when focused', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final Focus uiKitViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(UiKitView), matching: find.byType(Focus)),
      );
      final FocusNode uiKitViewFocusNode = uiKitViewFocusWidget.focusNode!;

      late Map<String, dynamic> channelArguments;
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall call,
      ) {
        if (call.method == 'TextInput.setPlatformViewClient') {
          channelArguments = call.arguments as Map<String, dynamic>;
        }
        return null;
      });

      expect(uiKitViewFocusNode.hasFocus, false);

      uiKitViewFocusNode.requestFocus();
      await tester.pump();

      expect(uiKitViewFocusNode.hasFocus, true);
      expect(channelArguments['platformViewId'], currentViewId + 1);
    });

    testWidgets('FocusNode is disposed on UIView dispose', (WidgetTester tester) async {
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );
      // casting to dynamic is required since the state class is private.
      // ignore: invalid_assignment
      final FocusNode node = (tester.state(find.byType(UiKitView)) as dynamic).focusNode;
      expect(() => ChangeNotifier.debugAssertNotDisposed(node), isNot(throwsAssertionError));
      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));
      expect(() => ChangeNotifier.debugAssertNotDisposed(node), throwsAssertionError);
    });

    testWidgets('UiKitView has correct semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      expect(currentViewId, greaterThanOrEqualTo(0));
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        Semantics(
          container: true,
          child: const Align(
            alignment: Alignment.bottomRight,
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );
      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final SemanticsNode semantics = tester.getSemantics(
        find.descendant(
          of: find.byType(UiKitView),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget.runtimeType.toString() == '_UiKitPlatformView',
          ),
        ),
      );

      expect(semantics.platformViewId, currentViewId + 1);
      expect(semantics.rect, const Rect.fromLTWH(0, 0, 200, 100));
      // A 200x100 rect positioned at bottom right of a 800x600 box.
      expect(semantics.transform, Matrix4.translationValues(600, 500, 0));
      expect(semantics.childrenCount, 0);

      handle.dispose();
    });
  });

  group('AppKitView', () {
    testWidgets('Create AppView', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAppKitView>[FakeAppKitView(currentViewId + 1, 'webview')]),
      );
    });

    testWidgets('Change AppKitView view type', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');
      viewsController.registerViewType('maps');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'maps', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAppKitView>[FakeAppKitView(currentViewId + 2, 'maps')]),
      );
    });

    testWidgets('Dispose AppKitView ', (WidgetTester tester) async {
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      expect(viewsController.views, isEmpty);
    });

    testWidgets('Dispose AppKitView before creation completed ', (WidgetTester tester) async {
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');
      viewsController.creationDelay = Completer<void>();
      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));

      viewsController.creationDelay!.complete();

      expect(viewsController.views, isEmpty);
    });

    testWidgets('AppKitView survives widget tree change', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr, key: key),
          ),
        ),
      );

      expect(
        viewsController.views,
        unorderedEquals(<FakeAppKitView>[FakeAppKitView(currentViewId + 1, 'webview')]),
      );
    });

    testWidgets('Create AppKitView with params', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(
              viewType: 'webview',
              layoutDirection: TextDirection.ltr,
              creationParams: 'creation parameters',
              creationParamsCodec: StringCodec(),
            ),
          ),
        ),
      );

      final FakeAppKitView fakeView = viewsController.views.first;
      final Uint8List rawCreationParams = fakeView.creationParams!;
      final ByteData byteData = ByteData.view(
        rawCreationParams.buffer,
        rawCreationParams.offsetInBytes,
        rawCreationParams.lengthInBytes,
      );
      final dynamic actualParams = const StringCodec().decodeMessage(byteData);

      expect(actualParams, 'creation parameters');
      expect(
        viewsController.views,
        unorderedEquals(<FakeAppKitView>[
          FakeAppKitView(currentViewId + 1, 'webview', fakeView.creationParams),
        ]),
      );
    });

    // TODO(schectman): De-skip the following tests once macOS gesture recognizers are present.
    // https://github.com/flutter/flutter/issues/128519
    testWidgets('AppKitView accepts gestures', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
    }, skip: true); // https://github.com/flutter/flutter/issues/128519

    testWidgets('AppKitView transparent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AppKitView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.transparent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);

      expect(numPointerDownsOnParent, 1);
    }, skip: true); // https://github.com/flutter/flutter/issues/128519

    testWidgets('AppKitView translucent hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AppKitView(
                    viewType: 'webview',
                    hitTestBehavior: PlatformViewHitTestBehavior.translucent,
                    layoutDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);

      expect(numPointerDownsOnParent, 1);
    }, skip: true); // https://github.com/flutter/flutter/issues/128519

    testWidgets('AppKitView opaque hit test behavior', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      int numPointerDownsOnParent = 0;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Listener(
                behavior: HitTestBehavior.opaque,
                onPointerDown: (PointerDownEvent e) {
                  numPointerDownsOnParent++;
                },
              ),
              const Positioned(
                child: SizedBox(
                  width: 200.0,
                  height: 100.0,
                  child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
                ),
              ),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(numPointerDownsOnParent, 0);
    }, skip: true); // https://github.com/flutter/flutter/issues/128519

    testWidgets('UiKitView can lose gesture arenas', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: Container(
            margin: const EdgeInsets.all(10.0),
            child: GestureDetector(
              onVerticalDragStart: (DragStartDetails d) {
                verticalDragAcceptedByParent = true;
              },
              child: const SizedBox(
                width: 200.0,
                height: 100.0,
                child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, true);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
      expect(viewsController.gesturesRejected[currentViewId + 1], 1);
    });

    testWidgets('UiKitView tap gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool gestureAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              gestureAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<VerticalDragGestureRecognizer>(() {
                    final VerticalDragGestureRecognizer recognizer =
                        VerticalDragGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(gestureAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView long press gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool gestureAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onLongPress: () {
              gestureAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<LongPressGestureRecognizer>(() {
                    final LongPressGestureRecognizer recognizer = LongPressGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      await tester.longPressAt(const Offset(50.0, 50.0));

      expect(gestureAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView drag gesture recognizers', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<TapGestureRecognizer>(() {
                    final TapGestureRecognizer recognizer = TapGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      await tester.tapAt(const Offset(50.0, 50.0));

      expect(verticalDragAcceptedByParent, false);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView can claim gesture after all pointers are up', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      bool verticalDragAcceptedByParent = false;
      // The long press recognizer rejects the gesture after the AndroidView gets the pointer up event.
      // This test makes sure that the Android view can win the gesture after it got the pointer up event.
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            onLongPress: () {},
            child: const SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView rebuilt during gesture', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));

      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );

      await gesture.up();

      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);
    });

    testWidgets('UiKitView with eager gesture recognizer', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {},
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: UiKitView(
                viewType: 'webview',
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<OneSequenceGestureRecognizer>(() {
                    final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
                layoutDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      // Normally (without the eager gesture recognizer) after just the pointer down event
      // no gesture arena member will claim the arena (so no motion events will be dispatched to
      // the Android view). Here we assert that with the eager recognizer in the gesture team the
      // pointer down event is immediately dispatched.
      expect(viewsController.gesturesAccepted[currentViewId + 1], 1);
      expect(viewsController.gesturesRejected[currentViewId + 1], 0);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets('UiKitView rejects gestures absorbed by siblings', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        Stack(
          alignment: Alignment.topLeft,
          children: <Widget>[
            const UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            Container(color: const Color.fromARGB(255, 255, 255, 255), width: 100, height: 100),
          ],
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(viewsController.gesturesRejected[currentViewId + 1], 1);
      expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
    });

    testWidgets(
      'UiKitView rejects gestures absorbed by siblings if the touch is outside of the platform view bounds but inside platform view frame',
      (WidgetTester tester) async {
        // UiKitView is positioned at (left=0, top=100, right=300, bottom=600).
        // Opaque container is on top of the UiKitView positioned at (left=0, top=500, right=300, bottom=600).
        // Touch on (550, 150) is expected to be absorbed by the container.
        final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
        final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
        viewsController.registerViewType('webview');

        await tester.pumpWidget(
          SizedBox(
            width: 300,
            height: 600,
            child: Stack(
              alignment: Alignment.topLeft,
              children: <Widget>[
                Transform.translate(
                  offset: const Offset(0, 100),
                  child: const SizedBox(
                    width: 300,
                    height: 500,
                    child: UiKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
                  ),
                ),
                Transform.translate(
                  offset: const Offset(0, 500),
                  child: Container(
                    color: const Color.fromARGB(255, 255, 255, 255),
                    width: 300,
                    height: 100,
                  ),
                ),
              ],
            ),
          ),
        );

        // First frame is before the platform view was created so the render object
        // is not yet in the tree.
        await tester.pump();

        final TestGesture gesture = await tester.startGesture(const Offset(150, 550));
        await gesture.up();

        expect(viewsController.gesturesRejected[currentViewId + 1], 1);
        expect(viewsController.gesturesAccepted[currentViewId + 1], 0);
      },
    );

    testWidgets('UiKitView rebuilt with same gestureRecognizers', (WidgetTester tester) async {
      final FakeIosPlatformViewsController viewsController = FakeIosPlatformViewsController();
      viewsController.registerViewType('webview');

      int factoryInvocationCount = 0;
      EagerGestureRecognizer constructRecognizer() {
        factoryInvocationCount += 1;
        final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
        addTearDown(recognizer.dispose);
        return recognizer;
      }

      await tester.pumpWidget(
        UiKitView(
          viewType: 'webview',
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      await tester.pumpWidget(
        UiKitView(
          viewType: 'webview',
          hitTestBehavior: PlatformViewHitTestBehavior.translucent,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<EagerGestureRecognizer>(constructRecognizer),
          },
          layoutDirection: TextDirection.ltr,
        ),
      );

      expect(factoryInvocationCount, 1);
    });

    testWidgets('AppKitView can take input focus', (WidgetTester tester) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      final GlobalKey containerKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              const SizedBox(
                width: 200.0,
                height: 100.0,
                child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
              ),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final Focus uiKitViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(AppKitView), matching: find.byType(Focus)),
      );
      final FocusNode uiKitViewFocusNode = uiKitViewFocusWidget.focusNode!;
      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();

      await tester.pump();

      expect(containerFocusNode.hasFocus, isTrue);
      expect(uiKitViewFocusNode.hasFocus, isFalse);

      viewsController.invokeViewFocused(currentViewId + 1);

      await tester.pump();

      expect(containerFocusNode.hasFocus, isFalse);
      expect(uiKitViewFocusNode.hasFocus, isTrue);
    });

    testWidgets('AppKitView sends TextInput.setPlatformViewClient when focused', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
      );

      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final Focus uiKitViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(AppKitView), matching: find.byType(Focus)),
      );
      final FocusNode uiKitViewFocusNode = uiKitViewFocusWidget.focusNode!;

      late Map<String, dynamic> channelArguments;
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall call,
      ) {
        if (call.method == 'TextInput.setPlatformViewClient') {
          channelArguments = call.arguments as Map<String, dynamic>;
        }
        return null;
      });

      expect(uiKitViewFocusNode.hasFocus, false);

      uiKitViewFocusNode.requestFocus();
      await tester.pump();

      expect(uiKitViewFocusNode.hasFocus, true);
      expect(channelArguments['platformViewId'], currentViewId + 1);
    });

    testWidgets('FocusNode is disposed on UIView dispose', (WidgetTester tester) async {
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        const Center(
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
          ),
        ),
      );
      // casting to dynamic is required since the state class is private.
      // ignore: invalid_assignment
      final FocusNode node = (tester.state(find.byType(AppKitView)) as dynamic).focusNode;
      expect(() => ChangeNotifier.debugAssertNotDisposed(node), isNot(throwsAssertionError));
      await tester.pumpWidget(const Center(child: SizedBox(width: 200.0, height: 100.0)));
      expect(() => ChangeNotifier.debugAssertNotDisposed(node), throwsAssertionError);
    });

    testWidgets('AppKitView has correct semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      expect(currentViewId, greaterThanOrEqualTo(0));
      final FakeMacosPlatformViewsController viewsController = FakeMacosPlatformViewsController();
      viewsController.registerViewType('webview');

      await tester.pumpWidget(
        Semantics(
          container: true,
          child: const Align(
            alignment: Alignment.bottomRight,
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: AppKitView(viewType: 'webview', layoutDirection: TextDirection.ltr),
            ),
          ),
        ),
      );
      // First frame is before the platform view was created so the render object
      // is not yet in the tree.
      await tester.pump();

      final SemanticsNode semantics = tester.getSemantics(
        find.descendant(
          of: find.byType(AppKitView),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget.runtimeType.toString() == '_AppKitPlatformView',
          ),
        ),
      );

      expect(semantics.platformViewId, currentViewId + 1);
      expect(semantics.rect, const Rect.fromLTWH(0, 0, 200, 100));
      // A 200x100 rect positioned at bottom right of a 800x600 box.
      expect(semantics.transform, Matrix4.translationValues(600, 500, 0));
      expect(semantics.childrenCount, 0);

      handle.dispose();
    });
  });

  group('Common PlatformView', () {
    late FakePlatformViewController controller;

    setUp(() {
      controller = FakePlatformViewController(0);
    });

    testWidgets('PlatformViewSurface should create platform view layer', (
      WidgetTester tester,
    ) async {
      final PlatformViewSurface surface = PlatformViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
      );
      await tester.pumpWidget(surface);
      expect(() => tester.layers.whereType<PlatformViewLayer>().first, returnsNormally);
    });

    testWidgets('PlatformViewSurface can lose gesture arenas', (WidgetTester tester) async {
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: Container(
            margin: const EdgeInsets.all(10.0),
            child: GestureDetector(
              onVerticalDragStart: (DragStartDetails d) {
                verticalDragAcceptedByParent = true;
              },
              child: SizedBox(
                width: 200.0,
                height: 100.0,
                child: PlatformViewSurface(
                  controller: controller,
                  gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
                  hitTestBehavior: PlatformViewHitTestBehavior.opaque,
                ),
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, true);
      expect(controller.dispatchedPointerEvents, isEmpty);
    });

    testWidgets('PlatformViewSurface gesture recognizers dispatch events', (
      WidgetTester tester,
    ) async {
      bool verticalDragAcceptedByParent = false;
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: PlatformViewSurface(
                controller: controller,
                hitTestBehavior: PlatformViewHitTestBehavior.opaque,
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<VerticalDragGestureRecognizer>(() {
                    final VerticalDragGestureRecognizer recognizer =
                        VerticalDragGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);
      expect(controller.dispatchedPointerEvents.length, 3);
    });

    testWidgets('PlatformViewSurface can claim gesture after all pointers are up', (
      WidgetTester tester,
    ) async {
      bool verticalDragAcceptedByParent = false;
      // The long press recognizer rejects the gesture after the PlatformViewSurface gets the pointer up event.
      // This test makes sure that the PlatformViewSurface can win the gesture after it got the pointer up event.
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {
              verticalDragAcceptedByParent = true;
            },
            onLongPress: () {},
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: PlatformViewSurface(
                controller: controller,
                hitTestBehavior: PlatformViewHitTestBehavior.opaque,
                gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.up();

      expect(verticalDragAcceptedByParent, false);
      expect(controller.dispatchedPointerEvents.length, 2);
    });

    testWidgets('PlatformViewSurface rebuilt during gesture', (WidgetTester tester) async {
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: PlatformViewSurface(
              controller: controller,
              hitTestBehavior: PlatformViewHitTestBehavior.opaque,
              gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));
      await gesture.moveBy(const Offset(0.0, 100.0));

      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 100.0,
            child: PlatformViewSurface(
              controller: controller,
              hitTestBehavior: PlatformViewHitTestBehavior.opaque,
              gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            ),
          ),
        ),
      );

      await gesture.up();

      expect(controller.dispatchedPointerEvents.length, 3);
    });

    testWidgets('PlatformViewSurface with eager gesture recognizer', (WidgetTester tester) async {
      await tester.pumpWidget(
        Align(
          alignment: Alignment.topLeft,
          child: GestureDetector(
            onVerticalDragStart: (DragStartDetails d) {},
            child: SizedBox(
              width: 200.0,
              height: 100.0,
              child: PlatformViewSurface(
                controller: controller,
                hitTestBehavior: PlatformViewHitTestBehavior.opaque,
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<OneSequenceGestureRecognizer>(() {
                    final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
                    addTearDown(recognizer.dispose);
                    return recognizer;
                  }),
                },
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 50.0));

      // Normally (without the eager gesture recognizer) after just the pointer down event
      // no gesture arena member will claim the arena (so no motion events will be dispatched to
      // the PlatformViewSurface). Here we assert that with the eager recognizer in the gesture team the
      // pointer down event is immediately dispatched.
      expect(controller.dispatchedPointerEvents.length, 1);

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });

    testWidgets('PlatformViewRenderBox reconstructed with same gestureRecognizers', (
      WidgetTester tester,
    ) async {
      int factoryInvocationCount = 0;
      EagerGestureRecognizer constructRecognizer() {
        ++factoryInvocationCount;
        final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
        addTearDown(recognizer.dispose);
        return recognizer;
      }

      final PlatformViewSurface platformViewSurface = PlatformViewSurface(
        controller: controller,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
          Factory<OneSequenceGestureRecognizer>(constructRecognizer),
        },
      );

      await tester.pumpWidget(platformViewSurface);
      await tester.pumpWidget(const SizedBox.shrink());
      await tester.pumpWidget(platformViewSurface);

      expect(factoryInvocationCount, 2);
    });

    testWidgets('PlatformViewSurface rebuilt with same gestureRecognizers', (
      WidgetTester tester,
    ) async {
      int factoryInvocationCount = 0;
      EagerGestureRecognizer constructRecognizer() {
        ++factoryInvocationCount;
        final EagerGestureRecognizer recognizer = EagerGestureRecognizer();
        addTearDown(recognizer.dispose);
        return recognizer;
      }

      await tester.pumpWidget(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<OneSequenceGestureRecognizer>(constructRecognizer),
          },
        ),
      );

      await tester.pumpWidget(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
            Factory<OneSequenceGestureRecognizer>(constructRecognizer),
          },
        ),
      );
      expect(factoryInvocationCount, 1);
    });

    testWidgets(
      'PlatformViewLink Widget init, should create a placeholder widget before onPlatformViewCreated and a PlatformViewSurface after',
      (WidgetTester tester) async {
        final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
        late int createdPlatformViewId;

        late PlatformViewCreatedCallback onPlatformViewCreatedCallBack;

        final PlatformViewLink platformViewLink = PlatformViewLink(
          viewType: 'webview',
          onCreatePlatformView: (PlatformViewCreationParams params) {
            onPlatformViewCreatedCallBack = params.onPlatformViewCreated;
            createdPlatformViewId = params.id;
            return FakePlatformViewController(params.id)..create();
          },
          surfaceFactory: (BuildContext context, PlatformViewController controller) {
            return PlatformViewSurface(
              gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
              controller: controller,
              hitTestBehavior: PlatformViewHitTestBehavior.opaque,
            );
          },
        );

        await tester.pumpWidget(platformViewLink);

        expect(
          tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
          containsAllInOrder(<String>['PlatformViewLink', '_PlatformViewPlaceHolder']),
        );

        onPlatformViewCreatedCallBack(createdPlatformViewId);

        await tester.pump();

        expect(
          tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
          containsAllInOrder(<String>[
            'PlatformViewLink',
            'Focus',
            '_FocusInheritedScope',
            'Semantics',
            'PlatformViewSurface',
          ]),
        );

        expect(createdPlatformViewId, currentViewId + 1);
      },
    );

    testWidgets('PlatformViewLink widget should not trigger creation with an empty size', (
      WidgetTester tester,
    ) async {
      late PlatformViewController controller;

      final Widget widget = Center(
        child: SizedBox(
          height: 0,
          child: PlatformViewLink(
            viewType: 'webview',
            onCreatePlatformView: (PlatformViewCreationParams params) {
              controller = FakeAndroidViewController(params.id, requiresSize: true);
              controller.create();
              // This test should be simulating one of the texture-based display
              // modes, where `create` is a no-op when not provided a size, and
              // creation is triggered via a later call to setSize, or to `create`
              // with a size.
              expect(controller.awaitingCreation, true);
              return controller;
            },
            surfaceFactory: (BuildContext context, PlatformViewController controller) {
              return PlatformViewSurface(
                gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
                controller: controller,
                hitTestBehavior: PlatformViewHitTestBehavior.opaque,
              );
            },
          ),
        ),
      );

      await tester.pumpWidget(widget);

      expect(
        tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
        containsAllInOrder(<String>[
          'Center',
          'SizedBox',
          'PlatformViewLink',
          '_PlatformViewPlaceHolder',
        ]),
      );

      // 'create' should not have been called by PlatformViewLink, since its
      // size is empty.
      expect(controller.awaitingCreation, true);
    });

    testWidgets('PlatformViewLink calls create when needed for Android texture display modes', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      late int createdPlatformViewId;

      late PlatformViewCreatedCallback onPlatformViewCreatedCallBack;
      late PlatformViewController controller;

      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          onPlatformViewCreatedCallBack = params.onPlatformViewCreated;
          createdPlatformViewId = params.id;
          controller = FakeAndroidViewController(params.id, requiresSize: true);
          controller.create();
          // This test should be simulating one of the texture-based display
          // modes, where `create` is a no-op when not provided a size, and
          // creation is triggered via a later call to setSize, or to `create`
          // with a size.
          expect(controller.awaitingCreation, true);
          return controller;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );

      await tester.pumpWidget(platformViewLink);

      expect(
        tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
        containsAllInOrder(<String>['PlatformViewLink', '_PlatformViewPlaceHolder']),
      );

      // Layout should have triggered a create call. Simulate the callback
      // that the real controller would make after creation.
      expect(controller.awaitingCreation, false);
      onPlatformViewCreatedCallBack(createdPlatformViewId);

      await tester.pump();

      expect(
        tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
        containsAllInOrder(<String>[
          'PlatformViewLink',
          'Focus',
          '_FocusInheritedScope',
          'Semantics',
          'PlatformViewSurface',
        ]),
      );

      expect(createdPlatformViewId, currentViewId + 1);
    });

    testWidgets('PlatformViewLink includes offset in create call when using texture layer', (
      WidgetTester tester,
    ) async {
      addTearDown(tester.view.reset);

      late FakeAndroidViewController controller;

      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          controller = FakeAndroidViewController(params.id, requiresSize: true);
          controller.create();
          // This test should be simulating one of the texture-based display
          // modes, where `create` is a no-op when not provided a size, and
          // creation is triggered via a later call to setSize, or to `create`
          // with a size.
          expect(controller.awaitingCreation, true);
          return controller;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );

      tester.view.physicalSize = const Size(400, 200);
      tester.view.devicePixelRatio = 1.0;

      await tester.pumpWidget(
        Container(
          constraints: const BoxConstraints.expand(),
          alignment: Alignment.center,
          child: SizedBox(width: 100, height: 50, child: platformViewLink),
        ),
      );

      expect(controller.createPosition, const Offset(150, 75));
    });

    testWidgets('PlatformViewLink does not double-call create for Android Hybrid Composition', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      late int createdPlatformViewId;

      late PlatformViewCreatedCallback onPlatformViewCreatedCallBack;
      late PlatformViewController controller;

      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          onPlatformViewCreatedCallBack = params.onPlatformViewCreated;
          createdPlatformViewId = params.id;
          controller = FakeAndroidViewController(params.id);
          controller.create();
          // This test should be simulating Hybrid Composition mode, where
          // `create` takes effect immediately.
          expect(controller.awaitingCreation, false);
          return controller;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );

      await tester.pumpWidget(platformViewLink);

      expect(
        tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
        containsAllInOrder(<String>['PlatformViewLink', '_PlatformViewPlaceHolder']),
      );

      onPlatformViewCreatedCallBack(createdPlatformViewId);

      await tester.pump();

      expect(
        tester.allWidgets.map((Widget widget) => widget.runtimeType.toString()).toList(),
        containsAllInOrder(<String>[
          'PlatformViewLink',
          'Focus',
          '_FocusInheritedScope',
          'Semantics',
          'PlatformViewSurface',
        ]),
      );

      expect(createdPlatformViewId, currentViewId + 1);
    });

    testWidgets('PlatformViewLink Widget dispose', (WidgetTester tester) async {
      late FakePlatformViewController disposedController;
      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          disposedController = FakePlatformViewController(params.id);
          params.onPlatformViewCreated(params.id);
          return disposedController;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );

      await tester.pumpWidget(platformViewLink);

      await tester.pumpWidget(Container());

      expect(disposedController.disposed, true);
    });

    testWidgets('PlatformViewLink handles onPlatformViewCreated when disposed', (
      WidgetTester tester,
    ) async {
      late PlatformViewCreationParams creationParams;
      late FakePlatformViewController controller;
      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          creationParams = params;
          return controller = FakePlatformViewController(params.id);
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );

      await tester.pumpWidget(platformViewLink);

      await tester.pumpWidget(Container());

      expect(controller.disposed, true);
      expect(() => creationParams.onPlatformViewCreated(creationParams.id), returnsNormally);
    });

    testWidgets('PlatformViewLink widget survives widget tree change', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final List<int> ids = <int>[];

      FakePlatformViewController controller;

      PlatformViewLink createPlatformViewLink() {
        return PlatformViewLink(
          key: key,
          viewType: 'webview',
          onCreatePlatformView: (PlatformViewCreationParams params) {
            ids.add(params.id);
            controller = FakePlatformViewController(params.id);
            params.onPlatformViewCreated(params.id);
            return controller;
          },
          surfaceFactory: (BuildContext context, PlatformViewController controller) {
            return PlatformViewSurface(
              gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
              controller: controller,
              hitTestBehavior: PlatformViewHitTestBehavior.opaque,
            );
          },
        );
      }

      await tester.pumpWidget(
        Center(child: SizedBox(width: 200.0, height: 100.0, child: createPlatformViewLink())),
      );

      await tester.pumpWidget(
        Center(child: SizedBox(width: 200.0, height: 100.0, child: createPlatformViewLink())),
      );

      expect(ids, unorderedEquals(<int>[currentViewId + 1]));
    });

    testWidgets('PlatformViewLink re-initializes when view type changes', (
      WidgetTester tester,
    ) async {
      final int currentViewId = platformViewsRegistry.getNextPlatformViewId();
      final List<int> ids = <int>[];
      final List<int> surfaceViewIds = <int>[];
      final List<String> viewTypes = <String>[];

      PlatformViewLink createPlatformViewLink(String viewType) {
        return PlatformViewLink(
          viewType: viewType,
          onCreatePlatformView: (PlatformViewCreationParams params) {
            ids.add(params.id);
            viewTypes.add(params.viewType);
            controller = FakePlatformViewController(params.id);
            params.onPlatformViewCreated(params.id);
            return controller;
          },
          surfaceFactory: (BuildContext context, PlatformViewController controller) {
            surfaceViewIds.add(controller.viewId);
            return PlatformViewSurface(
              gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
              controller: controller,
              hitTestBehavior: PlatformViewHitTestBehavior.opaque,
            );
          },
        );
      }

      await tester.pumpWidget(
        Center(
          child: SizedBox(width: 200.0, height: 100.0, child: createPlatformViewLink('webview')),
        ),
      );

      await tester.pumpWidget(
        Center(child: SizedBox(width: 200.0, height: 100.0, child: createPlatformViewLink('maps'))),
      );

      expect(ids, unorderedEquals(<int>[currentViewId + 1, currentViewId + 2]));

      expect(surfaceViewIds, unorderedEquals(<int>[currentViewId + 1, currentViewId + 2]));

      expect(viewTypes, unorderedEquals(<String>['webview', 'maps']));
    });

    testWidgets('PlatformViewLink can take any widget to return in the SurfaceFactory', (
      WidgetTester tester,
    ) async {
      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          params.onPlatformViewCreated(params.id);
          return FakePlatformViewController(params.id);
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return Container();
        },
      );

      await tester.pumpWidget(platformViewLink);

      expect(() => tester.allWidgets.whereType<Container>().first, returnsNormally);
    });

    testWidgets('PlatformViewLink manages the focus properly', (WidgetTester tester) async {
      final GlobalKey containerKey = GlobalKey();
      late FakePlatformViewController controller;
      late ValueChanged<bool> focusChanged;
      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'webview',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          params.onPlatformViewCreated(params.id);
          focusChanged = params.onFocusChanged;
          controller = FakePlatformViewController(params.id);
          return controller;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );
      await tester.pumpWidget(
        Center(
          child: Column(
            children: <Widget>[
              SizedBox(width: 300, height: 300, child: platformViewLink),
              Focus(debugLabel: 'container', child: Container(key: containerKey)),
            ],
          ),
        ),
      );
      final Focus platformViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(PlatformViewLink), matching: find.byType(Focus)),
      );
      final FocusNode platformViewFocusNode = platformViewFocusWidget.focusNode!;
      final Element containerElement = tester.element(find.byKey(containerKey));
      final FocusNode containerFocusNode = Focus.of(containerElement);

      containerFocusNode.requestFocus();
      await tester.pump();

      expect(containerFocusNode.hasFocus, true);
      expect(platformViewFocusNode.hasFocus, false);

      // ask the platform view to gain focus
      focusChanged(true);
      await tester.pump();

      expect(containerFocusNode.hasFocus, false);
      expect(platformViewFocusNode.hasFocus, true);
      expect(controller.focusCleared, false);
      // ask the container to gain focus, and the platform view should clear focus.
      containerFocusNode.requestFocus();
      await tester.pump();

      expect(containerFocusNode.hasFocus, true);
      expect(platformViewFocusNode.hasFocus, false);
      expect(controller.focusCleared, true);
    });

    testWidgets('PlatformViewLink sets a platform view text input client when focused', (
      WidgetTester tester,
    ) async {
      late FakePlatformViewController controller;
      late int viewId;

      final PlatformViewLink platformViewLink = PlatformViewLink(
        viewType: 'test',
        onCreatePlatformView: (PlatformViewCreationParams params) {
          viewId = params.id;
          params.onPlatformViewCreated(params.id);
          controller = FakePlatformViewController(params.id);
          return controller;
        },
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return PlatformViewSurface(
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            controller: controller,
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
      );
      await tester.pumpWidget(SizedBox(width: 300, height: 300, child: platformViewLink));

      final Focus platformViewFocusWidget = tester.widget(
        find.descendant(of: find.byType(PlatformViewLink), matching: find.byType(Focus)),
      );

      final FocusNode? focusNode = platformViewFocusWidget.focusNode;
      expect(focusNode, isNotNull);
      expect(focusNode!.hasFocus, false);

      late Map<String, dynamic> lastPlatformViewTextClient;
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall call,
      ) {
        if (call.method == 'TextInput.setPlatformViewClient') {
          lastPlatformViewTextClient = call.arguments as Map<String, dynamic>;
        }
        return null;
      });

      platformViewFocusWidget.focusNode!.requestFocus();
      await tester.pump();

      expect(focusNode.hasFocus, true);
      expect(lastPlatformViewTextClient.containsKey('platformViewId'), true);
      expect(lastPlatformViewTextClient['platformViewId'], viewId);
    });
  });

  testWidgets('Platform views respect hitTestBehavior', (WidgetTester tester) async {
    final FakePlatformViewController controller = FakePlatformViewController(0);

    final List<String> logs = <String>[];

    // -------------------------
    // | MouseRegion1          |       MouseRegion1
    // |  |-----------------|  |        |
    // |  | MouseRegion2    |  |        |- Stack
    // |  |   |---------|   |  |            |
    // |  |   |Platform |   |  |            |- MouseRegion2
    // |  |   |View     |   |  |            |- PlatformView
    // |  |   |---------|   |  |
    // |  |                 |  |
    // |  |-----------------|  |
    // |                       |
    // -------------------------
    Widget scaffold(Widget target) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 600,
            height: 600,
            child: MouseRegion(
              onEnter: (_) {
                logs.add('enter1');
              },
              onExit: (_) {
                logs.add('exit1');
              },
              cursor: SystemMouseCursors.forbidden,
              child: Stack(
                children: <Widget>[
                  Center(
                    child: SizedBox(
                      width: 400,
                      height: 400,
                      child: MouseRegion(
                        onEnter: (_) {
                          logs.add('enter2');
                        },
                        onExit: (_) {
                          logs.add('exit2');
                        },
                        cursor: SystemMouseCursors.text,
                      ),
                    ),
                  ),
                  Center(child: SizedBox(width: 200, height: 200, child: target)),
                ],
              ),
            ),
          ),
        ),
      );
    }

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 0,
    );

    // Test: Opaque
    await tester.pumpWidget(
      scaffold(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
        ),
      ),
    );
    logs.clear();

    await gesture.moveTo(const Offset(400, 300));
    expect(logs, <String>['enter1']);
    expect(controller.dispatchedPointerEvents, hasLength(1));
    expect(controller.dispatchedPointerEvents[0], isA<PointerHoverEvent>());
    logs.clear();
    controller.dispatchedPointerEvents.clear();

    // Test: changing no option does not trigger events
    await tester.pumpWidget(
      scaffold(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
        ),
      ),
    );
    expect(logs, isEmpty);
    expect(controller.dispatchedPointerEvents, isEmpty);

    // Test: Translucent
    await tester.pumpWidget(
      scaffold(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.translucent,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
        ),
      ),
    );
    expect(logs, <String>['enter2']);
    expect(controller.dispatchedPointerEvents, isEmpty);
    logs.clear();

    await gesture.moveBy(const Offset(1, 1));
    expect(logs, isEmpty);
    expect(controller.dispatchedPointerEvents, hasLength(1));
    expect(controller.dispatchedPointerEvents[0], isA<PointerHoverEvent>());
    expect(controller.dispatchedPointerEvents[0].position, const Offset(401, 301));
    expect(controller.dispatchedPointerEvents[0].localPosition, const Offset(101, 101));
    controller.dispatchedPointerEvents.clear();

    // Test: Transparent
    await tester.pumpWidget(
      scaffold(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.transparent,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
        ),
      ),
    );
    expect(logs, isEmpty);
    expect(controller.dispatchedPointerEvents, isEmpty);

    await gesture.moveBy(const Offset(1, 1));
    expect(logs, isEmpty);
    expect(controller.dispatchedPointerEvents, isEmpty);

    // Test: Back to opaque
    await tester.pumpWidget(
      scaffold(
        PlatformViewSurface(
          controller: controller,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
        ),
      ),
    );
    expect(logs, <String>['exit2']);
    expect(controller.dispatchedPointerEvents, isEmpty);
    logs.clear();

    await gesture.moveBy(const Offset(1, 1));
    expect(logs, isEmpty);
    expect(controller.dispatchedPointerEvents, hasLength(1));
    expect(controller.dispatchedPointerEvents[0], isA<PointerHoverEvent>());
  });

  testWidgets('HtmlElementView can be instantiated', (WidgetTester tester) async {
    late final Widget htmlElementView;
    expect(() {
      htmlElementView = const HtmlElementView(viewType: 'webview');
    }, returnsNormally);

    await tester.pumpWidget(
      Center(child: SizedBox(width: 100, height: 100, child: htmlElementView)),
    );
    await tester.pumpAndSettle();

    // This file runs on non-web platforms, so we expect `HtmlElementView` to
    // fail.
    final dynamic exception = tester.takeException();
    expect(exception, isUnimplementedError);
    expect(exception.toString(), contains('HtmlElementView is only available on Flutter Web'));
  });
}
