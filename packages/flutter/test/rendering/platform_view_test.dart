// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui;

import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import '../services/fake_platform_views.dart';
import 'rendering_tester.dart';

void main() {
  final TestRenderingFlutterBinding binding = TestRenderingFlutterBinding.ensureInitialized();

  group('PlatformViewRenderBox', () {
    late FakePlatformViewController fakePlatformViewController;
    late PlatformViewRenderBox platformViewRenderBox;
    setUp(() {
      fakePlatformViewController = FakePlatformViewController(0);
      platformViewRenderBox = PlatformViewRenderBox(
        controller: fakePlatformViewController,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
          Factory<VerticalDragGestureRecognizer>(() {
            return VerticalDragGestureRecognizer();
          }),
        },
      );
    });

    test('layout should size to max constraint', () {
      layout(platformViewRenderBox);
      platformViewRenderBox.layout(
        const BoxConstraints(minWidth: 50, minHeight: 50, maxWidth: 100, maxHeight: 100),
      );
      expect(platformViewRenderBox.size, const Size(100, 100));
    });

    test('send semantics update if id is changed', () {
      final RenderConstrainedBox tree = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(height: 20.0, width: 20.0),
        child: platformViewRenderBox,
      );
      int semanticsUpdateCount = 0;
      final SemanticsHandle semanticsHandle =
          TestRenderingFlutterBinding.instance.ensureSemantics();
      TestRenderingFlutterBinding.instance.pipelineOwner.semanticsOwner!.addListener(() {
        ++semanticsUpdateCount;
      });
      layout(tree, phase: EnginePhase.flushSemantics);
      // Initial semantics update
      expect(semanticsUpdateCount, 1);

      semanticsUpdateCount = 0;

      // Request semantics update even though nothing changed.
      platformViewRenderBox.markNeedsSemanticsUpdate();
      pumpFrame(phase: EnginePhase.flushSemantics);
      expect(semanticsUpdateCount, 0);

      semanticsUpdateCount = 0;

      final FakePlatformViewController updatedFakePlatformViewController =
          FakePlatformViewController(10);
      platformViewRenderBox.controller = updatedFakePlatformViewController;
      pumpFrame(phase: EnginePhase.flushSemantics);
      // Update id should update the semantics.
      expect(semanticsUpdateCount, 1);

      semanticsHandle.dispose();
    });

    test('mouse hover events are dispatched via PlatformViewController.dispatchPointerEvent', () {
      layout(platformViewRenderBox);
      pumpFrame(phase: EnginePhase.flushSemantics);

      RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
        ui.PointerDataPacket(
          data: <ui.PointerData>[
            _pointerData(ui.PointerChange.add, Offset.zero),
            _pointerData(ui.PointerChange.hover, const Offset(10, 10)),
            _pointerData(ui.PointerChange.remove, const Offset(10, 10)),
          ],
        ),
      );

      expect(fakePlatformViewController.dispatchedPointerEvents, isNotEmpty);
    });

    test('touch hover events are dispatched via PlatformViewController.dispatchPointerEvent', () {
      layout(platformViewRenderBox);
      pumpFrame(phase: EnginePhase.flushSemantics);

      RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
        ui.PointerDataPacket(
          data: <ui.PointerData>[
            _pointerData(ui.PointerChange.add, Offset.zero),
            _pointerData(ui.PointerChange.hover, const Offset(10, 10)),
            _pointerData(ui.PointerChange.remove, const Offset(10, 10)),
          ],
        ),
      );

      expect(fakePlatformViewController.dispatchedPointerEvents, isNotEmpty);
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/69431
  test('multi-finger touch test', () {
    final FakeAndroidPlatformViewsController viewsController = FakeAndroidPlatformViewsController();
    viewsController.registerViewType('webview');
    final AndroidViewController viewController = PlatformViewsService.initAndroidView(
      id: 0,
      viewType: 'webview',
      layoutDirection: TextDirection.rtl,
    );
    final PlatformViewRenderBox platformViewRenderBox = PlatformViewRenderBox(
      controller: viewController,
      hitTestBehavior: PlatformViewHitTestBehavior.opaque,
      gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
        Factory<VerticalDragGestureRecognizer>(() => VerticalDragGestureRecognizer()),
      },
    );
    layout(platformViewRenderBox);
    pumpFrame(phase: EnginePhase.flushSemantics);

    viewController.pointTransformer =
        (Offset offset) => platformViewRenderBox.globalToLocal(offset);

    FakeAsync().run((FakeAsync async) {
      // Put one pointer down.
      RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
        ui.PointerDataPacket(
          data: <ui.PointerData>[
            _pointerData(
              ui.PointerChange.add,
              Offset.zero,
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.down,
              const Offset(10, 10),
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.remove,
              const Offset(10, 10),
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
          ],
        ),
      );
      async.flushMicrotasks();

      // Put another pointer down and then cancel it.
      RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
        ui.PointerDataPacket(
          data: <ui.PointerData>[
            _pointerData(
              ui.PointerChange.add,
              Offset.zero,
              pointer: 2,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.down,
              const Offset(20, 10),
              pointer: 2,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.cancel,
              const Offset(20, 10),
              pointer: 2,
              kind: PointerDeviceKind.touch,
            ),
          ],
        ),
      );
      async.flushMicrotasks();

      // The first pointer can still moving without crashing.
      RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
        ui.PointerDataPacket(
          data: <ui.PointerData>[
            _pointerData(
              ui.PointerChange.add,
              Offset.zero,
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.move,
              const Offset(10, 10),
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
            _pointerData(
              ui.PointerChange.remove,
              const Offset(10, 10),
              pointer: 1,
              kind: PointerDeviceKind.touch,
            ),
          ],
        ),
      );
      async.flushMicrotasks();
    });

    // Passes if no crashes.
  });

  test('created callback is reset when controller is changed', () {
    final FakeAndroidPlatformViewsController viewsController = FakeAndroidPlatformViewsController();
    viewsController.registerViewType('webview');
    final AndroidViewController firstController = PlatformViewsService.initAndroidView(
      id: 0,
      viewType: 'webview',
      layoutDirection: TextDirection.rtl,
    );
    final RenderAndroidView renderBox = RenderAndroidView(
      viewController: firstController,
      hitTestBehavior: PlatformViewHitTestBehavior.opaque,
      gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{},
    );
    layout(renderBox);
    pumpFrame(phase: EnginePhase.flushSemantics);

    expect(firstController.createdCallbacks, isNotEmpty);
    expect(firstController.createdCallbacks.length, 1);

    final AndroidViewController secondController = PlatformViewsService.initAndroidView(
      id: 0,
      viewType: 'webview',
      layoutDirection: TextDirection.rtl,
    );
    // Reset controller.
    renderBox.controller = secondController;

    expect(firstController.createdCallbacks, isEmpty);
    expect(secondController.createdCallbacks, isNotEmpty);
    expect(secondController.createdCallbacks.length, 1);
  });

  test('render object changed its visual appearance after texture is created', () {
    FakeAsync().run((FakeAsync async) {
      final AndroidViewController viewController = PlatformViewsService.initAndroidView(
        id: 0,
        viewType: 'webview',
        layoutDirection: TextDirection.rtl,
      );
      final RenderAndroidView renderBox = RenderAndroidView(
        viewController: viewController,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{},
      );

      final Completer<void> viewCreation = Completer<void>();
      const MethodChannel channel = MethodChannel('flutter/platform_views');
      binding.defaultBinaryMessenger.setMockMethodCallHandler(channel, (
        MethodCall methodCall,
      ) async {
        assert(methodCall.method == 'create', 'Unexpected method call');
        await viewCreation.future;
        return /*textureId=*/ 0;
      });

      layout(renderBox);
      pumpFrame(phase: EnginePhase.paint);

      expect(renderBox.debugLayer, isNotNull);
      expect(renderBox.debugLayer!.hasChildren, isFalse);
      expect(viewController.isCreated, isFalse);
      expect(renderBox.debugNeedsPaint, isFalse);

      viewCreation.complete();
      async.flushMicrotasks();

      expect(viewController.isCreated, isTrue);
      expect(renderBox.debugNeedsPaint, isTrue);
      expect(renderBox.debugLayer!.hasChildren, isFalse);

      pumpFrame(phase: EnginePhase.paint);
      expect(renderBox.debugLayer!.hasChildren, isTrue);
      expect(renderBox.debugLayer!.firstChild, isA<TextureLayer>());
    });
  });

  test('markNeedsPaint does not get called on a disposed RO', () async {
    FakeAsync().run((FakeAsync async) {
      final AndroidViewController viewController = PlatformViewsService.initAndroidView(
        id: 0,
        viewType: 'webview',
        layoutDirection: TextDirection.rtl,
      );
      final RenderAndroidView renderBox = RenderAndroidView(
        viewController: viewController,
        hitTestBehavior: PlatformViewHitTestBehavior.opaque,
        gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{},
      );

      final Completer<void> viewCreation = Completer<void>();
      const MethodChannel channel = MethodChannel('flutter/platform_views');
      binding.defaultBinaryMessenger.setMockMethodCallHandler(channel, (
        MethodCall methodCall,
      ) async {
        assert(methodCall.method == 'create', 'Unexpected method call');
        await viewCreation.future;
        return /*textureId=*/ 0;
      });

      layout(renderBox);
      pumpFrame(phase: EnginePhase.paint);

      expect(renderBox.debugLayer, isNotNull);
      expect(renderBox.debugLayer!.hasChildren, isFalse);
      expect(viewController.isCreated, isFalse);
      expect(renderBox.debugNeedsPaint, isFalse);

      renderBox.dispose();
      viewCreation.complete();
      async.flushMicrotasks();

      expect(viewController.isCreated, isTrue);
      expect(renderBox.debugNeedsPaint, isFalse);
      expect(renderBox.debugLayer, isNull);

      pumpFrame(phase: EnginePhase.paint);
      expect(renderBox.debugLayer, isNull);
    });
  });

  test('markNeedsPaint does not get called when setting the same viewController', () {
    FakeAsync().run((FakeAsync async) {
      final Completer<void> viewCreation = Completer<void>();
      const MethodChannel channel = MethodChannel('flutter/platform_views');
      binding.defaultBinaryMessenger.setMockMethodCallHandler(channel, (
        MethodCall methodCall,
      ) async {
        assert(methodCall.method == 'create', 'Unexpected method call');
        await viewCreation.future;
        return /*textureId=*/ 0;
      });

      bool futureCallbackRan = false;

      PlatformViewsService.initUiKitView(
        id: 0,
        viewType: 'webview',
        layoutDirection: TextDirection.ltr,
      ).then((UiKitViewController viewController) {
        final RenderUiKitView renderBox = RenderUiKitView(
          viewController: viewController,
          hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{},
        );

        layout(renderBox);
        pumpFrame(phase: EnginePhase.paint);
        expect(renderBox.debugNeedsPaint, isFalse);

        renderBox.viewController = viewController;

        expect(renderBox.debugNeedsPaint, isFalse);

        futureCallbackRan = true;
      });

      viewCreation.complete();
      async.flushMicrotasks();
      expect(futureCallbackRan, true);
    });
  });
}

ui.PointerData _pointerData(
  ui.PointerChange change,
  Offset logicalPosition, {
  int device = 0,
  PointerDeviceKind kind = PointerDeviceKind.mouse,
  int pointer = 0,
}) {
  final double devicePixelRatio =
      RendererBinding.instance.platformDispatcher.implicitView!.devicePixelRatio;
  return ui.PointerData(
    pointerIdentifier: pointer,
    embedderId: pointer,
    change: change,
    physicalX: logicalPosition.dx * devicePixelRatio,
    physicalY: logicalPosition.dy * devicePixelRatio,
    kind: kind,
    device: device,
  );
}
