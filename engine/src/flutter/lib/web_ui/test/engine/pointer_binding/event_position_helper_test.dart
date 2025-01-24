// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser')
library;

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui show Offset;

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  late EngineFlutterView view;
  late DomElement rootElement;
  late DomElement eventSource;
  final StreamController<DomEvent> events = StreamController<DomEvent>.broadcast();

  /// Dispatches an event `e` on `target`, and returns it after it's gone through the browser.
  Future<DomPointerEvent> dispatchAndCatch(DomElement target, DomPointerEvent e) async {
    final Future<DomEvent> nextEvent = events.stream.first;
    target.dispatchEvent(e);
    return (await nextEvent) as DomPointerEvent;
  }

  group('computeEventOffsetToTarget', () {
    setUp(() {
      view = EngineFlutterView(EnginePlatformDispatcher.instance, domDocument.body!);
      EnginePlatformDispatcher.instance.viewManager.registerView(view);
      rootElement = view.dom.rootElement;
      eventSource = createDomElement('div-event-source');
      rootElement.append(eventSource);

      // make containers known fixed sizes, absolutely positioned elements, so
      // we can reason about screen coordinates relatively easily later!
      rootElement.style
        ..position = 'absolute'
        ..width = '320px'
        ..height = '240px'
        ..top = '0px'
        ..left = '0px';

      eventSource.style
        ..position = 'absolute'
        ..width = '100px'
        ..height = '80px'
        ..top = '100px'
        ..left = '120px';

      rootElement.addEventListener(
        'click',
        createDomEventListener((DomEvent e) {
          events.add(e);
        }),
      );
    });

    tearDown(() {
      EnginePlatformDispatcher.instance.viewManager.unregisterView(view.viewId);
      view.dispose();
    });

    test('Event dispatched by target returns offsetX, offsetY', () async {
      // Fire an event contained within target...
      final DomMouseEvent event = await dispatchAndCatch(
        rootElement,
        createDomPointerEvent('click', <String, Object>{
          'bubbles': true,
          'clientX': 10,
          'clientY': 20,
        }),
      );

      expect(event.offsetX, 10);
      expect(event.offsetY, 20);

      final ui.Offset offset = computeEventOffsetToTarget(event, view);

      expect(offset.dx, event.offsetX);
      expect(offset.dy, event.offsetY);
    });

    test('Event dispatched on child re-computes offset (offsetX/Y invalid)', () async {
      // Fire an event contained within target...
      final DomMouseEvent event = await dispatchAndCatch(
        eventSource,
        createDomPointerEvent('click', <String, Object>{
          'bubbles': true, // So it can be caught in `target`
          'clientX': 140, // x = 20px into `eventSource`.
          'clientY': 110, // y = 10px into `eventSource`.
        }),
      );

      expect(event.offsetX, 20);
      expect(event.offsetY, 10);

      final ui.Offset offset = computeEventOffsetToTarget(event, view);

      expect(offset.dx, 140);
      expect(offset.dy, 110);
    });

    test('eventTarget takes precedence', () async {
      final input = view.dom.textEditingHost.appendChild(createDomElement('input'));

      textEditing.strategy.enable(
        InputConfiguration(viewId: view.viewId),
        onChange: (_, _) {},
        onAction: (_) {},
      );

      addTearDown(() {
        textEditing.strategy.disable();
      });

      final moveEvent = createDomPointerEvent('pointermove', <String, Object>{
        'bubbles': true,
        'clientX': 10,
        'clientY': 20,
      });

      expect(() => computeEventOffsetToTarget(moveEvent, view), throwsA(anything));

      expect(
        () => computeEventOffsetToTarget(moveEvent, view, eventTarget: input),
        returnsNormally,
      );
    });

    test('Event dispatched by TalkBack gets a computed offset', () async {
      // Fill this in to test _computeOffsetForTalkbackEvent

      // To be implemented!
    }, skip: true);

    test(
      'Event dispatched on text editing node computes offset with framework geometry',
      () async {
        // Fill this in to test _computeOffsetForInputs
      },
      // To be implemented!
      skip: true,
    );
  });
}
