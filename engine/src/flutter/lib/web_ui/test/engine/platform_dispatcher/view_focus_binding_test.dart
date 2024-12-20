// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group(ViewFocusBinding, () {
    late List<ui.ViewFocusEvent> dispatchedViewFocusEvents;
    late EnginePlatformDispatcher dispatcher;

    setUp(() {
      ViewFocusBinding.isEnabled = true;

      dispatcher = EnginePlatformDispatcher.instance;
      dispatchedViewFocusEvents = <ui.ViewFocusEvent>[];
      dispatcher.onViewFocusChange = dispatchedViewFocusEvents.add;
    });

    tearDown(() {
      ViewFocusBinding.isEnabled = false;
      EngineSemantics.instance.semanticsEnabled = false;
    });

    test('The view is focusable and reachable by keyboard when registered', () async {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      // The root element should have a tabindex="0" to make the flutter view
      // focusable and reachable by the keyboard.
      expect(view.dom.rootElement.getAttribute('tabindex'), '0');
    });

    test('The view is focusable but not reachable by keyboard when focused', () async {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      view.dom.rootElement.focusWithoutScroll();

      // The root element should have a tabindex="-1" to make the flutter view
      // focusable but not reachable by the keyboard.
      expect(view.dom.rootElement.getAttribute('tabindex'), '-1');
    });

    test('marks the focusable views as reachable by the keyboard or not', () async {
      final EngineFlutterView view1 = createAndRegisterView(dispatcher);
      final EngineFlutterView view2 = createAndRegisterView(dispatcher);

      expect(view1.dom.rootElement.getAttribute('tabindex'), '0');
      expect(view2.dom.rootElement.getAttribute('tabindex'), '0');

      view1.dom.rootElement.focusWithoutScroll();
      expect(view1.dom.rootElement.getAttribute('tabindex'), '-1');
      expect(view2.dom.rootElement.getAttribute('tabindex'), '0');

      view2.dom.rootElement.focusWithoutScroll();
      expect(view1.dom.rootElement.getAttribute('tabindex'), '0');
      expect(view2.dom.rootElement.getAttribute('tabindex'), '-1');

      view2.dom.rootElement.blur();
      expect(view1.dom.rootElement.getAttribute('tabindex'), '0');
      expect(view2.dom.rootElement.getAttribute('tabindex'), '0');
    });

    test('fires a focus event - a view was focused', () async {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      view.dom.rootElement.focusWithoutScroll();

      expect(dispatchedViewFocusEvents, hasLength(1));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);
    });

    test('fires a focus event - a view was unfocused', () async {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      view.dom.rootElement.focusWithoutScroll();
      view.dom.rootElement.blur();

      expect(dispatchedViewFocusEvents, hasLength(2));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[1].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[1].state, ui.ViewFocusState.unfocused);
      expect(dispatchedViewFocusEvents[1].direction, ui.ViewFocusDirection.undefined);
    });

    test('fires a focus event - focus transitions between views', () async {
      final EngineFlutterView view1 = createAndRegisterView(dispatcher);
      final EngineFlutterView view2 = createAndRegisterView(dispatcher);

      view1.dom.rootElement.focusWithoutScroll();
      view2.dom.rootElement.focusWithoutScroll();
      // The statements simulate the user pressing shift + tab in the keyboard.
      // Synthetic keyboard events do not trigger focus changes.
      domDocument.body!.pressTabKey(shift: true);
      view1.dom.rootElement.focusWithoutScroll();
      domDocument.body!.releaseTabKey();

      expect(dispatchedViewFocusEvents, hasLength(3));

      expect(dispatchedViewFocusEvents[0].viewId, view1.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[1].viewId, view2.viewId);
      expect(dispatchedViewFocusEvents[1].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[1].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[2].viewId, view1.viewId);
      expect(dispatchedViewFocusEvents[2].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[2].direction, ui.ViewFocusDirection.backward);
    });

    test('fires a focus event - focus transitions on and off views', () async {
      final EngineFlutterView view1 = createAndRegisterView(dispatcher);
      final EngineFlutterView view2 = createAndRegisterView(dispatcher);

      view1.dom.rootElement.focusWithoutScroll();
      view2.dom.rootElement.focusWithoutScroll();
      view2.dom.rootElement.blur();

      expect(dispatchedViewFocusEvents, hasLength(3));

      expect(dispatchedViewFocusEvents[0].viewId, view1.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[1].viewId, view2.viewId);
      expect(dispatchedViewFocusEvents[1].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[1].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[2].viewId, view2.viewId);
      expect(dispatchedViewFocusEvents[2].state, ui.ViewFocusState.unfocused);
      expect(dispatchedViewFocusEvents[2].direction, ui.ViewFocusDirection.undefined);
    });

    test('requestViewFocusChange focuses the view', () {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );

      expect(domDocument.activeElement, view.dom.rootElement);

      expect(dispatchedViewFocusEvents, hasLength(1));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);
    });

    test('requestViewFocusChange blurs the view', () {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );

      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.unfocused,
        direction: ui.ViewFocusDirection.undefined,
      );

      expect(domDocument.activeElement, isNot(view.dom.rootElement));

      expect(dispatchedViewFocusEvents, hasLength(2));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);

      expect(dispatchedViewFocusEvents[1].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[1].state, ui.ViewFocusState.unfocused);
      expect(dispatchedViewFocusEvents[1].direction, ui.ViewFocusDirection.undefined);
    });

    test('requestViewFocusChange does nothing if the view does not exist', () {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      dispatcher.requestViewFocusChange(
        viewId: 5094555,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );

      expect(domDocument.activeElement, isNot(view.dom.rootElement));
      expect(dispatchedViewFocusEvents, isEmpty);
    });

    test('requestViewFocusChange does nothing if the view is already focused', () {
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );
      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );

      expect(dispatchedViewFocusEvents, hasLength(1));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);
    });

    test('requestViewFocusChange does not move the focus to the view', () {
      final DomElement input = createDomElement('input');
      final EngineFlutterView view = createAndRegisterView(dispatcher);

      view.dom.rootElement.append(input);
      input.focusWithoutScroll();

      dispatcher.requestViewFocusChange(
        viewId: view.viewId,
        state: ui.ViewFocusState.focused,
        direction: ui.ViewFocusDirection.forward,
      );

      expect(domDocument.activeElement, input);

      expect(dispatchedViewFocusEvents, hasLength(1));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);
    });

    test('works even if focus is changed in the middle of a blur call', () {
      final DomElement input1 = createDomElement('input');
      final DomElement input2 = createDomElement('input');
      final EngineFlutterView view = createAndRegisterView(dispatcher);
      final DomEventListener focusInput1Listener = createDomEventListener((DomEvent event) {
        input1.focusWithoutScroll();
      });

      view.dom.rootElement.append(input1);
      view.dom.rootElement.append(input2);

      input1.addEventListener('blur', focusInput1Listener);
      input1.focusWithoutScroll();
      // The event handler above should move the focus back to input1.
      input2.focusWithoutScroll();
      input1.removeEventListener('blur', focusInput1Listener);

      expect(dispatchedViewFocusEvents, hasLength(1));

      expect(dispatchedViewFocusEvents[0].viewId, view.viewId);
      expect(dispatchedViewFocusEvents[0].state, ui.ViewFocusState.focused);
      expect(dispatchedViewFocusEvents[0].direction, ui.ViewFocusDirection.forward);
    });
  });
}

EngineFlutterView createAndRegisterView(EnginePlatformDispatcher dispatcher) {
  final DomElement div = createDomElement('div');
  final EngineFlutterView view = EngineFlutterView(dispatcher, div);
  domDocument.body!.append(div);
  dispatcher.viewManager.registerView(view);
  return view;
}

extension on DomElement {
  void pressTabKey({bool shift = false}) {
    dispatchKeyboardEvent(type: 'keydown', key: 'Tab', shiftKey: shift);
  }

  void releaseTabKey({bool shift = false}) {
    dispatchKeyboardEvent(type: 'keyup', key: 'Tab', shiftKey: shift);
  }

  void dispatchKeyboardEvent({required String type, required String key, bool shiftKey = false}) {
    dispatchEvent(createDomKeyboardEvent(type, <String, Object>{'key': key, 'shiftKey': shiftKey}));
  }
}
