// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_util' as js_util;

import 'package:meta/meta.dart';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'keyboard_converter_test.dart';

const int _kNoButtonChange = -1;

List<ui.PointerData> _allPointerData(List<ui.PointerDataPacket> packets) {
  return packets.expand((ui.PointerDataPacket packet) => packet.data).toList();
}

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

late EngineFlutterView view;
DomElement get rootElement => view.dom.rootElement;

void testMain() {
  final DomElement hostElement = createDomHTMLDivElement();

  setUpAll(() {
    domDocument.body!.append(hostElement);
    // Remove <body> margins to avoid messing up with all the test coordinates.
    domDocument.body!.style.margin = '0';
  });
  tearDownAll(() {
    hostElement.remove();
  });

  late List<ui.KeyData> keyDataList;
  late KeyboardConverter keyboardConverter;
  late PointerBinding instance;
  late double dpi;

  KeyboardConverter createKeyboardConverter(List<ui.KeyData> keyDataList) {
    return KeyboardConverter((ui.KeyData key) {
      keyDataList.add(key);
      return true;
    }, ui_web.OperatingSystem.linux);
  }

  setUp(() {
    keyDataList = <ui.KeyData>[];
    keyboardConverter = createKeyboardConverter(keyDataList);

    view = EngineFlutterView(EnginePlatformDispatcher.instance, hostElement);
    instance = view.pointerBinding;
    instance.debugOverrideKeyboardConverter(keyboardConverter);

    ui.PlatformDispatcher.instance.onPointerDataPacket = null;
    dpi = EngineFlutterDisplay.instance.devicePixelRatio;
  });

  tearDown(() {
    keyboardConverter.dispose();
    view.dispose();
    PointerBinding.debugResetGlobalState();
  });

  test('ios workaround', () {
    debugEmulateIosSafari = true;
    addTearDown(() {
      debugEmulateIosSafari = false;
    });

    final MockSafariPointerEventWorkaround mockSafariWorkaround =
        MockSafariPointerEventWorkaround();
    final PointerBinding instance = PointerBinding(view, safariWorkaround: mockSafariWorkaround);
    expect(mockSafariWorkaround.workAroundInvoked, isIosSafari);
    instance.dispose();
  }, skip: !isSafari);

  test('_PointerEventContext generates expected events', () {
    DomPointerEvent expectCorrectType(DomEvent e) {
      expect(domInstanceOfString(e, 'PointerEvent'), isTrue);
      return e as DomPointerEvent;
    }

    List<DomPointerEvent> expectCorrectTypes(List<DomEvent> events) {
      return events.map(expectCorrectType).toList();
    }

    final _PointerEventContext context = _PointerEventContext();
    DomPointerEvent event;
    List<DomPointerEvent> events;

    event = expectCorrectType(context.primaryDown(clientX: 100, clientY: 101));
    expect(event.type, equals('pointerdown'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(0));
    expect(event.buttons, equals(1));
    expect(event.client.x, equals(100));
    expect(event.client.y, equals(101));

    event = expectCorrectType(context.mouseDown(clientX: 110, clientY: 111, button: 2, buttons: 2));
    expect(event.type, equals('pointerdown'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(2));
    expect(event.buttons, equals(2));
    expect(event.client.x, equals(110));
    expect(event.client.y, equals(111));

    events = expectCorrectTypes(
      context.multiTouchDown(const <_TouchDetails>[
        _TouchDetails(pointer: 100, clientX: 120, clientY: 121),
        _TouchDetails(pointer: 101, clientX: 122, clientY: 123),
      ]),
    );
    expect(events.length, equals(2));
    expect(events[0].type, equals('pointerdown'));
    expect(events[0].pointerId, equals(100));
    expect(events[0].button, equals(0));
    expect(events[0].buttons, equals(1));
    expect(events[0].client.x, equals(120));
    expect(events[0].client.y, equals(121));
    expect(events[1].type, equals('pointerdown'));
    expect(events[1].pointerId, equals(101));
    expect(events[1].button, equals(0));
    expect(events[1].buttons, equals(1));
    expect(events[1].client.x, equals(122));
    expect(events[1].client.y, equals(123));

    event = expectCorrectType(context.primaryMove(clientX: 200, clientY: 201));
    expect(event.type, equals('pointermove'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(-1));
    expect(event.buttons, equals(1));
    expect(event.client.x, equals(200));
    expect(event.client.y, equals(201));

    event = expectCorrectType(
      context.mouseMove(clientX: 210, clientY: 211, button: _kNoButtonChange, buttons: 6),
    );
    expect(event.type, equals('pointermove'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(-1));
    expect(event.buttons, equals(6));
    expect(event.client.x, equals(210));
    expect(event.client.y, equals(211));

    event = expectCorrectType(context.mouseMove(clientX: 212, clientY: 213, button: 2, buttons: 6));
    expect(event.type, equals('pointermove'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(2));
    expect(event.buttons, equals(6));
    expect(event.client.x, equals(212));
    expect(event.client.y, equals(213));

    event = expectCorrectType(context.mouseMove(clientX: 214, clientY: 215, button: 2, buttons: 1));
    expect(event.type, equals('pointermove'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(2));
    expect(event.buttons, equals(1));
    expect(event.client.x, equals(214));
    expect(event.client.y, equals(215));

    events = expectCorrectTypes(
      context.multiTouchMove(const <_TouchDetails>[
        _TouchDetails(pointer: 102, clientX: 220, clientY: 221),
        _TouchDetails(pointer: 103, clientX: 222, clientY: 223),
      ]),
    );
    expect(events.length, equals(2));
    expect(events[0].type, equals('pointermove'));
    expect(events[0].pointerId, equals(102));
    expect(events[0].button, equals(-1));
    expect(events[0].buttons, equals(1));
    expect(events[0].client.x, equals(220));
    expect(events[0].client.y, equals(221));
    expect(events[1].type, equals('pointermove'));
    expect(events[1].pointerId, equals(103));
    expect(events[1].button, equals(-1));
    expect(events[1].buttons, equals(1));
    expect(events[1].client.x, equals(222));
    expect(events[1].client.y, equals(223));

    event = expectCorrectType(context.mouseLeave(clientX: 1000, clientY: 2000, buttons: 6));
    expect(event.type, equals('pointerleave'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(0));
    expect(event.buttons, equals(6));
    expect(event.client.x, equals(1000));
    expect(event.client.y, equals(2000));

    event = expectCorrectType(context.primaryUp(clientX: 300, clientY: 301));
    expect(event.type, equals('pointerup'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(0));
    expect(event.buttons, equals(0));
    expect(event.client.x, equals(300));
    expect(event.client.y, equals(301));

    event = expectCorrectType(context.mouseUp(clientX: 310, clientY: 311, button: 2));
    expect(event.type, equals('pointerup'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(2));
    expect(event.buttons, equals(0));
    expect(event.client.x, equals(310));
    expect(event.client.y, equals(311));

    events = expectCorrectTypes(
      context.multiTouchUp(const <_TouchDetails>[
        _TouchDetails(pointer: 104, clientX: 320, clientY: 321),
        _TouchDetails(pointer: 105, clientX: 322, clientY: 323),
      ]),
    );
    expect(events.length, equals(2));
    expect(events[0].type, equals('pointerup'));
    expect(events[0].pointerId, equals(104));
    expect(events[0].button, equals(0));
    expect(events[0].buttons, equals(0));
    expect(events[0].client.x, equals(320));
    expect(events[0].client.y, equals(321));
    expect(events[1].type, equals('pointerup'));
    expect(events[1].pointerId, equals(105));
    expect(events[1].button, equals(0));
    expect(events[1].buttons, equals(0));
    expect(events[1].client.x, equals(322));
    expect(events[1].client.y, equals(323));

    event = expectCorrectType(context.hover(clientX: 400, clientY: 401));
    expect(event.type, equals('pointermove'));
    expect(event.pointerId, equals(1));
    expect(event.button, equals(-1));
    expect(event.buttons, equals(0));
    expect(event.client.x, equals(400));
    expect(event.client.y, equals(401));

    events = expectCorrectTypes(
      context.multiTouchCancel(const <_TouchDetails>[
        _TouchDetails(pointer: 106, clientX: 500, clientY: 501),
        _TouchDetails(pointer: 107, clientX: 502, clientY: 503),
      ]),
    );
    expect(events.length, equals(2));
    expect(events[0].type, equals('pointercancel'));
    expect(events[0].pointerId, equals(106));
    expect(events[0].button, equals(0));
    expect(events[0].buttons, equals(0));
    expect(events[0].client.x, equals(0));
    expect(events[0].client.y, equals(0));
    expect(events[1].type, equals('pointercancel'));
    expect(events[1].pointerId, equals(107));
    expect(events[1].button, equals(0));
    expect(events[1].buttons, equals(0));
    expect(events[1].client.x, equals(0));
    expect(events[1].client.y, equals(0));

    context.pressAllModifiers();
    event = expectCorrectType(context.primaryDown(clientX: 100, clientY: 101));
    expect(event.getModifierState('Alt'), true);
    expect(event.getModifierState('Control'), true);
    expect(event.getModifierState('Meta'), true);
    expect(event.getModifierState('Shift'), true);
    context.unpressAllModifiers();
    event = expectCorrectType(context.primaryDown(clientX: 100, clientY: 101));
    expect(event.getModifierState('Alt'), false);
    expect(event.getModifierState('Control'), false);
    expect(event.getModifierState('Meta'), false);
    expect(event.getModifierState('Shift'), false);
  });

  // The reason we listen for pointer events in the bubble phase instead of the
  // capture phase is to allow platform views and native text fields to receive
  // the event first. This way, they can potentially handle the event and stop
  // its propagation to prevent Flutter from receiving and handling it.
  test('event listeners are attached to the bubble phase', () {
    final _BasicEventContext context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    final DomElement child = createDomHTMLDivElement();
    rootElement.append(child);

    final DomEventListener stopPropagationListener = createDomEventListener((DomEvent event) {
      event.stopPropagation();
    });

    // The event reaches `PointerBinding` as expected.
    child.dispatchEvent(context.primaryDown());
    expect(packets, isNotEmpty);
    packets.clear();

    // The child stops propagation so the event doesn't reach `PointerBinding`.
    final DomEvent event = context.primaryDown();
    child.addEventListener(event.type, stopPropagationListener);
    child.dispatchEvent(event);
    expect(packets, isEmpty);
    packets.clear();

    child.remove();
  });

  test('allows default on touchstart events', () async {
    final event = createDomEvent('Event', 'touchstart');

    rootElement.dispatchEvent(event);

    expect(
      event.defaultPrevented,
      isFalse,
      reason: 'touchstart events should NOT be prevented. That breaks semantic taps!',
    );
  });

  test('can receive pointer events on the app root', () {
    final _BasicEventContext context = _PointerEventContext();
    ui.PointerDataPacket? receivedPacket;
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      receivedPacket = packet;
    };

    rootElement.dispatchEvent(context.primaryDown());

    expect(receivedPacket, isNotNull);
    expect(receivedPacket!.data[0].buttons, equals(1));
  });

  test('does create an add event if got a pointerdown', () {
    final _BasicEventContext context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(context.primaryDown());

    expect(packets, hasLength(1));
    expect(packets.single.data, hasLength(2));

    expect(packets.single.data[0].change, equals(ui.PointerChange.add));
    expect(packets.single.data[1].change, equals(ui.PointerChange.down));
  });

  test('synthesize modifier keys left down event if left or right are not pressed', () {
    final _BasicEventContext context = _PointerEventContext();

    // Should synthesize a modifier left key down event when DOM event indicates
    // that the modifier key is pressed and known pressing state doesn't contain
    // the modifier left key nor the modifier right key.
    void shouldSynthesizeLeftDownIfNotPressed(String key) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;
      final int logicalLeft = kWebLogicalLocationMap[key]![kLocationLeft]!;

      expect(keyboardConverter.keyIsPressed(physicalLeft), false);
      expect(keyboardConverter.keyIsPressed(physicalRight), false);
      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 1);
      expectKeyData(
        keyDataList.last,
        type: ui.KeyEventType.down,
        deviceType: ui.KeyEventDeviceType.keyboard,
        physical: physicalLeft,
        logical: logicalLeft,
        character: null,
        synthesized: true,
      );
      keyDataList.clear();
      keyboardConverter.clearPressedKeys();
    }

    context.altPressed = true;
    shouldSynthesizeLeftDownIfNotPressed('Alt');
    context.unpressAllModifiers();
    context.ctrlPressed = true;
    shouldSynthesizeLeftDownIfNotPressed('Control');
    context.unpressAllModifiers();
    context.metaPressed = true;
    shouldSynthesizeLeftDownIfNotPressed('Meta');
    context.unpressAllModifiers();
    context.shiftPressed = true;
    shouldSynthesizeLeftDownIfNotPressed('Shift');
    context.unpressAllModifiers();
  });

  test('should not synthesize modifier keys down event if left or right are pressed', () {
    final _BasicEventContext context = _PointerEventContext();

    // Should not synthesize a modifier down event when DOM event indicates
    // that the modifier key is pressed and known pressing state contains
    // the modifier left key.
    void shouldNotSynthesizeDownIfLeftPressed(String key, int modifiers) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;

      keyboardConverter.handleEvent(keyDownEvent('${key}Left', key, modifiers, kLocationLeft));
      expect(keyboardConverter.keyIsPressed(physicalLeft), true);
      expect(keyboardConverter.keyIsPressed(physicalRight), false);
      keyDataList.clear(); // Remove key data generated by handleEvent

      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 0);
      keyboardConverter.clearPressedKeys();
    }

    // Should not synthesize a modifier down event when DOM event indicates
    // that the modifier key is pressed and known pressing state contains
    // the modifier right key.
    void shouldNotSynthesizeDownIfRightPressed(String key, int modifiers) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;

      keyboardConverter.handleEvent(keyDownEvent('${key}Right', key, modifiers, kLocationRight));
      expect(keyboardConverter.keyIsPressed(physicalLeft), false);
      expect(keyboardConverter.keyIsPressed(physicalRight), true);
      keyDataList.clear(); // Remove key data generated by handleEvent

      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 0);
      keyboardConverter.clearPressedKeys();
    }

    context.altPressed = true;
    shouldNotSynthesizeDownIfLeftPressed('Alt', kAlt);
    shouldNotSynthesizeDownIfRightPressed('Alt', kAlt);
    context.unpressAllModifiers();
    context.ctrlPressed = true;
    shouldNotSynthesizeDownIfLeftPressed('Control', kCtrl);
    shouldNotSynthesizeDownIfRightPressed('Control', kCtrl);
    context.unpressAllModifiers();
    context.metaPressed = true;
    shouldNotSynthesizeDownIfLeftPressed('Meta', kMeta);
    shouldNotSynthesizeDownIfRightPressed('Meta', kMeta);
    context.unpressAllModifiers();
    context.shiftPressed = true;
    shouldNotSynthesizeDownIfLeftPressed('Shift', kShift);
    shouldNotSynthesizeDownIfRightPressed('Shift', kShift);
    context.unpressAllModifiers();
  });

  test('synthesize modifier keys up event if left or right are pressed', () {
    final _BasicEventContext context = _PointerEventContext();

    // Should synthesize a modifier left key up event when DOM event indicates
    // that the modifier key is not pressed and known pressing state contains
    // the modifier left key.
    void shouldSynthesizeLeftUpIfLeftPressed(String key, int modifiers) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;
      final int logicalLeft = kWebLogicalLocationMap[key]![kLocationLeft]!;

      keyboardConverter.handleEvent(keyDownEvent('${key}Left', key, modifiers, kLocationLeft));
      expect(keyboardConverter.keyIsPressed(physicalLeft), true);
      expect(keyboardConverter.keyIsPressed(physicalRight), false);
      keyDataList.clear(); // Remove key data generated by handleEvent

      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 1);
      expectKeyData(
        keyDataList.last,
        type: ui.KeyEventType.up,
        deviceType: ui.KeyEventDeviceType.keyboard,
        physical: physicalLeft,
        logical: logicalLeft,
        character: null,
        synthesized: true,
      );
      expect(keyboardConverter.keyIsPressed(physicalLeft), false);
      keyboardConverter.clearPressedKeys();
    }

    // Should synthesize a modifier right key up event when DOM event indicates
    // that the modifier key is not pressed and known pressing state contains
    // the modifier right key.
    void shouldSynthesizeRightUpIfRightPressed(String key, int modifiers) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;
      final int logicalRight = kWebLogicalLocationMap[key]![kLocationRight]!;

      keyboardConverter.handleEvent(keyDownEvent('${key}Right', key, modifiers, kLocationRight));
      expect(keyboardConverter.keyIsPressed(physicalLeft), false);
      expect(keyboardConverter.keyIsPressed(physicalRight), true);
      keyDataList.clear(); // Remove key data generated by handleEvent

      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 1);
      expectKeyData(
        keyDataList.last,
        type: ui.KeyEventType.up,
        deviceType: ui.KeyEventDeviceType.keyboard,
        physical: physicalRight,
        logical: logicalRight,
        character: null,
        synthesized: true,
      );
      expect(keyboardConverter.keyIsPressed(physicalRight), false);
      keyboardConverter.clearPressedKeys();
    }

    context.altPressed = false;
    shouldSynthesizeLeftUpIfLeftPressed('Alt', kAlt);
    shouldSynthesizeRightUpIfRightPressed('Alt', kAlt);
    context.ctrlPressed = false;
    shouldSynthesizeLeftUpIfLeftPressed('Control', kCtrl);
    shouldSynthesizeRightUpIfRightPressed('Control', kCtrl);
    context.metaPressed = false;
    shouldSynthesizeLeftUpIfLeftPressed('Meta', kMeta);
    shouldSynthesizeRightUpIfRightPressed('Meta', kMeta);
    context.shiftPressed = false;
    shouldSynthesizeLeftUpIfLeftPressed('Shift', kShift);
    shouldSynthesizeRightUpIfRightPressed('Shift', kShift);
  });

  test('should not synthesize modifier keys up event if left or right are not pressed', () {
    final _BasicEventContext context = _PointerEventContext();

    // Should not synthesize a modifier up event when DOM event indicates
    // that the modifier key is not pressed and known pressing state does
    // not contain the modifier left key nor the modifier right key.
    void shouldNotSynthesizeUpIfNotPressed(String key) {
      final int physicalLeft = kWebToPhysicalKey['${key}Left']!;
      final int physicalRight = kWebToPhysicalKey['${key}Right']!;

      expect(keyboardConverter.keyIsPressed(physicalLeft), false);
      expect(keyboardConverter.keyIsPressed(physicalRight), false);
      keyDataList.clear(); // Remove key data generated by handleEvent

      rootElement.dispatchEvent(context.primaryDown());
      expect(keyDataList.length, 0);
      keyboardConverter.clearPressedKeys();
    }

    context.altPressed = false;
    shouldNotSynthesizeUpIfNotPressed('Alt');
    context.ctrlPressed = false;
    shouldNotSynthesizeUpIfNotPressed('Control');
    context.metaPressed = false;
    shouldNotSynthesizeUpIfNotPressed('Meta');
    context.shiftPressed = false;
    shouldNotSynthesizeUpIfNotPressed('Shift');
  });

  test('should synthesize modifier keys up event for AltGraph', () {
    final _BasicEventContext context = _PointerEventContext();

    final int physicalAltRight = kWebToPhysicalKey['AltRight']!;
    final int logicalAltGraph = kWebLogicalLocationMap['AltGraph']![0]!;

    // Simulate pressing `AltGr` key.
    keyboardConverter.handleEvent(keyDownEvent('AltRight', 'AltGraph'));
    expect(keyboardConverter.keyIsPressed(physicalAltRight), true);
    keyDataList.clear(); // Remove key data generated by handleEvent.

    rootElement.dispatchEvent(context.primaryDown());
    expect(keyDataList.length, 1);
    expectKeyData(
      keyDataList.last,
      type: ui.KeyEventType.up,
      deviceType: ui.KeyEventDeviceType.keyboard,
      physical: physicalAltRight,
      logical: logicalAltGraph,
      character: null,
      synthesized: true,
    );
    expect(keyboardConverter.keyIsPressed(physicalAltRight), false);
    keyDataList.clear();
  });

  test(
    'correctly detects events on the semantics placeholder',
    () {
      final _ButtonedEventMixin context = _PointerEventContext();
      final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
      ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
        packets.add(packet);
      };

      final DomElement semanticsPlaceholder = createDomElement('flt-semantics-placeholder');
      rootElement.append(semanticsPlaceholder);

      // Press on the semantics placeholder.
      semanticsPlaceholder.dispatchEvent(context.primaryDown(clientX: 10.0, clientY: 10.0));
      expect(packets, hasLength(1));
      expect(packets[0].data, hasLength(2));
      expect(packets[0].data[0].change, equals(ui.PointerChange.add));
      expect(packets[0].data[1].change, equals(ui.PointerChange.down));
      expect(packets[0].data[1].physicalX, equals(10.0 * dpi));
      expect(packets[0].data[1].physicalY, equals(10.0 * dpi));
      packets.clear();

      // Drag on the semantics placeholder.
      semanticsPlaceholder.dispatchEvent(context.primaryMove(clientX: 12.0, clientY: 10.0));
      expect(packets, hasLength(1));
      expect(packets[0].data, hasLength(1));
      expect(packets[0].data[0].change, equals(ui.PointerChange.move));
      expect(packets[0].data[0].physicalX, equals(12.0 * dpi));
      expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
      packets.clear();

      // Keep dragging.
      semanticsPlaceholder.dispatchEvent(context.primaryMove(clientX: 15.0, clientY: 10.0));
      expect(packets[0].data, hasLength(1));
      expect(packets[0].data[0].change, equals(ui.PointerChange.move));
      expect(packets[0].data[0].physicalX, equals(15.0 * dpi));
      expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
      packets.clear();

      // Release the pointer on the semantics placeholder.
      rootElement.dispatchEvent(context.primaryUp(clientX: 100.0, clientY: 200.0));
      expect(packets, hasLength(1));
      expect(packets[0].data, hasLength(2));
      expect(packets[0].data[0].change, equals(ui.PointerChange.move));
      expect(packets[0].data[0].physicalX, equals(100.0 * dpi));
      expect(packets[0].data[0].physicalY, equals(200.0 * dpi));
      expect(packets[0].data[1].change, equals(ui.PointerChange.up));
      expect(packets[0].data[1].physicalX, equals(100.0 * dpi));
      expect(packets[0].data[1].physicalY, equals(200.0 * dpi));
      packets.clear();

      semanticsPlaceholder.remove();
    },
    skip: isFirefox, // https://bugzilla.mozilla.org/show_bug.cgi?id=1804190
  );

  // BUTTONED ADAPTERS

  test('creates an add event if the first pointer activity is a hover', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(context.hover());

    expect(packets, hasLength(1));
    expect(packets.single.data, hasLength(2));

    expect(packets.single.data[0].change, equals(ui.PointerChange.add));
    expect(packets.single.data[0].synthesized, isTrue);
    expect(packets.single.data[1].change, equals(ui.PointerChange.hover));
  });

  test('sends a pointermove event instead of the second pointerdown in a row', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(context.primaryDown(clientX: 10.0, clientY: 10.0));
    expect(packets, hasLength(1));
    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    packets.clear();

    rootElement.dispatchEvent(context.primaryDown(clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].buttons, equals(1));
    packets.clear();
  });

  test('wheel event - preventDefault called', () {
    // Synthesize a 'wheel' event.
    final DomEvent event = _PointerEventContext().wheel(
      buttons: 0,
      clientX: 10,
      clientY: 10,
      deltaX: 10,
      deltaY: 0,
    );
    rootElement.dispatchEvent(event);
    // Check that the engine called `preventDefault` on the event.
    expect(event.defaultPrevented, isTrue);
  });

  test('wheel event - framework can stop preventDefault (allowPlatformDefault)', () {
    // The framework calls `data.respond(allowPlatformDefault: true)`
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packet.data
          .where((ui.PointerData datum) => datum.signalKind == ui.PointerSignalKind.scroll)
          .forEach((ui.PointerData datum) {
            datum.respond(allowPlatformDefault: true);
          });
    };

    // Synthesize a 'wheel' event.
    final DomEvent event = _PointerEventContext().wheel(
      buttons: 0,
      clientX: 10,
      clientY: 10,
      deltaX: 10,
      deltaY: 0,
    );
    rootElement.dispatchEvent(event);

    // Check that the engine did NOT call `preventDefault` on the event.
    expect(event.defaultPrevented, isFalse);
  });

  test('wheel event - once allowPlatformDefault is set to true, it cannot be rolled back', () {
    // The framework calls `data.respond(allowPlatformDefault: true)`
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packet.data
          .where((ui.PointerData datum) => datum.signalKind == ui.PointerSignalKind.scroll)
          .forEach((ui.PointerData datum) {
            datum.respond(allowPlatformDefault: false);
            datum.respond(allowPlatformDefault: true);
            datum.respond(allowPlatformDefault: false);
          });
    };

    // Synthesize a 'wheel' event.
    final DomEvent event = _PointerEventContext().wheel(
      buttons: 0,
      clientX: 10,
      clientY: 10,
      deltaX: 10,
      deltaY: 0,
    );
    rootElement.dispatchEvent(event);

    // Check that the engine did NOT call `preventDefault` on the event.
    expect(event.defaultPrevented, isFalse);
  });

  test('does synthesize add or hover or move for scroll', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 10, clientY: 10, deltaX: 10, deltaY: 10),
    );

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 20, clientY: 50, deltaX: 10, deltaY: 10),
    );

    rootElement.dispatchEvent(
      context.mouseDown(button: 0, buttons: 1, clientX: 20.0, clientY: 50.0),
    );

    rootElement.dispatchEvent(
      context.wheel(buttons: 1, clientX: 30, clientY: 60, deltaX: 10, deltaY: 10),
    );

    expect(packets, hasLength(4));

    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].pointerIdentifier, equals(0));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[0].data[1].pointerIdentifier, equals(0));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0.0));
    expect(packets[0].data[1].physicalDeltaY, equals(0.0));

    // A hover will be synthesized.
    expect(packets[1].data, hasLength(2));
    expect(packets[1].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[1].data[0].pointerIdentifier, equals(0));
    expect(packets[1].data[0].synthesized, isTrue);
    expect(packets[1].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[1].data[0].physicalY, equals(50.0 * dpi));
    expect(packets[1].data[0].physicalDeltaX, equals(10.0 * dpi));
    expect(packets[1].data[0].physicalDeltaY, equals(40.0 * dpi));

    expect(packets[1].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[1].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[1].data[1].pointerIdentifier, equals(0));
    expect(packets[1].data[1].synthesized, isFalse);
    expect(packets[1].data[1].physicalX, equals(20.0 * dpi));
    expect(packets[1].data[1].physicalY, equals(50.0 * dpi));
    expect(packets[1].data[1].physicalDeltaX, equals(0.0));
    expect(packets[1].data[1].physicalDeltaY, equals(0.0));

    // No synthetic pointer data for down event.
    expect(packets[2].data, hasLength(1));
    expect(packets[2].data[0].change, equals(ui.PointerChange.down));
    expect(packets[2].data[0].signalKind, equals(ui.PointerSignalKind.none));
    expect(packets[2].data[0].pointerIdentifier, equals(1));
    expect(packets[2].data[0].synthesized, isFalse);
    expect(packets[2].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[2].data[0].physicalY, equals(50.0 * dpi));
    expect(packets[2].data[0].physicalDeltaX, equals(0.0));
    expect(packets[2].data[0].physicalDeltaY, equals(0.0));

    // A move will be synthesized instead of hover because the button is currently down.
    expect(packets[3].data, hasLength(2));
    expect(packets[3].data[0].change, equals(ui.PointerChange.move));
    expect(packets[3].data[0].pointerIdentifier, equals(1));
    expect(packets[3].data[0].synthesized, isTrue);
    expect(packets[3].data[0].physicalX, equals(30.0 * dpi));
    expect(packets[3].data[0].physicalY, equals(60.0 * dpi));
    expect(packets[3].data[0].physicalDeltaX, equals(10.0 * dpi));
    expect(packets[3].data[0].physicalDeltaY, equals(10.0 * dpi));

    expect(packets[3].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[3].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[3].data[1].pointerIdentifier, equals(1));
    expect(packets[3].data[1].synthesized, isFalse);
    expect(packets[3].data[1].physicalX, equals(30.0 * dpi));
    expect(packets[3].data[1].physicalY, equals(60.0 * dpi));
    expect(packets[3].data[1].physicalDeltaX, equals(0.0));
    expect(packets[3].data[1].physicalDeltaY, equals(0.0));
  });

  test('converts scroll delta to physical pixels (macOs)', () {
    final _ButtonedEventMixin context = _PointerEventContext();

    const double dpi = 2.5;
    ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.macOs;
    EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(dpi);

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 10, clientY: 10, deltaX: 10, deltaY: 10),
    );

    expect(packets, hasLength(1));

    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    // Scroll deltas should be multiplied by `dpi`.
    expect(packets[0].data[0].scrollDeltaX, equals(10.0 * dpi));
    expect(packets[0].data[0].scrollDeltaY, equals(10.0 * dpi));

    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    // Scroll deltas should be multiplied by `dpi`.
    expect(packets[0].data[0].scrollDeltaX, equals(10.0 * dpi));
    expect(packets[0].data[0].scrollDeltaY, equals(10.0 * dpi));

    EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
    ui_web.browser.debugBrowserEngineOverride = null;
  });

  test('does set pointer device kind based on delta precision and wheelDelta', () {
    if (isFirefox) {
      // Firefox does not support trackpad events, as they cannot be
      // disambiguated from smoothed mouse wheel events.
      return;
    }
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 119,
        deltaY: 119,
        wheelDeltaX: -357,
        wheelDeltaY: -357,
        timeStamp: 0,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 120,
        deltaY: 120,
        wheelDeltaX: -360,
        wheelDeltaY: -360,
        timeStamp: 10,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 120,
        deltaY: 120,
        wheelDeltaX: -360,
        wheelDeltaY: -360,
        timeStamp: 20,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 119,
        deltaY: 119,
        wheelDeltaX: -357,
        wheelDeltaY: -357,
        timeStamp: 1000,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: -120,
        deltaY: -120,
        wheelDeltaX: 360,
        wheelDeltaY: 360,
        timeStamp: 1010,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 0,
        deltaY: -120,
        wheelDeltaX: 0,
        wheelDeltaY: 360,
        timeStamp: 2000,
      ),
    );

    rootElement.dispatchEvent(
      context.wheel(
        buttons: 0,
        clientX: 10,
        clientY: 10,
        deltaX: 0,
        deltaY: 40,
        wheelDeltaX: 0,
        wheelDeltaY: -360,
        timeStamp: 3000,
      ),
    );

    expect(packets, hasLength(7));

    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].pointerIdentifier, equals(0));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));
    // Because the delta is not in increments of 120 and has matching wheelDelta,
    // it will be a trackpad event.
    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[0].data[1].kind, equals(ui.PointerDeviceKind.trackpad));
    expect(packets[0].data[1].device, equals(-2));
    expect(packets[0].data[1].pointerIdentifier, equals(0));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0.0));
    expect(packets[0].data[1].physicalDeltaY, equals(0.0));
    expect(packets[0].data[1].scrollDeltaX, equals(119.0));
    expect(packets[0].data[1].scrollDeltaY, equals(119.0));

    // Because the delta is in increments of 120, but is similar to the
    // previous event, it will be a trackpad event.
    expect(packets[1].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[1].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[1].data[0].kind, equals(ui.PointerDeviceKind.trackpad));
    expect(packets[1].data[0].device, equals(-2));
    expect(packets[1].data[0].pointerIdentifier, equals(0));
    expect(packets[1].data[0].synthesized, isFalse);
    expect(packets[1].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[1].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[1].data[0].physicalDeltaX, equals(0.0));
    expect(packets[1].data[0].physicalDeltaY, equals(0.0));
    expect(packets[1].data[0].scrollDeltaX, equals(120.0));
    expect(packets[1].data[0].scrollDeltaY, equals(120.0));

    // Because the delta is in increments of 120, but is again similar to the
    // previous event, it will be a trackpad event.
    expect(packets[2].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[2].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[2].data[0].kind, equals(ui.PointerDeviceKind.trackpad));
    expect(packets[2].data[0].device, equals(-2));
    expect(packets[2].data[0].pointerIdentifier, equals(0));
    expect(packets[2].data[0].synthesized, isFalse);
    expect(packets[2].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[2].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[2].data[0].physicalDeltaX, equals(0.0));
    expect(packets[2].data[0].physicalDeltaY, equals(0.0));
    expect(packets[2].data[0].scrollDeltaX, equals(120.0));
    expect(packets[2].data[0].scrollDeltaY, equals(120.0));

    // Because the delta is not in increments of 120 and has matching wheelDelta,
    // it will be a trackpad event.
    expect(packets[3].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[3].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[3].data[0].kind, equals(ui.PointerDeviceKind.trackpad));
    expect(packets[3].data[0].device, equals(-2));
    expect(packets[3].data[0].pointerIdentifier, equals(0));
    expect(packets[3].data[0].synthesized, isFalse);
    expect(packets[3].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[3].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[3].data[0].physicalDeltaX, equals(0.0));
    expect(packets[3].data[0].physicalDeltaY, equals(0.0));
    expect(packets[3].data[0].scrollDeltaX, equals(119.0));
    expect(packets[3].data[0].scrollDeltaY, equals(119.0));

    // Because the delta is in increments of 120, and is not similar to the
    // previous event, but occurred soon after the previous event, it will be
    // a trackpad event.
    expect(packets[4].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[4].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[4].data[0].kind, equals(ui.PointerDeviceKind.trackpad));
    expect(packets[4].data[0].device, equals(-2));
    expect(packets[4].data[0].pointerIdentifier, equals(0));
    expect(packets[4].data[0].synthesized, isFalse);
    expect(packets[4].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[4].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[4].data[0].physicalDeltaX, equals(0.0));
    expect(packets[4].data[0].physicalDeltaY, equals(0.0));
    expect(packets[4].data[0].scrollDeltaX, equals(-120.0));
    expect(packets[4].data[0].scrollDeltaY, equals(-120.0));

    // An add will be synthesized.
    expect(packets[5].data, hasLength(2));
    expect(packets[5].data[0].change, equals(ui.PointerChange.add));
    expect(packets[5].data[0].signalKind, equals(ui.PointerSignalKind.none));
    expect(packets[5].data[0].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[5].data[0].device, equals(-1));
    expect(packets[5].data[0].pointerIdentifier, equals(0));
    expect(packets[5].data[0].synthesized, isTrue);
    expect(packets[5].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[5].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[5].data[0].physicalDeltaX, equals(0.0));
    expect(packets[5].data[0].physicalDeltaY, equals(0.0));
    expect(packets[5].data[0].scrollDeltaX, equals(0.0));
    expect(packets[5].data[0].scrollDeltaY, equals(-120.0));
    // Because the delta is in increments of 120, and is not similar to
    // the previous event, and occurred long after the previous event, it will
    // be a mouse event.
    expect(packets[5].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[5].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[5].data[1].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[5].data[1].device, equals(-1));
    expect(packets[5].data[1].pointerIdentifier, equals(0));
    expect(packets[5].data[1].synthesized, isFalse);
    expect(packets[5].data[1].physicalX, equals(10.0 * dpi));
    expect(packets[5].data[1].physicalY, equals(10.0 * dpi));
    expect(packets[5].data[1].physicalDeltaX, equals(0.0));
    expect(packets[5].data[1].physicalDeltaY, equals(0.0));
    expect(packets[5].data[1].scrollDeltaX, equals(0.0));
    expect(packets[5].data[1].scrollDeltaY, equals(-120.0));

    // Because the delta is not in increments of 120 and has non-matching
    // wheelDelta, it will be a mouse event.
    expect(packets[6].data, hasLength(1));
    expect(packets[6].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[6].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[6].data[0].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[6].data[0].device, equals(-1));
    expect(packets[6].data[0].pointerIdentifier, equals(0));
    expect(packets[6].data[0].synthesized, isFalse);
    expect(packets[6].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[6].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[6].data[0].physicalDeltaX, equals(0.0));
    expect(packets[6].data[0].physicalDeltaY, equals(0.0));
    expect(packets[6].data[0].scrollDeltaX, equals(0.0));
    expect(packets[6].data[0].scrollDeltaY, equals(40.0));
  });

  test('does choose scroll vs scale based on ctrlKey', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    ui_web.browser.debugOperatingSystemOverride = ui_web.OperatingSystem.macOs;

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 10, clientY: 10, deltaX: 0, deltaY: 120),
    );

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 10, clientY: 10, deltaX: 0, deltaY: 100, ctrlKey: true),
    );

    keyboardConverter.handleEvent(keyDownEvent('ControlLeft', 'Control', kCtrl));

    rootElement.dispatchEvent(
      context.wheel(buttons: 0, clientX: 10, clientY: 10, deltaX: 0, deltaY: 240, ctrlKey: true),
    );

    keyboardConverter.handleEvent(keyUpEvent('ControlLeft', 'Control', kCtrl));

    expect(packets, hasLength(3));

    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].pointerIdentifier, equals(0));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));
    // Because ctrlKey is not pressed, it will be a scroll.
    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[0].data[1].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[0].data[1].pointerIdentifier, equals(0));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0.0));
    expect(packets[0].data[1].physicalDeltaY, equals(0.0));
    expect(packets[0].data[1].scrollDeltaX, equals(0.0));
    expect(packets[0].data[1].scrollDeltaY, equals(120.0));

    // Because ctrlKey is pressed, it will be a scale.
    expect(packets[1].data, hasLength(1));
    expect(packets[1].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[1].data[0].signalKind, equals(ui.PointerSignalKind.scale));
    expect(packets[1].data[0].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[1].data[0].pointerIdentifier, equals(0));
    expect(packets[1].data[0].synthesized, isFalse);
    expect(packets[1].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[1].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[1].data[0].physicalDeltaX, equals(0.0));
    expect(packets[1].data[0].physicalDeltaY, equals(0.0));
    expect(packets[1].data[0].scale, closeTo(0.60653065971, 1e-10)); // math.exp(-100/200)

    // [macOS only]: Because ctrlKey is true, but the key is pressed physically, it will be a scroll.
    expect(packets[2].data, hasLength(1));
    expect(packets[2].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[2].data[0].signalKind, equals(ui.PointerSignalKind.scroll));
    expect(packets[2].data[0].kind, equals(ui.PointerDeviceKind.mouse));
    expect(packets[2].data[0].pointerIdentifier, equals(0));
    expect(packets[2].data[0].synthesized, isFalse);
    expect(packets[2].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[2].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[2].data[0].physicalDeltaX, equals(0.0));
    expect(packets[2].data[0].physicalDeltaY, equals(0.0));
    expect(packets[2].data[0].scrollDeltaX, equals(0.0));
    expect(packets[2].data[0].scrollDeltaY, equals(240.0));

    ui_web.browser.debugOperatingSystemOverride = null;
  });

  test('does calculate delta and pointer identifier correctly', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(context.hover(clientX: 10.0, clientY: 10.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].pointerIdentifier, equals(0));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].pointerIdentifier, equals(0));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0.0));
    expect(packets[0].data[1].physicalDeltaY, equals(0.0));
    packets.clear();

    rootElement.dispatchEvent(context.hover(clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].pointerIdentifier, equals(0));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaY, equals(10.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(context.primaryDown(clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.down));
    expect(packets[0].data[0].pointerIdentifier, equals(1));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));
    packets.clear();

    rootElement.dispatchEvent(context.primaryMove(clientX: 40.0, clientY: 30.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].pointerIdentifier, equals(1));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(40.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(30.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalDeltaY, equals(10.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(context.primaryUp(clientX: 40.0, clientY: 30.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].pointerIdentifier, equals(1));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(40.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(30.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));
    packets.clear();

    rootElement.dispatchEvent(context.hover(clientX: 20.0, clientY: 10.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].pointerIdentifier, equals(1));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(-20.0 * dpi));
    expect(packets[0].data[0].physicalDeltaY, equals(-20.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(context.primaryDown(clientX: 20.0, clientY: 10.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.down));
    expect(packets[0].data[0].pointerIdentifier, equals(2));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(10.0 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0.0));
    expect(packets[0].data[0].physicalDeltaY, equals(0.0));
    packets.clear();
  });

  test('correctly converts buttons of down, move, leave, and up events', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Add and hover

    rootElement.dispatchEvent(context.hover(clientX: 10, clientY: 11));

    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10 * dpi));
    expect(packets[0].data[0].physicalY, equals(11 * dpi));

    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10 * dpi));
    expect(packets[0].data[1].physicalY, equals(11 * dpi));
    expect(packets[0].data[1].buttons, equals(0));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseDown(button: 0, buttons: 1, clientX: 10.0, clientY: 11.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.down));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(10 * dpi));
    expect(packets[0].data[0].physicalY, equals(11 * dpi));
    expect(packets[0].data[0].buttons, equals(1));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseMove(button: _kNoButtonChange, buttons: 1, clientX: 20.0, clientY: 21.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(1));
    packets.clear();

    rootElement.dispatchEvent(context.mouseUp(button: 0, clientX: 20.0, clientY: 21.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();

    // Drag with secondary button
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 20.0, clientY: 21.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.down));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(2));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseMove(button: _kNoButtonChange, buttons: 2, clientX: 30.0, clientY: 31.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(30 * dpi));
    expect(packets[0].data[0].physicalY, equals(31 * dpi));
    expect(packets[0].data[0].buttons, equals(2));
    packets.clear();

    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 30.0, clientY: 31.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(30 * dpi));
    expect(packets[0].data[0].physicalY, equals(31 * dpi));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();

    // Drag with middle button
    rootElement.dispatchEvent(
      context.mouseDown(button: 1, buttons: 4, clientX: 30.0, clientY: 31.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.down));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(30 * dpi));
    expect(packets[0].data[0].physicalY, equals(31 * dpi));
    expect(packets[0].data[0].buttons, equals(4));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseMove(button: _kNoButtonChange, buttons: 4, clientX: 40.0, clientY: 41.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(40 * dpi));
    expect(packets[0].data[0].physicalY, equals(41 * dpi));
    expect(packets[0].data[0].buttons, equals(4));
    packets.clear();

    rootElement.dispatchEvent(context.mouseUp(button: 1, clientX: 40.0, clientY: 41.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(40 * dpi));
    expect(packets[0].data[0].physicalY, equals(41 * dpi));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();

    // Leave

    rootElement.dispatchEvent(context.mouseLeave(buttons: 1, clientX: 1000.0, clientY: 2000.0));
    expect(packets, isEmpty);
    packets.clear();

    rootElement.dispatchEvent(context.mouseLeave(buttons: 0, clientX: 1000.0, clientY: 2000.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(1000 * dpi));
    expect(packets[0].data[0].physicalY, equals(2000 * dpi));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles button changes during a down sequence', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press LMB.
    rootElement.dispatchEvent(context.mouseDown(button: 0, buttons: 1));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(1));
    packets.clear();

    // Press MMB.
    rootElement.dispatchEvent(context.mouseMove(button: 1, buttons: 5));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(5));
    packets.clear();

    // Release LMB.
    rootElement.dispatchEvent(context.mouseMove(button: 0, buttons: 4));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(4));
    packets.clear();

    // Release MMB.
    rootElement.dispatchEvent(context.mouseUp(button: 1));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('synthesizes a pointerup event when pointermove comes before the up', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen when the user pops up the context menu by right
    // clicking, then dismisses it with a left click.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(context.mouseDown(button: 2, buttons: 2, clientX: 10, clientY: 11));

    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].physicalX, equals(10 * dpi));
    expect(packets[0].data[0].physicalY, equals(11 * dpi));

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].physicalX, equals(10 * dpi));
    expect(packets[0].data[1].physicalY, equals(11 * dpi));
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseMove(button: _kNoButtonChange, buttons: 2, clientX: 20.0, clientY: 21.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(2));
    packets.clear();

    rootElement.dispatchEvent(
      context.mouseMove(button: _kNoButtonChange, buttons: 2, clientX: 20.0, clientY: 21.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(2));
    packets.clear();

    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 20.0, clientY: 21.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].physicalX, equals(20 * dpi));
    expect(packets[0].data[0].physicalY, equals(21 * dpi));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles uncontinuous button changes during a down sequence', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking, but holds RMB;
    //  - Clicks LMB;
    //  - Releases RMB.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(context.mouseDown(button: 2, buttons: 2));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Press LMB. The event will have "button: -1" here, despite the change
    // in "buttons", probably because the "press" gesture was absorbed by
    // dismissing the context menu.
    rootElement.dispatchEvent(context.mouseMove(button: _kNoButtonChange, buttons: 3));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(3));
    packets.clear();

    // Release LMB.
    rootElement.dispatchEvent(context.mouseMove(button: 0, buttons: 2));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(2));
    packets.clear();

    // Release RMB.
    rootElement.dispatchEvent(context.mouseUp(button: 2));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles missing right mouse button up when followed by move', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking;
    //  - Clicks LMB to close context menu.
    //  - Moves mouse.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB popping up the context menu, then release by LMB down and up.
    // Browser won't send up event in that case.
    rootElement.dispatchEvent(context.mouseDown(button: 2, buttons: 2));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // User now hovers.
    rootElement.dispatchEvent(context.mouseMove(button: _kNoButtonChange, buttons: 0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(0));
    packets.clear();
  });

  test('handles RMB click when the browser sends it as a move', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // When the user clicks the RMB and moves the mouse quickly (before the
    // context menu shows up), the browser sends a move event before down.
    // The move event will have "button:-1, buttons:2".

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseMove(button: -1, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles hover after RMB click', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking, but holds RMB;
    //  - Move the pointer to hover.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Move the mouse. The event will have "buttons: 0" because RMB was
    // released but the browser didn't send a pointerup/mouseup event.
    // The hover is also triggered at a different position.
    rootElement.dispatchEvent(context.hover(clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(3));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].buttons, equals(2));
    expect(packets[0].data[1].change, equals(ui.PointerChange.up));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[2].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[2].synthesized, isFalse);
    expect(packets[0].data[2].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles LMB click after RMB click', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking, but holds RMB;
    //  - Clicks LMB in a different location;
    //  - Release LMB.
    //
    // The LMB click occurs in a different location because when RMB is
    // clicked, and the contextmenu is shown, the browser stops sending
    // `pointermove`/`mousemove` events. Then when the LMB click comes in, it
    // could be in a different location without any `*move` events in between.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);

    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Press LMB.
    rootElement.dispatchEvent(
      context.mouseDown(button: 0, buttons: 3, clientX: 20.0, clientY: 20.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(3));
    packets.clear();

    // Release LMB.
    rootElement.dispatchEvent(context.primaryUp(clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles two consecutive RMB clicks with no up in between', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking, but holds RMB;
    //  - Clicks RMB again in a different location;

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Press RMB again. In Chrome, when RMB is clicked again while the
    // context menu is still active, it sends a pointerdown/mousedown event
    // with "buttons:0". We convert this to pointer up, pointer down.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 0, clientX: 20.0, clientY: 20.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(3));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].buttons, equals(2));
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(20.0 * dpi));
    expect(packets[0].data[1].change, equals(ui.PointerChange.up));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[2].change, equals(ui.PointerChange.down));
    expect(packets[0].data[2].synthesized, isFalse);
    expect(packets[0].data[2].buttons, equals(2));
    packets.clear();

    // Release RMB.
    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles two consecutive RMB clicks with up in between', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking, but doesn't hold RMB;
    //  - Clicks RMB again in a different location;
    //
    // This seems to be happening sometimes when using RMB on the Mac trackpad.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // RMB up.
    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 10.0, clientY: 10.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();

    // Press RMB again. In Chrome, when RMB is clicked again while the
    // context menu is still active, it sends a pointerdown/mousedown event
    // with "buttons:0".
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 0, clientX: 20.0, clientY: 20.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Release RMB.
    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 20.0, clientY: 20.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly handles two consecutive RMB clicks in two different locations', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //  - Pops up the context menu by right clicking;
    //  - The browser sends RMB up event;
    //  - Click RMB again in a different location;
    //
    // This scenario happens occasionally. I'm still not sure why, but in some
    // cases, the browser actually sends an `up` event for the RMB click even
    // when the context menu is shown.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press RMB and hold, popping up the context menu.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 10.0, clientY: 10.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();

    // Release RMB.
    rootElement.dispatchEvent(context.mouseUp(button: 2, clientX: 10.0, clientY: 10.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();

    // Press RMB again, in a different location.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 2, clientX: 20.0, clientY: 20.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(2));
    packets.clear();
  });

  test('handles overlapping left/right down and up events', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen with the following gesture sequence:
    //
    //     LMB:   down-------------------up
    //     RMB:              down------------------up
    // Flutter:   down-------move-------move-------up

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press and hold LMB.
    rootElement.dispatchEvent(
      context.mouseDown(button: 0, buttons: 1, clientX: 5.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(1));
    expect(packets[0].data[1].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(100.0 * dpi));
    packets.clear();

    // Press and hold RMB. The pointer is already down, so we only send a move
    // to update the position of the pointer.
    rootElement.dispatchEvent(
      context.mouseDown(button: 2, buttons: 3, clientX: 20.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].buttons, equals(3));
    expect(packets[0].data[0].physicalX, equals(20.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(100.0 * dpi));
    packets.clear();

    // Release LMB. The pointer is still down (RMB), so we only send a move to
    // update the position of the pointer.
    rootElement.dispatchEvent(
      context.mouseUp(button: 0, buttons: 2, clientX: 30.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].buttons, equals(2));
    expect(packets[0].data[0].physicalX, equals(30.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(100.0 * dpi));
    packets.clear();

    // Release RMB. There's no more buttons down, so we send an up event.
    rootElement.dispatchEvent(
      context.mouseUp(button: 2, buttons: 0, clientX: 30.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].buttons, equals(0));
    packets.clear();
  });

  test('correctly detects up event outside of flutterViewElement', () {
    final _ButtonedEventMixin context = _PointerEventContext();
    // This can happen when the up event occurs while the mouse is outside the
    // browser window.

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Press and drag around.
    rootElement.dispatchEvent(context.primaryDown(clientX: 10.0, clientY: 10.0));
    rootElement.dispatchEvent(context.primaryMove(clientX: 12.0, clientY: 10.0));
    rootElement.dispatchEvent(context.primaryMove(clientX: 15.0, clientY: 10.0));
    rootElement.dispatchEvent(context.primaryMove(clientX: 20.0, clientY: 10.0));
    packets.clear();

    // Move outside the flutterViewElement.
    rootElement.dispatchEvent(context.primaryMove(clientX: 900.0, clientY: 1900.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].physicalX, equals(900.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(1900.0 * dpi));
    packets.clear();

    // Release outside the flutterViewElement.
    rootElement.dispatchEvent(context.primaryUp(clientX: 1000.0, clientY: 2000.0));
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.move));
    expect(packets[0].data[0].physicalX, equals(1000.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(2000.0 * dpi));
    expect(packets[0].data[1].change, equals(ui.PointerChange.up));
    expect(packets[0].data[1].physicalX, equals(1000.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(2000.0 * dpi));
    packets.clear();
  });

  // STYLUS

  test('handles stylus touches', () {
    // Repeated stylus touches use different pointerIds.

    final _PointerEventContext context = _PointerEventContext();

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(
      context.stylusTouchDown(pointerId: 100, buttons: 1, clientX: 5.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(1));
    expect(packets[0].data[1].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(100.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(
      context.stylusTouchUp(pointerId: 100, buttons: 0, clientX: 5.0, clientY: 100.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(100.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(
      context.stylusTouchDown(pointerId: 101, buttons: 1, clientX: 5.0, clientY: 150.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.hover));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(150.0 * dpi));
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].synthesized, isFalse);
    expect(packets[0].data[1].buttons, equals(1));
    expect(packets[0].data[1].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[1].physicalY, equals(150.0 * dpi));
    packets.clear();

    rootElement.dispatchEvent(
      context.stylusTouchUp(pointerId: 101, buttons: 0, clientX: 5.0, clientY: 150.0),
    );
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(1));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].synthesized, isFalse);
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(5.0 * dpi));
    expect(packets[0].data[0].physicalY, equals(150.0 * dpi));
    packets.clear();
  });

  // MULTIPOINTER ADAPTERS

  test('treats each pointer separately', () {
    final _MultiPointerEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    List<ui.PointerData> data;
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Two pointers down
    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 2, clientX: 100, clientY: 101),
          _TouchDetails(pointer: 3, clientX: 200, clientY: 201),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 2);
    expect(packets[0].data.length, 2);
    expect(packets[1].data.length, 2);

    data = _allPointerData(packets);
    expect(data, hasLength(4));
    expect(data[0].change, equals(ui.PointerChange.add));
    expect(data[0].synthesized, isTrue);
    expect(data[0].device, equals(2));
    expect(data[0].physicalX, equals(100 * dpi));
    expect(data[0].physicalY, equals(101 * dpi));

    expect(data[1].change, equals(ui.PointerChange.down));
    expect(data[1].device, equals(2));
    expect(data[1].buttons, equals(1));
    expect(data[1].physicalX, equals(100 * dpi));
    expect(data[1].physicalY, equals(101 * dpi));
    expect(data[1].physicalDeltaX, equals(0));
    expect(data[1].physicalDeltaY, equals(0));

    expect(data[2].change, equals(ui.PointerChange.add));
    expect(data[2].synthesized, isTrue);
    expect(data[2].device, equals(3));
    expect(data[2].physicalX, equals(200 * dpi));
    expect(data[2].physicalY, equals(201 * dpi));

    expect(data[3].change, equals(ui.PointerChange.down));
    expect(data[3].device, equals(3));
    expect(data[3].buttons, equals(1));
    expect(data[3].physicalX, equals(200 * dpi));
    expect(data[3].physicalY, equals(201 * dpi));
    expect(data[3].physicalDeltaX, equals(0));
    expect(data[3].physicalDeltaY, equals(0));
    packets.clear();

    // Two pointers move
    context
        .multiTouchMove(const <_TouchDetails>[
          _TouchDetails(pointer: 3, clientX: 300, clientY: 302),
          _TouchDetails(pointer: 2, clientX: 400, clientY: 402),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 2);
    expect(packets[0].data.length, 1);
    expect(packets[1].data.length, 1);

    data = _allPointerData(packets);
    expect(data, hasLength(2));
    expect(data[0].change, equals(ui.PointerChange.move));
    expect(data[0].device, equals(3));
    expect(data[0].buttons, equals(1));
    expect(data[0].physicalX, equals(300 * dpi));
    expect(data[0].physicalY, equals(302 * dpi));
    expect(data[0].physicalDeltaX, equals(100 * dpi));
    expect(data[0].physicalDeltaY, equals(101 * dpi));

    expect(data[1].change, equals(ui.PointerChange.move));
    expect(data[1].device, equals(2));
    expect(data[1].buttons, equals(1));
    expect(data[1].physicalX, equals(400 * dpi));
    expect(data[1].physicalY, equals(402 * dpi));
    expect(data[1].physicalDeltaX, equals(300 * dpi));
    expect(data[1].physicalDeltaY, equals(301 * dpi));
    packets.clear();

    // One pointer up
    context
        .multiTouchUp(const <_TouchDetails>[_TouchDetails(pointer: 3, clientX: 300, clientY: 302)])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].device, equals(3));
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(300 * dpi));
    expect(packets[0].data[0].physicalY, equals(302 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0));
    expect(packets[0].data[0].physicalDeltaY, equals(0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.remove));
    expect(packets[0].data[1].device, equals(3));
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[1].physicalX, equals(300 * dpi));
    expect(packets[0].data[1].physicalY, equals(302 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0));
    expect(packets[0].data[1].physicalDeltaY, equals(0));
    packets.clear();

    // Another pointer up
    context
        .multiTouchUp(const <_TouchDetails>[_TouchDetails(pointer: 2, clientX: 400, clientY: 402)])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].device, equals(2));
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(400 * dpi));
    expect(packets[0].data[0].physicalY, equals(402 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0));
    expect(packets[0].data[0].physicalDeltaY, equals(0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.remove));
    expect(packets[0].data[1].device, equals(2));
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[1].physicalX, equals(400 * dpi));
    expect(packets[0].data[1].physicalY, equals(402 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0));
    expect(packets[0].data[1].physicalDeltaY, equals(0));
    packets.clear();

    // Again two pointers down (reuse pointer ID)
    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 3, clientX: 500, clientY: 501),
          _TouchDetails(pointer: 2, clientX: 600, clientY: 601),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 2);
    expect(packets[0].data.length, 2);
    expect(packets[1].data.length, 2);

    data = _allPointerData(packets);
    expect(data, hasLength(4));
    expect(data[0].change, equals(ui.PointerChange.add));
    expect(data[0].synthesized, isTrue);
    expect(data[0].device, equals(3));
    expect(data[0].physicalX, equals(500 * dpi));
    expect(data[0].physicalY, equals(501 * dpi));

    expect(data[1].change, equals(ui.PointerChange.down));
    expect(data[1].device, equals(3));
    expect(data[1].buttons, equals(1));
    expect(data[1].physicalX, equals(500 * dpi));
    expect(data[1].physicalY, equals(501 * dpi));
    expect(data[1].physicalDeltaX, equals(0));
    expect(data[1].physicalDeltaY, equals(0));

    expect(data[2].change, equals(ui.PointerChange.add));
    expect(data[2].synthesized, isTrue);
    expect(data[2].device, equals(2));
    expect(data[2].physicalX, equals(600 * dpi));
    expect(data[2].physicalY, equals(601 * dpi));

    expect(data[3].change, equals(ui.PointerChange.down));
    expect(data[3].device, equals(2));
    expect(data[3].buttons, equals(1));
    expect(data[3].physicalX, equals(600 * dpi));
    expect(data[3].physicalY, equals(601 * dpi));
    expect(data[3].physicalDeltaX, equals(0));
    expect(data[3].physicalDeltaY, equals(0));
    packets.clear();
  });

  test('ignores pointerId on coalesced events', () {
    final _MultiPointerEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    List<ui.PointerData> data;
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 52, clientX: 100, clientY: 101),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 1);

    data = packets.single.data;
    expect(data, hasLength(2));
    expect(data[0].change, equals(ui.PointerChange.add));
    expect(data[0].synthesized, isTrue);
    expect(data[0].device, equals(52));
    expect(data[0].physicalX, equals(100 * dpi));
    expect(data[0].physicalY, equals(101 * dpi));

    expect(data[1].change, equals(ui.PointerChange.down));
    expect(data[1].device, equals(52));
    expect(data[1].buttons, equals(1));
    expect(data[1].physicalX, equals(100 * dpi));
    expect(data[1].physicalY, equals(101 * dpi));
    expect(data[1].physicalDeltaX, equals(0));
    expect(data[1].physicalDeltaY, equals(0));
    packets.clear();

    // Pointer move with coaleasced events
    context
        .multiTouchMove(const <_TouchDetails>[
          _TouchDetails(
            pointer: 52,
            coalescedEvents: <_CoalescedTouchDetails>[
              _CoalescedTouchDetails(pointer: 0, clientX: 301, clientY: 302),
              _CoalescedTouchDetails(pointer: 0, clientX: 401, clientY: 402),
            ],
          ),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 1);

    data = packets.single.data;
    expect(data, hasLength(2));
    expect(data[0].change, equals(ui.PointerChange.move));
    expect(data[0].device, equals(52));
    expect(data[0].buttons, equals(1));
    expect(data[0].physicalX, equals(301 * dpi));
    expect(data[0].physicalY, equals(302 * dpi));
    expect(data[0].physicalDeltaX, equals(201 * dpi));
    expect(data[0].physicalDeltaY, equals(201 * dpi));

    expect(data[1].change, equals(ui.PointerChange.move));
    expect(data[1].device, equals(52));
    expect(data[1].buttons, equals(1));
    expect(data[1].physicalX, equals(401 * dpi));
    expect(data[1].physicalY, equals(402 * dpi));
    expect(data[1].physicalDeltaX, equals(100 * dpi));
    expect(data[1].physicalDeltaY, equals(100 * dpi));
    packets.clear();

    // Pointer up
    context
        .multiTouchUp(const <_TouchDetails>[_TouchDetails(pointer: 52, clientX: 401, clientY: 402)])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.up));
    expect(packets[0].data[0].device, equals(52));
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(401 * dpi));
    expect(packets[0].data[0].physicalY, equals(402 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0));
    expect(packets[0].data[0].physicalDeltaY, equals(0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.remove));
    expect(packets[0].data[1].device, equals(52));
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[1].physicalX, equals(401 * dpi));
    expect(packets[0].data[1].physicalY, equals(402 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0));
    expect(packets[0].data[1].physicalDeltaY, equals(0));
    packets.clear();
  });

  test('correctly parses cancel event', () {
    final _MultiPointerEventMixin context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    // Two pointers down
    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 2, clientX: 100, clientY: 101),
          _TouchDetails(pointer: 3, clientX: 200, clientY: 201),
        ])
        .forEach(rootElement.dispatchEvent);
    packets.clear(); // Down event is tested in other tests.

    // One pointer cancel
    context
        .multiTouchCancel(const <_TouchDetails>[
          _TouchDetails(pointer: 3, clientX: 300, clientY: 302),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets.length, 1);
    expect(packets[0].data.length, 2);
    expect(packets[0].data[0].change, equals(ui.PointerChange.cancel));
    expect(packets[0].data[0].device, equals(3));
    expect(packets[0].data[0].buttons, equals(0));
    expect(packets[0].data[0].physicalX, equals(200 * dpi));
    expect(packets[0].data[0].physicalY, equals(201 * dpi));
    expect(packets[0].data[0].physicalDeltaX, equals(0));
    expect(packets[0].data[0].physicalDeltaY, equals(0));

    expect(packets[0].data[1].change, equals(ui.PointerChange.remove));
    expect(packets[0].data[1].device, equals(3));
    expect(packets[0].data[1].buttons, equals(0));
    expect(packets[0].data[1].physicalX, equals(200 * dpi));
    expect(packets[0].data[1].physicalY, equals(201 * dpi));
    expect(packets[0].data[1].physicalDeltaX, equals(0));
    expect(packets[0].data[1].physicalDeltaY, equals(0));
    packets.clear();
  });

  test('does not synthesize pointer up if from different device', () {
    final _PointerEventContext context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 1, clientX: 100, clientY: 101),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(1));
    // An add will be synthesized.
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].device, equals(1));
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].device, equals(1));
    packets.clear();

    context
        .multiTouchDown(const <_TouchDetails>[
          _TouchDetails(pointer: 2, clientX: 200, clientY: 202),
        ])
        .forEach(rootElement.dispatchEvent);
    // An add will be synthesized.
    expect(packets, hasLength(1));
    expect(packets[0].data, hasLength(2));
    expect(packets[0].data[0].change, equals(ui.PointerChange.add));
    expect(packets[0].data[0].synthesized, isTrue);
    expect(packets[0].data[0].device, equals(2));
    expect(packets[0].data[1].change, equals(ui.PointerChange.down));
    expect(packets[0].data[1].device, equals(2));
    packets.clear();
  });

  test('ignores pointer up or pointer cancel events for unknown device', () {
    final _PointerEventContext context = _PointerEventContext();
    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    context
        .multiTouchUp(const <_TouchDetails>[_TouchDetails(pointer: 23, clientX: 200, clientY: 202)])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(0));

    context
        .multiTouchCancel(const <_TouchDetails>[
          _TouchDetails(pointer: 24, clientX: 200, clientY: 202),
        ])
        .forEach(rootElement.dispatchEvent);
    expect(packets, hasLength(0));
  });

  test('handles random pointer id on up events', () {
    final _PointerEventContext context = _PointerEventContext();
    // This happens with pens that are simulated with mouse events
    // (e.g. Wacom). It sends events with the pointer type "mouse", and
    // assigns a random pointer ID to each event.
    //
    // For more info, see: https://github.com/flutter/flutter/issues/75559

    final List<ui.PointerDataPacket> packets = <ui.PointerDataPacket>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      packets.add(packet);
    };

    rootElement.dispatchEvent(
      context.mouseDown(pointerId: 12, button: 0, buttons: 1, clientX: 10.0, clientY: 10.0),
    );

    expect(packets, hasLength(1));
    expect(packets.single.data, hasLength(2));

    expect(packets.single.data[0].change, equals(ui.PointerChange.add));
    expect(packets.single.data[0].synthesized, isTrue);
    expect(packets.single.data[1].change, equals(ui.PointerChange.down));
    packets.clear();

    expect(() {
      rootElement.dispatchEvent(
        context.mouseUp(pointerId: 41, button: 0, buttons: 0, clientX: 10.0, clientY: 10.0),
      );
    }, returnsNormally);

    expect(packets, hasLength(1));
    expect(packets.single.data, hasLength(1));

    expect(packets.single.data[0].change, equals(ui.PointerChange.up));
  });

  test('throws if browser does not support pointer events', () {
    expect(
      () => PointerBinding(view, detector: MockPointerSupportDetector(false)),
      throwsUnsupportedError,
    );
  });

  group('Listener', () {
    late DomElement eventTarget;
    late DomEvent expected;
    late bool handled;

    setUp(() {
      eventTarget = createDomElement('div');
      expected = createDomEvent('Event', 'custom-event');
      handled = false;
    });

    test('listeners can be registered', () {
      Listener.register(
        event: 'custom-event',
        target: eventTarget,
        handler: (event) {
          expect(event, expected);
          handled = true;
        },
      );

      // Trigger the event...
      eventTarget.dispatchEvent(expected);
      expect(handled, isTrue);
    });

    test('listeners can be unregistered', () {
      final Listener listener = Listener.register(
        event: 'custom-event',
        target: eventTarget,
        handler: (event) {
          handled = true;
        },
      );
      listener.unregister();

      eventTarget.dispatchEvent(expected);
      expect(handled, isFalse);
    });

    test('listeners are registered only once', () {
      int timesHandled = 0;
      Listener.register(
        event: 'custom-event',
        target: eventTarget,
        handler: (event) {
          timesHandled++;
        },
      );
      eventTarget.dispatchEvent(expected);
      expect(timesHandled, 1, reason: 'The handler ran multiple times for a single event.');
    });
  });

  group('ClickDebouncer', () {
    _testClickDebouncer(getBinding: () => instance);
  });
}

typedef CapturedSemanticsEvent = ({ui.SemanticsAction type, int nodeId});

void _testClickDebouncer({required PointerBinding Function() getBinding}) {
  final DateTime testTime = DateTime(2018, 12, 17);
  late List<ui.PointerChange> pointerPackets;
  late List<CapturedSemanticsEvent> semanticsActions;
  late _PointerEventContext context;
  late PointerBinding binding;

  void testWithSemantics(
    String description,
    Future<void> Function() body, {
    Object? skip,
    @doNotSubmit bool solo = false,
  }) {
    test(
      description,
      () async {
        EngineSemantics.instance
          ..debugOverrideTimestampFunction(() => testTime)
          ..semanticsEnabled = true;
        await body();
        EngineSemantics.instance.semanticsEnabled = false;
      },
      skip: skip,
      solo: solo, // ignore: invalid_use_of_do_not_submit_member
    );
  }

  setUp(() {
    context = _PointerEventContext();
    pointerPackets = <ui.PointerChange>[];
    semanticsActions = <CapturedSemanticsEvent>[];
    ui.PlatformDispatcher.instance.onPointerDataPacket = (ui.PointerDataPacket packet) {
      for (final ui.PointerData data in packet.data) {
        pointerPackets.add(data.change);
      }
    };
    EnginePlatformDispatcher.instance.onSemanticsActionEvent = (ui.SemanticsActionEvent event) {
      semanticsActions.add((type: event.type, nodeId: event.nodeId));
    };
    binding = getBinding();
  });

  test('Forwards to framework when semantics is off', () {
    expect(EnginePlatformDispatcher.instance.semanticsEnabled, false);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);
    binding.rootElement.dispatchEvent(context.primaryDown());
    expect(pointerPackets, <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down]);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);
    expect(semanticsActions, isEmpty);
  });

  testWithSemantics('Forwards to framework when not debouncing', () async {
    expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    // This test DOM element is missing the `flt-tappable` attribute on purpose
    // so that the debouncer does not debounce events and simply lets
    // everything through.
    final DomElement testElement = createDomElement('flt-semantics');
    view.dom.semanticsHost.appendChild(testElement);

    testElement.dispatchEvent(context.primaryDown());
    testElement.dispatchEvent(context.primaryUp());
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    expect(pointerPackets, <ui.PointerChange>[
      ui.PointerChange.add,
      ui.PointerChange.down,
      ui.PointerChange.up,
    ]);
    expect(semanticsActions, isEmpty);
  });

  testWithSemantics('Accumulates pointer events starting from pointerdown', () async {
    expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    final DomElement testElement = createDomElement('flt-semantics');
    testElement.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(testElement);

    testElement.dispatchEvent(context.primaryDown());
    expect(
      reason: 'Should start debouncing at first pointerdown',
      PointerBinding.clickDebouncer.isDebouncing,
      true,
    );

    testElement.dispatchEvent(context.primaryUp());
    expect(
      reason: 'Should still be debouncing after pointerup',
      PointerBinding.clickDebouncer.isDebouncing,
      true,
    );

    expect(
      reason: 'Events are withheld from the framework while debouncing',
      pointerPackets,
      <ui.PointerChange>[],
    );
    expect(PointerBinding.clickDebouncer.debugState!.target, testElement);
    expect(PointerBinding.clickDebouncer.debugState!.timer.isActive, isTrue);
    expect(
      PointerBinding.clickDebouncer.debugState!.queue.map<String>((QueuedEvent e) => e.event.type),
      <String>['pointerdown', 'pointerup'],
    );

    await Future<void>.delayed(const Duration(milliseconds: 250));
    expect(
      reason: 'Should stop debouncing after timer expires.',
      PointerBinding.clickDebouncer.isDebouncing,
      false,
    );
    expect(
      reason: 'Queued up events should be flushed to the framework.',
      pointerPackets,
      <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down, ui.PointerChange.up],
    );
    expect(semanticsActions, isEmpty);
  });

  testWithSemantics('Flushes events to framework when target changes', () async {
    expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    final DomElement testElement = createDomElement('flt-semantics');
    testElement.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(testElement);

    testElement.dispatchEvent(context.primaryDown());
    expect(
      reason: 'Should start debouncing at first pointerdown',
      PointerBinding.clickDebouncer.isDebouncing,
      true,
    );

    final DomElement newTarget = createDomElement('flt-semantics');
    newTarget.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(newTarget);
    newTarget.dispatchEvent(context.primaryUp());

    expect(
      reason: 'Should stop debouncing when target changes.',
      PointerBinding.clickDebouncer.isDebouncing,
      false,
    );
    expect(
      reason: 'The state should be cleaned up after stopping debouncing.',
      PointerBinding.clickDebouncer.debugState,
      isNull,
    );
    expect(
      reason: 'Queued up events should be flushed to the framework.',
      pointerPackets,
      <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down, ui.PointerChange.up],
    );
    expect(semanticsActions, isEmpty);
  });

  testWithSemantics('Forwards click to framework when not debouncing but listening', () async {
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    final DomElement testElement = createDomElement('flt-semantics');
    testElement.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(testElement);

    final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
      'clientX': testElement.getBoundingClientRect().x,
      'clientY': testElement.getBoundingClientRect().y,
    });

    PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, true);
    expect(PointerBinding.clickDebouncer.isDebouncing, false);
    expect(pointerPackets, isEmpty);
    expect(semanticsActions, <CapturedSemanticsEvent>[(type: ui.SemanticsAction.tap, nodeId: 42)]);
  });

  testWithSemantics('Forwards click to framework when debouncing and listening', () async {
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    final DomElement testElement = createDomElement('flt-semantics');
    testElement.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(testElement);
    testElement.dispatchEvent(context.primaryDown());
    expect(PointerBinding.clickDebouncer.isDebouncing, true);

    final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
      'clientX': testElement.getBoundingClientRect().x,
      'clientY': testElement.getBoundingClientRect().y,
    });

    PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, true);
    expect(pointerPackets, isEmpty);
    expect(semanticsActions, <CapturedSemanticsEvent>[(type: ui.SemanticsAction.tap, nodeId: 42)]);
  });

  testWithSemantics('Dedupes click if debouncing but not listening', () async {
    expect(PointerBinding.clickDebouncer.isDebouncing, false);

    final DomElement testElement = createDomElement('flt-semantics');
    testElement.setAttribute('flt-tappable', '');
    view.dom.semanticsHost.appendChild(testElement);
    testElement.dispatchEvent(context.primaryDown());
    expect(PointerBinding.clickDebouncer.isDebouncing, true);

    final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
      'clientX': testElement.getBoundingClientRect().x,
      'clientY': testElement.getBoundingClientRect().y,
    });

    PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, false);
    expect(
      reason:
          'When tappable declares that it is not listening to click events '
          'the debouncer flushes the pointer events to the framework and '
          'lets it sort it out.',
      pointerPackets,
      <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down],
    );
    expect(semanticsActions, isEmpty);
  });

  testWithSemantics(
    'Dedupes click if pointer down/up flushed recently',
    () async {
      expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
      expect(PointerBinding.clickDebouncer.isDebouncing, false);

      final DomElement testElement = createDomElement('flt-semantics');
      testElement.setAttribute('flt-tappable', '');
      view.dom.semanticsHost.appendChild(testElement);

      testElement.dispatchEvent(context.primaryDown());

      // Simulate the user holding the pointer down for some time before releasing,
      // such that the pointerup event happens close to timer expiration. This
      // will create the situation that the click event arrives just after the
      // pointerup is flushed. Forwarding the click to the framework would look
      // like a double-click, so the click event is deduped.
      await Future<void>.delayed(const Duration(milliseconds: 190));

      testElement.dispatchEvent(context.primaryUp());
      expect(PointerBinding.clickDebouncer.isDebouncing, true);
      expect(reason: 'Timer has not expired yet', pointerPackets, isEmpty);

      // Wait for the timer to expire to make sure pointer events are flushed.
      await Future<void>.delayed(const Duration(milliseconds: 20));

      expect(
        reason:
            'Queued up events should be flushed to the framework because the '
            'time expired before the click event arrived.',
        pointerPackets,
        <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down, ui.PointerChange.up],
      );

      final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
        'clientX': testElement.getBoundingClientRect().x,
        'clientY': testElement.getBoundingClientRect().y,
      });
      PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, true);

      expect(reason: 'Because the DOM click event was deduped.', semanticsActions, isEmpty);
      // TODO(yjbanov): https://github.com/flutter/flutter/issues/142991.
    },
    skip: ui_web.browser.operatingSystem == ui_web.OperatingSystem.windows,
  );

  // Regression test for https://github.com/flutter/flutter/issues/147050
  //
  // This test emulates a long-press. Start with a "pointerdown" followed by no
  // activity long enough that the debounce timer expires and the state of the
  // ClickDebouncer is reset back to idle. Then a "pointerup" arrives seemingly
  // standalone. Since no gesture is being debounced, the debouncer simply
  // forwards it to the framework. However, "pointerup" will be immediately
  // followed by a "click". Since we sent the "pointerdown" and "pointerup" to
  // the framework already, the framework registered a tap. Forwarding the
  // "click" would lead to a double-tap. This was the bug.
  testWithSemantics(
    'Dedupes click if pointer up happened recently without debouncing',
    () async {
      expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
      expect(PointerBinding.clickDebouncer.isDebouncing, false);

      final DomElement testElement = createDomElement('flt-semantics');
      testElement.setAttribute('flt-tappable', '');
      view.dom.semanticsHost.appendChild(testElement);

      // Begin a long-press with a "pointerdown".
      testElement.dispatchEvent(context.primaryDown());

      // Expire the timer causing the debouncer to reset itself.
      await Future<void>.delayed(const Duration(milliseconds: 250));
      expect(
        reason: '"pointerdown" should be flushed when the timer expires.',
        pointerPackets,
        <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down],
      );
      pointerPackets.clear();

      // Send a "pointerup" while the debouncer is not debouncing anything.
      testElement.dispatchEvent(context.primaryUp());

      // A standalone "pointerup" should not start debouncing anything.
      expect(PointerBinding.clickDebouncer.isDebouncing, isFalse);
      expect(
        reason: 'The "pointerup" should be forwarded to the framework immediately',
        pointerPackets,
        <ui.PointerChange>[ui.PointerChange.up],
      );

      // Use a delay that's short enough for the click to be deduped.
      await Future<void>.delayed(const Duration(milliseconds: 10));

      final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
        'clientX': testElement.getBoundingClientRect().x,
        'clientY': testElement.getBoundingClientRect().y,
      });
      PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, true);

      expect(reason: 'Because the DOM click event was deduped.', semanticsActions, isEmpty);
      // TODO(yjbanov): https://github.com/flutter/flutter/issues/142991.
    },
    skip: ui_web.browser.operatingSystem == ui_web.OperatingSystem.windows,
  );

  testWithSemantics(
    'Forwards click if enough time passed after the last flushed pointerup',
    () async {
      expect(EnginePlatformDispatcher.instance.semanticsEnabled, true);
      expect(PointerBinding.clickDebouncer.isDebouncing, false);

      final DomElement testElement = createDomElement('flt-semantics');
      testElement.setAttribute('flt-tappable', '');
      view.dom.semanticsHost.appendChild(testElement);

      testElement.dispatchEvent(context.primaryDown());

      // Simulate the user holding the pointer down for some time before releasing,
      // such that the pointerup event happens close to timer expiration. This
      // makes it possible for the click to arrive early. However, this test in
      // particular will delay the click to check that the delay is checked
      // correctly. The inverse situation was already tested in the previous test.
      await Future<void>.delayed(const Duration(milliseconds: 190));

      testElement.dispatchEvent(context.primaryUp());
      expect(PointerBinding.clickDebouncer.isDebouncing, true);
      expect(reason: 'Timer has not expired yet', pointerPackets, isEmpty);

      // Wait for the timer to expire to make sure pointer events are flushed.
      await Future<void>.delayed(const Duration(milliseconds: 100));

      expect(
        reason:
            'Queued up events should be flushed to the framework because the '
            'time expired before the click event arrived.',
        pointerPackets,
        <ui.PointerChange>[ui.PointerChange.add, ui.PointerChange.down, ui.PointerChange.up],
      );

      final DomEvent click = createDomMouseEvent('click', <Object?, Object?>{
        'clientX': testElement.getBoundingClientRect().x,
        'clientY': testElement.getBoundingClientRect().y,
      });
      PointerBinding.clickDebouncer.onClick(click, view.viewId, 42, true);

      expect(
        reason:
            'The DOM click should still be sent to the framework because it '
            'happened far enough from the last pointerup that it is unlikely '
            'to be a duplicate.',
        semanticsActions,
        <CapturedSemanticsEvent>[(type: ui.SemanticsAction.tap, nodeId: 42)],
      );
    },
  );
}

class MockSafariPointerEventWorkaround implements SafariPointerEventWorkaround {
  bool workAroundInvoked = false;

  @override
  void workAroundMissingPointerEvents() {
    workAroundInvoked = true;
  }

  @override
  void dispose() {}
}

abstract class _BasicEventContext {
  String get name;

  // Accepted modifier keys are 'Alt', 'Control', 'Meta' and 'Shift'.
  // https://www.w3.org/TR/uievents-key/#keys-modifier defines more modifiers,
  // but only the four main modifiers could be set from MouseEvent, PointerEvent
  // and TouchEvent constructors.
  bool altPressed = false;
  bool ctrlPressed = false;
  bool metaPressed = false;
  bool shiftPressed = false;

  // Generate an event that is:
  //
  //  * For mouse, a left click
  //  * For touch, a touch down
  DomEvent primaryDown({double clientX, double clientY});

  // Generate an event that is:
  //
  //  * For mouse, a drag with LMB down
  //  * For touch, a touch drag
  DomEvent primaryMove({double clientX, double clientY});

  // Generate an event that is:
  //
  //  * For mouse, release LMB
  //  * For touch, a touch up
  DomEvent primaryUp({double clientX, double clientY});

  void pressAllModifiers() {
    altPressed = true;
    ctrlPressed = true;
    metaPressed = true;
    shiftPressed = true;
  }

  void unpressAllModifiers() {
    altPressed = false;
    ctrlPressed = false;
    metaPressed = false;
    shiftPressed = false;
  }
}

mixin _ButtonedEventMixin on _BasicEventContext {
  // Generate an event that is a mouse down with the specific buttons.
  DomEvent mouseDown({double? clientX, double? clientY, int? button, int? buttons});

  // Generate an event that is a mouse drag with the specific buttons, or button
  // changes during the drag.
  //
  // If there is no button change, assign `button` with _kNoButtonChange.
  DomEvent mouseMove({double? clientX, double? clientY, required int button, required int buttons});

  // Generate an event that moves the mouse outside of the tracked area.
  DomEvent mouseLeave({double? clientX, double? clientY, required int buttons});

  // Generate an event that releases all mouse buttons.
  DomEvent mouseUp({double? clientX, double? clientY, int? button, int? buttons});

  DomEvent hover({double? clientX, double? clientY}) {
    return mouseMove(buttons: 0, button: _kNoButtonChange, clientX: clientX, clientY: clientY);
  }

  @override
  DomEvent primaryDown({double? clientX, double? clientY}) {
    return mouseDown(buttons: 1, button: 0, clientX: clientX, clientY: clientY);
  }

  @override
  DomEvent primaryMove({double? clientX, double? clientY}) {
    return mouseMove(buttons: 1, button: _kNoButtonChange, clientX: clientX, clientY: clientY);
  }

  @override
  DomEvent primaryUp({double? clientX, double? clientY}) {
    return mouseUp(button: 0, clientX: clientX, clientY: clientY);
  }

  DomEvent wheel({
    required int? buttons,
    required double? clientX,
    required double? clientY,
    required double? deltaX,
    required double? deltaY,
    double? wheelDeltaX,
    double? wheelDeltaY,
    int? timeStamp,
    bool ctrlKey = false,
  }) {
    final DomEvent event = createDomWheelEvent('wheel', <String, Object>{
      if (buttons != null) 'buttons': buttons,
      if (clientX != null) 'clientX': clientX,
      if (clientY != null) 'clientY': clientY,
      if (deltaX != null) 'deltaX': deltaX,
      if (deltaY != null) 'deltaY': deltaY,
      if (wheelDeltaX != null) 'wheelDeltaX': wheelDeltaX,
      if (wheelDeltaY != null) 'wheelDeltaY': wheelDeltaY,
      'ctrlKey': ctrlKey,
      'cancelable': true,
      'bubbles': true,
      'composed': true,
    });
    // timeStamp can't be set in the constructor, need to override the getter.
    if (timeStamp != null) {
      js_util.callMethod<void>(objectConstructor, 'defineProperty', <dynamic>[
        event,
        'timeStamp',
        js_util.jsify(<String, dynamic>{'value': timeStamp, 'configurable': true}),
      ]);
    }
    return event;
  }
}

class _TouchDetails {
  const _TouchDetails({this.pointer, this.clientX, this.clientY, this.coalescedEvents});

  final int? pointer;
  final double? clientX;
  final double? clientY;

  final List<_CoalescedTouchDetails>? coalescedEvents;
}

class _CoalescedTouchDetails {
  const _CoalescedTouchDetails({this.pointer, this.clientX, this.clientY});

  final int? pointer;
  final double? clientX;
  final double? clientY;
}

mixin _MultiPointerEventMixin on _BasicEventContext {
  List<DomEvent> multiTouchDown(List<_TouchDetails> touches);
  List<DomEvent> multiTouchMove(List<_TouchDetails> touches);
  List<DomEvent> multiTouchUp(List<_TouchDetails> touches);
  List<DomEvent> multiTouchCancel(List<_TouchDetails> touches);

  @override
  DomEvent primaryDown({double? clientX, double? clientY}) {
    return multiTouchDown(<_TouchDetails>[
      _TouchDetails(pointer: 1, clientX: clientX, clientY: clientY),
    ])[0];
  }

  @override
  DomEvent primaryMove({double? clientX, double? clientY}) {
    return multiTouchMove(<_TouchDetails>[
      _TouchDetails(pointer: 1, clientX: clientX, clientY: clientY),
    ])[0];
  }

  @override
  DomEvent primaryUp({double? clientX, double? clientY}) {
    return multiTouchUp(<_TouchDetails>[
      _TouchDetails(pointer: 1, clientX: clientX, clientY: clientY),
    ])[0];
  }
}

// A test context for `_PointerAdapter`, including its name, and how to generate
// events.
//
// For the difference between MouseEvent and PointerEvent, see _MouseAdapter.
class _PointerEventContext extends _BasicEventContext
    with _ButtonedEventMixin
    implements _MultiPointerEventMixin {
  @override
  String get name => 'PointerAdapter';

  @override
  List<DomEvent> multiTouchDown(List<_TouchDetails> touches) {
    assert(
      touches.every((_TouchDetails details) => details.coalescedEvents == null),
      'Coalesced events are not allowed for pointerdown events.',
    );
    return touches
        .map(
          (_TouchDetails details) => _downWithFullDetails(
            pointer: details.pointer,
            buttons: 1,
            button: 0,
            clientX: details.clientX,
            clientY: details.clientY,
            pointerType: 'touch',
          ),
        )
        .toList();
  }

  @override
  DomEvent mouseDown({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointerId = 1,
  }) {
    return _downWithFullDetails(
      pointer: pointerId,
      buttons: buttons,
      button: button,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'mouse',
    );
  }

  DomEvent _downWithFullDetails({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointer,
    String? pointerType,
  }) {
    return createDomPointerEvent('pointerdown', <String, dynamic>{
      'bubbles': true,
      'pointerId': pointer,
      'button': button,
      'buttons': buttons,
      'clientX': clientX,
      'clientY': clientY,
      'pointerType': pointerType,
      'altKey': altPressed,
      'ctrlKey': ctrlPressed,
      'metaKey': metaPressed,
      'shiftKey': shiftPressed,
    });
  }

  @override
  List<DomEvent> multiTouchMove(List<_TouchDetails> touches) {
    return touches
        .map(
          (_TouchDetails details) => _moveWithFullDetails(
            pointer: details.pointer,
            buttons: 1,
            button: _kNoButtonChange,
            clientX: details.clientX,
            clientY: details.clientY,
            pointerType: 'touch',
            coalescedEvents: details.coalescedEvents,
          ),
        )
        .toList();
  }

  @override
  DomEvent mouseMove({
    double? clientX,
    double? clientY,
    required int button,
    required int buttons,
    int pointerId = 1,
  }) {
    return _moveWithFullDetails(
      pointer: pointerId,
      buttons: buttons,
      button: button,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'mouse',
    );
  }

  DomEvent _moveWithFullDetails({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointer,
    String? pointerType,
    List<_CoalescedTouchDetails>? coalescedEvents,
  }) {
    final event = createDomPointerEvent('pointermove', <String, dynamic>{
      'bubbles': true,
      'pointerId': pointer,
      'button': button,
      'buttons': buttons,
      'clientX': clientX,
      'clientY': clientY,
      'pointerType': pointerType,
    });

    if (coalescedEvents != null) {
      // There's no JS API for setting coalesced events, so we need to
      // monkey-patch the `getCoalescedEvents` method to return what we want.
      final coalescedEventJs =
          coalescedEvents
              .map(
                (_CoalescedTouchDetails details) => _moveWithFullDetails(
                  pointer: details.pointer,
                  button: button,
                  buttons: buttons,
                  clientX: details.clientX,
                  clientY: details.clientY,
                  pointerType: 'touch',
                ),
              )
              .toJSAnyDeep;

      js_util.setProperty(
        event,
        'getCoalescedEvents',
        js_util.allowInterop(() {
          return coalescedEventJs;
        }),
      );
    }

    return event;
  }

  @override
  DomEvent mouseLeave({double? clientX, double? clientY, required int buttons, int pointerId = 1}) {
    return _leaveWithFullDetails(
      pointer: pointerId,
      buttons: buttons,
      button: 0,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'mouse',
    );
  }

  DomEvent _leaveWithFullDetails({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointer,
    String? pointerType,
  }) {
    return createDomPointerEvent('pointerleave', <String, dynamic>{
      'bubbles': true,
      'pointerId': pointer,
      'button': button,
      'buttons': buttons,
      'clientX': clientX,
      'clientY': clientY,
      'pointerType': pointerType,
    });
  }

  @override
  List<DomEvent> multiTouchUp(List<_TouchDetails> touches) {
    assert(
      touches.every((_TouchDetails details) => details.coalescedEvents == null),
      'Coalesced events are not allowed for pointerup events.',
    );
    return touches
        .map(
          (_TouchDetails details) => _upWithFullDetails(
            pointer: details.pointer,
            button: 0,
            clientX: details.clientX,
            clientY: details.clientY,
            pointerType: 'touch',
          ),
        )
        .toList();
  }

  @override
  DomEvent mouseUp({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointerId = 1,
  }) {
    return _upWithFullDetails(
      pointer: pointerId,
      button: button,
      buttons: buttons,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'mouse',
    );
  }

  DomEvent _upWithFullDetails({
    double? clientX,
    double? clientY,
    int? button,
    int? buttons,
    int? pointer,
    String? pointerType,
  }) {
    return createDomPointerEvent('pointerup', <String, dynamic>{
      'bubbles': true,
      'pointerId': pointer,
      'button': button,
      'buttons': buttons,
      'clientX': clientX,
      'clientY': clientY,
      'pointerType': pointerType,
    });
  }

  @override
  List<DomEvent> multiTouchCancel(List<_TouchDetails> touches) {
    assert(
      touches.every((_TouchDetails details) => details.coalescedEvents == null),
      'Coalesced events are not allowed for pointercancel events.',
    );
    return touches
        .map(
          (_TouchDetails details) => createDomPointerEvent('pointercancel', <String, dynamic>{
            'bubbles': true,
            'pointerId': details.pointer,
            'button': 0,
            'buttons': 0,
            'clientX': 0,
            'clientY': 0,
            'pointerType': 'touch',
          }),
        )
        .toList();
  }

  // STYLUSES

  DomEvent stylusTouchDown({
    double? clientX,
    double? clientY,
    int? buttons,
    int? pointerId = 1000,
  }) {
    return _downWithFullDetails(
      pointer: pointerId,
      buttons: buttons,
      button: 0,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'pen',
    );
  }

  DomEvent stylusTouchUp({double? clientX, double? clientY, int? buttons, int? pointerId = 1000}) {
    return _upWithFullDetails(
      pointer: pointerId,
      buttons: buttons,
      button: 0,
      clientX: clientX,
      clientY: clientY,
      pointerType: 'pen',
    );
  }
}

class MockPointerSupportDetector implements PointerSupportDetector {
  MockPointerSupportDetector(this.hasPointerEvents);

  @override
  final bool hasPointerEvents;
}
