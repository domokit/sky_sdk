// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert';
import 'dart:core';
import 'dart:ffi' as ffi;
import 'dart:io';
import 'dart:isolate';
import 'dart:typed_data';
import 'dart:ui';

void main() {}

@pragma('vm:entry-point')
void customEntrypoint() {
  sayHiFromCustomEntrypoint();
}

@pragma('vm:external-name', 'SayHiFromCustomEntrypoint')
external void sayHiFromCustomEntrypoint();

@pragma('vm:entry-point')
void customEntrypoint1() {
  sayHiFromCustomEntrypoint1();
  sayHiFromCustomEntrypoint2();
  sayHiFromCustomEntrypoint3();
}

@pragma('vm:external-name', 'SayHiFromCustomEntrypoint1')
external void sayHiFromCustomEntrypoint1();
@pragma('vm:external-name', 'SayHiFromCustomEntrypoint2')
external void sayHiFromCustomEntrypoint2();
@pragma('vm:external-name', 'SayHiFromCustomEntrypoint3')
external void sayHiFromCustomEntrypoint3();

@pragma('vm:entry-point')
void terminateExitCodeHandler() {
  Process.runSync('ls', <String>[]);
}

@pragma('vm:entry-point')
void executableNameNotNull() {
  notifyStringValue(Platform.executable);
}

@pragma('vm:entry-point')
void implicitViewNotNull() {
  notifyBoolValue(PlatformDispatcher.instance.implicitView != null);
}

@pragma('vm:external-name', 'NotifyStringValue')
external void notifyStringValue(String value);
@pragma('vm:external-name', 'NotifyBoolValue')
external void notifyBoolValue(bool value);

@pragma('vm:entry-point')
void invokePlatformTaskRunner() {
  PlatformDispatcher.instance.sendPlatformMessage('OhHi', null, null);
}

@pragma('vm:entry-point')
void invokePlatformThreadIsolate() {
  signalNativeTest();
  runOnPlatformThread(ffiSignalNativeTest);
}

Float64List kTestTransform = () {
  final Float64List values = Float64List(16);
  values[0] = 1.0; // scaleX
  values[4] = 2.0; // skewX
  values[12] = 3.0; // transX
  values[1] = 4.0; // skewY
  values[5] = 5.0; // scaleY
  values[13] = 6.0; // transY
  values[3] = 7.0; // pers0
  values[7] = 8.0; // pers1
  values[15] = 9.0; // pers2
  return values;
}();

@pragma('vm:external-name', 'SignalNativeTest')
external void signalNativeTest();
@pragma('vm:external-name', 'SignalNativeCount')
external void signalNativeCount(int count);
@pragma('vm:external-name', 'SignalNativeMessage')
external void signalNativeMessage(String message);
@pragma('vm:external-name', 'NotifySemanticsEnabled')
external void notifySemanticsEnabled(bool enabled);
@pragma('vm:external-name', 'NotifyAccessibilityFeatures')
external void notifyAccessibilityFeatures(bool reduceMotion);
@pragma('vm:external-name', 'NotifySemanticsAction')
external void notifySemanticsAction(int nodeId, int action, List<int> data);

@ffi.Native<ffi.Void Function()>(symbol: 'FFISignalNativeTest')
external void ffiSignalNativeTest();

/// Returns a future that completes when
/// `PlatformDispatcher.instance.onSemanticsEnabledChanged` fires.
Future<void> get semanticsChanged {
  final Completer<void> semanticsChanged = Completer<void>();
  PlatformDispatcher.instance.onSemanticsEnabledChanged = semanticsChanged.complete;
  return semanticsChanged.future;
}

/// Returns a future that completes when
/// `PlatformDispatcher.instance.onAccessibilityFeaturesChanged` fires.
Future<void> get accessibilityFeaturesChanged {
  final Completer<void> featuresChanged = Completer<void>();
  PlatformDispatcher.instance.onAccessibilityFeaturesChanged = featuresChanged.complete;
  return featuresChanged.future;
}

Future<SemanticsActionEvent> get semanticsActionEvent {
  final Completer<SemanticsActionEvent> actionReceived = Completer<SemanticsActionEvent>();
  PlatformDispatcher.instance.onSemanticsActionEvent = (SemanticsActionEvent action) {
    actionReceived.complete(action);
  };
  return actionReceived.future;
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> a11y_main() async {
  // 1: Return initial state (semantics disabled).
  notifySemanticsEnabled(PlatformDispatcher.instance.semanticsEnabled);

  // 2: Await semantics enabled from embedder.
  await semanticsChanged;
  notifySemanticsEnabled(PlatformDispatcher.instance.semanticsEnabled);

  // 3: Return initial state of accessibility features.
  notifyAccessibilityFeatures(PlatformDispatcher.instance.accessibilityFeatures.reduceMotion);

  // 4: Await accessibility features changed from embedder.
  await accessibilityFeaturesChanged;
  notifyAccessibilityFeatures(PlatformDispatcher.instance.accessibilityFeatures.reduceMotion);

  // 5: Fire semantics update.
  final SemanticsUpdateBuilder builder =
      SemanticsUpdateBuilder()
        ..updateNode(
          id: 42,
          identifier: '',
          label: 'A: root',
          labelAttributes: <StringAttribute>[],
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          transform: kTestTransform,
          childrenInTraversalOrder: Int32List.fromList(<int>[84, 96]),
          childrenInHitTestOrder: Int32List.fromList(<int>[96, 84]),
          actions: 0,
          flags: 0,
          maxValueLength: 0,
          currentValueLength: 0,
          textSelectionBase: 0,
          textSelectionExtent: 0,
          platformViewId: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: 'tooltip',
          textDirection: TextDirection.ltr,
          additionalActions: Int32List(0),
        )
        ..updateNode(
          id: 84,
          identifier: '',
          label: 'B: leaf',
          labelAttributes: <StringAttribute>[],
          rect: const Rect.fromLTRB(40.0, 40.0, 80.0, 80.0),
          transform: kTestTransform,
          actions: 0,
          flags: 0,
          maxValueLength: 0,
          currentValueLength: 0,
          textSelectionBase: 0,
          textSelectionExtent: 0,
          platformViewId: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: 'tooltip',
          textDirection: TextDirection.ltr,
          additionalActions: Int32List(0),
          childrenInHitTestOrder: Int32List(0),
          childrenInTraversalOrder: Int32List(0),
        )
        ..updateNode(
          id: 96,
          identifier: '',
          label: 'C: branch',
          labelAttributes: <StringAttribute>[],
          rect: const Rect.fromLTRB(40.0, 40.0, 80.0, 80.0),
          transform: kTestTransform,
          childrenInTraversalOrder: Int32List.fromList(<int>[128]),
          childrenInHitTestOrder: Int32List.fromList(<int>[128]),
          actions: 0,
          flags: 0,
          maxValueLength: 0,
          currentValueLength: 0,
          textSelectionBase: 0,
          textSelectionExtent: 0,
          platformViewId: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: 'tooltip',
          textDirection: TextDirection.ltr,
          additionalActions: Int32List(0),
        )
        ..updateNode(
          id: 128,
          identifier: '',
          label: 'D: leaf',
          labelAttributes: <StringAttribute>[],
          rect: const Rect.fromLTRB(40.0, 40.0, 80.0, 80.0),
          transform: kTestTransform,
          additionalActions: Int32List.fromList(<int>[21]),
          platformViewId: 0x3f3,
          actions: 0,
          flags: 0,
          maxValueLength: 0,
          currentValueLength: 0,
          textSelectionBase: 0,
          textSelectionExtent: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: 'tooltip',
          textDirection: TextDirection.ltr,
          childrenInHitTestOrder: Int32List(0),
          childrenInTraversalOrder: Int32List(0),
        )
        ..updateCustomAction(id: 21, label: 'Archive', hint: 'archive message');

  PlatformDispatcher.instance.views.first.updateSemantics(builder.build());

  signalNativeTest();

  // 6: Await semantics action from embedder.
  final SemanticsActionEvent data = await semanticsActionEvent;
  final List<int> actionArgs = <int>[
    (data.arguments! as ByteData).getInt8(0),
    (data.arguments! as ByteData).getInt8(1),
  ];
  notifySemanticsAction(data.nodeId, data.type.index, actionArgs);

  // 7: Await semantics disabled from embedder.
  await semanticsChanged;
  notifySemanticsEnabled(PlatformDispatcher.instance.semanticsEnabled);
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> a11y_string_attributes() async {
  // 1: Wait until semantics are enabled.
  if (!PlatformDispatcher.instance.semanticsEnabled) {
    await semanticsChanged;
  }

  // 2: Update semantics with string attributes.
  final SemanticsUpdateBuilder builder =
      SemanticsUpdateBuilder()..updateNode(
        id: 42,
        identifier: 'identifier',
        label: 'What is the meaning of life?',
        labelAttributes: <StringAttribute>[
          LocaleStringAttribute(
            range: const TextRange(start: 0, end: 'What is the meaning of life?'.length),
            locale: const Locale('en'),
          ),
          SpellOutStringAttribute(range: const TextRange(start: 0, end: 1)),
        ],
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        transform: kTestTransform,
        childrenInTraversalOrder: Int32List.fromList(<int>[84, 96]),
        childrenInHitTestOrder: Int32List.fromList(<int>[96, 84]),
        actions: 0,
        flags: 0,
        maxValueLength: 0,
        currentValueLength: 0,
        textSelectionBase: 0,
        textSelectionExtent: 0,
        platformViewId: 0,
        scrollChildren: 0,
        scrollIndex: 0,
        scrollPosition: 0.0,
        scrollExtentMax: 0.0,
        scrollExtentMin: 0.0,
        elevation: 0.0,
        thickness: 0.0,
        hint: "It's a number",
        hintAttributes: <StringAttribute>[
          LocaleStringAttribute(
            range: const TextRange(start: 0, end: 1),
            locale: const Locale('en'),
          ),
          LocaleStringAttribute(
            range: const TextRange(start: 2, end: 3),
            locale: const Locale('fr'),
          ),
        ],
        value: '42',
        valueAttributes: <StringAttribute>[
          LocaleStringAttribute(
            range: const TextRange(start: 0, end: '42'.length),
            locale: const Locale('en', 'US'),
          ),
        ],
        increasedValue: '43',
        increasedValueAttributes: <StringAttribute>[
          SpellOutStringAttribute(range: const TextRange(start: 0, end: 1)),
          SpellOutStringAttribute(range: const TextRange(start: 1, end: 2)),
        ],
        decreasedValue: '41',
        decreasedValueAttributes: <StringAttribute>[],
        tooltip: 'tooltip',
        textDirection: TextDirection.ltr,
        additionalActions: Int32List(0),
      );

  PlatformDispatcher.instance.views.first.updateSemantics(builder.build());
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void platform_messages_response() {
  PlatformDispatcher.instance.onPlatformMessage = (
    String name,
    ByteData? data,
    PlatformMessageResponseCallback? callback,
  ) {
    callback!(data);
  };
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void platform_messages_no_response() {
  PlatformDispatcher.instance.onPlatformMessage = (
    String name,
    ByteData? data,
    PlatformMessageResponseCallback? callback,
  ) {
    final Uint8List list = data!.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
    signalNativeMessage(utf8.decode(list));
    // This does nothing because no one is listening on the other side. But complete the loop anyway
    // to make sure all null checking on response handles in the engine is in place.
    callback!(data);
  };
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void null_platform_messages() {
  PlatformDispatcher.instance.onPlatformMessage = (
    String name,
    ByteData? data,
    PlatformMessageResponseCallback? callback,
  ) {
    // This checks if the platform_message null data is converted to Flutter null.
    signalNativeMessage((null == data).toString());
    callback!(data);
  };
  signalNativeTest();
}

Picture createSimplePicture() {
  final Paint blackPaint = Paint();
  final Paint whitePaint = Paint()..color = const Color.fromARGB(255, 255, 255, 255);
  final PictureRecorder baseRecorder = PictureRecorder();
  final Canvas canvas = Canvas(baseRecorder);
  canvas.drawRect(const Rect.fromLTRB(0.0, 0.0, 1000.0, 1000.0), blackPaint);
  canvas.drawRect(const Rect.fromLTRB(10.0, 10.0, 990.0, 990.0), whitePaint);
  return baseRecorder.endRecording();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(const Offset(1.0, 1.0), createSimplePicture());
    builder.pushOffset(1.0, 2.0);
    builder.addPlatformView(42, width: 123.0, height: 456.0);
    builder.addPicture(const Offset(1.0, 1.0), createSimplePicture());
    builder.pop(); // offset
    signalNativeTest(); // Signal 2
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_with_opacity() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();

    // Root node
    builder.pushOffset(1.0, 2.0);

    // First sibling layer (no platform view, should be cached)
    builder.pushOpacity(127);
    builder.addPicture(const Offset(1.0, 1.0), createSimplePicture());
    builder.pop();

    // Second sibling layer (platform view, should not be cached)
    builder.pushOpacity(127);
    builder.addPlatformView(42, width: 123.0, height: 456.0);
    builder.pop();

    // Third sibling layer (no platform view, should be cached)
    builder.pushOpacity(127);
    builder.addPicture(const Offset(2.0, 1.0), createSimplePicture());
    builder.pop();

    signalNativeTest(); // Signal 2
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_with_opacity() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOpacity(127);
    builder.addPicture(const Offset(1.0, 1.0), createSimplePicture());
    builder.pop(); // offset
    signalNativeTest(); // Signal 2
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

Picture createColoredBox(Color color, Size size) {
  final Paint paint = Paint();
  paint.color = color;
  final PictureRecorder baseRecorder = PictureRecorder();
  final Canvas canvas = Canvas(baseRecorder);
  canvas.drawRect(Rect.fromLTRB(0.0, 0.0, size.width, size.height), paint);
  return baseRecorder.endRecording();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_with_known_scene() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Color blue = Color.fromARGB(127, 0, 0, 255);
    const Color gray = Color.fromARGB(127, 127, 127, 127);

    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);

    // 10 (Index 0)
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.pushOffset(20.0, 20.0);
    // 20 (Index 1)
    builder.addPlatformView(1, width: size.width, height: size.height); // green - platform
    builder.pop();

    // 30 (Index 2)
    builder.addPicture(const Offset(30.0, 30.0), createColoredBox(blue, size)); // blue - flutter

    builder.pushOffset(40.0, 40.0);
    // 40 (Index 3)
    builder.addPlatformView(2, width: size.width, height: size.height); // magenta - platform
    builder.pop();

    // 50  (Index 4)
    builder.addPicture(const Offset(50.0, 50.0), createColoredBox(gray, size)); // gray - flutter

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());

    signalNativeTest(); // Signal 2
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_transparent_overlay() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Color transparent = Color(0x00ffffff);

    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);

    // 10 (Index 0)
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.pushOffset(20.0, 20.0);
    // 20 (Index 1)
    builder.addPlatformView(1, width: size.width, height: size.height); // green - platform
    builder.pop();

    // 30 (Index 2)
    builder.addPicture(
      const Offset(30.0, 30.0),
      createColoredBox(transparent, size),
    ); // transparent picture, no layer should be created.

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());

    signalNativeTest(); // Signal 2
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_no_overlay() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);

    // 10 (Index 0)
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.pushOffset(20.0, 20.0);
    // 20 (Index 1)
    builder.addPlatformView(1, width: size.width, height: size.height); // green - platform
    builder.pop();
    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());

    signalNativeTest(); // Signal 2
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_with_root_layer_only() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);

    // 10 (Index 0)
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter
    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());

    signalNativeTest(); // Signal 2
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_composite_platform_views_with_platform_layer_on_bottom() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);

    // 10 (Index 0)
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.pushOffset(20.0, 20.0);
    // 20 (Index 1)
    builder.addPlatformView(1, width: size.width, height: size.height); // green - platform
    builder.pop();
    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());

    signalNativeTest(); // Signal 2
  };
  signalNativeTest(); // Signal 1
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:external-name', 'SignalBeginFrame')
// ignore: unreachable_from_main
external void signalBeginFrame();

@pragma('vm:entry-point')
Future<void>
// ignore: non_constant_identifier_names
texture_destruction_callback_called_without_custom_compositor() async {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Size size = Size(50.0, 150.0);
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_render_scene_without_custom_compositor() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Color green = Color.fromARGB(127, 0, 255, 0);
    const Color blue = Color.fromARGB(127, 0, 0, 255);
    const Color magenta = Color.fromARGB(127, 255, 0, 255);
    const Color gray = Color.fromARGB(127, 127, 127, 127);

    const Size size = Size(50.0, 150.0);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.addPicture(const Offset(20.0, 20.0), createColoredBox(green, size)); // green - flutter

    builder.addPicture(const Offset(30.0, 30.0), createColoredBox(blue, size)); // blue - flutter

    builder.addPicture(
      const Offset(40.0, 40.0),
      createColoredBox(magenta, size),
    ); // magenta - flutter

    builder.addPicture(const Offset(50.0, 50.0), createColoredBox(gray, size)); // gray - flutter

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

Picture createGradientBox(Size size) {
  final Paint paint = Paint();
  final List<Color> rainbow = <Color>[
    const Color.fromARGB(255, 255, 0, 0), // red
    const Color.fromARGB(255, 255, 165, 0), // orange
    const Color.fromARGB(255, 255, 255, 0), // yellow
    const Color.fromARGB(255, 0, 255, 0), // green
    const Color.fromARGB(255, 0, 0, 255), // blue
    const Color.fromARGB(255, 75, 0, 130), // indigo
    const Color.fromARGB(255, 238, 130, 238), // violet
  ];
  final List<double> stops = <double>[
    (1.0 / 7.0),
    (2.0 / 7.0),
    (3.0 / 7.0),
    (4.0 / 7.0),
    (5.0 / 7.0),
    (6.0 / 7.0),
    (7.0 / 7.0),
  ];
  paint.shader = Gradient.linear(Offset.zero, Offset(size.width, size.height), rainbow, stops);
  final PictureRecorder baseRecorder = PictureRecorder();
  final Canvas canvas = Canvas(baseRecorder);
  canvas.drawRect(Rect.fromLTRB(0.0, 0.0, size.width, size.height), paint);
  return baseRecorder.endRecording();
}

@pragma('vm:external-name', 'EchoKeyEvent')
external void _echoKeyEvent(
  int change,
  int timestamp,
  int physical,
  int logical,
  int charCode,
  bool synthesized,
  int deviceType,
);

// Convert `kind` in enum form to its integer form.
//
// It performs a reversed mapping from `UnserializeKeyEventType`
// in shell/platform/embedder/tests/embedder_unittests.cc.
int _serializeKeyEventType(KeyEventType change) {
  switch (change) {
    case KeyEventType.up:
      return 1;
    case KeyEventType.down:
      return 2;
    case KeyEventType.repeat:
      return 3;
  }
}

// Convert `deviceType` in enum form to its integer form.
//
// It performs a reversed mapping from `UnserializeKeyEventDeviceType`
// in shell/platform/embedder/tests/embedder_unittests.cc.
int _serializeKeyEventDeviceType(KeyEventDeviceType deviceType) {
  switch (deviceType) {
    case KeyEventDeviceType.keyboard:
      return 1;
    case KeyEventDeviceType.directionalPad:
      return 2;
    case KeyEventDeviceType.gamepad:
      return 3;
    case KeyEventDeviceType.joystick:
      return 4;
    case KeyEventDeviceType.hdmi:
      return 5;
  }
}

// Echo the event data with `_echoKeyEvent`, and returns synthesized as handled.
@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> key_data_echo() async {
  PlatformDispatcher.instance.onKeyData = (KeyData data) {
    _echoKeyEvent(
      _serializeKeyEventType(data.type),
      data.timeStamp.inMicroseconds,
      data.physical,
      data.logical,
      data.character == null ? 0 : data.character!.codeUnitAt(0),
      data.synthesized,
      _serializeKeyEventDeviceType(data.deviceType),
    );
    return data.synthesized;
  };
  signalNativeTest();
}

// After platform channel 'test/starts_echo' receives a message, starts echoing
// the event data with `_echoKeyEvent`, and returns synthesized as handled.
@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> key_data_late_echo() async {
  channelBuffers.setListener('test/starts_echo', (
    ByteData? data,
    PlatformMessageResponseCallback callback,
  ) {
    PlatformDispatcher.instance.onKeyData = (KeyData data) {
      _echoKeyEvent(
        _serializeKeyEventType(data.type),
        data.timeStamp.inMicroseconds,
        data.physical,
        data.logical,
        data.character == null ? 0 : data.character!.codeUnitAt(0),
        data.synthesized,
        _serializeKeyEventDeviceType(data.deviceType),
      );
      return data.synthesized;
    };
    callback(null);
  });
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_implicit_view() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Size size = Size(800.0, 600.0);
    const Color red = Color.fromARGB(127, 255, 0, 0);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    builder.addPicture(Offset.zero, createColoredBox(red, size));

    builder.pop();

    PlatformDispatcher.instance.implicitView?.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_all_views() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    for (final FlutterView view in PlatformDispatcher.instance.views) {
      const Size size = Size(800.0, 600.0);
      const Color red = Color.fromARGB(127, 255, 0, 0);

      final SceneBuilder builder = SceneBuilder();

      builder.pushOffset(0.0, 0.0);

      builder.addPicture(Offset.zero, createColoredBox(red, size));

      builder.pop();

      view.render(builder.build());
    }
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_gradient() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Size size = Size(800.0, 600.0);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    builder.addPicture(Offset.zero, createGradientBox(size)); // gradient - flutter

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_texture() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Size size = Size(800.0, 600.0);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    builder.addTexture(/*textureId*/ 1, width: size.width, height: size.height);

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_gradient_on_non_root_backing_store() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Size size = Size(800.0, 600.0);
    const Color red = Color.fromARGB(127, 255, 0, 0);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    // Even though this is occluded, add something so it is not elided.
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter

    builder.addPlatformView(1, width: 100, height: 200); // undefined - platform

    builder.addPicture(Offset.zero, createGradientBox(size)); // gradient - flutter

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void verify_b141980393() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    // The platform view in the test case is screen sized but with margins of 31
    // and 37 points from the top and bottom.
    const double topMargin = 31.0;
    const double bottomMargin = 37.0;
    const Size platformViewSize = Size(800.0, 600.0 - topMargin - bottomMargin);

    final SceneBuilder builder = SceneBuilder();

    builder.pushOffset(
      0.0, // x
      topMargin, // y
    );

    // The web view in example.
    builder.addPlatformView(1337, width: platformViewSize.width, height: platformViewSize.height);

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_display_platform_view_with_pixel_ratio() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushTransform(
      Float64List.fromList(<double>[
        2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
      ]),
    ); // base
    builder.addPicture(Offset.zero, createGradientBox(const Size(400.0, 300.0)));
    builder.pushOffset(0.0, 20.0); // offset
    builder.addPlatformView(42, width: 400.0, height: 280.0);
    builder.pop(); // offset
    builder.addPicture(
      Offset.zero,
      createColoredBox(const Color.fromARGB(128, 255, 0, 0), const Size(400.0, 300.0)),
    );
    builder.pop(); // base
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_receive_locale_updates() {
  PlatformDispatcher.instance.onLocaleChanged = () {
    signalNativeCount(PlatformDispatcher.instance.locales.length);
  };
  signalNativeTest();
}

// Verifies behavior tracked in https://github.com/flutter/flutter/issues/43732
@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void verify_b143464703() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0); // base

    // Background
    builder.addPicture(
      Offset.zero,
      createColoredBox(const Color.fromARGB(255, 128, 128, 128), const Size(1024.0, 600.0)),
    );

    builder.pushOpacity(128);
    builder.addPicture(
      const Offset(10.0, 10.0),
      createColoredBox(const Color.fromARGB(255, 0, 0, 255), const Size(25.0, 25.0)),
    );
    builder.pop(); // opacity 128

    // The top bar and the platform view are pushed to the side.
    builder.pushOffset(135.0, 0.0); // 1
    builder.pushOpacity(128); // opacity

    // Platform view offset from the top
    builder.pushOffset(0.0, 60.0); // 2
    builder.addPlatformView(42, width: 1024.0, height: 540.0);
    builder.pop(); // 2

    // Top bar
    builder.addPicture(Offset.zero, createGradientBox(const Size(1024.0, 60.0)));

    builder.pop(); // opacity
    builder.pop(); // 1

    builder.pop(); // base
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void push_frames_over_and_over() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    builder.addPicture(
      Offset.zero,
      createColoredBox(const Color.fromARGB(255, 128, 128, 128), const Size(1024.0, 600.0)),
    );
    builder.pushOpacity(128);
    builder.addPlatformView(42, width: 1024.0, height: 540.0);
    builder.pop();
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
    signalNativeTest();
    PlatformDispatcher.instance.scheduleFrame();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void platform_view_mutators() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0); // base
    builder.addPicture(Offset.zero, createGradientBox(const Size(800.0, 600.0)));

    builder.pushOpacity(128);
    builder.pushClipRect(const Rect.fromLTWH(10.0, 10.0, 800.0 - 20.0, 600.0 - 20.0));
    builder.pushClipRRect(
      RRect.fromLTRBR(10.0, 10.0, 800.0 - 10.0, 600.0 - 10.0, const Radius.circular(14.0)),
    );
    builder.addPlatformView(42, width: 800.0, height: 600.0);
    builder.pop(); // clip rrect
    builder.pop(); // clip rect
    builder.pop(); // opacity

    builder.pop(); // base
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void platform_view_mutators_with_pixel_ratio() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0); // base
    builder.addPicture(Offset.zero, createGradientBox(const Size(400.0, 300.0)));

    builder.pushOpacity(128);
    builder.pushClipRect(const Rect.fromLTWH(5.0, 5.0, 400.0 - 10.0, 300.0 - 10.0));
    builder.pushClipRRect(
      RRect.fromLTRBR(5.0, 5.0, 400.0 - 5.0, 300.0 - 5.0, const Radius.circular(7.0)),
    );
    builder.addPlatformView(42, width: 400.0, height: 300.0);
    builder.pop(); // clip rrect
    builder.pop(); // clip rect
    builder.pop(); // opacity

    builder.pop(); // base
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void empty_scene() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    PlatformDispatcher.instance.views.first.render(SceneBuilder().build());
    signalNativeTest();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void scene_with_no_container() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, createGradientBox(const Size(400.0, 300.0)));
    PlatformDispatcher.instance.views.first.render(builder.build());
    signalNativeTest();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

Picture createArcEndCapsPicture() {
  final PictureRecorder baseRecorder = PictureRecorder();
  final Canvas canvas = Canvas(baseRecorder);

  final style =
      Paint()
        ..strokeWidth = 12.0
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.miter;

  style.color = const Color.fromARGB(255, 255, 0, 0);
  canvas.drawArc(const Rect.fromLTRB(0.0, 0.0, 500.0, 500.0), 1.57, 1.0, false, style);

  return baseRecorder.endRecording();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void arc_end_caps_correct() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, createArcEndCapsPicture());
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void scene_builder_with_clips() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushClipRect(const Rect.fromLTRB(10.0, 10.0, 390.0, 290.0));
    builder.addPlatformView(42, width: 400.0, height: 300.0);
    builder.addPicture(Offset.zero, createGradientBox(const Size(400.0, 300.0)));
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void scene_builder_with_complex_clips() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();

    builder.pushClipRect(const Rect.fromLTRB(0.0, 0.0, 1024.0, 600.0));
    builder.pushOffset(512.0, 0.0);
    builder.pushClipRect(const Rect.fromLTRB(0.0, 0.0, 512.0, 600.0));
    builder.pushOffset(-256.0, 0.0);
    builder.pushClipRect(const Rect.fromLTRB(0.0, 0.0, 1024.0, 600.0));
    builder.addPlatformView(42, width: 1024.0, height: 600.0);

    builder.addPicture(Offset.zero, createGradientBox(const Size(1024.0, 600.0)));
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:external-name', 'SendObjectToNativeCode')
external void sendObjectToNativeCode(dynamic object);

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void objects_can_be_posted() {
  final ReceivePort port = ReceivePort();
  port.listen((dynamic message) {
    sendObjectToNativeCode(message);
  });
  signalNativeCount(port.sendPort.nativePort);
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void empty_scene_posts_zero_layers_to_compositor() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    // Should not render anything.
    builder.pushClipRect(const Rect.fromLTRB(0.0, 0.0, 300.0, 200.0));
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void compositor_can_post_only_platform_views() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.addPlatformView(42, width: 300.0, height: 200.0);
    builder.addPlatformView(24, width: 300.0, height: 200.0);
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_targets_are_recycled() {
  int frameCount = 0;
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    for (int i = 0; i < 10; i++) {
      builder.addPicture(Offset.zero, createGradientBox(const Size(30.0, 20.0)));
      builder.addPlatformView(42 + i, width: 30.0, height: 20.0);
    }
    PlatformDispatcher.instance.views.first.render(builder.build());
    frameCount++;
    if (frameCount == 8) {
      signalNativeTest();
    } else {
      PlatformDispatcher.instance.scheduleFrame();
    }
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_targets_are_in_stable_order() {
  int frameCount = 0;
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    for (int i = 0; i < 10; i++) {
      builder.addPicture(Offset.zero, createGradientBox(const Size(30.0, 20.0)));
      builder.addPlatformView(42 + i, width: 30.0, height: 20.0);
    }
    PlatformDispatcher.instance.views.first.render(builder.build());
    PlatformDispatcher.instance.scheduleFrame();
    frameCount++;
    if (frameCount == 8) {
      signalNativeTest();
    }
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:external-name', 'NativeArgumentsCallback')
external void nativeArgumentsCallback(List<String> args);

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void custom_logger(List<String> args) {
  print('hello world');
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void dart_entrypoint_args(List<String> args) {
  nativeArgumentsCallback(args);
}

@pragma('vm:external-name', 'SnapshotsCallback')
external void snapshotsCallback(Image bigImage, Image smallImage);

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> snapshot_large_scene(int maxSize) async {
  // Set width to double the max size, which will result in height being half the max size after scaling.
  final width = maxSize * 2.0, height = maxSize.toDouble();

  PictureRecorder recorder = PictureRecorder();
  {
    final Canvas canvas = Canvas(recorder, Rect.fromLTWH(0, 0, width, height));
    final Paint paint = Paint();
    // Bottom left
    paint.color = const Color.fromARGB(255, 100, 255, 100);
    canvas.drawRect(Rect.fromLTWH(0, height / 2, width / 2, height / 2), paint);
    // Top right
    paint.color = const Color.fromARGB(255, 100, 100, 255);
    canvas.drawRect(Rect.fromLTWH(width / 2, 0, width / 2, height / 2), paint);
  }
  Picture picture = recorder.endRecording();
  final Image bigImage = await picture.toImage(width.toInt(), height.toInt());

  // The max size varies across hardware/drivers, so normalize the result to a smaller target size in
  // order to reliably test against an image fixture.
  const smallWidth = 128.0;
  const smallHeight = 64.0;
  recorder = PictureRecorder();
  {
    final Canvas canvas = Canvas(recorder, const Rect.fromLTWH(0, 0, smallWidth, smallHeight));
    canvas.scale(smallWidth / bigImage.width);
    canvas.drawImage(bigImage, Offset.zero, Paint());
  }
  picture = recorder.endRecording();
  final Image smallImage = await picture.toImage(smallWidth.toInt(), smallHeight.toInt());

  snapshotsCallback(bigImage, smallImage);
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void invalid_backingstore() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Color red = Color.fromARGB(127, 255, 0, 0);
    const Size size = Size(50.0, 150.0);
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    builder.addPicture(const Offset(10.0, 10.0), createColoredBox(red, size)); // red - flutter
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.onDrawFrame = () {
    signalNativeTest();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void can_schedule_frame() {
  PlatformDispatcher.instance.onBeginFrame = (Duration beginTime) {
    signalNativeCount(beginTime.inMicroseconds);
  };
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void add_view_schedules_frame() {
  PlatformDispatcher.instance.onBeginFrame = (Duration beginTime) {
    for (final FlutterView view in PlatformDispatcher.instance.views) {
      if (view.viewId == 123) {
        signalNativeCount(beginTime.inMicroseconds);
      }
    }
  };
  signalNativeTest();
}

void drawSolidColor(Color c) {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    builder.addPicture(
      Offset.zero,
      createColoredBox(c, PlatformDispatcher.instance.views.first.physicalSize),
    );
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void draw_solid_red() {
  drawSolidColor(const Color.fromARGB(255, 255, 0, 0));
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void draw_solid_green() {
  drawSolidColor(const Color.fromARGB(255, 0, 255, 0));
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void draw_solid_blue() {
  drawSolidColor(const Color.fromARGB(255, 0, 0, 255));
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void pointer_data_packet() {
  PlatformDispatcher.instance.onPointerDataPacket = (PointerDataPacket packet) {
    signalNativeCount(packet.data.length);

    for (final PointerData pointerData in packet.data) {
      signalNativeMessage(pointerData.toString());
    }
  };

  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void pointer_data_packet_view_id() {
  PlatformDispatcher.instance.onPointerDataPacket = (PointerDataPacket packet) {
    assert(packet.data.length == 1);

    for (final PointerData pointerData in packet.data) {
      signalNativeMessage('ViewID: ${pointerData.viewId}');
    }
  };

  signalNativeTest();
}

Map<int, Size> _getAllViewSizes() {
  final Map<int, Size> result = <int, Size>{};
  for (final FlutterView view in PlatformDispatcher.instance.views) {
    result[view.viewId] = view.physicalSize;
  }
  return result;
}

List<int> _findDifferences(Map<int, Size> a, Map<int, Size> b) {
  final Set<int> result = <int>{};
  a.forEach((int viewId, Size sizeA) {
    if (!b.containsKey(viewId) || b[viewId] != sizeA) {
      result.add(viewId);
    }
  });
  b.forEach((int viewId, Size sizeB) {
    if (!a.containsKey(viewId)) {
      result.add(viewId);
    }
  });
  return result.toList()..sort();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void window_metrics_event_view_id() {
  Map<int, Size> sizes = _getAllViewSizes();
  PlatformDispatcher.instance.onMetricsChanged = () {
    final Map<int, Size> newSizes = _getAllViewSizes();
    final List<int> differences = _findDifferences(sizes, newSizes);
    sizes = newSizes;
    signalNativeMessage('Changed: $differences');
  };

  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void window_metrics_event_all_view_ids() {
  PlatformDispatcher.instance.onMetricsChanged = () {
    final List<int> viewIds = PlatformDispatcher.instance.views.map((view) => view.viewId).toList();

    viewIds.sort();

    signalNativeMessage('View IDs: [${viewIds.join(', ')}]');
  };

  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> channel_listener_response() async {
  channelBuffers.setListener('test/listen', (
    ByteData? data,
    PlatformMessageResponseCallback callback,
  ) {
    callback(null);
  });
  signalNativeTest();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_gradient_retained() {
  OffsetEngineLayer? offsetLayer; // Retain the offset layer.
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const Size size = Size(800.0, 600.0);

    final SceneBuilder builder = SceneBuilder();

    offsetLayer = builder.pushOffset(0.0, 0.0, oldLayer: offsetLayer);

    // display_list_layer will comparing the display_list
    // no need to retain the picture
    builder.addPicture(Offset.zero, createGradientBox(size));

    builder.pop();

    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_impeller_test() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    final Paint paint = Paint();
    paint.color = const Color.fromARGB(255, 0, 0, 255);
    final PictureRecorder baseRecorder = PictureRecorder();
    final Canvas canvas = Canvas(baseRecorder);
    canvas.drawPaint(Paint()..color = const Color.fromARGB(255, 255, 0, 0));
    canvas.drawRect(const Rect.fromLTRB(20.0, 20.0, 200.0, 150.0), paint);
    builder.addPicture(Offset.zero, baseRecorder.endRecording());
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
void render_impeller_text_test() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(0.0, 0.0);
    final Paint paint = Paint();
    paint.color = const Color.fromARGB(255, 0, 0, 255);
    final PictureRecorder baseRecorder = PictureRecorder();
    final Canvas canvas = Canvas(baseRecorder);

    final ParagraphBuilder paragraphBuilder = ParagraphBuilder(
      ParagraphStyle(fontFamily: 'sans-serif'),
    )..addText('Flutter is the best!');
    final Paragraph paragraph =
        paragraphBuilder.build()..layout(const ParagraphConstraints(width: 400));
    canvas.drawParagraph(paragraph, const Offset(20, 20));

    builder.addPicture(Offset.zero, baseRecorder.endRecording());
    builder.pop();
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
// ignore: non_constant_identifier_names
Future<void> render_impeller_image_snapshot_test() async {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  const Color color = Color.fromARGB(255, 0, 0, 123);
  canvas.drawPaint(Paint()..color = color);
  final Picture picture = recorder.endRecording();

  final Image image = await picture.toImage(100, 100);
  final ByteData? imageData = await image.toByteData();
  final int pixel = imageData!.getInt32(0);

  final bool result = (pixel & 0xFF) == color.alpha && ((pixel >> 8) & 0xFF) == color.blue;
  notifyBoolValue(result);
}
