// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;
import 'dart:typed_data' show ByteData, Uint8List;
import 'dart:ui' as ui;

// Signals a waiting latch in the native test.
@pragma('vm:external-name', 'Signal')
external void signal();

// Signals a waiting latch in the native test, passing a boolean value.
@pragma('vm:external-name', 'SignalBoolValue')
external void signalBoolValue(bool value);

// Signals a waiting latch in the native test, passing a string value.
@pragma('vm:external-name', 'SignalStringValue')
external void signalStringValue(String value);

// Signals a waiting latch in the native test, which returns a value to the fixture.
@pragma('vm:external-name', 'SignalBoolReturn')
external bool signalBoolReturn();

// Notify the native test that the first frame has been scheduled.
@pragma('vm:external-name', 'NotifyFirstFrameScheduled')
external void notifyFirstFrameScheduled();

void main() {}

@pragma('vm:entry-point')
void hiPlatformChannels() {
  ui.channelBuffers.setListener('hi', (
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) async {
    ui.PlatformDispatcher.instance.sendPlatformMessage('hi', data, (ByteData? reply) {
      ui.PlatformDispatcher.instance.sendPlatformMessage('hi', reply, (ByteData? reply) {});
    });
    callback(data);
  });
}

/// Returns a future that completes when
/// `PlatformDispatcher.instance.onSemanticsEnabledChanged` fires.
Future<void> get semanticsChanged {
  final Completer<void> semanticsChanged = Completer<void>();
  ui.PlatformDispatcher.instance.onSemanticsEnabledChanged = semanticsChanged.complete;
  return semanticsChanged.future;
}

@pragma('vm:entry-point')
Future<void> sendAccessibilityAnnouncement() async {
  // Wait until semantics are enabled.
  if (!ui.PlatformDispatcher.instance.semanticsEnabled) {
    await semanticsChanged;
  }

  // Standard message codec magic number identifiers.
  // See: https://github.com/flutter/flutter/blob/ee94fe262b63b0761e8e1f889ae52322fef068d2/packages/flutter/lib/src/services/message_codecs.dart#L262
  const int valueMap = 13, valueString = 7;

  // Corresponds to: {"type": "announce", "data": {"message": "hello"}}
  // See: https://github.com/flutter/flutter/blob/b781da9b5822de1461a769c3b245075359f5464d/packages/flutter/lib/src/semantics/semantics_event.dart#L86
  final Uint8List data = Uint8List.fromList([
    // Map with 2 entries
    valueMap, 2,
    // Map key: "type"
    valueString, 'type'.length, ...'type'.codeUnits,
    // Map value: "announce"
    valueString, 'announce'.length, ...'announce'.codeUnits,
    // Map key: "data"
    valueString, 'data'.length, ...'data'.codeUnits,
    // Map value: map with 1 entry
    valueMap, 1,
    // Map key: "message"
    valueString, 'message'.length, ...'message'.codeUnits,
    // Map value: "hello"
    valueString, 'hello'.length, ...'hello'.codeUnits,
  ]);
  final ByteData byteData = data.buffer.asByteData();

  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/accessibility',
    byteData,
    (ByteData? _) => signal(),
  );
}

@pragma('vm:entry-point')
Future<void> sendAccessibilityTooltipEvent() async {
  // Wait until semantics are enabled.
  if (!ui.PlatformDispatcher.instance.semanticsEnabled) {
    await semanticsChanged;
  }

  // Standard message codec magic number identifiers.
  // See: https://github.com/flutter/flutter/blob/ee94fe262b63b0761e8e1f889ae52322fef068d2/packages/flutter/lib/src/services/message_codecs.dart#L262
  const int valueMap = 13, valueString = 7;

  // Corresponds to: {"type": "tooltip", "data": {"message": "hello"}}
  // See: https://github.com/flutter/flutter/blob/b781da9b5822de1461a769c3b245075359f5464d/packages/flutter/lib/src/semantics/semantics_event.dart#L120
  final Uint8List data = Uint8List.fromList([
    // Map with 2 entries
    valueMap, 2,
    // Map key: "type"
    valueString, 'type'.length, ...'type'.codeUnits,
    // Map value: "tooltip"
    valueString, 'tooltip'.length, ...'tooltip'.codeUnits,
    // Map key: "data"
    valueString, 'data'.length, ...'data'.codeUnits,
    // Map value: map with 1 entry
    valueMap, 1,
    // Map key: "message"
    valueString, 'message'.length, ...'message'.codeUnits,
    // Map value: "hello"
    valueString, 'hello'.length, ...'hello'.codeUnits,
  ]);
  final ByteData byteData = data.buffer.asByteData();

  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/accessibility',
    byteData,
    (ByteData? _) => signal(),
  );
}

@pragma('vm:entry-point')
Future<void> exitTestExit() async {
  final Completer<ByteData?> closed = Completer<ByteData?>();
  ui.channelBuffers.setListener('flutter/platform', (
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) async {
    final String jsonString = json.encode(<Map<String, String>>[
      {'response': 'exit'},
    ]);
    final ByteData responseData = ByteData.sublistView(utf8.encode(jsonString));
    callback(responseData);
    closed.complete(data);
  });
  await closed.future;
}

@pragma('vm:entry-point')
Future<void> exitTestCancel() async {
  final Completer<ByteData?> closed = Completer<ByteData?>();
  ui.channelBuffers.setListener('flutter/platform', (
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) async {
    final String jsonString = json.encode(<Map<String, String>>[
      {'response': 'cancel'},
    ]);
    final ByteData responseData = ByteData.sublistView(utf8.encode(jsonString));
    callback(responseData);
    closed.complete(data);
  });
  await closed.future;

  // Because the request was canceled, the below shall execute.
  final Completer<ByteData?> exited = Completer<ByteData?>();
  final String jsonString = json.encode(<String, dynamic>{
    'method': 'System.exitApplication',
    'args': <String, dynamic>{'type': 'required', 'exitCode': 0},
  });
  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform',
    ByteData.sublistView(utf8.encode(jsonString)),
    (ByteData? reply) {
      exited.complete(reply);
    },
  );
  await exited.future;
}

@pragma('vm:entry-point')
Future<void> enableLifecycleTest() async {
  final Completer<ByteData?> finished = Completer<ByteData?>();
  ui.channelBuffers.setListener('flutter/lifecycle', (
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) async {
    if (data != null) {
      ui.PlatformDispatcher.instance.sendPlatformMessage('flutter/unittest', data, (
        ByteData? reply,
      ) {
        finished.complete();
      });
    }
  });
  await finished.future;
}

@pragma('vm:entry-point')
Future<void> enableLifecycleToFrom() async {
  ui.channelBuffers.setListener('flutter/lifecycle', (
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) async {
    if (data != null) {
      ui.PlatformDispatcher.instance.sendPlatformMessage(
        'flutter/unittest',
        data,
        (ByteData? reply) {},
      );
    }
  });
  final Completer<ByteData?> enabledLifecycle = Completer<ByteData?>();
  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform',
    ByteData.sublistView(utf8.encode('{"method":"System.initializationComplete"}')),
    (ByteData? data) {
      enabledLifecycle.complete(data);
    },
  );
}

@pragma('vm:entry-point')
Future<void> sendCreatePlatformViewMethod() async {
  // The platform view method channel uses the standard method codec.
  // See https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/services/message_codecs.dart#L262
  // for the implementation of the encoding and magic number identifiers.
  const int valueString = 7;
  const int valueMap = 13;
  const int valueInt32 = 3;
  const String method = 'create';
  const String typeKey = 'viewType';
  const String typeValue = 'type';
  const String idKey = 'id';
  final List<int> data = <int>[
    // Method name
    valueString, method.length, ...utf8.encode(method),
    // Method arguments: {'type': 'type':, 'id': 0}
    valueMap, 2,
    valueString, typeKey.length, ...utf8.encode(typeKey),
    valueString, typeValue.length, ...utf8.encode(typeValue),
    valueString, idKey.length, ...utf8.encode(idKey),
    valueInt32, 0, 0, 0, 0,
  ];

  final Completer<ByteData?> completed = Completer<ByteData?>();
  final ByteData bytes = ByteData.sublistView(Uint8List.fromList(data));
  ui.PlatformDispatcher.instance.sendPlatformMessage('flutter/platform_views', bytes, (
    ByteData? response,
  ) {
    completed.complete(response);
  });
  await completed.future;
}

@pragma('vm:entry-point')
Future<void> sendGetKeyboardState() async {
  // The keyboard method channel uses the standard method codec.
  // See https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/services/message_codecs.dart#L262
  // for the implementation of the encoding and magic number identifiers.
  const int valueNull = 0;
  const int valueString = 7;
  const int valueMap = 13;

  const String method = 'getKeyboardState';
  final List<int> data = <int>[
    // Method name
    valueString, method.length, ...utf8.encode(method),
    // Method arguments: null
    valueNull, 2,
  ];

  final Completer<void> completer = Completer<void>();
  final ByteData bytes = ByteData.sublistView(Uint8List.fromList(data));
  ui.PlatformDispatcher.instance.sendPlatformMessage('flutter/keyboard', bytes, (
    ByteData? response,
  ) {
    // For magic numbers for decoding a reply envelope, see:
    // https://github.com/flutter/flutter/blob/67271f69f7f88a4edba6d8023099e3bd27a072d2/packages/flutter/lib/src/services/message_codecs.dart#L577-L587
    const int replyEnvelopeSuccess = 0;

    // Ensure the response is a success containing a map of keyboard states.
    if (response == null) {
      signalStringValue('Unexpected null response');
    } else if (response.lengthInBytes < 2) {
      signalStringValue('Unexpected response length of ${response.lengthInBytes} bytes');
    } else if (response.getUint8(0) != replyEnvelopeSuccess) {
      signalStringValue('Unexpected response envelope status: ${response.getUint8(0)}');
    } else if (response.getUint8(1) != valueMap) {
      signalStringValue('Unexpected response value magic number: ${response.getUint8(1)}');
    } else {
      signalStringValue('Success');
    }
    completer.complete();
  });
  await completer.future;
}

@pragma('vm:entry-point')
void customEntrypoint() {}

@pragma('vm:entry-point')
void verifyNativeFunction() {
  signal();
}

@pragma('vm:entry-point')
void verifyNativeFunctionWithParameters() {
  signalBoolValue(true);
}

@pragma('vm:entry-point')
void verifyNativeFunctionWithReturn() {
  final value = signalBoolReturn();
  signalBoolValue(value);
}

@pragma('vm:entry-point')
void readPlatformExecutable() {
  signalStringValue(io.Platform.executable);
}

@pragma('vm:entry-point')
void drawHelloWorld() {
  ui.PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final ui.ParagraphBuilder paragraphBuilder = ui.ParagraphBuilder(ui.ParagraphStyle())
      ..addText('Hello world');
    final ui.Paragraph paragraph = paragraphBuilder.build();

    paragraph.layout(const ui.ParagraphConstraints(width: 800.0));

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);

    canvas.drawParagraph(paragraph, ui.Offset.zero);

    final ui.Picture picture = recorder.endRecording();
    final ui.SceneBuilder sceneBuilder =
        ui.SceneBuilder()
          ..addPicture(ui.Offset.zero, picture)
          ..pop();

    ui.PlatformDispatcher.instance.implicitView?.render(sceneBuilder.build());
  };

  ui.PlatformDispatcher.instance.scheduleFrame();
  notifyFirstFrameScheduled();
}

ui.Picture _createColoredBox(ui.Color color, ui.Size size) {
  final ui.Paint paint = ui.Paint();
  paint.color = color;
  final ui.PictureRecorder baseRecorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(baseRecorder);
  canvas.drawRect(ui.Rect.fromLTRB(0.0, 0.0, size.width, size.height), paint);
  return baseRecorder.endRecording();
}

@pragma('vm:entry-point')
void renderImplicitView() {
  ui.PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    const ui.Size size = ui.Size(800.0, 600.0);
    const ui.Color red = ui.Color.fromARGB(127, 255, 0, 0);

    final ui.SceneBuilder builder = ui.SceneBuilder();

    builder.pushOffset(0.0, 0.0);

    builder.addPicture(ui.Offset.zero, _createColoredBox(red, size));

    builder.pop();

    ui.PlatformDispatcher.instance.implicitView?.render(builder.build());
  };
  ui.PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void signalViewIds() {
  final Iterable<ui.FlutterView> views = ui.PlatformDispatcher.instance.views;
  final List<int> viewIds = views.map((ui.FlutterView view) => view.viewId).toList();

  viewIds.sort();

  signalStringValue('View IDs: [${viewIds.join(', ')}]');
}

@pragma('vm:entry-point')
void onMetricsChangedSignalViewIds() {
  ui.PlatformDispatcher.instance.onMetricsChanged = () {
    signalViewIds();
  };

  signal();
}
