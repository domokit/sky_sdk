// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert' show json, utf8;
import 'dart:isolate';
import 'dart:typed_data';
import 'dart:ui';

void expect(Object? a, Object? b) {
  if (a != b) {
    throw AssertionError('Expected $a to == $b');
  }
}

void main() {}

@pragma('vm:entry-point')
void mainNotifyNative() {
  notifyNative();
}

@pragma('vm:external-name', 'NativeReportTimingsCallback')
external void nativeReportTimingsCallback(List<int> timings);
@pragma('vm:external-name', 'NativeOnBeginFrame')
external void nativeOnBeginFrame(int microseconds);
@pragma('vm:external-name', 'NativeOnPointerDataPacket')
external void nativeOnPointerDataPacket(List<int> sequences);

@pragma('vm:entry-point')
void onErrorA() {
  PlatformDispatcher.instance.onError = (Object error, StackTrace? stack) {
    notifyErrorA(error.toString());
    return true;
  };
  Future<void>.delayed(const Duration(seconds: 2)).then((_) {
    throw Exception('I should be coming from A');
  });
}

@pragma('vm:entry-point')
void onErrorB() {
  PlatformDispatcher.instance.onError = (Object error, StackTrace? stack) {
    notifyErrorB(error.toString());
    return true;
  };
  throw Exception('I should be coming from B');
}

@pragma('vm:external-name', 'NotifyErrorA')
external void notifyErrorA(String message);
@pragma('vm:external-name', 'NotifyErrorB')
external void notifyErrorB(String message);

@pragma('vm:entry-point')
void drawFrames() {
  // Wait for native to tell us to start going.
  notifyNative();

  PlatformDispatcher.instance.onBeginFrame = (Duration beginTime) {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = const Color(0xFFABCDEF));
    final Picture picture = recorder.endRecording();
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    window.render(scene);

    scene.dispose();
    picture.dispose();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void reportTimingsMain() {
  PlatformDispatcher.instance.onReportTimings = (List<FrameTiming> timings) {
    final timestamps = <int>[];
    for (final t in timings) {
      for (final phase in FramePhase.values) {
        timestamps.add(t.timestampInMicroseconds(phase));
      }
    }
    nativeReportTimingsCallback(timestamps);
    PlatformDispatcher.instance.onReportTimings = (List<FrameTiming> timings) {};
  };
}

@pragma('vm:entry-point')
void onBeginFrameMain() {
  PlatformDispatcher.instance.onBeginFrame = (Duration beginTime) {
    nativeOnBeginFrame(beginTime.inMicroseconds);
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void onPointerDataPacketMain() {
  PlatformDispatcher.instance.onPointerDataPacket = (PointerDataPacket packet) {
    final sequence = <int>[];
    for (final data in packet.data) {
      sequence.add(PointerChange.values.indexOf(data.change));
    }
    nativeOnPointerDataPacket(sequence);
  };
}

@pragma('vm:entry-point')
void emptyMain() {}

@pragma('vm:entry-point')
void reportMetrics() {
  window.onMetricsChanged = () {
    _reportMetrics(window.devicePixelRatio, window.physicalSize.width, window.physicalSize.height);
  };
}

@pragma('vm:external-name', 'ReportMetrics')
external void _reportMetrics(double devicePixelRatio, double width, double height);

@pragma('vm:entry-point')
void dummyReportTimingsMain() {
  PlatformDispatcher.instance.onReportTimings = (List<FrameTiming> timings) {};
}

@pragma('vm:entry-point')
void fixturesAreFunctionalMain() {
  sayHiFromFixturesAreFunctionalMain();
}

@pragma('vm:external-name', 'SayHiFromFixturesAreFunctionalMain')
external void sayHiFromFixturesAreFunctionalMain();

@pragma('vm:entry-point')
@pragma('vm:external-name', 'NotifyNative')
external void notifyNative();

@pragma('vm:entry-point')
void thousandCallsToNative() {
  for (int i = 0; i < 1000; i++) {
    notifyNative();
  }
}

void secondaryIsolateMain(String message) {
  print('Secondary isolate got message: $message');
  notifyNative();
}

@pragma('vm:entry-point')
void testCanLaunchSecondaryIsolate() {
  Isolate.spawn(secondaryIsolateMain, 'Hello from root isolate.');
  notifyNative();
}

@pragma('vm:entry-point')
void testSkiaResourceCacheSendsResponse() {
  void callback(ByteData? data) {
    if (data == null) {
      throw AssertionError('Response must not be null.');
    }
    final String response = utf8.decode(data.buffer.asUint8List());
    final jsonResponse = (json.decode(response) as List).cast<bool>();
    if (!jsonResponse[0]) {
      throw AssertionError('Response was not true');
    }
    notifyNative();
  }

  const String jsonRequest = '''
{
  "method": "Skia.setResourceCacheMaxBytes",
  "args": 10000
}''';
  PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/skia',
    ByteData.sublistView(utf8.encode(jsonRequest)),
    callback,
  );
}

@pragma('vm:external-name', 'NotifyWidthHeight')
external void notifyWidthHeight(int width, int height);

@pragma('vm:entry-point')
void canCreateImageFromDecompressedData() {
  const int imageWidth = 10;
  const int imageHeight = 10;
  final Uint8List pixels = Uint8List.fromList(
    List<int>.generate(imageWidth * imageHeight * 4, (int i) => i % 4 < 2 ? 0x00 : 0xFF),
  );

  decodeImageFromPixels(pixels, imageWidth, imageHeight, PixelFormat.rgba8888, (Image image) {
    notifyWidthHeight(image.width, image.height);
  });
}

@pragma('vm:entry-point')
void canAccessIsolateLaunchData() {
  notifyMessage(
    utf8.decode(PlatformDispatcher.instance.getPersistentIsolateData()!.buffer.asUint8List()),
  );
}

@pragma('vm:entry-point')
void performanceModeImpactsNotifyIdle() {
  notifyNativeBool(false);
  PlatformDispatcher.instance.requestDartPerformanceMode(DartPerformanceMode.latency);
  notifyNativeBool(true);
  PlatformDispatcher.instance.requestDartPerformanceMode(DartPerformanceMode.balanced);
}

@pragma('vm:entry-point')
void callNotifyDestroyed() {
  notifyDestroyed();
}

@pragma('vm:external-name', 'NotifyMessage')
external void notifyMessage(String string);

@pragma('vm:entry-point')
void canRegisterImageDecoders() {
  decodeImageFromList(
    // The test ImageGenerator will always behave the same regardless of input.
    Uint8List(1),
    (Image result) {
      notifyWidthHeight(result.width, result.height);
    },
  );
}

@pragma('vm:external-name', 'NotifyLocalTime')
external void notifyLocalTime(String string);

@pragma('vm:external-name', 'WaitFixture')
external bool waitFixture();

// Return local date-time as a string, to an hour resolution.  So, "2020-07-23
// 14:03:22" will become "2020-07-23 14".
String localTimeAsString() {
  final now = DateTime.now().toLocal();
  // This is: "$y-$m-$d $h:$min:$sec.$ms$us";
  final timeStr = now.toString();
  // Forward only "$y-$m-$d $h" for timestamp comparison.  Not using DateTime
  // formatting since package:intl is not available.
  return timeStr.split(':')[0];
}

@pragma('vm:entry-point')
void localtimesMatch() {
  notifyLocalTime(localTimeAsString());
}

@pragma('vm:entry-point')
void timezonesChange() {
  do {
    notifyLocalTime(localTimeAsString());
  } while (waitFixture());
}

@pragma('vm:external-name', 'NotifyCanAccessResource')
external void notifyCanAccessResource(bool success);

@pragma('vm:external-name', 'NotifySetAssetBundlePath')
external void notifySetAssetBundlePath();

@pragma('vm:entry-point')
Future<void> canAccessResourceFromAssetDir() async {
  notifySetAssetBundlePath();
  window.sendPlatformMessage(
    'flutter/assets',
    ByteData.sublistView(utf8.encode('kernel_blob.bin')),
    (ByteData? byteData) {
      notifyCanAccessResource(byteData != null);
    },
  );
}

@pragma('vm:external-name', 'NotifyNativeWhenEngineRun')
external void notifyNativeWhenEngineRun(bool success);

@pragma('vm:external-name', 'NotifyNativeWhenEngineSpawn')
external void notifyNativeWhenEngineSpawn(bool success);

@pragma('vm:entry-point')
void canReceiveArgumentsWhenEngineRun(List<String> args) {
  notifyNativeWhenEngineRun(args.length == 2 && args[0] == 'foo' && args[1] == 'bar');
}

@pragma('vm:entry-point')
void canReceiveArgumentsWhenEngineSpawn(List<String> args) {
  notifyNativeWhenEngineSpawn(args.length == 2 && args[0] == 'arg1' && args[1] == 'arg2');
}

@pragma('vm:entry-point')
void onBeginFrameWithNotifyNativeMain() {
  PlatformDispatcher.instance.onBeginFrame = (Duration beginTime) {
    nativeOnBeginFrame(beginTime.inMicroseconds);
  };
  notifyNative();
}

@pragma('vm:entry-point')
void frameCallback(Object? image, int durationMilliseconds, String decodeError) {
  if (image == null) {
    throw Exception('Expeccted image in frame callback to be non-null');
  }
}

@pragma('vm:external-name', 'NativeOnBeforeToImageSync')
external void onBeforeToImageSync();

@pragma('vm:entry-point')
Future<void> toImageSync() async {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  canvas.drawPaint(Paint()..color = const Color(0xFFAAAAAA));
  final Picture picture = recorder.endRecording();

  onBeforeToImageSync();
  final Image image = picture.toImageSync(20, 25);
  expect(image.width, 20);
  expect(image.height, 25);

  final ByteData dataBefore = (await image.toByteData())!;
  expect(dataBefore.lengthInBytes, 20 * 25 * 4);
  for (final int byte in dataBefore.buffer.asUint32List()) {
    expect(byte, 0xFFAAAAAA);
  }

  // Cause the rasterizer to get torn down.
  notifyNative();

  final ByteData dataAfter = (await image.toByteData())!;
  expect(dataAfter.lengthInBytes, 20 * 25 * 4);
  for (final int byte in dataAfter.buffer.asUint32List()) {
    expect(byte, 0xFFAAAAAA);
  }

  // Verify that the image can be drawn successfully.
  final PictureRecorder recorder2 = PictureRecorder();
  final Canvas canvas2 = Canvas(recorder2);
  canvas2.drawImage(image, Offset.zero, Paint());
  final Picture picture2 = recorder2.endRecording();

  picture.dispose();
  picture2.dispose();
  notifyNative();
}

@pragma('vm:entry-point')
Future<void> included() async {}

Future<void> excluded() async {}

class IsolateParam {
  const IsolateParam(this.sendPort, this.rawHandle);
  final SendPort sendPort;
  final int rawHandle;
}

@pragma('vm:entry-point')
Future<void> runCallback(IsolateParam param) async {
  try {
    final Future<dynamic> Function() func =
        PluginUtilities.getCallbackFromHandle(CallbackHandle.fromRawHandle(param.rawHandle))!
            as Future<dynamic> Function();
    await func.call();
    param.sendPort.send(true);
  } on NoSuchMethodError {
    param.sendPort.send(false);
  }
}

@pragma('vm:entry-point')
@pragma('vm:external-name', 'NotifyNativeBool')
external void notifyNativeBool(bool value);
@pragma('vm:external-name', 'NotifyDestroyed')
external void notifyDestroyed();

@pragma('vm:entry-point')
Future<void> testPluginUtilitiesCallbackHandle() async {
  ReceivePort port = ReceivePort();
  await Isolate.spawn(
    runCallback,
    IsolateParam(port.sendPort, PluginUtilities.getCallbackHandle(included)!.toRawHandle()),
    onError: port.sendPort,
  );
  final dynamic result1 = await port.first;
  if (result1 != true) {
    print('Expected $result1 to == true');
    notifyNativeBool(false);
    return;
  }
  port.close();
  if (const bool.fromEnvironment('dart.vm.product')) {
    port = ReceivePort();
    await Isolate.spawn(
      runCallback,
      IsolateParam(port.sendPort, PluginUtilities.getCallbackHandle(excluded)!.toRawHandle()),
      onError: port.sendPort,
    );
    final dynamic result2 = await port.first;
    if (result2 != false) {
      print('Expected $result2 to == false');
      notifyNativeBool(false);
      return;
    }
    port.close();
  }
  notifyNativeBool(true);
}

@pragma('vm:entry-point')
Future<void> testThatAssetLoadingHappensOnWorkerThread() async {
  try {
    await ImmutableBuffer.fromAsset('DoesNotExist');
  } catch (err) {
    /* Do nothing */
  }
  notifyNative();
}

@pragma('vm:external-name', 'NativeReportViewIdsCallback')
external void nativeReportViewIdsCallback(bool hasImplicitView, List<int> viewIds);

List<int> getCurrentViewIds() {
  final List<int> result =
      PlatformDispatcher.instance.views.map((FlutterView view) => view.viewId).toList()..sort();
  assert(result.toSet().length == result.length, 'Unexpected duplicate view ID found: $result');
  return result;
}

bool listEquals<T>(List<T> a, List<T> b) {
  if (a.length != b.length) {
    return false;
  }
  for (int i = 0; i < a.length; i += 1) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

// This entrypoint reports whether there's an implicit view and the list of view
// IDs using nativeReportViewIdsCallback on initialization and every time the
// list of view IDs changes.
@pragma('vm:entry-point')
void testReportViewIds() {
  List<int> viewIds = getCurrentViewIds();
  nativeReportViewIdsCallback(PlatformDispatcher.instance.implicitView != null, viewIds);
  PlatformDispatcher.instance.onMetricsChanged = () {
    final List<int> newViewIds = getCurrentViewIds();
    if (!listEquals(viewIds, newViewIds)) {
      viewIds = newViewIds;
      nativeReportViewIdsCallback(PlatformDispatcher.instance.implicitView != null, viewIds);
    }
  };
}

// Returns a list of [view_id 1, view_width 1, view_id 2, view_width 2, ...]
// for all views.
List<int> getCurrentViewWidths() {
  final List<int> result = <int>[];
  for (final FlutterView view in PlatformDispatcher.instance.views) {
    result.add(view.viewId);
    result.add(view.physicalSize.width.round());
  }
  return result;
}

@pragma('vm:external-name', 'NativeReportViewWidthsCallback')
external void nativeReportViewWidthsCallback(List<int> viewWidthPacket);

// This entrypoint reports the list of views and their widths using
// nativeReportViewWidthsCallback on initialization and every onMetricsChanged.
@pragma('vm:entry-point')
void testReportViewWidths() {
  nativeReportViewWidthsCallback(getCurrentViewWidths());
  PlatformDispatcher.instance.onMetricsChanged = () {
    nativeReportViewWidthsCallback(getCurrentViewWidths());
  };
}

void renderDummyToView(FlutterView view) {
  final SceneBuilder builder = SceneBuilder();
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  canvas.drawPaint(Paint()..color = const Color(0xFFABCDEF));
  final Picture picture = recorder.endRecording();
  builder.addPicture(Offset.zero, picture);

  final Scene scene = builder.build();
  view.render(scene);

  scene.dispose();
  picture.dispose();
}

@pragma('vm:entry-point')
void onDrawFrameRenderAllViews() {
  PlatformDispatcher.instance.onDrawFrame = () {
    PlatformDispatcher.instance.views.forEach(renderDummyToView);
  };
  notifyNative();
}

@pragma('vm:entry-point')
void renderViewsInFrameAndOutOfFrame() {
  renderDummyToView(PlatformDispatcher.instance.view(id: 1)!);
  PlatformDispatcher.instance.onDrawFrame = () {
    renderDummyToView(PlatformDispatcher.instance.view(id: 2)!);
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:external-name', 'CaptureRootLayer')
external void _captureRootLayer(SceneBuilder sceneBuilder);

@pragma('vm:entry-point')
void renderTwiceForOneView() {
  final SceneBuilder builder = SceneBuilder();
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  canvas.drawPaint(Paint()..color = const Color(0xFFABCDEF));
  final Picture picture = recorder.endRecording();
  builder.addPicture(Offset.zero, picture);

  PlatformDispatcher.instance.onBeginFrame = (_) {
    // Tell engine the correct layer tree.
    _captureRootLayer(builder);
  };

  PlatformDispatcher.instance.onDrawFrame = () {
    final Scene scene = builder.build();
    PlatformDispatcher.instance.implicitView!.render(scene);
    scene.dispose();
    picture.dispose();

    // Render a second time. This duplicate render should be ignored.
    renderDummyToView(PlatformDispatcher.instance.implicitView!);
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void renderSingleViewAndCallAfterOnDrawFrame() {
  PlatformDispatcher.instance.onDrawFrame = () {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = const Color(0xFFABCDEF));
    final Picture picture = recorder.endRecording();
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    PlatformDispatcher.instance.implicitView!.render(scene);
    // Notify the engine after the render before the disposal.
    // The view should have been submitted for rasterization at this moment.
    notifyNative();

    scene.dispose();
    picture.dispose();
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void renderWarmUpImplicitView() {
  bool beginFrameCalled = false;

  PlatformDispatcher.instance.scheduleWarmUpFrame(
    beginFrame: () {
      expect(beginFrameCalled, false);
      beginFrameCalled = true;
    },
    drawFrame: () {
      expect(beginFrameCalled, true);
      renderDummyToView(PlatformDispatcher.instance.implicitView!);
    },
  );
}

@pragma('vm:entry-point')
void renderWarmUpView1and2() {
  bool beginFrameCalled = false;

  PlatformDispatcher.instance.scheduleWarmUpFrame(
    beginFrame: () {
      expect(beginFrameCalled, false);
      beginFrameCalled = true;
    },
    drawFrame: () {
      expect(beginFrameCalled, true);

      for (final int viewId in <int>[1, 2]) {
        renderDummyToView(PlatformDispatcher.instance.view(id: viewId)!);
      }
    },
  );
}

@pragma('vm:entry-point')
void testSemanticsActions() {
  PlatformDispatcher.instance.onSemanticsActionEvent = (SemanticsActionEvent action) async {
    await null;
    Future<void>.value().then((_) {
      notifyNative();
    });
  };
}

@pragma('vm:entry-point')
void testPointerActions() {
  PlatformDispatcher.instance.onPointerDataPacket = (PointerDataPacket pointer) async {
    await null;
    Future<void>.value().then((_) {
      notifyNative();
    });
  };
}
