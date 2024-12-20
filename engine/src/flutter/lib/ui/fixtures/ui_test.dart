// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui';
import 'dart:isolate';
import 'dart:ffi' hide Size;

void main() {}

/// Mutiple tests use this to signal to the C++ side that they are ready for
/// validation.
@pragma('vm:external-name', 'Finish')
external void _finish();

@pragma('vm:entry-point')
void customOnErrorTrue() {
  PlatformDispatcher.instance.onError = (Object error, StackTrace? stack) {
    _finish();
    return true;
  };
  throw Exception('true');
}

@pragma('vm:entry-point')
void customOnErrorFalse() {
  PlatformDispatcher.instance.onError = (Object error, StackTrace? stack) {
    _finish();
    return false;
  };
  throw Exception('false');
}

@pragma('vm:entry-point')
void customOnErrorThrow() {
  PlatformDispatcher.instance.onError = (Object error, StackTrace? stack) {
    _finish();
    throw Exception('throw2');
  };
  throw Exception('throw1');
}

@pragma('vm:entry-point')
void setLatencyPerformanceMode() {
  PlatformDispatcher.instance.requestDartPerformanceMode(DartPerformanceMode.latency);
  _finish();
}

@pragma('vm:entry-point')
void validateSceneBuilderAndScene() {
  final SceneBuilder builder = SceneBuilder();
  builder.pushOffset(10, 10);
  _validateBuilderHasLayers(builder);
  final Scene scene = builder.build();
  _validateBuilderHasNoLayers();
  _captureScene(scene);
  scene.dispose();
  _validateSceneHasNoLayers();
}

@pragma('vm:external-name', 'ValidateBuilderHasLayers')
external _validateBuilderHasLayers(SceneBuilder builder);
@pragma('vm:external-name', 'ValidateBuilderHasNoLayers')
external _validateBuilderHasNoLayers();
@pragma('vm:external-name', 'CaptureScene')
external _captureScene(Scene scene);
@pragma('vm:external-name', 'ValidateSceneHasNoLayers')
external _validateSceneHasNoLayers();

@pragma('vm:entry-point')
void validateEngineLayerDispose() {
  final SceneBuilder builder = SceneBuilder();
  final EngineLayer layer = builder.pushOffset(10, 10);
  _captureRootLayer(builder);
  final Scene scene = builder.build();
  scene.dispose();
  _validateLayerTreeCounts();
  layer.dispose();
  _validateEngineLayerDispose();
}

@pragma('vm:external-name', 'CaptureRootLayer')
external _captureRootLayer(SceneBuilder sceneBuilder);
@pragma('vm:external-name', 'ValidateLayerTreeCounts')
external _validateLayerTreeCounts();
@pragma('vm:external-name', 'ValidateEngineLayerDispose')
external _validateEngineLayerDispose();

@pragma('vm:entry-point')
Future<void> createSingleFrameCodec() async {
  final ImmutableBuffer buffer = await ImmutableBuffer.fromUint8List(
    Uint8List.fromList(List<int>.filled(4, 100)),
  );
  final ImageDescriptor descriptor = ImageDescriptor.raw(
    buffer,
    width: 1,
    height: 1,
    pixelFormat: PixelFormat.rgba8888,
  );
  final Codec codec = await descriptor.instantiateCodec();
  _validateCodec(codec);
  final FrameInfo info = await codec.getNextFrame();
  info.image.dispose();
  _validateCodec(codec);
  codec.dispose();
  descriptor.dispose();
  buffer.dispose();
  assert(buffer.debugDisposed);
  _finish();
}

@pragma('vm:external-name', 'ValidateCodec')
external void _validateCodec(Codec codec);

@pragma('vm:entry-point')
void createVertices() {
  const int uint16max = 65535;

  final Int32List colors = Int32List(uint16max);
  final Float32List coords = Float32List(uint16max * 2);
  final Uint16List indices = Uint16List(uint16max);
  final Float32List positions = Float32List(uint16max * 2);
  colors[0] = const Color(0xFFFF0000).value;
  colors[1] = const Color(0xFF00FF00).value;
  colors[2] = const Color(0xFF0000FF).value;
  colors[3] = const Color(0xFF00FFFF).value;
  indices[1] = indices[3] = 1;
  indices[2] = indices[5] = 3;
  indices[4] = 2;
  positions[2] = positions[4] = positions[5] = positions[7] = 250.0;

  final Vertices vertices = Vertices.raw(
    VertexMode.triangles,
    positions,
    textureCoordinates: coords,
    colors: colors,
    indices: indices,
  );
  _validateVertices(vertices);
}

@pragma('vm:external-name', 'ValidateVertices')
external void _validateVertices(Vertices vertices);

@pragma('vm:entry-point')
void sendSemanticsUpdate() {
  final SemanticsUpdateBuilder builder = SemanticsUpdateBuilder();
  final String identifier = "identifier";
  final String label = "label";
  final List<StringAttribute> labelAttributes = <StringAttribute>[
    SpellOutStringAttribute(range: TextRange(start: 1, end: 2)),
  ];

  final String value = "value";
  final List<StringAttribute> valueAttributes = <StringAttribute>[
    SpellOutStringAttribute(range: TextRange(start: 2, end: 3)),
  ];

  final String increasedValue = "increasedValue";
  final List<StringAttribute> increasedValueAttributes = <StringAttribute>[
    SpellOutStringAttribute(range: TextRange(start: 4, end: 5)),
  ];

  final String decreasedValue = "decreasedValue";
  final List<StringAttribute> decreasedValueAttributes = <StringAttribute>[
    SpellOutStringAttribute(range: TextRange(start: 5, end: 6)),
  ];

  final String hint = "hint";
  final List<StringAttribute> hintAttributes = <StringAttribute>[
    LocaleStringAttribute(locale: Locale('en', 'MX'), range: TextRange(start: 0, end: 1)),
  ];

  String tooltip = "tooltip";

  final Float64List transform = Float64List(16);
  final Int32List childrenInTraversalOrder = Int32List(0);
  final Int32List childrenInHitTestOrder = Int32List(0);
  final Int32List additionalActions = Int32List(0);
  transform[0] = 1;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;

  transform[4] = 0;
  transform[5] = 1;
  transform[6] = 0;
  transform[7] = 0;

  transform[8] = 0;
  transform[9] = 0;
  transform[10] = 1;
  transform[11] = 0;

  transform[12] = 0;
  transform[13] = 0;
  transform[14] = 0;
  transform[15] = 0;
  builder.updateNode(
    id: 0,
    flags: 0,
    actions: 0,
    maxValueLength: 0,
    currentValueLength: 0,
    textSelectionBase: -1,
    textSelectionExtent: -1,
    platformViewId: -1,
    scrollChildren: 0,
    scrollIndex: 0,
    scrollPosition: 0,
    scrollExtentMax: 0,
    scrollExtentMin: 0,
    rect: Rect.fromLTRB(0, 0, 10, 10),
    elevation: 0,
    thickness: 0,
    identifier: identifier,
    label: label,
    labelAttributes: labelAttributes,
    value: value,
    valueAttributes: valueAttributes,
    increasedValue: increasedValue,
    increasedValueAttributes: increasedValueAttributes,
    decreasedValue: decreasedValue,
    decreasedValueAttributes: decreasedValueAttributes,
    hint: hint,
    hintAttributes: hintAttributes,
    tooltip: tooltip,
    textDirection: TextDirection.ltr,
    transform: transform,
    childrenInTraversalOrder: childrenInTraversalOrder,
    childrenInHitTestOrder: childrenInHitTestOrder,
    additionalActions: additionalActions,
    headingLevel: 0,
    linkUrl: '',
  );
  _semanticsUpdate(builder.build());
}

@pragma('vm:external-name', 'SemanticsUpdate')
external void _semanticsUpdate(SemanticsUpdate update);

@pragma('vm:entry-point')
void createPath() {
  final Path path = Path()..lineTo(10, 10);
  _validatePath(path);
  // Arbitrarily hold a reference to the path to make sure it does not get
  // garbage collected.
  Future<void>.delayed(const Duration(days: 100)).then((_) {
    path.lineTo(100, 100);
  });
}

@pragma('vm:external-name', 'ValidatePath')
external void _validatePath(Path path);

@pragma('vm:entry-point')
void frameCallback(Object? image, int durationMilliseconds, String decodeError) {
  validateFrameCallback(image, durationMilliseconds, decodeError);
}

@pragma('vm:external-name', 'ValidateFrameCallback')
external void validateFrameCallback(Object? image, int durationMilliseconds, String decodeError);

@pragma('vm:entry-point')
void platformMessagePortResponseTest() async {
  ReceivePort receivePort = ReceivePort();
  _callPlatformMessageResponseDartPort(receivePort.sendPort.nativePort);
  List<dynamic> resultList = await receivePort.first;
  int identifier = resultList[0] as int;
  Uint8List? bytes = resultList[1] as Uint8List?;
  ByteData result = ByteData.sublistView(bytes!);
  if (result.lengthInBytes == 100) {
    _finishCallResponse(true);
  } else {
    _finishCallResponse(false);
  }
}

@pragma('vm:entry-point')
void platformMessageResponseTest() {
  _callPlatformMessageResponseDart((ByteData? result) {
    if (result is ByteData && result.lengthInBytes == 100) {
      int value = result.getInt8(0);
      bool didThrowOnModify = false;
      try {
        result.setInt8(0, value);
      } catch (e) {
        didThrowOnModify = true;
      }
      // This should be a read only buffer.
      _finishCallResponse(didThrowOnModify);
    } else {
      _finishCallResponse(false);
    }
  });
}

@pragma('vm:external-name', 'CallPlatformMessageResponseDartPort')
external void _callPlatformMessageResponseDartPort(int port);
@pragma('vm:external-name', 'CallPlatformMessageResponseDart')
external void _callPlatformMessageResponseDart(void Function(ByteData? result) callback);
@pragma('vm:external-name', 'FinishCallResponse')
external void _finishCallResponse(bool didPass);

@pragma('vm:entry-point')
void messageCallback(dynamic data) {}

@pragma('vm:entry-point')
@pragma('vm:external-name', 'ValidateConfiguration')
external void validateConfiguration();

// Draw a circle on a Canvas that has a PictureRecorder. Take the image from
// the PictureRecorder, and encode it as png. Check that the png data is
// backed by an external Uint8List.
@pragma('vm:entry-point')
Future<void> encodeImageProducesExternalUint8List() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  final Image image = await picture.toImage(100, 100);
  _encodeImage(image, ImageByteFormat.png.index, (Uint8List result, String? error) {
    // The buffer should be non-null and writable.
    result[0] = 0;
    // The buffer should be external typed data.
    _validateExternal(result);
  });
}

@pragma('vm:external-name', 'EncodeImage')
external void _encodeImage(Image i, int format, void Function(Uint8List result, String? error));
@pragma('vm:external-name', 'ValidateExternal')
external void _validateExternal(Uint8List result);
@pragma('vm:external-name', 'ValidateError')
external void _validateError(String? error);
@pragma('vm:external-name', 'TurnOffGPU')
external void _turnOffGPU(bool value);
@pragma('vm:external-name', 'FlushGpuAwaitingTasks')
external void _flushGpuAwaitingTasks();
@pragma('vm:external-name', 'ValidateNotNull')
external void _validateNotNull(Object? object);

@pragma('vm:entry-point')
Future<void> toByteDataWithoutGPU() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  final Image image = await picture.toImage(100, 100);
  _turnOffGPU(true);
  Timer flusher = Timer.periodic(Duration(milliseconds: 1), (timer) {
    _flushGpuAwaitingTasks();
  });
  try {
    ByteData? byteData = await image.toByteData();
    _validateError(null);
  } catch (error) {
    _validateError(error.toString());
  } finally {
    flusher.cancel();
  }
}

@pragma('vm:entry-point')
Future<void> toByteDataRetries() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  final Image image = await picture.toImage(100, 100);
  _turnOffGPU(true);
  Future<void>.delayed(Duration(milliseconds: 10), () {
    _turnOffGPU(false);
  });
  try {
    ByteData? byteData = await image.toByteData();
    _validateNotNull(byteData);
  } catch (error) {
    _validateNotNull(null);
  }
}

@pragma('vm:entry-point')
Future<void> toByteDataRetryOverflows() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  List<Image> images = [];
  // This number must be bigger than impeller::Context::kMaxTasksAwaitingGPU.
  int numJobs = 100;
  for (int i = 0; i < numJobs; ++i) {
    images.add(await picture.toImage(100, 100));
  }
  List<Future<ByteData?>> dataFutures = [];
  _turnOffGPU(true);
  for (Image image in images) {
    dataFutures.add(image.toByteData());
  }
  Future<void>.delayed(Duration(milliseconds: 10), () {
    _turnOffGPU(false);
  });

  ByteData? result;
  for (Future<ByteData?> future in dataFutures) {
    try {
      ByteData? byteData = await future;
      if (byteData != null) {
        result = byteData;
      }
    } catch (_) {
      // Ignore errors from unavailable gpu.
    }
  }
  _validateNotNull(result);
}

@pragma('vm:entry-point')
Future<void> toImageRetries() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  _turnOffGPU(true);
  Future<void>.delayed(Duration(milliseconds: 10), () {
    _turnOffGPU(false);
  });
  try {
    final Image image = await picture.toImage(100, 100);
    _validateNotNull(image);
  } catch (error) {
    _validateNotNull(null);
  }
}

@pragma('vm:entry-point')
Future<void> toImageRetryOverflows() async {
  final PictureRecorder pictureRecorder = PictureRecorder();
  final Canvas canvas = Canvas(pictureRecorder);
  final Paint paint =
      Paint()
        ..color = Color.fromRGBO(255, 255, 255, 1.0)
        ..style = PaintingStyle.fill;
  final Offset c = Offset(50.0, 50.0);
  canvas.drawCircle(c, 25.0, paint);
  final Picture picture = pictureRecorder.endRecording();
  _turnOffGPU(true);
  List<Future<Image>> imageFutures = [];
  // This number must be bigger than impeller::Context::kMaxTasksAwaitingGPU.
  int numJobs = 100;
  for (int i = 0; i < numJobs; i++) {
    imageFutures.add(picture.toImage(100, 100));
  }
  Future<void>.delayed(Duration(milliseconds: 10), () {
    _turnOffGPU(false);
  });
  late Image result;
  bool didSeeImage = false;
  for (Future<Image> future in imageFutures) {
    try {
      Image image = await future;
      result = image;
      didSeeImage = true;
    } catch (_) {
      // Ignore gpu not available errors.
    }
  }
  _validateNotNull(didSeeImage ? result : null);
}

@pragma('vm:entry-point')
Future<void> pumpImage() async {
  const int width = 60;
  const int height = 60;
  final Completer<Image> completer = Completer<Image>();
  decodeImageFromPixels(
    Uint8List.fromList(List<int>.filled(width * height * 4, 0xFF)),
    width,
    height,
    PixelFormat.rgba8888,
    (Image image) => completer.complete(image),
  );
  final Image image = await completer.future;
  late Picture picture;
  late OffsetEngineLayer layer;

  void renderBlank(Duration duration) {
    image.dispose();
    picture.dispose();
    layer.dispose();

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint());
    picture = recorder.endRecording();
    final SceneBuilder builder = SceneBuilder();
    layer = builder.pushOffset(0, 0);
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    window.render(scene);
    scene.dispose();

    _finish();
  }

  void renderImage(Duration duration) {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawImage(image, Offset.zero, Paint());
    picture = recorder.endRecording();

    final SceneBuilder builder = SceneBuilder();
    layer = builder.pushOffset(0, 0);
    builder.addPicture(Offset.zero, picture);

    _captureImageAndPicture(image, picture);

    final Scene scene = builder.build();
    window.render(scene);
    scene.dispose();

    window.onBeginFrame = renderBlank;
    window.scheduleFrame();
  }

  window.onBeginFrame = renderImage;
  window.scheduleFrame();
}

@pragma('vm:external-name', 'CaptureImageAndPicture')
external void _captureImageAndPicture(Image image, Picture picture);

@pragma('vm:entry-point')
void convertPaintToDlPaint() {
  Paint paint = Paint();
  paint.blendMode = BlendMode.modulate;
  paint.color = Color.fromARGB(0x11, 0x22, 0x33, 0x44);
  paint.colorFilter = ColorFilter.mode(Color.fromARGB(0x55, 0x66, 0x77, 0x88), BlendMode.xor);
  paint.maskFilter = MaskFilter.blur(BlurStyle.inner, .75);
  paint.style = PaintingStyle.stroke;
  _convertPaintToDlPaint(paint);
}

@pragma('vm:external-name', 'ConvertPaintToDlPaint')
external void _convertPaintToDlPaint(Paint paint);

/// Hooks for platform_configuration_unittests.cc
@pragma('vm:entry-point')
void _beginFrameHijack(int microseconds, int frameNumber) {
  nativeBeginFrame(microseconds, frameNumber);
}

@pragma('vm:entry-point')
@pragma('vm:external-name', 'BeginFrame')
external nativeBeginFrame(int microseconds, int frameNumber);

@pragma('vm:entry-point')
void hooksTests() async {
  Future<void> test(String name, FutureOr<void> Function() testFunction) async {
    try {
      await testFunction();
    } catch (e) {
      print('Test "$name" failed!');
      rethrow;
    }
  }

  void expectEquals(Object? value, Object? expected) {
    if (value != expected) {
      throw 'Expected $value to be $expected.';
    }
  }

  void expectIdentical(Object a, Object b) {
    if (!identical(a, b)) {
      throw 'Expected $a to be identical to $b.';
    }
  }

  void expectNotEquals(Object? value, Object? expected) {
    if (value == expected) {
      throw 'Expected $value to not be $expected.';
    }
  }

  await test('onMetricsChanged preserves callback zone', () {
    late Zone originalZone;
    late Zone callbackZone;
    late double devicePixelRatio;

    runZoned(() {
      originalZone = Zone.current;
      window.onMetricsChanged = () {
        callbackZone = Zone.current;
        devicePixelRatio = window.devicePixelRatio;
      };
    });

    window.onMetricsChanged!();
    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      0.1234, // device pixel ratio
      0.0, // width
      0.0, // height
      0.0, // padding top
      0.0, // padding right
      0.0, // padding bottom
      0.0, // padding left
      0.0, // inset top
      0.0, // inset right
      0.0, // inset bottom
      0.0, // inset left
      0.0, // system gesture inset top
      0.0, // system gesture inset right
      0.0, // system gesture inset bottom
      0.0, // system gesture inset left
      22.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectIdentical(originalZone, callbackZone);
    if (devicePixelRatio != 0.1234) {
      throw 'Expected devicePixelRatio to be 0.1234 but got $devicePixelRatio.';
    }
  });

  await test('onError preserves the callback zone', () {
    late Zone originalZone;
    late Zone callbackZone;
    final Object error = Exception('foo');
    StackTrace? stackTrace;

    runZoned(() {
      originalZone = Zone.current;
      PlatformDispatcher.instance.onError = (Object exception, StackTrace? stackTrace) {
        callbackZone = Zone.current;
        expectIdentical(exception, error);
        expectNotEquals(stackTrace, null);
        return true;
      };
    });

    _callHook('_onError', 2, error, StackTrace.current);
    PlatformDispatcher.instance.onError = null;
    expectIdentical(originalZone, callbackZone);
  });

  await test('updateUserSettings can handle an empty object', () {
    _callHook('_updateUserSettingsData', 1, '{}');
  });

  await test(
    'PlatformDispatcher.locale returns unknown locale when locales is set to empty list',
    () {
      late Locale locale;
      int callCount = 0;
      runZoned(() {
        window.onLocaleChanged = () {
          locale = PlatformDispatcher.instance.locale;
          callCount += 1;
        };
      });

      const Locale fakeLocale = Locale.fromSubtags(
        languageCode: '1',
        countryCode: '2',
        scriptCode: '3',
      );
      _callHook('_updateLocales', 1, <String>[
        fakeLocale.languageCode,
        fakeLocale.countryCode!,
        fakeLocale.scriptCode!,
        '',
      ]);
      if (callCount != 1) {
        throw 'Expected 1 call, have $callCount';
      }
      if (locale != fakeLocale) {
        throw 'Expected $locale to match $fakeLocale';
      }
      _callHook('_updateLocales', 1, <String>[]);
      if (callCount != 2) {
        throw 'Expected 2 calls, have $callCount';
      }

      if (locale != const Locale.fromSubtags()) {
        throw '$locale did not equal ${Locale.fromSubtags()}';
      }
      if (locale.languageCode != 'und') {
        throw '${locale.languageCode} did not equal "und"';
      }
    },
  );

  await test('deprecated region equals', () {
    // These are equal because ZR is deprecated and was mapped to CD.
    const Locale x = Locale('en', 'ZR');
    const Locale y = Locale('en', 'CD');
    expectEquals(x, y);
    expectEquals(x.countryCode, y.countryCode);
  });

  await test('PlatformDispatcher.view getter returns view with provided ID', () {
    const int viewId = 0;
    expectEquals(PlatformDispatcher.instance.view(id: viewId)?.viewId, viewId);
  });

  await test('View padding/insets/viewPadding/systemGestureInsets', () {
    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      1.0, // devicePixelRatio
      800.0, // width
      600.0, // height
      50.0, // paddingTop
      0.0, // paddingRight
      40.0, // paddingBottom
      0.0, // paddingLeft
      0.0, // insetTop
      0.0, // insetRight
      0.0, // insetBottom
      0.0, // insetLeft
      0.0, // systemGestureInsetTop
      0.0, // systemGestureInsetRight
      0.0, // systemGestureInsetBottom
      0.0, // systemGestureInsetLeft
      22.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectEquals(window.viewInsets.bottom, 0.0);
    expectEquals(window.viewPadding.bottom, 40.0);
    expectEquals(window.padding.bottom, 40.0);
    expectEquals(window.systemGestureInsets.bottom, 0.0);

    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      1.0, // devicePixelRatio
      800.0, // width
      600.0, // height
      50.0, // paddingTop
      0.0, // paddingRight
      40.0, // paddingBottom
      0.0, // paddingLeft
      0.0, // insetTop
      0.0, // insetRight
      400.0, // insetBottom
      0.0, // insetLeft
      0.0, // systemGestureInsetTop
      0.0, // systemGestureInsetRight
      44.0, // systemGestureInsetBottom
      0.0, // systemGestureInsetLeft
      22.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectEquals(window.viewInsets.bottom, 400.0);
    expectEquals(window.viewPadding.bottom, 40.0);
    expectEquals(window.padding.bottom, 0.0);
    expectEquals(window.systemGestureInsets.bottom, 44.0);
  });

  await test('Window physical touch slop', () {
    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      1.0, // devicePixelRatio
      800.0, // width
      600.0, // height
      50.0, // paddingTop
      0.0, // paddingRight
      40.0, // paddingBottom
      0.0, // paddingLeft
      0.0, // insetTop
      0.0, // insetRight
      0.0, // insetBottom
      0.0, // insetLeft
      0.0, // systemGestureInsetTop
      0.0, // systemGestureInsetRight
      0.0, // systemGestureInsetBottom
      0.0, // systemGestureInsetLeft
      11.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectEquals(window.gestureSettings, GestureSettings(physicalTouchSlop: 11.0));

    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      1.0, // devicePixelRatio
      800.0, // width
      600.0, // height
      50.0, // paddingTop
      0.0, // paddingRight
      40.0, // paddingBottom
      0.0, // paddingLeft
      0.0, // insetTop
      0.0, // insetRight
      400.0, // insetBottom
      0.0, // insetLeft
      0.0, // systemGestureInsetTop
      0.0, // systemGestureInsetRight
      44.0, // systemGestureInsetBottom
      0.0, // systemGestureInsetLeft
      -1.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectEquals(window.gestureSettings, GestureSettings(physicalTouchSlop: null));

    _callHook(
      '_updateWindowMetrics',
      21,
      0, // window Id
      1.0, // devicePixelRatio
      800.0, // width
      600.0, // height
      50.0, // paddingTop
      0.0, // paddingRight
      40.0, // paddingBottom
      0.0, // paddingLeft
      0.0, // insetTop
      0.0, // insetRight
      400.0, // insetBottom
      0.0, // insetLeft
      0.0, // systemGestureInsetTop
      0.0, // systemGestureInsetRight
      44.0, // systemGestureInsetBottom
      0.0, // systemGestureInsetLeft
      22.0, // physicalTouchSlop
      <double>[], // display features bounds
      <int>[], // display features types
      <int>[], // display features states
      0, // Display ID
    );

    expectEquals(window.gestureSettings, GestureSettings(physicalTouchSlop: 22.0));
  });

  await test('onLocaleChanged preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    Locale? locale;

    runZoned(() {
      innerZone = Zone.current;
      window.onLocaleChanged = () {
        runZone = Zone.current;
        locale = window.locale;
      };
    });

    _callHook('_updateLocales', 1, <String>['en', 'US', '', '']);
    expectIdentical(runZone, innerZone);
    expectEquals(locale, const Locale('en', 'US'));
  });

  await test('onBeginFrame preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late Duration start;

    runZoned(() {
      innerZone = Zone.current;
      window.onBeginFrame = (Duration value) {
        runZone = Zone.current;
        start = value;
      };
    });

    _callHook('_beginFrame', 2, 1234, 1);
    expectIdentical(runZone, innerZone);
    expectEquals(start, const Duration(microseconds: 1234));
  });

  await test('onDrawFrame preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;

    runZoned(() {
      innerZone = Zone.current;
      window.onDrawFrame = () {
        runZone = Zone.current;
      };
    });

    _callHook('_drawFrame');
    expectIdentical(runZone, innerZone);
  });

  await test('onReportTimings preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;

    runZoned(() {
      innerZone = Zone.current;
      window.onReportTimings = (List<FrameTiming> timings) {
        runZone = Zone.current;
      };
    });

    _callHook('_reportTimings', 1, <int>[]);
    expectIdentical(runZone, innerZone);
  });

  await test('onPointerDataPacket preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late PointerDataPacket data;

    runZoned(() {
      innerZone = Zone.current;
      window.onPointerDataPacket = (PointerDataPacket value) {
        runZone = Zone.current;
        data = value;
      };
    });

    final ByteData testData = ByteData.view(Uint8List(0).buffer);
    _callHook('_dispatchPointerDataPacket', 1, testData);
    expectIdentical(runZone, innerZone);
    expectEquals(data.data.length, 0);
  });

  await test('onSemanticsEnabledChanged preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late bool enabled;

    runZoned(() {
      innerZone = Zone.current;
      window.onSemanticsEnabledChanged = () {
        runZone = Zone.current;
        enabled = window.semanticsEnabled;
      };
    });

    final bool newValue = !window.semanticsEnabled; // needed?
    _callHook('_updateSemanticsEnabled', 1, newValue);
    expectIdentical(runZone, innerZone);
    expectEquals(enabled, newValue);
  });

  await test('onSemanticsActionEvent preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late SemanticsActionEvent action;

    runZoned(() {
      innerZone = Zone.current;
      PlatformDispatcher.instance.onSemanticsActionEvent = (SemanticsActionEvent actionEvent) {
        runZone = Zone.current;
        action = actionEvent;
      };
    });

    _callHook('_dispatchSemanticsAction', 3, 1234, 4, null);
    expectIdentical(runZone, innerZone);
    expectEquals(action.nodeId, 1234);
    expectEquals(action.type.index, 4);
  });

  await test('onPlatformMessage preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late String name;

    runZoned(() {
      innerZone = Zone.current;
      window.onPlatformMessage = (String value, _, __) {
        runZone = Zone.current;
        name = value;
      };
    });

    _callHook('_dispatchPlatformMessage', 3, 'testName', null, 123456789);
    expectIdentical(runZone, innerZone);
    expectEquals(name, 'testName');
  });

  await test('onTextScaleFactorChanged preserves callback zone', () {
    late Zone innerZone;
    late Zone runZoneTextScaleFactor;
    late Zone runZonePlatformBrightness;
    late double? textScaleFactor;
    late Brightness? platformBrightness;

    runZoned(() {
      innerZone = Zone.current;
      window.onTextScaleFactorChanged = () {
        runZoneTextScaleFactor = Zone.current;
        textScaleFactor = window.textScaleFactor;
      };
      window.onPlatformBrightnessChanged = () {
        runZonePlatformBrightness = Zone.current;
        platformBrightness = window.platformBrightness;
      };
    });

    window.onTextScaleFactorChanged!();

    _callHook(
      '_updateUserSettingsData',
      1,
      '{"textScaleFactor": 0.5, "platformBrightness": "light", "alwaysUse24HourFormat": true}',
    );
    expectIdentical(runZoneTextScaleFactor, innerZone);
    expectEquals(textScaleFactor, 0.5);

    textScaleFactor = null;
    platformBrightness = null;

    window.onPlatformBrightnessChanged!();
    _callHook(
      '_updateUserSettingsData',
      1,
      '{"textScaleFactor": 0.5, "platformBrightness": "dark", "alwaysUse24HourFormat": true}',
    );
    expectIdentical(runZonePlatformBrightness, innerZone);
    expectEquals(platformBrightness, Brightness.dark);
  });

  await test('onFrameDataChanged preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late int frameNumber;

    runZoned(() {
      innerZone = Zone.current;
      window.onFrameDataChanged = () {
        runZone = Zone.current;
        frameNumber = window.frameData.frameNumber;
      };
    });

    _callHook('_beginFrame', 2, 0, 2);
    expectNotEquals(runZone, null);
    expectIdentical(runZone, innerZone);
    expectEquals(frameNumber, 2);
  });

  await test('_updateDisplays preserves callback zone', () {
    late Zone innerZone;
    late Zone runZone;
    late Display display;

    runZoned(() {
      innerZone = Zone.current;
      window.onMetricsChanged = () {
        runZone = Zone.current;
        display = PlatformDispatcher.instance.displays.first;
      };
    });

    _callHook('_updateDisplays', 5, <int>[0], <double>[800], <double>[600], <double>[1.5], <double>[
      65,
    ]);
    expectNotEquals(runZone, null);
    expectIdentical(runZone, innerZone);
    expectEquals(display.id, 0);
    expectEquals(display.size, const Size(800, 600));
    expectEquals(display.devicePixelRatio, 1.5);
    expectEquals(display.refreshRate, 65);
  });

  await test('_futureize handles callbacker sync error', () async {
    String? callbacker(void Function(Object? arg) cb) {
      return 'failure';
    }

    Object? error;
    try {
      await _futurize(callbacker);
    } catch (err) {
      error = err;
    }
    expectNotEquals(error, null);
  });

  await test('_futureize does not leak sync uncaught exceptions into the zone', () async {
    String? callbacker(void Function(Object? arg) cb) {
      cb(null); // indicates failure
    }

    Object? error;
    try {
      await _futurize(callbacker);
    } catch (err) {
      error = err;
    }
    expectNotEquals(error, null);
  });

  await test('_futureize does not leak async uncaught exceptions into the zone', () async {
    String? callbacker(void Function(Object? arg) cb) {
      Timer.run(() {
        cb(null); // indicates failure
      });
    }

    Object? error;
    try {
      await _futurize(callbacker);
    } catch (err) {
      error = err;
    }
    expectNotEquals(error, null);
  });

  await test('_futureize successfully returns a value sync', () async {
    String? callbacker(void Function(Object? arg) cb) {
      cb(true);
    }

    final Object? result = await _futurize(callbacker);

    expectEquals(result, true);
  });

  await test('_futureize successfully returns a value async', () async {
    String? callbacker(void Function(Object? arg) cb) {
      Timer.run(() {
        cb(true);
      });
    }

    final Object? result = await _futurize(callbacker);

    expectEquals(result, true);
  });

  await test('root isolate token', () async {
    if (RootIsolateToken.instance == null) {
      throw Exception('We should have a token on a root isolate.');
    }
    ReceivePort receivePort = ReceivePort();
    Isolate.spawn(_backgroundRootIsolateTestMain, receivePort.sendPort);
    bool didPass = await receivePort.first as bool;
    if (!didPass) {
      throw Exception('Background isolate found a root isolate id.');
    }
  });

  await test('send port message without registering', () async {
    ReceivePort receivePort = ReceivePort();
    Isolate.spawn(_backgroundIsolateSendWithoutRegistering, receivePort.sendPort);
    bool didError = await receivePort.first as bool;
    if (!didError) {
      throw Exception(
        'Expected an error when not registering a root isolate and sending port messages.',
      );
    }
  });

  _finish();
}

/// Sends `true` on [port] if the isolate executing the function is not a root
/// isolate.
void _backgroundRootIsolateTestMain(SendPort port) {
  port.send(RootIsolateToken.instance == null);
}

/// Sends `true` on [port] if [PlatformDispatcher.sendPortPlatformMessage]
/// throws an exception without calling
/// [PlatformDispatcher.registerBackgroundIsolate].
void _backgroundIsolateSendWithoutRegistering(SendPort port) {
  bool didError = false;
  ReceivePort messagePort = ReceivePort();
  try {
    PlatformDispatcher.instance.sendPortPlatformMessage('foo', null, 1, messagePort.sendPort);
  } catch (_) {
    didError = true;
  }
  port.send(didError);
}

typedef _Callback<T> = void Function(T result);
typedef _Callbacker<T> = String? Function(_Callback<T?> callback);

// This is an exact copy of the function defined in painting.dart. If you change either
// then you must change both.
Future<T> _futurize<T>(_Callbacker<T> callbacker) {
  final Completer<T> completer = Completer<T>.sync();
  // If the callback synchronously throws an error, then synchronously
  // rethrow that error instead of adding it to the completer. This
  // prevents the Zone from receiving an uncaught exception.
  bool sync = true;
  final String? error = callbacker((T? t) {
    if (t == null) {
      if (sync) {
        throw Exception('operation failed');
      } else {
        completer.completeError(Exception('operation failed'));
      }
    } else {
      completer.complete(t);
    }
  });
  sync = false;
  if (error != null) throw Exception(error);
  return completer.future;
}

@pragma('vm:external-name', 'CallHook')
external void _callHook(
  String name, [
  int argCount = 0,
  Object? arg0,
  Object? arg1,
  Object? arg2,
  Object? arg3,
  Object? arg4,
  Object? arg5,
  Object? arg6,
  Object? arg8,
  Object? arg9,
  Object? arg10,
  Object? arg11,
  Object? arg12,
  Object? arg13,
  Object? arg14,
  Object? arg15,
  Object? arg16,
  Object? arg17,
  Object? arg18,
  Object? arg19,
  Object? arg20,
  Object? arg21,
]);
