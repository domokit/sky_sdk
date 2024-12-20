// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../image_data.dart';
import 'semantics_tester.dart';

void main() {
  late int originalCacheSize;
  late ui.Image image10x10;

  setUp(() async {
    originalCacheSize = imageCache.maximumSize;
    imageCache.clear();
    imageCache.clearLiveImages();
    image10x10 = await createTestImage(width: 10, height: 10);
  });

  tearDown(() {
    imageCache.maximumSize = originalCacheSize;
  });

  testWidgets('Verify Image does not use disposed handles', (WidgetTester tester) async {
    final ui.Image image100x100 =
        (await tester.runAsync(() async => createTestImage(width: 100, height: 100)))!;

    final _TestImageProvider imageProvider1 = _TestImageProvider();
    final _TestImageProvider imageProvider2 = _TestImageProvider();

    final ValueNotifier<_TestImageProvider> imageListenable = ValueNotifier<_TestImageProvider>(
      imageProvider1,
    );
    addTearDown(imageListenable.dispose);
    final ValueNotifier<int> innerListenable = ValueNotifier<int>(0);
    addTearDown(innerListenable.dispose);

    bool imageLoaded = false;

    await tester.pumpWidget(
      ValueListenableBuilder<_TestImageProvider>(
        valueListenable: imageListenable,
        builder:
            (BuildContext context, _TestImageProvider image, Widget? child) => Image(
              image: image,
              frameBuilder: (
                BuildContext context,
                Widget child,
                int? frame,
                bool wasSynchronouslyLoaded,
              ) {
                if (frame == 0) {
                  imageLoaded = true;
                }
                return LayoutBuilder(
                  builder:
                      (BuildContext context, BoxConstraints constraints) =>
                          ValueListenableBuilder<int>(
                            valueListenable: innerListenable,
                            builder:
                                (BuildContext context, int value, Widget? valueListenableChild) =>
                                    KeyedSubtree(key: UniqueKey(), child: child),
                          ),
                );
              },
            ),
      ),
    );

    imageLoaded = false;
    imageProvider1.complete(image10x10);
    await tester.idle();
    await tester.pump();
    expect(imageLoaded, true);

    imageLoaded = false;
    imageListenable.value = imageProvider2;
    innerListenable.value += 1;
    imageProvider2.complete(image100x100);
    await tester.idle();
    await tester.pump();
    expect(imageLoaded, true);
  });

  testWidgets('Verify Image resets its RenderImage when changing providers', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();
    final _TestImageProvider imageProvider1 = _TestImageProvider();
    await tester.pumpWidget(
      Container(key: key, child: Image(image: imageProvider1, excludeFromSemantics: true)),
      phase: EnginePhase.layout,
    );
    RenderImage renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNull);

    imageProvider1.complete(image10x10);
    await tester.idle(); // resolve the future from the image provider
    await tester.pump(null, EnginePhase.layout);

    renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNotNull);

    final _TestImageProvider imageProvider2 = _TestImageProvider();
    await tester.pumpWidget(
      Container(key: key, child: Image(image: imageProvider2, excludeFromSemantics: true)),
      phase: EnginePhase.layout,
    );

    renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNull);
  });

  testWidgets(
    "Verify Image doesn't reset its RenderImage when changing providers if it has gaplessPlayback set",
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final _TestImageProvider imageProvider1 = _TestImageProvider();
      await tester.pumpWidget(
        Container(
          key: key,
          child: Image(gaplessPlayback: true, image: imageProvider1, excludeFromSemantics: true),
        ),
        phase: EnginePhase.layout,
      );
      RenderImage renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNull);

      imageProvider1.complete(image10x10);
      await tester.idle(); // resolve the future from the image provider
      await tester.pump(null, EnginePhase.layout);

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);

      final _TestImageProvider imageProvider2 = _TestImageProvider();
      await tester.pumpWidget(
        Container(
          key: key,
          child: Image(gaplessPlayback: true, image: imageProvider2, excludeFromSemantics: true),
        ),
        phase: EnginePhase.layout,
      );

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);
    },
  );

  testWidgets('Verify Image resets its RenderImage when changing providers if it has a key', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();
    final _TestImageProvider imageProvider1 = _TestImageProvider();
    await tester.pumpWidget(
      Image(key: key, image: imageProvider1, excludeFromSemantics: true),
      phase: EnginePhase.layout,
    );
    RenderImage renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNull);

    imageProvider1.complete(image10x10);
    await tester.idle(); // resolve the future from the image provider
    await tester.pump(null, EnginePhase.layout);

    renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNotNull);

    final _TestImageProvider imageProvider2 = _TestImageProvider();
    await tester.pumpWidget(
      Image(key: key, image: imageProvider2, excludeFromSemantics: true),
      phase: EnginePhase.layout,
    );

    renderImage = key.currentContext!.findRenderObject()! as RenderImage;
    expect(renderImage.image, isNull);
  });

  testWidgets(
    "Verify Image doesn't reset its RenderImage when changing providers if it has gaplessPlayback set",
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final _TestImageProvider imageProvider1 = _TestImageProvider();
      await tester.pumpWidget(
        Image(key: key, gaplessPlayback: true, image: imageProvider1, excludeFromSemantics: true),
        phase: EnginePhase.layout,
      );
      RenderImage renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNull);

      imageProvider1.complete(image10x10);
      await tester.idle(); // resolve the future from the image provider
      await tester.pump(null, EnginePhase.layout);

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);

      final _TestImageProvider imageProvider2 = _TestImageProvider();
      await tester.pumpWidget(
        Image(key: key, gaplessPlayback: true, excludeFromSemantics: true, image: imageProvider2),
        phase: EnginePhase.layout,
      );

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);
    },
  );

  testWidgets('Verify ImageProvider configuration inheritance', (WidgetTester tester) async {
    final GlobalKey mediaQueryKey1 = GlobalKey(debugLabel: 'mediaQueryKey1');
    final GlobalKey mediaQueryKey2 = GlobalKey(debugLabel: 'mediaQueryKey2');
    final GlobalKey imageKey = GlobalKey(debugLabel: 'image');
    final _ConfigurationKeyedTestImageProvider imageProvider =
        _ConfigurationKeyedTestImageProvider();
    final Set<Object> seenKeys = <Object>{};
    final _DebouncingImageProvider debouncingProvider = _DebouncingImageProvider(
      imageProvider,
      seenKeys,
    );

    // Of the two nested MediaQuery objects, the innermost one,
    // mediaQuery2, should define the configuration of the imageProvider.
    await tester.pumpWidget(
      MediaQuery(
        key: mediaQueryKey1,
        data: const MediaQueryData(devicePixelRatio: 10.0),
        child: MediaQuery(
          key: mediaQueryKey2,
          data: const MediaQueryData(devicePixelRatio: 5.0),
          child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
        ),
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);

    // This is the same widget hierarchy as before except that the
    // two MediaQuery objects have exchanged places. The imageProvider
    // should be resolved again, with the new innermost MediaQuery.
    await tester.pumpWidget(
      MediaQuery(
        key: mediaQueryKey2,
        data: const MediaQueryData(devicePixelRatio: 5.0),
        child: MediaQuery(
          key: mediaQueryKey1,
          data: const MediaQueryData(devicePixelRatio: 10.0),
          child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
        ),
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 10.0);
  });

  testWidgets('Verify ImageProvider configuration inheritance again', (WidgetTester tester) async {
    final GlobalKey mediaQueryKey1 = GlobalKey(debugLabel: 'mediaQueryKey1');
    final GlobalKey mediaQueryKey2 = GlobalKey(debugLabel: 'mediaQueryKey2');
    final GlobalKey imageKey = GlobalKey(debugLabel: 'image');
    final _ConfigurationKeyedTestImageProvider imageProvider =
        _ConfigurationKeyedTestImageProvider();
    final Set<Object> seenKeys = <Object>{};
    final _DebouncingImageProvider debouncingProvider = _DebouncingImageProvider(
      imageProvider,
      seenKeys,
    );

    // This is just a variation on the previous test. In this version the location
    // of the Image changes and the MediaQuery widgets do not.
    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          MediaQuery(
            key: mediaQueryKey2,
            data: const MediaQueryData(devicePixelRatio: 5.0),
            child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
          ),
          MediaQuery(
            key: mediaQueryKey1,
            data: const MediaQueryData(devicePixelRatio: 10.0),
            child: Container(width: 100.0),
          ),
        ],
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);

    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          MediaQuery(
            key: mediaQueryKey2,
            data: const MediaQueryData(devicePixelRatio: 5.0),
            child: Container(width: 100.0),
          ),
          MediaQuery(
            key: mediaQueryKey1,
            data: const MediaQueryData(devicePixelRatio: 10.0),
            child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
          ),
        ],
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 10.0);
  });

  testWidgets('Verify ImageProvider does not inherit configuration when it does not key to it', (
    WidgetTester tester,
  ) async {
    final GlobalKey mediaQueryKey1 = GlobalKey(debugLabel: 'mediaQueryKey1');
    final GlobalKey mediaQueryKey2 = GlobalKey(debugLabel: 'mediaQueryKey2');
    final GlobalKey imageKey = GlobalKey(debugLabel: 'image');
    final _TestImageProvider imageProvider = _TestImageProvider();
    final Set<Object> seenKeys = <Object>{};
    final _DebouncingImageProvider debouncingProvider = _DebouncingImageProvider(
      imageProvider,
      seenKeys,
    );

    // Of the two nested MediaQuery objects, the innermost one,
    // mediaQuery2, should define the configuration of the imageProvider.
    await tester.pumpWidget(
      MediaQuery(
        key: mediaQueryKey1,
        data: const MediaQueryData(devicePixelRatio: 10.0),
        child: MediaQuery(
          key: mediaQueryKey2,
          data: const MediaQueryData(devicePixelRatio: 5.0),
          child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
        ),
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);

    // This is the same widget hierarchy as before except that the
    // two MediaQuery objects have exchanged places. The imageProvider
    // should not be resolved again, because it does not key to configuration.
    await tester.pumpWidget(
      MediaQuery(
        key: mediaQueryKey2,
        data: const MediaQueryData(devicePixelRatio: 5.0),
        child: MediaQuery(
          key: mediaQueryKey1,
          data: const MediaQueryData(devicePixelRatio: 10.0),
          child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
        ),
      ),
    );

    expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);
  });

  testWidgets(
    'Verify ImageProvider does not inherit configuration when it does not key to it again',
    (WidgetTester tester) async {
      final GlobalKey mediaQueryKey1 = GlobalKey(debugLabel: 'mediaQueryKey1');
      final GlobalKey mediaQueryKey2 = GlobalKey(debugLabel: 'mediaQueryKey2');
      final GlobalKey imageKey = GlobalKey(debugLabel: 'image');
      final _TestImageProvider imageProvider = _TestImageProvider();
      final Set<Object> seenKeys = <Object>{};
      final _DebouncingImageProvider debouncingProvider = _DebouncingImageProvider(
        imageProvider,
        seenKeys,
      );

      // This is just a variation on the previous test. In this version the location
      // of the Image changes and the MediaQuery widgets do not.
      await tester.pumpWidget(
        Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            MediaQuery(
              key: mediaQueryKey2,
              data: const MediaQueryData(devicePixelRatio: 5.0),
              child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
            ),
            MediaQuery(
              key: mediaQueryKey1,
              data: const MediaQueryData(devicePixelRatio: 10.0),
              child: Container(width: 100.0),
            ),
          ],
        ),
      );

      expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);

      await tester.pumpWidget(
        Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            MediaQuery(
              key: mediaQueryKey2,
              data: const MediaQueryData(devicePixelRatio: 5.0),
              child: Container(width: 100.0),
            ),
            MediaQuery(
              key: mediaQueryKey1,
              data: const MediaQueryData(devicePixelRatio: 10.0),
              child: Image(excludeFromSemantics: true, key: imageKey, image: debouncingProvider),
            ),
          ],
        ),
      );

      expect(imageProvider._lastResolvedConfiguration.devicePixelRatio, 5.0);
    },
  );

  testWidgets('Verify Image stops listening to ImageStream', (WidgetTester tester) async {
    final ui.Image image100x100 =
        (await tester.runAsync(() async => createTestImage(width: 100, height: 100)))!;
    // Web does not override the toString, whereas VM does
    final String imageString = image100x100.toString();

    final _TestImageProvider imageProvider = _TestImageProvider();
    await tester.pumpWidget(Image(image: imageProvider, excludeFromSemantics: true));
    final State<Image> image = tester.state /*State<Image>*/ (find.byType(Image));
    expect(
      image.toString(),
      equalsIgnoringHashCodes(
        '_ImageState#00000(stream: ImageStream#00000(OneFrameImageStreamCompleter#00000, unresolved, 2 listeners, 0 ephemeralErrorListeners), pixels: null, loadingProgress: null, frameNumber: null, wasSynchronouslyLoaded: false)',
      ),
    );
    imageProvider.complete(image100x100);
    await tester.pump();
    expect(
      image.toString(),
      equalsIgnoringHashCodes(
        '_ImageState#00000(stream: ImageStream#00000(OneFrameImageStreamCompleter#00000, $imageString @ 1.0x, 1 listener, 0 ephemeralErrorListeners), pixels: $imageString @ 1.0x, loadingProgress: null, frameNumber: 0, wasSynchronouslyLoaded: false)',
      ),
    );
    await tester.pumpWidget(Container());
    expect(
      image.toString(),
      equalsIgnoringHashCodes(
        '_ImageState#00000(lifecycle state: defunct, not mounted, stream: ImageStream#00000(OneFrameImageStreamCompleter#00000, $imageString @ 1.0x, 0 listeners, 0 ephemeralErrorListeners), pixels: null, loadingProgress: null, frameNumber: 0, wasSynchronouslyLoaded: false)',
      ),
    );
  });

  testWidgets('Stream completer errors can be listened to by attaching before resolving', (
    WidgetTester tester,
  ) async {
    dynamic capturedException;
    StackTrace? capturedStackTrace;
    ImageInfo? capturedImage;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
      capturedStackTrace = stackTrace;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    imageProvider.resolve(configuration);
    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    expect(capturedImage, isNull); // The image stream listeners should never be called.
    // The image stream error handler should have the original exception.
    expect(capturedException, testException);
    expect(capturedStackTrace, testStack);
    // If there is an error listener, there should be no FlutterError reported.
    expect(tester.takeException(), isNull);
  });

  testWidgets('Stream completer errors can be listened to by attaching after resolving', (
    WidgetTester tester,
  ) async {
    dynamic capturedException;
    StackTrace? capturedStackTrace;
    dynamic reportedException;
    StackTrace? reportedStackTrace;
    ImageInfo? capturedImage;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
      capturedStackTrace = stackTrace;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    FlutterError.onError = (FlutterErrorDetails flutterError) {
      reportedException = flutterError.exception;
      reportedStackTrace = flutterError.stack;
    };

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    final ImageStream streamUnderTest = imageProvider.resolve(configuration);

    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    // Since there's no listeners attached yet, report error up via
    // FlutterError.
    expect(reportedException, testException);
    expect(reportedStackTrace, testStack);

    streamUnderTest.addListener(ImageStreamListener(listener, onError: errorListener));

    expect(capturedImage, isNull); // The image stream listeners should never be called.
    // The image stream error handler should have the original exception.
    expect(capturedException, testException);
    expect(capturedStackTrace, testStack);
  });

  testWidgets('Duplicate listener registration does not affect error listeners', (
    WidgetTester tester,
  ) async {
    dynamic capturedException;
    StackTrace? capturedStackTrace;
    ImageInfo? capturedImage;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
      capturedStackTrace = stackTrace;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    // Add the exact same listener a second time without the errorListener.
    imageProvider._streamCompleter.addListener(ImageStreamListener(listener));
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    imageProvider.resolve(configuration);
    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    expect(capturedImage, isNull); // The image stream listeners should never be called.
    // The image stream error handler should have the original exception.
    expect(capturedException, testException);
    expect(capturedStackTrace, testStack);
    // If there is an error listener, there should be no FlutterError reported.
    expect(tester.takeException(), isNull);
  });

  testWidgets('Duplicate error listeners are all called', (WidgetTester tester) async {
    dynamic capturedException;
    StackTrace? capturedStackTrace;
    ImageInfo? capturedImage;
    int errorListenerCalled = 0;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
      capturedStackTrace = stackTrace;
      errorListenerCalled++;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    // Add the exact same errorListener a second time.
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    imageProvider.resolve(configuration);
    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    expect(capturedImage, isNull); // The image stream listeners should never be called.
    // The image stream error handler should have the original exception.
    expect(capturedException, testException);
    expect(capturedStackTrace, testStack);
    expect(errorListenerCalled, 2);
    // If there is an error listener, there should be no FlutterError reported.
    expect(tester.takeException(), isNull);
  });

  testWidgets('Listeners are only removed if callback tuple matches', (WidgetTester tester) async {
    bool errorListenerCalled = false;
    dynamic reportedException;
    StackTrace? reportedStackTrace;
    ImageInfo? capturedImage;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      errorListenerCalled = true;
      reportedException = exception;
      reportedStackTrace = stackTrace;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    // Now remove the listener the error listener is attached to.
    // Don't explicitly remove the error listener.
    imageProvider._streamCompleter.removeListener(ImageStreamListener(listener));
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    imageProvider.resolve(configuration);

    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    expect(errorListenerCalled, true);
    expect(reportedException, testException);
    expect(reportedStackTrace, testStack);
    expect(capturedImage, isNull); // The image stream listeners should never be called.
  });

  testWidgets('Removing listener removes one listener and error listener', (
    WidgetTester tester,
  ) async {
    int errorListenerCalled = 0;
    ImageInfo? capturedImage;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      errorListenerCalled++;
    }

    void listener(ImageInfo info, bool synchronous) {
      capturedImage = info;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    // Duplicates the same set of listener and errorListener.
    imageProvider._streamCompleter.addListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    // Now remove one entry of the specified listener and associated error listener.
    // Don't explicitly remove the error listener.
    imageProvider._streamCompleter.removeListener(
      ImageStreamListener(listener, onError: errorListener),
    );
    late ImageConfiguration configuration;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          configuration = createLocalImageConfiguration(context);
          return Container();
        },
      ),
    );
    imageProvider.resolve(configuration);

    imageProvider.fail(testException, testStack);

    expect(tester.binding.microtaskCount, 1);
    await tester.idle(); // Let the failed completer's future hit the stream completer.
    expect(tester.binding.microtaskCount, 0);

    expect(errorListenerCalled, 1);
    expect(capturedImage, isNull); // The image stream listeners should never be called.
  });

  testWidgets('Image.memory control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      Image.memory(Uint8List.fromList(kTransparentImage), excludeFromSemantics: true),
    );
  });

  testWidgets('Image color and colorBlend parameters', (WidgetTester tester) async {
    await tester.pumpWidget(
      Image(
        excludeFromSemantics: true,
        image: _TestImageProvider(),
        color: const Color(0xFF00FF00),
        colorBlendMode: BlendMode.clear,
      ),
    );
    final RenderImage renderer = tester.renderObject<RenderImage>(find.byType(Image));
    expect(renderer.color, const Color(0xFF00FF00));
    expect(renderer.colorBlendMode, BlendMode.clear);
  });

  testWidgets('Image opacity parameter', (WidgetTester tester) async {
    const Animation<double> opacity = AlwaysStoppedAnimation<double>(0.5);
    await tester.pumpWidget(
      Image(excludeFromSemantics: true, image: _TestImageProvider(), opacity: opacity),
    );
    final RenderImage renderer = tester.renderObject<RenderImage>(find.byType(Image));
    expect(renderer.opacity, opacity);
  });

  testWidgets('Precache', (WidgetTester tester) async {
    final _TestImageProvider provider = _TestImageProvider();
    late Future<void> precache;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          precache = precacheImage(provider, context);
          return Container();
        },
      ),
    );
    provider.complete(image10x10);
    await precache;
    expect(provider._lastResolvedConfiguration, isNotNull);

    // Check that a second resolve of the same image is synchronous.
    final ImageStream stream = provider.resolve(provider._lastResolvedConfiguration);
    late bool isSync;
    stream.addListener(
      ImageStreamListener((ImageInfo image, bool sync) {
        image.dispose();
        isSync = sync;
      }),
    );
    expect(isSync, isTrue);
  });

  testWidgets(
    'Precache removes original listener immediately after future completes, does not crash on successive calls #25143',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final _TestImageStreamCompleter imageStreamCompleter = _TestImageStreamCompleter();
      final _TestImageProvider provider = _TestImageProvider(streamCompleter: imageStreamCompleter);

      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            precacheImage(provider, context);
            return Container();
          },
        ),
      );

      // Two listeners - one is the listener added by precacheImage, the other by the ImageCache.
      final List<ImageStreamListener> listeners = imageStreamCompleter.listeners.toList();
      expect(listeners.length, 2);

      // Make sure the first listener can be called re-entrantly
      final ImageInfo imageInfo = ImageInfo(image: image10x10);

      listeners[1].onImage(imageInfo.clone(), false);
      listeners[1].onImage(imageInfo.clone(), false);

      // Make sure the second listener can be called re-entrantly.
      listeners[0].onImage(imageInfo.clone(), false);
      listeners[0].onImage(imageInfo.clone(), false);

      imageInfo.dispose();
      imageStreamCompleter.dispose();
      imageCache.clear();
    },
  );

  testWidgets('Precache completes with onError on error', (WidgetTester tester) async {
    dynamic capturedException;
    StackTrace? capturedStackTrace;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
      capturedStackTrace = stackTrace;
    }

    final Exception testException = Exception('cannot resolve host');
    final StackTrace testStack = StackTrace.current;
    final _TestImageProvider imageProvider = _TestImageProvider();
    late Future<void> precache;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          precache = precacheImage(imageProvider, context, onError: errorListener);
          return Container();
        },
      ),
    );
    imageProvider.fail(testException, testStack);
    await precache;

    // The image stream error handler should have the original exception.
    expect(capturedException, testException);
    expect(capturedStackTrace, testStack);
    // If there is an error listener, there should be no FlutterError reported.
    expect(tester.takeException(), isNull);
  });

  testWidgets('TickerMode controls stream registration', (WidgetTester tester) async {
    final _TestImageStreamCompleter imageStreamCompleter = _TestImageStreamCompleter();
    final Image image = Image(
      excludeFromSemantics: true,
      image: _TestImageProvider(streamCompleter: imageStreamCompleter),
    );
    await tester.pumpWidget(TickerMode(enabled: true, child: image));
    expect(imageStreamCompleter.listeners.length, 2);
    await tester.pumpWidget(TickerMode(enabled: false, child: image));
    expect(imageStreamCompleter.listeners.length, 1);
  });

  testWidgets(
    'Verify Image shows correct RenderImage when changing to an already completed provider',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();

      final _TestImageProvider imageProvider1 = _TestImageProvider();
      final _TestImageProvider imageProvider2 = _TestImageProvider();
      final ui.Image image100x100 =
          (await tester.runAsync(() async => createTestImage(width: 100, height: 100)))!;

      await tester.pumpWidget(
        Container(key: key, child: Image(excludeFromSemantics: true, image: imageProvider1)),
        phase: EnginePhase.layout,
      );
      RenderImage renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNull);

      imageProvider1.complete(image10x10);
      imageProvider2.complete(image100x100);
      await tester.idle(); // resolve the future from the image provider
      await tester.pump(null, EnginePhase.layout);

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);

      final ui.Image oldImage = renderImage.image!;

      await tester.pumpWidget(
        Container(key: key, child: Image(excludeFromSemantics: true, image: imageProvider2)),
        phase: EnginePhase.layout,
      );

      renderImage = key.currentContext!.findRenderObject()! as RenderImage;
      expect(renderImage.image, isNotNull);
      expect(renderImage.image, isNot(equals(oldImage)));
    },
  );

  testWidgets('Image State can be reconfigured to use another image', (WidgetTester tester) async {
    final Image image1 = Image(
      image: _TestImageProvider()..complete(image10x10.clone()),
      width: 10.0,
      excludeFromSemantics: true,
    );
    final Image image2 = Image(
      image: _TestImageProvider()..complete(image10x10.clone()),
      width: 20.0,
      excludeFromSemantics: true,
    );

    final Column column = Column(children: <Widget>[image1, image2]);
    await tester.pumpWidget(column, phase: EnginePhase.layout);

    final Column columnSwapped = Column(children: <Widget>[image2, image1]);
    await tester.pumpWidget(columnSwapped, phase: EnginePhase.layout);

    final List<RenderImage> renderObjects =
        tester.renderObjectList<RenderImage>(find.byType(Image)).toList();
    expect(renderObjects, hasLength(2));
    expect(renderObjects[0].image, isNotNull);
    expect(renderObjects[0].width, 20.0);
    expect(renderObjects[1].image, isNotNull);
    expect(renderObjects[1].width, 10.0);
  });

  testWidgets('Image contributes semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Row(
          children: <Widget>[
            Image(image: _TestImageProvider(), width: 100.0, height: 100.0, semanticLabel: 'test'),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              label: 'test',
              rect: const Rect.fromLTWH(0.0, 0.0, 100.0, 100.0),
              textDirection: TextDirection.ltr,
              flags: <SemanticsFlag>[SemanticsFlag.isImage],
            ),
          ],
        ),
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Image can exclude semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Image(
          image: _TestImageProvider(),
          width: 100.0,
          height: 100.0,
          excludeFromSemantics: true,
        ),
      ),
    );

    expect(semantics, hasSemantics(TestSemantics.root(children: <TestSemantics>[])));
    semantics.dispose();
  });

  testWidgets(
    'Image invokes frameBuilder with correct frameNumber argument',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final ui.Codec codec =
          (await tester.runAsync(() {
            return ui.instantiateImageCodec(Uint8List.fromList(kAnimatedGif));
          }))!;

      Future<ui.Image> nextFrame() async {
        final ui.FrameInfo frameInfo = (await tester.runAsync(codec.getNextFrame))!;
        return frameInfo.image;
      }

      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
      int? lastFrame;

      await tester.pumpWidget(
        Image(
          image: imageProvider,
          frameBuilder: (
            BuildContext context,
            Widget child,
            int? frame,
            bool wasSynchronouslyLoaded,
          ) {
            lastFrame = frame;
            return Center(child: child);
          },
        ),
      );

      expect(lastFrame, isNull);
      expect(find.byType(Center), findsOneWidget);
      expect(find.byType(RawImage), findsOneWidget);
      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 0);
      expect(find.byType(Center), findsOneWidget);
      expect(find.byType(RawImage), findsOneWidget);
      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 1);
      expect(find.byType(Center), findsOneWidget);
      expect(find.byType(RawImage), findsOneWidget);
    },
  );

  testWidgets('Image invokes frameBuilder with correct wasSynchronouslyLoaded=false', (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
    int? lastFrame;
    late bool lastFrameWasSync;

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        frameBuilder: (
          BuildContext context,
          Widget child,
          int? frame,
          bool wasSynchronouslyLoaded,
        ) {
          lastFrame = frame;
          lastFrameWasSync = wasSynchronouslyLoaded;
          return child;
        },
      ),
    );

    expect(lastFrame, isNull);
    expect(lastFrameWasSync, isFalse);
    expect(find.byType(RawImage), findsOneWidget);

    final ImageInfo info = ImageInfo(image: image10x10);
    addTearDown(info.dispose);
    streamCompleter.setData(imageInfo: info);
    await tester.pump();

    expect(lastFrame, 0);
    expect(lastFrameWasSync, isFalse);
  });

  testWidgets(
    'Image invokes frameBuilder with correct wasSynchronouslyLoaded=true',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter(
        ImageInfo(image: image10x10.clone()),
      );
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
      int? lastFrame;
      late bool lastFrameWasSync;

      await tester.pumpWidget(
        Image(
          image: imageProvider,
          frameBuilder: (
            BuildContext context,
            Widget child,
            int? frame,
            bool wasSynchronouslyLoaded,
          ) {
            lastFrame = frame;
            lastFrameWasSync = wasSynchronouslyLoaded;
            return child;
          },
        ),
      );

      expect(lastFrame, 0);
      expect(lastFrameWasSync, isTrue);
      expect(find.byType(RawImage), findsOneWidget);
      streamCompleter.setData(imageInfo: ImageInfo(image: image10x10.clone()));
      await tester.pump();
      expect(lastFrame, 1);
      expect(lastFrameWasSync, isTrue);
    },
  );

  testWidgets('Image state handles frameBuilder update', (WidgetTester tester) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        frameBuilder: (
          BuildContext context,
          Widget child,
          int? frame,
          bool wasSynchronouslyLoaded,
        ) {
          return Center(child: child);
        },
      ),
    );

    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    final State<Image> state = tester.state(find.byType(Image));

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        frameBuilder: (
          BuildContext context,
          Widget child,
          int? frame,
          bool wasSynchronouslyLoaded,
        ) {
          return Padding(padding: const EdgeInsets.all(1), child: child);
        },
      ),
    );

    expect(find.byType(Center), findsNothing);
    expect(find.byType(Padding), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    expect(tester.state(find.byType(Image)), same(state));
  });

  testWidgets(
    'Image state handles enabling and disabling of tickers',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final ui.Codec codec =
          (await tester.runAsync(() {
            return ui.instantiateImageCodec(Uint8List.fromList(kAnimatedGif));
          }))!;

      Future<ui.Image> nextFrame() async {
        final ui.FrameInfo frameInfo = (await tester.runAsync(codec.getNextFrame))!;
        return frameInfo.image;
      }

      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
      int? lastFrame;
      int buildCount = 0;

      Widget buildFrame(
        BuildContext context,
        Widget child,
        int? frame,
        bool wasSynchronouslyLoaded,
      ) {
        lastFrame = frame;
        buildCount++;
        return child;
      }

      await tester.pumpWidget(
        TickerMode(enabled: true, child: Image(image: imageProvider, frameBuilder: buildFrame)),
      );

      final State<Image> state = tester.state(find.byType(Image));
      expect(lastFrame, isNull);
      expect(buildCount, 1);
      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 0);
      expect(buildCount, 2);

      await tester.pumpWidget(
        TickerMode(enabled: false, child: Image(image: imageProvider, frameBuilder: buildFrame)),
      );

      expect(tester.state(find.byType(Image)), same(state));
      expect(lastFrame, 0);
      expect(buildCount, 3);
      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 0);
      expect(buildCount, 3);

      await tester.pumpWidget(
        TickerMode(enabled: true, child: Image(image: imageProvider, frameBuilder: buildFrame)),
      );

      expect(tester.state(find.byType(Image)), same(state));
      expect(lastFrame, 1); // missed a frame because we weren't animating at the time
      expect(buildCount, 4);
    },
  );

  testWidgets('Image invokes loadingBuilder on chunk event notification', (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
    final List<ImageChunkEvent?> chunkEvents = <ImageChunkEvent?>[];

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
          chunkEvents.add(loadingProgress);
          if (loadingProgress == null) {
            return child;
          }
          return Directionality(
            textDirection: TextDirection.ltr,
            child: Text(
              'loading ${loadingProgress.cumulativeBytesLoaded} / ${loadingProgress.expectedTotalBytes}',
            ),
          );
        },
      ),
    );

    expect(chunkEvents.length, 1);
    expect(chunkEvents.first, isNull);
    expect(tester.binding.hasScheduledFrame, isFalse);
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    expect(chunkEvents.length, 2);
    expect(find.text('loading 10 / 100'), findsOneWidget);
    expect(find.byType(RawImage), findsNothing);
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 30, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    expect(chunkEvents.length, 3);
    expect(find.text('loading 30 / 100'), findsOneWidget);
    expect(find.byType(RawImage), findsNothing);

    final ImageInfo info = ImageInfo(image: image10x10);
    addTearDown(info.dispose);
    streamCompleter.setData(imageInfo: info);
    await tester.pump();

    expect(chunkEvents.length, 4);
    expect(find.byType(Text), findsNothing);
    expect(find.byType(RawImage), findsOneWidget);
  });

  testWidgets("Image doesn't rebuild on chunk events if loadingBuilder is null", (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(Image(image: imageProvider, excludeFromSemantics: true));

    expect(tester.binding.hasScheduledFrame, isFalse);
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isFalse);
    final ImageInfo info = ImageInfo(image: image10x10);
    addTearDown(info.dispose);
    streamCompleter.setData(imageInfo: info);
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isFalse);
    expect(find.byType(RawImage), findsOneWidget);
  });

  testWidgets('Image chains the results of frameBuilder and loadingBuilder', (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        excludeFromSemantics: true,
        frameBuilder: (
          BuildContext context,
          Widget child,
          int? frame,
          bool wasSynchronouslyLoaded,
        ) {
          return Padding(padding: const EdgeInsets.all(1), child: child);
        },
        loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
          return Center(child: child);
        },
      ),
    );

    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(Padding), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    expect(tester.widget<Padding>(find.byType(Padding)).child, isA<RawImage>());
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    await tester.pump();
    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(Padding), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    expect(tester.widget<Center>(find.byType(Center)).child, isA<Padding>());
    expect(tester.widget<Padding>(find.byType(Padding)).child, isA<RawImage>());
  });

  testWidgets('Image state handles loadingBuilder update from null to non-null', (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(Image(image: imageProvider));

    expect(find.byType(RawImage), findsOneWidget);
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isFalse);
    final State<Image> state = tester.state(find.byType(Image));

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
          return Center(child: child);
        },
      ),
    );

    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    expect(tester.state(find.byType(Image)), same(state));
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
  });

  testWidgets('Image state handles loadingBuilder update from non-null to null', (
    WidgetTester tester,
  ) async {
    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(
      Image(
        image: imageProvider,
        loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
          return Center(child: child);
        },
      ),
    );

    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(RawImage), findsOneWidget);
    final State<Image> state = tester.state(find.byType(Image));

    await tester.pumpWidget(Image(image: imageProvider));

    expect(find.byType(Center), findsNothing);
    expect(find.byType(RawImage), findsOneWidget);
    expect(tester.state(find.byType(Image)), same(state));
    streamCompleter.setData(
      chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
    );
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  testWidgets('Verify Image resets its ImageListeners', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    final _TestImageStreamCompleter imageStreamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider1 = _TestImageProvider(
      streamCompleter: imageStreamCompleter,
    );
    await tester.pumpWidget(Container(key: key, child: Image(image: imageProvider1)));
    // listener from resolveStreamForKey is always added.
    expect(imageStreamCompleter.listeners.length, 2);

    final _TestImageProvider imageProvider2 = _TestImageProvider();
    await tester.pumpWidget(
      Container(key: key, child: Image(image: imageProvider2, excludeFromSemantics: true)),
      phase: EnginePhase.layout,
    );

    // only listener from resolveStreamForKey is left.
    expect(imageStreamCompleter.listeners.length, 1);
  });

  testWidgets('Verify Image resets its ErrorListeners', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    final _TestImageStreamCompleter imageStreamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider1 = _TestImageProvider(
      streamCompleter: imageStreamCompleter,
    );
    await tester.pumpWidget(
      Container(
        key: key,
        child: Image(image: imageProvider1, errorBuilder: (_, __, ___) => Container()),
      ),
    );
    // listener from resolveStreamForKey is always added.
    expect(imageStreamCompleter.listeners.length, 2);

    final _TestImageProvider imageProvider2 = _TestImageProvider();
    await tester.pumpWidget(
      Container(key: key, child: Image(image: imageProvider2, excludeFromSemantics: true)),
      phase: EnginePhase.layout,
    );

    // only listener from resolveStreamForKey is left.
    expect(imageStreamCompleter.listeners.length, 1);
  });

  testWidgets('Image defers loading while fast scrolling', (WidgetTester tester) async {
    const int gridCells = 1000;
    final List<_TestImageProvider> imageProviders = <_TestImageProvider>[];
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.builder(
          controller: controller,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
          itemCount: gridCells,
          itemBuilder: (_, int index) {
            final _TestImageProvider provider = _TestImageProvider();
            imageProviders.add(provider);
            return SizedBox(
              height: 250,
              width: 250,
              child: Image(image: provider, semanticLabel: index.toString()),
            );
          },
        ),
      ),
    );

    bool loadCalled(_TestImageProvider provider) => provider.loadCalled;
    bool loadNotCalled(_TestImageProvider provider) => !provider.loadCalled;

    expect(find.bySemanticsLabel('5'), findsOneWidget);
    expect(imageProviders.length, 12);
    expect(imageProviders.every(loadCalled), true);

    imageProviders.clear();

    // Simulate a very fast fling.
    controller.animateTo(30000, duration: const Duration(seconds: 2), curve: Curves.linear);
    await tester.pumpAndSettle();
    // The last 15 images on screen have loaded because the scrolling settled there.
    // The rest have not loaded.
    expect(imageProviders.length, 309);
    expect(imageProviders.skip(309 - 15).every(loadCalled), true);
    expect(imageProviders.take(309 - 15).every(loadNotCalled), true);
  });

  testWidgets('Same image provider in multiple parts of the tree, no cache room left', (
    WidgetTester tester,
  ) async {
    imageCache.maximumSize = 0;

    final _TestImageProvider provider1 = _TestImageProvider();
    final _TestImageProvider provider2 = _TestImageProvider();

    expect(provider1.loadCallCount, 0);
    expect(provider2.loadCallCount, 0);
    expect(imageCache.liveImageCount, 0);

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          Image(image: provider1),
          Image(image: provider2),
          Image(image: provider1),
          Image(image: provider1),
          Image(image: provider2),
        ],
      ),
    );

    expect(imageCache.liveImageCount, 2);
    expect(imageCache.statusForKey(provider1).live, true);
    expect(imageCache.statusForKey(provider1).pending, false);
    expect(imageCache.statusForKey(provider1).keepAlive, false);
    expect(imageCache.statusForKey(provider2).live, true);
    expect(imageCache.statusForKey(provider2).pending, false);
    expect(imageCache.statusForKey(provider2).keepAlive, false);

    expect(provider1.loadCallCount, 1);
    expect(provider2.loadCallCount, 1);

    provider1.complete(image10x10.clone());
    await tester.idle();

    provider2.complete(image10x10.clone());
    await tester.idle();

    expect(imageCache.liveImageCount, 2);
    expect(imageCache.currentSize, 0);

    await tester.pumpWidget(Image(image: provider2));
    await tester.idle();
    expect(imageCache.statusForKey(provider1).untracked, true);
    expect(imageCache.statusForKey(provider2).live, true);
    expect(imageCache.statusForKey(provider2).pending, false);
    expect(imageCache.statusForKey(provider2).keepAlive, false);
    expect(imageCache.liveImageCount, 1);

    await tester.pumpWidget(const SizedBox());
    await tester.idle();
    expect(provider1.loadCallCount, 1);
    expect(provider2.loadCallCount, 1);
    expect(imageCache.liveImageCount, 0);
  });

  testWidgets('precacheImage does not hold weak ref for more than a frame', (
    WidgetTester tester,
  ) async {
    imageCache.maximumSize = 0;
    final _TestImageProvider provider = _TestImageProvider();
    late Future<void> precache;
    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          precache = precacheImage(provider, context);
          return Container();
        },
      ),
    );
    provider.complete(image10x10);
    await precache;

    // Should have ended up with only a weak ref, not in cache because cache size is 0
    expect(imageCache.liveImageCount, 1);
    expect(imageCache.containsKey(provider), false);

    final ImageCacheStatus providerLocation =
        (await provider.obtainCacheStatus(configuration: ImageConfiguration.empty))!;

    expect(providerLocation, isNotNull);
    expect(providerLocation.live, true);
    expect(providerLocation.keepAlive, false);
    expect(providerLocation.pending, false);

    // Check that a second resolve of the same image is synchronous.
    expect(provider._lastResolvedConfiguration, isNotNull);
    final ImageStream stream = provider.resolve(provider._lastResolvedConfiguration);
    late bool isSync;
    final ImageStreamListener listener = ImageStreamListener((ImageInfo image, bool syncCall) {
      image.dispose();
      isSync = syncCall;
    });

    // Still have live ref because frame has not pumped yet.
    await tester.pump();
    expect(imageCache.liveImageCount, 1);

    SchedulerBinding.instance.scheduleFrame();
    await tester.pump();
    // Live ref should be gone - we didn't listen to the stream.
    expect(imageCache.liveImageCount, 0);
    expect(imageCache.currentSize, 0);

    stream.addListener(listener);
    expect(isSync, true); // because the stream still has the image.

    expect(imageCache.liveImageCount, 0);
    expect(imageCache.currentSize, 0);

    expect(provider.loadCallCount, 1);
  });

  testWidgets(
    'precacheImage allows time to take over weak reference',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final _TestImageProvider provider = _TestImageProvider();
      late Future<void> precache;
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            precache = precacheImage(provider, context);
            return Container();
          },
        ),
      );
      provider.complete(image10x10);
      await precache;

      // Should have ended up in the cache and have a weak reference.
      expect(imageCache.liveImageCount, 1);
      expect(imageCache.currentSize, 1);
      expect(imageCache.containsKey(provider), true);

      // Check that a second resolve of the same image is synchronous.
      expect(provider._lastResolvedConfiguration, isNotNull);
      final ImageStream stream = provider.resolve(provider._lastResolvedConfiguration);
      late bool isSync;
      final ImageStreamListener listener = ImageStreamListener((ImageInfo image, bool syncCall) {
        isSync = syncCall;
      });

      // Should have ended up in the cache and still have a weak reference.
      expect(imageCache.liveImageCount, 1);
      expect(imageCache.currentSize, 1);
      expect(imageCache.containsKey(provider), true);

      stream.addListener(listener);
      expect(isSync, true);

      expect(imageCache.liveImageCount, 1);
      expect(imageCache.currentSize, 1);
      expect(imageCache.containsKey(provider), true);

      SchedulerBinding.instance.scheduleFrame();
      await tester.pump();

      expect(imageCache.liveImageCount, 1);
      expect(imageCache.currentSize, 1);
      expect(imageCache.containsKey(provider), true);
      stream.removeListener(listener);

      expect(imageCache.liveImageCount, 0);
      expect(imageCache.currentSize, 1);
      expect(imageCache.containsKey(provider), true);
      expect(provider.loadCallCount, 1);
    },
  );

  testWidgets('evict an image during precache', (WidgetTester tester) async {
    // This test checks that the live image tracking does not hold on to a
    // pending image that will never complete because it has been evicted from
    // the cache.
    // The scenario may arise in a test harness that is trying to load real
    // images using `tester.runAsync()`, and wants to make sure that widgets
    // under test have not also tried to resolve the image in a FakeAsync zone.
    // The image loaded in the FakeAsync zone will never complete, and the
    // runAsync call wants to make sure it gets a load attempt from the correct
    // zone.
    final Uint8List bytes = Uint8List.fromList(kTransparentImage);
    final MemoryImage provider = MemoryImage(bytes);

    await tester.runAsync(() async {
      final List<Future<void>> futures = <Future<void>>[];
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            futures.add(precacheImage(provider, context));
            imageCache.evict(provider);
            futures.add(precacheImage(provider, context));
            return const SizedBox.expand();
          },
        ),
      );
      await Future.wait<void>(futures);
      expect(imageCache.statusForKey(provider).keepAlive, true);
      expect(imageCache.statusForKey(provider).live, true);

      // Schedule a frame to get precacheImage to stop listening.
      SchedulerBinding.instance.scheduleFrame();
      await tester.pump();
      expect(imageCache.statusForKey(provider).keepAlive, true);
      expect(imageCache.statusForKey(provider).live, false);

      imageCache.clear();
    });
  });

  testWidgets('errorBuilder - fails on key', (WidgetTester tester) async {
    final UniqueKey errorKey = UniqueKey();
    late Object caughtException;
    await tester.pumpWidget(
      Image(
        image: _FailingImageProvider(failOnObtainKey: true, throws: 'threw', image: image10x10),
        errorBuilder: (BuildContext context, Object error, StackTrace? stackTrace) {
          caughtException = error;
          return SizedBox.expand(key: errorKey);
        },
      ),
    );

    await tester.pump();

    expect(find.byKey(errorKey), findsOneWidget);
    expect(caughtException.toString(), 'threw');
    expect(tester.takeException(), isNull);
  });

  testWidgets('errorBuilder - fails on load', (WidgetTester tester) async {
    final UniqueKey errorKey = UniqueKey();
    late Object caughtException;
    await tester.pumpWidget(
      Image(
        image: _FailingImageProvider(failOnLoad: true, throws: 'threw', image: image10x10),
        errorBuilder: (BuildContext context, Object error, StackTrace? stackTrace) {
          caughtException = error;
          return SizedBox.expand(key: errorKey);
        },
      ),
    );

    await tester.pump();

    expect(find.byKey(errorKey), findsOneWidget);
    expect(caughtException.toString(), 'threw');
    expect(tester.takeException(), isNull);
  });

  testWidgets('no errorBuilder - failure reported to FlutterError', (WidgetTester tester) async {
    await tester.pumpWidget(
      Image(image: _FailingImageProvider(failOnLoad: true, throws: 'threw', image: image10x10)),
    );

    await tester.pump();

    expect(tester.takeException(), 'threw');
  });

  Future<void> testRotatedImage(WidgetTester tester, bool isAntiAlias) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      RepaintBoundary(
        key: key,
        child: Transform.rotate(
          angle: math.pi / 180,
          child: Image.memory(Uint8List.fromList(kBlueRectPng), isAntiAlias: isAntiAlias),
        ),
      ),
    );

    // precacheImage is needed, or the image in the golden file will be empty.
    if (!kIsWeb) {
      final Finder allImages = find.byType(Image);
      for (final Element e in allImages.evaluate()) {
        await tester.runAsync(() async {
          final Image image = e.widget as Image;
          await precacheImage(image.image, e);
        });
      }
      await tester.pumpAndSettle();
    }

    await expectLater(
      find.byKey(key),
      matchesGoldenFile('rotated_image_${isAntiAlias ? 'aa' : 'noaa'}.png'),
    );
  }

  testWidgets(
    'Rotated images',
    (WidgetTester tester) async {
      await testRotatedImage(tester, true);
      await testRotatedImage(tester, false);
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/87933.
  );

  testWidgets(
    'Image opacity',
    (WidgetTester tester) async {
      final Key key = UniqueKey();
      await tester.pumpWidget(
        RepaintBoundary(
          key: key,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            textDirection: TextDirection.ltr,
            children: <Widget>[
              Image.memory(
                Uint8List.fromList(kBlueRectPng),
                opacity: const AlwaysStoppedAnimation<double>(0.25),
              ),
              Image.memory(
                Uint8List.fromList(kBlueRectPng),
                opacity: const AlwaysStoppedAnimation<double>(0.5),
              ),
              Image.memory(
                Uint8List.fromList(kBlueRectPng),
                opacity: const AlwaysStoppedAnimation<double>(0.75),
              ),
              Image.memory(
                Uint8List.fromList(kBlueRectPng),
                opacity: const AlwaysStoppedAnimation<double>(1.0),
              ),
            ],
          ),
        ),
      );

      // precacheImage is needed, or the image in the golden file will be empty.
      if (!kIsWeb) {
        final Finder allImages = find.byType(Image);
        for (final Element e in allImages.evaluate()) {
          await tester.runAsync(() async {
            final Image image = e.widget as Image;
            await precacheImage(image.image, e);
          });
        }
        await tester.pumpAndSettle();
      }

      await expectLater(find.byKey(key), matchesGoldenFile('transparent_image.png'));
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/87933.
  );

  testWidgets(
    'Reports image size when painted',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      late ImageSizeInfo imageSizeInfo;
      int count = 0;
      debugOnPaintImage = (ImageSizeInfo info) {
        count += 1;
        imageSizeInfo = info;
      };

      final ui.Image image =
          (await tester.runAsync(() => createTestImage(width: 100, height: 100)))!;
      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter(
        ImageInfo(image: image, debugLabel: 'test.png'),
      );
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

      await tester.pumpWidget(
        Center(child: SizedBox(height: 50, width: 50, child: Image(image: imageProvider))),
      );

      expect(count, 1);
      expect(
        imageSizeInfo,
        const ImageSizeInfo(
          source: 'test.png',
          imageSize: Size(100, 100),
          displaySize: Size(150, 150),
        ),
      );

      debugOnPaintImage = null;
    },
  );

  testWidgets('Disposes image handle when disposed', (WidgetTester tester) async {
    final ui.Image image = (await tester.runAsync(() => createTestImage(cache: false)))!;

    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    final ImageProvider provider = _TestImageProvider(
      streamCompleter: OneFrameImageStreamCompleter(
        Future<ImageInfo>.value(ImageInfo(image: image, debugLabel: '_TestImage')),
      ),
    );

    // creating the provider should not have changed anything, and the provider
    // now owns the handle.
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    await tester.pumpWidget(Image(image: provider));

    // Image widget + 1, render object + 1
    expect(image.debugGetOpenHandleStackTraces()!.length, 3);

    await tester.pumpWidget(const SizedBox());

    // Image widget and render object go away
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    await provider.evict();

    tester.binding.scheduleFrame();
    await tester.pump();

    // Image cache listener go away and Image stream listeners go away.
    // Image is now at zero.
    expect(image.debugGetOpenHandleStackTraces()!.length, 0);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442

  testWidgets('Keeps stream alive when ticker mode is disabled', (WidgetTester tester) async {
    imageCache.maximumSize = 0;
    final ui.Image image = (await tester.runAsync(() => createTestImage(cache: false)))!;
    final _TestImageProvider provider = _TestImageProvider();
    provider.complete(image);

    await tester.pumpWidget(TickerMode(enabled: true, child: Image(image: provider)));
    expect(find.byType(Image), findsOneWidget);

    await tester.pumpWidget(TickerMode(enabled: false, child: Image(image: provider)));
    expect(find.byType(Image), findsOneWidget);

    await tester.pumpWidget(TickerMode(enabled: true, child: Image(image: provider)));
    expect(find.byType(Image), findsOneWidget);
  });

  testWidgets(
    'Load a good image after a bad image was loaded should not call errorBuilder',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final UniqueKey errorKey = UniqueKey();
      final ui.Image image = (await tester.runAsync(() => createTestImage()))!;
      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            height: 50,
            width: 50,
            child: Image(
              image: imageProvider,
              excludeFromSemantics: true,
              errorBuilder: (BuildContext context, Object error, StackTrace? stackTrace) {
                return Container(key: errorKey);
              },
              frameBuilder: (
                BuildContext context,
                Widget child,
                int? frame,
                bool wasSynchronouslyLoaded,
              ) {
                return Padding(padding: const EdgeInsets.all(1), child: child);
              },
            ),
          ),
        ),
      );

      // No error widget before loading a invalid image.
      expect(find.byKey(errorKey), findsNothing);

      // Loading good image succeed
      streamCompleter.setData(
        chunkEvent: const ImageChunkEvent(cumulativeBytesLoaded: 10, expectedTotalBytes: 100),
      );
      await tester.pump();
      expect(find.byType(Padding), findsOneWidget);

      // Loading bad image shows the error widget.
      streamCompleter.setError(exception: 'thrown');
      await tester.pump();
      expect(find.byKey(errorKey), findsOneWidget);

      // Loading good image shows the image widget instead of the error widget.
      streamCompleter.setData(imageInfo: ImageInfo(image: image));
      await tester.pump();
      expect(find.byType(Padding), findsOneWidget);
      expect(tester.widget<Padding>(find.byType(Padding)).child, isA<RawImage>());
      expect(find.byKey(errorKey), findsNothing);
    },
  );

  testWidgets(
    'Failed image loads in debug mode',
    (WidgetTester tester) async {
      final Key key = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: RepaintBoundary(
            key: key,
            child: Container(
              width: 150.0,
              height: 50.0,
              decoration: BoxDecoration(
                border: Border.all(width: 2.0, color: const Color(0xFF00FF99)),
              ),
              child: Image.asset('missing-asset'),
            ),
          ),
        ),
      );
      await expectLater(find.byKey(key), matchesGoldenFile('image_test.missing.1.png'));
      expect(
        tester.takeException().toString(),
        equals(
          'Unable to load asset: "missing-asset".\n'
          'The asset does not exist or has empty data.',
        ),
      );
      await tester.pump();
      await expectLater(find.byKey(key), matchesGoldenFile('image_test.missing.2.png'));
    },
    // https://github.com/flutter/flutter/issues/74935 (broken assets not being reported on web)
    skip: kIsWeb,
  );

  testWidgets('Image.file throws a non-implemented error on web', (WidgetTester tester) async {
    const String expectedError =
        'Image.file is not supported on Flutter Web. '
        'Consider using either Image.asset or Image.network instead.';
    final Uri uri = Uri.parse('/home/flutter/dash.png');
    final File file = File.fromUri(uri);
    expect(
      () => Image.file(file),
      kIsWeb
          // Web does not support file access, expect AssertionError
          ? throwsA(predicate((AssertionError e) => e.message == expectedError))
          // AOT supports file access, expect constructor to succeed
          : isNot(throwsA(anything)),
    );
  });

  testWidgets(
    'Animated GIFs do not require layout for subsequent frames',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // The test leaks by design, see [_TestImageStreamCompleter].
    (WidgetTester tester) async {
      final ui.Codec codec =
          (await tester.runAsync(() {
            return ui.instantiateImageCodec(Uint8List.fromList(kAnimatedGif));
          }))!;

      Future<ui.Image> nextFrame() async {
        final ui.FrameInfo frameInfo = (await tester.runAsync(codec.getNextFrame))!;
        return frameInfo.image;
      }

      final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
      final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);
      int? lastFrame;

      await tester.pumpWidget(
        Center(
          child: Image(
            image: imageProvider,
            frameBuilder: (
              BuildContext context,
              Widget child,
              int? frame,
              bool wasSynchronouslyLoaded,
            ) {
              lastFrame = frame;
              return child;
            },
          ),
        ),
      );

      expect(tester.getSize(find.byType(Image)), Size.zero);

      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 0);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsLayout, isFalse);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsPaint, isFalse);
      expect(tester.getSize(find.byType(Image)), const Size(1, 1));

      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      // We only complete the build phase and expect that it does not mark the
      // RenderImage for layout because the new frame has the same dimensions as
      // the old one. We only need to repaint.
      await tester.pump(null, EnginePhase.build);
      expect(lastFrame, 1);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsLayout, isFalse);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsPaint, isTrue);
      expect(tester.getSize(find.byType(Image)), const Size(1, 1));

      streamCompleter.setData(imageInfo: ImageInfo(image: await nextFrame()));
      await tester.pump();
      expect(lastFrame, 2);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsLayout, isFalse);
      expect(tester.allRenderObjects.whereType<RenderImage>().single.debugNeedsPaint, isFalse);
      expect(tester.getSize(find.byType(Image)), const Size(1, 1));

      codec.dispose();
    },
  );
}

@immutable
class _ConfigurationAwareKey {
  const _ConfigurationAwareKey(this.provider, this.configuration);

  final ImageProvider provider;
  final ImageConfiguration configuration;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _ConfigurationAwareKey &&
        other.provider == provider &&
        other.configuration == configuration;
  }

  @override
  int get hashCode => Object.hash(provider, configuration);
}

class _ConfigurationKeyedTestImageProvider extends _TestImageProvider {
  @override
  Future<_ConfigurationAwareKey> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<_ConfigurationAwareKey>(_ConfigurationAwareKey(this, configuration));
  }
}

class _TestImageProvider extends ImageProvider<Object> {
  _TestImageProvider({ImageStreamCompleter? streamCompleter}) {
    _streamCompleter = streamCompleter ?? OneFrameImageStreamCompleter(_completer.future);
  }

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();
  late ImageStreamCompleter _streamCompleter;
  late ImageConfiguration _lastResolvedConfiguration;

  bool get loadCalled => _loadCallCount > 0;
  int get loadCallCount => _loadCallCount;
  int _loadCallCount = 0;

  @override
  Future<Object> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<_TestImageProvider>(this);
  }

  @override
  void resolveStreamForKey(
    ImageConfiguration configuration,
    ImageStream stream,
    Object key,
    ImageErrorListener handleError,
  ) {
    _lastResolvedConfiguration = configuration;
    super.resolveStreamForKey(configuration, stream, key, handleError);
  }

  @override
  ImageStreamCompleter loadImage(Object key, ImageDecoderCallback decode) {
    _loadCallCount += 1;
    return _streamCompleter;
  }

  void complete(ui.Image image) {
    _completer.complete(ImageInfo(image: image));
  }

  void fail(Object exception, StackTrace? stackTrace) {
    _completer.completeError(exception, stackTrace);
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

/// An [ImageStreamCompleter] that gives access to the added listeners.
///
/// Such an access to listeners is hacky,
/// because it breaks encapsulation by allowing to invoke listeners without
/// taking care about lifecycle of the created images, that may result in not disposed images.
///
/// That's why some tests that use it
/// are opted out from leak tracking.
class _TestImageStreamCompleter extends ImageStreamCompleter {
  _TestImageStreamCompleter([this._currentImage]);

  ImageInfo? _currentImage;
  final Set<ImageStreamListener> listeners = <ImageStreamListener>{};

  @override
  void addListener(ImageStreamListener listener) {
    listeners.add(listener);
    if (_currentImage != null) {
      listener.onImage(_currentImage!.clone(), true);
    }
  }

  @override
  void removeListener(ImageStreamListener listener) {
    listeners.remove(listener);
  }

  void setData({ImageInfo? imageInfo, ImageChunkEvent? chunkEvent}) {
    if (imageInfo != null) {
      _currentImage?.dispose();
      _currentImage = imageInfo;
    }
    final List<ImageStreamListener> localListeners = listeners.toList();
    for (final ImageStreamListener listener in localListeners) {
      if (imageInfo != null) {
        listener.onImage(imageInfo.clone(), false);
      }
      if (chunkEvent != null && listener.onChunk != null) {
        listener.onChunk!(chunkEvent);
      }
    }
  }

  void setError({required Object exception, StackTrace? stackTrace}) {
    final List<ImageStreamListener> localListeners = listeners.toList();
    for (final ImageStreamListener listener in localListeners) {
      listener.onError?.call(exception, stackTrace);
    }
  }

  void dispose() {
    final List<ImageStreamListener> listenersCopy = listeners.toList();
    listenersCopy.forEach(removeListener);
  }
}

class _DebouncingImageProvider extends ImageProvider<Object> {
  _DebouncingImageProvider(this.imageProvider, this.seenKeys);

  /// A set of keys that will only get resolved the _first_ time they are seen.
  ///
  /// If an ImageProvider produces the same key for two different image
  /// configurations, it should only actually resolve once using this provider.
  /// However, if it does care about image configuration, it should make the
  /// property or properties it cares about part of the key material it
  /// produces.
  final Set<Object> seenKeys;
  final ImageProvider<Object> imageProvider;

  @override
  void resolveStreamForKey(
    ImageConfiguration configuration,
    ImageStream stream,
    Object key,
    ImageErrorListener handleError,
  ) {
    if (seenKeys.add(key)) {
      imageProvider.resolveStreamForKey(configuration, stream, key, handleError);
    }
  }

  @override
  Future<Object> obtainKey(ImageConfiguration configuration) =>
      imageProvider.obtainKey(configuration);

  @override
  ImageStreamCompleter loadImage(Object key, ImageDecoderCallback decode) =>
      imageProvider.loadImage(key, decode);
}

class _FailingImageProvider extends ImageProvider<int> {
  const _FailingImageProvider({
    this.failOnObtainKey = false,
    this.failOnLoad = false,
    required this.throws,
    required this.image,
  }) : assert(failOnLoad || failOnObtainKey);

  final bool failOnObtainKey;
  final bool failOnLoad;
  final Object throws;
  final ui.Image image;

  @override
  Future<int> obtainKey(ImageConfiguration configuration) {
    if (failOnObtainKey) {
      throw throws;
    }
    return SynchronousFuture<int>(hashCode);
  }

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    if (failOnLoad) {
      throw throws;
    }
    return OneFrameImageStreamCompleter(Future<ImageInfo>.value(ImageInfo(image: image, scale: 0)));
  }
}
