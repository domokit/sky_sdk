// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart' hide NetworkImage;
import 'package:flutter/src/painting/_network_image_web.dart';
import 'package:flutter/src/painting/_web_image_info_web.dart';
import 'package:flutter/src/web.dart' as web_shim;
import 'package:flutter/src/widgets/_web_image_web.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:web/web.dart' as web;

import '../image_data.dart';
import '_test_http_request.dart';

void runTests() {
  tearDown(() {
    debugRestoreHttpRequestFactory();
    debugRestoreImgElementFactory();
  });

  testWidgets('loads an image from the network with headers', (WidgetTester tester) async {
    final TestHttpRequest testHttpRequest =
        TestHttpRequest()
          ..status = 200
          ..mockEvent = MockEvent('load', web.Event('test error'))
          ..response = (Uint8List.fromList(kTransparentImage)).buffer;

    httpRequestFactory = () {
      return testHttpRequest.getMock() as web_shim.XMLHttpRequest;
    };

    const Map<String, String> headers = <String, String>{'flutter': 'flutter', 'second': 'second'};

    final Image image = Image.network('https://www.example.com/images/frame.png', headers: headers);

    await tester.pumpWidget(image);

    assert(mapEquals(testHttpRequest.responseHeaders, headers), true);
  });

  testWidgets('loads an image from the network with unsuccessful HTTP code', (
    WidgetTester tester,
  ) async {
    final TestHttpRequest testHttpRequest =
        TestHttpRequest()
          ..status = 404
          ..mockEvent = MockEvent('error', web.Event('test error'));

    httpRequestFactory = () {
      return testHttpRequest.getMock() as web_shim.XMLHttpRequest;
    };

    const Map<String, String> headers = <String, String>{'flutter': 'flutter', 'second': 'second'};

    final Image image = Image.network(
      'https://www.example.com/images/frame2.png',
      headers: headers,
    );

    await tester.pumpWidget(image);
    expect(
      tester.takeException(),
      isA<NetworkImageLoadException>().having(
        (NetworkImageLoadException e) => e.statusCode,
        'status code',
        404,
      ),
    );
  });

  testWidgets('loads an image from the network with empty response', (WidgetTester tester) async {
    final TestHttpRequest testHttpRequest =
        TestHttpRequest()
          ..status = 200
          ..mockEvent = MockEvent('load', web.Event('successful load'))
          ..response = (Uint8List.fromList(<int>[])).buffer;

    httpRequestFactory = () {
      return testHttpRequest.getMock() as web_shim.XMLHttpRequest;
    };

    const Map<String, String> headers = <String, String>{'flutter': 'flutter', 'second': 'second'};

    final Image image = Image.network(
      'https://www.example.com/images/frame3.png',
      headers: headers,
    );

    await tester.pumpWidget(image);
    expect(
      tester.takeException().toString(),
      'HTTP request failed, statusCode: 200, https://www.example.com/images/frame3.png',
    );
  });

  testWidgets('When strategy is default, emits an error if the image is cross-origin', (
    WidgetTester tester,
  ) async {
    final TestHttpRequest failingRequest =
        TestHttpRequest()
          ..status = 500
          ..mockEvent = MockEvent('load', web.Event('bytes inaccessible'))
          ..response = (Uint8List.fromList(<int>[])).buffer;

    httpRequestFactory = () {
      return failingRequest.getMock() as web_shim.XMLHttpRequest;
    };

    imgElementFactory = () {
      throw UnimplementedError();
    };

    const NetworkImage networkImage = NetworkImage('https://www.example.com/images/frame4.png');
    ImageInfo? imageInfo;
    Object? recordedError;
    Completer<void>? imageCompleter;
    await tester.runAsync(() async {
      imageCompleter = Completer<void>();
      final ImageStream stream = networkImage.resolve(ImageConfiguration.empty);
      stream.addListener(
        ImageStreamListener(
          (ImageInfo info, bool isSync) {
            imageInfo = info;
            imageCompleter!.complete();
          },
          onError: (Object error, StackTrace? stackTrace) {
            recordedError = error;
            imageCompleter!.complete();
          },
        ),
      );
    });
    await tester.runAsync(() async {
      await imageCompleter!.future;
    });
    expect(recordedError, isNotNull);
    expect(imageInfo, isNull);
  });

  testWidgets('When strategy is .fallback, emits a WebImageInfo if the image is cross-origin', (
    WidgetTester tester,
  ) async {
    final TestHttpRequest failingRequest =
        TestHttpRequest()
          ..status = 500
          ..mockEvent = MockEvent('load', web.Event('bytes inaccessible'))
          ..response = (Uint8List.fromList(<int>[])).buffer;
    final TestImgElement testImg = TestImgElement();

    httpRequestFactory = () {
      return failingRequest.getMock() as web_shim.XMLHttpRequest;
    };

    imgElementFactory = () {
      return testImg.getMock() as web_shim.HTMLImageElement;
    };

    const NetworkImage networkImage = NetworkImage(
      'https://www.example.com/images/frame5.png',
      webHtmlElementStrategy: WebHtmlElementStrategy.fallback,
    );
    ImageInfo? imageInfo;
    Object? recordedError;
    Completer<void>? imageCompleter;
    await tester.runAsync(() async {
      imageCompleter = Completer<void>();
      final ImageStream stream = networkImage.resolve(ImageConfiguration.empty);
      stream.addListener(
        ImageStreamListener(
          (ImageInfo info, bool isSync) {
            imageInfo = info;
            imageCompleter!.complete();
          },
          onError: (Object error, StackTrace? stackTrace) {
            recordedError = error;
            imageCompleter!.complete();
          },
        ),
      );
    });
    await tester.runAsync(() async {
      testImg.decodeSuccess();
      await imageCompleter!.future;
    });
    expect(recordedError, isNull);
    expect(imageInfo, isA<WebImageInfo>());

    final WebImageInfo webImageInfo = imageInfo! as WebImageInfo;
    expect(webImageInfo.htmlImage.src, equals('https://www.example.com/images/frame5.png'));
  });

  testWidgets(
    'When strategy is .fallback, emits an error if the image is cross-origin but fails to decode',
    (WidgetTester tester) async {
      final TestHttpRequest failingRequest =
          TestHttpRequest()
            ..status = 500
            ..mockEvent = MockEvent('load', web.Event('bytes inaccessible'))
            ..response = (Uint8List.fromList(<int>[])).buffer;
      final TestImgElement testImg = TestImgElement();

      httpRequestFactory = () {
        return failingRequest.getMock() as web_shim.XMLHttpRequest;
      };

      imgElementFactory = () {
        return testImg.getMock() as web_shim.HTMLImageElement;
      };

      const NetworkImage networkImage = NetworkImage(
        'https://www.example.com/images/frame6.png',
        webHtmlElementStrategy: WebHtmlElementStrategy.fallback,
      );
      ImageInfo? imageInfo;
      Object? recordedError;
      Completer<void>? imageCompleter;
      await tester.runAsync(() async {
        imageCompleter = Completer<void>();
        final ImageStream stream = networkImage.resolve(ImageConfiguration.empty);
        stream.addListener(
          ImageStreamListener(
            (ImageInfo info, bool isSync) {
              imageInfo = info;
              imageCompleter!.complete();
            },
            onError: (Object error, StackTrace? stackTrace) {
              recordedError = error;
              imageCompleter!.complete();
            },
          ),
        );
      });
      await tester.runAsync(() async {
        testImg.decodeFailure();
        await imageCompleter!.future;
      });
      expect(recordedError, isNotNull);
      expect(imageInfo, isNull);
    },
  );

  testWidgets('When strategy is .prefer, emits an WebImageInfo if the image is same-origin', (
    WidgetTester tester,
  ) async {
    final TestHttpRequest testHttpRequest =
        TestHttpRequest()
          ..status = 200
          ..mockEvent = MockEvent('load', web.Event('test error'))
          ..response = (Uint8List.fromList(kTransparentImage)).buffer;
    final TestImgElement testImg = TestImgElement();

    httpRequestFactory = () {
      return testHttpRequest.getMock() as web_shim.XMLHttpRequest;
    };

    imgElementFactory = () {
      return testImg.getMock() as web_shim.HTMLImageElement;
    };

    const NetworkImage networkImage = NetworkImage(
      'https://www.example.com/images/frame7.png',
      webHtmlElementStrategy: WebHtmlElementStrategy.prefer,
    );
    ImageInfo? imageInfo;
    Object? recordedError;
    Completer<void>? imageCompleter;
    await tester.runAsync(() async {
      imageCompleter = Completer<void>();
      final ImageStream stream = networkImage.resolve(ImageConfiguration.empty);
      stream.addListener(
        ImageStreamListener(
          (ImageInfo info, bool isSync) {
            imageInfo = info;
            imageCompleter!.complete();
          },
          onError: (Object error, StackTrace? stackTrace) {
            recordedError = error;
            imageCompleter!.complete();
          },
        ),
      );
    });
    await tester.runAsync(() async {
      testImg.decodeSuccess();
      await imageCompleter!.future;
    });
    expect(recordedError, isNull);
    expect(imageInfo, isA<WebImageInfo>());

    final WebImageInfo webImageInfo = imageInfo! as WebImageInfo;
    expect(webImageInfo.htmlImage.src, equals('https://www.example.com/images/frame7.png'));
  });

  testWidgets('When strategy is .prefer, emits a normal image if headers is not null', (
    WidgetTester tester,
  ) async {
    final TestHttpRequest testHttpRequest =
        TestHttpRequest()
          ..status = 200
          ..mockEvent = MockEvent('load', web.Event('test error'))
          ..response = (Uint8List.fromList(kTransparentImage)).buffer;
    final TestImgElement testImg = TestImgElement();

    httpRequestFactory = () {
      return testHttpRequest.getMock() as web_shim.XMLHttpRequest;
    };

    imgElementFactory = () {
      return testImg.getMock() as web_shim.HTMLImageElement;
    };

    const NetworkImage networkImage = NetworkImage(
      'https://www.example.com/images/frame8.png',
      webHtmlElementStrategy: WebHtmlElementStrategy.prefer,
      headers: <String, String>{'flutter': 'flutter', 'second': 'second'},
    );
    ImageInfo? imageInfo;
    Object? recordedError;
    Completer<void>? imageCompleter;
    await tester.runAsync(() async {
      imageCompleter = Completer<void>();
      final ImageStream stream = networkImage.resolve(ImageConfiguration.empty);
      stream.addListener(
        ImageStreamListener(
          (ImageInfo info, bool isSync) {
            imageInfo = info;
            imageCompleter!.complete();
          },
          onError: (Object error, StackTrace? stackTrace) {
            recordedError = error;
            imageCompleter!.complete();
          },
        ),
      );
    });
    await tester.runAsync(() async {
      testImg.decodeSuccess();
      await imageCompleter!.future;
    });
    expect(recordedError, isNull);
    expect(imageInfo, isNotNull);
    expect(imageInfo, isNot(isA<WebImageInfo>()));
  });

  testWidgets('Image renders an image using a Platform View if the image info is WebImageInfo', (
    WidgetTester tester,
  ) async {
    final TestImgElement testImg = TestImgElement();

    final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
    final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

    await tester.pumpWidget(Image(image: imageProvider));

    // Before getting a WebImageInfo, the Image resolves to a RawImage.
    expect(find.byType(RawImage), findsOneWidget);
    expect(find.byType(RawWebImage), findsNothing);
    expect(find.byType(PlatformViewLink), findsNothing);
    streamCompleter.setData(
      imageInfo: WebImageInfo(testImg.getMock() as web_shim.HTMLImageElement),
    );
    await tester.pump();
    expect(find.byType(RawImage), findsNothing);
    // After getting a WebImageInfo, the Image uses a Platform View to render.
    expect(find.byType(RawWebImage), findsOneWidget);
    expect(find.byType(PlatformViewLink), findsOneWidget);
  });
}

class _TestImageProvider extends ImageProvider<Object> {
  _TestImageProvider({required ImageStreamCompleter streamCompleter}) {
    _streamCompleter = streamCompleter;
  }

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();
  late ImageStreamCompleter _streamCompleter;

  @override
  Future<Object> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<_TestImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(Object key, ImageDecoderCallback decode) {
    return _streamCompleter;
  }

  void complete(web_shim.HTMLImageElement image) {
    _completer.complete(WebImageInfo(image));
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
/// That's why some tests that use it are opted out from leak tracking.
class _TestImageStreamCompleter extends ImageStreamCompleter {
  _TestImageStreamCompleter();

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
