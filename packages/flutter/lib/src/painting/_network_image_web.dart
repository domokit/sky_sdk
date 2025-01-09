// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:ui' as ui;
import 'dart:ui_web' as ui_web;

import 'package:flutter/foundation.dart';

import '../web.dart' as web;
import '_web_image_info_web.dart';
import 'image_provider.dart' as image_provider;
import 'image_stream.dart';

/// The type for an overridable factory function for creating an HTTP request,
/// used for testing purposes.
typedef HttpRequestFactory = web.XMLHttpRequest Function();

/// The type for an overridable factory function for creating <img> elements,
/// used for testing purposes.
typedef ImgElementFactory = web.HTMLImageElement Function();

// Method signature for _loadAsync decode callbacks.
typedef _SimpleDecoderCallback = Future<ui.Codec> Function(ui.ImmutableBuffer buffer);

/// The default HTTP client.
web.XMLHttpRequest _httpClient() {
  return web.XMLHttpRequest();
}

/// Creates an overridable factory function.
@visibleForTesting
HttpRequestFactory httpRequestFactory = _httpClient;

/// Restores the default HTTP request factory.
@visibleForTesting
void debugRestoreHttpRequestFactory() {
  httpRequestFactory = _httpClient;
}

/// The default <img> element factory.
web.HTMLImageElement _imgElementFactory() {
  return web.document.createElement('img') as web.HTMLImageElement;
}

/// The factory function that creates <img> elements, can be overridden for
/// tests.
@visibleForTesting
ImgElementFactory imgElementFactory = _imgElementFactory;

/// Restores the default <img> element factory.
@visibleForTesting
void debugRestoreImgElementFactory() {
  imgElementFactory = _imgElementFactory;
}

/// The web implementation of [image_provider.NetworkImage].
///
/// NetworkImage on the web does not support decoding to a specified size.
@immutable
class NetworkImage extends image_provider.ImageProvider<image_provider.NetworkImage>
    implements image_provider.NetworkImage {
  /// Creates an object that fetches the image at the given URL.
  const NetworkImage(this.url, {this.scale = 1.0, this.headers});

  @override
  final String url;

  @override
  final double scale;

  @override
  final Map<String, String>? headers;

  @override
  Future<NetworkImage> obtainKey(image_provider.ImageConfiguration configuration) {
    return SynchronousFuture<NetworkImage>(this);
  }

  @override
  ImageStreamCompleter loadBuffer(
    image_provider.NetworkImage key,
    image_provider.DecoderBufferCallback decode,
  ) {
    // Ownership of this controller is handed off to [_loadAsync]; it is that
    // method's responsibility to close the controller's stream when the image
    // has been loaded or an error is thrown.
    final StreamController<ImageChunkEvent> chunkEvents = StreamController<ImageChunkEvent>();

    return _ForwardingImageStreamCompleter(
      _loadAsync(key as NetworkImage, decode, chunkEvents),
      informationCollector: _imageStreamInformationCollector(key),
      debugLabel: key.url,
    );
  }

  @override
  ImageStreamCompleter loadImage(
    image_provider.NetworkImage key,
    image_provider.ImageDecoderCallback decode,
  ) {
    // Ownership of this controller is handed off to [_loadAsync]; it is that
    // method's responsibility to close the controller's stream when the image
    // has been loaded or an error is thrown.
    final StreamController<ImageChunkEvent> chunkEvents = StreamController<ImageChunkEvent>();

    return _ForwardingImageStreamCompleter(
      _loadAsync(key as NetworkImage, decode, chunkEvents),
      informationCollector: _imageStreamInformationCollector(key),
      debugLabel: key.url,
    );
  }

  InformationCollector? _imageStreamInformationCollector(image_provider.NetworkImage key) {
    InformationCollector? collector;
    assert(() {
      collector =
          () => <DiagnosticsNode>[
            DiagnosticsProperty<image_provider.ImageProvider>('Image provider', this),
            DiagnosticsProperty<NetworkImage>('Image key', key as NetworkImage),
          ];
      return true;
    }());
    return collector;
  }

  // HTML renderer does not support decoding network images to a specified size. The decode parameter
  // here is ignored and `ui_web.createImageCodecFromUrl` will be used directly
  // in place of the typical `instantiateImageCodec` method.
  Future<ImageStreamCompleter> _loadAsync(
    NetworkImage key,
    _SimpleDecoderCallback decode,
    StreamController<ImageChunkEvent> chunkEvents,
  ) async {
    assert(key == this);

    final Uri resolved = Uri.base.resolve(key.url);

    final bool containsNetworkImageHeaders = key.headers?.isNotEmpty ?? false;

    // We use a different method when headers are set because the
    // `ui_web.createImageCodecFromUrl` method is not capable of handling headers.
    if (containsNetworkImageHeaders) {
      // It is not possible to load an <img> element and pass the headers with
      // the request to fetch the image. Since the user has provided headers,
      // this function should assume the headers are required to resolve to
      // the correct resource and should not attempt to load the image in an
      // <img> tag without the headers.

      // Resolve the Codec before passing it to
      // [MultiFrameImageStreamCompleter] so any errors aren't reported
      // twice (once from the MultiFrameImageStreamCompleter and again
      // from the wrapping [ForwardingImageStreamCompleter]).
      final ui.Codec codec = await _fetchImageBytes(decode);
      return MultiFrameImageStreamCompleter(
        chunkEvents: chunkEvents.stream,
        codec: Future<ui.Codec>.value(codec),
        scale: key.scale,
        debugLabel: key.url,
        informationCollector: _imageStreamInformationCollector(key),
      );
    } else if (isSkiaWeb) {
      try {
        // Resolve the Codec before passing it to
        // [MultiFrameImageStreamCompleter] so any errors aren't reported
        // twice (once from the MultiFrameImageStreamCompleter and again
        // from the wrapping [ForwardingImageStreamCompleter]).
        final ui.Codec codec = await _fetchImageBytes(decode);
        return MultiFrameImageStreamCompleter(
          chunkEvents: chunkEvents.stream,
          codec: Future<ui.Codec>.value(codec),
          scale: key.scale,
          debugLabel: key.url,
          informationCollector: _imageStreamInformationCollector(key),
        );
      } catch (e) {
        // If we failed to fetch the bytes, try to load the image in an <img>
        // element instead.
        final web.HTMLImageElement imageElement = imgElementFactory();
        imageElement.src = key.url;
        // Decode the <img> element before creating the ImageStreamCompleter
        // to avoid double reporting the error.
        await imageElement.decode().toDart;
        return OneFrameImageStreamCompleter(
          Future<ImageInfo>.value(WebImageInfo(imageElement, debugLabel: key.url)),
          informationCollector: _imageStreamInformationCollector(key),
        )..debugLabel = key.url;
      }
    } else {
      // This branch is only hit by the HTML renderer, which is deprecated. The
      // HTML renderer supports loading images with CORS restrictions, so we
      // don't need to catch errors and try loading the image in an <img> tag
      // in this case.

      // Resolve the Codec before passing it to
      // [MultiFrameImageStreamCompleter] so any errors aren't reported
      // twice (once from the MultiFrameImageStreamCompleter) and again
      // from the wrapping [ForwardingImageStreamCompleter].
      final ui.Codec codec = await ui_web.createImageCodecFromUrl(
        resolved,
        chunkCallback: (int bytes, int total) {
          chunkEvents.add(ImageChunkEvent(cumulativeBytesLoaded: bytes, expectedTotalBytes: total));
        },
      );
      return MultiFrameImageStreamCompleter(
        chunkEvents: chunkEvents.stream,
        codec: Future<ui.Codec>.value(codec),
        scale: key.scale,
        debugLabel: key.url,
        informationCollector: _imageStreamInformationCollector(key),
      );
    }
  }

  Future<ui.Codec> _fetchImageBytes(_SimpleDecoderCallback decode) async {
    final Uri resolved = Uri.base.resolve(url);

    final bool containsNetworkImageHeaders = headers?.isNotEmpty ?? false;

    final Completer<web.XMLHttpRequest> completer = Completer<web.XMLHttpRequest>();
    final web.XMLHttpRequest request = httpRequestFactory();

    request.open('GET', url, true);
    request.responseType = 'arraybuffer';
    if (containsNetworkImageHeaders) {
      headers!.forEach((String header, String value) {
        request.setRequestHeader(header, value);
      });
    }

    request.addEventListener(
      'load',
      (web.Event e) {
        final int status = request.status;
        final bool accepted = status >= 200 && status < 300;
        final bool fileUri = status == 0; // file:// URIs have status of 0.
        final bool notModified = status == 304;
        final bool unknownRedirect = status > 307 && status < 400;
        final bool success = accepted || fileUri || notModified || unknownRedirect;

        if (success) {
          completer.complete(request);
        } else {
          completer.completeError(
            image_provider.NetworkImageLoadException(statusCode: status, uri: resolved),
          );
        }
      }.toJS,
    );

    request.addEventListener(
      'error',
      ((JSObject e) => completer.completeError(
            image_provider.NetworkImageLoadException(statusCode: request.status, uri: resolved),
          ))
          .toJS,
    );

    request.send();

    await completer.future;

    final Uint8List bytes = (request.response! as JSArrayBuffer).toDart.asUint8List();

    if (bytes.lengthInBytes == 0) {
      throw image_provider.NetworkImageLoadException(statusCode: request.status, uri: resolved);
    }

    return decode(await ui.ImmutableBuffer.fromUint8List(bytes));
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is NetworkImage && other.url == url && other.scale == scale;
  }

  @override
  int get hashCode => Object.hash(url, scale);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'NetworkImage')}("$url", scale: ${scale.toStringAsFixed(1)})';
}

/// An [ImageStreamCompleter] that delegates to another [ImageStreamCompleter]
/// that is loaded asynchronously.
///
/// This completer keeps its child completer alive until this completer is disposed.
class _ForwardingImageStreamCompleter extends ImageStreamCompleter {
  _ForwardingImageStreamCompleter(
    this.task, {
    InformationCollector? informationCollector,
    String? debugLabel,
  }) {
    this.debugLabel = debugLabel;
    task.then(
      (ImageStreamCompleter value) {
        resolved = true;
        if (_disposed) {
          // Add a listener since the delegate completer won't dispose if it never
          // had a listener.
          value.addListener(ImageStreamListener((_, __) {}));
          value.maybeDispose();
          return;
        }
        completer = value;
        handle = completer.keepAlive();
        completer.addListener(
          ImageStreamListener(
            (ImageInfo image, bool synchronousCall) {
              setImage(image);
            },
            onChunk: (ImageChunkEvent event) {
              reportImageChunkEvent(event);
            },
            onError: (Object exception, StackTrace? stackTrace) {
              reportError(exception: exception, stack: stackTrace);
            },
          ),
        );
      },
      onError: (Object error, StackTrace stack) {
        reportError(
          context: ErrorDescription('resolving an image stream completer'),
          exception: error,
          stack: stack,
          informationCollector: informationCollector,
          silent: true,
        );
      },
    );
  }

  final Future<ImageStreamCompleter> task;
  bool resolved = false;
  late final ImageStreamCompleter completer;
  late final ImageStreamCompleterHandle handle;

  bool _disposed = false;

  @override
  void onDisposed() {
    if (resolved) {
      handle.dispose();
    }
    _disposed = true;
    super.onDisposed();
  }
}
