// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui show Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

class TestImageInfo extends ImageInfo {
  TestImageInfo(this.value, {required super.image, super.scale, super.debugLabel});

  final int value;

  @override
  String toString() => '${objectRuntimeType(this, 'TestImageInfo')}($value)';

  @override
  TestImageInfo clone() {
    return TestImageInfo(value, image: image.clone(), scale: scale, debugLabel: debugLabel);
  }

  @override
  int get hashCode => Object.hash(value, image, scale, debugLabel);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TestImageInfo &&
        other.value == value &&
        other.image.isCloneOf(image) &&
        other.scale == scale &&
        other.debugLabel == debugLabel;
  }
}

class TestImageProvider extends ImageProvider<int> {
  const TestImageProvider(this.key, this.imageValue, {required this.image});

  final int key;
  final int imageValue;
  final ui.Image image;

  @override
  Future<int> obtainKey(ImageConfiguration configuration) {
    return Future<int>.value(key);
  }

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(
      SynchronousFuture<ImageInfo>(TestImageInfo(imageValue, image: image.clone())),
    );
  }

  @override
  String toString() => '${objectRuntimeType(this, 'TestImageProvider')}($key, $imageValue)';
}

class FailingTestImageProvider extends TestImageProvider {
  const FailingTestImageProvider(super.key, super.imageValue, {required super.image});

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(
      Future<ImageInfo>.sync(() => Future<ImageInfo>.error('loading failed!')),
    );
  }
}

Future<ImageInfo> extractOneFrame(ImageStream stream) {
  final Completer<ImageInfo> completer = Completer<ImageInfo>();
  late ImageStreamListener listener;
  listener = ImageStreamListener((ImageInfo image, bool synchronousCall) {
    completer.complete(image);
    stream.removeListener(listener);
  });
  stream.addListener(listener);
  return completer.future;
}

class ErrorImageProvider extends ImageProvider<ErrorImageProvider> {
  @override
  ImageStreamCompleter loadImage(ErrorImageProvider key, ImageDecoderCallback decode) {
    throw Error();
  }

  @override
  ImageStreamCompleter loadBuffer(ErrorImageProvider key, DecoderBufferCallback decode) {
    throw Error();
  }

  @override
  Future<ErrorImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<ErrorImageProvider>(this);
  }
}

class ObtainKeyErrorImageProvider extends ImageProvider<ObtainKeyErrorImageProvider> {
  @override
  ImageStreamCompleter loadImage(ObtainKeyErrorImageProvider key, ImageDecoderCallback decode) {
    throw Error();
  }

  @override
  ImageStreamCompleter loadBuffer(ObtainKeyErrorImageProvider key, DecoderBufferCallback decode) {
    throw UnimplementedError();
  }

  @override
  Future<ObtainKeyErrorImageProvider> obtainKey(ImageConfiguration configuration) {
    throw Error();
  }
}

class LoadErrorImageProvider extends ImageProvider<LoadErrorImageProvider> {
  @override
  ImageStreamCompleter loadImage(LoadErrorImageProvider key, ImageDecoderCallback decode) {
    throw Error();
  }

  @override
  ImageStreamCompleter loadBuffer(LoadErrorImageProvider key, DecoderBufferCallback decode) {
    throw UnimplementedError();
  }

  @override
  Future<LoadErrorImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<LoadErrorImageProvider>(this);
  }
}

class LoadErrorCompleterImageProvider extends ImageProvider<LoadErrorCompleterImageProvider> {
  @override
  ImageStreamCompleter loadImage(LoadErrorCompleterImageProvider key, ImageDecoderCallback decode) {
    final Completer<ImageInfo> completer = Completer<ImageInfo>.sync();
    completer.completeError(Error());
    return OneFrameImageStreamCompleter(completer.future);
  }

  @override
  Future<LoadErrorCompleterImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<LoadErrorCompleterImageProvider>(this);
  }
}

class TestImageStreamCompleter extends ImageStreamCompleter {
  void testSetImage(ui.Image image) {
    setImage(ImageInfo(image: image));
  }
}
