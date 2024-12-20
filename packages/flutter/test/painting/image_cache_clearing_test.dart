// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../rendering/rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test("Clearing images while they're pending does not crash", () async {
    final Uint8List bytes = Uint8List.fromList(kTransparentImage);
    final MemoryImage memoryImage = MemoryImage(bytes);
    final ImageStream stream = memoryImage.resolve(ImageConfiguration.empty);
    final Completer<void> completer = Completer<void>();
    FlutterError.onError = (FlutterErrorDetails error) {
      completer.completeError(error.exception, error.stack);
    };
    stream.addListener(
      ImageStreamListener((ImageInfo image, bool synchronousCall) {
        completer.complete();
      }),
    );
    imageCache.clearLiveImages();
    await completer.future;
  });
}
