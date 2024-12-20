// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

/// A screenshot from the Android emulator.
class Screenshot {
  Screenshot(this.filename, this.fileContent, this.pixelCount);

  /// The name of the screenshot.
  final String filename;

  /// The binary content of the screenshot.
  final Uint8List fileContent;

  /// The number of pixels in the screenshot.
  final int pixelCount;
}

/// Takes the input stream and transforms it into [Screenshot]s.
class ScreenshotBlobTransformer extends StreamTransformerBase<Uint8List, Screenshot> {
  const ScreenshotBlobTransformer();

  @override
  Stream<Screenshot> bind(Stream<Uint8List> stream) async* {
    final BytesBuilder pending = BytesBuilder();

    await for (final Uint8List blob in stream) {
      pending.add(blob);

      if (pending.length < 12) {
        continue;
      }

      // See ScreenshotUtil#writeFile in ScreenshotUtil.java for producer side.
      final Uint8List bytes = pending.toBytes();
      final ByteData byteData = bytes.buffer.asByteData();

      int off = 0;
      final int fnameLen = byteData.getInt32(off);
      off += 4;
      final int fcontentLen = byteData.getInt32(off);
      off += 4;
      final int pixelCount = byteData.getInt32(off);
      off += 4;

      assert(fnameLen > 0);
      assert(fcontentLen > 0);
      assert(pixelCount > 0);

      if (pending.length < off + fnameLen) {
        continue;
      }

      final String filename = utf8.decode(bytes.buffer.asUint8List(off, fnameLen));
      off += fnameLen;
      if (pending.length < off + fcontentLen) {
        continue;
      }

      final Uint8List fileContent = bytes.buffer.asUint8List(off, fcontentLen);
      off += fcontentLen;
      pending.clear();
      pending.add(bytes.buffer.asUint8List(off));

      yield Screenshot('$filename.png', fileContent, pixelCount);
    }
  }
}
