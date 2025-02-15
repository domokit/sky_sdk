// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'matchers.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:matcher/expect.dart';
import 'package:matcher/src/expect/async_matcher.dart'; // ignore: implementation_imports
import 'package:test_api/hooks.dart' show TestFailure;

import 'binding.dart';
import 'finders.dart';
import 'goldens.dart';

/// An unsupported method that exists for API compatibility.
Future<ui.Image> captureImage(Element element) {
  throw UnsupportedError('captureImage is not supported on the web.');
}

/// Whether or not [captureImage] is supported.
///
/// This can be used to skip tests on platforms that don't support
/// capturing images.
///
/// Currently this is true except when tests are running in the context of a web
/// browser (`flutter test --platform chrome`).
const bool canCaptureImage = false;

/// The matcher created by [matchesGoldenFile]. This class is enabled when the
/// test is running in a web browser using conditional import.
class MatchesGoldenFile extends AsyncMatcher {
  /// Creates an instance of [MatchesGoldenFile]. Called by [matchesGoldenFile].
  const MatchesGoldenFile(this.key, this.version);

  /// Creates an instance of [MatchesGoldenFile]. Called by [matchesGoldenFile].
  MatchesGoldenFile.forStringPath(String path, this.version) : key = Uri.parse(path);

  /// The [key] to the golden image.
  final Uri key;

  /// The [version] of the golden image.
  final int? version;

  @override
  Future<String?> matchAsync(dynamic item) async {
    if (item is! Finder) {
      return 'web goldens only supports matching finders.';
    }
    final Iterable<Element> elements = item.evaluate();
    if (elements.isEmpty) {
      return 'could not be rendered because no widget was found';
    } else if (elements.length > 1) {
      return 'matched too many widgets';
    }
    final Element element = elements.single;

    // In CanvasKit and Skwasm, use Layer.toImage to generate the screenshot.
    final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.instance;
    return binding.runAsync<String?>(() async {
      assert(element.renderObject != null);
      RenderObject renderObject = element.renderObject!;
      while (!renderObject.isRepaintBoundary) {
        renderObject = renderObject.parent!;
      }
      assert(!renderObject.debugNeedsPaint);
      final OffsetLayer layer = renderObject.debugLayer! as OffsetLayer;
      final ui.Image image = await layer.toImage(renderObject.paintBounds);
      try {
        final ByteData? bytes = await image.toByteData(format: ui.ImageByteFormat.png);
        if (bytes == null) {
          return 'could not encode screenshot.';
        }
        if (autoUpdateGoldenFiles) {
          await goldenFileComparator.update(key, bytes.buffer.asUint8List());
          return null;
        }
        try {
          final bool success = await goldenFileComparator.compare(bytes.buffer.asUint8List(), key);
          return success ? null : 'does not match';
        } on TestFailure catch (ex) {
          return ex.message;
        }
      } finally {
        image.dispose();
      }
    });
  }

  @override
  Description describe(Description description) {
    final Uri testNameUri = goldenFileComparator.getTestUri(key, version);
    return description.add('one widget whose rasterized image matches golden image "$testNameUri"');
  }
}
