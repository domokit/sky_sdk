// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show JsonEncoder;

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';
import 'package:flutter_gallery/demo_lists.dart';
import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_gallery/gallery/demos.dart';
import 'package:flutter_test/flutter_test.dart';

import 'run_demos.dart';

// All of the gallery demos, identified as "title@category".
//
// These names are reported by the test app, see _handleMessages()
// in transitions_perf.dart.
List<String> _allDemos =
    kAllGalleryDemos.map((GalleryDemo demo) => '${demo.title}@${demo.category.name}').toList();

Set<String> _unTestedDemos = Set<String>.from(_allDemos);

class _MessageHandler {
  static LiveWidgetController? controller;
  Future<String> call(String message) async {
    switch (message) {
      case 'demoNames':
        return const JsonEncoder.withIndent('  ').convert(_allDemos);
      case 'profileDemos':
        controller ??= LiveWidgetController(WidgetsBinding.instance);
        await runDemos(kProfiledDemos, controller!);
        _unTestedDemos.removeAll(kProfiledDemos);
        return const JsonEncoder.withIndent('  ').convert(kProfiledDemos);
      case 'restDemos':
        controller ??= LiveWidgetController(WidgetsBinding.instance);
        final List<String> restDemos = _unTestedDemos.toList();
        await runDemos(restDemos, controller!);
        return const JsonEncoder.withIndent('  ').convert(restDemos);
      default:
        throw ArgumentError;
    }
  }
}

void main() {
  enableFlutterDriverExtension(handler: (String? message) => _MessageHandler().call(message!));
  // As in lib/main.dart: overriding https://github.com/flutter/flutter/issues/13736
  // for better visual effect at the cost of performance.
  runApp(const GalleryApp(testMode: true));
}
