// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ATTENTION!
//
// This file is not named "*_test.dart", and as such will not run when you run
// "flutter test". It is only intended to be run as part of the
// flutter_gallery_instrumentation_test devicelab test.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/gestures.dart' show PointerDeviceKind, kPrimaryButton;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_gallery/gallery/demos.dart';
import 'package:flutter_test/flutter_test.dart';

// Reports success or failure to the native code.
const MethodChannel _kTestChannel = MethodChannel('io.flutter.demo.gallery/TestLifecycleListener');

// We don't want to wait for animations to complete before tapping the
// back button in the demos with these titles.
const List<String> _kUnsynchronizedDemoTitles = <String>[
  'Progress indicators',
  'Activity Indicator',
  'Video',
];

// These demos can't be backed out of by tapping a button whose
// tooltip is 'Back'.
const List<String> _kSkippedDemoTitles = <String>[
  'Progress indicators',
  'Activity Indicator',
  'Video',
];

// There are 3 places where the Gallery demos are traversed.
// 1- In widget tests such as dev/integration_tests/flutter_gallery/test/smoke_test.dart
// 2- In driver tests such as dev/integration_tests/flutter_gallery/test_driver/transitions_perf_test.dart
// 3- In on-device instrumentation tests such as dev/integration_tests/flutter_gallery/test/live_smoketest.dart
//
// If you change navigation behavior in the Gallery or in the framework, make
// sure all 3 are covered.

Future<void> main() async {
  try {
    // Verify that _kUnsynchronizedDemos and _kSkippedDemos identify
    // demos that actually exist.
    final List<String> allDemoTitles =
        kAllGalleryDemos.map((GalleryDemo demo) => demo.title).toList();
    if (!Set<String>.from(allDemoTitles).containsAll(_kUnsynchronizedDemoTitles)) {
      fail('Unrecognized demo titles in _kUnsynchronizedDemosTitles: $_kUnsynchronizedDemoTitles');
    }
    if (!Set<String>.from(allDemoTitles).containsAll(_kSkippedDemoTitles)) {
      fail('Unrecognized demo names in _kSkippedDemoTitles: $_kSkippedDemoTitles');
    }

    print('Starting app...');
    runApp(const GalleryApp(testMode: true));
    final _LiveWidgetController controller = _LiveWidgetController(WidgetsBinding.instance);
    for (final GalleryDemoCategory category in kAllGalleryDemoCategories) {
      print('Tapping "${category.name}" section...');
      await controller.tap(find.text(category.name));
      for (final GalleryDemo demo in kGalleryCategoryToDemos[category]!) {
        final Finder demoItem = find.text(demo.title);
        print('Scrolling to "${demo.title}"...');
        await controller.scrollIntoView(demoItem, alignment: 0.5);
        if (_kSkippedDemoTitles.contains(demo.title)) {
          continue;
        }
        for (int i = 0; i < 2; i += 1) {
          print('Tapping "${demo.title}"...');
          await controller.tap(demoItem); // Launch the demo
          controller.frameSync = !_kUnsynchronizedDemoTitles.contains(demo.title);
          print('Going back to demo list...');
          await controller.tap(backFinder);
          controller.frameSync = true;
        }
      }
      print('Going back to home screen...');
      await controller.tap(find.byTooltip('Back'));
    }
    print('Finished successfully!');
    _kTestChannel.invokeMethod<void>('success');
  } catch (error, stack) {
    print('Caught error: $error\n$stack');
    _kTestChannel.invokeMethod<void>('failure');
  }
}

final Finder backFinder = find.byElementPredicate(
  (Element element) => switch (element.widget) {
    Tooltip(message: 'Back') => true,
    CupertinoNavigationBarBackButton() => true,
    _ => false,
  },
  description: 'Material or Cupertino back button',
);

class _LiveWidgetController extends LiveWidgetController {
  _LiveWidgetController(super.binding);

  /// With [frameSync] enabled, Flutter Driver will wait to perform an action
  /// until there are no pending frames in the app under test.
  bool frameSync = true;

  /// Waits until at the end of a frame the provided [condition] is [true].
  Future<void> _waitUntilFrame(bool Function() condition, [Completer<void>? completer]) {
    completer ??= Completer<void>();
    if (!condition()) {
      SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
        _waitUntilFrame(condition, completer);
      });
    } else {
      completer.complete();
    }
    return completer.future;
  }

  /// Runs `finder` repeatedly until it finds one or more [Element]s.
  Future<FinderBase<Element>> _waitForElement(FinderBase<Element> finder) async {
    if (frameSync) {
      await _waitUntilFrame(() => binding.transientCallbackCount == 0);
    }
    await _waitUntilFrame(() => finder.tryEvaluate());
    if (frameSync) {
      await _waitUntilFrame(() => binding.transientCallbackCount == 0);
    }
    return finder;
  }

  Future<void> scrollIntoView(FinderBase<Element> finder, {required double alignment}) async {
    final FinderBase<Element> target = await _waitForElement(finder);
    await Scrollable.ensureVisible(
      target.evaluate().single,
      duration: const Duration(milliseconds: 100),
      alignment: alignment,
    );
  }

  @override
  Future<void> tap(
    FinderBase<Element> finder, {
    int? pointer,
    int buttons = kPrimaryButton,
    bool warnIfMissed = true,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) async {
    await super.tap(
      await _waitForElement(finder),
      pointer: pointer,
      buttons: buttons,
      warnIfMissed: warnIfMissed,
      kind: kind,
    );
  }
}
