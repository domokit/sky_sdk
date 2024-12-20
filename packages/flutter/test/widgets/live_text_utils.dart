// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

/// A mock class to control the return result of Live Text input functions.
class LiveTextInputTester {
  LiveTextInputTester() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      _handler,
    );
  }

  bool mockLiveTextInputEnabled = false;

  Future<Object?> _handler(MethodCall methodCall) async {
    // Need to set Clipboard.hasStrings method handler because when showing the tool bar,
    // the Clipboard.hasStrings will also be invoked. If this isn't handled,
    // an exception will be thrown.
    if (methodCall.method == 'Clipboard.hasStrings') {
      return <String, bool>{'value': true};
    }
    if (methodCall.method == 'LiveText.isLiveTextInputAvailable') {
      return mockLiveTextInputEnabled;
    }
    return false;
  }

  void dispose() {
    assert(
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
        SystemChannels.platform.name,
        _handler,
      ),
    );
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  }
}

/// A function to find the live text button.
///
/// LiveText button is displayed either using a custom painter,
/// a Text with an empty label, or a Text with the 'Scan text' label.
Finder findLiveTextButton() {
  final bool isMobile =
      defaultTargetPlatform == TargetPlatform.android ||
      defaultTargetPlatform == TargetPlatform.fuchsia ||
      defaultTargetPlatform == TargetPlatform.iOS;
  if (isMobile) {
    return find.byWidgetPredicate((Widget widget) {
      return (widget is CustomPaint &&
              '${widget.painter?.runtimeType}' == '_LiveTextIconPainter') ||
          (widget is Text &&
              widget.data == 'Scan text'); // Android and Fuchsia when inside a MaterialApp.
    });
  }
  if (defaultTargetPlatform == TargetPlatform.macOS) {
    return find.ancestor(
      of: find.text(''),
      matching: find.byType(CupertinoDesktopTextSelectionToolbarButton),
    );
  }
  return find.byWidgetPredicate((Widget widget) {
    return widget is Text && (widget.data == '' || widget.data == 'Scan text');
  });
}
