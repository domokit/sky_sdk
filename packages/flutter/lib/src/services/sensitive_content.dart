// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'package:flutter/foundation.dart';

import 'system_channels.dart';

/// The level of sensitivity that content in a particular Flutter view
/// contains.
/// 
/// * See [SensitiveContent] for how to set a [ContentSensitivity] level
///   in order for sensitive content to be obscured when the Flutter screen
///   is shared. 
enum ContentSensitivity {
  /// Content sensitivity is auto-detected by the native framework.
  /// 
  /// When this level is set via a [SensitiveContent] widget, the window
  /// hosting the screen will only be marked as secure if other [SensitiveContent]
  /// widgets with the [sensitive] level are present in the widget tree.
  /// 
  /// For Android, the sensitive content is unable to be auto-detected by the
  /// native framework.
  // TODO(camsim99): Implement `autoSensitive` mode that matches the behavior
  // of `CONTENT_SENSITIVITY_AUTO` on Android that has implemented based on autofill hints.
  // See https://developer.android.com/reference/android/view/View#CONTENT_SENSITIVITY_AUTO.
  autoSensitive,
  
  /// The view displays sensitive content.
  /// 
  /// When this level is set via a [SensitiveContent] widget, the window
  /// hosting the screen will be marked as secure during an active media
  /// projection session.
  /// 
  /// For Android, see https://developer.android.com/reference/android/view/View#CONTENT_SENSITIVITY_NOT_SENSITIVE.
  sensitive,

  /// The view does not display sensitive content.
  /// 
  /// When this level is set via a [SensitiveContent] widget, the window
  /// hosting the screen will only be marked as secure if other [SensitiveContent]
  /// widgets with the [sensitive] level are present in the widget tree.
  /// 
  /// For Android, see https://developer.android.com/reference/android/view/View#CONTENT_SENSITIVITY_NOT_SENSITIVE.
  notSensitive,
}

/// Service for setting the content sensitivity of native Flutter views.
class SensitiveContentService {
  /// Creates service to set content sensitivity of Flutter views via
  /// communication over the sensitive content [MethodChannel].
  SensitiveContentService() {
    sensitiveContentChannel = SystemChannels.sensitiveContent;
  }

  /// The channel used to communicate with the shell side to set the
  /// content sensitivity of Flutter views.
  late MethodChannel sensitiveContentChannel;

   /// Sets content sensitivity level of the native backing to a Flutter view
   /// with the specified [flutterViewId] to the level specified by
   /// [contentSensitivity] via a call to the native embedder.
  void setContentSensitivity(int flutterViewId, ContentSensitivity contentSensitivity) {
    try {
      sensitiveContentChannel.invokeMethod(
        'SensitiveContent.setContentSensitivity',
        <int>[flutterViewId, contentSensitivity.index],
      );
    } catch (e) {
      // Content sensitivity failed to be set.
      throw FlutterError(
        'Content sensitivity failed to be set. Please ensure that the '
        'flutterViewId set corresponds to a valid Android Flutter view '
        'or Fragment.'
      );
    }
  }
}
