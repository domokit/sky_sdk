// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart' show ContentSensitivity, SensitiveContentService;
import 'package:flutter/widgets.dart' show AsyncSnapshot, FutureBuilder;

import 'framework.dart';

/// Data structure used to track the [SensitiveContent] widgets in the
/// widget tree.
class ContentSensitivityState {
  /// Creates a [ContentSensitivityState].
  ContentSensitivityState(this.currentContentSensitivitySetting);

  /// The current [ContentSensitivity] level set for the entire widget tree.
  ContentSensitivity currentContentSensitivitySetting;

  /// The number of [SensitiveContent] widgets that have sensitivity level [ContentSensitivity.sensitive].
  int sensitiveWidgetCount = 0;

  /// The number of [SensitiveContent] widgets that have sensitivity level [ContentSensitivity.autoSensitive].
  int autoSensitiveWidgetCount = 0;

  /// The number of [SensitiveContent] widgets that have sensitivity level [ContentSensitivity.notSensitive].
  int notSensitiveWigetCount = 0;

  /// Increases the count of [SensitiveContent] widgets with [sensitivityLevel] set.
  void addWidgetWithContentSensitivity(ContentSensitivity sensitivityLevel) {
    switch (sensitivityLevel) {
      case ContentSensitivity.sensitive:
        sensitiveWidgetCount++;
      case ContentSensitivity.autoSensitive:
        autoSensitiveWidgetCount++;
      case ContentSensitivity.notSensitive:
        notSensitiveWigetCount++;
    }
  }

  /// Decreases the count of [SensitiveContent] widgets with [sensitivityLevel] set.
  void removeWidgetWithContentSensitivity(ContentSensitivity sensitivityLevel) {
    switch (sensitivityLevel) {
      case ContentSensitivity.sensitive:
        sensitiveWidgetCount--;
      case ContentSensitivity.autoSensitive:
        autoSensitiveWidgetCount--;
      case ContentSensitivity.notSensitive:
        notSensitiveWigetCount--;
    }
  }

  /// Returns the number of [SensitiveContent] widgets represented by this state.
  int getTotalNumberOfWidgets() {
    return sensitiveWidgetCount + autoSensitiveWidgetCount + notSensitiveWigetCount;
  }
}

/// Host of the current content sensitivity level for the widget tree that contains
/// some number [SensitiveContent] widgets.
class SensitiveContentSetting {
  SensitiveContentSetting._();

  bool? _contentSenstivityIsSupported;
  ContentSensitivityState? _contentSensitivityState;
  ContentSensitivity? _defaultContentSensitivitySetting;

  final SensitiveContentService _sensitiveContentService = SensitiveContentService();

  @visibleForTesting
  /// [SensitiveContentSetting] instance for the widget tree.
  static final SensitiveContentSetting instance = SensitiveContentSetting._();

  @visibleForTesting
  /// The state of content sensitivity in the widget tree.
  ///
  /// Contains the number of widgets with each [ContentSensitivity] level and
  /// the current [ContentSensitivity] setting.
  ContentSensitivityState? getContentSenstivityState() {
    return _contentSensitivityState;
  }

  /// Registers a [SensitiveContent] widget that will help determine the
  /// [ContentSensitivity] level for the widget tree.
  static Future<void> register(ContentSensitivity desiredSensitivityLevel) async {
    await instance._register(desiredSensitivityLevel);
  }

  Future<void> _register(ContentSensitivity desiredSensitivityLevel) async {
    _contentSenstivityIsSupported ??= await _sensitiveContentService.isSupported();
    if (!_contentSenstivityIsSupported!) {
      // Setting content sensitivity is not supported on this device.
      return;
    }

    final int currentContentSensitivityId = await _sensitiveContentService.getContentSensitivity();

    // If needed, set default content sensitivity level as set in native Android. This will be
    // auto sensitive if it is otherwise unset by the developer.
    _defaultContentSensitivitySetting ??= ContentSensitivity.getContentSensitivityById(
      currentContentSensitivityId,
    );

    // If needed, then set the initial content sensitivity state.
    _contentSensitivityState ??= ContentSensitivityState(_defaultContentSensitivitySetting!);

    // Update SensitiveContent widget count for those with desiredSensitivityLevel.
    _contentSensitivityState!.addWidgetWithContentSensitivity(desiredSensitivityLevel);

    // Verify that desiredSensitivityLevel should be set in order for sensitive
    // content to remain obscured.
    if (!shouldSetContentSensitivity(desiredSensitivityLevel)) {
      return;
    }

    // Set content sensitivity level as desiredSensitivityLevel and update stored data.
    _sensitiveContentService.setContentSensitivity(desiredSensitivityLevel);
    _contentSensitivityState!.currentContentSensitivitySetting = desiredSensitivityLevel;
  }

  /// Unregisters a [SensitiveContent] widget from the [ContentSensitivityState] tracking
  /// the content sensitivity level of the widget tree.
  static void unregister(ContentSensitivity widgetSensitivityLevel) {
    instance._unregister(widgetSensitivityLevel);
  }

  void _unregister(ContentSensitivity widgetSensitivityLevel) {
    if (!_contentSenstivityIsSupported!) {
      // Setting content sensitivity is not supported on this device.
      return;
    }

    // Update SensitiveContent widget count for those with
    // desiredSensitivityLevel.
    _contentSensitivityState!.removeWidgetWithContentSensitivity(widgetSensitivityLevel);

    if (_contentSensitivityState!.getTotalNumberOfWidgets() == 0) {
      // Restore default content sensitivity setting if there are no more SensitiveContent
      // widgets in the tree.
      _sensitiveContentService.setContentSensitivity(_defaultContentSensitivitySetting!);
      _contentSensitivityState!.currentContentSensitivitySetting =
          _defaultContentSensitivitySetting!;
      return;
    }

    // Determine if another sensitivity level needs to be restored.
    ContentSensitivity? contentSensitivityToRestore;
    switch (widgetSensitivityLevel) {
      case ContentSensitivity.sensitive:
        if (shouldSetContentSensitivity(ContentSensitivity.notSensitive)) {
          contentSensitivityToRestore = ContentSensitivity.notSensitive;
        } else if (shouldSetContentSensitivity(ContentSensitivity.autoSensitive)) {
          contentSensitivityToRestore = ContentSensitivity.autoSensitive;
        }
      case ContentSensitivity.autoSensitive:
        if (shouldSetContentSensitivity(ContentSensitivity.notSensitive)) {
          contentSensitivityToRestore = ContentSensitivity.notSensitive;
        }
      case ContentSensitivity.notSensitive:
      // Removing a not sensitive SensitiveContent widgets when there are other SensitiveContent widgets
      // in the tree will have no impact on the content sensitivity setting for the widget tree since
      // they have the least severe content sensitivity level.
    }

    if (contentSensitivityToRestore != null) {
      // Set content sensitivity level as contentSensitivityToRestore and update stored data.
      _contentSensitivityState!.currentContentSensitivitySetting = contentSensitivityToRestore;
      _sensitiveContentService.setContentSensitivity(contentSensitivityToRestore);
    }
  }

  /// Return whether or not [desiredSensitivityLevel] should be set as the new
  /// [ContentSensitivity] level for the widget tree.
  ///
  /// [desiredSensitivityLevel] should only be set if it is strictly more
  /// severe than any of the other [SensitiveContent] widgets in the widget tree.
  bool shouldSetContentSensitivity(ContentSensitivity desiredSensitivityLevel) {
    if (_contentSensitivityState!.currentContentSensitivitySetting == desiredSensitivityLevel) {
      return false;
    }

    switch (desiredSensitivityLevel) {
      case ContentSensitivity.sensitive:
        return true;
      case ContentSensitivity.autoSensitive:
        return _contentSensitivityState!.sensitiveWidgetCount == 0;
      case ContentSensitivity.notSensitive:
        return _contentSensitivityState!.sensitiveWidgetCount +
                _contentSensitivityState!.autoSensitiveWidgetCount ==
            0;
    }
  }
}

/// Widget to set the [ContentSensitivity] level of content in the widget
/// tree.
///
/// {@macro flutter.services.ContentSensitivity}
///
/// See also:
///
///  * [ContentSensitivity], which are the different content sensitivity levels that a
///    [SensitiveContent] widget can set.
class SensitiveContent extends StatefulWidget {
  /// Creates a [SensitiveContent] widget.
  const SensitiveContent({super.key, required this.sensitivityLevel, required this.child});

  /// The sensitivity level that the [SensitiveContent] widget should sets for the
  /// Android native `View`.
  final ContentSensitivity sensitivityLevel;

  /// The child widget of this [SensitiveContent].
  ///
  /// If the [sensitivityLevel] is set to [ContentSensitivity.sensitive], then
  /// the entire screen will be obscured when the screen is projected irrespective
  /// to the parent/child widgets.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  State<SensitiveContent> createState() => _SensitiveContentState();
}

class _SensitiveContentState extends State<SensitiveContent> {
  Future<void>? _sensitiveContentRegistrationFuture;

  @override
  void initState() {
    super.initState();
    _sensitiveContentRegistrationFuture = SensitiveContentSetting.register(widget.sensitivityLevel);
  }

  @override
  void dispose() {
    SensitiveContentSetting.unregister(widget.sensitivityLevel);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<void>(
      future: _sensitiveContentRegistrationFuture,
      builder: (BuildContext context, AsyncSnapshot<void> snapshot) => widget.child,
    );
  }
}
