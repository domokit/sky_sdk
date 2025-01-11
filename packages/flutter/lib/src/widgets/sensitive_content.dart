// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart' show ContentSensitivity, SensitiveContentService;

import 'framework.dart';

/// Data structure used to track the number of [SensitiveContent] widgets with
/// each of the different [ContentSensitivity] levels set in a particular
/// Flutter view.
class ViewContentSensitivityState {
  /// Creates a [ViewContentSensitivityState].
  ViewContentSensitivityState();

  /// The current [ContentSensitivity] level set for the Flutter view that this
  /// state represents.
  /// 
  /// By default, this level is [ContentSensitivity.autoSensitive] because this
  /// is the default level on Android and this feature is currently only
  /// supported for Android.
  ContentSensitivity currentContentSensitivitySetting = ContentSensitivity.autoSensitive;

  /// A map that contains the number of [SensitiveContent] widgets that have
  /// each of the different [ContentSensitivity] levels.
  final Map<ContentSensitivity, int> contentSensitivityCounts = <ContentSensitivity, int> {
    ContentSensitivity.sensitive: 0,
    ContentSensitivity.autoSensitive: 0,
    ContentSensitivity.notSensitive: 0,
  };

  /// Increases the count of [SensitiveContent] widgets with [sensitivityLevel] set.
  void addWidgetWithContentSensitivity(ContentSensitivity sensitivityLevel) {
    contentSensitivityCounts[sensitivityLevel] = contentSensitivityCounts[sensitivityLevel]! + 1;
  }

  /// Decreases the count of [SensitiveContent] widgets with [sensitivityLevel] set.
  void removeWidgetWithContentSensitivity(ContentSensitivity sensitivityLevel) { 
    contentSensitivityCounts[sensitivityLevel] = contentSensitivityCounts[sensitivityLevel]! - 1;
  }

  /// Returns the number of [SenstiveContent] widgets represented by this state.
  int getTotalNumberOfWidgets() {
    return contentSensitivityCounts.values.reduce((int sum, int value) => sum + value);
  }
}

/// Host of the current content sensitivity level for each Flutter view that
/// contains any [SensitiveContent] widgets.
class SensitiveContentSetting {
  SensitiveContentSetting._();

  final Map<int, ViewContentSensitivityState> _contentSensitivityStates = <int, ViewContentSensitivityState> {};
  final SensitiveContentService _sensitiveContentService = SensitiveContentService();

  static final SensitiveContentSetting _instance = SensitiveContentSetting._();

  /// Registers a [SensitiveContent] widget that will help determine the
  /// [ContentSensitivity] level for the Flutter view with ID [viewId].
  static void register(int viewId, ContentSensitivity desiredSensitivityLevel) {
     _instance._register(viewId, desiredSensitivityLevel);
  }

  void _register(int viewId, ContentSensitivity desiredSensitivityLevel) {
    if (!_contentSensitivityStates.containsKey(viewId)) {
      _contentSensitivityStates[viewId] = ViewContentSensitivityState();
    }

    // Update SensitiveContent widget count for those with desiredSensitivityLevel.
    final ViewContentSensitivityState contentSensitivityStateForView = _contentSensitivityStates[viewId]!;
    contentSensitivityStateForView.addWidgetWithContentSensitivity(desiredSensitivityLevel);


    // If only one SensitiveContent widget in the relevant view sets a
    // ContentSensitivity level, then we can immediately set
    // desiredSensitivityLevel for the view.
    if (contentSensitivityStateForView.getTotalNumberOfWidgets() == 1) {
      // Set content sensitivity level for view as desiredSensitivityLevel and update stored data.
      _sensitiveContentService.setContentSensitivity(viewId, desiredSensitivityLevel);
      contentSensitivityStateForView.currentContentSensitivitySetting = desiredSensitivityLevel;
      return;
    }

    // Verify that desiredSensitivityLevel should be set in order for sensitive
    // content in the view to remain secure.
    if (!shouldSetContentSensitivity(currentSensitivityLevel: contentSensitivityStateForView.currentContentSensitivitySetting, desiredSensitivityLevel: desiredSensitivityLevel)) {
      return;
    }

    // Set content sensitivity level for the view as desiredSensitivityLevel and update stored data.
    _sensitiveContentService.setContentSensitivity(viewId, desiredSensitivityLevel);
    contentSensitivityStateForView.currentContentSensitivitySetting = desiredSensitivityLevel;
  }

  /// Unregisters a [SensitiveContent] widget from the Flutter view with ID
  /// [viewId].
  static void unregister(int viewId, ContentSensitivity widgetSensitivityLevel) {
     _instance._unregister(viewId, widgetSensitivityLevel);
  }

  void _unregister(int viewId, ContentSensitivity widgetSensitivityLevel) {
    // Update SensitiveContent widget count for those with
    // desiredSensitivityLevel.
    final ViewContentSensitivityState contentSensitivityStateForView = _contentSensitivityStates[viewId]!;
    contentSensitivityStateForView.removeWidgetWithContentSensitivity(widgetSensitivityLevel);

    if (contentSensitivityStateForView.getTotalNumberOfWidgets() == 0) {
      // There is no longer sensitive content in the view. Reset to the default
      // mode.
      _sensitiveContentService.setContentSensitivity(viewId, ContentSensitivity.autoSensitive);
      return;
    }

    final ContentSensitivity currentSensitivityLevelForView = contentSensitivityStateForView.currentContentSensitivitySetting;
    final Map<ContentSensitivity, int> contentSensitivityCountsForView = contentSensitivityStateForView.contentSensitivityCounts;
    final int numWidgetsWithWidgetSensitivityLevel = contentSensitivityCountsForView[widgetSensitivityLevel]!;

    if (widgetSensitivityLevel != currentSensitivityLevelForView
      || numWidgetsWithWidgetSensitivityLevel > 0) {
      // Either another SensitiveContent widget has set a more severe ContentSensitivity
      // level for the view or there are other widgets that have requested the same level
      // in the view.
      return;
    }

    // If the SensitiveContent widget being unregistered had the most severe
    // ContentSensitivity level, find the SensitiveContent widget in the view
    // with the next most severe level and set this level for the view.
    ContentSensitivity? sensitivityLevelToSet;
    switch (widgetSensitivityLevel) {
      case ContentSensitivity.sensitive:
        if (contentSensitivityCountsForView[ContentSensitivity.autoSensitive]! > 0) {
          sensitivityLevelToSet = ContentSensitivity.autoSensitive;
          break;
        }
        continue auto;
      auto:
      case ContentSensitivity.autoSensitive:
        if (contentSensitivityCountsForView[ContentSensitivity.notSensitive]! > 0) {
          sensitivityLevelToSet = ContentSensitivity.notSensitive;
          break;
        }
        continue not;
      not:
      case ContentSensitivity.notSensitive:
        throw StateError('The SensitiveContentSetting has gotten out of sync with the SensitiveContent widgets in the Flutter view with ID $viewId.');
    }

    _sensitiveContentService.setContentSensitivity(viewId, sensitivityLevelToSet);
  }

  /// Return whether or not [desiredSensitivityLevel] should be set as the new
  /// [ContentSensitivity] level for a Flutter view that currently has
  /// [currentSensitivityLevel] set.
  ///
  /// [desiredSensitivityLevel] should be set only if it is striclty less
  /// severe than any of the other registered [SensitiveContent] widgets in the view.
  bool shouldSetContentSensitivity({required ContentSensitivity currentSensitivityLevel, required ContentSensitivity desiredSensitivityLevel}) {
    if (currentSensitivityLevel == desiredSensitivityLevel) {
      return false;
    }

    switch(desiredSensitivityLevel) {
      case ContentSensitivity.sensitive:
        return true;
      case ContentSensitivity.autoSensitive:
        return currentSensitivityLevel != ContentSensitivity.sensitive;
      case ContentSensitivity.notSensitive:
        return currentSensitivityLevel == ContentSensitivity.notSensitive;
    }
  }
}

/// Widget to set the [ContentSensitivity] level of content in a particular
/// Flutter view.
class SensitiveContent extends StatefulWidget {
  /// Creates a [SensitiveContent].
  const SensitiveContent({
    super.key,
    this.viewId = 0,
    required this.sensitivityLevel,
    required this.child,
  });

  /// The ID of the Flutter view that [sensitivityLevel] should be set for.
  /// 
  /// By default, this is 0. On Android, this is the the ID of the native
  /// `View` that is created by the default `FlutterActivity`, which is used by
  /// default in Flutter Android apps.
  final int viewId;

  /// The sensitivity level that the [SensitiveContent] widget should set.
  final ContentSensitivity sensitivityLevel;

  /// The child widget of this [SensitiveContent].
  ///
  /// If the [sensitivityLevel] is set to [ContentSensitivity.sensitive], then
  /// the entire screen will be obscured when the screen is projected regardless
  /// of the parent/child widgets.
  /// 
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  State<SensitiveContent> createState() => _SensitiveContentState();
}

class _SensitiveContentState extends State<SensitiveContent> {
  @override
  void initState() {
    super.initState();
    SensitiveContentSetting.register(widget.viewId, widget.sensitivityLevel);
  }

  @override
  void dispose() {
    SensitiveContentSetting.unregister(widget.viewId, widget.sensitivityLevel);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}
