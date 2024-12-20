// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:io';
library;

import 'package:flutter/foundation.dart';

import 'system_channels.dart';

/// Controls specific aspects of the system navigation stack.
abstract final class SystemNavigator {
  /// Informs the platform of whether or not the Flutter framework will handle
  /// back events.
  ///
  /// Currently, this is used only on Android to inform its use of the
  /// predictive back gesture when exiting the app. When true, predictive back
  /// is disabled.
  ///
  /// See also:
  ///
  ///  * The
  ///    [migration guide](https://developer.android.com/guide/navigation/predictive-back-gesture)
  ///    for predictive back in native Android apps.
  static Future<void> setFrameworkHandlesBack(bool frameworkHandlesBack) async {
    // Currently, this method call is only relevant on Android.
    if (kIsWeb) {
      return;
    }
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return;
      case TargetPlatform.android:
        return SystemChannels.platform.invokeMethod<void>(
          'SystemNavigator.setFrameworkHandlesBack',
          frameworkHandlesBack,
        );
    }
  }

  /// Removes the topmost Flutter instance, presenting what was before
  /// it.
  ///
  /// On Android, removes this activity from the stack and returns to
  /// the previous activity.
  ///
  /// On iOS, calls `popViewControllerAnimated:` if the root view
  /// controller is a `UINavigationController`, or
  /// `dismissViewControllerAnimated:completion:` if the top view
  /// controller is a `FlutterViewController`.
  ///
  /// The optional `animated` parameter is ignored on all platforms
  /// except iOS where it is an argument to the aforementioned
  /// methods.
  ///
  /// This method should be preferred over calling `dart:io`'s [exit]
  /// method, as the latter may cause the underlying platform to act
  /// as if the application had crashed.
  static Future<void> pop({bool? animated}) async {
    await SystemChannels.platform.invokeMethod<void>('SystemNavigator.pop', animated);
  }

  /// Selects the single-entry history mode.
  ///
  /// On web, this switches the browser history model to one that only tracks a
  /// single entry, so that calling [routeInformationUpdated] replaces the
  /// current entry.
  ///
  /// Currently, this is ignored on other platforms.
  ///
  /// See also:
  ///
  ///  * [selectMultiEntryHistory], which enables the browser history to have
  ///    multiple entries.
  static Future<void> selectSingleEntryHistory() {
    return SystemChannels.navigation.invokeMethod<void>('selectSingleEntryHistory');
  }

  /// Selects the multiple-entry history mode.
  ///
  /// On web, this switches the browser history model to one that tracks all
  /// updates to [routeInformationUpdated] to form a history stack. This is the
  /// default.
  ///
  /// Currently, this is ignored on other platforms.
  ///
  /// See also:
  ///
  ///  * [selectSingleEntryHistory], which forces the history to only have one
  ///    entry.
  static Future<void> selectMultiEntryHistory() {
    return SystemChannels.navigation.invokeMethod<void>('selectMultiEntryHistory');
  }

  /// Notifies the platform for a route information change.
  ///
  /// On web, this method behaves differently based on the single-entry or
  /// multiple-entries history mode. Use the [selectSingleEntryHistory] and
  /// [selectMultiEntryHistory] to toggle between modes.
  ///
  /// For single-entry mode, this method replaces the current URL and state in
  /// the current history entry. The flag `replace` is ignored.
  ///
  /// For multiple-entries mode, this method creates a new history entry on top
  /// of the current entry if the `replace` is false, thus the user will
  /// be on a new history entry as if the user has visited a new page, and the
  /// browser back button brings the user back to the previous entry. If
  /// `replace` is true, this method only updates the URL and the state in the
  /// current history entry without pushing a new one.
  ///
  /// This method is ignored on other platforms.
  ///
  /// The `replace` flag defaults to false.
  static Future<void> routeInformationUpdated({
    @Deprecated(
      'Pass Uri.parse(location) to uri parameter instead. '
      'This feature was deprecated after v3.8.0-3.0.pre.',
    )
    String? location,
    Uri? uri,
    Object? state,
    bool replace = false,
  }) {
    assert(
      (location != null) != (uri != null),
      'One of uri or location must be provided, but not both.',
    );
    uri ??= Uri.parse(location!);
    return SystemChannels.navigation.invokeMethod<void>(
      'routeInformationUpdated',
      <String, dynamic>{'uri': uri.toString(), 'state': state, 'replace': replace},
    );
  }
}
