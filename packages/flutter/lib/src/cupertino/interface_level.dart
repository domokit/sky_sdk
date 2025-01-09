// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import '../widgets/framework.dart';

/// Indicates the visual level for a piece of content. Equivalent to `UIUserInterfaceLevel`
/// from `UIKit`.
///
/// See also:
///
///  * `UIUserInterfaceLevel`, the UIKit equivalent: https://developer.apple.com/documentation/uikit/uiuserinterfacelevel.
enum CupertinoUserInterfaceLevelData {
  /// The level for your window's main content.
  base,

  /// The level for content visually above [base].
  elevated,
}

/// Establishes a subtree in which [CupertinoUserInterfaceLevel.of] resolves to
/// the given visual elevation from the [CupertinoUserInterfaceLevelData]. This
/// can be used to apply style differences based on a widget's elevation.
///
/// Querying the current elevation status using [CupertinoUserInterfaceLevel.of]
/// will cause your widget to rebuild automatically whenever the
/// [CupertinoUserInterfaceLevelData] changes.
///
/// If no [CupertinoUserInterfaceLevel] is in scope then the
/// [CupertinoUserInterfaceLevel.of] method will throw an exception.
/// Alternatively, [CupertinoUserInterfaceLevel.maybeOf] can be used, which
/// returns null instead of throwing if no [CupertinoUserInterfaceLevel] is in
/// scope.
///
/// See also:
///
///  * [CupertinoUserInterfaceLevelData], specifies the visual level for the content
///    in the subtree [CupertinoUserInterfaceLevel] established.
class CupertinoUserInterfaceLevel extends InheritedWidget {
  /// Creates a [CupertinoUserInterfaceLevel] to change descendant Cupertino widget's
  /// visual level.
  const CupertinoUserInterfaceLevel({
    super.key,
    required CupertinoUserInterfaceLevelData data,
    required super.child,
  }) : _data = data;

  final CupertinoUserInterfaceLevelData _data;

  @override
  bool updateShouldNotify(CupertinoUserInterfaceLevel oldWidget) => oldWidget._data != _data;

  /// The data from the closest instance of this class that encloses the given
  /// context.
  ///
  /// You can use this function to query the user interface elevation level within
  /// the given [BuildContext]. When that information changes, your widget will
  /// be scheduled to be rebuilt, keeping your widget up-to-date.
  ///
  /// See also:
  ///
  ///  * [maybeOf], which is similar, but will return null if no
  ///    [CupertinoUserInterfaceLevel] encloses the given context.
  static CupertinoUserInterfaceLevelData of(BuildContext context) {
    final CupertinoUserInterfaceLevel? query =
        context.dependOnInheritedWidgetOfExactType<CupertinoUserInterfaceLevel>();
    if (query != null) {
      return query._data;
    }
    throw FlutterError(
      'CupertinoUserInterfaceLevel.of() called with a context that does not contain a CupertinoUserInterfaceLevel.\n'
      'No CupertinoUserInterfaceLevel ancestor could be found starting from the context that was passed '
      'to CupertinoUserInterfaceLevel.of(). This can happen because you do not have a WidgetsApp or '
      'MaterialApp widget (those widgets introduce a CupertinoUserInterfaceLevel), or it can happen '
      'if the context you use comes from a widget above those widgets.\n'
      'The context used was:\n'
      '  $context',
    );
  }

  /// The data from the closest instance of this class that encloses the given
  /// context, if there is one.
  ///
  /// Returns null if no [CupertinoUserInterfaceLevel] encloses the given context.
  ///
  /// You can use this function to query the user interface elevation level within
  /// the given [BuildContext]. When that information changes, your widget will
  /// be scheduled to be rebuilt, keeping your widget up-to-date.
  ///
  /// See also:
  ///
  ///  * [of], which is similar, but will throw an exception if no
  ///    [CupertinoUserInterfaceLevel] encloses the given context.
  static CupertinoUserInterfaceLevelData? maybeOf(BuildContext context) {
    final CupertinoUserInterfaceLevel? query =
        context.dependOnInheritedWidgetOfExactType<CupertinoUserInterfaceLevel>();
    return query?._data;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<CupertinoUserInterfaceLevelData>('user interface level', _data));
  }
}
