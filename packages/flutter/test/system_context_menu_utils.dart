// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart' show FlutterError;

/// Returns a [IOSSystemContextMenuItem] of the correct subclass given its
/// json data.
IOSSystemContextMenuItemData systemContextMenuItemDataFromJson(Map<String, dynamic> json) {
  final String? type = json['type'] as String?;
  final String? title = json['title'] as String?;
  return switch (type) {
    'copy' => const IOSSystemContextMenuItemDataCopy(),
    'cut' => const IOSSystemContextMenuItemDataCut(),
    'paste' => const IOSSystemContextMenuItemDataPaste(),
    'selectAll' => const IOSSystemContextMenuItemDataSelectAll(),
    'searchWeb' => IOSSystemContextMenuItemDataSearchWeb(title: title!),
    'share' => IOSSystemContextMenuItemDataShare(title: title!),
    'lookUp' => IOSSystemContextMenuItemDataLookUp(title: title!),
    _ => throw FlutterError('Invalid json for IOSSystemContextMenuItem.type $type.'),
  };
}
