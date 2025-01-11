// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

/// Returns a [SystemContextMenuItemData] of the correct subclass given its
/// json data.
SystemContextMenuItemData systemContextMenuItemDataFromJson(Map<String, dynamic> json) {
  final String? type = json['type'] as String?;
  final String? title = json['title'] as String?;
  final VoidCallback? onPressed = json['onPressed'] as VoidCallback?;
  return switch (type) {
    'copy' => const SystemContextMenuItemDataCopy(),
    'cut' => const SystemContextMenuItemDataCut(),
    'paste' => const SystemContextMenuItemDataPaste(),
    'selectAll' => const SystemContextMenuItemDataSelectAll(),
    'searchWeb' => SystemContextMenuItemDataSearchWeb(title: title!),
    'share' => SystemContextMenuItemDataShare(title: title!),
    'lookUp' => SystemContextMenuItemDataLookUp(title: title!),
    'custom' => SystemContextMenuItemDataCustom(title: title!, onPressed: onPressed!),
    _ => throw FlutterError('Invalid json for SystemContextMenuItemData.type $type.'),
  };
}
