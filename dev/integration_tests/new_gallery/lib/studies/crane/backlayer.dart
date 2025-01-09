// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

abstract class BackLayerItem extends StatefulWidget {
  const BackLayerItem({super.key, required this.index});
  final int index;
}

class BackLayer extends StatefulWidget {
  const BackLayer({super.key, required this.backLayerItems, required this.tabController});
  final List<BackLayerItem> backLayerItems;
  final TabController tabController;

  @override
  State<BackLayer> createState() => _BackLayerState();
}

class _BackLayerState extends State<BackLayer> {
  @override
  void initState() {
    super.initState();
    widget.tabController.addListener(() => setState(() {}));
  }

  @override
  Widget build(BuildContext context) {
    final int tabIndex = widget.tabController.index;
    return IndexedStack(
      index: tabIndex,
      children: <Widget>[
        for (final BackLayerItem backLayerItem in widget.backLayerItems)
          ExcludeFocus(excluding: backLayerItem.index != tabIndex, child: backLayerItem),
      ],
    );
  }
}
