// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class _PageSelector extends StatelessWidget {
  const _PageSelector({this.icons});

  final List<Icon>? icons;

  void _handleArrowButtonPress(BuildContext context, int delta) {
    final TabController controller = DefaultTabController.of(context);
    if (!controller.indexIsChanging) {
      controller.animateTo((controller.index + delta).clamp(0, icons!.length - 1));
    }
  }

  @override
  Widget build(BuildContext context) {
    final TabController? controller = DefaultTabController.maybeOf(context);
    final Color color = Theme.of(context).colorScheme.secondary;
    return SafeArea(
      top: false,
      bottom: false,
      child: Column(
        children: <Widget>[
          Container(
            margin: const EdgeInsets.only(top: 16.0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                IconButton(
                  icon: const Icon(Icons.chevron_left),
                  color: color,
                  onPressed: () {
                    _handleArrowButtonPress(context, -1);
                  },
                  tooltip: 'Page back',
                ),
                TabPageSelector(controller: controller),
                IconButton(
                  icon: const Icon(Icons.chevron_right),
                  color: color,
                  onPressed: () {
                    _handleArrowButtonPress(context, 1);
                  },
                  tooltip: 'Page forward',
                ),
              ],
            ),
          ),
          Expanded(
            child: IconTheme(
              data: IconThemeData(size: 128.0, color: color),
              child: TabBarView(
                children:
                    icons!.map<Widget>((Icon icon) {
                      return Container(
                        padding: const EdgeInsets.all(12.0),
                        child: Card(child: Center(child: icon)),
                      );
                    }).toList(),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class PageSelectorDemo extends StatelessWidget {
  const PageSelectorDemo({super.key});

  static const String routeName = '/material/page-selector';
  static final List<Icon> icons = <Icon>[
    const Icon(Icons.event, semanticLabel: 'Event'),
    const Icon(Icons.home, semanticLabel: 'Home'),
    const Icon(Icons.android, semanticLabel: 'Android'),
    const Icon(Icons.alarm, semanticLabel: 'Alarm'),
    const Icon(Icons.face, semanticLabel: 'Face'),
    const Icon(Icons.language, semanticLabel: 'Language'),
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Page selector'),
        actions: <Widget>[MaterialDemoDocumentationButton(routeName)],
      ),
      body: DefaultTabController(length: icons.length, child: _PageSelector(icons: icons)),
    );
  }
}
