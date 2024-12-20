// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _explanatoryText =
    "When the Scaffold's floating action button changes, the new button fades and "
    'turns into view. In this demo, changing tabs can cause the app to be rebuilt '
    'with a FloatingActionButton that the Scaffold distinguishes from the others '
    'by its key.';

class _Page {
  _Page({this.label, this.colors, this.icon});

  final String? label;
  final MaterialColor? colors;
  final IconData? icon;

  Color get labelColor => colors != null ? colors!.shade300 : Colors.grey.shade300;
  bool get fabDefined => colors != null && icon != null;
  Color get fabColor => colors!.shade400;
  Icon get fabIcon => Icon(icon);
  Key get fabKey => ValueKey<Color>(fabColor);
}

final List<_Page> _allPages = <_Page>[
  _Page(label: 'Pink', colors: Colors.pink, icon: Icons.add),
  _Page(label: 'Eco', colors: Colors.green, icon: Icons.create),
  _Page(label: 'No'),
  _Page(label: 'Teal', colors: Colors.teal, icon: Icons.add),
  _Page(label: 'Red', colors: Colors.red, icon: Icons.create),
];

class TabsFabDemo extends StatefulWidget {
  const TabsFabDemo({super.key});

  static const String routeName = '/material/tabs-fab';

  @override
  State<TabsFabDemo> createState() => _TabsFabDemoState();
}

class _TabsFabDemoState extends State<TabsFabDemo> with SingleTickerProviderStateMixin {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();

  TabController? _controller;
  late _Page _selectedPage;
  bool _extendedButtons = false;

  @override
  void initState() {
    super.initState();
    _controller = TabController(vsync: this, length: _allPages.length);
    _controller!.addListener(_handleTabSelection);
    _selectedPage = _allPages[0];
  }

  @override
  void dispose() {
    _controller!.dispose();
    super.dispose();
  }

  void _handleTabSelection() {
    setState(() {
      _selectedPage = _allPages[_controller!.index];
    });
  }

  void _showExplanatoryText() {
    _scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return Container(
        decoration: BoxDecoration(
          border: Border(top: BorderSide(color: Theme.of(context).dividerColor)),
        ),
        child: Padding(
          padding: const EdgeInsets.all(32.0),
          child: Text(_explanatoryText, style: Theme.of(context).textTheme.titleMedium),
        ),
      );
    });
  }

  Widget buildTabView(_Page page) {
    return Builder(
      builder: (BuildContext context) {
        return Container(
          key: ValueKey<String?>(page.label),
          padding: const EdgeInsets.fromLTRB(48.0, 48.0, 48.0, 96.0),
          child: Card(
            child: Center(
              child: Text(
                page.label!,
                style: TextStyle(color: page.labelColor, fontSize: 32.0),
                textAlign: TextAlign.center,
              ),
            ),
          ),
        );
      },
    );
  }

  Widget? buildFloatingActionButton(_Page page) {
    if (!page.fabDefined) {
      return null;
    }

    if (_extendedButtons) {
      return FloatingActionButton.extended(
        key: ValueKey<Key>(page.fabKey),
        tooltip: 'Show explanation',
        backgroundColor: page.fabColor,
        icon: page.fabIcon,
        label: Text(page.label!.toUpperCase()),
        onPressed: _showExplanatoryText,
      );
    }

    return FloatingActionButton(
      key: page.fabKey,
      tooltip: 'Show explanation',
      backgroundColor: page.fabColor,
      onPressed: _showExplanatoryText,
      child: page.fabIcon,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _scaffoldKey,
      appBar: AppBar(
        title: const Text('FAB per tab'),
        bottom: TabBar(
          controller: _controller,
          tabs:
              _allPages.map<Widget>((_Page page) => Tab(text: page.label!.toUpperCase())).toList(),
        ),
        actions: <Widget>[
          MaterialDemoDocumentationButton(TabsFabDemo.routeName),
          IconButton(
            icon: const Icon(
              Icons.sentiment_very_satisfied,
              semanticLabel: 'Toggle extended buttons',
            ),
            onPressed: () {
              setState(() {
                _extendedButtons = !_extendedButtons;
              });
            },
          ),
        ],
      ),
      floatingActionButton: buildFloatingActionButton(_selectedPage),
      body: TabBarView(
        controller: _controller,
        children: _allPages.map<Widget>(buildTabView).toList(),
      ),
    );
  }
}
