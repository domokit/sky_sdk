// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TabController].

void main() => runApp(const TabControllerExampleApp());

class TabControllerExampleApp extends StatelessWidget {
  const TabControllerExampleApp({super.key});

  static const List<Tab> tabs = <Tab>[Tab(text: 'Zeroth'), Tab(text: 'First'), Tab(text: 'Second')];

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: TabControllerExample(tabs: tabs));
  }
}

class TabControllerExample extends StatelessWidget {
  const TabControllerExample({required this.tabs, super.key});

  final List<Tab> tabs;

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: tabs.length,
      child: DefaultTabControllerListener(
        onTabChanged: (int index) {
          debugPrint('tab changed: $index');
        },
        child: Scaffold(
          appBar: AppBar(bottom: TabBar(tabs: tabs)),
          body: TabBarView(
            children:
                tabs.map((Tab tab) {
                  return Center(
                    child: Text(
                      '${tab.text!} Tab',
                      style: Theme.of(context).textTheme.headlineSmall,
                    ),
                  );
                }).toList(),
          ),
        ),
      ),
    );
  }
}

class DefaultTabControllerListener extends StatefulWidget {
  const DefaultTabControllerListener({required this.onTabChanged, required this.child, super.key});

  final ValueChanged<int> onTabChanged;

  final Widget child;

  @override
  State<DefaultTabControllerListener> createState() => _DefaultTabControllerListenerState();
}

class _DefaultTabControllerListenerState extends State<DefaultTabControllerListener> {
  TabController? _controller;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();

    final TabController? defaultTabController = DefaultTabController.maybeOf(context);

    assert(() {
      if (defaultTabController == null) {
        throw FlutterError(
          'No DefaultTabController for ${widget.runtimeType}.\n'
          'When creating a ${widget.runtimeType}, you must ensure that there '
          'is a DefaultTabController above the ${widget.runtimeType}.',
        );
      }
      return true;
    }());

    if (defaultTabController != _controller) {
      _controller?.removeListener(_listener);
      _controller = defaultTabController;
      _controller?.addListener(_listener);
    }
  }

  void _listener() {
    final TabController? controller = _controller;

    if (controller == null || controller.indexIsChanging) {
      return;
    }

    widget.onTabChanged(controller.index);
  }

  @override
  void dispose() {
    _controller?.removeListener(_listener);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}
