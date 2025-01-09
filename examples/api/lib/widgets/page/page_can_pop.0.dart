// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This sample demonstrates showing a confirmation dialog before navigating
// away from a page.

import 'package:flutter/material.dart';

void main() => runApp(const PageApiExampleApp());

class PageApiExampleApp extends StatefulWidget {
  const PageApiExampleApp({super.key});

  @override
  State<PageApiExampleApp> createState() => _PageApiExampleAppState();
}

class _PageApiExampleAppState extends State<PageApiExampleApp> {
  final RouterDelegate<Object> delegate = MyRouterDelegate();

  @override
  Widget build(BuildContext context) {
    return MaterialApp.router(routerDelegate: delegate);
  }
}

class MyRouterDelegate extends RouterDelegate<Object>
    with PopNavigatorRouterDelegateMixin<Object>, ChangeNotifier {
  // This example doesn't use RouteInformationProvider.
  @override
  Future<void> setNewRoutePath(Object configuration) async => throw UnimplementedError();

  @override
  final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

  static MyRouterDelegate of(BuildContext context) =>
      Router.of(context).routerDelegate as MyRouterDelegate;

  bool get showDetailPage => _showDetailPage;
  bool _showDetailPage = false;
  set showDetailPage(bool value) {
    if (_showDetailPage == value) {
      return;
    }
    _showDetailPage = value;
    notifyListeners();
  }

  Future<bool> _showConfirmDialog() async {
    return await showDialog<bool>(
          context: navigatorKey.currentContext!,
          barrierDismissible: false,
          builder: (BuildContext context) {
            return AlertDialog(
              title: const Text('Are you sure?'),
              actions: <Widget>[
                TextButton(
                  child: const Text('Cancel'),
                  onPressed: () {
                    Navigator.of(context).pop(false);
                  },
                ),
                TextButton(
                  child: const Text('Confirm'),
                  onPressed: () {
                    Navigator.of(context).pop(true);
                  },
                ),
              ],
            );
          },
        ) ??
        false;
  }

  Future<void> _handlePopDetails(bool didPop, void result) async {
    if (didPop) {
      showDetailPage = false;
      return;
    }
    final bool confirmed = await _showConfirmDialog();
    if (confirmed) {
      showDetailPage = false;
    }
  }

  List<Page<Object?>> _getPages() {
    return <Page<Object?>>[
      const MaterialPage<void>(key: ValueKey<String>('home'), child: _HomePage()),
      if (showDetailPage)
        MaterialPage<void>(
          key: const ValueKey<String>('details'),
          child: const _DetailsPage(),
          canPop: false,
          onPopInvoked: _handlePopDetails,
        ),
    ];
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      key: navigatorKey,
      pages: _getPages(),
      onDidRemovePage: (Page<Object?> page) {
        assert(page.key == const ValueKey<String>('details'));
        showDetailPage = false;
      },
    );
  }
}

class _HomePage extends StatefulWidget {
  const _HomePage();

  @override
  State<_HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<_HomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Home')),
      body: Center(
        child: TextButton(
          onPressed: () {
            MyRouterDelegate.of(context).showDetailPage = true;
          },
          child: const Text('Go to details'),
        ),
      ),
    );
  }
}

class _DetailsPage extends StatefulWidget {
  const _DetailsPage();

  @override
  State<_DetailsPage> createState() => _DetailsPageState();
}

class _DetailsPageState extends State<_DetailsPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Details')),
      body: Center(
        child: TextButton(
          onPressed: () {
            Navigator.of(context).maybePop();
          },
          child: const Text('Go back'),
        ),
      ),
    );
  }
}
