// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This sample demonstrates nested navigation in a bottom navigation bar.

import 'package:flutter/material.dart';

// There are three possible tabs.
enum _Tab { home, one, two }

// Each tab has two possible pages.
enum _TabPage {
  home,
  one;

  static _TabPage? fromName(String? name) {
    return switch (name) {
      'home' => _TabPage.home,
      'one' => _TabPage.one,
      _ => null,
    };
  }
}

typedef _TabPageCallback = void Function(List<_TabPage> pages);

void main() => runApp(const NavigatorPopHandlerApp());

class NavigatorPopHandlerApp extends StatelessWidget {
  const NavigatorPopHandlerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      initialRoute: '/',
      restorationScopeId: 'root',
      onGenerateRoute: (RouteSettings settings) {
        return switch (settings.name) {
          '/' => MaterialPageRoute<void>(
            settings: const RouteSettings(name: '/'),
            builder: (BuildContext context) {
              return const _BottomNavPage();
            },
          ),
          _ => MaterialPageRoute<void>(
            settings: const RouteSettings(name: 'unknown_page'),
            builder: (BuildContext context) {
              return const _UnknownPage();
            },
          ),
        };
      },
    );
  }
}

class _BottomNavPage extends StatefulWidget {
  const _BottomNavPage();

  @override
  State<_BottomNavPage> createState() => _BottomNavPageState();
}

class _BottomNavPageState extends State<_BottomNavPage> with RestorationMixin {
  final _RestorableTab _restorableTab = _RestorableTab();

  final GlobalKey _tabHomeKey = GlobalKey();
  final GlobalKey _tabOneKey = GlobalKey();
  final GlobalKey _tabTwoKey = GlobalKey();

  final _RestorableTabPageList _restorableTabHomePages = _RestorableTabPageList();
  final _RestorableTabPageList _restorableTabOnePages = _RestorableTabPageList();
  final _RestorableTabPageList _restorableTabTwoPages = _RestorableTabPageList();

  BottomNavigationBarItem _itemForPage(_Tab page) {
    switch (page) {
      case _Tab.home:
        return const BottomNavigationBarItem(icon: Icon(Icons.home), label: 'Go to Home');
      case _Tab.one:
        return const BottomNavigationBarItem(icon: Icon(Icons.one_k), label: 'Go to One');
      case _Tab.two:
        return const BottomNavigationBarItem(icon: Icon(Icons.two_k), label: 'Go to Two');
    }
  }

  Widget _getPage(_Tab page) {
    switch (page) {
      case _Tab.home:
        return _BottomNavTab(
          key: _tabHomeKey,
          title: 'Home Tab',
          color: Colors.grey,
          pages: _restorableTabHomePages.value,
          onChangePages: (List<_TabPage> pages) {
            setState(() {
              _restorableTabHomePages.value = pages;
            });
          },
        );
      case _Tab.one:
        return _BottomNavTab(
          key: _tabOneKey,
          title: 'Tab One',
          color: Colors.amber,
          pages: _restorableTabOnePages.value,
          onChangePages: (List<_TabPage> pages) {
            setState(() {
              _restorableTabOnePages.value = pages;
            });
          },
        );
      case _Tab.two:
        return _BottomNavTab(
          key: _tabTwoKey,
          title: 'Tab Two',
          color: Colors.blueGrey,
          pages: _restorableTabTwoPages.value,
          onChangePages: (List<_TabPage> pages) {
            setState(() {
              _restorableTabTwoPages.value = pages;
            });
          },
        );
    }
  }

  void _onItemTapped(int index) {
    setState(() {
      _restorableTab.value = _Tab.values.elementAt(index);
    });
  }

  // Begin RestorationMixin.

  @override
  String? get restorationId => 'bottom-nav-page';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_restorableTab, 'tab');
    registerForRestoration(_restorableTabHomePages, 'tab-home-pages');
    registerForRestoration(_restorableTabOnePages, 'tab-one-pages');
    registerForRestoration(_restorableTabTwoPages, 'tab-two-pages');
  }

  /// End RestorationMixin.

  @override
  void dispose() {
    _restorableTab.dispose();
    _restorableTabHomePages.dispose();
    _restorableTabOnePages.dispose();
    _restorableTabTwoPages.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(child: _getPage(_restorableTab.value)),
      bottomNavigationBar: BottomNavigationBar(
        items: _Tab.values.map(_itemForPage).toList(),
        currentIndex: _Tab.values.indexOf(_restorableTab.value),
        selectedItemColor: Colors.amber[800],
        onTap: _onItemTapped,
      ),
    );
  }
}

class _BottomNavTab extends StatefulWidget {
  const _BottomNavTab({
    super.key,
    required this.color,
    required this.onChangePages,
    required this.pages,
    required this.title,
  });

  final Color color;
  final _TabPageCallback onChangePages;
  final List<_TabPage> pages;
  final String title;

  @override
  State<_BottomNavTab> createState() => _BottomNavTabState();
}

class _BottomNavTabState extends State<_BottomNavTab> {
  final GlobalKey<NavigatorState> _navigatorKey = GlobalKey<NavigatorState>();

  @override
  Widget build(BuildContext context) {
    return NavigatorPopHandler(
      onPop: () {
        _navigatorKey.currentState?.maybePop();
      },
      child: Navigator(
        key: _navigatorKey,
        restorationScopeId: 'nested-navigator-${widget.title}',
        onDidRemovePage: (Page<Object?> page) {
          final _TabPage? tabPage = _TabPage.fromName(page.name);
          if (tabPage == null) {
            return;
          }
          final List<_TabPage> nextPages = <_TabPage>[...widget.pages]..remove(tabPage);
          if (nextPages.length < widget.pages.length) {
            widget.onChangePages(nextPages);
          }
        },
        pages:
            widget.pages.map((_TabPage page) {
              switch (page) {
                case _TabPage.home:
                  return MaterialPage<void>(
                    restorationId: _TabPage.home.toString(),
                    name: 'home',
                    child: _LinksPage(
                      title: 'Bottom nav - tab ${widget.title} - route $page',
                      backgroundColor: widget.color,
                      buttons: <Widget>[
                        TextButton(
                          onPressed: () {
                            assert(!widget.pages.contains(_TabPage.one));
                            widget.onChangePages(<_TabPage>[...widget.pages, _TabPage.one]);
                          },
                          child: const Text('Go to another route in this nested Navigator'),
                        ),
                      ],
                    ),
                  );
                case _TabPage.one:
                  return MaterialPage<void>(
                    restorationId: _TabPage.one.toString(),
                    name: 'one',
                    child: _LinksPage(
                      backgroundColor: widget.color,
                      title: 'Bottom nav - tab ${widget.title} - route $page',
                      buttons: <Widget>[
                        TextButton(
                          onPressed: () {
                            widget.onChangePages(<_TabPage>[...widget.pages]..removeLast());
                          },
                          child: const Text('Go back'),
                        ),
                      ],
                    ),
                  );
              }
            }).toList(),
      ),
    );
  }
}

class _LinksPage extends StatelessWidget {
  const _LinksPage({
    required this.backgroundColor,
    this.buttons = const <Widget>[],
    required this.title,
  });

  final Color backgroundColor;
  final List<Widget> buttons;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: backgroundColor,
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[Text(title), ...buttons],
        ),
      ),
    );
  }
}

class _UnknownPage extends StatelessWidget {
  const _UnknownPage();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey.withBlue(180),
      body: const Center(
        child: Column(mainAxisAlignment: MainAxisAlignment.center, children: <Widget>[Text('404')]),
      ),
    );
  }
}

class _RestorableTab extends RestorableValue<_Tab> {
  @override
  _Tab createDefaultValue() => _Tab.home;

  @override
  void didUpdateValue(_Tab? oldValue) {
    if (oldValue == null || oldValue != value) {
      notifyListeners();
    }
  }

  @override
  _Tab fromPrimitives(Object? data) {
    if (data != null) {
      final String tabString = data as String;
      return _Tab.values.firstWhere((_Tab tab) => tabString == tab.name);
    }
    return _Tab.home;
  }

  @override
  Object toPrimitives() {
    return value.name;
  }
}

class _RestorableTabPageList extends RestorableValue<List<_TabPage>> {
  @override
  List<_TabPage> createDefaultValue() => <_TabPage>[_TabPage.home];

  @override
  void didUpdateValue(List<_TabPage>? oldValue) {
    if (oldValue == null || oldValue != value) {
      notifyListeners();
    }
  }

  @override
  List<_TabPage> fromPrimitives(Object? data) {
    if (data != null) {
      final String dataString = data as String;
      final List<String> listOfStrings = dataString.split(',');
      return listOfStrings.map((String tabPageName) {
        return _TabPage.values.firstWhere((_TabPage tabPage) => tabPageName == tabPage.name);
      }).toList();
    }
    return <_TabPage>[];
  }

  @override
  Object toPrimitives() {
    return value.map((_TabPage tabPage) => tabPage.name).join(',');
  }
}
