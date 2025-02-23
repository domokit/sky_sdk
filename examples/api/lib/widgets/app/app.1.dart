// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

// This is an example of Navigator 2.0 with state restoration. If the app is
// unloaded by the OS while backgrounded and then the user returns to it, it
// will properly restore the entire navigation stack for both Navigators.

void main() async {
  runApp(const WidgetsAppExample());
}

class WidgetsAppExample extends StatefulWidget {
  const WidgetsAppExample({super.key});

  @override
  State<WidgetsAppExample> createState() => _WidgetsAppExampleState();
}

class _WidgetsAppExampleState extends State<WidgetsAppExample> {
  final _MyRouteInformationParser _routeInformationParser = _MyRouteInformationParser();
  final _MyRouterDelegate _routerDelegate = _MyRouterDelegate();

  @override
  Widget build(BuildContext context) {
    return WidgetsApp.router(
      restorationScopeId: 'widgets-app',
      color: Colors.red,
      localizationsDelegates: const <LocalizationsDelegate<dynamic>>[
        DefaultMaterialLocalizations.delegate,
      ],
      title: 'Nested navigation with state restoration example',
      routeInformationParser: _routeInformationParser,
      routerDelegate: _routerDelegate,
    );
  }
}

class _MyRouteInformationParser extends RouteInformationParser<_MyNavigationConfiguration> {
  @override
  SynchronousFuture<_MyNavigationConfiguration> parseRouteInformation(RouteInformation routeInformation) {
    // If no navigation stack info was saved, then just make the given URI the
    // only route in the new stack.
    if (routeInformation.state == null) {
      final _MyPage page = _MyPage.values.firstWhere((_MyPage pageConfiguration) {
        return pageConfiguration.uriString == routeInformation.uri.toString();
      },
        orElse: () => _MyPage.unknown,
      );
      return SynchronousFuture<_MyNavigationConfiguration>(_MyNavigationConfiguration.single(page));
    }

    // Otherwise, restore the whole given navigation stack.
    final Map<Object?, Object?> state = routeInformation.state! as Map<Object?, Object?>;
    final List<Object?>? pageNameObjects = state['pages'] as List<Object?>?;
    assert(pageNameObjects != null && pageNameObjects.isNotEmpty);
    final List<String> pageNames = pageNameObjects!.cast<String>();
    final List<_MyPage> pages = pageNames.map(_MyPage.fromName).toList();
    return SynchronousFuture<_MyNavigationConfiguration>(_MyNavigationConfiguration(pages: pages));
  }

  @override
  RouteInformation? restoreRouteInformation(_MyNavigationConfiguration configuration) {
    return RouteInformation(
      uri: configuration.uri,
      // Pass the state of the navigation stack in the state parameter.
      state: <String, List<String>>{
        'pages': configuration.pages.map((_MyPage page) => page.name).toList(),
      },
    );
  }
}

class _MyRouterDelegate extends RouterDelegate<_MyNavigationConfiguration>
    with ChangeNotifier, PopNavigatorRouterDelegateMixin<_MyNavigationConfiguration> {
  final Set<VoidCallback> _listeners = <VoidCallback>{};
  final List<_MyPage> _pages = <_MyPage>[];

  @override
  final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

  void _notifyListeners() {
    for (final VoidCallback listener in _listeners) {
      listener();
    }
  }

  void _onNavigateToLeaf() {
    _pages.clear();
    _pages.add(_MyPage.home);
    _pages.add(_MyPage.leaf);
    _notifyListeners();
  }

  void _onNavigateToHome() {
    _pages.clear();
    _pages.add(_MyPage.home);
    _notifyListeners();
  }

  void _onNavigateToNested() {
    _pages.clear();
    _pages.add(_MyPage.home);
    _pages.add(_MyPage.leaf);
    _pages.add(_MyPage.nestedNavigator);
    _notifyListeners();
  }

  @override
  void addListener(VoidCallback listener) {
    _listeners.add(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    _listeners.remove(listener);
  }

  // Called for a system back event.
  @override
  Future<bool> popRoute() {
    // If you did this manually, by removing from _pages, it wouldn't work in
    // the case where there's a nested Navigator.
    final NavigatorState navigator = navigatorKey.currentState!;
    return navigator.maybePop();
  }

  @override
  Future<void> setNewRoutePath(_MyNavigationConfiguration configuration) {
    assert(configuration.pages.length == 1);
    _pages.add(configuration.pages.first);
    _notifyListeners();
    return SynchronousFuture<void>(null);
  }

  @override
  Future<void> setRestoredRoutePath(_MyNavigationConfiguration configuration) {
    assert(_pages.isEmpty);

    _pages.addAll(configuration.pages);
    _notifyListeners();
    return SynchronousFuture<void>(null);
  }

  @override
  _MyNavigationConfiguration? get currentConfiguration {
    return _MyNavigationConfiguration(pages: _pages);
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      key: navigatorKey,
      restorationScopeId: 'root',
      onDidRemovePage: (Page<void> page) {
        final _MyPage myPage = _MyPage.fromName(page.name!);
        assert(!_pages.contains(myPage) || _pages.length > 1);
        _pages.remove(myPage);
      },
      pages: _pages.map((_MyPage myPage) => switch (myPage) {
        _MyPage.unknown => _MyUnknownPage(),
        _MyPage.home => _MyHomePage(
          onNavigateToLeaf: _onNavigateToLeaf,
        ),
        _MyPage.leaf => _MyLeafPage(
          onNavigateToHome: _onNavigateToHome,
          onNavigateToNested: _onNavigateToNested,
        ),
        _MyPage.nestedNavigator => _MyNestedNavigatorPage(
          onNavigateToRootNavigator: _onNavigateToLeaf,
        ),
      }).toList(),
    );
  }
}

class _MyUnknownPage extends MaterialPage<dynamic> {
  _MyUnknownPage() : super(
    restorationId: 'unknown-page',
    child: Scaffold(
      backgroundColor: const Color(0xff660000),
      appBar: AppBar(title: const Text('404')),
      body: const Center(
        child: Text('404'),
      ),
    ),
  );

  @override
  String get name => _MyPage.unknown.name;
}

class _MyHomePage extends MaterialPage<dynamic> {
  _MyHomePage({
    required VoidCallback onNavigateToLeaf,
  }) : super(
    restorationId: 'home-page',
    child: Scaffold(
      backgroundColor: const Color(0xffddddff),
      appBar: AppBar(title: const Text('Home Page')),
      body: Center(
        child: ElevatedButton(
          onPressed: onNavigateToLeaf,
          child: const Text('Go to leaf page'),
        ),
      ),
    ),
  );

  @override
  String get name => _MyPage.home.name;
}

class _MyLeafPage extends MaterialPage<dynamic> {
  _MyLeafPage({
    required VoidCallback onNavigateToHome,
    required VoidCallback onNavigateToNested,
  }) : super(
    restorationId: 'leaf-page',
    child: Scaffold(
      backgroundColor: const Color(0xffaaaaff),
      appBar: AppBar(title: const Text('Leaf Page')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            ElevatedButton(
              onPressed: onNavigateToNested,
              child: const Text('Go to nested Navigator page'),
            ),
            ElevatedButton(
              onPressed: onNavigateToHome,
              child: const Text('Go back to home'),
            ),
          ],
        ),
      ),
    ),
  );

  @override
  String get name => _MyPage.leaf.name;
}

class _MyNestedNavigatorPage extends MaterialPage<dynamic> {
  _MyNestedNavigatorPage({
    required VoidCallback onNavigateToRootNavigator,
  }) : super(
    restorationId: 'nested-navigator-page',
    child: _MyNestedNavigator(
      onNavigateToRootNavigator: onNavigateToRootNavigator,
    ),
  );

  @override
  String get name => _MyPage.nestedNavigator.name;
}

class _MyNestedNavigator extends StatefulWidget {
  const _MyNestedNavigator({
    required this.onNavigateToRootNavigator,
  });

  final VoidCallback onNavigateToRootNavigator;

  @override
  State<_MyNestedNavigator> createState() => _MyNestedNavigatorState();
}

class _MyNestedNavigatorState extends State<_MyNestedNavigator> with RestorationMixin {
  final _RestorablePages _pages = _RestorablePages();

  void _onNavigateToLeaf() {
    assert(!_pages.value.contains(_MyNestedPage.leaf), 'Should not ever be two leaf pages on the navigation stack.');
    setState(() {
      _pages.value = <_MyNestedPage>[
        ..._pages.value,
        _MyNestedPage.leaf,
      ];
    });
  }

  void _onNavigateToHome() {
    setState(() {
      _pages.value = <_MyNestedPage>[
        _MyNestedPage.home,
      ];
    });
  }

  // Begin RestorationMixin.

  @override
  String? get restorationId => 'nested-navigator-page';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_pages, 'pages');
  }

  /// End RestorationMixin.

  @override
  void dispose() {
    _pages.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      restorationScopeId: 'nested-navigator',
      onDidRemovePage: (Page<void> page) {
        final _MyNestedPage pageConfiguration = _MyNestedPage.fromName(page.name!);
        assert(!_pages.value.contains(pageConfiguration) || _pages.value.length > 1);
        final Iterable<_MyNestedPage> nextPages = _pages.value
            .where((_MyNestedPage nestedPageConfiguration) {
              return nestedPageConfiguration != pageConfiguration;
            });
        assert(nextPages.length == _pages.value.length - 1);
        _pages.value = nextPages;
      },
      pages: _pages.value.map((_MyNestedPage page) => switch (page) {
        _MyNestedPage.unknown => _MyUnknownPage(),
        _MyNestedPage.home => _MyNestedHomePage(
          onNavigateToLeaf: _onNavigateToLeaf,
          onNavigateToRootNavigator: widget.onNavigateToRootNavigator,
        ),
        _MyNestedPage.leaf => _MyNestedLeafPage(
          onNavigateToHome: _onNavigateToHome,
        ),
      }).toList(),
    );
  }
}

class _RestorablePages extends RestorableValue<Iterable<_MyNestedPage>> {
  @override
  List<_MyNestedPage> createDefaultValue() => <_MyNestedPage>[
    _MyNestedPage.home,
  ];

  @override
  void didUpdateValue(Iterable<_MyNestedPage>? oldValue) {
    if (oldValue == null || oldValue != value) {
      notifyListeners();
    }
  }

  @override
  List<_MyNestedPage> fromPrimitives(Object? data) {
    if (data != null) {
      final String dataString = data as String;
      final List<String> listOfStrings = dataString.split(',');
      return listOfStrings.map((String nestedPageConfigurationName) {
        return _MyNestedPage.values
            .firstWhere((_MyNestedPage nestedPageConfiguration) {
              return nestedPageConfigurationName == nestedPageConfiguration.name;
            });
      }).toList();
    }
    return <_MyNestedPage>[];
  }

  @override
  Object toPrimitives() {
    return value
        .map((_MyNestedPage nestedPageConfiguration) => nestedPageConfiguration.name)
        .join(',');
  }
}

class _MyNestedHomePage extends MaterialPage<dynamic> {
  _MyNestedHomePage({
    required VoidCallback onNavigateToLeaf,
    required VoidCallback onNavigateToRootNavigator,
  }) : super(
    restorationId: 'nested-home-page',
    child: Scaffold(
      backgroundColor: const Color(0xffddffdd),
      appBar: AppBar(title: const Text('Nested Home Page')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            ElevatedButton(
              onPressed: onNavigateToLeaf,
              child: const Text('Go to nested leaf page'),
            ),
            ElevatedButton(
              onPressed: onNavigateToRootNavigator,
              child: const Text('Go back to root Navigator'),
            ),
          ],
        ),
      ),
    ),
  );

  @override
  String get name => _MyPage.home.name;
}

class _MyNestedLeafPage extends MaterialPage<dynamic> {
  _MyNestedLeafPage({
    required VoidCallback onNavigateToHome,
  }) : super(
    restorationId: 'nested-leaf-page',
    child: Scaffold(
      backgroundColor: const Color(0xffaaffaa),
      appBar: AppBar(title: const Text('Nested Leaf Page')),
      body: Center(
        child: ElevatedButton(
          onPressed: onNavigateToHome,
          child: const Text('Go back to nested home'),
        ),
      ),
    ),
  );

  @override
  String get name => _MyPage.leaf.name;
}

class _MyNavigationConfiguration {
  _MyNavigationConfiguration({
    required this.pages,
  }) : assert(pages.isNotEmpty);

  factory _MyNavigationConfiguration.single(
    _MyPage page,
  ) {
    return _MyNavigationConfiguration(pages: <_MyPage>[page]);
  }

  final List<_MyPage> pages;

  String  get uriString => pages.first.uriString;
  Uri get uri => pages.first.uri;

  @override
  String toString() {
    return '_MyNavigationConfiguration(pages: $pages)';
  }
}

enum _MyPage {
  home(uriString: '/'),
  leaf(uriString: '/leaf'),
  nestedNavigator(uriString: '/nested_navigator'),
  unknown(uriString: '/404');

  const _MyPage({
    required this.uriString,
  });

  final String uriString;

  static _MyPage fromName(String testName) {
    return values.firstWhere((_MyPage page) => page.name == testName);
  }


  Uri get uri => Uri.parse(uriString);

  @override
  String toString() => '_MyPage($name)';
}

enum _MyNestedPage {
  home(uriString: '/'),
  leaf(uriString: '/leaf'),
  unknown(uriString: '/404');

  const _MyNestedPage({
    required this.uriString,
  });

  final String uriString;

  static _MyNestedPage fromName(String testName) {
    return values.firstWhere((_MyNestedPage page) => page.name == testName);
  }

  Uri get uri => Uri.parse(uriString);

  @override
  String toString() => '_MyNestedPage($name)';
}
