// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Router state restoration without RouteInformationProvider', (
    WidgetTester tester,
  ) async {
    final UniqueKey router = UniqueKey();
    _TestRouterDelegate delegate() =>
        tester.widget<Router<Object?>>(find.byKey(router)).routerDelegate as _TestRouterDelegate;

    await tester.pumpWidget(_TestWidget(routerKey: router));
    expect(find.text('Current config: null'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, isEmpty);

    delegate().currentConfiguration = '/foo';
    await tester.pumpAndSettle();
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, isEmpty);

    await tester.restartAndRestore();
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, <String>['/foo']);

    final TestRestorationData restorationData = await tester.getRestorationData();

    delegate().currentConfiguration = '/bar';
    await tester.pumpAndSettle();
    expect(find.text('Current config: /bar'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, <String>['/foo']);

    await tester.restoreFrom(restorationData);
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, <String>['/foo', '/foo']);
  });

  testWidgets('Router state restoration with RouteInformationProvider', (
    WidgetTester tester,
  ) async {
    final UniqueKey router = UniqueKey();
    _TestRouterDelegate delegate() =>
        tester.widget<Router<Object?>>(find.byKey(router)).routerDelegate as _TestRouterDelegate;
    _TestRouteInformationProvider provider() =>
        tester.widget<Router<Object?>>(find.byKey(router)).routeInformationProvider!
            as _TestRouteInformationProvider;

    await tester.pumpWidget(_TestWidget(routerKey: router, withInformationProvider: true));
    expect(find.text('Current config: /home'), findsOneWidget);
    expect(delegate().newRoutePaths, <String>['/home']);
    expect(delegate().restoredRoutePaths, isEmpty);

    provider().value = RouteInformation(uri: Uri(path: '/foo'));
    await tester.pumpAndSettle();
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, <String>['/home', '/foo']);
    expect(delegate().restoredRoutePaths, isEmpty);

    await tester.restartAndRestore();
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, isEmpty);
    expect(delegate().restoredRoutePaths, <String>['/foo']);

    final TestRestorationData restorationData = await tester.getRestorationData();

    provider().value = RouteInformation(uri: Uri.parse('/bar'));
    await tester.pumpAndSettle();
    expect(find.text('Current config: /bar'), findsOneWidget);
    expect(delegate().newRoutePaths, <String>['/bar']);
    expect(delegate().restoredRoutePaths, <String>['/foo']);

    await tester.restoreFrom(restorationData);
    expect(find.text('Current config: /foo'), findsOneWidget);
    expect(delegate().newRoutePaths, <String>['/bar']);
    expect(delegate().restoredRoutePaths, <String>['/foo', '/foo']);
  });
}

class _TestRouteInformationParser extends RouteInformationParser<String> {
  @override
  Future<String> parseRouteInformation(RouteInformation routeInformation) {
    return SynchronousFuture<String>(routeInformation.uri.toString());
  }

  @override
  RouteInformation? restoreRouteInformation(String configuration) {
    return RouteInformation(uri: Uri.parse(configuration));
  }
}

class _TestRouterDelegate extends RouterDelegate<String> with ChangeNotifier {
  _TestRouterDelegate() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  final List<String> newRoutePaths = <String>[];
  final List<String> restoredRoutePaths = <String>[];

  @override
  String? get currentConfiguration => _currentConfiguration;
  String? _currentConfiguration;
  set currentConfiguration(String? value) {
    if (value == _currentConfiguration) {
      return;
    }
    _currentConfiguration = value;
    notifyListeners();
  }

  @override
  Future<void> setNewRoutePath(String configuration) {
    _currentConfiguration = configuration;
    newRoutePaths.add(configuration);
    return SynchronousFuture<void>(null);
  }

  @override
  Future<void> setRestoredRoutePath(String configuration) {
    _currentConfiguration = configuration;
    restoredRoutePaths.add(configuration);
    return SynchronousFuture<void>(null);
  }

  @override
  Widget build(BuildContext context) {
    return Text('Current config: $currentConfiguration', textDirection: TextDirection.ltr);
  }

  @override
  Future<bool> popRoute() async => throw UnimplementedError();
}

class _TestRouteInformationProvider extends RouteInformationProvider with ChangeNotifier {
  _TestRouteInformationProvider() {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  @override
  RouteInformation get value => _value;
  RouteInformation _value = RouteInformation(uri: Uri.parse('/home'));
  set value(RouteInformation value) {
    if (value == _value) {
      return;
    }
    _value = value;
    notifyListeners();
  }
}

class _TestWidget extends StatefulWidget {
  const _TestWidget({this.withInformationProvider = false, this.routerKey});

  final bool withInformationProvider;
  final Key? routerKey;

  @override
  State<_TestWidget> createState() => _TestWidgetState();
}

class _TestWidgetState extends State<_TestWidget> {
  final _TestRouterDelegate _delegate = _TestRouterDelegate();
  final _TestRouteInformationProvider _routeInformationProvider = _TestRouteInformationProvider();

  @override
  void dispose() {
    _delegate.dispose();
    _routeInformationProvider.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return RootRestorationScope(
      restorationId: 'root',
      child: Router<String>(
        key: widget.routerKey,
        restorationScopeId: 'router',
        routerDelegate: _delegate,
        routeInformationParser: _TestRouteInformationParser(),
        routeInformationProvider: widget.withInformationProvider ? _routeInformationProvider : null,
      ),
    );
  }
}
