// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:quiver/testing/async.dart';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart';

import '../common/matchers.dart';
import '../common/spy.dart';
import '../common/test_initialization.dart';

EngineFlutterWindow get implicitView => EnginePlatformDispatcher.instance.implicitView!;

Map<String, dynamic> _wrapOriginState(dynamic state) {
  return <String, dynamic>{'origin': true, 'state': state};
}

Map<String, dynamic> _tagStateWithSerialCount(dynamic state, int serialCount) {
  return <String, dynamic>{'serialCount': serialCount, 'state': state};
}

const Map<String, bool> flutterState = <String, bool>{'flutter': true};

const MethodCodec codec = JSONMethodCodec();

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpImplicitView();

  test('createHistoryForExistingState', () {
    TestUrlStrategy strategy;
    BrowserHistory history;

    // No url strategy.
    history = createHistoryForExistingState(null);
    expect(history, isA<MultiEntriesBrowserHistory>());
    expect(history.urlStrategy, isNull);

    // Random history state.
    strategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry(<dynamic, dynamic>{'foo': 123.0}, null, '/'),
    );
    history = createHistoryForExistingState(strategy);
    expect(history, isA<MultiEntriesBrowserHistory>());
    expect(history.urlStrategy, strategy);

    // Multi-entry history state.
    final Map<dynamic, dynamic> state = <dynamic, dynamic>{
      'serialCount': 1.0,
      'state': <dynamic, dynamic>{'foo': 123.0},
    };
    strategy = TestUrlStrategy.fromEntry(TestHistoryEntry(state, null, '/'));
    history = createHistoryForExistingState(strategy);
    expect(history, isA<MultiEntriesBrowserHistory>());
    expect(history.urlStrategy, strategy);

    // Single-entry history "origin" state.
    strategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry(<dynamic, dynamic>{'origin': true}, null, '/'),
    );
    history = createHistoryForExistingState(strategy);
    expect(history, isA<SingleEntryBrowserHistory>());
    expect(history.urlStrategy, strategy);

    // Single-entry history "flutter" state.
    strategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry(<dynamic, dynamic>{'flutter': true}, null, '/'),
    );
    history = createHistoryForExistingState(strategy);
    expect(history, isA<SingleEntryBrowserHistory>());
    expect(history.urlStrategy, strategy);
  });

  group('$SingleEntryBrowserHistory', () {
    final PlatformMessagesSpy spy = PlatformMessagesSpy();

    setUp(() async {
      spy.setUp();
    });

    tearDown(() async {
      spy.tearDown();
      await implicitView.resetHistory();
    });

    test('basic setup works', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/initial'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      // There should be two entries: origin and flutter.
      expect(strategy.history, hasLength(2));

      // The origin entry is set up but its path should remain unchanged.
      final TestHistoryEntry originEntry = strategy.history[0];
      expect(originEntry.state, _wrapOriginState('initial state'));
      expect(originEntry.url, '/initial');

      // The flutter entry is pushed and its path should be derived from the
      // origin entry.
      final TestHistoryEntry flutterEntry = strategy.history[1];
      expect(flutterEntry.state, flutterState);
      expect(flutterEntry.url, '/initial');

      // The flutter entry is the current entry.
      expect(strategy.currentEntry, flutterEntry);
    });

    test('disposes of its listener without touching history', () async {
      const String unwrappedOriginState = 'initial state';
      final Map<String, dynamic> wrappedOriginState = _wrapOriginState(unwrappedOriginState);

      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(unwrappedOriginState, null, '/initial'),
      );
      expect(strategy.listeners, isEmpty);

      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      // There should be one `popstate` listener and two history entries.
      expect(strategy.listeners, hasLength(1));
      expect(strategy.history, hasLength(2));
      expect(strategy.history[0].state, wrappedOriginState);
      expect(strategy.history[0].url, '/initial');
      expect(strategy.history[1].state, flutterState);
      expect(strategy.history[1].url, '/initial');

      FakeAsync().run((FakeAsync fakeAsync) {
        implicitView.browserHistory.dispose();
        // The `TestUrlStrategy` implementation uses microtasks to schedule the
        // removal of event listeners.
        fakeAsync.flushMicrotasks();
      });

      // After disposing, there should no listeners, and the history entries
      // remain unaffected.
      expect(strategy.listeners, isEmpty);
      expect(strategy.history, hasLength(2));
      expect(strategy.history[0].state, wrappedOriginState);
      expect(strategy.history[0].url, '/initial');
      expect(strategy.history[1].state, flutterState);
      expect(strategy.history[1].url, '/initial');

      // An extra call to dispose should be safe.
      FakeAsync().run((FakeAsync fakeAsync) {
        expect(() => implicitView.browserHistory.dispose(), returnsNormally);
        fakeAsync.flushMicrotasks();
      });

      // Same expectations should remain true after the second dispose.
      expect(strategy.listeners, isEmpty);
      expect(strategy.history, hasLength(2));
      expect(strategy.history[0].state, wrappedOriginState);
      expect(strategy.history[0].url, '/initial');
      expect(strategy.history[1].state, flutterState);
      expect(strategy.history[1].url, '/initial');

      // Can still teardown after being disposed.
      await implicitView.browserHistory.tearDown();
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntry.state, unwrappedOriginState);
      expect(strategy.currentEntry.url, '/initial');
    });

    test('disposes gracefully when url strategy is null', () async {
      await implicitView.debugInitializeHistory(null, useSingle: true);
      expect(() => implicitView.browserHistory.dispose(), returnsNormally);
    });

    test('browser back button pops routes correctly', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(null, null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      // Initially, we should be on the flutter entry.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/home');
      await routeUpdated('/page1');
      // The number of entries shouldn't change.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      // But the url of the current entry (flutter entry) should be updated.
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/page1');

      // No platform messages have been sent so far.
      expect(spy.messages, isEmpty);
      // Clicking back should take us to page1.
      await strategy.go(-1);
      // First, the framework should've received a `popRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'popRoute');
      expect(spy.messages[0].methodArguments, isNull);
      // We still have 2 entries.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      // The url of the current entry (flutter entry) should go back to /home.
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/home');
    });

    test('multiple browser back clicks', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(null, null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      await routeUpdated('/page1');
      await routeUpdated('/page2');

      // Make sure we are on page2.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/page2');

      // Back to page1.
      await strategy.go(-1);
      // 1. The engine sends a `popRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'popRoute');
      expect(spy.messages[0].methodArguments, isNull);
      spy.messages.clear();
      // 2. The framework sends a `routePopped` platform message.
      await routeUpdated('/page1');
      // 3. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/page1');

      // Back to home.
      await strategy.go(-1);
      // 1. The engine sends a `popRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'popRoute');
      expect(spy.messages[0].methodArguments, isNull);
      spy.messages.clear();
      // 2. The framework sends a `routePopped` platform message.
      await routeUpdated('/home');
      // 3. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/home');

      // The next browser back will exit the app. We store the strategy locally
      // because it will be remove from the browser history class once it exits
      // the app.
      final TestUrlStrategy originalStrategy = strategy;
      await originalStrategy.go(-1);
      // 1. The engine sends a `popRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'popRoute');
      expect(spy.messages[0].methodArguments, isNull);
      spy.messages.clear();
      // 2. The framework sends a `SystemNavigator.pop` platform message
      // because there are no more routes to pop.
      await systemNavigatorPop();
      // 3. The active entry doesn't belong to our history anymore because we
      // navigated past it.
      expect(originalStrategy.currentEntryIndex, -1);
    });

    test('handle user-provided url', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(null, null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      await strategy.simulateUserTypingUrl('/page3');
      // This delay is necessary to wait for [BrowserHistory] because it
      // performs a `back` operation which results in a new event loop.
      await Future<void>.delayed(Duration.zero);
      // 1. The engine sends a `pushRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRoute');
      expect(spy.messages[0].methodArguments, '/page3');
      spy.messages.clear();
      // 2. The framework sends a `routePushed` platform message.
      await routeUpdated('/page3');
      // 3. The history state should reflect that /page3 is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/page3');

      // Back to home.
      await strategy.go(-1);
      // 1. The engine sends a `popRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'popRoute');
      expect(spy.messages[0].methodArguments, isNull);
      spy.messages.clear();
      // 2. The framework sends a `routePopped` platform message.
      await routeUpdated('/home');
      // 3. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/home');
    });

    test('user types unknown url', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(null, null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: true);

      await strategy.simulateUserTypingUrl('/unknown');
      // This delay is necessary to wait for [BrowserHistory] because it
      // performs a `back` operation which results in a new event loop.
      await Future<void>.delayed(Duration.zero);
      // 1. The engine sends a `pushRoute` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRoute');
      expect(spy.messages[0].methodArguments, '/unknown');
      spy.messages.clear();
      // 2. The framework doesn't recognize the route name and ignores it.
      // 3. The history state should reflect that /home is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, flutterState);
      expect(strategy.currentEntry.url, '/home');
    });
  });

  group('$MultiEntriesBrowserHistory', () {
    final PlatformMessagesSpy spy = PlatformMessagesSpy();

    setUp(() async {
      spy.setUp();
    });

    tearDown(() async {
      spy.tearDown();
      await implicitView.resetHistory();
    });

    test('basic setup works', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/initial'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      // There should be only one entry.
      expect(strategy.history, hasLength(1));

      // The origin entry is tagged and its path should remain unchanged.
      final TestHistoryEntry taggedOriginEntry = strategy.history[0];
      expect(taggedOriginEntry.state, _tagStateWithSerialCount('initial state', 0));
      expect(taggedOriginEntry.url, '/initial');
    });

    test('disposes of its listener without touching history', () async {
      const String untaggedState = 'initial state';
      final Map<String, dynamic> taggedState = _tagStateWithSerialCount(untaggedState, 0);

      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry(untaggedState, null, '/initial'),
      );
      expect(strategy.listeners, isEmpty);

      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      // There should be one `popstate` listener and one history entry.
      expect(strategy.listeners, hasLength(1));
      expect(strategy.history, hasLength(1));
      expect(strategy.history.single.state, taggedState);
      expect(strategy.history.single.url, '/initial');

      FakeAsync().run((FakeAsync fakeAsync) {
        implicitView.browserHistory.dispose();
        // The `TestUrlStrategy` implementation uses microtasks to schedule the
        // removal of event listeners.
        fakeAsync.flushMicrotasks();
      });

      // After disposing, there should no listeners, and the history entries
      // remain unaffected.
      expect(strategy.listeners, isEmpty);
      expect(strategy.history, hasLength(1));
      expect(strategy.history.single.state, taggedState);
      expect(strategy.history.single.url, '/initial');

      // An extra call to dispose should be safe.
      FakeAsync().run((FakeAsync fakeAsync) {
        expect(() => implicitView.browserHistory.dispose(), returnsNormally);
        fakeAsync.flushMicrotasks();
      });

      // Same expectations should remain true after the second dispose.
      expect(strategy.listeners, isEmpty);
      expect(strategy.history, hasLength(1));
      expect(strategy.history.single.state, taggedState);
      expect(strategy.history.single.url, '/initial');

      // Can still teardown after being disposed.
      await implicitView.browserHistory.tearDown();
      expect(strategy.history, hasLength(1));
      expect(strategy.history.single.state, untaggedState);
      expect(strategy.history.single.url, '/initial');
    });

    test('disposes gracefully when url strategy is null', () async {
      await implicitView.debugInitializeHistory(null, useSingle: false);
      expect(() => implicitView.browserHistory.dispose(), returnsNormally);
    });

    test('browser back button push route information correctly', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      // Initially, we should be on the flutter entry.
      expect(strategy.history, hasLength(1));
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('initial state', 0));
      expect(strategy.currentEntry.url, '/home');
      await routeInformationUpdated('/page1', 'page1 state');
      // Should have two history entries now.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      // But the url of the current entry (flutter entry) should be updated.
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page1 state', 1));
      expect(strategy.currentEntry.url, '/page1');

      // No platform messages have been sent so far.
      expect(spy.messages, isEmpty);
      // Clicking back should take us to page1.
      await strategy.go(-1);
      // First, the framework should've received a `pushRouteInformation`
      // platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/home',
        'state': 'initial state',
      });
      // There are still two browser history entries, but we are back to the
      // original state.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 0);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('initial state', 0));
      expect(strategy.currentEntry.url, '/home');
    });

    test('multiple browser back clicks', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      await routeInformationUpdated('/page1', 'page1 state');
      await routeInformationUpdated('/page2', 'page2 state');

      // Make sure we are on page2.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 2);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page2 state', 2));
      expect(strategy.currentEntry.url, '/page2');

      // Back to page1.
      await strategy.go(-1);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/page1',
        'state': 'page1 state',
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page1 state', 1));
      expect(strategy.currentEntry.url, '/page1');
      // Back to home.
      await strategy.go(-1);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/home',
        'state': 'initial state',
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 0);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('initial state', 0));
      expect(strategy.currentEntry.url, '/home');
    });

    test('handle user-provided url', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      await strategy.simulateUserTypingUrl('/page3');
      // This delay is necessary to wait for [BrowserHistory] because it
      // performs a `back` operation which results in a new event loop.
      await Future<void>.delayed(Duration.zero);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/page3',
        'state': null,
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page3 is currently active.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount(null, 1));
      expect(strategy.currentEntry.url, '/page3');

      // Back to home.
      await strategy.go(-1);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/home',
        'state': 'initial state',
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(2));
      expect(strategy.currentEntryIndex, 0);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('initial state', 0));
      expect(strategy.currentEntry.url, '/home');
    });

    test('forward button works', () async {
      final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
        const TestHistoryEntry('initial state', null, '/home'),
      );
      await implicitView.debugInitializeHistory(strategy, useSingle: false);

      await routeInformationUpdated('/page1', 'page1 state');
      await routeInformationUpdated('/page2', 'page2 state');

      // Make sure we are on page2.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 2);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page2 state', 2));
      expect(strategy.currentEntry.url, '/page2');

      // Back to page1.
      await strategy.go(-1);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/page1',
        'state': 'page1 state',
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page1 is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 1);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page1 state', 1));
      expect(strategy.currentEntry.url, '/page1');

      // Forward to page2
      await strategy.go(1);
      // 1. The engine sends a `pushRouteInformation` platform message.
      expect(spy.messages, hasLength(1));
      expect(spy.messages[0].channel, 'flutter/navigation');
      expect(spy.messages[0].methodName, 'pushRouteInformation');
      expect(spy.messages[0].methodArguments, <dynamic, dynamic>{
        'location': '/page2',
        'state': 'page2 state',
      });
      spy.messages.clear();
      // 2. The history state should reflect that /page2 is currently active.
      expect(strategy.history, hasLength(3));
      expect(strategy.currentEntryIndex, 2);
      expect(strategy.currentEntry.state, _tagStateWithSerialCount('page2 state', 2));
      expect(strategy.currentEntry.url, '/page2');
    });
  });

  group('$HashUrlStrategy', () {
    late TestPlatformLocation location;

    setUp(() {
      location = TestPlatformLocation();
    });

    tearDown(() {
      location = TestPlatformLocation();
    });

    test('leading slash is optional', () {
      final HashUrlStrategy strategy = HashUrlStrategy(location);

      location.hash = '#/';
      expect(strategy.getPath(), '/');

      location.hash = '#/foo';
      expect(strategy.getPath(), '/foo');

      location.hash = '#foo';
      expect(strategy.getPath(), 'foo');
    });

    test('path should not be empty', () {
      final HashUrlStrategy strategy = HashUrlStrategy(location);

      location.hash = '';
      expect(strategy.getPath(), '/');

      location.hash = '#';
      expect(strategy.getPath(), '/');
    });

    test('prepareExternalUrl', () {
      const String internalUrl = '/menu?foo=bar';
      final HashUrlStrategy strategy = HashUrlStrategy(location);

      location.pathname = '/';
      expect(strategy.prepareExternalUrl(internalUrl), '/#/menu?foo=bar');

      location.pathname = '/main';
      expect(strategy.prepareExternalUrl(internalUrl), '/main#/menu?foo=bar');

      location.search = '?foo=bar';
      expect(strategy.prepareExternalUrl(internalUrl), '/main?foo=bar#/menu?foo=bar');
    });

    test('removes /#/ from the home page', () {
      const String internalUrl = '/';
      final HashUrlStrategy strategy = HashUrlStrategy(location);

      location.pathname = '/';
      expect(strategy.prepareExternalUrl(internalUrl), '/');

      location.pathname = '/main';
      expect(strategy.prepareExternalUrl(internalUrl), '/main');

      location.search = '?foo=bar';
      expect(strategy.prepareExternalUrl(internalUrl), '/main?foo=bar');
    });

    test('addPopStateListener fn unwraps DomPopStateEvent state', () {
      final HashUrlStrategy strategy = HashUrlStrategy(location);
      const String expected = 'expected value';
      final List<Object?> states = <Object?>[];

      // Put the popStates received from the `location` in a list
      strategy.addPopStateListener(states.add);

      // Simulate a popstate with a null state:
      location.debugTriggerPopState(null);

      expect(states, hasLength(1));
      expect(states[0], isNull);

      // Simulate a popstate event with `expected` as its 'state'.
      location.debugTriggerPopState(expected);

      expect(states, hasLength(2));
      final Object? state = states[1];
      expect(state, isNotNull);
      // flutter/flutter#125228
      expect(state, isNot(isA<DomEvent>()));
      expect(state, expected);
    });
  });

  group('$BrowserPlatformLocation', () {
    test('getOrCreateDomEventListener caches funcions', () {
      const BrowserPlatformLocation location = BrowserPlatformLocation();
      void myListener(Object event) {}

      expect(
        identical(
          location.getOrCreateDomEventListener(myListener),
          location.getOrCreateDomEventListener(myListener),
        ),
        isTrue,
      );
    });

    test('throws if removing an invalid listener', () {
      const BrowserPlatformLocation location = BrowserPlatformLocation();
      void myAddedListener(Object event) {}
      void myNonAddedListener(Object event) {}

      location.addPopStateListener(myAddedListener);
      expect(() => location.removePopStateListener(myAddedListener), returnsNormally);
      // Removing the same listener twice should throw.
      expect(() => location.removePopStateListener(myAddedListener), throwsAssertionError);

      // A listener that was never added.
      expect(() => location.removePopStateListener(myNonAddedListener), throwsAssertionError);
    });

    test('returns a non-empty baseUri', () {
      const BrowserPlatformLocation location = BrowserPlatformLocation();
      expect(location.getBaseHref(), isNotNull);
    });
  });
}

Future<void> routeUpdated(String routeName) {
  final Completer<void> completer = Completer<void>();
  EnginePlatformDispatcher.instance.sendPlatformMessage(
    'flutter/navigation',
    codec.encodeMethodCall(MethodCall('routeUpdated', <String, dynamic>{'routeName': routeName})),
    (_) => completer.complete(),
  );
  return completer.future;
}

Future<void> routeInformationUpdated(String location, dynamic state) {
  final Completer<void> completer = Completer<void>();
  EnginePlatformDispatcher.instance.sendPlatformMessage(
    'flutter/navigation',
    codec.encodeMethodCall(
      MethodCall('routeInformationUpdated', <String, dynamic>{
        'location': location,
        'state': state,
      }),
    ),
    (_) => completer.complete(),
  );
  return completer.future;
}

Future<void> systemNavigatorPop() {
  final Completer<void> completer = Completer<void>();
  EnginePlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform',
    codec.encodeMethodCall(const MethodCall('SystemNavigator.pop')),
    (_) => completer.complete(),
  );
  return completer.future;
}

/// A mock implementation of [PlatformLocation] that doesn't access the browser.
class TestPlatformLocation implements PlatformLocation {
  @override
  String? hash;

  @override
  dynamic state;

  List<EventListener> popStateListeners = <EventListener>[];

  @override
  String pathname = '';

  @override
  String search = '';

  /// Calls all the registered `popStateListeners` with a 'popstate'
  /// event with value `state`
  void debugTriggerPopState(Object? state) {
    final DomEvent event = createDomPopStateEvent('popstate', <Object, Object>{
      if (state != null) 'state': state,
    });
    for (final EventListener listener in popStateListeners) {
      listener(event);
    }
  }

  @override
  void addPopStateListener(EventListener fn) {
    popStateListeners.add(fn);
  }

  @override
  void removePopStateListener(EventListener fn) {
    throw UnimplementedError();
  }

  @override
  void pushState(dynamic state, String title, String url) {
    throw UnimplementedError();
  }

  @override
  void replaceState(dynamic state, String title, String url) {
    throw UnimplementedError();
  }

  @override
  void go(int count) {
    throw UnimplementedError();
  }

  @override
  String getBaseHref() => '/';
}
