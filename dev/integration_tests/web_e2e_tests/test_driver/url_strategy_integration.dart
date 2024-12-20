// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:ui' as ui;
import 'dart:ui_web';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_web_plugins/flutter_web_plugins.dart';
import 'package:integration_test/integration_test.dart';
import 'package:web/helpers.dart';
import 'package:web/web.dart' as web;
import 'package:web_e2e_tests/url_strategy_main.dart' as app;

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('Can customize url strategy', (WidgetTester tester) async {
    final TestUrlStrategy strategy = TestUrlStrategy.fromEntry(
      const TestHistoryEntry('initial state', null, '/'),
    );
    setUrlStrategy(strategy);

    app.appRoutes = <String, WidgetBuilder>{
      '/': (BuildContext context) => Container(),
      '/foo': (BuildContext context) => Container(),
    };
    app.main();
    await tester.pumpAndSettle();

    // checking whether the previously set strategy is properly preserved
    expect(urlStrategy, strategy);

    expect(strategy.getPath(), '/');

    final NavigatorState navigator = app.navKey.currentState!;
    navigator.pushNamed('/foo');
    await tester.pump();
    expect(strategy.getPath(), '/foo');
  });
}

/// This URL strategy mimics the browser's history as closely as possible
/// while doing it all in memory with no interaction with the browser.
///
/// It keeps a list of history entries and event listeners in memory and
/// manipulates them in order to achieve the desired functionality.
class TestUrlStrategy extends UrlStrategy {
  /// Creates an instance of [TestUrlStrategy] and populates it with a list
  /// that has [initialEntry] as the only item.
  TestUrlStrategy.fromEntry(TestHistoryEntry initialEntry)
    : _currentEntryIndex = 0,
      history = <TestHistoryEntry>[initialEntry];

  @override
  String getPath() => currentEntry.url;

  @override
  Object? getState() => currentEntry.state;

  int _currentEntryIndex;
  final List<TestHistoryEntry> history;

  TestHistoryEntry get currentEntry {
    assert(withinAppHistory);
    return history[_currentEntryIndex];
  }

  set currentEntry(TestHistoryEntry entry) {
    assert(withinAppHistory);
    history[_currentEntryIndex] = entry;
  }

  /// Whether we are still within the history of the Flutter Web app. This
  /// remains true until we go back in history beyond the entry where the app
  /// started.
  bool get withinAppHistory => _currentEntryIndex >= 0;

  @override
  void pushState(dynamic state, String title, String url) {
    assert(withinAppHistory);
    _currentEntryIndex++;
    // When pushing a new state, we need to remove all entries that exist after
    // the current entry.
    //
    // If the user goes A -> B -> C -> D, then goes back to B and pushes a new
    // entry called E, we should end up with: A -> B -> E in the history list.
    history.removeRange(_currentEntryIndex, history.length);
    history.add(TestHistoryEntry(state, title, url));
  }

  @override
  void replaceState(dynamic state, String title, String url) {
    assert(withinAppHistory);
    if (url == '') {
      url = currentEntry.url;
    }
    currentEntry = TestHistoryEntry(state, title, url);
  }

  @override
  Future<void> go(int count) {
    assert(withinAppHistory);
    // Browsers don't move in history immediately. They do it at the next
    // event loop. So let's simulate that.
    return _nextEventLoop(() {
      _currentEntryIndex = _currentEntryIndex + count;
      if (withinAppHistory) {
        _firePopStateEvent();
      }
    });
  }

  final List<PopStateListener> listeners = <PopStateListener>[];

  @override
  ui.VoidCallback addPopStateListener(PopStateListener fn) {
    listeners.add(fn);
    return () {
      // Schedule a micro task here to avoid removing the listener during
      // iteration in [_firePopStateEvent].
      scheduleMicrotask(() => listeners.remove(fn));
    };
  }

  /// Simulates the scheduling of a new event loop by creating a delayed future.
  /// Details explained here: https://webdev.dartlang.org/articles/performance/event-loop
  Future<void> _nextEventLoop(ui.VoidCallback callback) {
    return Future<void>.delayed(Duration.zero).then((_) => callback());
  }

  /// Invokes all the attached event listeners in order of
  /// attaching. This method should be called asynchronously to make it behave
  /// like a real browser.
  void _firePopStateEvent() {
    assert(withinAppHistory);
    final web.PopStateEvent event = web.PopStateEvent(
      'popstate',
      PopStateEventInit(state: currentEntry.state?.toJSBox),
    );
    for (int i = 0; i < listeners.length; i++) {
      listeners[i](event);
    }
  }

  @override
  String prepareExternalUrl(String internalUrl) => internalUrl;

  @override
  String toString() {
    final List<String> lines = <String>[];
    for (int i = 0; i < history.length; i++) {
      final TestHistoryEntry entry = history[i];
      lines.add(_currentEntryIndex == i ? '* $entry' : '  $entry');
    }
    return '$runtimeType: [\n${lines.join('\n')}\n]';
  }
}

class TestHistoryEntry {
  const TestHistoryEntry(this.state, this.title, this.url);

  final Object? state;
  final String? title;
  final String url;

  @override
  String toString() {
    return '$runtimeType(state:$state, title:"$title", url:"$url")';
  }
}
