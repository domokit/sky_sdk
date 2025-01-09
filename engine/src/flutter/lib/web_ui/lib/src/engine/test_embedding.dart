// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(yjbanov): this does not need to be in the production sources.
//                https://github.com/flutter/flutter/issues/100394

import 'dart:async';

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

const bool _debugLogHistoryActions = false;

class TestHistoryEntry {
  const TestHistoryEntry(this.state, this.title, this.url);

  final dynamic state;
  final String? title;
  final String url;

  @override
  String toString() {
    return '$runtimeType(state:$state, title:"$title", url:"$url")';
  }
}

/// This URL strategy mimics the browser's history as closely as possible
/// while doing it all in memory with no interaction with the browser.
///
/// It keeps a list of history entries and event listeners in memory and
/// manipulates them in order to achieve the desired functionality.
class TestUrlStrategy implements ui_web.UrlStrategy {
  /// Creates a instance of [TestUrlStrategy] with an empty string as the
  /// path.
  factory TestUrlStrategy() => TestUrlStrategy.fromEntry(const TestHistoryEntry(null, null, ''));

  /// Creates an instance of [TestUrlStrategy] and populates it with a list
  /// that has [initialEntry] as the only item.
  TestUrlStrategy.fromEntry(TestHistoryEntry initialEntry)
    : _currentEntryIndex = 0,
      history = <TestHistoryEntry>[initialEntry];

  @override
  String getPath() => currentEntry.url;

  @override
  dynamic getState() => currentEntry.state;

  int _currentEntryIndex;
  int get currentEntryIndex => _currentEntryIndex;

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

    if (_debugLogHistoryActions) {
      print('$runtimeType.pushState(...) -> $this');
    }
  }

  @override
  void replaceState(dynamic state, String title, String? url) {
    assert(withinAppHistory);
    if (url == null || url == '') {
      url = currentEntry.url;
    }
    currentEntry = TestHistoryEntry(state, title, url);

    if (_debugLogHistoryActions) {
      print('$runtimeType.replaceState(...) -> $this');
    }
  }

  /// This simulates the case where a user types in a url manually. It causes
  /// a new state to be pushed, and all event listeners will be invoked.
  Future<void> simulateUserTypingUrl(String url) {
    assert(withinAppHistory);
    return _nextEventLoop(() {
      pushState(null, '', url);
      _firePopStateEvent();
    });
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

      if (_debugLogHistoryActions) {
        print('$runtimeType.back() -> $this');
      }
    });
  }

  final List<ui_web.PopStateListener> listeners = <ui_web.PopStateListener>[];

  @override
  ui.VoidCallback addPopStateListener(ui_web.PopStateListener fn) {
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
    for (int i = 0; i < listeners.length; i++) {
      listeners[i](currentEntry.state);
    }

    if (_debugLogHistoryActions) {
      print('$runtimeType: fired popstate with state ${currentEntry.state}');
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
