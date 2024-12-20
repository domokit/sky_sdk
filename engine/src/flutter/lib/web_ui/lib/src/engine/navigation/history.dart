// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../platform_dispatcher.dart';
import '../services/message_codec.dart';
import '../services/message_codecs.dart';

/// Infers the history mode from the existing browser history state, then
/// creates the appropriate instance of [BrowserHistory] for it.
///
/// If it can't infer, it creates a [MultiEntriesBrowserHistory] by default.
BrowserHistory createHistoryForExistingState(ui_web.UrlStrategy? urlStrategy) {
  if (urlStrategy != null) {
    final Object? state = urlStrategy.getState();
    if (SingleEntryBrowserHistory._isOriginEntry(state) ||
        SingleEntryBrowserHistory._isFlutterEntry(state)) {
      return SingleEntryBrowserHistory(urlStrategy: urlStrategy);
    }
  }
  return MultiEntriesBrowserHistory(urlStrategy: urlStrategy);
}

/// An abstract class that provides the API for [EngineWindow] to delegate its
/// navigating events.
///
/// Subclasses will have access to [BrowserHistory.locationStrategy] to
/// interact with the html browser history and should come up with their own
/// ways to manage the states in the browser history.
///
/// There should only be one global instance among all subclasses.
///
/// See also:
///
///  * [SingleEntryBrowserHistory]: which creates a single fake browser history
///    entry and delegates all browser navigating events to the flutter
///    framework.
///  * [MultiEntriesBrowserHistory]: which creates a set of states that records
///    the navigating events happened in the framework.
abstract class BrowserHistory {
  late ui.VoidCallback _unsubscribe;

  /// The strategy to interact with html browser history.
  ui_web.UrlStrategy? get urlStrategy;

  bool _isTornDown = false;
  bool _isDisposed = false;

  void _setupStrategy(ui_web.UrlStrategy strategy) {
    _unsubscribe = strategy.addPopStateListener(onPopState);
  }

  /// Release any resources held by this [BrowserHistory] instance.
  ///
  /// This method has no effect on the browser history entries. Use [tearDown]
  /// instead to revert this instance's modifications to browser history
  /// entries.
  @mustCallSuper
  void dispose() {
    if (_isDisposed || urlStrategy == null) {
      return;
    }
    _isDisposed = true;
    _unsubscribe();
  }

  /// Exit this application and return to the previous page.
  Future<void> exit() async {
    if (urlStrategy != null) {
      await tearDown();
      // Now the history should be in the original state, back one more time to
      // exit the application.
      await urlStrategy!.go(-1);
    }
  }

  /// This method does the same thing as the browser back button.
  Future<void> back() async {
    return urlStrategy?.go(-1);
  }

  /// The path of the current location of the user's browser.
  String get currentPath => urlStrategy?.getPath() ?? '/';

  /// The state of the current location of the user's browser.
  Object? get currentState => urlStrategy?.getState();

  /// Update the url with the given `routeName` and `state`.
  ///
  /// If `replace` is false, the caller wants to push a new `routeName` and
  /// `state` on top of the existing ones; otherwise, the caller wants to replace
  /// the current `routeName` and `state` with the new ones.
  void setRouteName(String? routeName, {Object? state, bool replace = false});

  /// A callback method to handle browser backward or forward buttons.
  ///
  /// Subclasses should send appropriate system messages to update the flutter
  /// applications accordingly.
  void onPopState(Object? state);

  /// Restore any modifications to the html browser history during the lifetime
  /// of this class.
  Future<void> tearDown();
}

/// A browser history class that creates a set of browser history entries to
/// support browser backward and forward button natively.
///
/// This class pushes a browser history entry every time the framework reports
/// a route change and sends a `pushRouteInformation` method call to the
/// framework when the browser jumps to a specific browser history entry.
///
/// The web engine uses this class to manage its browser history when the
/// framework uses a Router for routing.
///
/// See also:
///
/// * [SingleEntryBrowserHistory], which is used when the framework does not use
///   a Router for routing.
class MultiEntriesBrowserHistory extends BrowserHistory {
  MultiEntriesBrowserHistory({required this.urlStrategy}) {
    final ui_web.UrlStrategy? strategy = urlStrategy;
    if (strategy == null) {
      return;
    }

    _setupStrategy(strategy);
    if (!_hasSerialCount(currentState)) {
      strategy.replaceState(_tagWithSerialCount(currentState, 0), 'flutter', currentPath);
    }
    // If we restore from a page refresh, the _currentSerialCount may not be 0.
    _lastSeenSerialCount = _currentSerialCount;
  }

  @override
  final ui_web.UrlStrategy? urlStrategy;

  late int _lastSeenSerialCount;
  int get _currentSerialCount {
    if (_hasSerialCount(currentState)) {
      final Map<dynamic, dynamic> stateMap = currentState! as Map<dynamic, dynamic>;
      return (stateMap['serialCount'] as double).toInt();
    }
    return 0;
  }

  Object _tagWithSerialCount(Object? originialState, int count) {
    return <dynamic, dynamic>{'serialCount': count.toDouble(), 'state': originialState};
  }

  bool _hasSerialCount(Object? state) {
    return state is Map && state['serialCount'] != null;
  }

  @override
  void setRouteName(String? routeName, {Object? state, bool replace = false}) {
    if (urlStrategy != null) {
      assert(routeName != null);
      if (replace) {
        urlStrategy!.replaceState(
          _tagWithSerialCount(state, _lastSeenSerialCount),
          'flutter',
          routeName!,
        );
      } else {
        _lastSeenSerialCount += 1;
        urlStrategy!.pushState(
          _tagWithSerialCount(state, _lastSeenSerialCount),
          'flutter',
          routeName!,
        );
      }
    }
  }

  @override
  void onPopState(Object? state) {
    assert(urlStrategy != null);
    // May be a result of direct url access while the flutter application is
    // already running.
    if (!_hasSerialCount(state)) {
      // In this case we assume this will be the next history entry from the
      // last seen entry.
      urlStrategy!.replaceState(
        _tagWithSerialCount(state, _lastSeenSerialCount + 1),
        'flutter',
        currentPath,
      );
    }
    _lastSeenSerialCount = _currentSerialCount;
    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/navigation',
      const JSONMethodCodec().encodeMethodCall(
        MethodCall('pushRouteInformation', <dynamic, dynamic>{
          'location': currentPath,
          'state': (state as Map<dynamic, dynamic>?)?['state'],
        }),
      ),
      (_) {},
    );
  }

  @override
  Future<void> tearDown() async {
    dispose();

    if (_isTornDown || urlStrategy == null) {
      return;
    }
    _isTornDown = true;

    // Restores the html browser history.
    assert(
      _hasSerialCount(currentState),
      currentState == null
          ? 'unexpected null history state'
          : "history state is missing field 'serialCount'",
    );
    final int backCount = _currentSerialCount;
    if (backCount > 0) {
      await urlStrategy!.go(-backCount);
    }
    // Unwrap state.
    assert(_hasSerialCount(currentState) && _currentSerialCount == 0);
    final Map<dynamic, dynamic> stateMap = currentState! as Map<dynamic, dynamic>;
    urlStrategy!.replaceState(stateMap['state'], 'flutter', currentPath);
  }
}

/// The browser history class is responsible for integrating Flutter Web apps
/// with the browser history so that the back button works as expected.
///
/// It does that by always keeping a single entry (conventionally called the
/// "flutter" entry) at the top of the browser history. That way, the browser's
/// back button always triggers a `popstate` event and never closes the app (we
/// close the app programmatically by calling [SystemNavigator.pop] when there
/// are no more app routes to be popped).
///
/// The web engine uses this class when the framework does not use Router for
/// routing, and it does not support browser forward button.
///
/// See also:
///
///  * [MultiEntriesBrowserHistory], which is used when the framework uses a
///    Router for routing.
class SingleEntryBrowserHistory extends BrowserHistory {
  SingleEntryBrowserHistory({required this.urlStrategy}) {
    final ui_web.UrlStrategy? strategy = urlStrategy;
    if (strategy == null) {
      return;
    }

    _setupStrategy(strategy);

    final String path = currentPath;
    if (!_isFlutterEntry(domWindow.history.state)) {
      // An entry may not have come from Flutter, for example, when the user
      // refreshes the page. They land directly on the "flutter" entry, so
      // there's no need to set up the "origin" and "flutter" entries, we can
      // safely assume they are already set up.
      _setupOriginEntry(strategy);
      _setupFlutterEntry(strategy, path: path);
    }
  }

  @override
  final ui_web.UrlStrategy? urlStrategy;

  static const MethodCall _popRouteMethodCall = MethodCall('popRoute');
  static const String _kFlutterTag = 'flutter';
  static const String _kOriginTag = 'origin';

  Map<String, dynamic> _wrapOriginState(Object? state) {
    return <String, dynamic>{_kOriginTag: true, 'state': state};
  }

  Object? _unwrapOriginState(Object? state) {
    assert(_isOriginEntry(state));
    final Map<dynamic, dynamic> originState = state! as Map<dynamic, dynamic>;
    return originState['state'];
  }

  final Map<String, bool> _flutterState = <String, bool>{_kFlutterTag: true};

  /// The origin entry is the history entry that the Flutter app landed on. It's
  /// created by the browser when the user navigates to the url of the app.
  static bool _isOriginEntry(Object? state) {
    return state is Map && state[_kOriginTag] == true;
  }

  /// The flutter entry is a history entry that we maintain on top of the origin
  /// entry. It allows us to catch popstate events when the user hits the back
  /// button.
  static bool _isFlutterEntry(Object? state) {
    return state is Map && state[_kFlutterTag] == true;
  }

  @override
  void setRouteName(String? routeName, {Object? state, bool replace = false}) {
    if (urlStrategy != null) {
      _setupFlutterEntry(urlStrategy!, replace: true, path: routeName);
    }
  }

  String? _userProvidedRouteName;
  @override
  void onPopState(Object? state) {
    if (_isOriginEntry(state)) {
      _setupFlutterEntry(urlStrategy!);

      // 2. Send a 'popRoute' platform message so the app can handle it accordingly.
      EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
        'flutter/navigation',
        const JSONMethodCodec().encodeMethodCall(_popRouteMethodCall),
        (_) {},
      );
    } else if (_isFlutterEntry(state)) {
      // We get into this scenario when the user changes the url manually. It
      // causes a new entry to be pushed on top of our "flutter" one. When this
      // happens it first goes to the "else" section below where we capture the
      // path into `_userProvidedRouteName` then trigger a history back which
      // brings us here.
      assert(_userProvidedRouteName != null);

      final String newRouteName = _userProvidedRouteName!;
      _userProvidedRouteName = null;

      // Send a 'pushRoute' platform message so the app handles it accordingly.
      EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
        'flutter/navigation',
        const JSONMethodCodec().encodeMethodCall(MethodCall('pushRoute', newRouteName)),
        (_) {},
      );
    } else {
      // The user has pushed a new entry on top of our flutter entry. This could
      // happen when the user modifies the hash part of the url directly, for
      // example.

      // 1. We first capture the user's desired path.
      _userProvidedRouteName = currentPath;

      // 2. Then we remove the new entry.
      // This will take us back to our "flutter" entry and it causes a new
      // popstate event that will be handled in the "else if" section above.
      urlStrategy!.go(-1);
    }
  }

  /// This method should be called when the Origin Entry is active. It just
  /// replaces the state of the entry so that we can recognize it later using
  /// [_isOriginEntry] inside [_popStateListener].
  void _setupOriginEntry(ui_web.UrlStrategy strategy) {
    strategy.replaceState(_wrapOriginState(currentState), 'origin', '');
  }

  /// This method is used manipulate the Flutter Entry which is always the
  /// active entry while the Flutter app is running.
  void _setupFlutterEntry(ui_web.UrlStrategy strategy, {bool replace = false, String? path}) {
    path ??= currentPath;
    if (replace) {
      strategy.replaceState(_flutterState, 'flutter', path);
    } else {
      strategy.pushState(_flutterState, 'flutter', path);
    }
  }

  @override
  Future<void> tearDown() async {
    dispose();

    if (_isTornDown || urlStrategy == null) {
      return;
    }
    _isTornDown = true;

    // We need to remove the flutter entry that we pushed in setup.
    await urlStrategy!.go(-1);
    // Restores original state.
    urlStrategy!.replaceState(_unwrapOriginState(currentState), 'flutter', currentPath);
  }
}
