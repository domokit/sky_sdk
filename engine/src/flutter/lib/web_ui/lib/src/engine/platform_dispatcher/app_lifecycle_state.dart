// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;

import '../../engine.dart';

/// Signature of functions added as a listener to [ui.AppLifecycleState] changes
typedef AppLifecycleStateListener = void Function(ui.AppLifecycleState state);

/// Determines the [ui.AppLifecycleState].
abstract class AppLifecycleState {
  static AppLifecycleState create(FlutterViewManager viewManager) {
    return _BrowserAppLifecycleState(viewManager);
  }

  ui.AppLifecycleState get appLifecycleState => _appLifecycleState;
  ui.AppLifecycleState _appLifecycleState = ui.AppLifecycleState.resumed;

  final List<AppLifecycleStateListener> _listeners = <AppLifecycleStateListener>[];

  void addListener(AppLifecycleStateListener listener) {
    if (_listeners.isEmpty) {
      activate();
    }
    _listeners.add(listener);
    listener(_appLifecycleState);
  }

  void removeListener(AppLifecycleStateListener listener) {
    _listeners.remove(listener);
    if (_listeners.isEmpty) {
      deactivate();
    }
  }

  @protected
  void activate();

  @protected
  void deactivate();

  @visibleForTesting
  void onAppLifecycleStateChange(ui.AppLifecycleState newState) {
    if (newState != _appLifecycleState) {
      _appLifecycleState = newState;
      for (final AppLifecycleStateListener listener in _listeners) {
        listener(newState);
      }
    }
  }
}

/// Manages [ui.AppLifecycleState] within a web context by monitoring specific
/// browser events.
///
/// This class listens to:
/// - 'visibilitychange' on [DomHTMLDocument] to observe visibility changes,
/// - 'focus' and 'blur' on [DomWindow] to track application focus shifts.
class _BrowserAppLifecycleState extends AppLifecycleState {
  _BrowserAppLifecycleState(this._viewManager);

  final FlutterViewManager _viewManager;
  final List<StreamSubscription<void>> _subscriptions = <StreamSubscription<void>>[];

  @override
  void activate() {
    domWindow.addEventListener('focus', _focusListener);
    domWindow.addEventListener('blur', _blurListener);
    domDocument.addEventListener('visibilitychange', _visibilityChangeListener);
    _subscriptions
      ..add(_viewManager.onViewCreated.listen(_onViewCountChanged))
      ..add(_viewManager.onViewDisposed.listen(_onViewCountChanged));
  }

  @override
  void deactivate() {
    domWindow.removeEventListener('focus', _focusListener);
    domWindow.removeEventListener('blur', _blurListener);
    domDocument.removeEventListener('visibilitychange', _visibilityChangeListener);
    for (final StreamSubscription<void> subscription in _subscriptions) {
      subscription.cancel();
    }
    _subscriptions.clear();
  }

  late final DomEventListener _focusListener = createDomEventListener((DomEvent event) {
    onAppLifecycleStateChange(ui.AppLifecycleState.resumed);
  });

  late final DomEventListener _blurListener = createDomEventListener((DomEvent event) {
    onAppLifecycleStateChange(ui.AppLifecycleState.inactive);
  });

  late final DomEventListener _visibilityChangeListener = createDomEventListener((DomEvent event) {
    if (domDocument.visibilityState == 'visible') {
      onAppLifecycleStateChange(ui.AppLifecycleState.resumed);
    } else if (domDocument.visibilityState == 'hidden') {
      onAppLifecycleStateChange(ui.AppLifecycleState.hidden);
    }
  });

  void _onViewCountChanged(int _) {
    if (_viewManager.views.isEmpty) {
      onAppLifecycleStateChange(ui.AppLifecycleState.detached);
    } else {
      onAppLifecycleStateChange(ui.AppLifecycleState.resumed);
    }
  }
}
