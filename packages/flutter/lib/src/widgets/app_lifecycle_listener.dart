// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
library;

import 'dart:ui';
import 'package:flutter/foundation.dart';

import 'binding.dart';

/// A callback type that is used by [AppLifecycleListener.onExitRequested] to
/// ask the application if it wants to cancel application termination or not.
typedef AppExitRequestCallback = Future<AppExitResponse> Function();

/// A listener that can be used to listen to changes in the application
/// lifecycle.
///
/// To listen for requests for the application to exit, and to decide whether or
/// not the application should exit when requested, create an
/// [AppLifecycleListener] and set the [onExitRequested] callback.
///
/// To listen for changes in the application lifecycle state, define an
/// [onStateChange] callback. See the [AppLifecycleState] enum for details on
/// the various states.
///
/// The [onStateChange] callback is called for each state change, and the
/// individual state transitions ([onResume], [onInactive], etc.) are also
/// called if the state transition they represent occurs.
///
/// State changes will occur in accordance with the state machine described by
/// this diagram:
///
/// ![Diagram of the application lifecycle defined by the AppLifecycleState enum](
/// https://flutter.github.io/assets-for-api-docs/assets/dart-ui/app_lifecycle.png)
///
/// The initial state of the state machine is the [AppLifecycleState.detached]
/// state, and the arrows describe valid state transitions. Transitions in blue
/// are transitions that only happen on iOS and Android.
///
/// {@tool dartpad}
/// This example shows how an application can listen to changes in the
/// application state.
///
/// ** See code in examples/api/lib/widgets/app_lifecycle_listener/app_lifecycle_listener.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how an application can optionally decide to abort a
/// request for exiting instead of obeying the request.
///
/// ** See code in examples/api/lib/widgets/app_lifecycle_listener/app_lifecycle_listener.1.dart **
/// {@end-tool}
///
/// See also:
///
/// * [ServicesBinding.exitApplication] for a function to call that will request
///   that the application exits.
/// * [WidgetsBindingObserver.didRequestAppExit] for the handler which this
///   class uses to receive exit requests.
/// * [WidgetsBindingObserver.didChangeAppLifecycleState] for the handler which
///   this class uses to receive lifecycle state changes.
class AppLifecycleListener with WidgetsBindingObserver, Diagnosticable {
  /// Creates an [AppLifecycleListener].
  AppLifecycleListener({
    WidgetsBinding? binding,
    this.onResume,
    this.onInactive,
    this.onHide,
    this.onShow,
    this.onPause,
    this.onRestart,
    this.onDetach,
    this.onExitRequested,
    this.onStateChange,
  }) : binding = binding ?? WidgetsBinding.instance,
       _lifecycleState = (binding ?? WidgetsBinding.instance).lifecycleState {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$AppLifecycleListener',
        object: this,
      );
    }
    this.binding.addObserver(this);
  }

  AppLifecycleState? _lifecycleState;

  /// The [WidgetsBinding] to listen to for application lifecycle events.
  ///
  /// Typically, this is set to [WidgetsBinding.instance], but may be
  /// substituted for testing or other specialized bindings.
  ///
  /// Defaults to [WidgetsBinding.instance].
  final WidgetsBinding binding;

  /// Called anytime the state changes, passing the new state.
  final ValueChanged<AppLifecycleState>? onStateChange;

  /// A callback that is called when the application loses input focus.
  ///
  /// On mobile platforms, this can be during a phone call or when a system
  /// dialog is visible.
  ///
  /// On desktop platforms, this is when all views in an application have lost
  /// input focus but at least one view of the application is still visible.
  ///
  /// On the web, this is when the window (or tab) has lost input focus.
  final VoidCallback? onInactive;

  /// A callback that is called when a view in the application gains input
  /// focus.
  ///
  /// A call to this callback indicates that the application is entering a state
  /// where it is visible, active, and accepting user input.
  final VoidCallback? onResume;

  /// A callback that is called when the application is hidden.
  ///
  /// On mobile platforms, this is usually just before the application is
  /// replaced by another application in the foreground.
  ///
  /// On desktop platforms, this is just before the application is hidden by
  /// being minimized or otherwise hiding all views of the application.
  ///
  /// On the web, this is just before a window (or tab) is hidden.
  final VoidCallback? onHide;

  /// A callback that is called when the application is shown.
  ///
  /// On mobile platforms, this is usually just before the application replaces
  /// another application in the foreground.
  ///
  /// On desktop platforms, this is just before the application is shown after
  /// being minimized or otherwise made to show at least one view of the
  /// application.
  ///
  /// On the web, this is just before a window (or tab) is shown.
  final VoidCallback? onShow;

  /// A callback that is called when the application is paused.
  ///
  /// On mobile platforms, this happens right before the application is replaced
  /// by another application.
  ///
  /// On desktop platforms and the web, this function is not called.
  final VoidCallback? onPause;

  /// A callback that is called when the application is resumed after being
  /// paused.
  ///
  /// On mobile platforms, this happens just before this application takes over
  /// as the active application.
  ///
  /// On desktop platforms and the web, this function is not called.
  final VoidCallback? onRestart;

  /// A callback used to ask the application if it will allow exiting the
  /// application for cases where the exit is cancelable.
  ///
  /// Exiting the application isn't always cancelable, but when it is, this
  /// function will be called before exit occurs.
  ///
  /// Responding [AppExitResponse.exit] will continue termination, and
  /// responding [AppExitResponse.cancel] will cancel it. If termination is not
  /// canceled, the application will immediately exit.
  final AppExitRequestCallback? onExitRequested;

  /// A callback that is called when an application has exited, and detached all
  /// host views from the engine.
  ///
  /// This callback is only called on iOS and Android.
  final VoidCallback? onDetach;

  bool _debugDisposed = false;

  /// Call when the listener is no longer in use.
  ///
  /// Do not use the object after calling [dispose].
  ///
  /// Subclasses must call this method in their overridden [dispose], if any.
  @mustCallSuper
  void dispose() {
    assert(_debugAssertNotDisposed());
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    binding.removeObserver(this);
    assert(() {
      _debugDisposed = true;
      return true;
    }());
  }

  bool _debugAssertNotDisposed() {
    assert(() {
      if (_debugDisposed) {
        throw FlutterError(
          'A $runtimeType was used after being disposed.\n'
          'Once you have called dispose() on a $runtimeType, it '
          'can no longer be used.',
        );
      }
      return true;
    }());
    return true;
  }

  @override
  Future<AppExitResponse> didRequestAppExit() async {
    assert(_debugAssertNotDisposed());
    if (onExitRequested == null) {
      return AppExitResponse.exit;
    }
    return onExitRequested!();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    assert(_debugAssertNotDisposed());
    final AppLifecycleState? previousState = _lifecycleState;
    if (state == previousState) {
      // Transitioning to the same state twice doesn't produce any
      // notifications (but also won't actually occur).
      return;
    }
    _lifecycleState = state;
    switch (state) {
      case AppLifecycleState.resumed:
        assert(
          previousState == null ||
              previousState == AppLifecycleState.inactive ||
              previousState == AppLifecycleState.detached,
          'Invalid state transition from $previousState to $state',
        );
        onResume?.call();
      case AppLifecycleState.inactive:
        assert(
          previousState == null ||
              previousState == AppLifecycleState.hidden ||
              previousState == AppLifecycleState.resumed,
          'Invalid state transition from $previousState to $state',
        );
        if (previousState == AppLifecycleState.hidden) {
          onShow?.call();
        } else if (previousState == null || previousState == AppLifecycleState.resumed) {
          onInactive?.call();
        }
      case AppLifecycleState.hidden:
        assert(
          previousState == null ||
              previousState == AppLifecycleState.paused ||
              previousState == AppLifecycleState.inactive,
          'Invalid state transition from $previousState to $state',
        );
        if (previousState == AppLifecycleState.paused) {
          onRestart?.call();
        } else if (previousState == null || previousState == AppLifecycleState.inactive) {
          onHide?.call();
        }
      case AppLifecycleState.paused:
        assert(
          previousState == null || previousState == AppLifecycleState.hidden,
          'Invalid state transition from $previousState to $state',
        );
        if (previousState == null || previousState == AppLifecycleState.hidden) {
          onPause?.call();
        }
      case AppLifecycleState.detached:
        assert(
          previousState == null || previousState == AppLifecycleState.paused,
          'Invalid state transition from $previousState to $state',
        );
        onDetach?.call();
    }
    // At this point, it can't be null anymore.
    onStateChange?.call(_lifecycleState!);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<WidgetsBinding>('binding', binding));
    properties.add(
      FlagProperty('onStateChange', value: onStateChange != null, ifTrue: 'onStateChange'),
    );
    properties.add(FlagProperty('onInactive', value: onInactive != null, ifTrue: 'onInactive'));
    properties.add(FlagProperty('onResume', value: onResume != null, ifTrue: 'onResume'));
    properties.add(FlagProperty('onHide', value: onHide != null, ifTrue: 'onHide'));
    properties.add(FlagProperty('onShow', value: onShow != null, ifTrue: 'onShow'));
    properties.add(FlagProperty('onPause', value: onPause != null, ifTrue: 'onPause'));
    properties.add(FlagProperty('onRestart', value: onRestart != null, ifTrue: 'onRestart'));
    properties.add(
      FlagProperty('onExitRequested', value: onExitRequested != null, ifTrue: 'onExitRequested'),
    );
    properties.add(FlagProperty('onDetach', value: onDetach != null, ifTrue: 'onDetach'));
  }
}
