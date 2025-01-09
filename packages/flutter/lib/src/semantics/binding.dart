// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'semantics.dart';
library;

import 'dart:ui' as ui show AccessibilityFeatures, SemanticsActionEvent, SemanticsUpdateBuilder;

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'debug.dart';

export 'dart:ui' show AccessibilityFeatures, SemanticsActionEvent, SemanticsUpdateBuilder;

/// The glue between the semantics layer and the Flutter engine.
mixin SemanticsBinding on BindingBase {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    _accessibilityFeatures = platformDispatcher.accessibilityFeatures;
    platformDispatcher
      ..onSemanticsEnabledChanged = _handleSemanticsEnabledChanged
      ..onSemanticsActionEvent = _handleSemanticsActionEvent
      ..onAccessibilityFeaturesChanged = handleAccessibilityFeaturesChanged;
    _handleSemanticsEnabledChanged();
  }

  /// The current [SemanticsBinding], if one has been created.
  ///
  /// Provides access to the features exposed by this mixin. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [runApp] or [WidgetsFlutterBinding.ensureInitialized].
  static SemanticsBinding get instance => BindingBase.checkInstance(_instance);
  static SemanticsBinding? _instance;

  /// Whether semantics information must be collected.
  ///
  /// Returns true if either the platform has requested semantics information
  /// to be generated or if [ensureSemantics] has been called otherwise.
  ///
  /// To get notified when this value changes register a listener with
  /// [addSemanticsEnabledListener].
  bool get semanticsEnabled {
    assert(_semanticsEnabled.value == (_outstandingHandles > 0));
    return _semanticsEnabled.value;
  }

  late final ValueNotifier<bool> _semanticsEnabled = ValueNotifier<bool>(
    platformDispatcher.semanticsEnabled,
  );

  /// Adds a `listener` to be called when [semanticsEnabled] changes.
  ///
  /// See also:
  ///
  ///  * [removeSemanticsEnabledListener] to remove the listener again.
  ///  * [ValueNotifier.addListener], which documents how and when listeners are
  ///    called.
  void addSemanticsEnabledListener(VoidCallback listener) {
    _semanticsEnabled.addListener(listener);
  }

  /// Removes a `listener` added by [addSemanticsEnabledListener].
  ///
  /// See also:
  ///
  ///  * [ValueNotifier.removeListener], which documents how listeners are
  ///    removed.
  void removeSemanticsEnabledListener(VoidCallback listener) {
    _semanticsEnabled.removeListener(listener);
  }

  /// The number of clients registered to listen for semantics.
  ///
  /// The number is increased whenever [ensureSemantics] is called and decreased
  /// when [SemanticsHandle.dispose] is called.
  int get debugOutstandingSemanticsHandles => _outstandingHandles;
  int _outstandingHandles = 0;

  /// Creates a new [SemanticsHandle] and requests the collection of semantics
  /// information.
  ///
  /// Semantics information are only collected when there are clients interested
  /// in them. These clients express their interest by holding a
  /// [SemanticsHandle].
  ///
  /// Clients can close their [SemanticsHandle] by calling
  /// [SemanticsHandle.dispose]. Once all outstanding [SemanticsHandle] objects
  /// are closed, semantics information are no longer collected.
  SemanticsHandle ensureSemantics() {
    assert(_outstandingHandles >= 0);
    _outstandingHandles++;
    assert(_outstandingHandles > 0);
    _semanticsEnabled.value = true;
    return SemanticsHandle._(_didDisposeSemanticsHandle);
  }

  void _didDisposeSemanticsHandle() {
    assert(_outstandingHandles > 0);
    _outstandingHandles--;
    assert(_outstandingHandles >= 0);
    _semanticsEnabled.value = _outstandingHandles > 0;
  }

  // Handle for semantics request from the platform.
  SemanticsHandle? _semanticsHandle;

  void _handleSemanticsEnabledChanged() {
    if (platformDispatcher.semanticsEnabled) {
      _semanticsHandle ??= ensureSemantics();
    } else {
      _semanticsHandle?.dispose();
      _semanticsHandle = null;
    }
  }

  void _handleSemanticsActionEvent(ui.SemanticsActionEvent action) {
    final Object? arguments = action.arguments;
    final ui.SemanticsActionEvent decodedAction =
        arguments is ByteData
            ? action.copyWith(arguments: const StandardMessageCodec().decodeMessage(arguments))
            : action;
    performSemanticsAction(decodedAction);
  }

  /// Called whenever the platform requests an action to be performed on a
  /// [SemanticsNode].
  ///
  /// This callback is invoked when a user interacts with the app via an
  /// accessibility service (e.g. TalkBack and VoiceOver) and initiates an
  /// action on the focused node.
  ///
  /// Bindings that mixin the [SemanticsBinding] must implement this method and
  /// perform the given `action` on the [SemanticsNode] specified by
  /// [ui.SemanticsActionEvent.nodeId].
  ///
  /// See [dart:ui.PlatformDispatcher.onSemanticsActionEvent].
  @protected
  void performSemanticsAction(ui.SemanticsActionEvent action);

  /// The currently active set of [ui.AccessibilityFeatures].
  ///
  /// This is set when the binding is first initialized and updated whenever a
  /// flag is changed.
  ///
  /// To listen to changes to accessibility features, create a
  /// [WidgetsBindingObserver] and listen to
  /// [WidgetsBindingObserver.didChangeAccessibilityFeatures].
  ui.AccessibilityFeatures get accessibilityFeatures => _accessibilityFeatures;
  late ui.AccessibilityFeatures _accessibilityFeatures;

  /// Called when the platform accessibility features change.
  ///
  /// See [dart:ui.PlatformDispatcher.onAccessibilityFeaturesChanged].
  @protected
  @mustCallSuper
  void handleAccessibilityFeaturesChanged() {
    _accessibilityFeatures = platformDispatcher.accessibilityFeatures;
  }

  /// Creates an empty semantics update builder.
  ///
  /// The caller is responsible for filling out the semantics node updates.
  ///
  /// This method is used by the [SemanticsOwner] to create builder for all its
  /// semantics updates.
  ui.SemanticsUpdateBuilder createSemanticsUpdateBuilder() {
    return ui.SemanticsUpdateBuilder();
  }

  /// The platform is requesting that animations be disabled or simplified.
  ///
  /// This setting can be overridden for testing or debugging by setting
  /// [debugSemanticsDisableAnimations].
  bool get disableAnimations {
    bool value = _accessibilityFeatures.disableAnimations;
    assert(() {
      if (debugSemanticsDisableAnimations != null) {
        value = debugSemanticsDisableAnimations!;
      }
      return true;
    }());
    return value;
  }
}

/// A reference to the semantics information generated by the framework.
///
/// Semantics information are only collected when there are clients interested
/// in them. These clients express their interest by holding a
/// [SemanticsHandle]. When the client no longer needs the
/// semantics information, it must call [dispose] on the [SemanticsHandle] to
/// close it. When all open [SemanticsHandle]s are disposed, the framework will
/// stop updating the semantics information.
///
/// To obtain a [SemanticsHandle], call [SemanticsBinding.ensureSemantics].
class SemanticsHandle {
  SemanticsHandle._(this._onDispose) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/semantics.dart',
        className: '$SemanticsHandle',
        object: this,
      );
    }
  }

  final VoidCallback _onDispose;

  /// Closes the semantics handle.
  ///
  /// When all the outstanding [SemanticsHandle] objects are closed, the
  /// framework will stop generating semantics information.
  @mustCallSuper
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }

    _onDispose();
  }
}
