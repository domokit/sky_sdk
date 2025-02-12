// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/src/engine/display.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/window.dart';
import 'package:ui/ui.dart' as ui show Size;

import 'dimensions_provider.dart';

/// This class provides observable, real-time dimensions of a host element.
///
/// This class needs a `Stream` of `devicePixelRatio` changes, like the one
/// provided by [DisplayDprStream], because html resize observers do not report
/// DPR changes.
///
/// All the measurements returned from this class are potentially *expensive*,
/// and should be cached as needed. Every call to every method on this class
/// WILL perform actual DOM measurements.
///
/// This broadcasts `null` size events, to match the implementation of the
/// FullPageDimensionsProvider, but it could broadcast the size coming from the
/// DomResizeObserverEntry. Further changes in the engine are required for this
/// to be effective.
class CustomElementDimensionsProvider extends DimensionsProvider {
  /// Creates a [CustomElementDimensionsProvider] from a [_hostElement].
  CustomElementDimensionsProvider(this._hostElement, {Stream<double>? onDprChange}) {
    // Send a resize event when the page DPR changes.
    _dprChangeStreamSubscription = onDprChange?.listen((_) {
      _broadcastSize(null);
    });

    // Hook up a resize observer on the hostElement (if supported!).
    _hostElementResizeObserver = createDomResizeObserver((
      List<DomResizeObserverEntry> entries,
      DomResizeObserver _,
    ) {
      for (final DomResizeObserverEntry _ in entries) {
        _broadcastSize(null);
      }
    });

    assert(() {
      if (_hostElementResizeObserver == null) {
        domWindow.console.warn(
          'ResizeObserver API not supported. '
          'Flutter will not resize with its hostElement.',
        );
      }
      return true;
    }());

    _hostElementResizeObserver?.observe(_hostElement);
  }

  // The host element that will be used to retrieve (and observe) app size measurements.
  final DomElement _hostElement;

  // Handle resize events
  late DomResizeObserver? _hostElementResizeObserver;
  late StreamSubscription<double>? _dprChangeStreamSubscription;
  final StreamController<ui.Size?> _onResizeStreamController =
      StreamController<ui.Size?>.broadcast();

  // Broadcasts the last seen `Size`.
  void _broadcastSize(ui.Size? size) {
    _onResizeStreamController.add(size);
  }

  @override
  void close() {
    super.close();
    _hostElementResizeObserver?.disconnect();
    _dprChangeStreamSubscription?.cancel();
    _onResizeStreamController.close();
  }

  @override
  Stream<ui.Size?> get onResize => _onResizeStreamController.stream;

  @override
  ui.Size computePhysicalSize() {
    final double devicePixelRatio = EngineFlutterDisplay.instance.devicePixelRatio;
    return ui.Size(
      _hostElement.clientWidth * devicePixelRatio,
      _hostElement.clientHeight * devicePixelRatio,
    );
  }

  @override
  ViewPadding computeKeyboardInsets(double physicalHeight, bool isEditingOnMobile) {
    return const ViewPadding(top: 0, right: 0, bottom: 0, left: 0);
  }
}
