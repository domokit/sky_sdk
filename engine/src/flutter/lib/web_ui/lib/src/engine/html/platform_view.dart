// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import '../platform_dispatcher.dart';
import '../platform_views/slots.dart';
import '../window.dart';
import 'surface.dart';

/// A surface containing a platform view, which is an HTML element.
class PersistedPlatformView extends PersistedLeafSurface {
  PersistedPlatformView(this.platformViewId, this.dx, this.dy, this.width, this.height) {
    // Ensure platform view with `viewId` is injected into the `implicitView`
    // before rendering its shadow DOM `slot`.
    final EngineFlutterView implicitView = EnginePlatformDispatcher.instance.implicitView!;
    implicitView.dom.injectPlatformView(platformViewId);
  }

  final int platformViewId;
  final double dx;
  final double dy;
  final double width;
  final double height;

  @override
  DomElement createElement() {
    return createPlatformViewSlot(platformViewId);
  }

  @override
  void apply() {
    // See `_compositeWithParams` in the HtmlViewEmbedder for the canvaskit equivalent.
    rootElement!.style
      ..transform = 'translate(${dx}px, ${dy}px)'
      ..width = '${width}px'
      ..height = '${height}px'
      ..position = 'absolute';
  }

  // Platform Views can only be updated if their viewId matches.
  @override
  bool canUpdateAsMatch(PersistedSurface oldSurface) {
    if (super.canUpdateAsMatch(oldSurface)) {
      // super checks the runtimeType of the surface, so we can just cast...
      return platformViewId == ((oldSurface as PersistedPlatformView).platformViewId);
    }
    return false;
  }

  @override
  double matchForUpdate(PersistedPlatformView existingSurface) {
    return existingSurface.platformViewId == platformViewId ? 0.0 : 1.0;
  }

  @override
  void update(PersistedPlatformView oldSurface) {
    assert(
      platformViewId == oldSurface.platformViewId,
      'PersistedPlatformView with different platformViewId should never be updated. Check the canUpdateAsMatch method.',
    );
    super.update(oldSurface);
    // Only update if the view has been resized
    if (dx != oldSurface.dx ||
        dy != oldSurface.dy ||
        width != oldSurface.width ||
        height != oldSurface.height) {
      // A change in any of the dimensions is performed by calling apply.
      apply();
    }
  }
}
