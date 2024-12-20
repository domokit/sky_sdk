// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui show Offset;

import '../dom.dart';
import '../semantics.dart' show EngineSemantics;
import '../text_editing/text_editing.dart';
import '../vector_math.dart';
import '../window.dart';

/// Returns an [ui.Offset] of the position of [event], relative to the position
/// of the Flutter [view].
///
/// The offset is *not* multiplied by DPR or anything else, it's the closest
/// to what the DOM would return if we had currentTarget readily available.
///
/// This takes an optional `eventTarget`, because the `event.target` may have
/// the wrong value for "coalesced" events. See:
///
/// - https://github.com/flutter/flutter/issues/155987
/// - https://github.com/flutter/flutter/issues/159804
/// - https://g-issues.chromium.org/issues/382473107
///
/// It also takes into account semantics being enabled to fix the case where
/// offsetX, offsetY == 0 (TalkBack events).
ui.Offset computeEventOffsetToTarget(
  DomMouseEvent event,
  EngineFlutterView view, {
  DomEventTarget? eventTarget,
}) {
  final DomElement actualTarget = view.dom.rootElement;
  // On a TalkBack event
  if (EngineSemantics.instance.semanticsEnabled && event.offsetX == 0 && event.offsetY == 0) {
    return _computeOffsetForTalkbackEvent(event, actualTarget);
  }

  // On one of our text-editing nodes
  eventTarget ??= event.target!;
  final bool isInput = view.dom.textEditingHost.contains(eventTarget as DomNode);
  if (isInput) {
    final EditableTextGeometry? inputGeometry = textEditing.strategy.geometry;
    if (inputGeometry != null) {
      return _computeOffsetForInputs(event, eventTarget, inputGeometry);
    }
  }

  // On another DOM Element (normally a platform view)
  final bool isTargetOutsideOfShadowDOM = eventTarget != actualTarget;
  if (isTargetOutsideOfShadowDOM) {
    final DomRect origin = actualTarget.getBoundingClientRect();
    // event.clientX/Y and origin.x/y are relative **to the viewport**.
    // (This doesn't work with 3D translations of the parent element.)
    // TODO(dit): Make this understand 3D transforms, https://github.com/flutter/flutter/issues/117091
    return ui.Offset(event.clientX - origin.x, event.clientY - origin.y);
  }

  // Return the offsetX/Y in the normal case.
  // (This works with 3D translations of the parent element.)
  return ui.Offset(event.offsetX, event.offsetY);
}

/// Computes the offsets for input nodes, which live outside of the shadowDOM.
/// Since inputs can be transformed (scaled, translated, etc), we can't rely on
/// `_computeOffsetRelativeToActualTarget` to calculate accurate coordinates, as
/// it only handles the case where inputs are translated, but will have issues
/// for scaled inputs (see: https://github.com/flutter/flutter/issues/125948).
///
/// We compute the offsets here by using the text input geometry data that is
/// sent from the framework, which includes information on how to transform the
/// underlying input element. We transform the `event.offset` points we receive
/// using the values from the input's transform matrix.
///
/// See [computeEventOffsetToTarget] for more information about `eventTarget`.
ui.Offset _computeOffsetForInputs(
  DomMouseEvent event,
  DomEventTarget eventTarget,
  EditableTextGeometry inputGeometry,
) {
  final DomElement targetElement = eventTarget as DomElement;
  final DomHTMLElement domElement = textEditing.strategy.activeDomElement;
  assert(
    targetElement == domElement,
    'The targeted input element must be the active input element',
  );
  final Float32List transformValues = inputGeometry.globalTransform;
  assert(transformValues.length == 16);
  final Matrix4 transform = Matrix4.fromFloat32List(transformValues);
  final Vector3 transformedPoint = transform.perspectiveTransform(
    x: event.offsetX,
    y: event.offsetY,
    z: 0,
  );

  return ui.Offset(transformedPoint.x, transformedPoint.y);
}

/// Computes the event offset when TalkBack is firing the event.
///
/// In this case, we need to use the clientX/Y position of the event (which are
/// relative to the absolute top-left corner of the page, including scroll), then
/// deduct the offsetLeft/Top from every offsetParent of the `actualTarget`.
///
///  ×-Page----║-------------------------------+
///  |         ║                               |
///  | ×-------║--------offsetParent(s)-----+  |
///  | |\                                   |  |
///  | | offsetLeft, offsetTop              |  |
///  | |                                    |  |
///  | |                                    |  |
///  | | ×-----║-------------actualTarget-+ |  |
///  | | |                                | |  |
///  ═════     × ─ (scrollLeft, scrollTop)═ ═  ═
///  | | |                                | |  |
///  | | |           ×                    | |  |
///  | | |            \                   | |  |
///  | | |             clientX, clientY   | |  |
///  | | |   (Relative to Page + Scroll)  | |  |
///  | | +-----║--------------------------+ |  |
///  | +-------║----------------------------+  |
///  +---------║-------------------------------+
///
/// Computing the offset of the event relative to the actualTarget requires to
/// compute the clientX, clientY of the actualTarget. To do that, we iterate
/// up the offsetParent elements of actualTarget adding their offset and scroll
/// positions. Finally, we deduct that from clientX, clientY of the event.
// TODO(dit): Make this understand 3D transforms, https://github.com/flutter/flutter/issues/117091
ui.Offset _computeOffsetForTalkbackEvent(DomMouseEvent event, DomElement actualTarget) {
  assert(EngineSemantics.instance.semanticsEnabled);
  // Use clientX/clientY as the position of the event (this is relative to
  // the top left of the page, including scroll)
  double offsetX = event.clientX;
  double offsetY = event.clientY;
  // Compute the scroll offset of actualTarget
  DomHTMLElement parent = actualTarget as DomHTMLElement;
  while (parent.offsetParent != null) {
    offsetX -= parent.offsetLeft - parent.scrollLeft;
    offsetY -= parent.offsetTop - parent.scrollTop;
    parent = parent.offsetParent!;
  }
  return ui.Offset(offsetX, offsetY);
}
