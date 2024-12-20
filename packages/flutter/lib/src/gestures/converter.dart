// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
library;

import 'dart:ui' as ui show PointerChange, PointerData, PointerSignalKind;

import 'events.dart';

export 'dart:ui' show PointerData;

export 'events.dart' show PointerEvent;

// Add `kPrimaryButton` to [buttons] when a pointer of certain devices is down.
//
// TODO(tongmu): This patch is supposed to be done by embedders. Patching it
// in framework is a workaround before [PointerEventConverter] is moved to embedders.
// https://github.com/flutter/flutter/issues/30454
int _synthesiseDownButtons(int buttons, PointerDeviceKind kind) {
  switch (kind) {
    case PointerDeviceKind.mouse:
    case PointerDeviceKind.trackpad:
      return buttons;
    case PointerDeviceKind.touch:
    case PointerDeviceKind.stylus:
    case PointerDeviceKind.invertedStylus:
      return buttons == 0 ? kPrimaryButton : buttons;
    case PointerDeviceKind.unknown:
      // We have no information about the device but we know we never want
      // buttons to be 0 when the pointer is down.
      return buttons == 0 ? kPrimaryButton : buttons;
  }
}

/// Signature for a callback that returns the device pixel ratio of a
/// [FlutterView] identified by the provided `viewId`.
///
/// Returns null if no view with the provided ID exists.
///
/// Used by [PointerEventConverter.expand].
///
/// See also:
///
///  * [FlutterView.devicePixelRatio] for an explanation of device pixel ratio.
typedef DevicePixelRatioGetter = double? Function(int viewId);

/// Converts from engine pointer data to framework pointer events.
///
/// This takes [PointerDataPacket] objects, as received from the engine via
/// [dart:ui.PlatformDispatcher.onPointerDataPacket], and converts them to
/// [PointerEvent] objects.
abstract final class PointerEventConverter {
  /// Expand the given packet of pointer data into a sequence of framework
  /// pointer events.
  ///
  /// The `devicePixelRatioForView` is used to obtain the device pixel ratio for
  /// the view a particular event occurred in to convert its data from physical
  /// coordinates to logical pixels. See the discussion at [PointerEvent] for
  /// more details on the [PointerEvent] coordinate space.
  static Iterable<PointerEvent> expand(
    Iterable<ui.PointerData> data,
    DevicePixelRatioGetter devicePixelRatioForView,
  ) {
    return data
        .where((ui.PointerData datum) => datum.signalKind != ui.PointerSignalKind.unknown)
        .map<PointerEvent?>((ui.PointerData datum) {
          final double? devicePixelRatio = devicePixelRatioForView(datum.viewId);
          if (devicePixelRatio == null) {
            // View doesn't exist anymore.
            return null;
          }
          final Offset position = Offset(datum.physicalX, datum.physicalY) / devicePixelRatio;
          final Offset delta =
              Offset(datum.physicalDeltaX, datum.physicalDeltaY) / devicePixelRatio;
          final double radiusMinor = _toLogicalPixels(datum.radiusMinor, devicePixelRatio);
          final double radiusMajor = _toLogicalPixels(datum.radiusMajor, devicePixelRatio);
          final double radiusMin = _toLogicalPixels(datum.radiusMin, devicePixelRatio);
          final double radiusMax = _toLogicalPixels(datum.radiusMax, devicePixelRatio);
          final Duration timeStamp = datum.timeStamp;
          final PointerDeviceKind kind = datum.kind;
          switch (datum.signalKind ?? ui.PointerSignalKind.none) {
            case ui.PointerSignalKind.none:
              switch (datum.change) {
                case ui.PointerChange.add:
                  return PointerAddedEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    obscured: datum.obscured,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distance: datum.distance,
                    distanceMax: datum.distanceMax,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.hover:
                  return PointerHoverEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    delta: delta,
                    buttons: datum.buttons,
                    obscured: datum.obscured,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distance: datum.distance,
                    distanceMax: datum.distanceMax,
                    size: datum.size,
                    radiusMajor: radiusMajor,
                    radiusMinor: radiusMinor,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    synthesized: datum.synthesized,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.down:
                  return PointerDownEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    buttons: _synthesiseDownButtons(datum.buttons, kind),
                    obscured: datum.obscured,
                    pressure: datum.pressure,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distanceMax: datum.distanceMax,
                    size: datum.size,
                    radiusMajor: radiusMajor,
                    radiusMinor: radiusMinor,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.move:
                  return PointerMoveEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    delta: delta,
                    buttons: _synthesiseDownButtons(datum.buttons, kind),
                    obscured: datum.obscured,
                    pressure: datum.pressure,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distanceMax: datum.distanceMax,
                    size: datum.size,
                    radiusMajor: radiusMajor,
                    radiusMinor: radiusMinor,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    platformData: datum.platformData,
                    synthesized: datum.synthesized,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.up:
                  return PointerUpEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    buttons: datum.buttons,
                    obscured: datum.obscured,
                    pressure: datum.pressure,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distance: datum.distance,
                    distanceMax: datum.distanceMax,
                    size: datum.size,
                    radiusMajor: radiusMajor,
                    radiusMinor: radiusMinor,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.cancel:
                  return PointerCancelEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    buttons: datum.buttons,
                    obscured: datum.obscured,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distance: datum.distance,
                    distanceMax: datum.distanceMax,
                    size: datum.size,
                    radiusMajor: radiusMajor,
                    radiusMinor: radiusMinor,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    orientation: datum.orientation,
                    tilt: datum.tilt,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.remove:
                  return PointerRemovedEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    kind: kind,
                    device: datum.device,
                    position: position,
                    obscured: datum.obscured,
                    pressureMin: datum.pressureMin,
                    pressureMax: datum.pressureMax,
                    distanceMax: datum.distanceMax,
                    radiusMin: radiusMin,
                    radiusMax: radiusMax,
                    embedderId: datum.embedderId,
                  );
                case ui.PointerChange.panZoomStart:
                  return PointerPanZoomStartEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    device: datum.device,
                    position: position,
                    embedderId: datum.embedderId,
                    synthesized: datum.synthesized,
                  );
                case ui.PointerChange.panZoomUpdate:
                  final Offset pan = Offset(datum.panX, datum.panY) / devicePixelRatio;
                  final Offset panDelta =
                      Offset(datum.panDeltaX, datum.panDeltaY) / devicePixelRatio;
                  return PointerPanZoomUpdateEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    device: datum.device,
                    position: position,
                    pan: pan,
                    panDelta: panDelta,
                    scale: datum.scale,
                    rotation: datum.rotation,
                    embedderId: datum.embedderId,
                    synthesized: datum.synthesized,
                  );
                case ui.PointerChange.panZoomEnd:
                  return PointerPanZoomEndEvent(
                    viewId: datum.viewId,
                    timeStamp: timeStamp,
                    pointer: datum.pointerIdentifier,
                    device: datum.device,
                    position: position,
                    embedderId: datum.embedderId,
                    synthesized: datum.synthesized,
                  );
              }
            case ui.PointerSignalKind.scroll:
              if (!datum.scrollDeltaX.isFinite ||
                  !datum.scrollDeltaY.isFinite ||
                  devicePixelRatio <= 0) {
                return null;
              }
              final Offset scrollDelta =
                  Offset(datum.scrollDeltaX, datum.scrollDeltaY) / devicePixelRatio;
              return PointerScrollEvent(
                viewId: datum.viewId,
                timeStamp: timeStamp,
                kind: kind,
                device: datum.device,
                position: position,
                scrollDelta: scrollDelta,
                embedderId: datum.embedderId,
                onRespond: datum.respond,
              );
            case ui.PointerSignalKind.scrollInertiaCancel:
              return PointerScrollInertiaCancelEvent(
                viewId: datum.viewId,
                timeStamp: timeStamp,
                kind: kind,
                device: datum.device,
                position: position,
                embedderId: datum.embedderId,
              );
            case ui.PointerSignalKind.scale:
              return PointerScaleEvent(
                viewId: datum.viewId,
                timeStamp: timeStamp,
                kind: kind,
                device: datum.device,
                position: position,
                embedderId: datum.embedderId,
                scale: datum.scale,
              );
            case ui.PointerSignalKind.unknown:
              throw StateError('Unreachable');
          }
        })
        .whereType<PointerEvent>();
  }

  static double _toLogicalPixels(double physicalPixels, double devicePixelRatio) =>
      physicalPixels / devicePixelRatio;
}
