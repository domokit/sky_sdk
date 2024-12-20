// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'constants.dart';
import 'events.dart';
import 'recognizer.dart';
import 'velocity_tracker.dart';

export 'dart:ui' show Offset, PointerDeviceKind;

export 'events.dart' show PointerDownEvent, PointerEvent, PointerPanZoomStartEvent;
export 'recognizer.dart' show DragStartBehavior;
export 'velocity_tracker.dart' show Velocity;

/// The default conversion factor when treating mouse scrolling as scaling.
///
/// The value was arbitrarily chosen to feel natural for most mousewheels on
/// all supported platforms.
const double kDefaultMouseScrollToScaleFactor = 200;

/// The default conversion factor when treating trackpad scrolling as scaling.
///
/// This factor matches the default [kDefaultMouseScrollToScaleFactor] of 200 to
/// feel natural for most trackpads, and the convention that scrolling up means
/// zooming in.
const Offset kDefaultTrackpadScrollToScaleFactor = Offset(0, -1 / kDefaultMouseScrollToScaleFactor);

/// The possible states of a [ScaleGestureRecognizer].
enum _ScaleState {
  /// The recognizer is ready to start recognizing a gesture.
  ready,

  /// The sequence of pointer events seen thus far is consistent with a scale
  /// gesture but the gesture has not been accepted definitively.
  possible,

  /// The sequence of pointer events seen thus far has been accepted
  /// definitively as a scale gesture.
  accepted,

  /// The sequence of pointer events seen thus far has been accepted
  /// definitively as a scale gesture and the pointers established a focal point
  /// and initial scale.
  started,
}

class _PointerPanZoomData {
  _PointerPanZoomData.fromStartEvent(this.parent, PointerPanZoomStartEvent event)
    : _position = event.position,
      _pan = Offset.zero,
      _scale = 1,
      _rotation = 0;

  _PointerPanZoomData.fromUpdateEvent(this.parent, PointerPanZoomUpdateEvent event)
    : _position = event.position,
      _pan = event.pan,
      _scale = event.scale,
      _rotation = event.rotation;

  final ScaleGestureRecognizer parent;
  final Offset _position;
  final Offset _pan;
  final double _scale;
  final double _rotation;

  Offset get focalPoint {
    if (parent.trackpadScrollCausesScale) {
      return _position;
    }
    return _position + _pan;
  }

  double get scale {
    if (parent.trackpadScrollCausesScale) {
      return _scale *
          math.exp(
            (_pan.dx * parent.trackpadScrollToScaleFactor.dx) +
                (_pan.dy * parent.trackpadScrollToScaleFactor.dy),
          );
    }
    return _scale;
  }

  double get rotation => _rotation;

  @override
  String toString() =>
      '_PointerPanZoomData(parent: $parent, _position: $_position, _pan: $_pan, _scale: $_scale, _rotation: $_rotation)';
}

/// Details for [GestureScaleStartCallback].
class ScaleStartDetails {
  /// Creates details for [GestureScaleStartCallback].
  ScaleStartDetails({
    this.focalPoint = Offset.zero,
    Offset? localFocalPoint,
    this.pointerCount = 0,
    this.sourceTimeStamp,
  }) : localFocalPoint = localFocalPoint ?? focalPoint;

  /// The initial focal point of the pointers in contact with the screen.
  ///
  /// Reported in global coordinates.
  ///
  /// See also:
  ///
  ///  * [localFocalPoint], which is the same value reported in local
  ///    coordinates.
  final Offset focalPoint;

  /// The initial focal point of the pointers in contact with the screen.
  ///
  /// Reported in local coordinates. Defaults to [focalPoint] if not set in the
  /// constructor.
  ///
  /// See also:
  ///
  ///  * [focalPoint], which is the same value reported in global
  ///    coordinates.
  final Offset localFocalPoint;

  /// The number of pointers being tracked by the gesture recognizer.
  ///
  /// Typically this is the number of fingers being used to pan the widget using the gesture
  /// recognizer.
  final int pointerCount;

  /// Recorded timestamp of the source pointer event that triggered the scale
  /// event.
  ///
  /// Could be null if triggered from proxied events such as accessibility.
  final Duration? sourceTimeStamp;

  @override
  String toString() =>
      'ScaleStartDetails(focalPoint: $focalPoint, localFocalPoint: $localFocalPoint, pointersCount: $pointerCount)';
}

/// Details for [GestureScaleUpdateCallback].
class ScaleUpdateDetails {
  /// Creates details for [GestureScaleUpdateCallback].
  ///
  /// The [scale], [horizontalScale], and [verticalScale] arguments must be
  /// greater than or equal to zero.
  ScaleUpdateDetails({
    this.focalPoint = Offset.zero,
    Offset? localFocalPoint,
    this.scale = 1.0,
    this.horizontalScale = 1.0,
    this.verticalScale = 1.0,
    this.rotation = 0.0,
    this.pointerCount = 0,
    this.focalPointDelta = Offset.zero,
    this.sourceTimeStamp,
  }) : assert(scale >= 0.0),
       assert(horizontalScale >= 0.0),
       assert(verticalScale >= 0.0),
       localFocalPoint = localFocalPoint ?? focalPoint;

  /// The amount the gesture's focal point has moved in the coordinate space of
  /// the event receiver since the previous update.
  ///
  /// Defaults to zero if not specified in the constructor.
  final Offset focalPointDelta;

  /// The focal point of the pointers in contact with the screen.
  ///
  /// Reported in global coordinates.
  ///
  /// See also:
  ///
  ///  * [localFocalPoint], which is the same value reported in local
  ///    coordinates.
  final Offset focalPoint;

  /// The focal point of the pointers in contact with the screen.
  ///
  /// Reported in local coordinates. Defaults to [focalPoint] if not set in the
  /// constructor.
  ///
  /// See also:
  ///
  ///  * [focalPoint], which is the same value reported in global
  ///    coordinates.
  final Offset localFocalPoint;

  /// The scale implied by the average distance between the pointers in contact
  /// with the screen.
  ///
  /// This value must be greater than or equal to zero.
  ///
  /// See also:
  ///
  ///  * [horizontalScale], which is the scale along the horizontal axis.
  ///  * [verticalScale], which is the scale along the vertical axis.
  final double scale;

  /// The scale implied by the average distance along the horizontal axis
  /// between the pointers in contact with the screen.
  ///
  /// This value must be greater than or equal to zero.
  ///
  /// See also:
  ///
  ///  * [scale], which is the general scale implied by the pointers.
  ///  * [verticalScale], which is the scale along the vertical axis.
  final double horizontalScale;

  /// The scale implied by the average distance along the vertical axis
  /// between the pointers in contact with the screen.
  ///
  /// This value must be greater than or equal to zero.
  ///
  /// See also:
  ///
  ///  * [scale], which is the general scale implied by the pointers.
  ///  * [horizontalScale], which is the scale along the horizontal axis.
  final double verticalScale;

  /// The angle implied by the first two pointers to enter in contact with
  /// the screen.
  ///
  /// Expressed in radians.
  final double rotation;

  /// The number of pointers being tracked by the gesture recognizer.
  ///
  /// Typically this is the number of fingers being used to pan the widget using the gesture
  /// recognizer. Due to platform limitations, trackpad gestures count as two fingers
  /// even if more than two fingers are used.
  final int pointerCount;

  /// Recorded timestamp of the source pointer event that triggered the scale
  /// event.
  ///
  /// Could be null if triggered from proxied events such as accessibility.
  final Duration? sourceTimeStamp;

  @override
  String toString() =>
      'ScaleUpdateDetails('
      'focalPoint: $focalPoint,'
      ' localFocalPoint: $localFocalPoint,'
      ' scale: $scale,'
      ' horizontalScale: $horizontalScale,'
      ' verticalScale: $verticalScale,'
      ' rotation: $rotation,'
      ' pointerCount: $pointerCount,'
      ' focalPointDelta: $focalPointDelta,'
      ' sourceTimeStamp: $sourceTimeStamp)';
}

/// Details for [GestureScaleEndCallback].
class ScaleEndDetails {
  /// Creates details for [GestureScaleEndCallback].
  ScaleEndDetails({this.velocity = Velocity.zero, this.scaleVelocity = 0, this.pointerCount = 0});

  /// The velocity of the last pointer to be lifted off of the screen.
  final Velocity velocity;

  /// The final velocity of the scale factor reported by the gesture.
  final double scaleVelocity;

  /// The number of pointers being tracked by the gesture recognizer.
  ///
  /// Typically this is the number of fingers being used to pan the widget using the gesture
  /// recognizer.
  final int pointerCount;

  @override
  String toString() =>
      'ScaleEndDetails(velocity: $velocity, scaleVelocity: $scaleVelocity, pointerCount: $pointerCount)';
}

/// Signature for when the pointers in contact with the screen have established
/// a focal point and initial scale of 1.0.
typedef GestureScaleStartCallback = void Function(ScaleStartDetails details);

/// Signature for when the pointers in contact with the screen have indicated a
/// new focal point and/or scale.
typedef GestureScaleUpdateCallback = void Function(ScaleUpdateDetails details);

/// Signature for when the pointers are no longer in contact with the screen.
typedef GestureScaleEndCallback = void Function(ScaleEndDetails details);

bool _isFlingGesture(Velocity velocity) {
  final double speedSquared = velocity.pixelsPerSecond.distanceSquared;
  return speedSquared > kMinFlingVelocity * kMinFlingVelocity;
}

/// Defines a line between two pointers on screen.
///
/// [_LineBetweenPointers] is an abstraction of a line between two pointers in
/// contact with the screen. Used to track the rotation of a scale gesture.
class _LineBetweenPointers {
  /// Creates a [_LineBetweenPointers]. None of the [pointerStartLocation], [pointerStartId]
  /// [pointerEndLocation] and [pointerEndId] must be null. [pointerStartId] and [pointerEndId]
  /// should be different.
  _LineBetweenPointers({
    this.pointerStartLocation = Offset.zero,
    this.pointerStartId = 0,
    this.pointerEndLocation = Offset.zero,
    this.pointerEndId = 1,
  }) : assert(pointerStartId != pointerEndId);

  // The location and the id of the pointer that marks the start of the line.
  final Offset pointerStartLocation;
  final int pointerStartId;

  // The location and the id of the pointer that marks the end of the line.
  final Offset pointerEndLocation;
  final int pointerEndId;
}

/// Recognizes a scale gesture.
///
/// [ScaleGestureRecognizer] tracks the pointers in contact with the screen and
/// calculates their focal point, indicated scale, and rotation. When a focal
/// point is established, the recognizer calls [onStart]. As the focal point,
/// scale, and rotation change, the recognizer calls [onUpdate]. When the
/// pointers are no longer in contact with the screen, the recognizer calls
/// [onEnd].
class ScaleGestureRecognizer extends OneSequenceGestureRecognizer {
  /// Create a gesture recognizer for interactions intended for scaling content.
  ///
  /// {@macro flutter.gestures.GestureRecognizer.supportedDevices}
  ScaleGestureRecognizer({
    super.debugOwner,
    super.supportedDevices,
    super.allowedButtonsFilter,
    this.dragStartBehavior = DragStartBehavior.down,
    this.trackpadScrollCausesScale = false,
    this.trackpadScrollToScaleFactor = kDefaultTrackpadScrollToScaleFactor,
  });

  /// Determines what point is used as the starting point in all calculations
  /// involving this gesture.
  ///
  /// When set to [DragStartBehavior.down], the scale is calculated starting
  /// from the position where the pointer first contacted the screen.
  ///
  /// When set to [DragStartBehavior.start], the scale is calculated starting
  /// from the position where the scale gesture began. The scale gesture may
  /// begin after the time that the pointer first contacted the screen if there
  /// are multiple listeners competing for the gesture. In that case, the
  /// gesture arena waits to determine whether or not the gesture is a scale
  /// gesture before giving the gesture to this GestureRecognizer. This happens
  /// in the case of nested GestureDetectors, for example.
  ///
  /// Defaults to [DragStartBehavior.down].
  ///
  /// See also:
  ///
  /// * https://flutter.dev/to/gesture-disambiguation,
  ///   which provides more information about the gesture arena.
  DragStartBehavior dragStartBehavior;

  /// The pointers in contact with the screen have established a focal point and
  /// initial scale of 1.0.
  ///
  /// This won't be called until the gesture arena has determined that this
  /// GestureRecognizer has won the gesture.
  ///
  /// See also:
  ///
  /// * https://flutter.dev/to/gesture-disambiguation,
  ///   which provides more information about the gesture arena.
  GestureScaleStartCallback? onStart;

  /// The pointers in contact with the screen have indicated a new focal point
  /// and/or scale.
  GestureScaleUpdateCallback? onUpdate;

  /// The pointers are no longer in contact with the screen.
  GestureScaleEndCallback? onEnd;

  _ScaleState _state = _ScaleState.ready;

  Matrix4? _lastTransform;

  /// {@template flutter.gestures.scale.trackpadScrollCausesScale}
  /// Whether scrolling up/down on a trackpad should cause scaling instead of
  /// panning.
  ///
  /// Defaults to false.
  /// {@endtemplate}
  bool trackpadScrollCausesScale;

  /// {@template flutter.gestures.scale.trackpadScrollToScaleFactor}
  /// A factor to control the direction and magnitude of scale when converting
  /// trackpad scrolling.
  ///
  /// Incoming trackpad pan offsets will be divided by this factor to get scale
  /// values. Increasing this offset will reduce the amount of scaling caused by
  /// a fixed amount of trackpad scrolling.
  ///
  /// Defaults to [kDefaultTrackpadScrollToScaleFactor].
  /// {@endtemplate}
  Offset trackpadScrollToScaleFactor;

  /// The number of pointers being tracked by the gesture recognizer.
  ///
  /// Typically this is the number of fingers being used to pan the widget using the gesture
  /// recognizer.
  int get pointerCount {
    // PointerPanZoom protocol doesn't contain the exact number of pointers
    // used on the trackpad, as it isn't exposed by all platforms. However, it
    // will always be at least two.
    return (2 * _pointerPanZooms.length) + _pointerQueue.length;
  }

  late Offset _initialFocalPoint;
  Offset? _currentFocalPoint;
  late double _initialSpan;
  late double _currentSpan;
  late double _initialHorizontalSpan;
  late double _currentHorizontalSpan;
  late double _initialVerticalSpan;
  late double _currentVerticalSpan;
  late Offset _localFocalPoint;
  _LineBetweenPointers? _initialLine;
  _LineBetweenPointers? _currentLine;
  final Map<int, Offset> _pointerLocations = <int, Offset>{};
  final List<int> _pointerQueue = <int>[]; // A queue to sort pointers in order of entrance
  final Map<int, VelocityTracker> _velocityTrackers = <int, VelocityTracker>{};
  VelocityTracker? _scaleVelocityTracker;
  late Offset _delta;
  final Map<int, _PointerPanZoomData> _pointerPanZooms = <int, _PointerPanZoomData>{};
  double _initialPanZoomScaleFactor = 1;
  double _initialPanZoomRotationFactor = 0;
  Duration? _initialEventTimestamp;

  double get _pointerScaleFactor => _initialSpan > 0.0 ? _currentSpan / _initialSpan : 1.0;

  double get _pointerHorizontalScaleFactor =>
      _initialHorizontalSpan > 0.0 ? _currentHorizontalSpan / _initialHorizontalSpan : 1.0;

  double get _pointerVerticalScaleFactor =>
      _initialVerticalSpan > 0.0 ? _currentVerticalSpan / _initialVerticalSpan : 1.0;

  double get _scaleFactor {
    double scale = _pointerScaleFactor;
    for (final _PointerPanZoomData p in _pointerPanZooms.values) {
      scale *= p.scale / _initialPanZoomScaleFactor;
    }
    return scale;
  }

  double get _horizontalScaleFactor {
    double scale = _pointerHorizontalScaleFactor;
    for (final _PointerPanZoomData p in _pointerPanZooms.values) {
      scale *= p.scale / _initialPanZoomScaleFactor;
    }
    return scale;
  }

  double get _verticalScaleFactor {
    double scale = _pointerVerticalScaleFactor;
    for (final _PointerPanZoomData p in _pointerPanZooms.values) {
      scale *= p.scale / _initialPanZoomScaleFactor;
    }
    return scale;
  }

  double _computeRotationFactor() {
    double factor = 0.0;
    if (_initialLine != null && _currentLine != null) {
      final double fx = _initialLine!.pointerStartLocation.dx;
      final double fy = _initialLine!.pointerStartLocation.dy;
      final double sx = _initialLine!.pointerEndLocation.dx;
      final double sy = _initialLine!.pointerEndLocation.dy;

      final double nfx = _currentLine!.pointerStartLocation.dx;
      final double nfy = _currentLine!.pointerStartLocation.dy;
      final double nsx = _currentLine!.pointerEndLocation.dx;
      final double nsy = _currentLine!.pointerEndLocation.dy;

      final double angle1 = math.atan2(fy - sy, fx - sx);
      final double angle2 = math.atan2(nfy - nsy, nfx - nsx);

      factor = angle2 - angle1;
    }
    for (final _PointerPanZoomData p in _pointerPanZooms.values) {
      factor += p.rotation;
    }
    factor -= _initialPanZoomRotationFactor;
    return factor;
  }

  @override
  void addAllowedPointer(PointerDownEvent event) {
    super.addAllowedPointer(event);
    _velocityTrackers[event.pointer] = VelocityTracker.withKind(event.kind);
    _initialEventTimestamp = event.timeStamp;
    if (_state == _ScaleState.ready) {
      _state = _ScaleState.possible;
      _initialSpan = 0.0;
      _currentSpan = 0.0;
      _initialHorizontalSpan = 0.0;
      _currentHorizontalSpan = 0.0;
      _initialVerticalSpan = 0.0;
      _currentVerticalSpan = 0.0;
    }
  }

  @override
  bool isPointerPanZoomAllowed(PointerPanZoomStartEvent event) => true;

  @override
  void addAllowedPointerPanZoom(PointerPanZoomStartEvent event) {
    super.addAllowedPointerPanZoom(event);
    startTrackingPointer(event.pointer, event.transform);
    _velocityTrackers[event.pointer] = VelocityTracker.withKind(event.kind);
    _initialEventTimestamp = event.timeStamp;
    if (_state == _ScaleState.ready) {
      _state = _ScaleState.possible;
      _initialPanZoomScaleFactor = 1.0;
      _initialPanZoomRotationFactor = 0.0;
    }
  }

  @override
  void handleEvent(PointerEvent event) {
    assert(_state != _ScaleState.ready);
    bool didChangeConfiguration = false;
    bool shouldStartIfAccepted = false;
    if (event is PointerMoveEvent) {
      final VelocityTracker tracker = _velocityTrackers[event.pointer]!;
      if (!event.synthesized) {
        tracker.addPosition(event.timeStamp, event.position);
      }
      _pointerLocations[event.pointer] = event.position;
      shouldStartIfAccepted = true;
      _lastTransform = event.transform;
    } else if (event is PointerDownEvent) {
      _pointerLocations[event.pointer] = event.position;
      _pointerQueue.add(event.pointer);
      didChangeConfiguration = true;
      shouldStartIfAccepted = true;
      _lastTransform = event.transform;
    } else if (event is PointerUpEvent || event is PointerCancelEvent) {
      _pointerLocations.remove(event.pointer);
      _pointerQueue.remove(event.pointer);
      didChangeConfiguration = true;
      _lastTransform = event.transform;
    } else if (event is PointerPanZoomStartEvent) {
      assert(_pointerPanZooms[event.pointer] == null);
      _pointerPanZooms[event.pointer] = _PointerPanZoomData.fromStartEvent(this, event);
      didChangeConfiguration = true;
      shouldStartIfAccepted = true;
      _lastTransform = event.transform;
    } else if (event is PointerPanZoomUpdateEvent) {
      assert(_pointerPanZooms[event.pointer] != null);
      if (!event.synthesized && !trackpadScrollCausesScale) {
        _velocityTrackers[event.pointer]!.addPosition(event.timeStamp, event.pan);
      }
      _pointerPanZooms[event.pointer] = _PointerPanZoomData.fromUpdateEvent(this, event);
      _lastTransform = event.transform;
      shouldStartIfAccepted = true;
    } else if (event is PointerPanZoomEndEvent) {
      assert(_pointerPanZooms[event.pointer] != null);
      _pointerPanZooms.remove(event.pointer);
      didChangeConfiguration = true;
    }

    _updateLines();
    _update();

    if (!didChangeConfiguration || _reconfigure(event.pointer)) {
      _advanceStateMachine(shouldStartIfAccepted, event);
    }
    stopTrackingIfPointerNoLongerDown(event);
  }

  void _update() {
    final Offset? previousFocalPoint = _currentFocalPoint;

    // Compute the focal point
    Offset focalPoint = Offset.zero;
    for (final int pointer in _pointerLocations.keys) {
      focalPoint += _pointerLocations[pointer]!;
    }
    for (final _PointerPanZoomData p in _pointerPanZooms.values) {
      focalPoint += p.focalPoint;
    }
    _currentFocalPoint =
        focalPoint / math.max(1, _pointerLocations.length + _pointerPanZooms.length).toDouble();

    if (previousFocalPoint == null) {
      _localFocalPoint = PointerEvent.transformPosition(_lastTransform, _currentFocalPoint!);
      _delta = Offset.zero;
    } else {
      final Offset localPreviousFocalPoint = _localFocalPoint;
      _localFocalPoint = PointerEvent.transformPosition(_lastTransform, _currentFocalPoint!);
      _delta = _localFocalPoint - localPreviousFocalPoint;
    }

    final int count = _pointerLocations.keys.length;

    Offset pointerFocalPoint = Offset.zero;
    for (final int pointer in _pointerLocations.keys) {
      pointerFocalPoint += _pointerLocations[pointer]!;
    }
    if (count > 0) {
      pointerFocalPoint = pointerFocalPoint / count.toDouble();
    }

    // Span is the average deviation from focal point. Horizontal and vertical
    // spans are the average deviations from the focal point's horizontal and
    // vertical coordinates, respectively.
    double totalDeviation = 0.0;
    double totalHorizontalDeviation = 0.0;
    double totalVerticalDeviation = 0.0;
    for (final int pointer in _pointerLocations.keys) {
      totalDeviation += (pointerFocalPoint - _pointerLocations[pointer]!).distance;
      totalHorizontalDeviation += (pointerFocalPoint.dx - _pointerLocations[pointer]!.dx).abs();
      totalVerticalDeviation += (pointerFocalPoint.dy - _pointerLocations[pointer]!.dy).abs();
    }
    _currentSpan = count > 0 ? totalDeviation / count : 0.0;
    _currentHorizontalSpan = count > 0 ? totalHorizontalDeviation / count : 0.0;
    _currentVerticalSpan = count > 0 ? totalVerticalDeviation / count : 0.0;
  }

  /// Updates [_initialLine] and [_currentLine] accordingly to the situation of
  /// the registered pointers.
  void _updateLines() {
    final int count = _pointerLocations.keys.length;
    assert(_pointerQueue.length >= count);

    /// In case of just one pointer registered, reconfigure [_initialLine]
    if (count < 2) {
      _initialLine = _currentLine;
    } else if (_initialLine != null &&
        _initialLine!.pointerStartId == _pointerQueue[0] &&
        _initialLine!.pointerEndId == _pointerQueue[1]) {
      /// Rotation updated, set the [_currentLine]
      _currentLine = _LineBetweenPointers(
        pointerStartId: _pointerQueue[0],
        pointerStartLocation: _pointerLocations[_pointerQueue[0]]!,
        pointerEndId: _pointerQueue[1],
        pointerEndLocation: _pointerLocations[_pointerQueue[1]]!,
      );
    } else {
      /// A new rotation process is on the way, set the [_initialLine]
      _initialLine = _LineBetweenPointers(
        pointerStartId: _pointerQueue[0],
        pointerStartLocation: _pointerLocations[_pointerQueue[0]]!,
        pointerEndId: _pointerQueue[1],
        pointerEndLocation: _pointerLocations[_pointerQueue[1]]!,
      );
      _currentLine = _initialLine;
    }
  }

  bool _reconfigure(int pointer) {
    _initialFocalPoint = _currentFocalPoint!;
    _initialSpan = _currentSpan;
    _initialLine = _currentLine;
    _initialHorizontalSpan = _currentHorizontalSpan;
    _initialVerticalSpan = _currentVerticalSpan;
    if (_pointerPanZooms.isEmpty) {
      _initialPanZoomScaleFactor = 1.0;
      _initialPanZoomRotationFactor = 0.0;
    } else {
      _initialPanZoomScaleFactor = _scaleFactor / _pointerScaleFactor;
      _initialPanZoomRotationFactor = _pointerPanZooms.values
          .map((_PointerPanZoomData x) => x.rotation)
          .reduce((double a, double b) => a + b);
    }
    if (_state == _ScaleState.started) {
      if (onEnd != null) {
        final VelocityTracker tracker = _velocityTrackers[pointer]!;

        Velocity velocity = tracker.getVelocity();
        if (_isFlingGesture(velocity)) {
          final Offset pixelsPerSecond = velocity.pixelsPerSecond;
          if (pixelsPerSecond.distanceSquared > kMaxFlingVelocity * kMaxFlingVelocity) {
            velocity = Velocity(
              pixelsPerSecond: (pixelsPerSecond / pixelsPerSecond.distance) * kMaxFlingVelocity,
            );
          }
          invokeCallback<void>(
            'onEnd',
            () => onEnd!(
              ScaleEndDetails(
                velocity: velocity,
                scaleVelocity: _scaleVelocityTracker?.getVelocity().pixelsPerSecond.dx ?? -1,
                pointerCount: pointerCount,
              ),
            ),
          );
        } else {
          invokeCallback<void>(
            'onEnd',
            () => onEnd!(
              ScaleEndDetails(
                scaleVelocity: _scaleVelocityTracker?.getVelocity().pixelsPerSecond.dx ?? -1,
                pointerCount: pointerCount,
              ),
            ),
          );
        }
      }
      _state = _ScaleState.accepted;
      _scaleVelocityTracker = VelocityTracker.withKind(
        PointerDeviceKind.touch,
      ); // arbitrary PointerDeviceKind
      return false;
    }
    _scaleVelocityTracker = VelocityTracker.withKind(
      PointerDeviceKind.touch,
    ); // arbitrary PointerDeviceKind
    return true;
  }

  void _advanceStateMachine(bool shouldStartIfAccepted, PointerEvent event) {
    if (_state == _ScaleState.ready) {
      _state = _ScaleState.possible;
    }

    if (_state == _ScaleState.possible) {
      final double spanDelta = (_currentSpan - _initialSpan).abs();
      final double focalPointDelta = (_currentFocalPoint! - _initialFocalPoint).distance;
      if (spanDelta > computeScaleSlop(event.kind) ||
          focalPointDelta > computePanSlop(event.kind, gestureSettings) ||
          math.max(_scaleFactor / _pointerScaleFactor, _pointerScaleFactor / _scaleFactor) > 1.05) {
        resolve(GestureDisposition.accepted);
      }
    } else if (_state.index >= _ScaleState.accepted.index) {
      resolve(GestureDisposition.accepted);
    }

    if (_state == _ScaleState.accepted && shouldStartIfAccepted) {
      _initialEventTimestamp = event.timeStamp;
      _state = _ScaleState.started;
      _dispatchOnStartCallbackIfNeeded();
    }

    if (_state == _ScaleState.started) {
      _scaleVelocityTracker?.addPosition(event.timeStamp, Offset(_scaleFactor, 0));
      if (onUpdate != null) {
        invokeCallback<void>('onUpdate', () {
          onUpdate!(
            ScaleUpdateDetails(
              scale: _scaleFactor,
              horizontalScale: _horizontalScaleFactor,
              verticalScale: _verticalScaleFactor,
              focalPoint: _currentFocalPoint!,
              localFocalPoint: _localFocalPoint,
              rotation: _computeRotationFactor(),
              pointerCount: pointerCount,
              focalPointDelta: _delta,
              sourceTimeStamp: event.timeStamp,
            ),
          );
        });
      }
    }
  }

  void _dispatchOnStartCallbackIfNeeded() {
    assert(_state == _ScaleState.started);
    if (onStart != null) {
      invokeCallback<void>('onStart', () {
        onStart!(
          ScaleStartDetails(
            focalPoint: _currentFocalPoint!,
            localFocalPoint: _localFocalPoint,
            pointerCount: pointerCount,
            sourceTimeStamp: _initialEventTimestamp,
          ),
        );
      });
    }
    _initialEventTimestamp = null;
  }

  @override
  void acceptGesture(int pointer) {
    if (_state == _ScaleState.possible) {
      _state = _ScaleState.started;
      _dispatchOnStartCallbackIfNeeded();
      if (dragStartBehavior == DragStartBehavior.start) {
        _initialFocalPoint = _currentFocalPoint!;
        _initialSpan = _currentSpan;
        _initialLine = _currentLine;
        _initialHorizontalSpan = _currentHorizontalSpan;
        _initialVerticalSpan = _currentVerticalSpan;
        if (_pointerPanZooms.isEmpty) {
          _initialPanZoomScaleFactor = 1.0;
          _initialPanZoomRotationFactor = 0.0;
        } else {
          _initialPanZoomScaleFactor = _scaleFactor / _pointerScaleFactor;
          _initialPanZoomRotationFactor = _pointerPanZooms.values
              .map((_PointerPanZoomData x) => x.rotation)
              .reduce((double a, double b) => a + b);
        }
      }
    }
  }

  @override
  void rejectGesture(int pointer) {
    _pointerPanZooms.remove(pointer);
    _pointerLocations.remove(pointer);
    _pointerQueue.remove(pointer);
    stopTrackingPointer(pointer);
  }

  @override
  void didStopTrackingLastPointer(int pointer) {
    switch (_state) {
      case _ScaleState.possible:
        resolve(GestureDisposition.rejected);
      case _ScaleState.ready:
        assert(false); // We should have not seen a pointer yet
      case _ScaleState.accepted:
        break;
      case _ScaleState.started:
        assert(false); // We should be in the accepted state when user is done
    }
    _state = _ScaleState.ready;
  }

  @override
  void dispose() {
    _velocityTrackers.clear();
    super.dispose();
  }

  @override
  String get debugDescription => 'scale';
}
