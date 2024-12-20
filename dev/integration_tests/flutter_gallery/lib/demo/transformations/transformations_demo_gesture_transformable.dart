// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;
import 'transformations_demo_inertial_motion.dart';

// This widget allows 2D transform interactions on its child in relation to its
// parent. The user can transform the child by dragging to pan or pinching to
// zoom and rotate. All event callbacks for GestureDetector are supported, and
// the coordinates that are given are untransformed and in relation to the
// original position of the child.
@immutable
class GestureTransformable extends StatefulWidget {
  const GestureTransformable({
    super.key,
    // The child to perform the transformations on.
    required this.child,
    // The desired visible size of the widget and the area that is receptive to
    // gestures. If a widget that's as big as possible is desired, then wrap
    // this in a LayoutBuilder and pass
    // `Size(constraints.maxWidth, constraints.maxHeight)`.
    required this.size,
    // The scale will be clamped to between these values. A maxScale of null has
    // no bounds. minScale must be greater than zero.
    this.maxScale = 2.5,
    this.minScale = 0.8,
    // Transforms will be limited so that the viewport can not view beyond this
    // Rect. The Rect does not rotate with the rest of the scene, so it is
    // always aligned with the viewport. A null boundaryRect results in no
    // limits to the distance that the viewport can be transformed to see.
    this.boundaryRect,
    // Initial values for the transform can be provided.
    this.initialTranslation,
    this.initialScale,
    this.initialRotation,
    // Any and all of the possible transformations can be disabled.
    this.disableTranslation = false,
    this.disableScale = false,
    this.disableRotation = false,
    // If set to true, this widget will animate back to its initial transform
    // and call onResetEnd when done. When utilizing reset, onResetEnd should
    // also be implemented, and it should set reset to false when called.
    this.reset = false,
    // Access to event callbacks from GestureDetector. Called with untransformed
    // coordinates in an Offset.
    this.onTapDown,
    this.onTapUp,
    this.onTap,
    this.onTapCancel,
    this.onDoubleTap,
    this.onLongPress,
    this.onLongPressUp,
    this.onVerticalDragDown,
    this.onVerticalDragStart,
    this.onVerticalDragUpdate,
    this.onVerticalDragEnd,
    this.onVerticalDragCancel,
    this.onHorizontalDragDown,
    this.onHorizontalDragStart,
    this.onHorizontalDragUpdate,
    this.onHorizontalDragEnd,
    this.onHorizontalDragCancel,
    this.onPanDown,
    this.onPanStart,
    this.onPanUpdate,
    this.onPanEnd,
    this.onPanCancel,
    this.onResetEnd,
    this.onScaleStart,
    this.onScaleUpdate,
    this.onScaleEnd,
  }) : assert(minScale > 0),
       assert(!reset || onResetEnd != null, 'Must implement onResetEnd to use reset.');

  final Widget child;
  final Size size;
  final bool reset;
  final GestureTapDownCallback? onTapDown;
  final GestureTapUpCallback? onTapUp;
  final GestureTapCallback? onTap;
  final GestureTapCancelCallback? onTapCancel;
  final GestureTapCallback? onDoubleTap;
  final GestureLongPressCallback? onLongPress;
  final GestureLongPressUpCallback? onLongPressUp;
  final GestureDragDownCallback? onVerticalDragDown;
  final GestureDragStartCallback? onVerticalDragStart;
  final GestureDragUpdateCallback? onVerticalDragUpdate;
  final GestureDragEndCallback? onVerticalDragEnd;
  final GestureDragCancelCallback? onVerticalDragCancel;
  final GestureDragDownCallback? onHorizontalDragDown;
  final GestureDragStartCallback? onHorizontalDragStart;
  final GestureDragUpdateCallback? onHorizontalDragUpdate;
  final GestureDragEndCallback? onHorizontalDragEnd;
  final GestureDragCancelCallback? onHorizontalDragCancel;
  final GestureDragDownCallback? onPanDown;
  final GestureDragStartCallback? onPanStart;
  final GestureDragUpdateCallback? onPanUpdate;
  final GestureDragEndCallback? onPanEnd;
  final GestureDragCancelCallback? onPanCancel;
  final VoidCallback? onResetEnd;
  final GestureScaleStartCallback? onScaleStart;
  final GestureScaleUpdateCallback? onScaleUpdate;
  final GestureScaleEndCallback? onScaleEnd;
  final double maxScale;
  final double minScale;
  final Rect? boundaryRect;
  final bool disableTranslation;
  final bool disableScale;
  final bool disableRotation;
  final Offset? initialTranslation;
  final double? initialScale;
  final double? initialRotation;

  @override
  State<GestureTransformable> createState() => _GestureTransformableState();
}

// A single user event can only represent one of these gestures. The user can't
// do multiple at the same time, which results in more precise transformations.
enum _GestureType { translate, scale, rotate }

// This is public only for access from a unit test.
class _GestureTransformableState extends State<GestureTransformable> with TickerProviderStateMixin {
  Animation<Offset>? _animation;
  late AnimationController _controller;
  Animation<Matrix4>? _animationReset;
  late AnimationController _controllerReset;
  // The translation that will be applied to the scene (not viewport).
  // A positive x offset moves the scene right, viewport left.
  // A positive y offset moves the scene down, viewport up.
  Offset? _translateFromScene; // Point where a single translation began.
  double? _scaleStart; // Scale value at start of scaling gesture.
  double? _rotationStart = 0.0; // Rotation at start of rotation gesture.
  late Rect _boundaryRect;
  Matrix4 _transform = Matrix4.identity();
  double _currentRotation = 0.0;
  _GestureType? gestureType;

  // The transformation matrix that gives the initial home position.
  Matrix4 get _initialTransform {
    Matrix4 matrix = Matrix4.identity();
    if (widget.initialTranslation != null) {
      matrix = matrixTranslate(matrix, widget.initialTranslation);
    }
    if (widget.initialScale != null) {
      matrix = matrixScale(matrix, widget.initialScale);
    }
    if (widget.initialRotation != null) {
      matrix = matrixRotate(matrix, widget.initialRotation, Offset.zero);
    }
    return matrix;
  }

  // Return the scene point at the given viewport point.
  static Offset fromViewport(Offset viewportPoint, Matrix4 transform) {
    // On viewportPoint, perform the inverse transformation of the scene to get
    // where the point would be in the scene before the transformation.
    final Matrix4 inverseMatrix = Matrix4.inverted(transform);
    final Vector3 untransformed = inverseMatrix.transform3(
      Vector3(viewportPoint.dx, viewportPoint.dy, 0),
    );
    return Offset(untransformed.x, untransformed.y);
  }

  // Get the offset of the current widget from the global screen coordinates.
  static Offset getOffset(BuildContext context) {
    assert(
      context.findRenderObject() != null,
      'The given context must have a renderObject, such as after the first build has completed.',
    );
    final RenderBox renderObject = context.findRenderObject()! as RenderBox;
    return renderObject.localToGlobal(Offset.zero);
  }

  @override
  void initState() {
    super.initState();
    _boundaryRect = widget.boundaryRect ?? Offset.zero & widget.size;
    _transform = _initialTransform;
    _controller = AnimationController(vsync: this);
    _controllerReset = AnimationController(vsync: this);
    if (widget.reset) {
      _animateResetInitialize();
    }
  }

  @override
  void didUpdateWidget(GestureTransformable oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.reset && !oldWidget.reset && _animationReset == null) {
      _animateResetInitialize();
    } else if (!widget.reset && oldWidget.reset && _animationReset != null) {
      _animateResetStop();
    }
  }

  @override
  Widget build(BuildContext context) {
    // A GestureDetector allows the detection of panning and zooming gestures on
    // its child, which is the CustomPaint.
    return GestureDetector(
      behavior: HitTestBehavior.opaque, // Necessary when translating off screen
      onTapDown:
          widget.onTapDown == null
              ? null
              : (TapDownDetails details) {
                widget.onTapDown!(
                  TapDownDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onTapUp:
          widget.onTapUp == null
              ? null
              : (TapUpDetails details) {
                widget.onTapUp!(
                  TapUpDetails(
                    kind: details.kind,
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onTap: widget.onTap,
      onTapCancel: widget.onTapCancel,
      onDoubleTap: widget.onDoubleTap,
      onLongPress: widget.onLongPress,
      onLongPressUp: widget.onLongPressUp,
      onVerticalDragDown:
          widget.onVerticalDragDown == null
              ? null
              : (DragDownDetails details) {
                widget.onVerticalDragDown!(
                  DragDownDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onVerticalDragStart:
          widget.onVerticalDragStart == null
              ? null
              : (DragStartDetails details) {
                widget.onVerticalDragStart!(
                  DragStartDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onVerticalDragUpdate:
          widget.onVerticalDragUpdate == null
              ? null
              : (DragUpdateDetails details) {
                widget.onVerticalDragUpdate!(
                  DragUpdateDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onVerticalDragEnd: widget.onVerticalDragEnd,
      onVerticalDragCancel: widget.onVerticalDragCancel,
      onHorizontalDragDown:
          widget.onHorizontalDragDown == null
              ? null
              : (DragDownDetails details) {
                widget.onHorizontalDragDown!(
                  DragDownDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onHorizontalDragStart:
          widget.onHorizontalDragStart == null
              ? null
              : (DragStartDetails details) {
                widget.onHorizontalDragStart!(
                  DragStartDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onHorizontalDragUpdate:
          widget.onHorizontalDragUpdate == null
              ? null
              : (DragUpdateDetails details) {
                widget.onHorizontalDragUpdate!(
                  DragUpdateDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onHorizontalDragEnd: widget.onHorizontalDragEnd,
      onHorizontalDragCancel: widget.onHorizontalDragCancel,
      onPanDown:
          widget.onPanDown == null
              ? null
              : (DragDownDetails details) {
                widget.onPanDown!(
                  DragDownDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onPanStart:
          widget.onPanStart == null
              ? null
              : (DragStartDetails details) {
                widget.onPanStart!(
                  DragStartDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onPanUpdate:
          widget.onPanUpdate == null
              ? null
              : (DragUpdateDetails details) {
                widget.onPanUpdate!(
                  DragUpdateDetails(
                    globalPosition: fromViewport(
                      details.globalPosition - getOffset(context),
                      _transform,
                    ),
                  ),
                );
              },
      onPanEnd: widget.onPanEnd,
      onPanCancel: widget.onPanCancel,
      onScaleEnd: _onScaleEnd,
      onScaleStart: _onScaleStart,
      onScaleUpdate: _onScaleUpdate,
      child: ClipRect(
        // The scene is panned/zoomed/rotated using this Transform widget.
        child: Transform(
          transform: _transform,
          child: SizedBox(
            height: widget.size.height,
            width: widget.size.width,
            child: widget.child,
          ),
        ),
      ),
    );
  }

  // Return a new matrix representing the given matrix after applying the given
  // translation.
  Matrix4 matrixTranslate(Matrix4 matrix, Offset? translation) {
    if (widget.disableTranslation || translation == Offset.zero) {
      return matrix;
    }

    // Clamp translation so the viewport remains inside _boundaryRect.
    final double scale = _transform.getMaxScaleOnAxis();
    final Size scaledSize = widget.size / scale;
    final Rect viewportBoundaries = Rect.fromLTRB(
      _boundaryRect.left,
      _boundaryRect.top,
      _boundaryRect.right - scaledSize.width,
      _boundaryRect.bottom - scaledSize.height,
    );
    // Translation is reversed (a positive translation moves the scene to the
    // right, viewport to the left).
    final Rect translationBoundaries = Rect.fromLTRB(
      -scale * viewportBoundaries.right,
      -scale * viewportBoundaries.bottom,
      -scale * viewportBoundaries.left,
      -scale * viewportBoundaries.top,
    );
    final Matrix4 nextMatrix = matrix.clone()..translate(translation!.dx, translation.dy);
    final Vector3 nextTranslationVector = nextMatrix.getTranslation();
    final Offset nextTranslation = Offset(nextTranslationVector.x, nextTranslationVector.y);
    final bool inBoundaries = translationBoundaries.contains(
      Offset(nextTranslation.dx, nextTranslation.dy),
    );
    if (!inBoundaries) {
      return matrix;
    }

    return nextMatrix;
  }

  // Return a new matrix representing the given matrix after applying the given
  // scale transform.
  Matrix4 matrixScale(Matrix4 matrix, double? scale) {
    if (widget.disableScale || scale == 1) {
      return matrix;
    }
    assert(scale != 0);

    // Don't allow a scale that moves the viewport outside of _boundaryRect.
    final Offset tl = fromViewport(Offset.zero, _transform);
    final Offset tr = fromViewport(Offset(widget.size.width, 0), _transform);
    final Offset bl = fromViewport(Offset(0, widget.size.height), _transform);
    final Offset br = fromViewport(Offset(widget.size.width, widget.size.height), _transform);
    if (!_boundaryRect.contains(tl) ||
        !_boundaryRect.contains(tr) ||
        !_boundaryRect.contains(bl) ||
        !_boundaryRect.contains(br)) {
      return matrix;
    }

    // Don't allow a scale that results in an overall scale beyond min/max
    // scale.
    final double currentScale = _transform.getMaxScaleOnAxis();
    final double totalScale = currentScale * scale!;
    final double clampedTotalScale = totalScale.clamp(widget.minScale, widget.maxScale);
    final double clampedScale = clampedTotalScale / currentScale;
    return matrix..scale(clampedScale);
  }

  // Return a new matrix representing the given matrix after applying the given
  // rotation transform.
  // Rotating the scene cannot cause the viewport to view beyond _boundaryRect.
  Matrix4 matrixRotate(Matrix4 matrix, double? rotation, Offset focalPoint) {
    if (widget.disableRotation || rotation == 0) {
      return matrix;
    }
    final Offset focalPointScene = fromViewport(focalPoint, matrix);
    return matrix
      ..translate(focalPointScene.dx, focalPointScene.dy)
      ..rotateZ(-rotation!)
      ..translate(-focalPointScene.dx, -focalPointScene.dy);
  }

  // Handle the start of a gesture of _GestureType.
  void _onScaleStart(ScaleStartDetails details) {
    widget.onScaleStart?.call(details);

    if (_controller.isAnimating) {
      _controller.stop();
      _controller.reset();
      _animation?.removeListener(_onAnimate);
      _animation = null;
    }
    if (_controllerReset.isAnimating) {
      _animateResetStop();
    }

    gestureType = null;
    setState(() {
      _scaleStart = _transform.getMaxScaleOnAxis();
      _translateFromScene = fromViewport(details.focalPoint, _transform);
      _rotationStart = _currentRotation;
    });
  }

  // Handle an update to an ongoing gesture of _GestureType.
  void _onScaleUpdate(ScaleUpdateDetails details) {
    double scale = _transform.getMaxScaleOnAxis();
    widget.onScaleUpdate?.call(
      ScaleUpdateDetails(
        focalPoint: fromViewport(details.focalPoint, _transform),
        scale: details.scale,
        rotation: details.rotation,
      ),
    );
    final Offset focalPointScene = fromViewport(details.focalPoint, _transform);
    if (gestureType == null) {
      // Decide which type of gesture this is by comparing the amount of scale
      // and rotation in the gesture, if any. Scale starts at 1 and rotation
      // starts at 0. Translate will have 0 scale and 0 rotation because it uses
      // only one finger.
      if ((details.scale - 1).abs() > details.rotation.abs()) {
        gestureType = _GestureType.scale;
      } else if (details.rotation != 0) {
        gestureType = _GestureType.rotate;
      } else {
        gestureType = _GestureType.translate;
      }
    }
    setState(() {
      if (gestureType == _GestureType.scale && _scaleStart != null) {
        // details.scale gives us the amount to change the scale as of the
        // start of this gesture, so calculate the amount to scale as of the
        // previous call to _onScaleUpdate.
        final double desiredScale = _scaleStart! * details.scale;
        final double scaleChange = desiredScale / scale;
        _transform = matrixScale(_transform, scaleChange);
        scale = _transform.getMaxScaleOnAxis();

        // While scaling, translate such that the user's two fingers stay on the
        // same places in the scene. That means that the focal point of the
        // scale should be on the same place in the scene before and after the
        // scale.
        final Offset focalPointSceneNext = fromViewport(details.focalPoint, _transform);
        _transform = matrixTranslate(_transform, focalPointSceneNext - focalPointScene);
      } else if (gestureType == _GestureType.rotate && details.rotation != 0.0) {
        final double desiredRotation = _rotationStart! + details.rotation;
        _transform = matrixRotate(
          _transform,
          _currentRotation - desiredRotation,
          details.focalPoint,
        );
        _currentRotation = desiredRotation;
      } else if (_translateFromScene != null && details.scale == 1.0) {
        // Translate so that the same point in the scene is underneath the
        // focal point before and after the movement.
        final Offset translationChange = focalPointScene - _translateFromScene!;
        _transform = matrixTranslate(_transform, translationChange);
        _translateFromScene = fromViewport(details.focalPoint, _transform);
      }
    });
  }

  // Handle the end of a gesture of _GestureType.
  void _onScaleEnd(ScaleEndDetails details) {
    widget.onScaleEnd?.call(details);
    setState(() {
      _scaleStart = null;
      _rotationStart = null;
      _translateFromScene = null;
    });

    _animation?.removeListener(_onAnimate);
    _controller.reset();

    // If the scale ended with velocity, animate inertial movement
    final double velocityTotal =
        details.velocity.pixelsPerSecond.dx.abs() + details.velocity.pixelsPerSecond.dy.abs();
    if (velocityTotal == 0) {
      return;
    }

    final Vector3 translationVector = _transform.getTranslation();
    final Offset translation = Offset(translationVector.x, translationVector.y);
    final InertialMotion inertialMotion = InertialMotion(details.velocity, translation);
    _animation = Tween<Offset>(
      begin: translation,
      end: inertialMotion.finalPosition,
    ).animate(_controller);
    _controller.duration = Duration(milliseconds: inertialMotion.duration.toInt());
    _animation!.addListener(_onAnimate);
    _controller.fling();
  }

  // Handle inertia drag animation.
  void _onAnimate() {
    setState(() {
      // Translate _transform such that the resulting translation is
      // _animation.value.
      final Vector3 translationVector = _transform.getTranslation();
      final Offset translation = Offset(translationVector.x, translationVector.y);
      final Offset translationScene = fromViewport(translation, _transform);
      final Offset animationScene = fromViewport(_animation!.value, _transform);
      final Offset translationChangeScene = animationScene - translationScene;
      _transform = matrixTranslate(_transform, translationChangeScene);
    });
    if (!_controller.isAnimating) {
      _animation?.removeListener(_onAnimate);
      _animation = null;
      _controller.reset();
    }
  }

  // Handle reset to home transform animation.
  void _onAnimateReset() {
    setState(() {
      _transform = _animationReset!.value;
    });
    if (!_controllerReset.isAnimating) {
      _animationReset?.removeListener(_onAnimateReset);
      _animationReset = null;
      _controllerReset.reset();
      widget.onResetEnd!();
    }
  }

  // Initialize the reset to home transform animation.
  void _animateResetInitialize() {
    _controllerReset.reset();
    _animationReset = Matrix4Tween(
      begin: _transform,
      end: _initialTransform,
    ).animate(_controllerReset);
    _controllerReset.duration = const Duration(milliseconds: 400);
    _animationReset!.addListener(_onAnimateReset);
    _controllerReset.forward();
  }

  // Stop a running reset to home transform animation.
  void _animateResetStop() {
    _controllerReset.stop();
    _animationReset?.removeListener(_onAnimateReset);
    _animationReset = null;
    _controllerReset.reset();
    widget.onResetEnd!();
  }

  @override
  void dispose() {
    _controller.dispose();
    _controllerReset.dispose();
    super.dispose();
  }
}
