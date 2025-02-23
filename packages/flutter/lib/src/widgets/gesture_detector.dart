// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'container.dart';
/// @docImport 'scrollable.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'media_query.dart';
import 'scroll_configuration.dart';

export 'package:flutter/gestures.dart'
    show
        DragDownDetails,
        DragEndDetails,
        DragStartDetails,
        DragUpdateDetails,
        ForcePressDetails,
        GestureDragCancelCallback,
        GestureDragDownCallback,
        GestureDragEndCallback,
        GestureDragStartCallback,
        GestureDragUpdateCallback,
        GestureForcePressEndCallback,
        GestureForcePressPeakCallback,
        GestureForcePressStartCallback,
        GestureForcePressUpdateCallback,
        GestureLongPressCallback,
        GestureLongPressEndCallback,
        GestureLongPressMoveUpdateCallback,
        GestureLongPressStartCallback,
        GestureLongPressUpCallback,
        GestureScaleEndCallback,
        GestureScaleStartCallback,
        GestureScaleUpdateCallback,
        GestureTapCallback,
        GestureTapCancelCallback,
        GestureTapDownCallback,
        GestureTapUpCallback,
        LongPressEndDetails,
        LongPressMoveUpdateDetails,
        LongPressStartDetails,
        ScaleEndDetails,
        ScaleStartDetails,
        ScaleUpdateDetails,
        TapDownDetails,
        TapUpDetails,
        Velocity;
export 'package:flutter/rendering.dart' show RenderSemanticsGestureHandler;

// Examples can assume:
// late bool _lights;
// void setState(VoidCallback fn) { }
// late String _last;
// late Color _color;

/// Factory for creating gesture recognizers.
///
/// `T` is the type of gesture recognizer this class manages.
///
/// Used by [RawGestureDetector.gestures].
@optionalTypeArgs
abstract class GestureRecognizerFactory<T extends GestureRecognizer> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const GestureRecognizerFactory();

  /// Must return an instance of T.
  T constructor();

  /// Must configure the given instance (which will have been created by
  /// `constructor`).
  ///
  /// This normally means setting the callbacks.
  void initializer(T instance);

  bool _debugAssertTypeMatches(Type type) {
    assert(
      type == T,
      'GestureRecognizerFactory of type $T was used where type $type was specified.',
    );
    return true;
  }
}

/// Signature for closures that implement [GestureRecognizerFactory.constructor].
typedef GestureRecognizerFactoryConstructor<T extends GestureRecognizer> = T Function();

/// Signature for closures that implement [GestureRecognizerFactory.initializer].
typedef GestureRecognizerFactoryInitializer<T extends GestureRecognizer> =
    void Function(T instance);

/// Factory for creating gesture recognizers that delegates to callbacks.
///
/// Used by [RawGestureDetector.gestures].
class GestureRecognizerFactoryWithHandlers<T extends GestureRecognizer>
    extends GestureRecognizerFactory<T> {
  /// Creates a gesture recognizer factory with the given callbacks.
  const GestureRecognizerFactoryWithHandlers(this._constructor, this._initializer);

  final GestureRecognizerFactoryConstructor<T> _constructor;

  final GestureRecognizerFactoryInitializer<T> _initializer;

  @override
  T constructor() => _constructor();

  @override
  void initializer(T instance) => _initializer(instance);
}

/// A widget that detects gestures.
///
/// Attempts to recognize gestures that correspond to its non-null callbacks.
///
/// If this widget has a child, it defers to that child for its sizing behavior.
/// If it does not have a child, it grows to fit the parent instead.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=WhVXkCFPmK4}
///
/// By default a GestureDetector with an invisible child ignores touches;
/// this behavior can be controlled with [behavior].
///
/// GestureDetector also listens for accessibility events and maps
/// them to the callbacks. To ignore accessibility events, set
/// [excludeFromSemantics] to true.
///
/// See <http://flutter.dev/to/gestures> for additional information.
///
/// Material design applications typically react to touches with ink splash
/// effects. The [InkWell] class implements this effect and can be used in place
/// of a [GestureDetector] for handling taps.
///
/// {@tool dartpad}
/// This example contains a black light bulb wrapped in a [GestureDetector]. It
/// turns the light bulb yellow when the "TURN LIGHT ON" button is tapped by
/// setting the `_lights` field, and off again when "TURN LIGHT OFF" is tapped.
///
/// ** See code in examples/api/lib/widgets/gesture_detector/gesture_detector.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example uses a [Container] that wraps a [GestureDetector] widget which
/// detects a tap.
///
/// Since the [GestureDetector] does not have a child, it takes on the size of its
/// parent, making the entire area of the surrounding [Container] clickable. When
/// tapped, the [Container] turns yellow by setting the `_color` field. When
/// tapped again, it goes back to white.
///
/// ** See code in examples/api/lib/widgets/gesture_detector/gesture_detector.1.dart **
/// {@end-tool}
///
/// ### Troubleshooting
///
/// Why isn't my parent [GestureDetector.onTap] method called?
///
/// Given a parent [GestureDetector] with an onTap callback, and a child
/// GestureDetector that also defines an onTap callback, when the inner
/// GestureDetector is tapped, both GestureDetectors send a [GestureRecognizer]
/// into the gesture arena. This is because the pointer coordinates are within the
/// bounds of both GestureDetectors. The child GestureDetector wins in this
/// scenario because it was the first to enter the arena, resolving as first come,
/// first served. The child onTap is called, and the parent's is not as the gesture has
/// been consumed.
/// For more information on gesture disambiguation see:
/// [Gesture disambiguation](https://flutter.dev/to/gesture-disambiguation).
///
/// Setting [GestureDetector.behavior] to [HitTestBehavior.opaque]
/// or [HitTestBehavior.translucent] has no impact on parent-child relationships:
/// both GestureDetectors send a GestureRecognizer into the gesture arena, only one wins.
///
/// Some callbacks (e.g. onTapDown) can fire before a recognizer wins the arena,
/// and others (e.g. onTapCancel) fire even when it loses the arena. Therefore,
/// the parent detector in the example above may call some of its callbacks even
/// though it loses in the arena.
///
/// {@tool dartpad}
/// This example uses a [GestureDetector] that wraps a green [Container] and a second
/// GestureDetector that wraps a yellow Container. The second GestureDetector is
/// a child of the green Container.
/// Both GestureDetectors define an onTap callback. When the callback is called it
/// adds a red border to the corresponding Container.
///
/// When the green Container is tapped, it's parent GestureDetector enters
/// the gesture arena. It wins because there is no competing GestureDetector and
/// the green Container shows a red border.
/// When the yellow Container is tapped, it's parent GestureDetector enters
/// the gesture arena. The GestureDetector that wraps the green Container also
/// enters the gesture arena (the pointer events coordinates are inside both
/// GestureDetectors bounds). The GestureDetector that wraps the yellow Container
/// wins because it was the first detector to enter the arena.
///
/// This example sets [debugPrintGestureArenaDiagnostics] to true.
/// This flag prints useful information about gesture arenas.
///
/// Changing the [GestureDetector.behavior] property to [HitTestBehavior.translucent]
/// or [HitTestBehavior.opaque] has no impact: both GestureDetectors send a [GestureRecognizer]
/// into the gesture arena, only one wins.
///
/// ** See code in examples/api/lib/widgets/gesture_detector/gesture_detector.2.dart **
/// {@end-tool}
///
/// ## Debugging
///
/// To see how large the hit test box of a [GestureDetector] is for debugging
/// purposes, set [debugPaintPointersEnabled] to true.
///
/// See also:
///
///  * [Listener], a widget for listening to lower-level raw pointer events.
///  * [MouseRegion], a widget that tracks the movement of mice, even when no
///    button is pressed.
///  * [RawGestureDetector], a widget that is used to detect custom gestures.
class GestureDetector extends StatelessWidget {
  /// Creates a widget that detects gestures.
  ///
  /// Pan and scale callbacks cannot be used simultaneously because scale is a
  /// superset of pan. Use the scale callbacks instead.
  ///
  /// Horizontal and vertical drag callbacks cannot be used simultaneously
  /// because a combination of a horizontal and vertical drag is a pan.
  /// Use the pan callbacks instead.
  ///
  /// {@youtube 560 315 https://www.youtube.com/watch?v=WhVXkCFPmK4}
  ///
  /// By default, gesture detectors contribute semantic information to the tree
  /// that is used by assistive technology.
  GestureDetector({
    super.key,
    this.child,
    this.onTapDown,
    this.onTapUp,
    this.onTap,
    this.onTapCancel,
    this.onSecondaryTap,
    this.onSecondaryTapDown,
    this.onSecondaryTapUp,
    this.onSecondaryTapCancel,
    this.onTertiaryTapDown,
    this.onTertiaryTapUp,
    this.onTertiaryTapCancel,
    this.onDoubleTapDown,
    this.onDoubleTap,
    this.onDoubleTapCancel,
    this.onLongPressDown,
    this.onLongPressCancel,
    this.onLongPress,
    this.onLongPressStart,
    this.onLongPressMoveUpdate,
    this.onLongPressUp,
    this.onLongPressEnd,
    this.onSecondaryLongPressDown,
    this.onSecondaryLongPressCancel,
    this.onSecondaryLongPress,
    this.onSecondaryLongPressStart,
    this.onSecondaryLongPressMoveUpdate,
    this.onSecondaryLongPressUp,
    this.onSecondaryLongPressEnd,
    this.onTertiaryLongPressDown,
    this.onTertiaryLongPressCancel,
    this.onTertiaryLongPress,
    this.onTertiaryLongPressStart,
    this.onTertiaryLongPressMoveUpdate,
    this.onTertiaryLongPressUp,
    this.onTertiaryLongPressEnd,
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
    this.onForcePressStart,
    this.onForcePressPeak,
    this.onForcePressUpdate,
    this.onForcePressEnd,
    this.onPanDown,
    this.onPanStart,
    this.onPanUpdate,
    this.onPanEnd,
    this.onPanCancel,
    this.onScaleStart,
    this.onScaleUpdate,
    this.onScaleEnd,
    this.behavior,
    this.excludeFromSemantics = false,
    this.dragStartBehavior = DragStartBehavior.start,
    this.trackpadScrollCausesScale = false,
    this.trackpadScrollToScaleFactor = kDefaultTrackpadScrollToScaleFactor,
    this.supportedDevices,
  }) : assert(() {
         final bool haveVerticalDrag =
             onVerticalDragStart != null ||
             onVerticalDragUpdate != null ||
             onVerticalDragEnd != null;
         final bool haveHorizontalDrag =
             onHorizontalDragStart != null ||
             onHorizontalDragUpdate != null ||
             onHorizontalDragEnd != null;
         final bool havePan = onPanStart != null || onPanUpdate != null || onPanEnd != null;
         final bool haveScale = onScaleStart != null || onScaleUpdate != null || onScaleEnd != null;
         if (havePan || haveScale) {
           if (havePan && haveScale) {
             throw FlutterError.fromParts(<DiagnosticsNode>[
               ErrorSummary('Incorrect GestureDetector arguments.'),
               ErrorDescription(
                 'Having both a pan gesture recognizer and a scale gesture recognizer is redundant; scale is a superset of pan.',
               ),
               ErrorHint('Just use the scale gesture recognizer.'),
             ]);
           }
           final String recognizer = havePan ? 'pan' : 'scale';
           if (haveVerticalDrag && haveHorizontalDrag) {
             throw FlutterError(
               'Incorrect GestureDetector arguments.\n'
               'Simultaneously having a vertical drag gesture recognizer, a horizontal drag gesture recognizer, and a $recognizer gesture recognizer '
               'will result in the $recognizer gesture recognizer being ignored, since the other two will catch all drags.',
             );
           }
         }
         return true;
       }());

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// A pointer that might cause a tap with a primary button has contacted the
  /// screen at a particular location.
  ///
  /// This is called after a short timeout, even if the winning gesture has not
  /// yet been selected. If the tap gesture wins, [onTapUp] will be called,
  /// otherwise [onTapCancel] will be called.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapDownCallback? onTapDown;

  /// A pointer that will trigger a tap with a primary button has stopped
  /// contacting the screen at a particular location.
  ///
  /// This triggers immediately before [onTap] in the case of the tap gesture
  /// winning. If the tap gesture did not win, [onTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapUpCallback? onTapUp;

  /// A tap with a primary button has occurred.
  ///
  /// This triggers when the tap gesture wins. If the tap gesture did not win,
  /// [onTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onTapUp], which is called at the same time but includes details
  ///    regarding the pointer position.
  final GestureTapCallback? onTap;

  /// The pointer that previously triggered [onTapDown] will not end up causing
  /// a tap.
  ///
  /// This is called after [onTapDown], and instead of [onTapUp] and [onTap], if
  /// the tap gesture did not win.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapCancelCallback? onTapCancel;

  /// A tap with a secondary button has occurred.
  ///
  /// This triggers when the tap gesture wins. If the tap gesture did not win,
  /// [onSecondaryTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onSecondaryTapUp], which is called at the same time but includes details
  ///    regarding the pointer position.
  final GestureTapCallback? onSecondaryTap;

  /// A pointer that might cause a tap with a secondary button has contacted the
  /// screen at a particular location.
  ///
  /// This is called after a short timeout, even if the winning gesture has not
  /// yet been selected. If the tap gesture wins, [onSecondaryTapUp] will be
  /// called, otherwise [onSecondaryTapCancel] will be called.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  final GestureTapDownCallback? onSecondaryTapDown;

  /// A pointer that will trigger a tap with a secondary button has stopped
  /// contacting the screen at a particular location.
  ///
  /// This triggers in the case of the tap gesture winning. If the tap gesture
  /// did not win, [onSecondaryTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [onSecondaryTap], a handler triggered right after this one that doesn't
  ///    pass any details about the tap.
  ///  * [kSecondaryButton], the button this callback responds to.
  final GestureTapUpCallback? onSecondaryTapUp;

  /// The pointer that previously triggered [onSecondaryTapDown] will not end up
  /// causing a tap.
  ///
  /// This is called after [onSecondaryTapDown], and instead of
  /// [onSecondaryTapUp], if the tap gesture did not win.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  final GestureTapCancelCallback? onSecondaryTapCancel;

  /// A pointer that might cause a tap with a tertiary button has contacted the
  /// screen at a particular location.
  ///
  /// This is called after a short timeout, even if the winning gesture has not
  /// yet been selected. If the tap gesture wins, [onTertiaryTapUp] will be
  /// called, otherwise [onTertiaryTapCancel] will be called.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  final GestureTapDownCallback? onTertiaryTapDown;

  /// A pointer that will trigger a tap with a tertiary button has stopped
  /// contacting the screen at a particular location.
  ///
  /// This triggers in the case of the tap gesture winning. If the tap gesture
  /// did not win, [onTertiaryTapCancel] is called instead.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  final GestureTapUpCallback? onTertiaryTapUp;

  /// The pointer that previously triggered [onTertiaryTapDown] will not end up
  /// causing a tap.
  ///
  /// This is called after [onTertiaryTapDown], and instead of
  /// [onTertiaryTapUp], if the tap gesture did not win.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  final GestureTapCancelCallback? onTertiaryTapCancel;

  /// A pointer that might cause a double tap has contacted the screen at a
  /// particular location.
  ///
  /// Triggered immediately after the down event of the second tap.
  ///
  /// If the user completes the double tap and the gesture wins, [onDoubleTap]
  /// will be called after this callback. Otherwise, [onDoubleTapCancel] will
  /// be called after this callback.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapDownCallback? onDoubleTapDown;

  /// The user has tapped the screen with a primary button at the same location
  /// twice in quick succession.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapCallback? onDoubleTap;

  /// The pointer that previously triggered [onDoubleTapDown] will not end up
  /// causing a double tap.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureTapCancelCallback? onDoubleTapCancel;

  /// The pointer has contacted the screen with a primary button, which might
  /// be the start of a long-press.
  ///
  /// This triggers after the pointer down event.
  ///
  /// If the user completes the long-press, and this gesture wins,
  /// [onLongPressStart] will be called after this callback. Otherwise,
  /// [onLongPressCancel] will be called after this callback.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [onSecondaryLongPressDown], a similar callback but for a secondary button.
  ///  * [onTertiaryLongPressDown], a similar callback but for a tertiary button.
  ///  * [LongPressGestureRecognizer.onLongPressDown], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressDownCallback? onLongPressDown;

  /// A pointer that previously triggered [onLongPressDown] will not end up
  /// causing a long-press.
  ///
  /// This triggers once the gesture loses if [onLongPressDown] has previously
  /// been triggered.
  ///
  /// If the user completed the long-press, and the gesture won, then
  /// [onLongPressStart] and [onLongPress] are called instead.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPressCancel], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressCancelCallback? onLongPressCancel;

  /// Called when a long press gesture with a primary button has been recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately after) [onLongPressStart].
  /// The only difference between the two is that this callback does not
  /// contain details of the position at which the pointer initially contacted
  /// the screen.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPress], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressCallback? onLongPress;

  /// Called when a long press gesture with a primary button has been recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately before) [onLongPress].
  /// The only difference between the two is that this callback contains
  /// details of the position at which the pointer initially contacted the
  /// screen, whereas [onLongPress] does not.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPressStart], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressStartCallback? onLongPressStart;

  /// A pointer has been drag-moved after a long-press with a primary button.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPressMoveUpdate], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressMoveUpdateCallback? onLongPressMoveUpdate;

  /// A pointer that has triggered a long-press with a primary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately after) [onLongPressEnd].
  /// The only difference between the two is that this callback does not
  /// contain details of the state of the pointer when it stopped contacting
  /// the screen.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPressUp], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressUpCallback? onLongPressUp;

  /// A pointer that has triggered a long-press with a primary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately before) [onLongPressUp].
  /// The only difference between the two is that this callback contains
  /// details of the state of the pointer when it stopped contacting the
  /// screen, whereas [onLongPressUp] does not.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onLongPressEnd], which exposes this
  ///    callback at the gesture layer.
  final GestureLongPressEndCallback? onLongPressEnd;

  /// The pointer has contacted the screen with a secondary button, which might
  /// be the start of a long-press.
  ///
  /// This triggers after the pointer down event.
  ///
  /// If the user completes the long-press, and this gesture wins,
  /// [onSecondaryLongPressStart] will be called after this callback. Otherwise,
  /// [onSecondaryLongPressCancel] will be called after this callback.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [onLongPressDown], a similar callback but for a secondary button.
  ///  * [onTertiaryLongPressDown], a similar callback but for a tertiary button.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressDown], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressDownCallback? onSecondaryLongPressDown;

  /// A pointer that previously triggered [onSecondaryLongPressDown] will not
  /// end up causing a long-press.
  ///
  /// This triggers once the gesture loses if [onSecondaryLongPressDown] has
  /// previously been triggered.
  ///
  /// If the user completed the long-press, and the gesture won, then
  /// [onSecondaryLongPressStart] and [onSecondaryLongPress] are called instead.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressCancel], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressCancelCallback? onSecondaryLongPressCancel;

  /// Called when a long press gesture with a secondary button has been
  /// recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately after)
  /// [onSecondaryLongPressStart]. The only difference between the two is that
  /// this callback does not contain details of the position at which the
  /// pointer initially contacted the screen.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPress], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressCallback? onSecondaryLongPress;

  /// Called when a long press gesture with a secondary button has been
  /// recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately before)
  /// [onSecondaryLongPress]. The only difference between the two is that this
  /// callback contains details of the position at which the pointer initially
  /// contacted the screen, whereas [onSecondaryLongPress] does not.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressStart], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressStartCallback? onSecondaryLongPressStart;

  /// A pointer has been drag-moved after a long press with a secondary button.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressMoveUpdate], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressMoveUpdateCallback? onSecondaryLongPressMoveUpdate;

  /// A pointer that has triggered a long-press with a secondary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately after)
  /// [onSecondaryLongPressEnd]. The only difference between the two is that
  /// this callback does not contain details of the state of the pointer when
  /// it stopped contacting the screen.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressUp], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressUpCallback? onSecondaryLongPressUp;

  /// A pointer that has triggered a long-press with a secondary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately before)
  /// [onSecondaryLongPressUp]. The only difference between the two is that
  /// this callback contains details of the state of the pointer when it
  /// stopped contacting the screen, whereas [onSecondaryLongPressUp] does not.
  ///
  /// See also:
  ///
  ///  * [kSecondaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onSecondaryLongPressEnd], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressEndCallback? onSecondaryLongPressEnd;

  /// The pointer has contacted the screen with a tertiary button, which might
  /// be the start of a long-press.
  ///
  /// This triggers after the pointer down event.
  ///
  /// If the user completes the long-press, and this gesture wins,
  /// [onTertiaryLongPressStart] will be called after this callback. Otherwise,
  /// [onTertiaryLongPressCancel] will be called after this callback.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [onLongPressDown], a similar callback but for a primary button.
  ///  * [onSecondaryLongPressDown], a similar callback but for a secondary button.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressDown], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressDownCallback? onTertiaryLongPressDown;

  /// A pointer that previously triggered [onTertiaryLongPressDown] will not
  /// end up causing a long-press.
  ///
  /// This triggers once the gesture loses if [onTertiaryLongPressDown] has
  /// previously been triggered.
  ///
  /// If the user completed the long-press, and the gesture won, then
  /// [onTertiaryLongPressStart] and [onTertiaryLongPress] are called instead.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressCancel], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressCancelCallback? onTertiaryLongPressCancel;

  /// Called when a long press gesture with a tertiary button has been
  /// recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately after)
  /// [onTertiaryLongPressStart]. The only difference between the two is that
  /// this callback does not contain details of the position at which the
  /// pointer initially contacted the screen.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPress], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressCallback? onTertiaryLongPress;

  /// Called when a long press gesture with a tertiary button has been
  /// recognized.
  ///
  /// Triggered when a pointer has remained in contact with the screen at the
  /// same location for a long period of time.
  ///
  /// This is equivalent to (and is called immediately before)
  /// [onTertiaryLongPress]. The only difference between the two is that this
  /// callback contains details of the position at which the pointer initially
  /// contacted the screen, whereas [onTertiaryLongPress] does not.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressStart], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressStartCallback? onTertiaryLongPressStart;

  /// A pointer has been drag-moved after a long press with a tertiary button.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressMoveUpdate], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressMoveUpdateCallback? onTertiaryLongPressMoveUpdate;

  /// A pointer that has triggered a long-press with a tertiary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately after)
  /// [onTertiaryLongPressEnd]. The only difference between the two is that
  /// this callback does not contain details of the state of the pointer when
  /// it stopped contacting the screen.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressUp], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressUpCallback? onTertiaryLongPressUp;

  /// A pointer that has triggered a long-press with a tertiary button has
  /// stopped contacting the screen.
  ///
  /// This is equivalent to (and is called immediately before)
  /// [onTertiaryLongPressUp]. The only difference between the two is that
  /// this callback contains details of the state of the pointer when it
  /// stopped contacting the screen, whereas [onTertiaryLongPressUp] does not.
  ///
  /// See also:
  ///
  ///  * [kTertiaryButton], the button this callback responds to.
  ///  * [LongPressGestureRecognizer.onTertiaryLongPressEnd], which exposes
  ///    this callback at the gesture layer.
  final GestureLongPressEndCallback? onTertiaryLongPressEnd;

  /// A pointer has contacted the screen with a primary button and might begin
  /// to move vertically.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragDownCallback? onVerticalDragDown;

  /// A pointer has contacted the screen with a primary button and has begun to
  /// move vertically.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragStartCallback? onVerticalDragStart;

  /// A pointer that is in contact with the screen with a primary button and
  /// moving vertically has moved in the vertical direction.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragUpdateCallback? onVerticalDragUpdate;

  /// A pointer that was previously in contact with the screen with a primary
  /// button and moving vertically is no longer in contact with the screen and
  /// was moving at a specific velocity when it stopped contacting the screen.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragEndCallback? onVerticalDragEnd;

  /// The pointer that previously triggered [onVerticalDragDown] did not
  /// complete.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragCancelCallback? onVerticalDragCancel;

  /// A pointer has contacted the screen with a primary button and might begin
  /// to move horizontally.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragDownCallback? onHorizontalDragDown;

  /// A pointer has contacted the screen with a primary button and has begun to
  /// move horizontally.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragStartCallback? onHorizontalDragStart;

  /// A pointer that is in contact with the screen with a primary button and
  /// moving horizontally has moved in the horizontal direction.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragUpdateCallback? onHorizontalDragUpdate;

  /// A pointer that was previously in contact with the screen with a primary
  /// button and moving horizontally is no longer in contact with the screen and
  /// was moving at a specific velocity when it stopped contacting the screen.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragEndCallback? onHorizontalDragEnd;

  /// The pointer that previously triggered [onHorizontalDragDown] did not
  /// complete.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragCancelCallback? onHorizontalDragCancel;

  /// A pointer has contacted the screen with a primary button and might begin
  /// to move.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragDownCallback? onPanDown;

  /// A pointer has contacted the screen with a primary button and has begun to
  /// move.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragStartCallback? onPanStart;

  /// A pointer that is in contact with the screen with a primary button and
  /// moving has moved again.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragUpdateCallback? onPanUpdate;

  /// A pointer that was previously in contact with the screen with a primary
  /// button and moving is no longer in contact with the screen and was moving
  /// at a specific velocity when it stopped contacting the screen.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragEndCallback? onPanEnd;

  /// The pointer that previously triggered [onPanDown] did not complete.
  ///
  /// See also:
  ///
  ///  * [kPrimaryButton], the button this callback responds to.
  final GestureDragCancelCallback? onPanCancel;

  /// The pointers in contact with the screen have established a focal point and
  /// initial scale of 1.0.
  final GestureScaleStartCallback? onScaleStart;

  /// The pointers in contact with the screen have indicated a new focal point
  /// and/or scale.
  final GestureScaleUpdateCallback? onScaleUpdate;

  /// The pointers are no longer in contact with the screen.
  final GestureScaleEndCallback? onScaleEnd;

  /// The pointer is in contact with the screen and has pressed with sufficient
  /// force to initiate a force press. The amount of force is at least
  /// [ForcePressGestureRecognizer.startPressure].
  ///
  /// This callback will only be fired on devices with pressure
  /// detecting screens.
  final GestureForcePressStartCallback? onForcePressStart;

  /// The pointer is in contact with the screen and has pressed with the maximum
  /// force. The amount of force is at least
  /// [ForcePressGestureRecognizer.peakPressure].
  ///
  /// This callback will only be fired on devices with pressure
  /// detecting screens.
  final GestureForcePressPeakCallback? onForcePressPeak;

  /// A pointer is in contact with the screen, has previously passed the
  /// [ForcePressGestureRecognizer.startPressure] and is either moving on the
  /// plane of the screen, pressing the screen with varying forces or both
  /// simultaneously.
  ///
  /// This callback will only be fired on devices with pressure
  /// detecting screens.
  final GestureForcePressUpdateCallback? onForcePressUpdate;

  /// The pointer tracked by [onForcePressStart] is no longer in contact with the screen.
  ///
  /// This callback will only be fired on devices with pressure
  /// detecting screens.
  final GestureForcePressEndCallback? onForcePressEnd;

  /// How this gesture detector should behave during hit testing when deciding
  /// how the hit test propagates to children and whether to consider targets
  /// behind this one.
  ///
  /// This defaults to [HitTestBehavior.deferToChild] if [child] is not null and
  /// [HitTestBehavior.translucent] if child is null.
  ///
  /// See [HitTestBehavior] for the allowed values and their meanings.
  final HitTestBehavior? behavior;

  /// Whether to exclude these gestures from the semantics tree. For
  /// example, the long-press gesture for showing a tooltip is
  /// excluded because the tooltip itself is included in the semantics
  /// tree directly and so having a gesture to show it would result in
  /// duplication of information.
  final bool excludeFromSemantics;

  /// Determines the way that drag start behavior is handled.
  ///
  /// If set to [DragStartBehavior.start], gesture drag behavior will
  /// begin at the position where the drag gesture won the arena. If set to
  /// [DragStartBehavior.down] it will begin at the position where a down event
  /// is first detected.
  ///
  /// In general, setting this to [DragStartBehavior.start] will make drag
  /// animation smoother and setting it to [DragStartBehavior.down] will make
  /// drag behavior feel slightly more reactive.
  ///
  /// By default, the drag start behavior is [DragStartBehavior.start].
  ///
  /// Only the [DragGestureRecognizer.onStart] callbacks for the
  /// [VerticalDragGestureRecognizer], [HorizontalDragGestureRecognizer] and
  /// [PanGestureRecognizer] are affected by this setting.
  ///
  /// See also:
  ///
  ///  * [DragGestureRecognizer.dragStartBehavior], which gives an example for the different behaviors.
  final DragStartBehavior dragStartBehavior;

  /// The kind of devices that are allowed to be recognized.
  ///
  /// If set to null, events from all device types will be recognized. Defaults to null.
  final Set<PointerDeviceKind>? supportedDevices;

  /// {@macro flutter.gestures.scale.trackpadScrollCausesScale}
  final bool trackpadScrollCausesScale;

  /// {@macro flutter.gestures.scale.trackpadScrollToScaleFactor}
  final Offset trackpadScrollToScaleFactor;

  @override
  Widget build(BuildContext context) {
    final Map<Type, GestureRecognizerFactory> gestures = <Type, GestureRecognizerFactory>{};
    final DeviceGestureSettings? gestureSettings = MediaQuery.maybeGestureSettingsOf(context);
    final ScrollBehavior configuration = ScrollConfiguration.of(context);

    if (onTapDown != null ||
        onTapUp != null ||
        onTap != null ||
        onTapCancel != null ||
        onSecondaryTap != null ||
        onSecondaryTapDown != null ||
        onSecondaryTapUp != null ||
        onSecondaryTapCancel != null ||
        onTertiaryTapDown != null ||
        onTertiaryTapUp != null ||
        onTertiaryTapCancel != null) {
      gestures[TapGestureRecognizer] = GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
        () => TapGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
        (TapGestureRecognizer instance) {
          instance
            ..onTapDown = onTapDown
            ..onTapUp = onTapUp
            ..onTap = onTap
            ..onTapCancel = onTapCancel
            ..onSecondaryTap = onSecondaryTap
            ..onSecondaryTapDown = onSecondaryTapDown
            ..onSecondaryTapUp = onSecondaryTapUp
            ..onSecondaryTapCancel = onSecondaryTapCancel
            ..onTertiaryTapDown = onTertiaryTapDown
            ..onTertiaryTapUp = onTertiaryTapUp
            ..onTertiaryTapCancel = onTertiaryTapCancel
            ..gestureSettings = gestureSettings
            ..supportedDevices = supportedDevices;
        },
      );
    }

    if (onDoubleTap != null || onDoubleTapDown != null || onDoubleTapCancel != null) {
      gestures[DoubleTapGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<DoubleTapGestureRecognizer>(
            () => DoubleTapGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
            (DoubleTapGestureRecognizer instance) {
              instance
                ..onDoubleTapDown = onDoubleTapDown
                ..onDoubleTap = onDoubleTap
                ..onDoubleTapCancel = onDoubleTapCancel
                ..gestureSettings = gestureSettings
                ..supportedDevices = supportedDevices;
            },
          );
    }

    if (onLongPressDown != null ||
        onLongPressCancel != null ||
        onLongPress != null ||
        onLongPressStart != null ||
        onLongPressMoveUpdate != null ||
        onLongPressUp != null ||
        onLongPressEnd != null ||
        onSecondaryLongPressDown != null ||
        onSecondaryLongPressCancel != null ||
        onSecondaryLongPress != null ||
        onSecondaryLongPressStart != null ||
        onSecondaryLongPressMoveUpdate != null ||
        onSecondaryLongPressUp != null ||
        onSecondaryLongPressEnd != null ||
        onTertiaryLongPressDown != null ||
        onTertiaryLongPressCancel != null ||
        onTertiaryLongPress != null ||
        onTertiaryLongPressStart != null ||
        onTertiaryLongPressMoveUpdate != null ||
        onTertiaryLongPressUp != null ||
        onTertiaryLongPressEnd != null) {
      gestures[LongPressGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<LongPressGestureRecognizer>(
            () => LongPressGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
            (LongPressGestureRecognizer instance) {
              instance
                ..onLongPressDown = onLongPressDown
                ..onLongPressCancel = onLongPressCancel
                ..onLongPress = onLongPress
                ..onLongPressStart = onLongPressStart
                ..onLongPressMoveUpdate = onLongPressMoveUpdate
                ..onLongPressUp = onLongPressUp
                ..onLongPressEnd = onLongPressEnd
                ..onSecondaryLongPressDown = onSecondaryLongPressDown
                ..onSecondaryLongPressCancel = onSecondaryLongPressCancel
                ..onSecondaryLongPress = onSecondaryLongPress
                ..onSecondaryLongPressStart = onSecondaryLongPressStart
                ..onSecondaryLongPressMoveUpdate = onSecondaryLongPressMoveUpdate
                ..onSecondaryLongPressUp = onSecondaryLongPressUp
                ..onSecondaryLongPressEnd = onSecondaryLongPressEnd
                ..onTertiaryLongPressDown = onTertiaryLongPressDown
                ..onTertiaryLongPressCancel = onTertiaryLongPressCancel
                ..onTertiaryLongPress = onTertiaryLongPress
                ..onTertiaryLongPressStart = onTertiaryLongPressStart
                ..onTertiaryLongPressMoveUpdate = onTertiaryLongPressMoveUpdate
                ..onTertiaryLongPressUp = onTertiaryLongPressUp
                ..onTertiaryLongPressEnd = onTertiaryLongPressEnd
                ..gestureSettings = gestureSettings
                ..supportedDevices = supportedDevices;
            },
          );
    }

    if (onVerticalDragDown != null ||
        onVerticalDragStart != null ||
        onVerticalDragUpdate != null ||
        onVerticalDragEnd != null ||
        onVerticalDragCancel != null) {
      gestures[VerticalDragGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<VerticalDragGestureRecognizer>(
            () =>
                VerticalDragGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
            (VerticalDragGestureRecognizer instance) {
              instance
                ..onDown = onVerticalDragDown
                ..onStart = onVerticalDragStart
                ..onUpdate = onVerticalDragUpdate
                ..onEnd = onVerticalDragEnd
                ..onCancel = onVerticalDragCancel
                ..dragStartBehavior = dragStartBehavior
                ..multitouchDragStrategy = configuration.getMultitouchDragStrategy(context)
                ..gestureSettings = gestureSettings
                ..supportedDevices = supportedDevices;
            },
          );
    }

    if (onHorizontalDragDown != null ||
        onHorizontalDragStart != null ||
        onHorizontalDragUpdate != null ||
        onHorizontalDragEnd != null ||
        onHorizontalDragCancel != null) {
      gestures[HorizontalDragGestureRecognizer] = GestureRecognizerFactoryWithHandlers<
        HorizontalDragGestureRecognizer
      >(
        () => HorizontalDragGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
        (HorizontalDragGestureRecognizer instance) {
          instance
            ..onDown = onHorizontalDragDown
            ..onStart = onHorizontalDragStart
            ..onUpdate = onHorizontalDragUpdate
            ..onEnd = onHorizontalDragEnd
            ..onCancel = onHorizontalDragCancel
            ..dragStartBehavior = dragStartBehavior
            ..multitouchDragStrategy = configuration.getMultitouchDragStrategy(context)
            ..gestureSettings = gestureSettings
            ..supportedDevices = supportedDevices;
        },
      );
    }

    if (onPanDown != null ||
        onPanStart != null ||
        onPanUpdate != null ||
        onPanEnd != null ||
        onPanCancel != null) {
      gestures[PanGestureRecognizer] = GestureRecognizerFactoryWithHandlers<PanGestureRecognizer>(
        () => PanGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
        (PanGestureRecognizer instance) {
          instance
            ..onDown = onPanDown
            ..onStart = onPanStart
            ..onUpdate = onPanUpdate
            ..onEnd = onPanEnd
            ..onCancel = onPanCancel
            ..dragStartBehavior = dragStartBehavior
            ..multitouchDragStrategy = configuration.getMultitouchDragStrategy(context)
            ..gestureSettings = gestureSettings
            ..supportedDevices = supportedDevices;
        },
      );
    }

    if (onScaleStart != null || onScaleUpdate != null || onScaleEnd != null) {
      gestures[ScaleGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<ScaleGestureRecognizer>(
            () => ScaleGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
            (ScaleGestureRecognizer instance) {
              instance
                ..onStart = onScaleStart
                ..onUpdate = onScaleUpdate
                ..onEnd = onScaleEnd
                ..dragStartBehavior = dragStartBehavior
                ..gestureSettings = gestureSettings
                ..trackpadScrollCausesScale = trackpadScrollCausesScale
                ..trackpadScrollToScaleFactor = trackpadScrollToScaleFactor
                ..supportedDevices = supportedDevices;
            },
          );
    }

    if (onForcePressStart != null ||
        onForcePressPeak != null ||
        onForcePressUpdate != null ||
        onForcePressEnd != null) {
      gestures[ForcePressGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<ForcePressGestureRecognizer>(
            () => ForcePressGestureRecognizer(debugOwner: this, supportedDevices: supportedDevices),
            (ForcePressGestureRecognizer instance) {
              instance
                ..onStart = onForcePressStart
                ..onPeak = onForcePressPeak
                ..onUpdate = onForcePressUpdate
                ..onEnd = onForcePressEnd
                ..gestureSettings = gestureSettings
                ..supportedDevices = supportedDevices;
            },
          );
    }

    return RawGestureDetector(
      gestures: gestures,
      behavior: behavior,
      excludeFromSemantics: excludeFromSemantics,
      child: child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<DragStartBehavior>('startBehavior', dragStartBehavior));
  }
}

/// A widget that detects gestures described by the given gesture
/// factories.
///
/// For common gestures, use a [GestureDetector].
/// [RawGestureDetector] is useful primarily when developing your
/// own gesture recognizers.
///
/// Configuring the gesture recognizers requires a carefully constructed map, as
/// described in [gestures] and as shown in the example below.
///
/// {@tool snippet}
///
/// This example shows how to hook up a [TapGestureRecognizer]. It assumes that
/// the code is being used inside a [State] object with a `_last` field that is
/// then displayed as the child of the gesture detector.
///
/// ```dart
/// RawGestureDetector(
///   gestures: <Type, GestureRecognizerFactory>{
///     TapGestureRecognizer: GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
///       () => TapGestureRecognizer(),
///       (TapGestureRecognizer instance) {
///         instance
///           ..onTapDown = (TapDownDetails details) { setState(() { _last = 'down'; }); }
///           ..onTapUp = (TapUpDetails details) { setState(() { _last = 'up'; }); }
///           ..onTap = () { setState(() { _last = 'tap'; }); }
///           ..onTapCancel = () { setState(() { _last = 'cancel'; }); };
///       },
///     ),
///   },
///   child: Container(width: 300.0, height: 300.0, color: Colors.yellow, child: Text(_last)),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [GestureDetector], a less flexible but much simpler widget that does the same thing.
///  * [Listener], a widget that reports raw pointer events.
///  * [GestureRecognizer], the class that you extend to create a custom gesture recognizer.
class RawGestureDetector extends StatefulWidget {
  /// Creates a widget that detects gestures.
  ///
  /// Gesture detectors can contribute semantic information to the tree that is
  /// used by assistive technology. The behavior can be configured by
  /// [semantics], or disabled with [excludeFromSemantics].
  const RawGestureDetector({
    super.key,
    this.child,
    this.gestures = const <Type, GestureRecognizerFactory>{},
    this.behavior,
    this.excludeFromSemantics = false,
    this.semantics,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The gestures that this widget will attempt to recognize.
  ///
  /// This should be a map from [GestureRecognizer] subclasses to
  /// [GestureRecognizerFactory] subclasses specialized with the same type.
  ///
  /// This value can be late-bound at layout time using
  /// [RawGestureDetectorState.replaceGestureRecognizers].
  final Map<Type, GestureRecognizerFactory> gestures;

  /// How this gesture detector should behave during hit testing.
  ///
  /// This defaults to [HitTestBehavior.deferToChild] if [child] is not null and
  /// [HitTestBehavior.translucent] if child is null.
  final HitTestBehavior? behavior;

  /// Whether to exclude these gestures from the semantics tree. For
  /// example, the long-press gesture for showing a tooltip is
  /// excluded because the tooltip itself is included in the semantics
  /// tree directly and so having a gesture to show it would result in
  /// duplication of information.
  final bool excludeFromSemantics;

  /// Describes the semantics notations that should be added to the underlying
  /// render object [RenderSemanticsGestureHandler].
  ///
  /// It has no effect if [excludeFromSemantics] is true.
  ///
  /// When [semantics] is null, [RawGestureDetector] will fall back to a
  /// default delegate which checks if the detector owns certain gesture
  /// recognizers and calls their callbacks if they exist:
  ///
  ///  * During a semantic tap, it calls [TapGestureRecognizer]'s
  ///    `onTapDown`, `onTapUp`, and `onTap`.
  ///  * During a semantic long press, it calls [LongPressGestureRecognizer]'s
  ///    `onLongPressDown`, `onLongPressStart`, `onLongPress`, `onLongPressEnd`
  ///    and `onLongPressUp`.
  ///  * During a semantic horizontal drag, it calls [HorizontalDragGestureRecognizer]'s
  ///    `onDown`, `onStart`, `onUpdate` and `onEnd`, then
  ///    [PanGestureRecognizer]'s `onDown`, `onStart`, `onUpdate` and `onEnd`.
  ///  * During a semantic vertical drag, it calls [VerticalDragGestureRecognizer]'s
  ///    `onDown`, `onStart`, `onUpdate` and `onEnd`, then
  ///    [PanGestureRecognizer]'s `onDown`, `onStart`, `onUpdate` and `onEnd`.
  ///
  /// {@tool snippet}
  /// This custom gesture detector listens to force presses, while also allows
  /// the same callback to be triggered by semantic long presses.
  ///
  /// ```dart
  /// class ForcePressGestureDetectorWithSemantics extends StatelessWidget {
  ///   const ForcePressGestureDetectorWithSemantics({
  ///     super.key,
  ///     required this.child,
  ///     required this.onForcePress,
  ///   });
  ///
  ///   final Widget child;
  ///   final VoidCallback onForcePress;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return RawGestureDetector(
  ///       gestures: <Type, GestureRecognizerFactory>{
  ///         ForcePressGestureRecognizer: GestureRecognizerFactoryWithHandlers<ForcePressGestureRecognizer>(
  ///           () => ForcePressGestureRecognizer(debugOwner: this),
  ///           (ForcePressGestureRecognizer instance) {
  ///             instance.onStart = (_) => onForcePress();
  ///           }
  ///         ),
  ///       },
  ///       behavior: HitTestBehavior.opaque,
  ///       semantics: _LongPressSemanticsDelegate(onForcePress),
  ///       child: child,
  ///     );
  ///   }
  /// }
  ///
  /// class _LongPressSemanticsDelegate extends SemanticsGestureDelegate {
  ///   _LongPressSemanticsDelegate(this.onLongPress);
  ///
  ///   VoidCallback onLongPress;
  ///
  ///   @override
  ///   void assignSemantics(RenderSemanticsGestureHandler renderObject) {
  ///     renderObject.onLongPress = onLongPress;
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  final SemanticsGestureDelegate? semantics;

  @override
  RawGestureDetectorState createState() => RawGestureDetectorState();
}

/// State for a [RawGestureDetector].
class RawGestureDetectorState extends State<RawGestureDetector> {
  Map<Type, GestureRecognizer>? _recognizers = const <Type, GestureRecognizer>{};
  SemanticsGestureDelegate? _semantics;

  @protected
  @override
  void initState() {
    super.initState();
    _semantics = widget.semantics ?? _DefaultSemanticsGestureDelegate(this);
    _syncAll(widget.gestures);
  }

  @protected
  @override
  void didUpdateWidget(RawGestureDetector oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!(oldWidget.semantics == null && widget.semantics == null)) {
      _semantics = widget.semantics ?? _DefaultSemanticsGestureDelegate(this);
    }
    _syncAll(widget.gestures);
  }

  /// This method can be called after the build phase, during the
  /// layout of the nearest descendant [RenderObjectWidget] of the
  /// gesture detector, to update the list of active gesture
  /// recognizers.
  ///
  /// The typical use case is [Scrollable]s, which put their viewport
  /// in their gesture detector, and then need to know the dimensions
  /// of the viewport and the viewport's child to determine whether
  /// the gesture detector should be enabled.
  ///
  /// The argument should follow the same conventions as
  /// [RawGestureDetector.gestures]. It acts like a temporary replacement for
  /// that value until the next build.
  void replaceGestureRecognizers(Map<Type, GestureRecognizerFactory> gestures) {
    assert(() {
      if (!context.findRenderObject()!.owner!.debugDoingLayout) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'Unexpected call to replaceGestureRecognizers() method of RawGestureDetectorState.',
          ),
          ErrorDescription(
            'The replaceGestureRecognizers() method can only be called during the layout phase.',
          ),
          ErrorHint(
            'To set the gesture recognizers at other times, trigger a new build using setState() '
            'and provide the new gesture recognizers as constructor arguments to the corresponding '
            'RawGestureDetector or GestureDetector object.',
          ),
        ]);
      }
      return true;
    }());
    _syncAll(gestures);
    if (!widget.excludeFromSemantics) {
      final RenderSemanticsGestureHandler semanticsGestureHandler =
          context.findRenderObject()! as RenderSemanticsGestureHandler;
      _updateSemanticsForRenderObject(semanticsGestureHandler);
    }
  }

  /// This method can be called to filter the list of available semantic actions,
  /// after the render object was created.
  ///
  /// The actual filtering is happening in the next frame and a frame will be
  /// scheduled if non is pending.
  ///
  /// This is used by [Scrollable] to configure system accessibility tools so
  /// that they know in which direction a particular list can be scrolled.
  ///
  /// If this is never called, then the actions are not filtered. If the list of
  /// actions to filter changes, it must be called again.
  void replaceSemanticsActions(Set<SemanticsAction> actions) {
    if (widget.excludeFromSemantics) {
      return;
    }

    final RenderSemanticsGestureHandler? semanticsGestureHandler =
        context.findRenderObject() as RenderSemanticsGestureHandler?;
    assert(() {
      if (semanticsGestureHandler == null) {
        throw FlutterError(
          'Unexpected call to replaceSemanticsActions() method of RawGestureDetectorState.\n'
          'The replaceSemanticsActions() method can only be called after the RenderSemanticsGestureHandler has been created.',
        );
      }
      return true;
    }());

    semanticsGestureHandler!.validActions =
        actions; // will call _markNeedsSemanticsUpdate(), if required.
  }

  @protected
  @override
  void dispose() {
    for (final GestureRecognizer recognizer in _recognizers!.values) {
      recognizer.dispose();
    }
    _recognizers = null;
    super.dispose();
  }

  void _syncAll(Map<Type, GestureRecognizerFactory> gestures) {
    assert(_recognizers != null);
    final Map<Type, GestureRecognizer> oldRecognizers = _recognizers!;
    _recognizers = <Type, GestureRecognizer>{};
    for (final Type type in gestures.keys) {
      assert(gestures[type] != null);
      assert(gestures[type]!._debugAssertTypeMatches(type));
      assert(!_recognizers!.containsKey(type));
      _recognizers![type] = oldRecognizers[type] ?? gestures[type]!.constructor();
      assert(
        _recognizers![type].runtimeType == type,
        'GestureRecognizerFactory of type $type created a GestureRecognizer of type ${_recognizers![type].runtimeType}. The GestureRecognizerFactory must be specialized with the type of the class that it returns from its constructor method.',
      );
      gestures[type]!.initializer(_recognizers![type]!);
    }
    for (final Type type in oldRecognizers.keys) {
      if (!_recognizers!.containsKey(type)) {
        oldRecognizers[type]!.dispose();
      }
    }
  }

  void _handlePointerDown(PointerDownEvent event) {
    assert(_recognizers != null);
    for (final GestureRecognizer recognizer in _recognizers!.values) {
      recognizer.addPointer(event);
    }
  }

  void _handlePointerPanZoomStart(PointerPanZoomStartEvent event) {
    assert(_recognizers != null);
    for (final GestureRecognizer recognizer in _recognizers!.values) {
      recognizer.addPointerPanZoom(event);
    }
  }

  HitTestBehavior get _defaultBehavior {
    return widget.child == null ? HitTestBehavior.translucent : HitTestBehavior.deferToChild;
  }

  void _updateSemanticsForRenderObject(RenderSemanticsGestureHandler renderObject) {
    assert(!widget.excludeFromSemantics);
    assert(_semantics != null);
    _semantics!.assignSemantics(renderObject);
  }

  @protected
  @override
  Widget build(BuildContext context) {
    Widget result = Listener(
      onPointerDown: _handlePointerDown,
      onPointerPanZoomStart: _handlePointerPanZoomStart,
      behavior: widget.behavior ?? _defaultBehavior,
      child: widget.child,
    );
    if (!widget.excludeFromSemantics) {
      result = _GestureSemantics(
        behavior: widget.behavior ?? _defaultBehavior,
        assignSemantics: _updateSemanticsForRenderObject,
        child: result,
      );
    }
    return result;
  }

  @protected
  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    if (_recognizers == null) {
      properties.add(DiagnosticsNode.message('DISPOSED'));
    } else {
      final List<String> gestures =
          _recognizers!.values
              .map<String>((GestureRecognizer recognizer) => recognizer.debugDescription)
              .toList();
      properties.add(IterableProperty<String>('gestures', gestures, ifEmpty: '<none>'));
      properties.add(
        IterableProperty<GestureRecognizer>(
          'recognizers',
          _recognizers!.values,
          level: DiagnosticLevel.fine,
        ),
      );
      properties.add(
        DiagnosticsProperty<bool>(
          'excludeFromSemantics',
          widget.excludeFromSemantics,
          defaultValue: false,
        ),
      );
      if (!widget.excludeFromSemantics) {
        properties.add(
          DiagnosticsProperty<SemanticsGestureDelegate>(
            'semantics',
            widget.semantics,
            defaultValue: null,
          ),
        );
      }
    }
    properties.add(EnumProperty<HitTestBehavior>('behavior', widget.behavior, defaultValue: null));
  }
}

typedef _AssignSemantics = void Function(RenderSemanticsGestureHandler);

class _GestureSemantics extends SingleChildRenderObjectWidget {
  const _GestureSemantics({super.child, required this.behavior, required this.assignSemantics});

  final HitTestBehavior behavior;
  final _AssignSemantics assignSemantics;

  @override
  RenderSemanticsGestureHandler createRenderObject(BuildContext context) {
    final RenderSemanticsGestureHandler renderObject =
        RenderSemanticsGestureHandler()..behavior = behavior;
    assignSemantics(renderObject);
    return renderObject;
  }

  @override
  void updateRenderObject(BuildContext context, RenderSemanticsGestureHandler renderObject) {
    renderObject.behavior = behavior;
    assignSemantics(renderObject);
  }
}

/// A base class that describes what semantics notations a [RawGestureDetector]
/// should add to the render object [RenderSemanticsGestureHandler].
///
/// It is used to allow custom [GestureDetector]s to add semantics notations.
abstract class SemanticsGestureDelegate {
  /// Create a delegate of gesture semantics.
  const SemanticsGestureDelegate();

  /// Assigns semantics notations to the [RenderSemanticsGestureHandler] render
  /// object of the gesture detector.
  ///
  /// This method is called when the widget is created, updated, or during
  /// [RawGestureDetectorState.replaceGestureRecognizers].
  void assignSemantics(RenderSemanticsGestureHandler renderObject);

  @override
  String toString() => '${objectRuntimeType(this, 'SemanticsGestureDelegate')}()';
}

// The default semantics delegate of [RawGestureDetector]. Its behavior is
// described in [RawGestureDetector.semantics].
//
// For readers who come here to learn how to write custom semantics delegates:
// this is not a proper sample code. It has access to the detector state as well
// as its private properties, which are inaccessible normally. It is designed
// this way in order to work independently in a [RawGestureRecognizer] to
// preserve existing behavior.
//
// Instead, a normal delegate will store callbacks as properties, and use them
// in `assignSemantics`.
class _DefaultSemanticsGestureDelegate extends SemanticsGestureDelegate {
  _DefaultSemanticsGestureDelegate(this.detectorState);

  final RawGestureDetectorState detectorState;

  @override
  void assignSemantics(RenderSemanticsGestureHandler renderObject) {
    assert(!detectorState.widget.excludeFromSemantics);
    final Map<Type, GestureRecognizer> recognizers = detectorState._recognizers!;
    renderObject
      ..onTap = _getTapHandler(recognizers)
      ..onLongPress = _getLongPressHandler(recognizers)
      ..onHorizontalDragUpdate = _getHorizontalDragUpdateHandler(recognizers)
      ..onVerticalDragUpdate = _getVerticalDragUpdateHandler(recognizers);
  }

  GestureTapCallback? _getTapHandler(Map<Type, GestureRecognizer> recognizers) {
    final TapGestureRecognizer? tap = recognizers[TapGestureRecognizer] as TapGestureRecognizer?;
    if (tap == null) {
      return null;
    }

    return () {
      tap.onTapDown?.call(const TapDownDetails());
      tap.onTapUp?.call(const TapUpDetails(kind: PointerDeviceKind.unknown));
      tap.onTap?.call();
    };
  }

  GestureLongPressCallback? _getLongPressHandler(Map<Type, GestureRecognizer> recognizers) {
    final LongPressGestureRecognizer? longPress =
        recognizers[LongPressGestureRecognizer] as LongPressGestureRecognizer?;
    if (longPress == null) {
      return null;
    }

    return () {
      longPress.onLongPressDown?.call(const LongPressDownDetails());
      longPress.onLongPressStart?.call(const LongPressStartDetails());
      longPress.onLongPress?.call();
      longPress.onLongPressEnd?.call(const LongPressEndDetails());
      longPress.onLongPressUp?.call();
    };
  }

  GestureDragUpdateCallback? _getHorizontalDragUpdateHandler(
    Map<Type, GestureRecognizer> recognizers,
  ) {
    final HorizontalDragGestureRecognizer? horizontal =
        recognizers[HorizontalDragGestureRecognizer] as HorizontalDragGestureRecognizer?;
    final PanGestureRecognizer? pan = recognizers[PanGestureRecognizer] as PanGestureRecognizer?;

    final GestureDragUpdateCallback? horizontalHandler =
        horizontal == null
            ? null
            : (DragUpdateDetails details) {
              horizontal.onDown?.call(const DragDownDetails());
              horizontal.onStart?.call(const DragStartDetails());
              horizontal.onUpdate?.call(details);
              horizontal.onEnd?.call(DragEndDetails(primaryVelocity: 0.0));
            };

    final GestureDragUpdateCallback? panHandler =
        pan == null
            ? null
            : (DragUpdateDetails details) {
              pan.onDown?.call(const DragDownDetails());
              pan.onStart?.call(const DragStartDetails());
              pan.onUpdate?.call(details);
              pan.onEnd?.call(DragEndDetails());
            };

    if (horizontalHandler == null && panHandler == null) {
      return null;
    }
    return (DragUpdateDetails details) {
      horizontalHandler?.call(details);
      panHandler?.call(details);
    };
  }

  GestureDragUpdateCallback? _getVerticalDragUpdateHandler(
    Map<Type, GestureRecognizer> recognizers,
  ) {
    final VerticalDragGestureRecognizer? vertical =
        recognizers[VerticalDragGestureRecognizer] as VerticalDragGestureRecognizer?;
    final PanGestureRecognizer? pan = recognizers[PanGestureRecognizer] as PanGestureRecognizer?;

    final GestureDragUpdateCallback? verticalHandler =
        vertical == null
            ? null
            : (DragUpdateDetails details) {
              vertical.onDown?.call(const DragDownDetails());
              vertical.onStart?.call(const DragStartDetails());
              vertical.onUpdate?.call(details);
              vertical.onEnd?.call(DragEndDetails(primaryVelocity: 0.0));
            };

    final GestureDragUpdateCallback? panHandler =
        pan == null
            ? null
            : (DragUpdateDetails details) {
              pan.onDown?.call(const DragDownDetails());
              pan.onStart?.call(const DragStartDetails());
              pan.onUpdate?.call(details);
              pan.onEnd?.call(DragEndDetails());
            };

    if (verticalHandler == null && panHandler == null) {
      return null;
    }
    return (DragUpdateDetails details) {
      verticalHandler?.call(details);
      panHandler?.call(details);
    };
  }
}
