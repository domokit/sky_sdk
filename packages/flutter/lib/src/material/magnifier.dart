// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';

/// A [Magnifier] positioned by rules dictated by the native Android magnifier.
///
/// The positioning rules are based on [magnifierInfo], as follows:
///
/// - The loupe tracks the gesture's _x_ coordinate, clamping to the beginning
///   and end of the currently editing line.
///
/// - The focal point never contains anything out of the bounds of the text
///   field or other widget being magnified (the [MagnifierInfo.fieldBounds]).
///
/// - The focal point always remains aligned with the _y_ coordinate of the touch.
///
/// - The loupe always remains on the screen.
///
/// - When the line targeted by the touch's _y_ coordinate changes, the position
///   is animated over [jumpBetweenLinesAnimationDuration].
///
/// This behavior was based on the Android 12 source code, where possible, and
/// on eyeballing a Pixel 6 running Android 12 otherwise.
class TextMagnifier extends StatefulWidget {
  /// Creates a [TextMagnifier].
  ///
  /// The [magnifierInfo] must be provided, and must be updated with new values
  /// as the user's touch changes.
  const TextMagnifier({super.key, required this.magnifierInfo});

  /// A [TextMagnifierConfiguration] that returns a [CupertinoTextMagnifier] on
  /// iOS, [TextMagnifier] on Android, and null on all other platforms, and
  /// shows the editing handles only on iOS.
  static TextMagnifierConfiguration adaptiveMagnifierConfiguration = TextMagnifierConfiguration(
    shouldDisplayHandlesInMagnifier: defaultTargetPlatform == TargetPlatform.iOS,
    magnifierBuilder: (
      BuildContext context,
      MagnifierController controller,
      ValueNotifier<MagnifierInfo> magnifierInfo,
    ) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
          return CupertinoTextMagnifier(controller: controller, magnifierInfo: magnifierInfo);
        case TargetPlatform.android:
          return TextMagnifier(magnifierInfo: magnifierInfo);
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          return null;
      }
    },
  );

  /// The duration that the position is animated if [TextMagnifier] just switched
  /// between lines.
  static const Duration jumpBetweenLinesAnimationDuration = Duration(milliseconds: 70);

  /// The current status of the user's touch.
  ///
  /// As the value of the [magnifierInfo] changes, the position of the loupe is
  /// adjusted automatically, according to the rules described in the
  /// [TextMagnifier] class description.
  final ValueNotifier<MagnifierInfo> magnifierInfo;

  @override
  State<TextMagnifier> createState() => _TextMagnifierState();
}

class _TextMagnifierState extends State<TextMagnifier> {
  // Should _only_ be null on construction. This is because of the animation logic.
  //
  // Animations are added when `last_build_y != current_build_y`. This condition
  // is true on the initial render, which would mean that the initial
  // build would be animated - this is undesired. Thus, this is null for the
  // first frame and the condition becomes `magnifierPosition != null && last_build_y != this_build_y`.
  Offset? _magnifierPosition;

  // A timer that unsets itself after an animation duration.
  // If the timer exists, then the magnifier animates its position -
  // if this timer does not exist, the magnifier tracks the gesture (with respect
  // to the positioning rules) directly.
  Timer? _positionShouldBeAnimatedTimer;
  bool get _positionShouldBeAnimated => _positionShouldBeAnimatedTimer != null;

  Offset _extraFocalPointOffset = Offset.zero;

  @override
  void initState() {
    super.initState();
    widget.magnifierInfo.addListener(_determineMagnifierPositionAndFocalPoint);
  }

  @override
  void dispose() {
    widget.magnifierInfo.removeListener(_determineMagnifierPositionAndFocalPoint);
    _positionShouldBeAnimatedTimer?.cancel();
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    _determineMagnifierPositionAndFocalPoint();
    super.didChangeDependencies();
  }

  @override
  void didUpdateWidget(TextMagnifier oldWidget) {
    if (oldWidget.magnifierInfo != widget.magnifierInfo) {
      oldWidget.magnifierInfo.removeListener(_determineMagnifierPositionAndFocalPoint);
      widget.magnifierInfo.addListener(_determineMagnifierPositionAndFocalPoint);
    }
    super.didUpdateWidget(oldWidget);
  }

  void _determineMagnifierPositionAndFocalPoint() {
    final MagnifierInfo selectionInfo = widget.magnifierInfo.value;
    final Rect screenRect = Offset.zero & MediaQuery.sizeOf(context);

    // Since by default we draw at the top left corner, this offset
    // shifts the magnifier so we draw at the center, and then also includes
    // the "above touch point" shift.
    final Offset basicMagnifierOffset = Offset(
      Magnifier.kDefaultMagnifierSize.width / 2,
      Magnifier.kDefaultMagnifierSize.height + Magnifier.kStandardVerticalFocalPointShift,
    );

    // Since the magnifier should not go past the edges of the line,
    // but must track the gesture otherwise, constrain the X of the magnifier
    // to always stay between line start and end.
    final double magnifierX = clampDouble(
      selectionInfo.globalGesturePosition.dx,
      selectionInfo.currentLineBoundaries.left,
      selectionInfo.currentLineBoundaries.right,
    );

    // Place the magnifier at the previously calculated X, and the Y should be
    // exactly at the center of the handle.
    final Rect unadjustedMagnifierRect =
        Offset(magnifierX, selectionInfo.caretRect.center.dy) - basicMagnifierOffset &
        Magnifier.kDefaultMagnifierSize;

    // Shift the magnifier so that, if we are ever out of the screen, we become in bounds.
    // This probably won't have much of an effect on the X, since it is already bound
    // to the currentLineBoundaries, but will shift vertically if the magnifier is out of bounds.
    final Rect screenBoundsAdjustedMagnifierRect = MagnifierController.shiftWithinBounds(
      bounds: screenRect,
      rect: unadjustedMagnifierRect,
    );

    // Done with the magnifier position!
    final Offset finalMagnifierPosition = screenBoundsAdjustedMagnifierRect.topLeft;

    // The insets, from either edge, that the focal point should not point
    // past lest the magnifier displays something out of bounds.
    final double horizontalMaxFocalPointEdgeInsets =
        (Magnifier.kDefaultMagnifierSize.width / 2) / Magnifier._magnification;

    // Adjust the focal point horizontally such that none of the magnifier
    // ever points to anything out of bounds.
    final double newGlobalFocalPointX;

    // If the text field is so narrow that we must show out of bounds,
    // then settle for pointing to the center all the time.
    if (selectionInfo.fieldBounds.width < horizontalMaxFocalPointEdgeInsets * 2) {
      newGlobalFocalPointX = selectionInfo.fieldBounds.center.dx;
    } else {
      // Otherwise, we can clamp the focal point to always point in bounds.
      newGlobalFocalPointX = clampDouble(
        screenBoundsAdjustedMagnifierRect.center.dx,
        selectionInfo.fieldBounds.left + horizontalMaxFocalPointEdgeInsets,
        selectionInfo.fieldBounds.right - horizontalMaxFocalPointEdgeInsets,
      );
    }

    // Since the previous value is now a global offset (i.e. `newGlobalFocalPoint`
    // is now a global offset), we must subtract the magnifier's global offset
    // to obtain the relative shift in the focal point.
    final double newRelativeFocalPointX =
        newGlobalFocalPointX - screenBoundsAdjustedMagnifierRect.center.dx;

    // The Y component means that if we are pressed up against the top of the screen,
    // then we should adjust the focal point such that it now points to how far we moved
    // the magnifier. screenBoundsAdjustedMagnifierRect.top == unadjustedMagnifierRect.top for most cases,
    // but when pressed up against the top of the screen, we adjust the focal point by
    // the amount that we shifted from our "natural" position.
    final Offset focalPointAdjustmentForScreenBoundsAdjustment = Offset(
      newRelativeFocalPointX,
      unadjustedMagnifierRect.top - screenBoundsAdjustedMagnifierRect.top,
    );

    Timer? positionShouldBeAnimated = _positionShouldBeAnimatedTimer;

    if (_magnifierPosition != null && finalMagnifierPosition.dy != _magnifierPosition!.dy) {
      if (_positionShouldBeAnimatedTimer != null && _positionShouldBeAnimatedTimer!.isActive) {
        _positionShouldBeAnimatedTimer!.cancel();
      }

      // Create a timer that deletes itself when the timer is complete.
      // This is `mounted` safe, since the timer is canceled in `dispose`.
      positionShouldBeAnimated = Timer(
        TextMagnifier.jumpBetweenLinesAnimationDuration,
        () => setState(() {
          _positionShouldBeAnimatedTimer = null;
        }),
      );
    }

    setState(() {
      _magnifierPosition = finalMagnifierPosition;
      _positionShouldBeAnimatedTimer = positionShouldBeAnimated;
      _extraFocalPointOffset = focalPointAdjustmentForScreenBoundsAdjustment;
    });
  }

  @override
  Widget build(BuildContext context) {
    assert(
      _magnifierPosition != null,
      'Magnifier position should only be null before the first build.',
    );

    return AnimatedPositioned(
      top: _magnifierPosition!.dy,
      left: _magnifierPosition!.dx,
      // Material magnifier typically does not animate, unless we jump between lines,
      // in which case we animate between lines.
      duration:
          _positionShouldBeAnimated
              ? TextMagnifier.jumpBetweenLinesAnimationDuration
              : Duration.zero,
      child: Magnifier(additionalFocalPointOffset: _extraFocalPointOffset),
    );
  }
}

/// A Material-styled magnifying glass.
///
/// {@macro flutter.widgets.magnifier.intro}
///
/// This widget focuses on mimicking the _style_ of the magnifier on material.
/// For a widget that is focused on mimicking the _behavior_ of a material
/// magnifier, see [TextMagnifier], which uses [Magnifier].
///
/// The styles implemented in this widget were based on the Android 12 source
/// code, where possible, and on eyeballing a Pixel 6 running Android 12
/// otherwise.
class Magnifier extends StatelessWidget {
  /// Creates a [RawMagnifier] in the Material style.
  const Magnifier({
    super.key,
    this.additionalFocalPointOffset = Offset.zero,
    this.borderRadius = const BorderRadius.all(Radius.circular(_borderRadius)),
    this.filmColor = const Color.fromARGB(8, 158, 158, 158),
    this.shadows = const <BoxShadow>[
      BoxShadow(
        blurRadius: 1.5,
        offset: Offset(0.0, 2.0),
        spreadRadius: 0.75,
        color: Color.fromARGB(25, 0, 0, 0),
      ),
    ],
    this.clipBehavior = Clip.hardEdge,
    this.size = Magnifier.kDefaultMagnifierSize,
  });

  /// The default size of this [Magnifier].
  ///
  /// The size of the magnifier may be modified through the constructor;
  /// [kDefaultMagnifierSize] is extracted from the default parameter of
  /// [Magnifier]'s constructor so that positioners may depend on it.
  static const Size kDefaultMagnifierSize = Size(77.37, 37.9);

  /// The vertical distance that the magnifier should be above the focal point.
  ///
  /// The [kStandardVerticalFocalPointShift] value is a constant so that
  /// positioning of this [Magnifier] can be done with a guaranteed size, as
  /// opposed to an estimate.
  static const double kStandardVerticalFocalPointShift = 22.0;

  static const double _borderRadius = 40;
  static const double _magnification = 1.25;

  /// Any additional offset the focal point requires to "point"
  /// to the correct place.
  ///
  /// This value is added to [kStandardVerticalFocalPointShift] to obtain the
  /// actual offset.
  ///
  /// This is useful for instances where the magnifier is not pointing to
  /// something directly below it.
  final Offset additionalFocalPointOffset;

  /// The border radius for this magnifier.
  ///
  /// The magnifier's shape is a [RoundedRectangleBorder] with this radius.
  final BorderRadius borderRadius;

  /// The color to tint the image in this [Magnifier].
  ///
  /// On native Android, there is a almost transparent gray tint to the
  /// magnifier, in order to better distinguish the contents of the lens from
  /// the background.
  final Color filmColor;

  /// A list of shadows cast by the [Magnifier].
  ///
  /// If the shadows use a [BlurStyle] that paints inside the shape, or if they
  /// are offset, then a [clipBehavior] that enables clipping (such as the
  /// default [Clip.hardEdge]) is recommended, otherwise the shadow will occlude
  /// the magnifier (the shadow is drawn above the magnifier so as to not be
  /// included in the magnified image).
  ///
  /// By default, the shadows are offset vertically by two logical pixels, so
  /// clipping is recommended.
  ///
  /// A shadow that uses [BlurStyle.outer] and is not offset does not need
  /// clipping; in that case, consider setting [clipBehavior] to [Clip.none].
  final List<BoxShadow> shadows;

  /// Whether and how to clip the [shadows] that render inside the loupe.
  ///
  /// Defaults to [Clip.hardEdge].
  ///
  /// A value of [Clip.none] can be used if the shadow will not paint where the
  /// magnified image appears, or if doing so is intentional (e.g. to blur the
  /// edges of the magnified image).
  ///
  /// See the discussion at [shadows].
  final Clip clipBehavior;

  /// The [Size] of this [Magnifier].
  ///
  /// The [shadows] are drawn outside of the [size].
  final Size size;

  @override
  Widget build(BuildContext context) {
    return RawMagnifier(
      decoration: MagnifierDecoration(
        shape: RoundedRectangleBorder(borderRadius: borderRadius),
        shadows: shadows,
      ),
      clipBehavior: clipBehavior,
      magnificationScale: _magnification,
      focalPointOffset:
          additionalFocalPointOffset +
          Offset(0, kStandardVerticalFocalPointShift + kDefaultMagnifierSize.height / 2),
      size: size,
      child: ColoredBox(
        // This couldn't be part of the decoration (even if the
        // MagnifierDecoration supported specifying a color) because the
        // decoration's shadows are offset and therefore we set a clipBehavior
        // that clips the inner part of the decoration to avoid occluding the
        // magnified image with the shadow.
        color: filmColor,
      ),
    );
  }
}
