// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;

import 'package:flutter/widgets.dart';

import 'theme.dart';

/// A [CupertinoMagnifier] used for magnifying text in cases where a user's
/// finger may be blocking the point of interest, like a selection handle.
///
/// {@tool dartpad}
/// This sample demonstrates how to use [CupertinoTextMagnifier].
///
/// ** See code in examples/api/lib/widgets/magnifier/cupertino_text_magnifier.0.dart **
/// {@end-tool}
///
/// Delegates styling to [CupertinoMagnifier] with its position depending on
/// [magnifierInfo].
///
/// Specifically, the [CupertinoTextMagnifier] follows the following rules.
/// [CupertinoTextMagnifier]:
/// - is positioned horizontally inside the screen width, with [horizontalScreenEdgePadding] padding.
/// - is hidden if a gesture is detected [hideBelowThreshold] units below the line
///   that the magnifier is on, shown otherwise.
/// - follows the x coordinate of the gesture directly (with respect to rule 1).
/// - has some vertical drag resistance; i.e. if a gesture is detected k units below the field,
///   then has vertical offset [dragResistance] * k.
class CupertinoTextMagnifier extends StatefulWidget {
  /// Constructs a [RawMagnifier] in the Cupertino style, positioning with respect to
  /// [magnifierInfo].
  ///
  /// The default constructor parameters and constants were eyeballed on
  /// an iPhone XR iOS v15.5.
  const CupertinoTextMagnifier({
    super.key,
    this.animationCurve = Curves.easeOut,
    required this.controller,
    this.dragResistance = 10.0,
    this.hideBelowThreshold = 48.0,
    this.horizontalScreenEdgePadding = 10.0,
    required this.magnifierInfo,
  });

  /// The curve used for the in / out animations.
  final Curve animationCurve;

  /// This magnifier's controller.
  ///
  /// The [CupertinoTextMagnifier] requires a [MagnifierController]
  /// in order to show / hide itself without removing itself from the
  /// overlay.
  final MagnifierController controller;

  /// A drag resistance on the downward Y position of the lens.
  final double dragResistance;

  /// The difference in Y between the gesture position and the caret center
  /// so that the magnifier hides itself.
  final double hideBelowThreshold;

  /// The padding on either edge of the screen that any part of the magnifier
  /// cannot exist past.
  ///
  /// This includes any part of the magnifier, not just the center; for example,
  /// the left edge of the magnifier cannot be outside the [horizontalScreenEdgePadding].v
  ///
  /// If the screen has width w, then the magnifier is bound to
  /// `_kHorizontalScreenEdgePadding, w - _kHorizontalScreenEdgePadding`.
  final double horizontalScreenEdgePadding;

  /// [CupertinoTextMagnifier] will determine its own positioning
  /// based on the [MagnifierInfo] of this notifier.
  final ValueNotifier<MagnifierInfo> magnifierInfo;

  /// The duration that the magnifier drags behind its final position.
  static const Duration _kDragAnimationDuration = Duration(milliseconds: 45);

  @override
  State<CupertinoTextMagnifier> createState() => _CupertinoTextMagnifierState();
}

class _CupertinoTextMagnifierState extends State<CupertinoTextMagnifier>
    with SingleTickerProviderStateMixin {
  // Initialize to dummy values for the event that the initial call to
  // _determineMagnifierPositionAndFocalPoint calls hide, and thus does not
  // set these values.
  Offset _currentAdjustedMagnifierPosition = Offset.zero;
  double _verticalFocalPointAdjustment = 0;
  late final AnimationController _ioAnimationController;
  late final Animation<double> _ioAnimation;
  late final CurvedAnimation _ioCurvedAnimation;

  @override
  void initState() {
    super.initState();
    _ioAnimationController = AnimationController(
      value: 0,
      vsync: this,
      duration: CupertinoMagnifier._kInOutAnimationDuration,
    )..addListener(() => setState(() {}));

    widget.controller.animationController = _ioAnimationController;
    widget.magnifierInfo.addListener(_determineMagnifierPositionAndFocalPoint);
    _ioCurvedAnimation = CurvedAnimation(
      parent: _ioAnimationController,
      curve: widget.animationCurve,
    );
    _ioAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(_ioCurvedAnimation);
  }

  @override
  void dispose() {
    widget.controller.animationController = null;
    _ioAnimationController.dispose();
    _ioCurvedAnimation.dispose();
    widget.magnifierInfo.removeListener(_determineMagnifierPositionAndFocalPoint);
    super.dispose();
  }

  @override
  void didUpdateWidget(CupertinoTextMagnifier oldWidget) {
    if (oldWidget.magnifierInfo != widget.magnifierInfo) {
      oldWidget.magnifierInfo.removeListener(_determineMagnifierPositionAndFocalPoint);
      widget.magnifierInfo.addListener(_determineMagnifierPositionAndFocalPoint);
    }
    super.didUpdateWidget(oldWidget);
  }

  @override
  void didChangeDependencies() {
    _determineMagnifierPositionAndFocalPoint();
    super.didChangeDependencies();
  }

  void _determineMagnifierPositionAndFocalPoint() {
    final MagnifierInfo textEditingContext = widget.magnifierInfo.value;

    // The exact Y of the center of the current line.
    final double verticalCenterOfCurrentLine = textEditingContext.caretRect.center.dy;

    // If the magnifier is currently showing, but we have dragged out of threshold,
    // we should hide it.
    if (verticalCenterOfCurrentLine - textEditingContext.globalGesturePosition.dy <
        -widget.hideBelowThreshold) {
      // Only signal a hide if we are currently showing.
      if (widget.controller.shown) {
        widget.controller.hide(removeFromOverlay: false);
      }
      return;
    }

    // If we are gone, but got to this point, we shouldn't be: show.
    if (!widget.controller.shown) {
      _ioAnimationController.forward();
    }

    // Never go above the center of the line, but have some resistance
    // going downward if the drag goes too far.
    final double verticalPositionOfLens = math.max(
      verticalCenterOfCurrentLine,
      verticalCenterOfCurrentLine -
          (verticalCenterOfCurrentLine - textEditingContext.globalGesturePosition.dy) /
              widget.dragResistance,
    );

    // The raw position, tracking the gesture directly.
    final Offset rawMagnifierPosition = Offset(
      textEditingContext.globalGesturePosition.dx - CupertinoMagnifier.kDefaultSize.width / 2,
      verticalPositionOfLens -
          (CupertinoMagnifier.kDefaultSize.height - CupertinoMagnifier.kMagnifierAboveFocalPoint),
    );

    final Rect screenRect = Offset.zero & MediaQuery.sizeOf(context);

    // Adjust the magnifier position so that it never exists outside the horizontal
    // padding.
    final Offset adjustedMagnifierPosition =
        MagnifierController.shiftWithinBounds(
          bounds: Rect.fromLTRB(
            screenRect.left + widget.horizontalScreenEdgePadding,
            // iOS doesn't reposition for Y, so we should expand the threshold
            // so we can send the whole magnifier out of bounds if need be.
            screenRect.top -
                (CupertinoMagnifier.kDefaultSize.height +
                    CupertinoMagnifier.kMagnifierAboveFocalPoint),
            screenRect.right - widget.horizontalScreenEdgePadding,
            screenRect.bottom +
                (CupertinoMagnifier.kDefaultSize.height +
                    CupertinoMagnifier.kMagnifierAboveFocalPoint),
          ),
          rect: rawMagnifierPosition & CupertinoMagnifier.kDefaultSize,
        ).topLeft;

    setState(() {
      _currentAdjustedMagnifierPosition = adjustedMagnifierPosition;
      // The lens should always point to the center of the line.
      _verticalFocalPointAdjustment = verticalCenterOfCurrentLine - verticalPositionOfLens;
    });
  }

  @override
  Widget build(BuildContext context) {
    final CupertinoThemeData themeData = CupertinoTheme.of(context);
    return AnimatedPositioned(
      duration: CupertinoTextMagnifier._kDragAnimationDuration,
      curve: widget.animationCurve,
      left: _currentAdjustedMagnifierPosition.dx,
      top: _currentAdjustedMagnifierPosition.dy,
      child: CupertinoMagnifier(
        inOutAnimation: _ioAnimation,
        additionalFocalPointOffset: Offset(0, _verticalFocalPointAdjustment),
        borderSide: BorderSide(color: themeData.primaryColor, width: 2.0),
      ),
    );
  }
}

/// A [RawMagnifier] used for magnifying text in cases where a user's
/// finger may be blocking the point of interest, like a selection handle.
///
/// {@tool dartpad}
/// This sample demonstrates how to use [CupertinoMagnifier].
///
/// ** See code in examples/api/lib/widgets/magnifier/cupertino_magnifier.0.dart **
/// {@end-tool}
///
/// [CupertinoMagnifier] is a wrapper around [RawMagnifier] that handles styling
/// and transitions.
///
/// {@macro flutter.widgets.magnifier.intro}
///
/// See also:
///
/// * [RawMagnifier], the backing implementation.
/// * [CupertinoTextMagnifier], a widget that positions [CupertinoMagnifier] based on
/// [MagnifierInfo].
/// * [MagnifierController], the controller for this magnifier.
class CupertinoMagnifier extends StatelessWidget {
  /// Creates a [RawMagnifier] in the Cupertino style.
  ///
  /// The default constructor parameters and constants were eyeballed on
  /// an iPhone 16 iOS v18.1.
  const CupertinoMagnifier({
    super.key,
    this.size = kDefaultSize,
    this.borderRadius = const BorderRadius.all(Radius.elliptical(60, 50)),
    this.additionalFocalPointOffset = Offset.zero,
    this.shadows = const <BoxShadow>[
      BoxShadow(
        color: Color.fromARGB(25, 0, 0, 0),
        blurRadius: 11,
        spreadRadius: 0.2,
        blurStyle: BlurStyle.outer,
      ),
    ],
    this.clipBehavior = Clip.none,
    this.borderSide = const BorderSide(color: Color.fromARGB(255, 0, 124, 255), width: 2.0),
    this.inOutAnimation,
    this.magnificationScale = 1.0,
  }) : assert(magnificationScale > 0, 'The magnification scale should be greater than zero.');

  /// A list of shadows cast by the [Magnifier].
  ///
  /// If the shadows use a [BlurStyle] that paints inside the shape, or if they
  /// are offset, then a [clipBehavior] that enables clipping (such as
  /// [Clip.hardEdge]) is recommended, otherwise the shadow will occlude the
  /// magnifier (the shadow is drawn above the magnifier so as to not be
  /// included in the magnified image).
  ///
  /// A shadow that uses [BlurStyle.outer] and is not offset does not need
  /// clipping.
  ///
  /// By default, the [shadows] are not offset and use [BlurStyle.outer], and
  /// correspondingly the default [clipBehavior] is [Clip.none].
  final List<BoxShadow> shadows;

  /// Whether and how to clip the [shadows] that render inside the loupe.
  ///
  /// Defaults to [Clip.none], which is useful if the shadow will not paint
  /// where the magnified image appears, or if doing so is intentional (e.g. to
  /// blur the edges of the magnified image).
  ///
  /// The default configuration of [CupertinoMagnifier] does not render inside
  /// the loupe (the shadows are not offset and use [BlurStyle.outer]).
  ///
  /// Other values (e.g. [Clip.hardEdge]) are recommended when the [shadows]
  /// have an offset.
  ///
  /// See the discussion at [shadows].
  final Clip clipBehavior;

  /// The border, or "rim", of this magnifier.
  ///
  /// This border is drawn on a [RoundedRectangleBorder] with radius
  /// [borderRadius], and increases the [size] of the magnifier by the
  /// [BorderSide.width].
  final BorderSide borderSide;

  /// The vertical offset that the magnifier is along the Y axis above
  /// the focal point.
  static const double kMagnifierAboveFocalPoint = -26;

  /// The default size of the magnifier.
  ///
  /// This is public so that positioners can choose to depend on it, although
  /// it is overridable.
  static const Size kDefaultSize = Size(80, 47.5);

  /// The duration that this magnifier animates in / out for.
  ///
  /// The animation is a translation and a fade. The translation
  /// begins at the focal point, and ends at [kMagnifierAboveFocalPoint].
  /// The opacity begins at 0 and ends at 1.
  static const Duration _kInOutAnimationDuration = Duration(milliseconds: 150);

  /// The size of this magnifier.
  ///
  /// The size does not include the [borderSide] or [shadows].
  final Size size;

  /// The border radius of this magnifier.
  ///
  /// The magnifier's shape is a [RoundedRectangleBorder] with this radius.
  final BorderRadius borderRadius;

  /// This [RawMagnifier]'s controller.
  ///
  /// Since [CupertinoMagnifier] has no knowledge of shown / hidden state,
  /// this animation should be driven by an external actor.
  final Animation<double>? inOutAnimation;

  /// Any additional focal point offset, applied over the regular focal
  /// point offset defined in [kMagnifierAboveFocalPoint].
  final Offset additionalFocalPointOffset;

  /// The magnification scale for the magnifier.
  ///
  /// Defaults to 1.0, which indicates that the magnifier does not apply any magnification.
  final double magnificationScale;

  @override
  Widget build(BuildContext context) {
    Offset focalPointOffset = Offset(0, (kDefaultSize.height / 2) - kMagnifierAboveFocalPoint);
    focalPointOffset.scale(1, inOutAnimation?.value ?? 1);
    focalPointOffset += additionalFocalPointOffset;

    return Transform.translate(
      offset:
          Offset.lerp(
            const Offset(0, -kMagnifierAboveFocalPoint),
            Offset.zero,
            inOutAnimation?.value ?? 1,
          )!,
      child: RawMagnifier(
        size: size,
        focalPointOffset: focalPointOffset,
        decoration: MagnifierDecoration(
          opacity: inOutAnimation?.value ?? 1,
          shape: RoundedRectangleBorder(borderRadius: borderRadius, side: borderSide),
          shadows: shadows,
        ),
        clipBehavior: clipBehavior,
        magnificationScale: magnificationScale,
      ),
    );
  }
}
