// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'text_selection.dart';
library;

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'container.dart';
import 'framework.dart';
import 'inherited_theme.dart';
import 'navigator.dart';
import 'overlay.dart';

/// Signature for a builder that builds a [Widget] with a [MagnifierController].
///
/// The builder is called exactly once per magnifier.
///
/// If the `controller` parameter's [MagnifierController.animationController]
/// field is set (by the builder) to an [AnimationController], the
/// [MagnifierController] will drive the animation during entry and exit.
///
/// The `magnifierInfo` parameter is updated with new [MagnifierInfo] instances
/// during the lifetime of the built magnifier, e.g. as the user moves their
/// finger around the text field.
typedef MagnifierBuilder =
    Widget? Function(
      BuildContext context,
      MagnifierController controller,
      ValueNotifier<MagnifierInfo> magnifierInfo,
    );

/// A data class that contains the geometry information of text layouts
/// and selection gestures, used to position magnifiers.
@immutable
class MagnifierInfo {
  /// Constructs a [MagnifierInfo] from provided geometry values.
  const MagnifierInfo({
    required this.globalGesturePosition,
    required this.caretRect,
    required this.fieldBounds,
    required this.currentLineBoundaries,
  });

  /// Const [MagnifierInfo] with all values set to 0.
  static const MagnifierInfo empty = MagnifierInfo(
    globalGesturePosition: Offset.zero,
    caretRect: Rect.zero,
    currentLineBoundaries: Rect.zero,
    fieldBounds: Rect.zero,
  );

  /// The offset of the gesture position that the magnifier should be shown at.
  final Offset globalGesturePosition;

  /// The rect of the current line the magnifier should be shown at, without
  /// taking into account any padding of the field; only the position of the
  /// first and last character.
  final Rect currentLineBoundaries;

  /// The rect of the handle that the magnifier should follow.
  final Rect caretRect;

  /// The bounds of the entire text field that the magnifier is bound to.
  final Rect fieldBounds;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MagnifierInfo &&
        other.globalGesturePosition == globalGesturePosition &&
        other.caretRect == caretRect &&
        other.currentLineBoundaries == currentLineBoundaries &&
        other.fieldBounds == fieldBounds;
  }

  @override
  int get hashCode =>
      Object.hash(globalGesturePosition, caretRect, fieldBounds, currentLineBoundaries);

  @override
  String toString() {
    return '${objectRuntimeType(this, 'MagnifierInfo')}('
        'position: $globalGesturePosition, '
        'line: $currentLineBoundaries, '
        'caret: $caretRect, '
        'field: $fieldBounds'
        ')';
  }
}

/// A configuration object for a magnifier (e.g. in a text field).
///
/// In general, most features of the magnifier can be configured by controlling
/// the widgets built by the [magnifierBuilder].
class TextMagnifierConfiguration {
  /// Constructs a [TextMagnifierConfiguration] from parts.
  ///
  /// If [magnifierBuilder] is null, a default [MagnifierBuilder] will be used
  /// that does not build a magnifier.
  const TextMagnifierConfiguration({
    MagnifierBuilder? magnifierBuilder,
    this.shouldDisplayHandlesInMagnifier = true,
  }) : _magnifierBuilder = magnifierBuilder;

  /// The builder callback that creates the widget that renders the magnifier.
  MagnifierBuilder get magnifierBuilder => _magnifierBuilder ?? _none;
  final MagnifierBuilder? _magnifierBuilder;

  static Widget? _none(
    BuildContext context,
    MagnifierController controller,
    ValueNotifier<MagnifierInfo> magnifierInfo,
  ) => null;

  /// Whether a magnifier should show the text editing handles or not.
  ///
  /// This flag is used by [SelectionOverlay.showMagnifier] to control the order
  /// of layers in the rendering; specifically, whether to place the layer
  /// containing the handles above or below the layer containing the magnifier
  /// in the [Overlay].
  final bool shouldDisplayHandlesInMagnifier;

  /// A constant for a [TextMagnifierConfiguration] that is disabled, meaning it
  /// never builds anything, regardless of platform.
  static const TextMagnifierConfiguration disabled = TextMagnifierConfiguration();
}

/// A controller for a magnifier.
///
/// [MagnifierController]'s main benefit over holding a raw [OverlayEntry] is that
/// [MagnifierController] will handle logic around waiting for a magnifier to animate in or out.
///
/// If a magnifier chooses to have an entry / exit animation, it should provide the animation
/// controller to [MagnifierController.animationController]. [MagnifierController] will then drive
/// the [AnimationController] and wait for it to be complete before removing it from the
/// [Overlay].
///
/// To check the status of the magnifier, see [MagnifierController.shown].
class MagnifierController {
  /// If there is no in / out animation for the magnifier, [animationController] should be left
  /// null.
  MagnifierController({this.animationController}) {
    animationController?.value = 0;
  }

  /// The controller that will be driven in / out when show / hide is triggered,
  /// respectively.
  AnimationController? animationController;

  /// The magnifier's [OverlayEntry], if currently in the overlay.
  ///
  /// This is exposed so that other overlay entries can be positioned above or
  /// below this [overlayEntry]. Anything in the paint order after the
  /// [RawMagnifier] in this [OverlayEntry] will not be displayed in the
  /// magnifier; if it is desired for an overlay entry to be displayed in the
  /// magnifier, it _must_ be positioned below the magnifier.
  ///
  /// {@tool snippet}
  /// ```dart
  /// void magnifierShowExample(BuildContext context) {
  ///   final MagnifierController myMagnifierController = MagnifierController();
  ///
  ///   // Placed below the magnifier, so it will show.
  ///   Overlay.of(context).insert(OverlayEntry(
  ///       builder: (BuildContext context) => const Text('I WILL display in the magnifier')));
  ///
  ///   // Will display in the magnifier, since this entry was passed to show.
  ///   final OverlayEntry displayInMagnifier = OverlayEntry(
  ///       builder: (BuildContext context) =>
  ///           const Text('I WILL display in the magnifier'));
  ///
  ///   Overlay.of(context)
  ///       .insert(displayInMagnifier);
  ///   myMagnifierController.show(
  ///       context: context,
  ///       below: displayInMagnifier,
  ///       builder: (BuildContext context) => const RawMagnifier(
  ///             size: Size(100, 100),
  ///           ));
  ///
  ///   // By default, new entries will be placed over the top entry.
  ///   Overlay.of(context).insert(OverlayEntry(
  ///       builder: (BuildContext context) => const Text('I WILL NOT display in the magnifier')));
  ///
  ///   Overlay.of(context).insert(
  ///       below:
  ///           myMagnifierController.overlayEntry, // Explicitly placed below the magnifier.
  ///       OverlayEntry(
  ///           builder: (BuildContext context) => const Text('I WILL display in the magnifier')));
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// To check if a magnifier is in the overlay, use [shown]. The [overlayEntry]
  /// field may be non-null even when the magnifier is not visible.
  OverlayEntry? get overlayEntry => _overlayEntry;
  OverlayEntry? _overlayEntry;

  /// Whether the magnifier is currently being shown.
  ///
  /// This is false when nothing is in the overlay, when the
  /// [animationController] is in the [AnimationStatus.dismissed] state, or when
  /// the [animationController] is animating out (i.e. in the
  /// [AnimationStatus.reverse] state).
  ///
  /// It is true in the opposite cases, i.e. when the overlay is not empty, and
  /// either the [animationController] is null, in the
  /// [AnimationStatus.completed] state, or in the [AnimationStatus.forward]
  /// state.
  bool get shown => overlayEntry != null && (animationController?.isForwardOrCompleted ?? true);

  /// Displays the magnifier.
  ///
  /// Returns a future that completes when the magnifier is fully shown, i.e. done
  /// with its entry animation.
  ///
  /// To control what overlays are shown in the magnifier, use `below`. See
  /// [overlayEntry] for more details on how to utilize `below`.
  ///
  /// If the magnifier already exists (i.e. [overlayEntry] != null), then [show]
  /// will replace the old overlay without playing an exit animation. Consider
  /// awaiting [hide] first, to animate from the old magnifier to the new one.
  Future<void> show({
    required BuildContext context,
    required WidgetBuilder builder,
    Widget? debugRequiredFor,
    OverlayEntry? below,
  }) async {
    _overlayEntry?.remove();
    _overlayEntry?.dispose();

    final OverlayState overlayState = Overlay.of(
      context,
      rootOverlay: true,
      debugRequiredFor: debugRequiredFor,
    );

    final CapturedThemes capturedThemes = InheritedTheme.capture(
      from: context,
      to: Navigator.maybeOf(context)?.context,
    );

    _overlayEntry = OverlayEntry(
      builder: (BuildContext context) => capturedThemes.wrap(builder(context)),
    );
    overlayState.insert(overlayEntry!, below: below);

    if (animationController != null) {
      await animationController?.forward();
    }
  }

  /// Schedules a hide of the magnifier.
  ///
  /// If this [MagnifierController] has an [AnimationController],
  /// then [hide] reverses the animation controller and waits
  /// for the animation to complete. Then, if [removeFromOverlay]
  /// is true, remove the magnifier from the overlay.
  ///
  /// In general, `removeFromOverlay` should be true, unless
  /// the magnifier needs to preserve states between shows / hides.
  ///
  /// See also:
  ///
  ///  * [removeFromOverlay] which removes the [OverlayEntry] from the [Overlay]
  ///    synchronously.
  Future<void> hide({bool removeFromOverlay = true}) async {
    if (overlayEntry == null) {
      return;
    }

    if (animationController != null) {
      await animationController?.reverse();
    }

    if (removeFromOverlay) {
      this.removeFromOverlay();
    }
  }

  /// Remove the [OverlayEntry] from the [Overlay].
  ///
  /// This method removes the [OverlayEntry] synchronously,
  /// regardless of exit animation: this leads to abrupt removals
  /// of [OverlayEntry]s with animations.
  ///
  /// To allow the [OverlayEntry] to play its exit animation, consider calling
  /// [hide] instead, with `removeFromOverlay` set to true, and optionally await
  /// the returned Future.
  @visibleForTesting
  void removeFromOverlay() {
    _overlayEntry?.remove();
    _overlayEntry?.dispose();
    _overlayEntry = null;
  }

  /// A utility for calculating a new [Rect] from [rect] such that
  /// [rect] is fully constrained within [bounds].
  ///
  /// Any point in the output rect is guaranteed to also be a point contained in [bounds].
  ///
  /// It is a runtime error for [rect].width to be greater than [bounds].width,
  /// and it is also an error for [rect].height to be greater than [bounds].height.
  ///
  /// This algorithm translates [rect] the shortest distance such that it is entirely within
  /// [bounds].
  ///
  /// If [rect] is already within [bounds], no shift will be applied to [rect] and
  /// [rect] will be returned as-is.
  ///
  /// It is perfectly valid for the output rect to have a point along the edge of the
  /// [bounds]. If the desired output rect requires that no edges are parallel to edges
  /// of [bounds], see [Rect.deflate] by 1 on [bounds] to achieve this effect.
  static Rect shiftWithinBounds({required Rect rect, required Rect bounds}) {
    assert(
      rect.width <= bounds.width,
      'attempted to shift $rect within $bounds, but the rect has a greater width.',
    );
    assert(
      rect.height <= bounds.height,
      'attempted to shift $rect within $bounds, but the rect has a greater height.',
    );

    Offset rectShift = Offset.zero;
    if (rect.left < bounds.left) {
      rectShift += Offset(bounds.left - rect.left, 0);
    } else if (rect.right > bounds.right) {
      rectShift += Offset(bounds.right - rect.right, 0);
    }

    if (rect.top < bounds.top) {
      rectShift += Offset(0, bounds.top - rect.top);
    } else if (rect.bottom > bounds.bottom) {
      rectShift += Offset(0, bounds.bottom - rect.bottom);
    }

    return rect.shift(rectShift);
  }
}

/// The decorations to put around the loupe in a [RawMagnifier].
///
/// See also:
///
///  * [Decoration], a more general solution for [DecoratedBox].
@immutable
class MagnifierDecoration {
  /// Constructs a [MagnifierDecoration].
  ///
  /// By default, [MagnifierDecoration] is a rectangular magnifier with no
  /// shadows, and fully opaque.
  const MagnifierDecoration({
    this.opacity = 1.0,
    this.shadows,
    this.shape = const RoundedRectangleBorder(),
  });

  // TODO(ianh): deprecate [opacity] (moving it to [RawMagnifier]), and then
  // once [opacity] can be removed, replace [MagnifierDecoration] with a
  // `typedef` to [ShapeDecoration] and make anywhere that accepts a
  // [MagnifierDecoration] accept a [ShapeDecoration] instead. This would allow
  // magnifiers that don't offset the shadows to use the decoration to paint
  // over the loupe rather than having to have a Stack of widgets to do so.

  /// The opacity of the magnifier and decorations around the magnifier.
  ///
  /// When this is 1.0, the magnified image shows in the [shape] of the
  /// magnifier. When this is less than 1.0, the magnified image is transparent
  /// and shows through the unmagnified background.
  ///
  /// Generally this is only useful for animating the magnifier in and out, as a
  /// transparent magnifier looks quite confusing.
  final double opacity;

  /// A list of shadows cast by the [shape].
  ///
  /// If the shadows are offset, consider setting [RawMagnifier.clipBehavior] to
  /// [Clip.hardEdge] (or similar) to ensure the shadow does not occlude the
  /// magnifier (the shadow is drawn above the magnifier).
  ///
  /// If the shadows are _not_ offset, consider using [BlurStyle.outer] in the
  /// shadows instead, to avoid having to introduce a clip.
  ///
  /// In the event that [shape] consists of a stack of borders, the shadow is
  /// drawn using the bounds of the last one.
  ///
  /// See also:
  ///
  ///  * [kElevationToShadow], which defines some shadows for Material design.
  ///    Those shadows use [BlurStyle.normal] and may need to be converted to
  ///    [BlurStyle.outer] for use with [MagnifierDecoration].
  final List<BoxShadow>? shadows;

  /// The shape of the magnifier and the outline (border) around it.
  ///
  /// Shapes can be stacked (using the `+` operator). In that case, the
  /// magnifier and shadow are drawn according to the outside edge of the last
  /// shape, with the borders painted on top.
  final ShapeBorder shape;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MagnifierDecoration &&
        other.opacity == opacity &&
        listEquals<BoxShadow>(other.shadows, shadows) &&
        other.shape == shape;
  }

  @override
  int get hashCode =>
      Object.hash(opacity, shape, shadows == null ? null : Object.hashAll(shadows!));
}

/// A common base class for magnifiers.
///
/// {@tool dartpad}
/// This sample demonstrates what a magnifier is, and how it can be used.
///
/// ** See code in examples/api/lib/widgets/magnifier/magnifier.0.dart **
/// {@end-tool}
///
/// {@template flutter.widgets.magnifier.intro}
/// This magnifying glass is useful for scenarios on mobile devices where
/// the user's finger may be covering part of the screen where a granular
/// action is being performed, such as navigating a small cursor with a drag
/// gesture, on an image or text.
/// {@endtemplate}
///
/// A magnifier can be conveniently managed by [MagnifierController], which handles
/// showing and hiding the magnifier, with an optional entry / exit animation.
///
/// See:
/// * [MagnifierController], a controller to handle magnifiers in an overlay.
class RawMagnifier extends StatelessWidget {
  /// Constructs a [RawMagnifier].
  ///
  /// By default, this magnifier uses the default [MagnifierDecoration] (which
  /// draws nothing), the focal point is directly under the magnifier, and there
  /// is no magnification; this means that a default magnifier will be entirely
  /// invisible to the naked eye, painting exactly what is under it, exactly
  /// where it was painted originally.
  const RawMagnifier({
    super.key,
    this.child,
    this.decoration = const MagnifierDecoration(),
    this.clipBehavior = Clip.none,
    this.focalPointOffset = Offset.zero,
    this.magnificationScale = 1,
    required this.size,
  }) : assert(magnificationScale != 0, 'Magnification scale of 0 results in undefined behavior.');

  /// An optional widget to position inside the len of the [RawMagnifier].
  ///
  /// This is positioned over the [RawMagnifier] - it may be useful for tinting the
  /// [RawMagnifier], or drawing a crosshair-like UI.
  final Widget? child;

  /// This magnifier's decoration.
  ///
  /// This sets the shape of the loupe, plus any borders and shadows that it
  /// casts. The default has no border and no shadow; combined with the default
  /// [magnificationScale] of 1.0, this results in the magnifier having no
  /// visible effect.
  ///
  /// If the [decoration] has a [MagnifierDecoration.shadows] that uses offset
  /// shadows or uses a [BlurStyle] that would obscure the magnified image,
  /// consider setting [clipBehavior] to [Clip.hardEdge] (or similar) to ensure
  /// the magnified image is visible.
  final MagnifierDecoration decoration;

  /// Whether and how to clip the parts of [decoration] that render inside the
  /// loupe.
  ///
  /// Defaults to [Clip.none].
  ///
  /// See the discussion at [decoration].
  final Clip clipBehavior;

  /// The offset of the magnifier from [RawMagnifier]'s center.
  ///
  /// {@template flutter.widgets.magnifier.offset}
  /// For example, if [RawMagnifier] is globally positioned at Offset(100, 100),
  /// and [focalPointOffset] is Offset(-20, -20), then [RawMagnifier] will see
  /// the content at global offset (80, 80).
  ///
  /// If left as [Offset.zero], the [RawMagnifier] will show the content that
  /// is directly below it.
  /// {@endtemplate}
  final Offset focalPointOffset;

  /// How "zoomed in" the magnification subject is in the lens.
  ///
  /// The default is 1.0, which is no magnification.
  final double magnificationScale;

  /// The size of the magnifier.
  ///
  /// This does not include the border from the [decoration]; it only includes
  /// the size of the magnifier.
  final Size size;

  @override
  Widget build(BuildContext context) {
    return Stack(
      clipBehavior: Clip.none,
      alignment: Alignment.center,
      children: <Widget>[
        // The magnified image is clipped to the outer path of the shape.
        ClipPath.shape(
          shape: decoration.shape,
          child: Opacity(
            opacity: decoration.opacity,
            child: _Magnifier(
              focalPointOffset: focalPointOffset,
              magnificationScale: magnificationScale,
              child: SizedBox.fromSize(size: size, child: child),
            ),
          ),
        ),
        // Because `BackdropFilter` will filter any widgets before it, we apply
        // these styles after (i.e. in a younger sibling) to avoid the magnifier
        // from seeing its own styling.
        IgnorePointer(
          child: Opacity(
            opacity: decoration.opacity,
            child: ClipPath(
              clipBehavior: clipBehavior,
              clipper: _NegativeClip(shape: decoration.shape),
              child: DecoratedBox(
                decoration: ShapeDecoration(shape: decoration.shape, shadows: decoration.shadows),
                child: SizedBox.fromSize(size: size),
              ),
            ),
          ),
        ),
      ],
    );
  }
}

// A clip that renders everything except the inside of a shape.
class _NegativeClip extends CustomClipper<Path> {
  _NegativeClip({required this.shape});

  final ShapeBorder shape;

  @override
  Path getClip(Size size) {
    return Path()
      ..fillType = PathFillType.evenOdd
      ..addRect(Rect.largest)
      ..addPath(shape.getInnerPath(Offset.zero & size), Offset.zero);
  }

  @override
  bool shouldReclip(_NegativeClip oldClipper) => oldClipper.shape != shape;
}

class _Magnifier extends SingleChildRenderObjectWidget {
  const _Magnifier({super.child, this.magnificationScale = 1, this.focalPointOffset = Offset.zero});

  // The Offset that the center of the _Magnifier points to, relative
  // to the center of the magnifier.
  final Offset focalPointOffset;

  // The enlarge multiplier of the magnification.
  //
  // If equal to 1.0, the content in the magnifier is true to its real size.
  // If greater than 1.0, the content appears bigger in the magnifier.
  final double magnificationScale;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderMagnification(focalPointOffset, magnificationScale);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderMagnification renderObject) {
    renderObject
      ..focalPointOffset = focalPointOffset
      ..magnificationScale = magnificationScale;
  }
}

class _RenderMagnification extends RenderProxyBox {
  _RenderMagnification(this._focalPointOffset, this._magnificationScale, {RenderBox? child})
    : super(child);

  Offset get focalPointOffset => _focalPointOffset;
  Offset _focalPointOffset;
  set focalPointOffset(Offset value) {
    if (_focalPointOffset == value) {
      return;
    }
    _focalPointOffset = value;
    markNeedsPaint();
  }

  double get magnificationScale => _magnificationScale;
  double _magnificationScale;
  set magnificationScale(double value) {
    if (_magnificationScale == value) {
      return;
    }
    _magnificationScale = value;
    markNeedsPaint();
  }

  @override
  bool get alwaysNeedsCompositing => true;

  @override
  BackdropFilterLayer? get layer => super.layer as BackdropFilterLayer?;

  @override
  void paint(PaintingContext context, Offset offset) {
    final Offset thisCenter = Alignment.center.alongSize(size) + offset;
    final Matrix4 matrix =
        Matrix4.identity()
          ..translate(
            magnificationScale * ((focalPointOffset.dx * -1) - thisCenter.dx) + thisCenter.dx,
            magnificationScale * ((focalPointOffset.dy * -1) - thisCenter.dy) + thisCenter.dy,
          )
          ..scale(magnificationScale);
    final ImageFilter filter = ImageFilter.matrix(
      matrix.storage,
      filterQuality: FilterQuality.high,
    );

    if (layer == null) {
      layer = BackdropFilterLayer(filter: filter);
    } else {
      layer!.filter = filter;
    }

    context.pushLayer(layer!, super.paint, offset);
  }
}
