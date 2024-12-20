// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:collection';
library;

import 'dart:math' as math;
import 'dart:math';

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';

// Extracted from https://developer.apple.com/design/resources/.
// Default values have been updated to match iOS 17 figma file: https://www.figma.com/community/file/1248375255495415511.

// Minimum padding from edges of the segmented control to edges of
// encompassing widget.
const EdgeInsetsGeometry _kHorizontalItemPadding = EdgeInsets.symmetric(vertical: 2, horizontal: 3);

// The corner radius of the segmented control.
const Radius _kCornerRadius = Radius.circular(9);

// The corner radius of the thumb.
const Radius _kThumbRadius = Radius.circular(7);
// The amount of space by which to expand the thumb from the size of the currently
// selected child.
const EdgeInsets _kThumbInsets = EdgeInsets.symmetric(horizontal: 1);

// Minimum height of the segmented control.
const double _kMinSegmentedControlHeight = 28.0;

const Color _kSeparatorColor = Color(0x4D8E8E93);

const CupertinoDynamicColor _kThumbColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFFFFFFF),
  darkColor: Color(0xFF636366),
);

// The amount of space by which to inset each separator.
const EdgeInsets _kSeparatorInset = EdgeInsets.symmetric(vertical: 5);
const double _kSeparatorWidth = 1;
const Radius _kSeparatorRadius = Radius.circular(_kSeparatorWidth / 2);

// The minimum scale factor of the thumb, when being pressed on for a sufficient
// amount of time.
const double _kMinThumbScale = 0.95;

// The minimum horizontal distance between the edges of the separator and the
// closest child.
const double _kSegmentMinPadding = 10;

// The threshold value used in hasDraggedTooFar, for checking against the square
// L2 distance from the location of the current drag pointer, to the closest
// vertex of the CupertinoSlidingSegmentedControl's Rect.
//
// Both the mechanism and the value are speculated.
const double _kTouchYDistanceThreshold = 50.0 * 50.0;

// The minimum opacity of an unselected segment, when the user presses on the
// segment and it starts to fadeout.
//
// Inspected from iOS 17.5 simulator.
const double _kContentPressedMinOpacity = 0.2;

// Inspected from iOS 17.5 simulator.
const double _kFontSize = 13.0;

// Inspected from iOS 17.5 simulator.
const FontWeight _kFontWeight = FontWeight.w500;

// Inspected from iOS 17.5 simulator.
const FontWeight _kHighlightedFontWeight = FontWeight.w600;

// Inspected from iOS 17.5 simulator
const Color _kDisabledContentColor = Color.fromARGB(115, 122, 122, 122);

// The spring animation used when the thumb changes its rect.
final SpringSimulation _kThumbSpringAnimationSimulation = SpringSimulation(
  const SpringDescription(mass: 1, stiffness: 503.551, damping: 44.8799),
  0,
  1,
  0, // Every time a new spring animation starts the previous animation stops.
);

const Duration _kSpringAnimationDuration = Duration(milliseconds: 412);

const Duration _kOpacityAnimationDuration = Duration(milliseconds: 470);

const Duration _kHighlightAnimationDuration = Duration(milliseconds: 200);

class _Segment<T> extends StatefulWidget {
  const _Segment({
    required ValueKey<T> key,
    required this.child,
    required this.pressed,
    required this.highlighted,
    required this.isDragging,
    required this.enabled,
    required this.segmentLocation,
  }) : super(key: key);

  final Widget child;

  final bool pressed;
  final bool highlighted;
  final bool enabled;
  final _SegmentLocation segmentLocation;

  // Whether the thumb of the parent widget (CupertinoSlidingSegmentedControl)
  // is currently being dragged.
  final bool isDragging;

  bool get shouldFadeoutContent => pressed && !highlighted && enabled;
  bool get shouldScaleContent => pressed && highlighted && isDragging && enabled;

  @override
  _SegmentState<T> createState() => _SegmentState<T>();
}

class _SegmentState<T> extends State<_Segment<T>> with TickerProviderStateMixin<_Segment<T>> {
  late final AnimationController highlightPressScaleController;
  late Animation<double> highlightPressScaleAnimation;

  @override
  void initState() {
    super.initState();
    highlightPressScaleController = AnimationController(
      duration: _kOpacityAnimationDuration,
      value: widget.shouldScaleContent ? 1 : 0,
      vsync: this,
    );

    highlightPressScaleAnimation = highlightPressScaleController.drive(
      Tween<double>(begin: 1.0, end: _kMinThumbScale),
    );
  }

  @override
  void didUpdateWidget(_Segment<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    assert(oldWidget.key == widget.key);

    if (oldWidget.shouldScaleContent != widget.shouldScaleContent) {
      highlightPressScaleAnimation = highlightPressScaleController.drive(
        Tween<double>(
          begin: highlightPressScaleAnimation.value,
          end: widget.shouldScaleContent ? _kMinThumbScale : 1.0,
        ),
      );
      highlightPressScaleController.animateWith(_kThumbSpringAnimationSimulation);
    }
  }

  @override
  void dispose() {
    highlightPressScaleController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final Alignment scaleAlignment = switch (widget.segmentLocation) {
      _SegmentLocation.leftmost => Alignment.centerLeft,
      _SegmentLocation.rightmost => Alignment.centerRight,
      _SegmentLocation.inbetween => Alignment.center,
    };

    return MetaData(
      // Expand the hitTest area of this widget.
      behavior: HitTestBehavior.opaque,
      child: IndexedStack(
        alignment: Alignment.center,
        children: <Widget>[
          AnimatedOpacity(
            opacity: widget.shouldFadeoutContent ? _kContentPressedMinOpacity : 1,
            duration: _kOpacityAnimationDuration,
            curve: Curves.ease,
            child: AnimatedDefaultTextStyle(
              style: DefaultTextStyle.of(context).style.merge(
                TextStyle(
                  fontWeight: widget.highlighted ? _kHighlightedFontWeight : _kFontWeight,
                  fontSize: _kFontSize,
                  color: widget.enabled ? null : _kDisabledContentColor,
                ),
              ),
              duration: _kHighlightAnimationDuration,
              curve: Curves.ease,
              child: ScaleTransition(
                alignment: scaleAlignment,
                scale: highlightPressScaleAnimation,
                child: widget.child,
              ),
            ),
          ),
          // The entire widget will assume the size of this widget, so when a
          // segment's "highlight" animation plays the size of the parent stays
          // the same and will always be greater than equal to that of the
          // visible child (at index 0), to keep the size of the entire
          // SegmentedControl widget consistent throughout the animation.
          DefaultTextStyle.merge(
            style: const TextStyle(fontWeight: _kHighlightedFontWeight, fontSize: _kFontSize),
            child: widget.child,
          ),
        ],
      ),
    );
  }
}

// Fadeout the separator when either adjacent segment is highlighted.
class _SegmentSeparator extends StatefulWidget {
  const _SegmentSeparator({required ValueKey<int> key, required this.highlighted})
    : super(key: key);

  final bool highlighted;

  @override
  _SegmentSeparatorState createState() => _SegmentSeparatorState();
}

class _SegmentSeparatorState extends State<_SegmentSeparator>
    with TickerProviderStateMixin<_SegmentSeparator> {
  late final AnimationController separatorOpacityController;

  @override
  void initState() {
    super.initState();

    separatorOpacityController = AnimationController(
      duration: _kSpringAnimationDuration,
      value: widget.highlighted ? 0 : 1,
      vsync: this,
    );
  }

  @override
  void didUpdateWidget(_SegmentSeparator oldWidget) {
    super.didUpdateWidget(oldWidget);
    assert(oldWidget.key == widget.key);

    if (oldWidget.highlighted != widget.highlighted) {
      separatorOpacityController.animateTo(
        widget.highlighted ? 0 : 1,
        duration: _kSpringAnimationDuration,
        curve: Curves.ease,
      );
    }
  }

  @override
  void dispose() {
    separatorOpacityController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: separatorOpacityController,
      child: const SizedBox(width: _kSeparatorWidth),
      builder: (BuildContext context, Widget? child) {
        return Padding(
          padding: _kSeparatorInset,
          child: DecoratedBox(
            decoration: BoxDecoration(
              color: _kSeparatorColor.withOpacity(
                _kSeparatorColor.opacity * separatorOpacityController.value,
              ),
              borderRadius: const BorderRadius.all(_kSeparatorRadius),
            ),
            child: child,
          ),
        );
      },
    );
  }
}

/// An iOS 13 style segmented control.
///
/// Displays the widgets provided in the [Map] of [children] in a horizontal list.
/// It allows the user to select between a number of mutually exclusive options,
/// by tapping or dragging within the segmented control.
///
/// A segmented control can feature any [Widget] as one of the values in its
/// [Map] of [children]. The type T is the type of the [Map] keys used to identify
/// each widget and determine which widget is selected. As required by the [Map]
/// class, keys must be of consistent types and must be comparable. The [children]
/// argument must be an ordered [Map] such as a [LinkedHashMap], the ordering of
/// the keys will determine the order of the widgets in the segmented control.
///
/// The widget calls the [onValueChanged] callback *when a valid user gesture
/// completes on an unselected segment*. The map key associated with the newly
/// selected widget is returned in the [onValueChanged] callback. Typically,
/// widgets that use a segmented control will listen for the [onValueChanged]
/// callback and rebuild the segmented control with a new [groupValue] to update
/// which option is currently selected.
///
/// The [children] will be displayed in the order of the keys in the [Map],
/// along the current [TextDirection]. Each child widget will have the same size.
/// The height of the segmented control is determined by the height of the
/// tallest child widget. The width of each child will be the intrinsic width of
/// the widest child, or the available horizontal space divided by the number of
/// [children], which ever is smaller.
///
/// A segmented control may optionally be created with custom colors. The
/// [thumbColor], [backgroundColor] arguments can be used to override the
/// segmented control's colors from its defaults.
///
/// {@tool dartpad}
/// This example shows a [CupertinoSlidingSegmentedControl] with an enum type.
///
/// The callback provided to [onValueChanged] should update the state of
/// the parent [StatefulWidget] using the [State.setState] method, so that
/// the parent gets rebuilt; for example:
///
/// ** See code in examples/api/lib/cupertino/segmented_control/cupertino_sliding_segmented_control.0.dart **
/// {@end-tool}
/// See also:
///
///  * <https://developer.apple.com/design/human-interface-guidelines/ios/controls/segmented-controls/>
class CupertinoSlidingSegmentedControl<T extends Object> extends StatefulWidget {
  /// Creates an iOS-style segmented control bar.
  ///
  /// The [children] argument must be an ordered [Map] such as a
  /// [LinkedHashMap]. Further, the length of the [children] list must be
  /// greater than one.
  ///
  /// Each widget value in the map of [children] must have an associated key
  /// that uniquely identifies this widget. This key is what will be returned
  /// in the [onValueChanged] callback when a new value from the [children] map
  /// is selected.
  ///
  /// The [groupValue] is the currently selected value for the segmented control.
  /// If no [groupValue] is provided, or the [groupValue] is null, no widget will
  /// appear as selected. The [groupValue] must be either null or one of the keys
  /// in the [children] map.
  CupertinoSlidingSegmentedControl({
    super.key,
    required this.children,
    required this.onValueChanged,
    this.disabledChildren = const <Never>{},
    this.groupValue,
    this.thumbColor = _kThumbColor,
    this.padding = _kHorizontalItemPadding,
    this.backgroundColor = CupertinoColors.tertiarySystemFill,
    this.proportionalWidth = false,
  }) : assert(children.length >= 2),
       assert(
         groupValue == null || children.keys.contains(groupValue),
         'The groupValue must be either null or one of the keys in the children map.',
       );

  /// The identifying keys and corresponding widget values in the
  /// segmented control.
  ///
  /// This attribute must be an ordered [Map] such as a [LinkedHashMap]. Each
  /// widget is typically a single-line [Text] widget or an [Icon] widget.
  ///
  /// The map must have more than one entry.
  final Map<T, Widget> children;

  /// The set of identifying keys that correspond to the segments that should be
  /// disabled.
  ///
  /// Disabled children cannot be selected by dragging, but they can be selected
  /// programmatically. For example, if the [groupValue] is set to a disabled
  /// segment, the segment is still selected but the segment content looks disabled.
  ///
  /// If an enabled segment is selected by dragging gesture and becomes disabled
  /// before dragging finishes, [onValueChanged] will be triggered when finger is
  /// released and the disabled segment is selected.
  ///
  /// By default, all segments are selectable.
  final Set<T> disabledChildren;

  /// The identifier of the widget that is currently selected.
  ///
  /// This must be one of the keys in the [Map] of [children].
  /// If this attribute is null, no widget will be initially selected.
  final T? groupValue;

  /// The callback that is called when a new option is tapped.
  ///
  /// The segmented control passes the newly selected widget's associated key
  /// to the callback but does not actually change state until the parent
  /// widget rebuilds the segmented control with the new [groupValue].
  ///
  /// The callback provided to [onValueChanged] should update the state of
  /// the parent [StatefulWidget] using the [State.setState] method, so that
  /// the parent gets rebuilt; for example:
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// class SegmentedControlExample extends StatefulWidget {
  ///   const SegmentedControlExample({super.key});
  ///
  ///   @override
  ///   State createState() => SegmentedControlExampleState();
  /// }
  ///
  /// class SegmentedControlExampleState extends State<SegmentedControlExample> {
  ///   final Map<int, Widget> children = const <int, Widget>{
  ///     0: Text('Child 1'),
  ///     1: Text('Child 2'),
  ///   };
  ///
  ///   int? currentValue;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return CupertinoSlidingSegmentedControl<int>(
  ///       children: children,
  ///       onValueChanged: (int? newValue) {
  ///         setState(() {
  ///           currentValue = newValue;
  ///         });
  ///       },
  ///       groupValue: currentValue,
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  final ValueChanged<T?> onValueChanged;

  /// The color used to paint the rounded rect behind the [children] and the separators.
  ///
  /// The default value is [CupertinoColors.tertiarySystemFill]. The background
  /// will not be painted if null is specified.
  final Color backgroundColor;

  /// Determine whether segments have proportional widths based on their content.
  ///
  /// If false, all segments will have the same width, determined by the longest
  /// segment. If true, each segment's width will be determined by its individual
  /// content.
  ///
  /// If the max width of parent constraints is smaller than the width that the
  /// segmented control needs, The segment widths will scale down proportionally
  /// to ensure the segment control fits within the boundaries; similarly, if
  /// the min width of parent constraints is larger, the segment width will scales
  /// up to meet the min width requirement.
  ///
  /// Defaults to false.
  final bool proportionalWidth;

  /// The color used to paint the interior of the thumb that appears behind the
  /// currently selected item.
  ///
  /// The default value is a [CupertinoDynamicColor] that appears white in light
  /// mode and becomes a gray color in dark mode.
  final Color thumbColor;

  /// The amount of space by which to inset the [children].
  ///
  /// Defaults to `EdgeInsets.symmetric(vertical: 2, horizontal: 3)`.
  final EdgeInsetsGeometry padding;

  @override
  State<CupertinoSlidingSegmentedControl<T>> createState() => _SegmentedControlState<T>();
}

class _SegmentedControlState<T extends Object> extends State<CupertinoSlidingSegmentedControl<T>>
    with TickerProviderStateMixin<CupertinoSlidingSegmentedControl<T>> {
  late final AnimationController thumbController = AnimationController(
    duration: _kSpringAnimationDuration,
    value: 0,
    vsync: this,
  );
  Animatable<Rect?>? thumbAnimatable;

  late final AnimationController thumbScaleController = AnimationController(
    duration: _kSpringAnimationDuration,
    value: 0,
    vsync: this,
  );
  late Animation<double> thumbScaleAnimation = thumbScaleController.drive(
    Tween<double>(begin: 1, end: _kMinThumbScale),
  );

  final TapGestureRecognizer tap = TapGestureRecognizer();
  final HorizontalDragGestureRecognizer drag = HorizontalDragGestureRecognizer();
  final LongPressGestureRecognizer longPress = LongPressGestureRecognizer();
  final GlobalKey segmentedControlRenderWidgetKey = GlobalKey();

  @override
  void initState() {
    super.initState();

    // If the long press or horizontal drag recognizer gets accepted, we know for
    // sure the gesture is meant for the segmented control. Hand everything to
    // the drag gesture recognizer.
    final GestureArenaTeam team = GestureArenaTeam();
    longPress.team = team;
    drag.team = team;
    team.captain = drag;

    drag
      ..onDown = onDown
      ..onUpdate = onUpdate
      ..onEnd = onEnd
      ..onCancel = onCancel;

    tap.onTapUp = onTapUp;

    // Empty callback to enable the long press recognizer.
    longPress.onLongPress = () {};

    highlighted = widget.groupValue;
  }

  @override
  void didUpdateWidget(CupertinoSlidingSegmentedControl<T> oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Temporarily ignore highlight changes from the widget when the thumb is
    // being dragged. When the drag gesture finishes the widget will be forced
    // to build (see the onEnd method), and didUpdateWidget will be called again.
    if (!isThumbDragging && highlighted != widget.groupValue) {
      thumbController.animateWith(_kThumbSpringAnimationSimulation);
      thumbAnimatable = null;
      highlighted = widget.groupValue;
    }
  }

  @override
  void dispose() {
    thumbScaleController.dispose();
    thumbController.dispose();

    drag.dispose();
    tap.dispose();
    longPress.dispose();

    super.dispose();
  }

  // Whether the current drag gesture started on a selected segment. When this
  // flag is false, the `onUpdate` method does not update `highlighted`.
  // Otherwise the thumb can be dragged around in an ongoing drag gesture.
  bool? _startedOnSelectedSegment;

  // Whether the current drag gesture started on a disabled segment. When this
  // flag is true, drag gestures will be ignored.
  bool _startedOnDisabledSegment = false;

  // Whether an ongoing horizontal drag gesture that started on the thumb is
  // present. When true, defer/ignore changes to the `highlighted` variable
  // from other sources (except for semantics) until the gesture ends, preventing
  // them from interfering with the active drag gesture.
  bool get isThumbDragging => (_startedOnSelectedSegment ?? false) && !_startedOnDisabledSegment;

  // Converts local coordinate to segments.
  T segmentForXPosition(double dx) {
    final BuildContext currentContext = segmentedControlRenderWidgetKey.currentContext!;
    final _RenderSegmentedControl<T> renderBox =
        currentContext.findRenderObject()! as _RenderSegmentedControl<T>;

    final int numOfChildren = widget.children.length;
    assert(renderBox.hasSize);
    assert(numOfChildren >= 2);

    int segmentIndex = renderBox.getClosestSegmentIndex(dx);

    switch (Directionality.of(context)) {
      case TextDirection.ltr:
        break;
      case TextDirection.rtl:
        segmentIndex = numOfChildren - 1 - segmentIndex;
    }
    return widget.children.keys.elementAt(segmentIndex);
  }

  bool _hasDraggedTooFar(DragUpdateDetails details) {
    final RenderBox renderBox = context.findRenderObject()! as RenderBox;
    assert(renderBox.hasSize);
    final Size size = renderBox.size;
    final Offset offCenter = details.localPosition - Offset(size.width / 2, size.height / 2);
    final double l2 =
        math.pow(math.max(0.0, offCenter.dx.abs() - size.width / 2), 2) +
                math.pow(math.max(0.0, offCenter.dy.abs() - size.height / 2), 2)
            as double;
    return l2 > _kTouchYDistanceThreshold;
  }

  // The thumb shrinks when the user presses on it, and starts expanding when
  // the user lets go.
  // This animation must be synced with the segment scale animation (see the
  // _Segment widget) to make the overall animation look natural when the thumb
  // is not sliding.
  void _playThumbScaleAnimation({required bool isExpanding}) {
    thumbScaleAnimation = thumbScaleController.drive(
      Tween<double>(begin: thumbScaleAnimation.value, end: isExpanding ? 1 : _kMinThumbScale),
    );
    thumbScaleController.animateWith(_kThumbSpringAnimationSimulation);
  }

  void onHighlightChangedByGesture(T newValue) {
    if (highlighted == newValue) {
      return;
    }

    setState(() {
      highlighted = newValue;
    });
    // Additionally, start the thumb animation if the highlighted segment
    // changes. If the thumbController is already running, the render object's
    // paint method will create a new tween to drive the animation with.
    // TODO(LongCatIsLooong): https://github.com/flutter/flutter/issues/74356:
    // the current thumb will be painted at the same location twice (before and
    // after the new animation starts).
    thumbController.animateWith(_kThumbSpringAnimationSimulation);
    thumbAnimatable = null;
  }

  void onPressedChangedByGesture(T? newValue) {
    if (pressed != newValue) {
      setState(() {
        pressed = newValue;
      });
    }
  }

  void onTapUp(TapUpDetails details) {
    // No gesture should interfere with an ongoing thumb drag.
    if (isThumbDragging) {
      return;
    }
    final T segment = segmentForXPosition(details.localPosition.dx);
    onPressedChangedByGesture(null);
    if (segment != widget.groupValue && !widget.disabledChildren.contains(segment)) {
      widget.onValueChanged(segment);
    }
  }

  void onDown(DragDownDetails details) {
    final T touchDownSegment = segmentForXPosition(details.localPosition.dx);
    _startedOnSelectedSegment = touchDownSegment == highlighted;
    _startedOnDisabledSegment = widget.disabledChildren.contains(touchDownSegment);
    if (widget.disabledChildren.contains(touchDownSegment)) {
      return;
    }
    onPressedChangedByGesture(touchDownSegment);

    if (isThumbDragging) {
      _playThumbScaleAnimation(isExpanding: false);
    }
  }

  void onUpdate(DragUpdateDetails details) {
    // If drag gesture starts on disabled segment, no update needed.
    if (_startedOnDisabledSegment) {
      return;
    }

    // If drag gesture starts on enabled segment and dragging on disabled segment,
    // no update needed.
    final T touchDownSegment = segmentForXPosition(details.localPosition.dx);
    if (widget.disabledChildren.contains(touchDownSegment)) {
      return;
    }
    if (isThumbDragging) {
      onPressedChangedByGesture(touchDownSegment);
      onHighlightChangedByGesture(touchDownSegment);
    } else {
      final T? segment =
          _hasDraggedTooFar(details) ? null : segmentForXPosition(details.localPosition.dx);
      onPressedChangedByGesture(segment);
    }
  }

  void onEnd(DragEndDetails details) {
    final T? pressed = this.pressed;
    if (isThumbDragging) {
      _playThumbScaleAnimation(isExpanding: true);
      if (highlighted != widget.groupValue) {
        widget.onValueChanged(highlighted);
      }
    } else if (pressed != null) {
      onHighlightChangedByGesture(pressed);
      assert(pressed == highlighted);
      if (highlighted != widget.groupValue) {
        widget.onValueChanged(highlighted);
      }
    }

    onPressedChangedByGesture(null);
    _startedOnSelectedSegment = null;
  }

  void onCancel() {
    if (isThumbDragging) {
      _playThumbScaleAnimation(isExpanding: true);
    }
    onPressedChangedByGesture(null);
    _startedOnSelectedSegment = null;
  }

  // The segment the sliding thumb is currently located at, or animating to. It
  // may have a different value from widget.groupValue, since this widget does
  // not report a selection change via `onValueChanged` until the user stops
  // interacting with the widget (onTapUp). For example, the user can drag the
  // thumb around, and the `onValueChanged` callback will not be invoked until
  // the thumb is let go.
  T? highlighted;

  // The segment the user is currently pressing.
  T? pressed;

  @override
  Widget build(BuildContext context) {
    assert(widget.children.length >= 2);
    List<Widget> children = <Widget>[];
    bool isPreviousSegmentHighlighted = false;

    int index = 0;
    int? highlightedIndex;
    for (final MapEntry<T, Widget> entry in widget.children.entries) {
      final bool isHighlighted = highlighted == entry.key;
      if (isHighlighted) {
        highlightedIndex = index;
      }

      if (index != 0) {
        children.add(
          _SegmentSeparator(
            // Let separators be TextDirection-invariant. If the TextDirection
            // changes, the separators should mostly stay where they were.
            key: ValueKey<int>(index),
            highlighted: isPreviousSegmentHighlighted || isHighlighted,
          ),
        );
      }

      final TextDirection textDirection = Directionality.of(context);
      final _SegmentLocation segmentLocation = switch (textDirection) {
        TextDirection.ltr when index == 0 => _SegmentLocation.leftmost,
        TextDirection.ltr when index == widget.children.length - 1 => _SegmentLocation.rightmost,
        TextDirection.rtl when index == widget.children.length - 1 => _SegmentLocation.leftmost,
        TextDirection.rtl when index == 0 => _SegmentLocation.rightmost,
        TextDirection.ltr || TextDirection.rtl => _SegmentLocation.inbetween,
      };
      children.add(
        Semantics(
          button: true,
          onTap: () {
            if (widget.disabledChildren.contains(entry.key)) {
              return;
            }
            widget.onValueChanged(entry.key);
          },
          inMutuallyExclusiveGroup: true,
          selected: widget.groupValue == entry.key,
          child: MouseRegion(
            cursor: kIsWeb ? SystemMouseCursors.click : MouseCursor.defer,
            child: _Segment<T>(
              key: ValueKey<T>(entry.key),
              highlighted: isHighlighted,
              pressed: pressed == entry.key,
              isDragging: isThumbDragging,
              enabled: !widget.disabledChildren.contains(entry.key),
              segmentLocation: segmentLocation,
              child: entry.value,
            ),
          ),
        ),
      );

      index += 1;
      isPreviousSegmentHighlighted = isHighlighted;
    }

    assert((highlightedIndex == null) == (highlighted == null));

    switch (Directionality.of(context)) {
      case TextDirection.ltr:
        break;
      case TextDirection.rtl:
        children = children.reversed.toList(growable: false);
        if (highlightedIndex != null) {
          highlightedIndex = index - 1 - highlightedIndex;
        }
    }

    return UnconstrainedBox(
      constrainedAxis: Axis.horizontal,
      child: Container(
        // Clip the thumb shadow if it is outside of the segmented control. This
        // behavior is eyeballed by the iOS 17.5 simulator.
        clipBehavior: Clip.antiAlias,
        padding: widget.padding.resolve(Directionality.of(context)),
        decoration: BoxDecoration(
          borderRadius: const BorderRadius.all(_kCornerRadius),
          color: CupertinoDynamicColor.resolve(widget.backgroundColor, context),
        ),
        child: AnimatedBuilder(
          animation: thumbScaleAnimation,
          builder: (BuildContext context, Widget? child) {
            return _SegmentedControlRenderWidget<T>(
              key: segmentedControlRenderWidgetKey,
              highlightedIndex: highlightedIndex,
              thumbColor: CupertinoDynamicColor.resolve(widget.thumbColor, context),
              thumbScale: thumbScaleAnimation.value,
              proportionalWidth: widget.proportionalWidth,
              state: this,
              children: children,
            );
          },
        ),
      ),
    );
  }
}

class _SegmentedControlRenderWidget<T extends Object> extends MultiChildRenderObjectWidget {
  const _SegmentedControlRenderWidget({
    super.key,
    super.children,
    required this.highlightedIndex,
    required this.thumbColor,
    required this.thumbScale,
    required this.proportionalWidth,
    required this.state,
  });

  final int? highlightedIndex;
  final Color thumbColor;
  final double thumbScale;
  final bool proportionalWidth;
  final _SegmentedControlState<T> state;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderSegmentedControl<T>(
      highlightedIndex: highlightedIndex,
      thumbColor: thumbColor,
      thumbScale: thumbScale,
      proportionalWidth: proportionalWidth,
      state: state,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSegmentedControl<T> renderObject) {
    assert(renderObject.state == state);
    renderObject
      ..thumbColor = thumbColor
      ..thumbScale = thumbScale
      ..highlightedIndex = highlightedIndex
      ..proportionalWidth = proportionalWidth;
  }
}

class _SegmentedControlContainerBoxParentData extends ContainerBoxParentData<RenderBox> {}

enum _SegmentLocation { leftmost, rightmost, inbetween }

// The behavior of a UISegmentedControl as observed on iOS 13.1:
//
// 1. Tap up inside events will set the current selected index to the index of the
//    segment at the tap up location instantaneously (there might be animation but
//    the index change seems to happen before animation finishes), unless the tap
//    down event from the same touch event didn't happen within the segmented
//    control, in which case the touch event will be ignored entirely (will be
//    referring to these touch events as invalid touch events below).
//
// 2. A valid tap up event will also trigger the sliding CASpringAnimation (even
//    when it lands on the current segment), starting from the current `frame`
//    of the thumb. The previous sliding animation, if still playing, will be
//    removed and its velocity reset to 0. The sliding animation has a fixed
//    duration, regardless of the distance or transform.
//
// 3. When the sliding animation plays two other animations take place. In one animation
//    the content of the current segment gradually becomes "highlighted", turning the
//    font weight to semibold (CABasicAnimation, timingFunction = default, duration = 0.2).
//    The other is the separator fadein/fadeout animation (duration = 0.41).
//
// 4. A tap down event on the segment pointed to by the current selected
//    index will trigger a CABasicAnimation that shrinks the thumb to 95% of its
//    original size, even if the sliding animation is still playing. The
///   corresponding tap up event inverts the process (eyeballed).
//
// 5. A tap down event on other segments will trigger a CABasicAnimation
//    (timingFunction = default, duration = 0.47.) that fades out the content
//    from its current alpha, eventually reducing the alpha of that segment to
//    20% unless interrupted by a tap up event or the pointer moves out of the
//    region (either outside of the segmented control's vicinity or to a
//    different segment). The reverse animation has the same duration and timing
//    function.
class _RenderSegmentedControl<T extends Object> extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, ContainerBoxParentData<RenderBox>>,
        RenderBoxContainerDefaultsMixin<RenderBox, ContainerBoxParentData<RenderBox>> {
  _RenderSegmentedControl({
    required int? highlightedIndex,
    required Color thumbColor,
    required double thumbScale,
    required bool proportionalWidth,
    required this.state,
  }) : _highlightedIndex = highlightedIndex,
       _thumbColor = thumbColor,
       _thumbScale = thumbScale,
       _proportionalWidth = proportionalWidth;

  final _SegmentedControlState<T> state;

  // The current **Unscaled** Thumb Rect in this RenderBox's coordinate space.
  Rect? currentThumbRect;

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    state.thumbController.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    state.thumbController.removeListener(markNeedsPaint);
    super.detach();
  }

  double get thumbScale => _thumbScale;
  double _thumbScale;
  set thumbScale(double value) {
    if (_thumbScale == value) {
      return;
    }

    _thumbScale = value;
    if (state.highlighted != null) {
      markNeedsPaint();
    }
  }

  int? get highlightedIndex => _highlightedIndex;
  int? _highlightedIndex;
  set highlightedIndex(int? value) {
    if (_highlightedIndex == value) {
      return;
    }

    _highlightedIndex = value;
    markNeedsPaint();
  }

  Color get thumbColor => _thumbColor;
  Color _thumbColor;
  set thumbColor(Color value) {
    if (_thumbColor == value) {
      return;
    }
    _thumbColor = value;
    markNeedsPaint();
  }

  bool get proportionalWidth => _proportionalWidth;
  bool _proportionalWidth;
  set proportionalWidth(bool value) {
    if (_proportionalWidth == value) {
      return;
    }
    _proportionalWidth = value;
    markNeedsLayout();
  }

  @override
  void handleEvent(PointerEvent event, BoxHitTestEntry entry) {
    assert(debugHandleEvent(event, entry));
    // No gesture should interfere with an ongoing thumb drag.
    if (event is PointerDownEvent && !state.isThumbDragging) {
      state.tap.addPointer(event);
      state.longPress.addPointer(event);
      state.drag.addPointer(event);
    }
  }

  // Intrinsic Dimensions
  double get separatorWidth => _kSeparatorInset.horizontal + _kSeparatorWidth;
  double get totalSeparatorWidth => separatorWidth * (childCount ~/ 2);

  int getClosestSegmentIndex(double dx) {
    int index = 0;
    RenderBox? child = firstChild;
    while (child != null) {
      final _SegmentedControlContainerBoxParentData childParentData =
          child.parentData! as _SegmentedControlContainerBoxParentData;
      final double clampX = clampDouble(
        dx,
        childParentData.offset.dx,
        child.size.width + childParentData.offset.dx,
      );

      if (dx <= clampX) {
        break;
      }

      index++;
      child = nonSeparatorChildAfter(child);
    }

    final int segmentCount = childCount ~/ 2 + 1;
    // When the thumb is dragging out of bounds, the return result must be
    // smaller than segment count.
    return min(index, segmentCount - 1);
  }

  RenderBox? nonSeparatorChildAfter(RenderBox child) {
    final RenderBox? nextChild = childAfter(child);
    return nextChild == null ? null : childAfter(nextChild);
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    final int childCount = this.childCount ~/ 2 + 1;
    RenderBox? child = firstChild;
    double maxMinChildWidth = 0;
    while (child != null) {
      final double childWidth = child.getMinIntrinsicWidth(height);
      maxMinChildWidth = math.max(maxMinChildWidth, childWidth);
      child = nonSeparatorChildAfter(child);
    }
    return (maxMinChildWidth + 2 * _kSegmentMinPadding) * childCount + totalSeparatorWidth;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final int childCount = this.childCount ~/ 2 + 1;
    RenderBox? child = firstChild;
    double maxMaxChildWidth = 0;
    while (child != null) {
      final double childWidth = child.getMaxIntrinsicWidth(height);
      maxMaxChildWidth = math.max(maxMaxChildWidth, childWidth);
      child = nonSeparatorChildAfter(child);
    }
    return (maxMaxChildWidth + 2 * _kSegmentMinPadding) * childCount + totalSeparatorWidth;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    RenderBox? child = firstChild;
    double maxMinChildHeight = _kMinSegmentedControlHeight;
    while (child != null) {
      final double childHeight = child.getMinIntrinsicHeight(width);
      maxMinChildHeight = math.max(maxMinChildHeight, childHeight);
      child = nonSeparatorChildAfter(child);
    }
    return maxMinChildHeight;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    RenderBox? child = firstChild;
    double maxMaxChildHeight = _kMinSegmentedControlHeight;
    while (child != null) {
      final double childHeight = child.getMaxIntrinsicHeight(width);
      maxMaxChildHeight = math.max(maxMaxChildHeight, childHeight);
      child = nonSeparatorChildAfter(child);
    }
    return maxMaxChildHeight;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToHighestActualBaseline(baseline);
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! _SegmentedControlContainerBoxParentData) {
      child.parentData = _SegmentedControlContainerBoxParentData();
    }
  }

  double _getMaxChildWidth(BoxConstraints constraints) {
    final int childCount = this.childCount ~/ 2 + 1;
    double childWidth = (constraints.minWidth - totalSeparatorWidth) / childCount;
    RenderBox? child = firstChild;
    while (child != null) {
      childWidth = math.max(
        childWidth,
        child.getMaxIntrinsicWidth(double.infinity) + 2 * _kSegmentMinPadding,
      );
      child = nonSeparatorChildAfter(child);
    }
    return math.min(childWidth, (constraints.maxWidth - totalSeparatorWidth) / childCount);
  }

  double _getMaxChildHeight(BoxConstraints constraints, double childWidth) {
    double maxHeight = _kMinSegmentedControlHeight;
    RenderBox? child = firstChild;
    while (child != null) {
      final double boxHeight = child.getMaxIntrinsicHeight(childWidth);
      maxHeight = math.max(maxHeight, boxHeight);
      child = nonSeparatorChildAfter(child);
    }
    return maxHeight;
  }

  List<double> _getChildWidths(BoxConstraints constraints) {
    if (!proportionalWidth) {
      final double maxChildWidth = _getMaxChildWidth(constraints);
      final int segmentCount = childCount ~/ 2 + 1;
      return List<double>.filled(segmentCount, maxChildWidth);
    }

    final List<double> segmentWidths = <double>[];
    RenderBox? child = firstChild;
    while (child != null) {
      final double childWidth =
          child.getMaxIntrinsicWidth(double.infinity) + 2 * _kSegmentMinPadding;
      child = nonSeparatorChildAfter(child);
      segmentWidths.add(childWidth);
    }

    final double totalWidth = segmentWidths.sum;

    // If the sum of the children's width is larger than the allowed max width,
    // each segment width should scale down until the overall size can fit in
    // the parent constraints; similarly, if the sum of the children's width is
    // smaller than the allowed min width, each segment width should scale up
    // until the overall size can fit in the parent constraints.
    final double allowedMaxWidth = constraints.maxWidth - totalSeparatorWidth;
    final double allowedMinWidth = constraints.minWidth - totalSeparatorWidth;

    final double scale = clampDouble(totalWidth, allowedMinWidth, allowedMaxWidth) / totalWidth;
    if (scale != 1) {
      for (int i = 0; i < segmentWidths.length; i++) {
        segmentWidths[i] = segmentWidths[i] * scale;
      }
    }
    return segmentWidths;
  }

  Size _computeOverallSize(BoxConstraints constraints) {
    final double maxChildHeight = _getMaxChildHeight(constraints, constraints.maxWidth);
    return constraints.constrain(
      Size(_getChildWidths(constraints).sum + totalSeparatorWidth, maxChildHeight),
    );
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final List<double> segmentWidths = _getChildWidths(constraints);
    final double childHeight = _getMaxChildHeight(constraints, constraints.maxWidth);

    int index = 0;
    BaselineOffset baselineOffset = BaselineOffset.noBaseline;
    RenderBox? child = firstChild;
    while (child != null) {
      final BoxConstraints childConstraints = BoxConstraints.tight(
        Size(segmentWidths[index], childHeight),
      );
      baselineOffset = baselineOffset.minOf(
        BaselineOffset(child.getDryBaseline(childConstraints, baseline)),
      );

      child = nonSeparatorChildAfter(child);
      index++;
    }

    return baselineOffset.offset;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return _computeOverallSize(constraints);
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    final List<double> segmentWidths = _getChildWidths(constraints);

    final double childHeight = _getMaxChildHeight(constraints, double.infinity);
    final BoxConstraints separatorConstraints = BoxConstraints(
      minHeight: childHeight,
      maxHeight: childHeight,
    );
    RenderBox? child = firstChild;
    int index = 0;
    double start = 0;
    while (child != null) {
      final BoxConstraints childConstraints = BoxConstraints.tight(
        Size(segmentWidths[index ~/ 2], childHeight),
      );
      child.layout(index.isEven ? childConstraints : separatorConstraints, parentUsesSize: true);
      final _SegmentedControlContainerBoxParentData childParentData =
          child.parentData! as _SegmentedControlContainerBoxParentData;
      final Offset childOffset = Offset(start, 0);
      childParentData.offset = childOffset;
      start += child.size.width;
      assert(
        index.isEven || child.size.width == _kSeparatorWidth + _kSeparatorInset.horizontal,
        '${child.size.width} != ${_kSeparatorWidth + _kSeparatorInset.horizontal}',
      );
      child = childAfter(child);
      index += 1;
    }
    size = _computeOverallSize(constraints);
  }

  // This method is used to convert the original unscaled thumb rect painted in
  // the previous frame, to a Rect that is within the valid boundary defined by
  // the child segments.
  //
  // The overall size does not include that of the thumb. That is, if the thumb
  // is located at the first or the last segment, the thumb can get cut off if
  // one of the values in _kThumbInsets is positive.
  Rect? moveThumbRectInBound(Rect? thumbRect, List<RenderBox> children) {
    assert(hasSize);
    assert(children.length >= 2);
    if (thumbRect == null) {
      return null;
    }

    final Offset firstChildOffset =
        (children.first.parentData! as _SegmentedControlContainerBoxParentData).offset;
    final double leftMost = firstChildOffset.dx;
    final double rightMost =
        (children.last.parentData! as _SegmentedControlContainerBoxParentData).offset.dx +
        children.last.size.width;
    assert(rightMost > leftMost);

    // Ignore the horizontal position and the height of `thumbRect`, and
    // calculates them from `children`.
    return Rect.fromLTRB(
      math.max(thumbRect.left, leftMost - _kThumbInsets.left),
      firstChildOffset.dy - _kThumbInsets.top,
      math.min(thumbRect.right, rightMost + _kThumbInsets.right),
      firstChildOffset.dy + children.first.size.height + _kThumbInsets.bottom,
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final List<RenderBox> children = getChildrenAsList();

    // Children contains both segment and separator and the order is segment ->
    // separator -> segment. So to paint separators, index should start from 1 and
    // the step should be 2.
    for (int index = 1; index < childCount; index += 2) {
      _paintSeparator(context, offset, children[index]);
    }

    final int? highlightedChildIndex = highlightedIndex;
    // Paint thumb if there's a highlighted segment.
    if (highlightedChildIndex != null) {
      final RenderBox selectedChild = children[highlightedChildIndex * 2];

      final _SegmentedControlContainerBoxParentData childParentData =
          selectedChild.parentData! as _SegmentedControlContainerBoxParentData;
      final Rect newThumbRect = _kThumbInsets.inflateRect(
        childParentData.offset & selectedChild.size,
      );

      // Update thumb animation's tween, in case the end rect changed (e.g., a
      // new segment is added during the animation).
      if (state.thumbController.isAnimating) {
        final Animatable<Rect?>? thumbTween = state.thumbAnimatable;
        if (thumbTween == null) {
          // This is the first frame of the animation.
          final Rect startingRect =
              moveThumbRectInBound(currentThumbRect, children) ?? newThumbRect;
          state.thumbAnimatable = RectTween(begin: startingRect, end: newThumbRect);
        } else if (newThumbRect != thumbTween.transform(1)) {
          // The thumbTween of the running sliding animation needs updating,
          // without restarting the animation.
          final Rect startingRect =
              moveThumbRectInBound(currentThumbRect, children) ?? newThumbRect;
          state.thumbAnimatable = RectTween(
            begin: startingRect,
            end: newThumbRect,
          ).chain(CurveTween(curve: Interval(state.thumbController.value, 1)));
        }
      } else {
        state.thumbAnimatable = null;
      }

      final Rect unscaledThumbRect =
          state.thumbAnimatable?.evaluate(state.thumbController) ?? newThumbRect;
      currentThumbRect = unscaledThumbRect;

      final _SegmentLocation childLocation;
      if (highlightedChildIndex == 0) {
        childLocation = _SegmentLocation.leftmost;
      } else if (highlightedChildIndex == children.length ~/ 2) {
        childLocation = _SegmentLocation.rightmost;
      } else {
        childLocation = _SegmentLocation.inbetween;
      }

      final double delta = switch (childLocation) {
        _SegmentLocation.leftmost => unscaledThumbRect.width - unscaledThumbRect.width * thumbScale,
        _SegmentLocation.rightmost =>
          unscaledThumbRect.width * thumbScale - unscaledThumbRect.width,
        _SegmentLocation.inbetween => 0,
      };

      final Rect thumbRect = Rect.fromCenter(
        center: unscaledThumbRect.center - Offset(delta / 2, 0),
        width: unscaledThumbRect.width * thumbScale,
        height: unscaledThumbRect.height * thumbScale,
      );

      _paintThumb(context, offset, thumbRect);
    } else {
      currentThumbRect = null;
    }

    for (int index = 0; index < children.length; index += 2) {
      // Children contains both segment and separator and the order is segment ->
      // separator -> segment. So to paint separators, index should start from 0 and
      // the step should be 2.
      _paintChild(context, offset, children[index]);
    }
  }

  // Paint the separator to the right of the given child.
  final Paint separatorPaint = Paint();
  void _paintSeparator(PaintingContext context, Offset offset, RenderBox child) {
    final _SegmentedControlContainerBoxParentData childParentData =
        child.parentData! as _SegmentedControlContainerBoxParentData;
    context.paintChild(child, offset + childParentData.offset);
  }

  void _paintChild(PaintingContext context, Offset offset, RenderBox child) {
    final _SegmentedControlContainerBoxParentData childParentData =
        child.parentData! as _SegmentedControlContainerBoxParentData;
    context.paintChild(child, childParentData.offset + offset);
  }

  void _paintThumb(PaintingContext context, Offset offset, Rect thumbRect) {
    // Colors extracted from https://developer.apple.com/design/resources/.
    const List<BoxShadow> thumbShadow = <BoxShadow>[
      BoxShadow(color: Color(0x1F000000), offset: Offset(0, 3), blurRadius: 8),
      BoxShadow(color: Color(0x0A000000), offset: Offset(0, 3), blurRadius: 1),
    ];

    final RRect thumbRRect = RRect.fromRectAndRadius(thumbRect.shift(offset), _kThumbRadius);

    for (final BoxShadow shadow in thumbShadow) {
      context.canvas.drawRRect(thumbRRect.shift(shadow.offset), shadow.toPaint());
    }

    context.canvas.drawRRect(thumbRRect.inflate(0.5), Paint()..color = const Color(0x0A000000));

    context.canvas.drawRRect(thumbRRect, Paint()..color = thumbColor);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    RenderBox? child = lastChild;
    while (child != null) {
      final _SegmentedControlContainerBoxParentData childParentData =
          child.parentData! as _SegmentedControlContainerBoxParentData;
      if ((childParentData.offset & child.size).contains(position)) {
        return result.addWithPaintOffset(
          offset: childParentData.offset,
          position: position,
          hitTest: (BoxHitTestResult result, Offset localOffset) {
            assert(localOffset == position - childParentData.offset);
            return child!.hitTest(result, position: localOffset);
          },
        );
      }
      child = childParentData.previousSibling;
    }
    return false;
  }
}
