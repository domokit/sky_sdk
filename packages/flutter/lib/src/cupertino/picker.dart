// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'route.dart';
/// @docImport 'text_theme.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'theme.dart';

// Eyeballed values comparing with a native picker to produce the right
// curvatures and densities.
const double _kDefaultDiameterRatio = 1.07;
const double _kDefaultPerspective = 0.003;
const double _kSqueeze = 1.45;

// Opacity fraction value that dims the wheel above and below the "magnifier"
// lens.
const double _kOverAndUnderCenterOpacity = 0.447;

// The duration and curve of the tap-to-scroll gesture's animation when a picker
// item is tapped.
//
// Eyeballed from an iPhone 15 Pro simulator running iOS 17.5.
const Duration _kCupertinoPickerTapToScrollDuration = Duration(milliseconds: 300);
const Curve _kCupertinoPickerTapToScrollCurve = Curves.easeInOut;

/// An iOS-styled picker.
///
/// Displays its children widgets on a wheel for selection and
/// calls back when the currently selected item changes.
///
/// By default, the first child in `children` will be the initially selected child.
/// The index of a different child can be specified in [scrollController], to make
/// that child the initially selected child.
///
/// Can be used with [showCupertinoModalPopup] to display the picker modally at the
/// bottom of the screen. When calling [showCupertinoModalPopup], be sure to set
/// `semanticsDismissible` to true to enable dismissing the modal via semantics.
///
/// Sizes itself to its parent. All children are sized to the same size based
/// on [itemExtent].
///
/// By default, descendent texts are shown with [CupertinoTextThemeData.pickerTextStyle].
///
/// {@tool dartpad}
/// This example shows a [CupertinoPicker] that displays a list of fruits on a wheel for
/// selection.
///
/// ** See code in examples/api/lib/cupertino/picker/cupertino_picker.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ListWheelScrollView], the generic widget backing this picker without
///    the iOS design specific chrome.
///  * <https://developer.apple.com/design/human-interface-guidelines/pickers/>
class CupertinoPicker extends StatefulWidget {
  /// Creates a picker from a concrete list of children.
  ///
  /// The [itemExtent] must be greater than zero.
  ///
  /// The [backgroundColor] defaults to null, which disables background painting entirely.
  /// (i.e. the picker is going to have a completely transparent background), to match
  /// the native UIPicker and UIDatePicker. Also, if it has transparency, no gradient
  /// effect will be rendered.
  ///
  /// The [scrollController] argument can be used to specify a custom
  /// [FixedExtentScrollController] for programmatically reading or changing
  /// the current picker index or for selecting an initial index value.
  ///
  /// The [looping] argument decides whether the child list loops and can be
  /// scrolled infinitely. If set to true, scrolling past the end of the list
  /// will loop the list back to the beginning. If set to false, the list will
  /// stop scrolling when you reach the end or the beginning.
  CupertinoPicker({
    super.key,
    this.diameterRatio = _kDefaultDiameterRatio,
    this.backgroundColor,
    this.offAxisFraction = 0.0,
    this.useMagnifier = false,
    this.magnification = 1.0,
    this.scrollController,
    this.squeeze = _kSqueeze,
    required this.itemExtent,
    required this.onSelectedItemChanged,
    required List<Widget> children,
    this.selectionOverlay = const CupertinoPickerDefaultSelectionOverlay(),
    bool looping = false,
  }) : assert(diameterRatio > 0.0, RenderListWheelViewport.diameterRatioZeroMessage),
       assert(magnification > 0),
       assert(itemExtent > 0),
       assert(squeeze > 0),
       childDelegate =
           looping
               ? ListWheelChildLoopingListDelegate(children: children)
               : ListWheelChildListDelegate(children: children);

  /// Creates a picker from an [IndexedWidgetBuilder] callback where the builder
  /// is dynamically invoked during layout.
  ///
  /// A child is lazily created when it starts becoming visible in the viewport.
  /// All of the children provided by the builder are cached and reused, so
  /// normally the builder is only called once for each index (except when
  /// rebuilding - the cache is cleared).
  ///
  /// The [childCount] argument reflects the number of children that will be
  /// provided by the [itemBuilder].
  /// {@macro flutter.widgets.ListWheelChildBuilderDelegate.childCount}
  ///
  /// The [itemExtent] argument must be positive.
  ///
  /// The [backgroundColor] defaults to null, which disables background painting entirely.
  /// (i.e. the picker is going to have a completely transparent background), to match
  /// the native UIPicker and UIDatePicker.
  CupertinoPicker.builder({
    super.key,
    this.diameterRatio = _kDefaultDiameterRatio,
    this.backgroundColor,
    this.offAxisFraction = 0.0,
    this.useMagnifier = false,
    this.magnification = 1.0,
    this.scrollController,
    this.squeeze = _kSqueeze,
    required this.itemExtent,
    required this.onSelectedItemChanged,
    required NullableIndexedWidgetBuilder itemBuilder,
    int? childCount,
    this.selectionOverlay = const CupertinoPickerDefaultSelectionOverlay(),
  }) : assert(diameterRatio > 0.0, RenderListWheelViewport.diameterRatioZeroMessage),
       assert(magnification > 0),
       assert(itemExtent > 0),
       assert(squeeze > 0),
       childDelegate = ListWheelChildBuilderDelegate(builder: itemBuilder, childCount: childCount);

  /// Relative ratio between this picker's height and the simulated cylinder's diameter.
  ///
  /// Smaller values creates more pronounced curvatures in the scrollable wheel.
  ///
  /// For more details, see [ListWheelScrollView.diameterRatio].
  ///
  /// Defaults to 1.1 to visually mimic iOS.
  final double diameterRatio;

  /// Background color behind the children.
  ///
  /// Defaults to null, which disables background painting entirely.
  /// (i.e. the picker is going to have a completely transparent background), to match
  /// the native UIPicker and UIDatePicker.
  ///
  /// Any alpha value less 255 (fully opaque) will cause the removal of the
  /// wheel list edge fade gradient from rendering of the widget.
  final Color? backgroundColor;

  /// {@macro flutter.rendering.RenderListWheelViewport.offAxisFraction}
  final double offAxisFraction;

  /// {@macro flutter.rendering.RenderListWheelViewport.useMagnifier}
  final bool useMagnifier;

  /// {@macro flutter.rendering.RenderListWheelViewport.magnification}
  final double magnification;

  /// A [FixedExtentScrollController] to read and control the current item, and
  /// to set the initial item.
  ///
  /// If null, an implicit one will be created internally.
  final FixedExtentScrollController? scrollController;

  /// {@template flutter.cupertino.picker.itemExtent}
  /// The uniform height of all children.
  ///
  /// All children will be given the [BoxConstraints] to match this exact
  /// height. Must be a positive value.
  /// {@endtemplate}
  final double itemExtent;

  /// {@macro flutter.rendering.RenderListWheelViewport.squeeze}
  ///
  /// Defaults to `1.45` to visually mimic iOS.
  final double squeeze;

  /// An option callback when the currently centered item changes.
  ///
  /// Value changes when the item closest to the center changes.
  ///
  /// This can be called during scrolls and during ballistic flings. To get the
  /// value only when the scrolling settles, use a [NotificationListener],
  /// listen for [ScrollEndNotification] and read its [FixedExtentMetrics].
  final ValueChanged<int>? onSelectedItemChanged;

  /// A delegate that lazily instantiates children.
  final ListWheelChildDelegate childDelegate;

  /// A widget overlaid on the picker to highlight the currently selected entry.
  ///
  /// The [selectionOverlay] widget drawn above the [CupertinoPicker]'s picker
  /// wheel.
  /// It is vertically centered in the picker and is constrained to have the
  /// same height as the center row.
  ///
  /// If unspecified, it defaults to a [CupertinoPickerDefaultSelectionOverlay]
  /// which is a gray rounded rectangle overlay in iOS 14 style.
  /// This property can be set to null to remove the overlay.
  final Widget? selectionOverlay;

  @override
  State<StatefulWidget> createState() => _CupertinoPickerState();
}

class _CupertinoPickerState extends State<CupertinoPicker> {
  int? _lastHapticIndex;
  FixedExtentScrollController? _controller;

  @override
  void initState() {
    super.initState();
    if (widget.scrollController == null) {
      _controller = FixedExtentScrollController();
    }
  }

  @override
  void didUpdateWidget(CupertinoPicker oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.scrollController != null && oldWidget.scrollController == null) {
      _controller?.dispose();
      _controller = null;
    } else if (widget.scrollController == null && oldWidget.scrollController != null) {
      assert(_controller == null);
      _controller = FixedExtentScrollController();
    }
  }

  @override
  void dispose() {
    _controller?.dispose();
    super.dispose();
  }

  void _handleSelectedItemChanged(int index) {
    // Only the haptic engine hardware on iOS devices would produce the
    // intended effects.
    final bool hasSuitableHapticHardware;
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
        hasSuitableHapticHardware = true;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        hasSuitableHapticHardware = false;
    }
    if (hasSuitableHapticHardware && index != _lastHapticIndex) {
      _lastHapticIndex = index;
      HapticFeedback.selectionClick();
    }

    widget.onSelectedItemChanged?.call(index);
  }

  void _handleChildTap(int index, FixedExtentScrollController controller) {
    controller.animateToItem(
      index,
      duration: _kCupertinoPickerTapToScrollDuration,
      curve: _kCupertinoPickerTapToScrollCurve,
    );
  }

  /// Draws the selectionOverlay.
  Widget _buildSelectionOverlay(Widget selectionOverlay) {
    final double height = widget.itemExtent * widget.magnification;

    return IgnorePointer(
      child: Center(
        child: ConstrainedBox(
          constraints: BoxConstraints.expand(height: height),
          child: selectionOverlay,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final TextStyle textStyle = CupertinoTheme.of(context).textTheme.pickerTextStyle;
    final Color? resolvedBackgroundColor = CupertinoDynamicColor.maybeResolve(
      widget.backgroundColor,
      context,
    );

    assert(RenderListWheelViewport.defaultPerspective == _kDefaultPerspective);
    final FixedExtentScrollController controller = widget.scrollController ?? _controller!;
    final Widget result = DefaultTextStyle(
      style: textStyle.copyWith(
        color: CupertinoDynamicColor.maybeResolve(textStyle.color, context),
      ),
      child: Stack(
        children: <Widget>[
          Positioned.fill(
            child: _CupertinoPickerSemantics(
              scrollController: controller,
              child: ListWheelScrollView.useDelegate(
                controller: controller,
                physics: const FixedExtentScrollPhysics(),
                diameterRatio: widget.diameterRatio,
                offAxisFraction: widget.offAxisFraction,
                useMagnifier: widget.useMagnifier,
                magnification: widget.magnification,
                overAndUnderCenterOpacity: _kOverAndUnderCenterOpacity,
                itemExtent: widget.itemExtent,
                squeeze: widget.squeeze,
                onSelectedItemChanged: _handleSelectedItemChanged,
                dragStartBehavior: DragStartBehavior.down,
                childDelegate: _CupertinoPickerListWheelChildDelegateWrapper(
                  widget.childDelegate,
                  onTappedChild: (int index) => _handleChildTap(index, controller),
                ),
              ),
            ),
          ),
          if (widget.selectionOverlay != null) _buildSelectionOverlay(widget.selectionOverlay!),
        ],
      ),
    );

    return DecoratedBox(decoration: BoxDecoration(color: resolvedBackgroundColor), child: result);
  }
}

/// A default selection overlay for [CupertinoPicker]s.
///
/// It draws a gray rounded rectangle to match the picker visuals introduced in
/// iOS 14.
///
/// This widget is typically only used in [CupertinoPicker.selectionOverlay].
/// In an iOS 14 multi-column picker, the selection overlay is a single rounded
/// rectangle that spans the entire multi-column picker.
/// To achieve the same effect using [CupertinoPickerDefaultSelectionOverlay],
/// the additional margin and corner radii on the left or the right side can be
/// disabled by turning off [capStartEdge] and [capEndEdge], so this selection
/// overlay visually connects with selection overlays of adjoining
/// [CupertinoPicker]s (i.e., other "column"s).
///
/// See also:
///
///  * [CupertinoPicker], which uses this widget as its default [CupertinoPicker.selectionOverlay].
class CupertinoPickerDefaultSelectionOverlay extends StatelessWidget {
  /// Creates an iOS 14 style selection overlay that highlights the magnified
  /// area (or the currently selected item, depending on how you described it
  /// elsewhere) of a [CupertinoPicker].
  ///
  /// The [background] argument default value is
  /// [CupertinoColors.tertiarySystemFill].
  ///
  /// The [capStartEdge] and [capEndEdge] arguments decide whether to add a
  /// default margin and use rounded corners on the left and right side of the
  /// rectangular overlay, and they both default to true.
  const CupertinoPickerDefaultSelectionOverlay({
    super.key,
    this.background = CupertinoColors.tertiarySystemFill,
    this.capStartEdge = true,
    this.capEndEdge = true,
  });

  /// Whether to use the default use rounded corners and margin on the start side.
  final bool capStartEdge;

  /// Whether to use the default use rounded corners and margin on the end side.
  final bool capEndEdge;

  /// The color to fill in the background of the [CupertinoPickerDefaultSelectionOverlay].
  /// It Support for use [CupertinoDynamicColor].
  ///
  /// Typically this should not be set to a fully opaque color, as the currently
  /// selected item of the underlying [CupertinoPicker] should remain visible.
  /// Defaults to [CupertinoColors.tertiarySystemFill].
  final Color background;

  /// Default margin of the 'SelectionOverlay'.
  static const double _defaultSelectionOverlayHorizontalMargin = 9;

  /// Default radius of the 'SelectionOverlay'.
  static const double _defaultSelectionOverlayRadius = 8;

  @override
  Widget build(BuildContext context) {
    const Radius radius = Radius.circular(_defaultSelectionOverlayRadius);

    return Container(
      margin: EdgeInsetsDirectional.only(
        start: capStartEdge ? _defaultSelectionOverlayHorizontalMargin : 0,
        end: capEndEdge ? _defaultSelectionOverlayHorizontalMargin : 0,
      ),
      decoration: BoxDecoration(
        borderRadius: BorderRadiusDirectional.horizontal(
          start: capStartEdge ? radius : Radius.zero,
          end: capEndEdge ? radius : Radius.zero,
        ),
        color: CupertinoDynamicColor.resolve(background, context),
      ),
    );
  }
}

// Turns the scroll semantics of the ListView into a single adjustable semantics
// node. This is done by removing all of the child semantics of the scroll
// wheel and using the scroll indexes to look up the current, previous, and
// next semantic label. This label is then turned into the value of a new
// adjustable semantic node, with adjustment callbacks wired to move the
// scroll controller.
class _CupertinoPickerSemantics extends SingleChildRenderObjectWidget {
  const _CupertinoPickerSemantics({super.child, required this.scrollController});

  final FixedExtentScrollController scrollController;

  @override
  RenderObject createRenderObject(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    return _RenderCupertinoPickerSemantics(scrollController, Directionality.of(context));
  }

  @override
  void updateRenderObject(
    BuildContext context,
    covariant _RenderCupertinoPickerSemantics renderObject,
  ) {
    assert(debugCheckHasDirectionality(context));
    renderObject
      ..textDirection = Directionality.of(context)
      ..controller = scrollController;
  }
}

class _RenderCupertinoPickerSemantics extends RenderProxyBox {
  _RenderCupertinoPickerSemantics(FixedExtentScrollController controller, this._textDirection) {
    _updateController(null, controller);
  }

  FixedExtentScrollController get controller => _controller;
  late FixedExtentScrollController _controller;
  set controller(FixedExtentScrollController value) => _updateController(_controller, value);

  // This method exists to allow controller to be non-null. It is only called with a null oldValue from constructor.
  void _updateController(FixedExtentScrollController? oldValue, FixedExtentScrollController value) {
    if (value == oldValue) {
      return;
    }
    if (oldValue != null) {
      oldValue.removeListener(_handleScrollUpdate);
    } else {
      _currentIndex = value.initialItem;
    }
    value.addListener(_handleScrollUpdate);
    _controller = value;
  }

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsSemanticsUpdate();
  }

  int _currentIndex = 0;

  void _handleIncrease() {
    controller.jumpToItem(_currentIndex + 1);
  }

  void _handleDecrease() {
    controller.jumpToItem(_currentIndex - 1);
  }

  void _handleScrollUpdate() {
    if (controller.selectedItem == _currentIndex) {
      return;
    }
    _currentIndex = controller.selectedItem;
    markNeedsSemanticsUpdate();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = true;
    config.textDirection = textDirection;
  }

  @override
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    if (children.isEmpty) {
      return super.assembleSemanticsNode(node, config, children);
    }
    final SemanticsNode scrollable = children.first;
    final Map<int, SemanticsNode> indexedChildren = <int, SemanticsNode>{};
    scrollable.visitChildren((SemanticsNode child) {
      assert(child.indexInParent != null);
      indexedChildren[child.indexInParent!] = child;
      return true;
    });
    if (indexedChildren[_currentIndex] == null) {
      return node.updateWith(config: config);
    }
    config.value = indexedChildren[_currentIndex]!.label;
    final SemanticsNode? previousChild = indexedChildren[_currentIndex - 1];
    final SemanticsNode? nextChild = indexedChildren[_currentIndex + 1];
    if (nextChild != null) {
      config.increasedValue = nextChild.label;
      config.onIncrease = _handleIncrease;
    }
    if (previousChild != null) {
      config.decreasedValue = previousChild.label;
      config.onDecrease = _handleDecrease;
    }
    node.updateWith(config: config);
  }

  @override
  void dispose() {
    super.dispose();
    controller.removeListener(_handleScrollUpdate);
  }
}

class _CupertinoPickerListWheelChildDelegateWrapper implements ListWheelChildDelegate {
  _CupertinoPickerListWheelChildDelegateWrapper(this._wrapped, {required this.onTappedChild});
  final ListWheelChildDelegate _wrapped;
  final void Function(int index) onTappedChild;

  @override
  Widget? build(BuildContext context, int index) {
    final Widget? child = _wrapped.build(context, index);
    if (child == null) {
      return child;
    }
    return GestureDetector(
      behavior: HitTestBehavior.translucent,
      excludeFromSemantics: true,
      onTap: () => onTappedChild(index),
      child: child,
    );
  }

  @override
  int? get estimatedChildCount => _wrapped.estimatedChildCount;

  @override
  bool shouldRebuild(covariant _CupertinoPickerListWheelChildDelegateWrapper oldDelegate) =>
      _wrapped.shouldRebuild(oldDelegate._wrapped);

  @override
  int trueIndexOf(int index) => _wrapped.trueIndexOf(index);
}
