// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'primary_scroll_controller.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'two_dimensional_scroll_view.dart';
/// @docImport 'viewport.dart';
library;

import 'dart:math' as math;

import 'package:flutter/animation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';
import 'scroll_delegate.dart';
import 'scroll_notification.dart';
import 'scroll_position.dart';

export 'package:flutter/rendering.dart' show AxisDirection;

// Examples can assume:
// late final RenderBox child;
// late final BoxConstraints constraints;
// class RenderSimpleTwoDimensionalViewport extends RenderTwoDimensionalViewport {
//   RenderSimpleTwoDimensionalViewport({
//     required super.horizontalOffset,
//     required super.horizontalAxisDirection,
//     required super.verticalOffset,
//     required super.verticalAxisDirection,
//     required super.delegate,
//     required super.mainAxis,
//     required super.childManager,
//     super.cacheExtent,
//     super.clipBehavior = Clip.hardEdge,
//   });
//   @override
//   void layoutChildSequence() { }
// }

/// Signature for a function that creates a widget for a given [ChildVicinity],
/// e.g., in a [TwoDimensionalScrollView], but may return null.
///
/// Used by [TwoDimensionalChildBuilderDelegate.builder] and other APIs that
/// use lazily-generated widgets where the child count may not be known
/// ahead of time.
///
/// Unlike most builders, this callback can return null, indicating the
/// [ChildVicinity.xIndex] or [ChildVicinity.yIndex] is out of range. Whether
/// and when this is valid depends on the semantics of the builder. For example,
/// [TwoDimensionalChildBuilderDelegate.builder] returns
/// null when one or both of the indices is out of range, where the range is
/// defined by the [TwoDimensionalChildBuilderDelegate.maxXIndex] or
/// [TwoDimensionalChildBuilderDelegate.maxYIndex]; so in that case the
/// vicinity values may determine whether returning null is valid or not.
///
/// See also:
///
///  * [WidgetBuilder], which is similar but only takes a [BuildContext].
///  * [NullableIndexedWidgetBuilder], which is similar but may return null.
///  * [IndexedWidgetBuilder], which is similar but not nullable.
typedef TwoDimensionalIndexedWidgetBuilder =
    Widget? Function(BuildContext context, ChildVicinity vicinity);

/// A widget through which a portion of larger content can be viewed, typically
/// in combination with a [TwoDimensionalScrollable].
///
/// [TwoDimensionalViewport] is the visual workhorse of the two dimensional
/// scrolling machinery. It displays a subset of its children according to its
/// own dimensions and the given [horizontalOffset] an [verticalOffset]. As the
/// offsets vary, different children are visible through the viewport.
///
/// Subclasses must implement [createRenderObject] and [updateRenderObject].
/// Both of these methods require the render object to be a subclass of
/// [RenderTwoDimensionalViewport]. This class will create its own
/// [RenderObjectElement] which already implements the
/// [TwoDimensionalChildManager], which means subclasses should cast the
/// [BuildContext] to provide as the child manager to the
/// [RenderTwoDimensionalViewport].
///
/// {@tool snippet}
/// This is an example of a subclass implementation of [TwoDimensionalViewport],
/// `SimpleTwoDimensionalViewport`. The `RenderSimpleTwoDimensionalViewport` is
/// a subclass of [RenderTwoDimensionalViewport].
///
/// ```dart
/// class SimpleTwoDimensionalViewport extends TwoDimensionalViewport {
///   const SimpleTwoDimensionalViewport({
///     super.key,
///     required super.verticalOffset,
///     required super.verticalAxisDirection,
///     required super.horizontalOffset,
///     required super.horizontalAxisDirection,
///     required super.delegate,
///     required super.mainAxis,
///     super.cacheExtent,
///     super.clipBehavior = Clip.hardEdge,
///   });
///
///   @override
///   RenderSimpleTwoDimensionalViewport createRenderObject(BuildContext context) {
///     return RenderSimpleTwoDimensionalViewport(
///       horizontalOffset: horizontalOffset,
///       horizontalAxisDirection: horizontalAxisDirection,
///       verticalOffset: verticalOffset,
///       verticalAxisDirection: verticalAxisDirection,
///       mainAxis: mainAxis,
///       delegate: delegate,
///       childManager: context as TwoDimensionalChildManager,
///       cacheExtent: cacheExtent,
///       clipBehavior: clipBehavior,
///     );
///   }
///
///   @override
///   void updateRenderObject(BuildContext context, RenderSimpleTwoDimensionalViewport renderObject) {
///     renderObject
///       ..horizontalOffset = horizontalOffset
///       ..horizontalAxisDirection = horizontalAxisDirection
///       ..verticalOffset = verticalOffset
///       ..verticalAxisDirection = verticalAxisDirection
///       ..mainAxis = mainAxis
///       ..delegate = delegate
///       ..cacheExtent = cacheExtent
///       ..clipBehavior = clipBehavior;
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///   * [Viewport], the equivalent of this widget that scrolls in only one
///     dimension.
abstract class TwoDimensionalViewport extends RenderObjectWidget {
  /// Creates a viewport for [RenderBox] objects that extend and scroll in both
  /// horizontal and vertical dimensions.
  ///
  /// The viewport listens to the [horizontalOffset] and [verticalOffset], which
  /// means this widget does not need to be rebuilt when the offsets change.
  const TwoDimensionalViewport({
    super.key,
    required this.verticalOffset,
    required this.verticalAxisDirection,
    required this.horizontalOffset,
    required this.horizontalAxisDirection,
    required this.delegate,
    required this.mainAxis,
    this.cacheExtent,
    this.clipBehavior = Clip.hardEdge,
  }) : assert(
         verticalAxisDirection == AxisDirection.down || verticalAxisDirection == AxisDirection.up,
         'TwoDimensionalViewport.verticalAxisDirection is not Axis.vertical.',
       ),
       assert(
         horizontalAxisDirection == AxisDirection.left ||
             horizontalAxisDirection == AxisDirection.right,
         'TwoDimensionalViewport.horizontalAxisDirection is not Axis.horizontal.',
       );

  /// Which part of the content inside the viewport should be visible in the
  /// vertical axis.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport vertically, this value changes, which changes the
  /// content that is displayed.
  ///
  /// Typically a [ScrollPosition].
  final ViewportOffset verticalOffset;

  /// The direction in which the [verticalOffset]'s [ViewportOffset.pixels]
  /// increases.
  ///
  /// For example, if the axis direction is [AxisDirection.down], a scroll
  /// offset of zero is at the top of the viewport and increases towards the
  /// bottom of the viewport.
  ///
  /// Must be either [AxisDirection.down] or [AxisDirection.up] in correlation
  /// with an [Axis.vertical].
  final AxisDirection verticalAxisDirection;

  /// Which part of the content inside the viewport should be visible in the
  /// horizontal axis.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport horizontally, this value changes, which changes the
  /// content that is displayed.
  ///
  /// Typically a [ScrollPosition].
  final ViewportOffset horizontalOffset;

  /// The direction in which the [horizontalOffset]'s [ViewportOffset.pixels]
  /// increases.
  ///
  /// For example, if the axis direction is [AxisDirection.right], a scroll
  /// offset of zero is at the left of the viewport and increases towards the
  /// right of the viewport.
  ///
  /// Must be either [AxisDirection.left] or [AxisDirection.right] in correlation
  /// with an [Axis.horizontal].
  final AxisDirection horizontalAxisDirection;

  /// The main axis of the two.
  ///
  /// Used to determine the paint order of the children of the viewport. When
  /// the main axis is [Axis.vertical], children will be painted in row major
  /// order, according to their associated [ChildVicinity]. When the main axis
  /// is [Axis.horizontal], the children will be painted in column major order.
  final Axis mainAxis;

  /// {@macro flutter.rendering.RenderViewportBase.cacheExtent}
  final double? cacheExtent;

  /// {@macro flutter.material.Material.clipBehavior}
  final Clip clipBehavior;

  /// A delegate that provides the children for the [TwoDimensionalViewport].
  final TwoDimensionalChildDelegate delegate;

  @override
  RenderObjectElement createElement() => _TwoDimensionalViewportElement(this);

  @override
  RenderTwoDimensionalViewport createRenderObject(BuildContext context);

  @override
  void updateRenderObject(BuildContext context, RenderTwoDimensionalViewport renderObject);
}

class _TwoDimensionalViewportElement extends RenderObjectElement
    with NotifiableElementMixin, ViewportElementMixin
    implements TwoDimensionalChildManager {
  _TwoDimensionalViewportElement(super.widget);

  @override
  RenderTwoDimensionalViewport get renderObject =>
      super.renderObject as RenderTwoDimensionalViewport;

  // Contains all children, including those that are keyed.
  Map<ChildVicinity, Element> _vicinityToChild = <ChildVicinity, Element>{};
  Map<Key, Element> _keyToChild = <Key, Element>{};
  // Used between _startLayout() & _endLayout() to compute the new values for
  // _vicinityToChild and _keyToChild.
  Map<ChildVicinity, Element>? _newVicinityToChild;
  Map<Key, Element>? _newKeyToChild;

  @override
  void performRebuild() {
    super.performRebuild();
    // Children list is updated during layout since we only know during layout
    // which children will be visible.
    renderObject.markNeedsLayout(withDelegateRebuild: true);
  }

  @override
  void forgetChild(Element child) {
    assert(!_debugIsDoingLayout);
    super.forgetChild(child);
    _vicinityToChild.remove(child.slot);
    if (child.widget.key != null) {
      _keyToChild.remove(child.widget.key);
    }
  }

  @override
  void insertRenderObjectChild(RenderBox child, ChildVicinity slot) {
    renderObject._insertChild(child, slot);
  }

  @override
  void moveRenderObjectChild(RenderBox child, ChildVicinity oldSlot, ChildVicinity newSlot) {
    renderObject._moveChild(child, from: oldSlot, to: newSlot);
  }

  @override
  void removeRenderObjectChild(RenderBox child, ChildVicinity slot) {
    renderObject._removeChild(child, slot);
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    _vicinityToChild.values.forEach(visitor);
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<Element> children = _vicinityToChild.values.toList()..sort(_compareChildren);
    return <DiagnosticsNode>[
      for (final Element child in children) child.toDiagnosticsNode(name: child.slot.toString()),
    ];
  }

  static int _compareChildren(Element a, Element b) {
    final ChildVicinity aSlot = a.slot! as ChildVicinity;
    final ChildVicinity bSlot = b.slot! as ChildVicinity;
    return aSlot.compareTo(bSlot);
  }

  // ---- ChildManager implementation ----

  bool get _debugIsDoingLayout => _newKeyToChild != null && _newVicinityToChild != null;

  @override
  void _startLayout() {
    assert(!_debugIsDoingLayout);
    _newVicinityToChild = <ChildVicinity, Element>{};
    _newKeyToChild = <Key, Element>{};
  }

  @override
  void _buildChild(ChildVicinity vicinity) {
    assert(_debugIsDoingLayout);
    owner!.buildScope(this, () {
      final Widget? newWidget = (widget as TwoDimensionalViewport).delegate.build(this, vicinity);
      if (newWidget == null) {
        return;
      }
      final Element? oldElement = _retrieveOldElement(newWidget, vicinity);
      final Element? newChild = updateChild(oldElement, newWidget, vicinity);
      assert(newChild != null);
      // Ensure we are not overwriting an existing child.
      assert(_newVicinityToChild![vicinity] == null);
      _newVicinityToChild![vicinity] = newChild!;
      if (newWidget.key != null) {
        // Ensure we are not overwriting an existing key
        assert(_newKeyToChild![newWidget.key!] == null);
        _newKeyToChild![newWidget.key!] = newChild;
      }
    });
  }

  Element? _retrieveOldElement(Widget newWidget, ChildVicinity vicinity) {
    if (newWidget.key != null) {
      final Element? result = _keyToChild.remove(newWidget.key);
      if (result != null) {
        _vicinityToChild.remove(result.slot);
      }
      return result;
    }
    final Element? potentialOldElement = _vicinityToChild[vicinity];
    if (potentialOldElement != null && potentialOldElement.widget.key == null) {
      return _vicinityToChild.remove(vicinity);
    }
    return null;
  }

  @override
  void _reuseChild(ChildVicinity vicinity) {
    assert(_debugIsDoingLayout);
    final Element? elementToReuse = _vicinityToChild.remove(vicinity);
    assert(
      elementToReuse != null,
      'Expected to re-use an element at $vicinity, but none was found.',
    );
    _newVicinityToChild![vicinity] = elementToReuse!;
    if (elementToReuse.widget.key != null) {
      assert(_keyToChild.containsKey(elementToReuse.widget.key));
      assert(_keyToChild[elementToReuse.widget.key] == elementToReuse);
      _newKeyToChild![elementToReuse.widget.key!] = _keyToChild.remove(elementToReuse.widget.key)!;
    }
  }

  @override
  void _endLayout() {
    assert(_debugIsDoingLayout);

    // Unmount all elements that have not been reused in the layout cycle.
    for (final Element element in _vicinityToChild.values) {
      if (element.widget.key == null) {
        // If it has a key, we handle it below.
        updateChild(element, null, null);
      } else {
        assert(_keyToChild.containsValue(element));
      }
    }
    for (final Element element in _keyToChild.values) {
      assert(element.widget.key != null);
      updateChild(element, null, null);
    }

    _vicinityToChild = _newVicinityToChild!;
    _keyToChild = _newKeyToChild!;
    _newVicinityToChild = null;
    _newKeyToChild = null;
    assert(!_debugIsDoingLayout);
  }
}

/// Parent data structure used by [RenderTwoDimensionalViewport].
///
/// The parent data primarily describes where a child is in the viewport. The
/// [layoutOffset] must be set by subclasses of [RenderTwoDimensionalViewport],
/// during [RenderTwoDimensionalViewport.layoutChildSequence] which represents
/// the position of the child in the viewport.
///
/// The [paintOffset] is computed by [RenderTwoDimensionalViewport] after
/// [RenderTwoDimensionalViewport.layoutChildSequence]. If subclasses of
/// RenderTwoDimensionalViewport override the paint method, the [paintOffset]
/// should be used to position the child in the viewport in order to account for
/// a reversed [AxisDirection] in one or both dimensions.
class TwoDimensionalViewportParentData extends ParentData with KeepAliveParentDataMixin {
  /// The offset at which to paint the child in the parent's coordinate system.
  ///
  /// This [Offset] represents the top left corner of the child of the
  /// [TwoDimensionalViewport].
  ///
  /// This value must be set by implementors during
  /// [RenderTwoDimensionalViewport.layoutChildSequence]. After the method is
  /// complete, the [RenderTwoDimensionalViewport] will compute the
  /// [paintOffset] based on this value to account for the [AxisDirection].
  Offset? layoutOffset;

  /// The logical positioning of children in two dimensions.
  ///
  /// While children may not be strictly laid out in rows and columns, the
  /// relative positioning determines traversal of
  /// children in row or column major format.
  ///
  /// This is set in the [RenderTwoDimensionalViewport.buildOrObtainChildFor].
  ChildVicinity vicinity = ChildVicinity.invalid;

  /// Whether or not the child is actually visible within the viewport.
  ///
  /// For example, if a child is contained within the
  /// [RenderTwoDimensionalViewport.cacheExtent] and out of view.
  ///
  /// This is used during [RenderTwoDimensionalViewport.paint] in order to skip
  /// painting children that cannot be seen.
  bool get isVisible {
    assert(() {
      if (_paintExtent == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('The paint extent of the child has not been determined yet.'),
          ErrorDescription(
            'The paint extent, and therefore the visibility, of a child of a '
            'RenderTwoDimensionalViewport is computed after '
            'RenderTwoDimensionalViewport.layoutChildSequence.',
          ),
        ]);
      }
      return true;
    }());
    return _paintExtent != Size.zero || _paintExtent!.height != 0.0 || _paintExtent!.width != 0.0;
  }

  /// Represents the extent in both dimensions of the child that is actually
  /// visible.
  ///
  /// For example, if a child [RenderBox] had a height of 100 pixels, and a
  /// width of 100 pixels, but was scrolled to positions such that only 50
  /// pixels of both width and height were visible, the paintExtent would be
  /// represented as `Size(50.0, 50.0)`.
  ///
  /// This is set in [RenderTwoDimensionalViewport.updateChildPaintData].
  Size? _paintExtent;

  /// The previous sibling in the parent's child list according to the traversal
  /// order specified by [RenderTwoDimensionalViewport.mainAxis].
  RenderBox? _previousSibling;

  /// The next sibling in the parent's child list according to the traversal
  /// order specified by [RenderTwoDimensionalViewport.mainAxis].
  RenderBox? _nextSibling;

  /// The position of the child relative to the bounds and [AxisDirection] of
  /// the viewport.
  ///
  /// This is the distance from the top left visible corner of the parent to the
  /// top left visible corner of the child. When the [AxisDirection]s are
  /// [AxisDirection.down] or [AxisDirection.right], this value is the same as
  /// the [layoutOffset]. This value deviates when scrolling in the reverse
  /// directions of [AxisDirection.up] and [AxisDirection.left] to reposition
  /// the children correctly.
  ///
  /// This is set in [RenderTwoDimensionalViewport.updateChildPaintData], after
  /// [RenderTwoDimensionalViewport.layoutChildSequence].
  ///
  /// If overriding [RenderTwoDimensionalViewport.paint], use this value to
  /// position the children instead of [layoutOffset].
  Offset? paintOffset;

  @override
  bool get keptAlive => keepAlive && !isVisible;

  @override
  String toString() {
    return 'vicinity=$vicinity; '
        'layoutOffset=$layoutOffset; '
        'paintOffset=$paintOffset; '
        '${_paintExtent == null ? 'not visible; ' : '${!isVisible ? 'not ' : ''}visible - paintExtent=$_paintExtent; '}'
        '${keepAlive ? "keepAlive; " : ""}';
  }
}

/// A base class for viewing render objects that scroll in two dimensions.
///
/// The viewport listens to two [ViewportOffset]s, which determines the
/// visible content.
///
/// Subclasses must implement [layoutChildSequence], calling on
/// [buildOrObtainChildFor] to manage the children of the viewport.
///
/// Subclasses should not override [performLayout], as it handles housekeeping
/// on either side of the call to [layoutChildSequence].
abstract class RenderTwoDimensionalViewport extends RenderBox implements RenderAbstractViewport {
  /// Initializes fields for subclasses.
  ///
  /// The [cacheExtent], if null, defaults to
  /// [RenderAbstractViewport.defaultCacheExtent].
  RenderTwoDimensionalViewport({
    required ViewportOffset horizontalOffset,
    required AxisDirection horizontalAxisDirection,
    required ViewportOffset verticalOffset,
    required AxisDirection verticalAxisDirection,
    required TwoDimensionalChildDelegate delegate,
    required Axis mainAxis,
    required TwoDimensionalChildManager childManager,
    double? cacheExtent,
    Clip clipBehavior = Clip.hardEdge,
  }) : assert(
         verticalAxisDirection == AxisDirection.down || verticalAxisDirection == AxisDirection.up,
         'TwoDimensionalViewport.verticalAxisDirection is not Axis.vertical.',
       ),
       assert(
         horizontalAxisDirection == AxisDirection.left ||
             horizontalAxisDirection == AxisDirection.right,
         'TwoDimensionalViewport.horizontalAxisDirection is not Axis.horizontal.',
       ),
       _childManager = childManager,
       _horizontalOffset = horizontalOffset,
       _horizontalAxisDirection = horizontalAxisDirection,
       _verticalOffset = verticalOffset,
       _verticalAxisDirection = verticalAxisDirection,
       _delegate = delegate,
       _mainAxis = mainAxis,
       _cacheExtent = cacheExtent ?? RenderAbstractViewport.defaultCacheExtent,
       _clipBehavior = clipBehavior {
    assert(() {
      _debugDanglingKeepAlives = <RenderBox>[];
      return true;
    }());
  }

  /// Which part of the content inside the viewport should be visible in the
  /// horizontal axis.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport horizontally, this value changes, which changes the
  /// content that is displayed.
  ///
  /// Typically a [ScrollPosition].
  ViewportOffset get horizontalOffset => _horizontalOffset;
  ViewportOffset _horizontalOffset;
  set horizontalOffset(ViewportOffset value) {
    if (_horizontalOffset == value) {
      return;
    }
    if (attached) {
      _horizontalOffset.removeListener(markNeedsLayout);
    }
    _horizontalOffset = value;
    if (attached) {
      _horizontalOffset.addListener(markNeedsLayout);
    }
    markNeedsLayout();
  }

  /// The direction in which the [horizontalOffset] increases.
  ///
  /// For example, if the axis direction is [AxisDirection.right], a scroll
  /// offset of zero is at the left of the viewport and increases towards the
  /// right of the viewport.
  AxisDirection get horizontalAxisDirection => _horizontalAxisDirection;
  AxisDirection _horizontalAxisDirection;
  set horizontalAxisDirection(AxisDirection value) {
    if (_horizontalAxisDirection == value) {
      return;
    }
    _horizontalAxisDirection = value;
    markNeedsLayout();
  }

  /// Which part of the content inside the viewport should be visible in the
  /// vertical axis.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport vertically, this value changes, which changes the
  /// content that is displayed.
  ///
  /// Typically a [ScrollPosition].
  ViewportOffset get verticalOffset => _verticalOffset;
  ViewportOffset _verticalOffset;
  set verticalOffset(ViewportOffset value) {
    if (_verticalOffset == value) {
      return;
    }
    if (attached) {
      _verticalOffset.removeListener(markNeedsLayout);
    }
    _verticalOffset = value;
    if (attached) {
      _verticalOffset.addListener(markNeedsLayout);
    }
    markNeedsLayout();
  }

  /// The direction in which the [verticalOffset] increases.
  ///
  /// For example, if the axis direction is [AxisDirection.down], a scroll
  /// offset of zero is at the top the viewport and increases towards the
  /// bottom of the viewport.
  AxisDirection get verticalAxisDirection => _verticalAxisDirection;
  AxisDirection _verticalAxisDirection;
  set verticalAxisDirection(AxisDirection value) {
    if (_verticalAxisDirection == value) {
      return;
    }
    _verticalAxisDirection = value;
    markNeedsLayout();
  }

  /// Supplies children for layout in the viewport.
  TwoDimensionalChildDelegate get delegate => _delegate;
  TwoDimensionalChildDelegate _delegate;
  set delegate(covariant TwoDimensionalChildDelegate value) {
    if (_delegate == value) {
      return;
    }
    if (attached) {
      _delegate.removeListener(_handleDelegateNotification);
    }
    final TwoDimensionalChildDelegate oldDelegate = _delegate;
    _delegate = value;
    if (attached) {
      _delegate.addListener(_handleDelegateNotification);
    }
    if (_delegate.runtimeType != oldDelegate.runtimeType || _delegate.shouldRebuild(oldDelegate)) {
      _handleDelegateNotification();
    }
  }

  /// The major axis of the two dimensions.
  ///
  /// This is can be used by subclasses to determine paint order,
  /// visitor patterns like row and column major ordering, or hit test
  /// precedence.
  ///
  /// See also:
  ///
  ///  * [TwoDimensionalScrollView], which assigns the [PrimaryScrollController]
  ///    to the [TwoDimensionalScrollView.mainAxis] and shares this value.
  Axis get mainAxis => _mainAxis;
  Axis _mainAxis;
  set mainAxis(Axis value) {
    if (_mainAxis == value) {
      return;
    }
    _mainAxis = value;
    // Child order needs to be resorted, which happens in performLayout.
    markNeedsLayout();
  }

  /// {@macro flutter.rendering.RenderViewportBase.cacheExtent}
  double get cacheExtent => _cacheExtent ?? RenderAbstractViewport.defaultCacheExtent;
  double? _cacheExtent;
  set cacheExtent(double? value) {
    if (_cacheExtent == value) {
      return;
    }
    _cacheExtent = value;
    markNeedsLayout();
  }

  /// {@macro flutter.material.Material.clipBehavior}
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior;
  set clipBehavior(Clip value) {
    if (_clipBehavior == value) {
      return;
    }
    _clipBehavior = value;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  final TwoDimensionalChildManager _childManager;
  final Map<ChildVicinity, RenderBox> _children = <ChildVicinity, RenderBox>{};

  /// Children that have been laid out (or re-used) during the course of
  /// performLayout, used to update the keep alive bucket at the end of
  /// performLayout.
  final Map<ChildVicinity, RenderBox> _activeChildrenForLayoutPass = <ChildVicinity, RenderBox>{};

  /// The nodes being kept alive despite not being visible.
  final Map<ChildVicinity, RenderBox> _keepAliveBucket = <ChildVicinity, RenderBox>{};

  late List<RenderBox> _debugDanglingKeepAlives;

  bool _hasVisualOverflow = false;
  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  bool get isRepaintBoundary => true;

  @override
  bool get sizedByParent => true;

  // Keeps track of the upper and lower bounds of ChildVicinity indices when
  // subclasses call buildOrObtainChildFor during layoutChildSequence. These
  // values are used to sort children in accordance with the mainAxis for
  // paint order.
  int? _leadingXIndex;
  int? _trailingXIndex;
  int? _leadingYIndex;
  int? _trailingYIndex;

  /// The first child of the viewport according to the traversal order of the
  /// [mainAxis].
  ///
  /// {@template flutter.rendering.twoDimensionalViewport.paintOrder}
  /// The [mainAxis] correlates with the [ChildVicinity] of each child to paint
  /// the children in a row or column major order.
  ///
  /// By default, the [mainAxis] is [Axis.vertical], which would result in a
  /// row major paint order, visiting children in the horizontal indices before
  /// advancing to the next vertical index.
  /// {@endtemplate}
  ///
  /// This value is null during [layoutChildSequence] as children are reified
  /// into the correct order after layout is completed. This can be used when
  /// overriding [paint] in order to paint the children in the correct order.
  RenderBox? get firstChild => _firstChild;
  RenderBox? _firstChild;

  /// The last child in the viewport according to the traversal order of the
  /// [mainAxis].
  ///
  /// {@macro flutter.rendering.twoDimensionalViewport.paintOrder}
  ///
  /// This value is null during [layoutChildSequence] as children are reified
  /// into the correct order after layout is completed. This can be used when
  /// overriding [paint] in order to paint the children in the correct order.
  RenderBox? get lastChild => _lastChild;
  RenderBox? _lastChild;

  /// The previous child before the given child in the child list according to
  /// the traversal order of the [mainAxis].
  ///
  /// {@macro flutter.rendering.twoDimensionalViewport.paintOrder}
  ///
  /// This method is useful when overriding [paint] in order to paint children
  /// in the correct order.
  RenderBox? childBefore(RenderBox child) {
    assert(child.parent == this);
    return parentDataOf(child)._previousSibling;
  }

  /// The next child after the given child in the child list according to
  /// the traversal order of the [mainAxis].
  ///
  /// {@macro flutter.rendering.twoDimensionalViewport.paintOrder}
  ///
  /// This method is useful when overriding [paint] in order to paint children
  /// in the correct order.
  RenderBox? childAfter(RenderBox child) {
    assert(child.parent == this);
    return parentDataOf(child)._nextSibling;
  }

  void _handleDelegateNotification() {
    return markNeedsLayout(withDelegateRebuild: true);
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! TwoDimensionalViewportParentData) {
      child.parentData = TwoDimensionalViewportParentData();
    }
  }

  /// Convenience method for retrieving and casting the [ParentData] of the
  /// viewport's children.
  ///
  /// Children must have a [ParentData] of type
  /// [TwoDimensionalViewportParentData], or a subclass thereof.
  @protected
  @mustCallSuper
  TwoDimensionalViewportParentData parentDataOf(RenderBox child) {
    assert(
      _children.containsValue(child) ||
          _keepAliveBucket.containsValue(child) ||
          _debugOrphans!.contains(child),
    );
    return child.parentData! as TwoDimensionalViewportParentData;
  }

  /// Returns the active child located at the provided [ChildVicinity], if there
  /// is one.
  ///
  /// This can be used by subclasses to access currently active children to make
  /// use of their size or [TwoDimensionalViewportParentData], such as when
  /// overriding the [paint] method.
  ///
  /// Returns null if there is no active child for the given [ChildVicinity].
  @protected
  RenderBox? getChildFor(covariant ChildVicinity vicinity) => _children[vicinity];

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _horizontalOffset.addListener(markNeedsLayout);
    _verticalOffset.addListener(markNeedsLayout);
    _delegate.addListener(_handleDelegateNotification);
    for (final RenderBox child in _children.values) {
      child.attach(owner);
    }
    for (final RenderBox child in _keepAliveBucket.values) {
      child.attach(owner);
    }
  }

  @override
  void detach() {
    super.detach();
    _horizontalOffset.removeListener(markNeedsLayout);
    _verticalOffset.removeListener(markNeedsLayout);
    _delegate.removeListener(_handleDelegateNotification);
    for (final RenderBox child in _children.values) {
      child.detach();
    }
    for (final RenderBox child in _keepAliveBucket.values) {
      child.detach();
    }
  }

  @override
  void redepthChildren() {
    for (final RenderBox child in _children.values) {
      child.redepthChildren();
    }
    _keepAliveBucket.values.forEach(redepthChild);
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    RenderBox? child = _firstChild;
    while (child != null) {
      visitor(child);
      child = parentDataOf(child)._nextSibling;
    }
    _keepAliveBucket.values.forEach(visitor);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    // Only children that are visible should be visited, and they must be in
    // paint order.
    RenderBox? child = _firstChild;
    while (child != null) {
      final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
      visitor(child);
      child = childParentData._nextSibling;
    }
    // Do not visit children in [_keepAliveBucket].
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> debugChildren = <DiagnosticsNode>[
      ..._children.keys.map<DiagnosticsNode>((ChildVicinity vicinity) {
        return _children[vicinity]!.toDiagnosticsNode(name: vicinity.toString());
      }),
    ];
    return debugChildren;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    assert(debugCheckHasBoundedAxis(Axis.vertical, constraints));
    assert(debugCheckHasBoundedAxis(Axis.horizontal, constraints));
    return constraints.biggest;
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    for (final RenderBox child in _children.values) {
      final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
      if (!childParentData.isVisible) {
        // Can't hit a child that is not visible.
        continue;
      }
      final bool isHit = result.addWithPaintOffset(
        offset: childParentData.paintOffset,
        position: position,
        hitTest: (BoxHitTestResult result, Offset transformed) {
          assert(transformed == position - childParentData.paintOffset!);
          return child.hitTest(result, position: transformed);
        },
      );
      if (isHit) {
        return true;
      }
    }
    return false;
  }

  /// The dimensions of the viewport.
  ///
  /// This [Size] represents the width and height of the visible area.
  Size get viewportDimension {
    assert(hasSize);
    return size;
  }

  @override
  void performResize() {
    final Size? oldSize = hasSize ? size : null;
    super.performResize();
    // Ignoring return value since we are doing a layout either way
    // (performLayout will be invoked next).
    horizontalOffset.applyViewportDimension(size.width);
    verticalOffset.applyViewportDimension(size.height);
    if (oldSize != size) {
      // Specs can depend on viewport size.
      _didResize = true;
    }
  }

  @protected
  @override
  RevealedOffset getOffsetToReveal(
    RenderObject target,
    double alignment, {
    Rect? rect,
    Axis? axis,
  }) {
    // If an axis has not been specified, use the mainAxis.
    axis ??= mainAxis;

    final (double offset, AxisDirection axisDirection) = switch (axis) {
      Axis.vertical => (verticalOffset.pixels, verticalAxisDirection),
      Axis.horizontal => (horizontalOffset.pixels, horizontalAxisDirection),
    };

    rect ??= target.paintBounds;
    // `child` will be the last RenderObject before the viewport when walking
    // up from `target`.
    RenderObject child = target;
    while (child.parent != this) {
      child = child.parent!;
    }

    assert(child.parent == this);
    final RenderBox box = child as RenderBox;
    final Rect rectLocal = MatrixUtils.transformRect(target.getTransformTo(child), rect);

    double leadingScrollOffset = offset;

    // The scroll offset of `rect` within `child`.
    leadingScrollOffset += switch (axisDirection) {
      AxisDirection.up => child.size.height - rectLocal.bottom,
      AxisDirection.left => child.size.width - rectLocal.right,
      AxisDirection.right => rectLocal.left,
      AxisDirection.down => rectLocal.top,
    };

    // The scroll offset in the viewport to `rect`.
    final Offset paintOffset = parentDataOf(box).paintOffset!;
    leadingScrollOffset += switch (axisDirection) {
      AxisDirection.up => viewportDimension.height - paintOffset.dy - box.size.height,
      AxisDirection.left => viewportDimension.width - paintOffset.dx - box.size.width,
      AxisDirection.right => paintOffset.dx,
      AxisDirection.down => paintOffset.dy,
    };

    // This step assumes the viewport's layout is up-to-date, i.e., if
    // the position is changed after the last performLayout, the new scroll
    // position will not be accounted for.
    final Matrix4 transform = target.getTransformTo(this);
    Rect targetRect = MatrixUtils.transformRect(transform, rect);

    final double mainAxisExtentDifference = switch (axis) {
      Axis.horizontal => viewportDimension.width - rectLocal.width,
      Axis.vertical => viewportDimension.height - rectLocal.height,
    };

    final double targetOffset = leadingScrollOffset - mainAxisExtentDifference * alignment;

    final double offsetDifference = switch (axis) {
      Axis.horizontal => horizontalOffset.pixels - targetOffset,
      Axis.vertical => verticalOffset.pixels - targetOffset,
    };

    targetRect = switch (axisDirection) {
      AxisDirection.up => targetRect.translate(0.0, -offsetDifference),
      AxisDirection.down => targetRect.translate(0.0, offsetDifference),
      AxisDirection.left => targetRect.translate(-offsetDifference, 0.0),
      AxisDirection.right => targetRect.translate(offsetDifference, 0.0),
    };

    final RevealedOffset revealedOffset = RevealedOffset(offset: targetOffset, rect: targetRect);
    return revealedOffset;
  }

  @override
  void showOnScreen({
    RenderObject? descendant,
    Rect? rect,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
  }) {
    // It is possible for one and not both axes to allow for implicit scrolling,
    // so handling is split between the options for allowed implicit scrolling.
    final bool allowHorizontal = horizontalOffset.allowImplicitScrolling;
    final bool allowVertical = verticalOffset.allowImplicitScrolling;
    AxisDirection? axisDirection;
    switch ((allowHorizontal, allowVertical)) {
      case (true, true):
        // Both allow implicit scrolling.
        break;
      case (false, true):
        // Only the vertical Axis allows implicit scrolling.
        axisDirection = verticalAxisDirection;
      case (true, false):
        // Only the horizontal Axis allows implicit scrolling.
        axisDirection = horizontalAxisDirection;
      case (false, false):
        // Neither axis allows for implicit scrolling.
        return super.showOnScreen(
          descendant: descendant,
          rect: rect,
          duration: duration,
          curve: curve,
        );
    }

    final Rect? newRect = RenderTwoDimensionalViewport.showInViewport(
      descendant: descendant,
      viewport: this,
      axisDirection: axisDirection,
      rect: rect,
      duration: duration,
      curve: curve,
    );

    super.showOnScreen(rect: newRect, duration: duration, curve: curve);
  }

  /// Make (a portion of) the given `descendant` of the given `viewport` fully
  /// visible in one or both dimensions of the `viewport` by manipulating the
  /// [ViewportOffset]s.
  ///
  /// The `axisDirection` determines from which axes the `descendant` will be
  /// revealed. When the `axisDirection` is null, both axes will be updated to
  /// reveal the descendant.
  ///
  /// The optional `rect` parameter describes which area of the `descendant`
  /// should be shown in the viewport. If `rect` is null, the entire
  /// `descendant` will be revealed. The `rect` parameter is interpreted
  /// relative to the coordinate system of `descendant`.
  ///
  /// The returned [Rect] describes the new location of `descendant` or `rect`
  /// in the viewport after it has been revealed. See [RevealedOffset.rect]
  /// for a full definition of this [Rect].
  ///
  /// The parameter `viewport` is required and cannot be null. If `descendant`
  /// is null, this is a no-op and `rect` is returned.
  ///
  /// If both `descendant` and `rect` are null, null is returned because there
  /// is nothing to be shown in the viewport.
  ///
  /// The `duration` parameter can be set to a non-zero value to animate the
  /// target object into the viewport with an animation defined by `curve`.
  ///
  /// See also:
  ///
  /// * [RenderObject.showOnScreen], overridden by
  ///   [RenderTwoDimensionalViewport] to delegate to this method.
  static Rect? showInViewport({
    RenderObject? descendant,
    Rect? rect,
    required RenderTwoDimensionalViewport viewport,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    AxisDirection? axisDirection,
  }) {
    if (descendant == null) {
      return rect;
    }

    Rect? showVertical(Rect? rect) {
      return RenderTwoDimensionalViewport._showInViewportForAxisDirection(
        descendant: descendant,
        viewport: viewport,
        axis: Axis.vertical,
        rect: rect,
        duration: duration,
        curve: curve,
      );
    }

    Rect? showHorizontal(Rect? rect) {
      return RenderTwoDimensionalViewport._showInViewportForAxisDirection(
        descendant: descendant,
        viewport: viewport,
        axis: Axis.horizontal,
        rect: rect,
        duration: duration,
        curve: curve,
      );
    }

    switch (axisDirection) {
      case AxisDirection.left:
      case AxisDirection.right:
        return showHorizontal(rect);
      case AxisDirection.up:
      case AxisDirection.down:
        return showVertical(rect);
      case null:
        // Update rect after revealing in one axis before revealing in the next.
        rect = showHorizontal(rect) ?? rect;
        // We only return the final rect after both have been revealed.
        rect = showVertical(rect);
        if (rect == null) {
          // `descendant` is between leading and trailing edge and hence already
          //  fully shown on screen.
          assert(viewport.parent != null);
          final Matrix4 transform = descendant.getTransformTo(viewport.parent);
          return MatrixUtils.transformRect(transform, rect ?? descendant.paintBounds);
        }
        return rect;
    }
  }

  static Rect? _showInViewportForAxisDirection({
    required RenderObject descendant,
    Rect? rect,
    required RenderTwoDimensionalViewport viewport,
    required Axis axis,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
  }) {
    final ViewportOffset offset = switch (axis) {
      Axis.vertical => viewport.verticalOffset,
      Axis.horizontal => viewport.horizontalOffset,
    };

    final RevealedOffset leadingEdgeOffset = viewport.getOffsetToReveal(
      descendant,
      0.0,
      rect: rect,
      axis: axis,
    );
    final RevealedOffset trailingEdgeOffset = viewport.getOffsetToReveal(
      descendant,
      1.0,
      rect: rect,
      axis: axis,
    );
    final double currentOffset = offset.pixels;

    final RevealedOffset? targetOffset = RevealedOffset.clampOffset(
      leadingEdgeOffset: leadingEdgeOffset,
      trailingEdgeOffset: trailingEdgeOffset,
      currentOffset: currentOffset,
    );
    if (targetOffset == null) {
      // Already visible in this axis.
      return null;
    }

    offset.moveTo(targetOffset.offset, duration: duration, curve: curve);
    return targetOffset.rect;
  }

  /// Should be used by subclasses to invalidate any cached metrics for the
  /// viewport.
  ///
  /// This is set to true when the viewport has been resized, indicating that
  /// any cached dimensions are invalid.
  ///
  /// After performLayout, the value is set to false until the viewport
  /// dimensions are changed again in [performResize].
  ///
  /// Subclasses are not required to use this value, but it can be used to
  /// safely cache layout information in between layout calls.
  bool get didResize => _didResize;
  bool _didResize = true;

  /// Should be used by subclasses to invalidate any cached data from the
  /// [delegate].
  ///
  /// This value is set to false after [layoutChildSequence]. If
  /// [markNeedsLayout] is called `withDelegateRebuild` set to true, then this
  /// value will be updated to true, signifying any cached delegate information
  /// needs to be updated in the next call to [layoutChildSequence].
  ///
  /// Subclasses are not required to use this value, but it can be used to
  /// safely cache layout information in between layout calls.
  @protected
  bool get needsDelegateRebuild => _needsDelegateRebuild;
  bool _needsDelegateRebuild = true;

  @override
  void markNeedsLayout({bool withDelegateRebuild = false}) {
    _needsDelegateRebuild = _needsDelegateRebuild || withDelegateRebuild;
    super.markNeedsLayout();
  }

  /// Primary work horse of [performLayout].
  ///
  /// Subclasses must implement this method to layout the children of the
  /// viewport. The [TwoDimensionalViewportParentData.layoutOffset] must be set
  /// during this method in order for the children to be positioned during paint.
  /// Further, children of the viewport must be laid out with the expectation
  /// that the parent (this viewport) will use their size.
  ///
  /// ```dart
  /// child.layout(constraints, parentUsesSize: true);
  /// ```
  ///
  /// The primary methods used for creating and obtaining children is
  /// [buildOrObtainChildFor], which takes a [ChildVicinity] that is used by the
  /// [TwoDimensionalChildDelegate]. If a child is not provided by the delegate
  /// for the provided vicinity, the method will return null, otherwise, it will
  /// return the [RenderBox] of the child.
  ///
  /// After [layoutChildSequence] is completed, any remaining children that were
  /// not obtained will be disposed.
  void layoutChildSequence();

  @override
  void performLayout() {
    _firstChild = null;
    _lastChild = null;
    _activeChildrenForLayoutPass.clear();
    _childManager._startLayout();

    // Subclass lays out children.
    layoutChildSequence();

    assert(_debugCheckContentDimensions());
    _didResize = false;
    _needsDelegateRebuild = false;
    _cacheKeepAlives();
    invokeLayoutCallback<BoxConstraints>((BoxConstraints _) {
      _childManager._endLayout();
      assert(_debugOrphans?.isEmpty ?? true);
      assert(_debugDanglingKeepAlives.isEmpty);
      // Ensure we are not keeping anything alive that should not be any longer.
      assert(
        _keepAliveBucket.values.where((RenderBox child) {
          return !parentDataOf(child).keepAlive;
        }).isEmpty,
      );
      // Organize children in paint order and complete parent data after
      // un-used children are disposed of by the childManager.
      _reifyChildren();
    });
  }

  void _cacheKeepAlives() {
    final List<RenderBox> remainingChildren =
        _children.values.toSet().difference(_activeChildrenForLayoutPass.values.toSet()).toList();
    for (final RenderBox child in remainingChildren) {
      final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
      if (childParentData.keepAlive) {
        _keepAliveBucket[childParentData.vicinity] = child;
        // Let the child manager know we intend to keep this.
        _childManager._reuseChild(childParentData.vicinity);
      }
    }
  }

  // Ensures all children have a layoutOffset, sets paintExtent & paintOffset,
  // and arranges children in paint order.
  void _reifyChildren() {
    assert(_leadingXIndex != null);
    assert(_trailingXIndex != null);
    assert(_leadingYIndex != null);
    assert(_trailingYIndex != null);
    assert(_firstChild == null);
    assert(_lastChild == null);
    RenderBox? previousChild;
    switch (mainAxis) {
      case Axis.vertical:
        // Row major traversal.
        // This seems backwards, but the vertical axis is the typical default
        // axis for scrolling in Flutter, while Row-major ordering is the
        // typical default for matrices, which is why the inverse follows
        // through in the horizontal case below.
        // Minor
        for (int minorIndex = _leadingYIndex!; minorIndex <= _trailingYIndex!; minorIndex++) {
          // Major
          for (int majorIndex = _leadingXIndex!; majorIndex <= _trailingXIndex!; majorIndex++) {
            final ChildVicinity vicinity = ChildVicinity(xIndex: majorIndex, yIndex: minorIndex);
            previousChild =
                _completeChildParentData(vicinity, previousChild: previousChild) ?? previousChild;
          }
        }
      case Axis.horizontal:
        // Column major traversal
        // Minor
        for (int minorIndex = _leadingXIndex!; minorIndex <= _trailingXIndex!; minorIndex++) {
          // Major
          for (int majorIndex = _leadingYIndex!; majorIndex <= _trailingYIndex!; majorIndex++) {
            final ChildVicinity vicinity = ChildVicinity(xIndex: minorIndex, yIndex: majorIndex);
            previousChild =
                _completeChildParentData(vicinity, previousChild: previousChild) ?? previousChild;
          }
        }
    }
    _lastChild = previousChild;
    if (_lastChild != null) {
      parentDataOf(_lastChild!)._nextSibling = null;
    }
    // Reset for next layout pass.
    _leadingXIndex = null;
    _trailingXIndex = null;
    _leadingYIndex = null;
    _trailingYIndex = null;
  }

  RenderBox? _completeChildParentData(ChildVicinity vicinity, {RenderBox? previousChild}) {
    assert(vicinity != ChildVicinity.invalid);
    // It is possible and valid for a vicinity to be skipped.
    // For example, a table can have merged cells, spanning multiple
    // indices, but only represented by one RenderBox and ChildVicinity.
    if (_children.containsKey(vicinity)) {
      final RenderBox child = _children[vicinity]!;
      assert(parentDataOf(child).vicinity == vicinity);
      updateChildPaintData(child);
      if (previousChild == null) {
        // _firstChild is only set once.
        assert(_firstChild == null);
        _firstChild = child;
      } else {
        parentDataOf(previousChild)._nextSibling = child;
        parentDataOf(child)._previousSibling = previousChild;
      }
      return child;
    }
    return null;
  }

  bool _debugCheckContentDimensions() {
    const String hint =
        'Subclasses should call applyContentDimensions on the '
        'verticalOffset and horizontalOffset to set the min and max scroll offset. '
        'If the contents exceed one or both sides of the viewportDimension, '
        'ensure the viewportDimension height or width is subtracted in that axis '
        'for the correct extent.';
    assert(() {
      if (!(verticalOffset as ScrollPosition).hasContentDimensions) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The verticalOffset was not given content dimensions during '
            'layoutChildSequence.',
          ),
          ErrorHint(hint),
        ]);
      }
      return true;
    }());
    assert(() {
      if (!(horizontalOffset as ScrollPosition).hasContentDimensions) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The horizontalOffset was not given content dimensions during '
            'layoutChildSequence.',
          ),
          ErrorHint(hint),
        ]);
      }
      return true;
    }());
    return true;
  }

  /// Returns the child for a given [ChildVicinity], should be called during
  /// [layoutChildSequence] in order to instantiate or retrieve children.
  ///
  /// This method will build the child if it has not been already, or will reuse
  /// it if it already exists, whether it was part of the previous frame or kept
  /// alive.
  ///
  /// Children for the given [ChildVicinity] will be inserted into the active
  /// children list, and so should be visible, or contained within the
  /// [cacheExtent].
  RenderBox? buildOrObtainChildFor(ChildVicinity vicinity) {
    assert(vicinity != ChildVicinity.invalid);
    // This should only be called during layout.
    assert(debugDoingThisLayout);
    if (_leadingXIndex == null ||
        _trailingXIndex == null ||
        _leadingXIndex == null ||
        _trailingYIndex == null) {
      // First child of this layout pass. Set leading and trailing trackers.
      _leadingXIndex = vicinity.xIndex;
      _trailingXIndex = vicinity.xIndex;
      _leadingYIndex = vicinity.yIndex;
      _trailingYIndex = vicinity.yIndex;
    } else {
      // If any of these are still null, we missed a child.
      assert(_leadingXIndex != null);
      assert(_trailingXIndex != null);
      assert(_leadingYIndex != null);
      assert(_trailingYIndex != null);

      // Update as we go.
      _leadingXIndex = math.min(vicinity.xIndex, _leadingXIndex!);
      _trailingXIndex = math.max(vicinity.xIndex, _trailingXIndex!);
      _leadingYIndex = math.min(vicinity.yIndex, _leadingYIndex!);
      _trailingYIndex = math.max(vicinity.yIndex, _trailingYIndex!);
    }
    if (_needsDelegateRebuild ||
        (!_children.containsKey(vicinity) && !_keepAliveBucket.containsKey(vicinity))) {
      invokeLayoutCallback<BoxConstraints>((BoxConstraints _) {
        _childManager._buildChild(vicinity);
      });
    } else {
      _keepAliveBucket.remove(vicinity);
      _childManager._reuseChild(vicinity);
    }
    if (!_children.containsKey(vicinity)) {
      // There is no child for this vicinity, we may have reached the end of the
      // children in one or both of the x/y indices.
      return null;
    }

    assert(_children.containsKey(vicinity));
    final RenderBox child = _children[vicinity]!;
    _activeChildrenForLayoutPass[vicinity] = child;
    parentDataOf(child).vicinity = vicinity;
    return child;
  }

  /// Called after [layoutChildSequence] to compute the
  /// [TwoDimensionalViewportParentData.paintOffset] and
  /// [TwoDimensionalViewportParentData._paintExtent] of the child.
  void updateChildPaintData(RenderBox child) {
    final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
    assert(
      childParentData.layoutOffset != null,
      'The child with ChildVicinity(xIndex: ${childParentData.vicinity.xIndex}, '
      'yIndex: ${childParentData.vicinity.yIndex}) was not provided a '
      'layoutOffset. This should be set during layoutChildSequence, '
      'representing the position of the child.',
    );
    assert(child.hasSize); // Child must have been laid out by now.

    // Set paintExtent (and visibility)
    childParentData._paintExtent = computeChildPaintExtent(
      childParentData.layoutOffset!,
      child.size,
    );
    // Set paintOffset
    childParentData.paintOffset = computeAbsolutePaintOffsetFor(
      child,
      layoutOffset: childParentData.layoutOffset!,
    );
    // If the child is partially visible, or not visible at all, there is
    // visual overflow.
    _hasVisualOverflow =
        _hasVisualOverflow ||
        childParentData.layoutOffset != childParentData._paintExtent ||
        !childParentData.isVisible;
  }

  /// Computes the portion of the child that is visible, assuming that only the
  /// region from the [ViewportOffset.pixels] of both dimensions to the
  /// [cacheExtent] is visible, and that the relationship between scroll offsets
  /// and paint offsets is linear.
  ///
  /// For example, if the [ViewportOffset]s each have a scroll offset of 100 and
  /// the arguments to this method describe a child with [layoutOffset] of
  /// `Offset(50.0, 50.0)`, with a size of `Size(200.0, 200.0)`, then the
  /// returned value would be `Size(150.0, 150.0)`, representing the visible
  /// extent of the child.
  Size computeChildPaintExtent(Offset layoutOffset, Size childSize) {
    if (childSize == Size.zero || childSize.height == 0.0 || childSize.width == 0.0) {
      return Size.zero;
    }
    // Horizontal extent
    final double width;
    if (layoutOffset.dx < 0.0) {
      // The child is positioned beyond the leading edge of the viewport.
      if (layoutOffset.dx + childSize.width <= 0.0) {
        // The child does not extend into the viewable area, it is not visible.
        return Size.zero;
      }
      // If the child is positioned starting at -50, then the paint extent is
      // the width + (-50).
      width = layoutOffset.dx + childSize.width;
    } else if (layoutOffset.dx >= viewportDimension.width) {
      // The child is positioned after the trailing edge of the viewport, also
      // not visible.
      return Size.zero;
    } else {
      // The child is positioned within the viewport bounds, but may extend
      // beyond it.
      assert(layoutOffset.dx >= 0 && layoutOffset.dx < viewportDimension.width);
      if (layoutOffset.dx + childSize.width > viewportDimension.width) {
        width = viewportDimension.width - layoutOffset.dx;
      } else {
        assert(layoutOffset.dx + childSize.width <= viewportDimension.width);
        width = childSize.width;
      }
    }

    // Vertical extent
    final double height;
    if (layoutOffset.dy < 0.0) {
      // The child is positioned beyond the leading edge of the viewport.
      if (layoutOffset.dy + childSize.height <= 0.0) {
        // The child does not extend into the viewable area, it is not visible.
        return Size.zero;
      }
      // If the child is positioned starting at -50, then the paint extent is
      // the width + (-50).
      height = layoutOffset.dy + childSize.height;
    } else if (layoutOffset.dy >= viewportDimension.height) {
      // The child is positioned after the trailing edge of the viewport, also
      // not visible.
      return Size.zero;
    } else {
      // The child is positioned within the viewport bounds, but may extend
      // beyond it.
      assert(layoutOffset.dy >= 0 && layoutOffset.dy < viewportDimension.height);
      if (layoutOffset.dy + childSize.height > viewportDimension.height) {
        height = viewportDimension.height - layoutOffset.dy;
      } else {
        assert(layoutOffset.dy + childSize.height <= viewportDimension.height);
        height = childSize.height;
      }
    }

    return Size(width, height);
  }

  /// The offset at which the given `child` should be painted.
  ///
  /// The returned offset is from the top left corner of the inside of the
  /// viewport to the top left corner of the paint coordinate system of the
  /// `child`.
  ///
  /// This is useful when the one or both of the axes of the viewport are
  /// reversed. The normalized layout offset of the child is used to compute
  /// the paint offset in relation to the [verticalAxisDirection] and
  /// [horizontalAxisDirection].
  @protected
  Offset computeAbsolutePaintOffsetFor(RenderBox child, {required Offset layoutOffset}) {
    // This is only usable once we have sizes.
    assert(hasSize);
    assert(child.hasSize);
    final double xOffset = switch (horizontalAxisDirection) {
      AxisDirection.right => layoutOffset.dx,
      AxisDirection.left => viewportDimension.width - (layoutOffset.dx + child.size.width),
      AxisDirection.up || AxisDirection.down => throw Exception('This should not happen'),
    };
    final double yOffset = switch (verticalAxisDirection) {
      AxisDirection.up => viewportDimension.height - (layoutOffset.dy + child.size.height),
      AxisDirection.down => layoutOffset.dy,
      AxisDirection.right || AxisDirection.left => throw Exception('This should not happen'),
    };
    return Offset(xOffset, yOffset);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (_children.isEmpty) {
      return;
    }
    if (_hasVisualOverflow && clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & viewportDimension,
        _paintChildren,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      _paintChildren(context, offset);
    }
  }

  void _paintChildren(PaintingContext context, Offset offset) {
    RenderBox? child = _firstChild;
    while (child != null) {
      final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
      if (childParentData.isVisible) {
        context.paintChild(child, offset + childParentData.paintOffset!);
      }
      child = childParentData._nextSibling;
    }
  }

  // ---- Called from _TwoDimensionalViewportElement ----

  void _insertChild(RenderBox child, ChildVicinity slot) {
    assert(_debugTrackOrphans(newOrphan: _children[slot]));
    assert(!_keepAliveBucket.containsValue(child));
    _children[slot] = child;
    adoptChild(child);
  }

  void _moveChild(RenderBox child, {required ChildVicinity from, required ChildVicinity to}) {
    final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
    if (!childParentData.keptAlive) {
      if (_children[from] == child) {
        _children.remove(from);
      }
      assert(_debugTrackOrphans(newOrphan: _children[to], noLongerOrphan: child));
      _children[to] = child;
      return;
    }
    // If the child in the bucket is not current child, that means someone has
    // already moved and replaced current child, and we cannot remove this
    // child.
    if (_keepAliveBucket[childParentData.vicinity] == child) {
      _keepAliveBucket.remove(childParentData.vicinity);
    }
    assert(() {
      _debugDanglingKeepAlives.remove(child);
      return true;
    }());
    // If there is an existing child in the new slot, that mean that child
    // will be moved to other index. In other cases, the existing child should
    // have been removed by _removeChild. Thus, it is ok to overwrite it.
    assert(() {
      if (_keepAliveBucket.containsKey(childParentData.vicinity)) {
        _debugDanglingKeepAlives.add(_keepAliveBucket[childParentData.vicinity]!);
      }
      return true;
    }());
    _keepAliveBucket[childParentData.vicinity] = child;
  }

  void _removeChild(RenderBox child, ChildVicinity slot) {
    final TwoDimensionalViewportParentData childParentData = parentDataOf(child);
    if (!childParentData.keptAlive) {
      if (_children[slot] == child) {
        _children.remove(slot);
      }
      assert(_debugTrackOrphans(noLongerOrphan: child));
      if (_keepAliveBucket[childParentData.vicinity] == child) {
        _keepAliveBucket.remove(childParentData.vicinity);
      }
      assert(_keepAliveBucket[childParentData.vicinity] != child);
      dropChild(child);
      return;
    }
    assert(_keepAliveBucket[childParentData.vicinity] == child);
    assert(() {
      _debugDanglingKeepAlives.remove(child);
      return true;
    }());
    _keepAliveBucket.remove(childParentData.vicinity);
    dropChild(child);
  }

  List<RenderBox>? _debugOrphans;

  // When a child is inserted into a slot currently occupied by another child,
  // it becomes an orphan until it is either moved to another slot or removed.
  bool _debugTrackOrphans({RenderBox? newOrphan, RenderBox? noLongerOrphan}) {
    assert(() {
      _debugOrphans ??= <RenderBox>[];
      if (newOrphan != null) {
        _debugOrphans!.add(newOrphan);
      }
      if (noLongerOrphan != null) {
        _debugOrphans!.remove(noLongerOrphan);
      }
      return true;
    }());
    return true;
  }

  /// Throws an exception saying that the object does not support returning
  /// intrinsic dimensions if, in debug mode, we are not in the
  /// [RenderObject.debugCheckingIntrinsics] mode.
  ///
  /// This is used by [computeMinIntrinsicWidth] et al because viewports do not
  /// generally support returning intrinsic dimensions. See the discussion at
  /// [computeMinIntrinsicWidth].
  @protected
  bool debugThrowIfNotCheckingIntrinsics() {
    assert(() {
      if (!RenderObject.debugCheckingIntrinsics) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType does not support returning intrinsic dimensions.'),
          ErrorDescription(
            'Calculating the intrinsic dimensions would require instantiating every child of '
            'the viewport, which defeats the point of viewports being lazy.',
          ),
        ]);
      }
      return true;
    }());
    return true;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    final Offset paintOffset = parentDataOf(child).paintOffset!;
    transform.translate(paintOffset.dx, paintOffset.dy);
  }

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }
}

/// A delegate used by [RenderTwoDimensionalViewport] to manage its children.
///
/// [RenderTwoDimensionalViewport] objects reify their children lazily to avoid
/// spending resources on children that are not visible in the viewport. This
/// delegate lets these objects create, reuse and remove children.
abstract class TwoDimensionalChildManager {
  void _startLayout();
  void _buildChild(ChildVicinity vicinity);
  void _reuseChild(ChildVicinity vicinity);
  void _endLayout();
}

/// The relative position of a child in a [TwoDimensionalViewport] in relation
/// to other children of the viewport.
///
/// While children can be plotted arbitrarily in two dimensional space, the
/// [ChildVicinity] is used to disambiguate their positions, determining how to
/// traverse the children of the space.
///
/// Combined with the [RenderTwoDimensionalViewport.mainAxis], each child's
/// vicinity determines its paint order among all of the children.
@immutable
class ChildVicinity implements Comparable<ChildVicinity> {
  /// Creates a reference to a child in a two dimensional plane, with the
  /// [xIndex] and [yIndex] being relative to other children in the viewport.
  const ChildVicinity({required this.xIndex, required this.yIndex})
    : assert(xIndex >= -1),
      assert(yIndex >= -1);

  /// Represents an unassigned child position. The given child may be in the
  /// process of moving from one position to another.
  static const ChildVicinity invalid = ChildVicinity(xIndex: -1, yIndex: -1);

  /// The index of the child in the horizontal axis, relative to neighboring
  /// children.
  ///
  /// While children's offset and positioning may not be strictly defined in
  /// terms of rows and columns, like a table, [ChildVicinity.xIndex] and
  /// [ChildVicinity.yIndex] represents order of traversal in row or column
  /// major format.
  final int xIndex;

  /// The index of the child in the vertical axis, relative to neighboring
  /// children.
  ///
  /// While children's offset and positioning may not be strictly defined in
  /// terms of rows and columns, like a table, [ChildVicinity.xIndex] and
  /// [ChildVicinity.yIndex] represents order of traversal in row or column
  /// major format.
  final int yIndex;

  @override
  bool operator ==(Object other) {
    return other is ChildVicinity && other.xIndex == xIndex && other.yIndex == yIndex;
  }

  @override
  int get hashCode => Object.hash(xIndex, yIndex);

  @override
  int compareTo(ChildVicinity other) {
    if (xIndex == other.xIndex) {
      return yIndex - other.yIndex;
    }
    return xIndex - other.xIndex;
  }

  @override
  String toString() {
    return '(xIndex: $xIndex, yIndex: $yIndex)';
  }
}
