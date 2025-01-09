// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'stack.dart';
library;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'object.dart';

// For SingleChildLayoutDelegate and RenderCustomSingleChildLayoutBox, see shifted_box.dart

/// [ParentData] used by [RenderCustomMultiChildLayoutBox].
class MultiChildLayoutParentData extends ContainerBoxParentData<RenderBox> {
  /// An object representing the identity of this child.
  Object? id;

  @override
  String toString() => '${super.toString()}; id=$id';
}

/// A delegate that controls the layout of multiple children.
///
/// Used with [CustomMultiChildLayout] (in the widgets library) and
/// [RenderCustomMultiChildLayoutBox] (in the rendering library).
///
/// Delegates must be idempotent. Specifically, if two delegates are equal, then
/// they must produce the same layout. To change the layout, replace the
/// delegate with a different instance whose [shouldRelayout] returns true when
/// given the previous instance.
///
/// Override [getSize] to control the overall size of the layout. The size of
/// the layout cannot depend on layout properties of the children. This was
/// a design decision to simplify the delegate implementations: This way,
/// the delegate implementations do not have to also handle various intrinsic
/// sizing functions if the parent's size depended on the children.
/// If you want to build a custom layout where you define the size of that widget
/// based on its children, then you will have to create a custom render object.
/// See [MultiChildRenderObjectWidget] with [ContainerRenderObjectMixin] and
/// [RenderBoxContainerDefaultsMixin] to get started or [RenderStack] for an
/// example implementation.
///
/// Override [performLayout] to size and position the children. An
/// implementation of [performLayout] must call [layoutChild] exactly once for
/// each child, but it may call [layoutChild] on children in an arbitrary order.
/// Typically a delegate will use the size returned from [layoutChild] on one
/// child to determine the constraints for [performLayout] on another child or
/// to determine the offset for [positionChild] for that child or another child.
///
/// Override [shouldRelayout] to determine when the layout of the children needs
/// to be recomputed when the delegate changes.
///
/// The most efficient way to trigger a relayout is to supply a `relayout`
/// argument to the constructor of the [MultiChildLayoutDelegate]. The custom
/// layout will listen to this value and relayout whenever the Listenable
/// notifies its listeners, such as when an [Animation] ticks. This allows
/// the custom layout to avoid the build phase of the pipeline.
///
/// Each child must be wrapped in a [LayoutId] widget to assign the id that
/// identifies it to the delegate. The [LayoutId.id] needs to be unique among
/// the children that the [CustomMultiChildLayout] manages.
///
/// {@tool snippet}
///
/// Below is an example implementation of [performLayout] that causes one widget
/// (the follower) to be the same size as another (the leader):
///
/// ```dart
/// // Define your own slot numbers, depending upon the id assigned by LayoutId.
/// // Typical usage is to define an enum like the one below, and use those
/// // values as the ids.
/// enum _Slot {
///   leader,
///   follower,
/// }
///
/// class FollowTheLeader extends MultiChildLayoutDelegate {
///   @override
///   void performLayout(Size size) {
///     Size leaderSize = Size.zero;
///
///     if (hasChild(_Slot.leader)) {
///       leaderSize = layoutChild(_Slot.leader, BoxConstraints.loose(size));
///       positionChild(_Slot.leader, Offset.zero);
///     }
///
///     if (hasChild(_Slot.follower)) {
///       layoutChild(_Slot.follower, BoxConstraints.tight(leaderSize));
///       positionChild(_Slot.follower, Offset(size.width - leaderSize.width,
///           size.height - leaderSize.height));
///     }
///   }
///
///   @override
///   bool shouldRelayout(MultiChildLayoutDelegate oldDelegate) => false;
/// }
/// ```
/// {@end-tool}
///
/// The delegate gives the leader widget loose constraints, which means the
/// child determines what size to be (subject to fitting within the given size).
/// The delegate then remembers the size of that child and places it in the
/// upper left corner.
///
/// The delegate then gives the follower widget tight constraints, forcing it to
/// match the size of the leader widget. The delegate then places the follower
/// widget in the bottom right corner.
///
/// The leader and follower widget will paint in the order they appear in the
/// child list, regardless of the order in which [layoutChild] is called on
/// them.
///
/// See also:
///
///  * [CustomMultiChildLayout], the widget that uses this delegate.
///  * [RenderCustomMultiChildLayoutBox], render object that uses this
///    delegate.
abstract class MultiChildLayoutDelegate {
  /// Creates a layout delegate.
  ///
  /// The layout will update whenever [relayout] notifies its listeners.
  MultiChildLayoutDelegate({Listenable? relayout}) : _relayout = relayout;

  final Listenable? _relayout;

  Map<Object, RenderBox>? _idToChild;
  Set<RenderBox>? _debugChildrenNeedingLayout;

  /// True if a non-null LayoutChild was provided for the specified id.
  ///
  /// Call this from the [performLayout] method to determine which children
  /// are available, if the child list might vary.
  ///
  /// This method cannot be called from [getSize] as the size is not allowed
  /// to depend on the children.
  bool hasChild(Object childId) => _idToChild![childId] != null;

  /// Ask the child to update its layout within the limits specified by
  /// the constraints parameter. The child's size is returned.
  ///
  /// Call this from your [performLayout] function to lay out each
  /// child. Every child must be laid out using this function exactly
  /// once each time the [performLayout] function is called.
  Size layoutChild(Object childId, BoxConstraints constraints) {
    final RenderBox? child = _idToChild![childId];
    assert(() {
      if (child == null) {
        throw FlutterError(
          'The $this custom multichild layout delegate tried to lay out a non-existent child.\n'
          'There is no child with the id "$childId".',
        );
      }
      if (!_debugChildrenNeedingLayout!.remove(child)) {
        throw FlutterError(
          'The $this custom multichild layout delegate tried to lay out the child with id "$childId" more than once.\n'
          'Each child must be laid out exactly once.',
        );
      }
      try {
        assert(constraints.debugAssertIsValid(isAppliedConstraint: true));
      } on AssertionError catch (exception) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'The $this custom multichild layout delegate provided invalid box constraints for the child with id "$childId".',
          ),
          DiagnosticsProperty<AssertionError>('Exception', exception, showName: false),
          ErrorDescription(
            'The minimum width and height must be greater than or equal to zero.\n'
            'The maximum width must be greater than or equal to the minimum width.\n'
            'The maximum height must be greater than or equal to the minimum height.',
          ),
        ]);
      }
      return true;
    }());
    child!.layout(constraints, parentUsesSize: true);
    return child.size;
  }

  /// Specify the child's origin relative to this origin.
  ///
  /// Call this from your [performLayout] function to position each
  /// child. If you do not call this for a child, its position will
  /// remain unchanged. Children initially have their position set to
  /// (0,0), i.e. the top left of the [RenderCustomMultiChildLayoutBox].
  void positionChild(Object childId, Offset offset) {
    final RenderBox? child = _idToChild![childId];
    assert(() {
      if (child == null) {
        throw FlutterError(
          'The $this custom multichild layout delegate tried to position out a non-existent child:\n'
          'There is no child with the id "$childId".',
        );
      }
      return true;
    }());
    final MultiChildLayoutParentData childParentData =
        child!.parentData! as MultiChildLayoutParentData;
    childParentData.offset = offset;
  }

  DiagnosticsNode _debugDescribeChild(RenderBox child) {
    final MultiChildLayoutParentData childParentData =
        child.parentData! as MultiChildLayoutParentData;
    return DiagnosticsProperty<RenderBox>('${childParentData.id}', child);
  }

  void _callPerformLayout(Size size, RenderBox? firstChild) {
    // A particular layout delegate could be called reentrantly, e.g. if it used
    // by both a parent and a child. So, we must restore the _idToChild map when
    // we return.
    final Map<Object, RenderBox>? previousIdToChild = _idToChild;

    Set<RenderBox>? debugPreviousChildrenNeedingLayout;
    assert(() {
      debugPreviousChildrenNeedingLayout = _debugChildrenNeedingLayout;
      _debugChildrenNeedingLayout = <RenderBox>{};
      return true;
    }());

    try {
      _idToChild = <Object, RenderBox>{};
      RenderBox? child = firstChild;
      while (child != null) {
        final MultiChildLayoutParentData childParentData =
            child.parentData! as MultiChildLayoutParentData;
        assert(() {
          if (childParentData.id == null) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              ErrorSummary(
                'Every child of a RenderCustomMultiChildLayoutBox must have an ID in its parent data.',
              ),
              child!.describeForError('The following child has no ID'),
            ]);
          }
          return true;
        }());
        _idToChild![childParentData.id!] = child;
        assert(() {
          _debugChildrenNeedingLayout!.add(child!);
          return true;
        }());
        child = childParentData.nextSibling;
      }
      performLayout(size);
      assert(() {
        if (_debugChildrenNeedingLayout!.isNotEmpty) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary('Each child must be laid out exactly once.'),
            DiagnosticsBlock(
              name:
                  'The $this custom multichild layout delegate forgot '
                  'to lay out the following '
                  '${_debugChildrenNeedingLayout!.length > 1 ? 'children' : 'child'}',
              properties:
                  _debugChildrenNeedingLayout!.map<DiagnosticsNode>(_debugDescribeChild).toList(),
            ),
          ]);
        }
        return true;
      }());
    } finally {
      _idToChild = previousIdToChild;
      assert(() {
        _debugChildrenNeedingLayout = debugPreviousChildrenNeedingLayout;
        return true;
      }());
    }
  }

  /// Override this method to return the size of this object given the
  /// incoming constraints.
  ///
  /// The size cannot reflect the sizes of the children. If this layout has a
  /// fixed width or height the returned size can reflect that; the size will be
  /// constrained to the given constraints.
  ///
  /// By default, attempts to size the box to the biggest size
  /// possible given the constraints.
  Size getSize(BoxConstraints constraints) => constraints.biggest;

  /// Override this method to lay out and position all children given this
  /// widget's size.
  ///
  /// This method must call [layoutChild] for each child. It should also specify
  /// the final position of each child with [positionChild].
  void performLayout(Size size);

  /// Override this method to return true when the children need to be
  /// laid out.
  ///
  /// This should compare the fields of the current delegate and the given
  /// `oldDelegate` and return true if the fields are such that the layout would
  /// be different.
  bool shouldRelayout(covariant MultiChildLayoutDelegate oldDelegate);

  /// Override this method to include additional information in the
  /// debugging data printed by [debugDumpRenderTree] and friends.
  ///
  /// By default, returns the [runtimeType] of the class.
  @override
  String toString() => objectRuntimeType(this, 'MultiChildLayoutDelegate');
}

/// Defers the layout of multiple children to a delegate.
///
/// The delegate can determine the layout constraints for each child and can
/// decide where to position each child. The delegate can also determine the
/// size of the parent, but the size of the parent cannot depend on the sizes of
/// the children.
class RenderCustomMultiChildLayoutBox extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, MultiChildLayoutParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, MultiChildLayoutParentData> {
  /// Creates a render object that customizes the layout of multiple children.
  RenderCustomMultiChildLayoutBox({
    List<RenderBox>? children,
    required MultiChildLayoutDelegate delegate,
  }) : _delegate = delegate {
    addAll(children);
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! MultiChildLayoutParentData) {
      child.parentData = MultiChildLayoutParentData();
    }
  }

  /// The delegate that controls the layout of the children.
  MultiChildLayoutDelegate get delegate => _delegate;
  MultiChildLayoutDelegate _delegate;
  set delegate(MultiChildLayoutDelegate newDelegate) {
    if (_delegate == newDelegate) {
      return;
    }
    final MultiChildLayoutDelegate oldDelegate = _delegate;
    if (newDelegate.runtimeType != oldDelegate.runtimeType ||
        newDelegate.shouldRelayout(oldDelegate)) {
      markNeedsLayout();
    }
    _delegate = newDelegate;
    if (attached) {
      oldDelegate._relayout?.removeListener(markNeedsLayout);
      newDelegate._relayout?.addListener(markNeedsLayout);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _delegate._relayout?.addListener(markNeedsLayout);
  }

  @override
  void detach() {
    _delegate._relayout?.removeListener(markNeedsLayout);
    super.detach();
  }

  Size _getSize(BoxConstraints constraints) {
    assert(constraints.debugAssertIsValid());
    return constraints.constrain(_delegate.getSize(constraints));
  }

  // TODO(ianh): It's a bit dubious to be using the getSize function from the delegate to
  // figure out the intrinsic dimensions. We really should either not support intrinsics,
  // or we should expose intrinsic delegate callbacks and throw if they're not implemented.

  @override
  double computeMinIntrinsicWidth(double height) {
    final double width = _getSize(BoxConstraints.tightForFinite(height: height)).width;
    if (width.isFinite) {
      return width;
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final double width = _getSize(BoxConstraints.tightForFinite(height: height)).width;
    if (width.isFinite) {
      return width;
    }
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    final double height = _getSize(BoxConstraints.tightForFinite(width: width)).height;
    if (height.isFinite) {
      return height;
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    final double height = _getSize(BoxConstraints.tightForFinite(width: width)).height;
    if (height.isFinite) {
      return height;
    }
    return 0.0;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _getSize(constraints);
  }

  @override
  void performLayout() {
    size = _getSize(constraints);
    delegate._callPerformLayout(size, firstChild);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    defaultPaint(context, offset);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }
}
