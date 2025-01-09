// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'basic.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver_fill.dart';
library;

import 'package:flutter/rendering.dart';

import 'framework.dart';
import 'scroll_delegate.dart';
import 'sliver.dart';

/// A sliver that places its box children in a linear array and constrains them
/// to have the same extent as a prototype item along the main axis.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// [SliverPrototypeExtentList] arranges its children in a line along
/// the main axis starting at offset zero and without gaps. Each child is
/// constrained to the same extent as the [prototypeItem] along the main axis
/// and the [SliverConstraints.crossAxisExtent] along the cross axis.
///
/// [SliverPrototypeExtentList] is more efficient than [SliverList] because
/// [SliverPrototypeExtentList] does not need to lay out its children to obtain
/// their extent along the main axis. It's a little more flexible than
/// [SliverFixedExtentList] because there's no need to determine the appropriate
/// item extent in pixels.
///
/// See also:
///
///  * [SliverFixedExtentList], whose children are forced to a given pixel
///    extent.
///  * [SliverVariedExtentList], which supports children with varying (but known
///    upfront) extents.
///  * [SliverList], which does not require its children to have the same
///    extent in the main axis.
///  * [SliverFillViewport], which sizes its children based on the
///    size of the viewport, regardless of what else is in the scroll view.
class SliverPrototypeExtentList extends SliverMultiBoxAdaptorWidget {
  /// Creates a sliver that places its box children in a linear array and
  /// constrains them to have the same extent as a prototype item along
  /// the main axis.
  const SliverPrototypeExtentList({
    super.key,
    required super.delegate,
    required this.prototypeItem,
  });

  /// A sliver that places its box children in a linear array and constrains them
  /// to have the same extent as a prototype item along the main axis.
  ///
  /// This constructor is appropriate for sliver lists with a large (or
  /// infinite) number of children whose extent is already determined.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverGrid]
  /// to estimate the maximum scroll extent.
  ///
  /// `itemBuilder` will be called only with indices greater than or equal to
  /// zero and less than `itemCount`.
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// The `prototypeItem` argument is used to determine the extent of each item.
  ///
  /// {@macro flutter.widgets.PageView.findChildIndexCallback}
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example, which would be inserted into a [CustomScrollView.slivers]
  /// list, shows an infinite number of items in varying shades of blue:
  ///
  /// ```dart
  /// SliverPrototypeExtentList.builder(
  ///   prototypeItem: Container(
  ///     alignment: Alignment.center,
  ///     child: const Text('list item prototype'),
  ///   ),
  ///   itemBuilder: (BuildContext context, int index) {
  ///     return Container(
  ///       alignment: Alignment.center,
  ///       color: Colors.lightBlue[100 * (index % 9)],
  ///       child: Text('list item $index'),
  ///     );
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  SliverPrototypeExtentList.builder({
    super.key,
    required NullableIndexedWidgetBuilder itemBuilder,
    required this.prototypeItem,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// This constructor uses a list of [Widget]s to build the sliver.
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example, which would be inserted into a [CustomScrollView.slivers]
  /// list, shows an infinite number of items in varying shades of blue:
  ///
  /// ```dart
  /// SliverPrototypeExtentList.list(
  ///   prototypeItem: const Text('Hello'),
  ///   children: const <Widget>[
  ///     Text('Hello'),
  ///     Text('World!'),
  ///   ],
  /// );
  /// ```
  /// {@end-tool}
  SliverPrototypeExtentList.list({
    super.key,
    required List<Widget> children,
    required this.prototypeItem,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildListDelegate(
           children,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// Defines the main axis extent of all of this sliver's children.
  ///
  /// The [prototypeItem] is laid out before the rest of the sliver's children
  /// and its size along the main axis fixes the size of each child. The
  /// [prototypeItem] is essentially [Offstage]: it is not painted and it
  /// cannot respond to input.
  final Widget prototypeItem;

  @override
  RenderSliverMultiBoxAdaptor createRenderObject(BuildContext context) {
    final _SliverPrototypeExtentListElement element = context as _SliverPrototypeExtentListElement;
    return _RenderSliverPrototypeExtentList(childManager: element);
  }

  @override
  SliverMultiBoxAdaptorElement createElement() => _SliverPrototypeExtentListElement(this);
}

class _SliverPrototypeExtentListElement extends SliverMultiBoxAdaptorElement {
  _SliverPrototypeExtentListElement(SliverPrototypeExtentList super.widget);

  @override
  _RenderSliverPrototypeExtentList get renderObject =>
      super.renderObject as _RenderSliverPrototypeExtentList;

  Element? _prototype;
  static final Object _prototypeSlot = Object();

  @override
  void insertRenderObjectChild(covariant RenderObject child, covariant Object slot) {
    if (slot == _prototypeSlot) {
      assert(child is RenderBox);
      renderObject.child = child as RenderBox;
    } else {
      super.insertRenderObjectChild(child, slot as int);
    }
  }

  @override
  void didAdoptChild(RenderBox child) {
    if (child != renderObject.child) {
      super.didAdoptChild(child);
    }
  }

  @override
  void moveRenderObjectChild(RenderBox child, Object oldSlot, Object newSlot) {
    if (newSlot == _prototypeSlot) {
      // There's only one prototype child so it cannot be moved.
      assert(false);
    } else {
      super.moveRenderObjectChild(child, oldSlot as int, newSlot as int);
    }
  }

  @override
  void removeRenderObjectChild(RenderBox child, Object slot) {
    if (renderObject.child == child) {
      renderObject.child = null;
    } else {
      super.removeRenderObjectChild(child, slot as int);
    }
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    if (_prototype != null) {
      visitor(_prototype!);
    }
    super.visitChildren(visitor);
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    _prototype = updateChild(
      _prototype,
      (widget as SliverPrototypeExtentList).prototypeItem,
      _prototypeSlot,
    );
  }

  @override
  void update(SliverPrototypeExtentList newWidget) {
    super.update(newWidget);
    assert(widget == newWidget);
    _prototype = updateChild(
      _prototype,
      (widget as SliverPrototypeExtentList).prototypeItem,
      _prototypeSlot,
    );
  }
}

class _RenderSliverPrototypeExtentList extends RenderSliverFixedExtentBoxAdaptor {
  _RenderSliverPrototypeExtentList({required _SliverPrototypeExtentListElement childManager})
    : super(childManager: childManager);

  RenderBox? _child;
  RenderBox? get child => _child;
  set child(RenderBox? value) {
    if (_child != null) {
      dropChild(_child!);
    }
    _child = value;
    if (_child != null) {
      adoptChild(_child!);
    }
    markNeedsLayout();
  }

  @override
  void performLayout() {
    child!.layout(constraints.asBoxConstraints(), parentUsesSize: true);
    super.performLayout();
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _child?.attach(owner);
  }

  @override
  void detach() {
    super.detach();
    _child?.detach();
  }

  @override
  void redepthChildren() {
    if (_child != null) {
      redepthChild(_child!);
    }
    super.redepthChildren();
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    if (_child != null) {
      visitor(_child!);
    }
    super.visitChildren(visitor);
  }

  @override
  double get itemExtent {
    assert(child != null && child!.hasSize);
    return constraints.axis == Axis.vertical ? child!.size.height : child!.size.width;
  }
}
