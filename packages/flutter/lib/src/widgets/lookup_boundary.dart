// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'framework.dart';

// Examples can assume:
// class MyWidget extends StatelessWidget { const MyWidget({super.key, required this.child}); final Widget child; @override Widget build(BuildContext context) => child; }

/// A lookup boundary controls what entities are visible to descendants of the
/// boundary via the static lookup methods provided by the boundary.
///
/// The static lookup methods of the boundary mirror the lookup methods by the
/// same name exposed on [BuildContext] and they can be used as direct
/// replacements. Unlike the methods on [BuildContext], these methods do not
/// find any ancestor entities of the closest [LookupBoundary] surrounding the
/// provided [BuildContext]. The root of the tree is an implicit lookup boundary.
///
/// {@tool snippet}
/// In the example below, the [LookupBoundary.findAncestorWidgetOfExactType]
/// call returns null because the [LookupBoundary] "hides" `MyWidget` from the
/// [BuildContext] that was queried.
///
/// ```dart
/// MyWidget(
///   child: LookupBoundary(
///     child: Builder(
///       builder: (BuildContext context) {
///         MyWidget? widget = LookupBoundary.findAncestorWidgetOfExactType<MyWidget>(context);
///         return Text('$widget'); // "null"
///       },
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// A [LookupBoundary] only affects the behavior of the static lookup methods
/// defined on the boundary. It does not affect the behavior of the lookup
/// methods defined on [BuildContext].
///
/// A [LookupBoundary] is rarely instantiated directly. They are inserted at
/// locations of the widget tree where the render tree diverges from the element
/// tree, which is rather uncommon. Such anomalies are created by
/// [RenderObjectElement]s that don't attach their [RenderObject] to the closest
/// ancestor [RenderObjectElement], e.g. because they bootstrap a separate
/// stand-alone render tree. This behavior breaks the assumption some widgets
/// have about the structure of the render tree: These widgets may try to reach
/// out to an ancestor widget, assuming that their associated [RenderObject]s
/// are also ancestors, which due to the anomaly may not be the case. At the
/// point where the divergence in the two trees is introduced, a
/// [LookupBoundary] can be used to hide that ancestor from the querying widget.
/// The [ViewAnchor], for example, wraps its [ViewAnchor.view] child in a
/// [LookupBoundary] because the [RenderObject] produced by that widget subtree
/// is not attached to the render tree that the [ViewAnchor] itself belongs to.
///
/// As an example, [Material.of] relies on lookup boundaries to hide the
/// [Material] widget from certain descendant button widget. Buttons reach out
/// to their [Material] ancestor to draw ink splashes on its associated render
/// object. This only produces the desired effect if the button render object
/// is a descendant of the [Material] render object. If the element tree and
/// the render tree are not in sync due to anomalies described above, this may
/// not be the case. To avoid incorrect visuals, the [Material] relies on
/// lookup boundaries to hide itself from descendants in subtrees with such
/// anomalies. Those subtrees are expected to introduce their own [Material]
/// widget that buttons there can utilize without crossing a lookup boundary.
class LookupBoundary extends InheritedWidget {
  /// Creates a [LookupBoundary].
  ///
  /// A none-null [child] widget must be provided.
  const LookupBoundary({super.key, required super.child});

  /// Obtains the nearest widget of the given type `T` within the current
  /// [LookupBoundary] of `context`, which must be the type of a concrete
  /// [InheritedWidget] subclass, and registers the provided build `context`
  /// with that widget such that when that widget changes (or a new widget of
  /// that type is introduced, or the widget goes away), the build context is
  /// rebuilt so that it can obtain new values from that widget.
  ///
  /// This method behaves exactly like
  /// [BuildContext.dependOnInheritedWidgetOfExactType], except it only
  /// considers [InheritedWidget]s of the specified type `T` between the
  /// provided [BuildContext] and its closest [LookupBoundary] ancestor.
  /// [InheritedWidget]s past that [LookupBoundary] are invisible to this
  /// method. The root of the tree is treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.dependOnInheritedWidgetOfExactType}
  static T? dependOnInheritedWidgetOfExactType<T extends InheritedWidget>(
    BuildContext context, {
    Object? aspect,
  }) {
    // The following call makes sure that context depends on something so
    // Element.didChangeDependencies is called when context moves in the tree
    // even when requested dependency remains unfulfilled (i.e. null is
    // returned).
    context.dependOnInheritedWidgetOfExactType<LookupBoundary>();
    final InheritedElement? candidate = getElementForInheritedWidgetOfExactType<T>(context);
    if (candidate == null) {
      return null;
    }
    context.dependOnInheritedElement(candidate, aspect: aspect);
    return candidate.widget as T;
  }

  /// Obtains the element corresponding to the nearest widget of the given type
  /// `T` within the current [LookupBoundary] of `context`.
  ///
  /// `T` must be the type of a concrete [InheritedWidget] subclass. Returns
  /// null if no such element is found.
  ///
  /// This method behaves exactly like
  /// [BuildContext.getElementForInheritedWidgetOfExactType], except it only
  /// considers [InheritedWidget]s of the specified type `T` between the
  /// provided [BuildContext] and its closest [LookupBoundary] ancestor.
  /// [InheritedWidget]s past that [LookupBoundary] are invisible to this
  /// method. The root of the tree is treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.getElementForInheritedWidgetOfExactType}
  static InheritedElement? getElementForInheritedWidgetOfExactType<T extends InheritedWidget>(
    BuildContext context,
  ) {
    final InheritedElement? candidate = context.getElementForInheritedWidgetOfExactType<T>();
    if (candidate == null) {
      return null;
    }
    final Element? boundary = context.getElementForInheritedWidgetOfExactType<LookupBoundary>();
    if (boundary != null && boundary.depth > candidate.depth) {
      return null;
    }
    return candidate;
  }

  /// Returns the nearest ancestor widget of the given type `T` within the
  /// current [LookupBoundary] of `context`.
  ///
  /// `T` must be the type of a concrete [Widget] subclass.
  ///
  /// This method behaves exactly like
  /// [BuildContext.findAncestorWidgetOfExactType], except it only considers
  /// [Widget]s of the specified type `T` between the provided [BuildContext]
  /// and its closest [LookupBoundary] ancestor. [Widget]s past that
  /// [LookupBoundary] are invisible to this method. The root of the tree is
  /// treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.findAncestorWidgetOfExactType}
  static T? findAncestorWidgetOfExactType<T extends Widget>(BuildContext context) {
    Element? target;
    context.visitAncestorElements((Element ancestor) {
      if (ancestor.widget.runtimeType == T) {
        target = ancestor;
        return false;
      }
      return ancestor.widget.runtimeType != LookupBoundary;
    });
    return target?.widget as T?;
  }

  /// Returns the [State] object of the nearest ancestor [StatefulWidget] widget
  /// within the current [LookupBoundary] of `context` that is an instance of
  /// the given type `T`.
  ///
  /// This method behaves exactly like
  /// [BuildContext.findAncestorWidgetOfExactType], except it only considers
  /// [State] objects of the specified type `T` between the provided
  /// [BuildContext] and its closest [LookupBoundary] ancestor. [State] objects
  /// past that [LookupBoundary] are invisible to this method. The root of the
  /// tree is treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.findAncestorStateOfType}
  static T? findAncestorStateOfType<T extends State>(BuildContext context) {
    StatefulElement? target;
    context.visitAncestorElements((Element ancestor) {
      if (ancestor is StatefulElement && ancestor.state is T) {
        target = ancestor;
        return false;
      }
      return ancestor.widget.runtimeType != LookupBoundary;
    });
    return target?.state as T?;
  }

  /// Returns the [State] object of the furthest ancestor [StatefulWidget]
  /// widget within the current [LookupBoundary] of `context` that is an
  /// instance of the given type `T`.
  ///
  /// This method behaves exactly like
  /// [BuildContext.findRootAncestorStateOfType], except it considers the
  /// closest [LookupBoundary] ancestor of `context` to be the root. [State]
  /// objects past that [LookupBoundary] are invisible to this method. The root
  /// of the tree is treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.findRootAncestorStateOfType}
  static T? findRootAncestorStateOfType<T extends State>(BuildContext context) {
    StatefulElement? target;
    context.visitAncestorElements((Element ancestor) {
      if (ancestor is StatefulElement && ancestor.state is T) {
        target = ancestor;
      }
      return ancestor.widget.runtimeType != LookupBoundary;
    });
    return target?.state as T?;
  }

  /// Returns the [RenderObject] object of the nearest ancestor
  /// [RenderObjectWidget] widget within the current [LookupBoundary] of
  /// `context` that is an instance of the given type `T`.
  ///
  /// This method behaves exactly like
  /// [BuildContext.findAncestorRenderObjectOfType], except it only considers
  /// [RenderObject]s of the specified type `T` between the provided
  /// [BuildContext] and its closest [LookupBoundary] ancestor. [RenderObject]s
  /// past that [LookupBoundary] are invisible to this method. The root of the
  /// tree is treated as an implicit lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.findAncestorRenderObjectOfType}
  static T? findAncestorRenderObjectOfType<T extends RenderObject>(BuildContext context) {
    Element? target;
    context.visitAncestorElements((Element ancestor) {
      if (ancestor is RenderObjectElement && ancestor.renderObject is T) {
        target = ancestor;
        return false;
      }
      return ancestor.widget.runtimeType != LookupBoundary;
    });
    return target?.renderObject as T?;
  }

  /// Walks the ancestor chain, starting with the parent of the build context's
  /// widget, invoking the argument for each ancestor until a [LookupBoundary]
  /// or the root is reached.
  ///
  /// This method behaves exactly like [BuildContext.visitAncestorElements],
  /// except it only walks the tree up to the closest [LookupBoundary] ancestor
  /// of the provided context. The root of the tree is treated as an implicit
  /// lookup boundary.
  ///
  /// {@macro flutter.widgets.BuildContext.visitAncestorElements}
  static void visitAncestorElements(BuildContext context, ConditionalElementVisitor visitor) {
    context.visitAncestorElements((Element ancestor) {
      return visitor(ancestor) && ancestor.widget.runtimeType != LookupBoundary;
    });
  }

  /// Walks the non-[LookupBoundary] child [Element]s of the provided
  /// `context`.
  ///
  /// This method behaves exactly like [BuildContext.visitChildElements],
  /// except it only visits children that are not a [LookupBoundary].
  ///
  /// {@macro flutter.widgets.BuildContext.visitChildElements}
  static void visitChildElements(BuildContext context, ElementVisitor visitor) {
    context.visitChildElements((Element child) {
      if (child.widget.runtimeType != LookupBoundary) {
        visitor(child);
      }
    });
  }

  /// Returns true if a [LookupBoundary] is hiding the nearest
  /// [Widget] of the specified type `T` from the provided [BuildContext].
  ///
  /// This method throws when asserts are disabled.
  static bool debugIsHidingAncestorWidgetOfExactType<T extends Widget>(BuildContext context) {
    bool? result;
    assert(() {
      bool hiddenByBoundary = false;
      bool ancestorFound = false;
      context.visitAncestorElements((Element ancestor) {
        if (ancestor.widget.runtimeType == T) {
          ancestorFound = true;
          return false;
        }
        hiddenByBoundary = hiddenByBoundary || ancestor.widget.runtimeType == LookupBoundary;
        return true;
      });
      result = ancestorFound & hiddenByBoundary;
      return true;
    }());
    return result!;
  }

  /// Returns true if a [LookupBoundary] is hiding the nearest [StatefulWidget]
  /// with a [State] of the specified type `T` from the provided [BuildContext].
  ///
  /// This method throws when asserts are disabled.
  static bool debugIsHidingAncestorStateOfType<T extends State>(BuildContext context) {
    bool? result;
    assert(() {
      bool hiddenByBoundary = false;
      bool ancestorFound = false;
      context.visitAncestorElements((Element ancestor) {
        if (ancestor is StatefulElement && ancestor.state is T) {
          ancestorFound = true;
          return false;
        }
        hiddenByBoundary = hiddenByBoundary || ancestor.widget.runtimeType == LookupBoundary;
        return true;
      });
      result = ancestorFound & hiddenByBoundary;
      return true;
    }());
    return result!;
  }

  /// Returns true if a [LookupBoundary] is hiding the nearest
  /// [RenderObjectWidget] with a [RenderObject] of the specified type `T`
  /// from the provided [BuildContext].
  ///
  /// This method throws when asserts are disabled.
  static bool debugIsHidingAncestorRenderObjectOfType<T extends RenderObject>(
    BuildContext context,
  ) {
    bool? result;
    assert(() {
      bool hiddenByBoundary = false;
      bool ancestorFound = false;
      context.visitAncestorElements((Element ancestor) {
        if (ancestor is RenderObjectElement && ancestor.renderObject is T) {
          ancestorFound = true;
          return false;
        }
        hiddenByBoundary = hiddenByBoundary || ancestor.widget.runtimeType == LookupBoundary;
        return true;
      });
      result = ancestorFound & hiddenByBoundary;
      return true;
    }());
    return result!;
  }

  @override
  bool updateShouldNotify(covariant InheritedWidget oldWidget) => false;
}
