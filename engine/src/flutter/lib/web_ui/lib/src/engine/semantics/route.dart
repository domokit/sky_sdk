// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import '../semantics.dart';
import '../util.dart';

/// Denotes that all descendant nodes are inside a route.
///
/// Routes can include dialogs, pop-up menus, sub-screens, and more.
///
/// See also:
///
///   * [RouteName], which provides a description for this route in the absense
///     of an explicit route label set on the route itself.
class SemanticRoute extends SemanticRole {
  SemanticRoute(SemanticsObject semanticsObject)
    : super.blank(SemanticRoleKind.route, semanticsObject) {
    // The following behaviors can coexist with the route. Generic `RouteName`
    // and `LabelAndValue` are not used by this role because when the route
    // names its own route an `aria-label` is used instead of
    // `aria-describedby`.
    addFocusManagement();
    addLiveRegion();

    // When a route is pushed it is expected that the screen reader will focus
    // on something inside it. There could be two possibilities:
    //
    // 1. The framework explicitly marked a node inside the route as focused
    //    via the `isFocusable` and `isFocused` flags. In this case, the node
    //    will request focus directly and there's nothing to do on top of that.
    // 2. No node inside the route takes focus explicitly. In this case, the
    //    expectation is to look through all nodes in traversal order and focus
    //    on the first one.
    semanticsObject.owner.addOneTimePostUpdateCallback(() {
      if (semanticsObject.owner.hasNodeRequestingFocus) {
        // Case 1: a node requested explicit focus. Nothing extra to do.
        return;
      }

      // Case 2: nothing requested explicit focus. Focus on the first descendant.
      _setDefaultFocus();
    });

    // Lacking any more specific information, ARIA role "dialog" is the
    // closest thing to Flutter's route. This can be revisited if better
    // options become available, especially if the framework volunteers more
    // specific information about the route. Other attributes in the vicinity
    // of routes include: "alertdialog", `aria-modal`, "menu", "tooltip".
    setAriaRole('dialog');
  }

  void _setDefaultFocus() {
    semanticsObject.visitDepthFirstInTraversalOrder((SemanticsObject node) {
      final SemanticRole? role = node.semanticRole;
      if (role == null) {
        return true;
      }

      // If the node does not take focus (e.g. focusing on it does not make
      // sense at all). Despair not. Keep looking.
      final bool didTakeFocus = role.focusAsRouteDefault();
      return !didTakeFocus;
    });
  }

  @override
  void update() {
    super.update();

    // If semantic object corresponding to the route also provides the label for
    // itself it is applied as `aria-label`. See also [describeBy].
    if (semanticsObject.namesRoute) {
      final String? label = semanticsObject.label;
      assert(() {
        if (label == null || label.trim().isEmpty) {
          printWarning(
            'Semantic node ${semanticsObject.id} had both scopesRoute and '
            'namesRoute set, indicating a self-labelled route, but it is '
            'missing the label. A route should be labelled either by setting '
            'namesRoute on itself and providing a label, or by containing a '
            'child node with namesRoute that can describe it with its content.',
          );
        }
        return true;
      }());

      setAttribute('aria-label', label ?? '');
    }
  }

  /// Sets the description of this route based on a [RouteName] descendant
  /// node, unless the route provides its own label.
  void describeBy(RouteName routeName) {
    if (semanticsObject.namesRoute) {
      // The route provides its own label, which takes precedence.
      return;
    }

    setAttribute('aria-describedby', routeName.semanticsObject.element.id);
  }

  @override
  bool focusAsRouteDefault() {
    // Routes are the ones that look inside themselves to find elements to
    // focus on. It doesn't make sense to focus on the route itself.
    return false;
  }
}

/// Supplies a description for the nearest ancestor [SemanticRoute].
///
/// This role is assigned to nodes that have `namesRoute` set but not
/// `scopesRoute`. When both flags are set the node only gets the
/// [SemanticRoute] role.
///
/// If the ancestor route is missing, this role has no effect. It is up to the
/// framework, widget, and app authors to make sure a route name is scoped
/// under a route.
class RouteName extends SemanticBehavior {
  RouteName(super.semanticsObject, super.owner);

  SemanticRoute? _route;

  @override
  void update() {
    // NOTE(yjbanov): this does not handle the case when the node structure
    // changes such that this RouteName is no longer attached to the same route.
    // While this is technically expressible using the semantics API, after
    // discussing this case with customers I decided that this case is not
    // interesting enough to support. A tree restructure like this is likely to
    // confuse screen readers, and it would add complexity to the engine's
    // semantics code. Since reparenting can be done with no update to either
    // the SemanticRoute or RouteName we'd have to scan intermediate nodes
    // for structural changes.
    if (!semanticsObject.namesRoute) {
      return;
    }

    if (semanticsObject.isLabelDirty) {
      final SemanticRoute? route = _route;
      if (route != null) {
        // Already attached to a route, just update the description.
        route.describeBy(this);
      } else {
        // Setting the label for the first time. Wait for the DOM tree to be
        // established, then find the nearest route and update its label.
        semanticsObject.owner.addOneTimePostUpdateCallback(() {
          if (!isDisposed) {
            _lookUpNearestAncestorRoute();
            _route?.describeBy(this);
          }
        });
      }
    }
  }

  void _lookUpNearestAncestorRoute() {
    SemanticsObject? parent = semanticsObject.parent;
    while (parent != null && parent.semanticRole?.kind != SemanticRoleKind.route) {
      parent = parent.parent;
    }
    if (parent != null && parent.semanticRole?.kind == SemanticRoleKind.route) {
      _route = parent.semanticRole! as SemanticRoute;
    }
  }
}
