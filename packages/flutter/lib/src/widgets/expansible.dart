// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'basic.dart';
import 'framework.dart';
import 'page_storage.dart';
import 'ticker_provider.dart';
import 'transitions.dart';

/// A controller for managing the expansion state of a widget that uses
/// [ExpansibleStateMixin].
///
/// This controller provides methods to programmatically expand or collapse the
/// widget, and it allows external components to query the current expansion
/// state.
///
/// The controller's [expand] and [collapse] methods cause the
/// the expansible widget to rebuild, so they may not be called from
/// a build method.
class ExpansibleController<S extends StatefulWidget> {
  /// Creates a controller to be used with [ExpansibleStateMixin.controller].
  ExpansibleController();

  ExpansibleStateMixin<S>? _state;

  /// Whether the expansible widget built with this controller is in expanded
  /// state.
  ///
  /// This property doesn't take the animation into account. It reports `true`
  /// even if the expansion animation is not completed.
  ///
  /// See also:
  ///
  ///  * [expand], which expands the expansible widget.
  ///  * [collapse], which collapses the expansible widget.
  bool get isExpanded {
    assert(_state != null);
    return _state!.isExpanded;
  }

  /// Expands the expansible widget that was built with this controller.
  ///
  /// If the widget is already in the expanded state (see [isExpanded]), calling
  /// this method has no effect.
  ///
  /// Calling this method may cause the expansible widget to rebuild, so it may
  /// not be called from a build method.
  ///
  /// Calling this method will trigger an
  /// [ExpansibleStateMixin.onExpansionChanged] callback.
  ///
  /// See also:
  ///
  ///  * [collapse], which collapses the expansible widget.
  ///  * [isExpanded] to check whether the expansible widget is expanded.
  void expand() {
    assert(_state != null);
    if (!isExpanded) {
      _state!.toggleExpansion();
    }
  }

  /// Collapses the expansible widget that was built with this controller.
  ///
  /// If the widget is already in the collapsed state (see [isExpanded]),
  /// calling this method has no effect.
  ///
  /// Calling this method may cause the expansible widget to rebuild, so it may
  /// not be called from a build method.
  ///
  /// Calling this method will trigger an
  /// [ExpansibleStateMixin.onExpansionChanged] callback.
  ///
  /// See also:
  ///
  ///  * [expand], which expands the tile.
  ///  * [isExpanded] to check whether the tile is expanded.
  void collapse() {
    assert(_state != null);
    if (isExpanded) {
      _state!.toggleExpansion();
    }
  }
}

/// A mixin for [StatefulWidget]s that provides expanding and collapsing
/// behavior, like [ExpansionTile] for example.
///
/// This mixin manages the expansion state, animation, and layout of a widget
/// that can be expanded or collapsed.
///
/// An expansible widget consists of a header, which is always shown, and a
/// body, which is hidden in the collapsed state and shown in its expanded
/// state.
///
/// The widget is expanded or collapsed with an animation driven by
/// [animationController]. When the widget is expanded, the height of its body
/// animates from 0 to its fully expanded height.
///
/// To use this mixin, implement the [buildHeader] and [buildBody] methods to
/// provide the header and body widgets. The [buildExpansible] method can be
/// overridden to further customize the layout of the widget.
mixin ExpansibleStateMixin<S extends StatefulWidget> on TickerProviderStateMixin<S> {
  /// Whether this widget is currently expanded.
  ///
  /// Defaults to false.
  bool get isExpanded => _isExpanded;
  bool _isExpanded = false;

  /// Used by subclasses to manage the expansion animation.
  AnimationController get animationController => _animationController;
  late AnimationController _animationController;

  /// Sets the height of the expanded body.
  ///
  /// When in the expanded state, the body's height animates from 0 to its
  /// fully extended height.
  CurvedAnimation get heightFactor => _heightFactor;
  late CurvedAnimation _heightFactor;

  /// The list of widgets that are displayed when this is expanded.
  List<Widget> get children;

  /// Called when this widget expands or collapses.
  ///
  /// When the widget starts expanding, this function is called with value
  /// true. When the widget starts collapsing, this function is called with
  /// value false.
  ValueChanged<bool>? get onExpansionChanged;

  /// The duration of the expansion animation.
  Duration get expansionDuration;

  /// The curve of the expansion animation.
  Curve get expansionCurve;

  /// True if the widget is initially expanded, and false otherwise.
  bool get initiallyExpanded;

  /// Specifies whether the state of the children is maintained when the widget
  /// expands and collapses.
  ///
  /// If true, the children are kept in the tree while the widget is
  /// collapsed. Otherwise, the children are removed from the tree when the
  /// widget is collapsed and recreated upon expansion.
  bool get maintainState;

  /// If provided, the controller can be used to programmatically expand and
  /// collapse the widget.
  ExpansibleController<S> get controller => _fallbackController;
  late ExpansibleController<S> _fallbackController;

  @override
  void initState() {
    super.initState();
    _fallbackController = ExpansibleController<S>();
    _animationController = AnimationController(duration: expansionDuration, vsync: this);
    _isExpanded = PageStorage.maybeOf(context)?.readState(context) as bool? ?? initiallyExpanded;
    if (_isExpanded) {
      _animationController.value = 1.0;
    }
    final Tween<double> heightFactorTween = Tween<double>(begin: 0.0, end: 1.0);
    _heightFactor = CurvedAnimation(
      parent: animationController.drive(heightFactorTween),
      curve: expansionCurve,
    );

    assert(controller._state == null);
    controller._state = this;
  }

  @override
  void dispose() {
    controller._state = null;
    _animationController.dispose();
    _heightFactor.dispose();
    super.dispose();
  }

  /// Toggles the expansion state of the widget.
  ///
  /// This method is called when the user taps the header or when the
  /// [controller] is used to programmatically expand or collapse the widget.
  @protected
  @mustCallSuper
  void toggleExpansion() {
    setState(() {
      _isExpanded = !_isExpanded;
      if (_isExpanded) {
        _animationController.forward();
      } else {
        _animationController.reverse().then<void>((void value) {
          if (!mounted) {
            return;
          }
          setState(() {
            // Rebuild without widget.children.
          });
        });
      }
      PageStorage.maybeOf(context)?.writeState(context, _isExpanded);
    });
    onExpansionChanged?.call(_isExpanded);
  }

  /// Lays out the widget with the results of [buildHeader] and [buildBody].
  ///
  /// By default, this method puts the header and body in a [Column], but it can
  /// be overridden to further customize the layout of the header and body.
  @protected
  Widget buildExpansible(BuildContext context, Widget header, Widget body) {
    return Column(mainAxisSize: MainAxisSize.min, children: <Widget>[header, body]);
  }

  /// Builds the always displayed header.
  ///
  /// When the header is tapped, call [toggleExpansion] to trigger the
  /// expansion.
  @protected
  Widget buildHeader(BuildContext context);

  /// Builds the collapsible body.
  ///
  /// The body is composed of the list of [children].
  @protected
  Widget buildBody(BuildContext context);

  @override
  Widget build(BuildContext context) {
    final bool closed = !isExpanded && animationController.isDismissed;
    final bool shouldRemoveChildren = closed && !maintainState;

    final Widget result = Offstage(
      offstage: closed,
      child: TickerMode(enabled: !closed, child: buildBody(context)),
    );

    return AnimatedBuilder(
      animation: animationController.view,
      builder: (BuildContext context, Widget? child) {
        return buildExpansible(
          context,
          buildHeader(context),
          ClipRect(child: Align(heightFactor: heightFactor.value, child: child)),
        );
      },
      child: shouldRemoveChildren ? null : result,
    );
  }
}
