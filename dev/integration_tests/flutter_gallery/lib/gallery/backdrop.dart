// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';

const double _kFrontHeadingHeight = 32.0; // front layer beveled rectangle
const double _kFrontClosedHeight = 92.0; // front layer height when closed
const double _kBackAppBarHeight = 56.0; // back layer (options) appbar height

// The size of the front layer heading's left and right beveled corners.
final Animatable<BorderRadius?> _kFrontHeadingBevelRadius = BorderRadiusTween(
  begin: const BorderRadius.only(topLeft: Radius.circular(12.0), topRight: Radius.circular(12.0)),
  end: const BorderRadius.only(
    topLeft: Radius.circular(_kFrontHeadingHeight),
    topRight: Radius.circular(_kFrontHeadingHeight),
  ),
);

class _TappableWhileStatusIs extends StatefulWidget {
  const _TappableWhileStatusIs(this.status, {this.controller, this.child});

  final AnimationController? controller;
  final AnimationStatus status;
  final Widget? child;

  @override
  _TappableWhileStatusIsState createState() => _TappableWhileStatusIsState();
}

class _TappableWhileStatusIsState extends State<_TappableWhileStatusIs> {
  bool? _active;

  @override
  void initState() {
    super.initState();
    widget.controller!.addStatusListener(_handleStatusChange);
    _active = widget.controller!.status == widget.status;
  }

  @override
  void dispose() {
    widget.controller!.removeStatusListener(_handleStatusChange);
    super.dispose();
  }

  void _handleStatusChange(AnimationStatus status) {
    final bool value = widget.controller!.status == widget.status;
    if (_active != value) {
      setState(() {
        _active = value;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    Widget child = AbsorbPointer(absorbing: !_active!, child: widget.child);

    if (!_active!) {
      child = FocusScope(
        canRequestFocus: false,
        debugLabel: '$_TappableWhileStatusIs',
        child: child,
      );
    }
    return child;
  }
}

class _CrossFadeTransition extends AnimatedWidget {
  const _CrossFadeTransition({
    this.alignment = Alignment.center,
    required Animation<double> progress,
    this.child0,
    this.child1,
  }) : super(listenable: progress);

  final AlignmentGeometry alignment;
  final Widget? child0;
  final Widget? child1;

  @override
  Widget build(BuildContext context) {
    final Animation<double> progress = listenable as Animation<double>;

    final double opacity1 =
        CurvedAnimation(parent: ReverseAnimation(progress), curve: const Interval(0.5, 1.0)).value;

    final double opacity2 =
        CurvedAnimation(parent: progress, curve: const Interval(0.5, 1.0)).value;

    return Stack(
      alignment: alignment,
      children: <Widget>[
        Opacity(
          opacity: opacity1,
          child: Semantics(scopesRoute: true, explicitChildNodes: true, child: child1),
        ),
        Opacity(
          opacity: opacity2,
          child: Semantics(scopesRoute: true, explicitChildNodes: true, child: child0),
        ),
      ],
    );
  }
}

class _BackAppBar extends StatelessWidget {
  const _BackAppBar({
    this.leading = const SizedBox(width: 56.0),
    required this.title,
    this.trailing,
  });

  final Widget leading;
  final Widget title;
  final Widget? trailing;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return IconTheme.merge(
      data: theme.primaryIconTheme,
      child: DefaultTextStyle(
        style: theme.primaryTextTheme.titleLarge!,
        child: SizedBox(
          height: _kBackAppBarHeight,
          child: Row(
            children: <Widget>[
              Container(alignment: Alignment.center, width: 56.0, child: leading),
              Expanded(child: title),
              if (trailing != null)
                Container(alignment: Alignment.center, width: 56.0, child: trailing),
            ],
          ),
        ),
      ),
    );
  }
}

class Backdrop extends StatefulWidget {
  const Backdrop({
    super.key,
    this.frontAction,
    this.frontTitle,
    this.frontHeading,
    this.frontLayer,
    this.backTitle,
    this.backLayer,
  });

  final Widget? frontAction;
  final Widget? frontTitle;
  final Widget? frontLayer;
  final Widget? frontHeading;
  final Widget? backTitle;
  final Widget? backLayer;

  @override
  State<Backdrop> createState() => _BackdropState();
}

class _BackdropState extends State<Backdrop> with SingleTickerProviderStateMixin {
  final GlobalKey _backdropKey = GlobalKey(debugLabel: 'Backdrop');
  AnimationController? _controller;
  late Animation<double> _frontOpacity;

  static final Animatable<double> _frontOpacityTween = Tween<double>(
    begin: 0.2,
    end: 1.0,
  ).chain(CurveTween(curve: const Interval(0.0, 0.4, curve: Curves.easeInOut)));

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: 300),
      value: 1.0,
      vsync: this,
    );
    _controller!.addStatusListener((AnimationStatus status) {
      setState(() {
        // This is intentionally left empty. The state change itself takes
        // place inside the AnimationController, so there's nothing to update.
        // All we want is for the widget to rebuild and read the new animation
        // state from the AnimationController.
      });
    });
    _frontOpacity = _controller!.drive(_frontOpacityTween);
  }

  @override
  void dispose() {
    _controller!.dispose();
    super.dispose();
  }

  double get _backdropHeight {
    // Warning: this can be safely called from the event handlers but it may
    // not be called at build time.
    final RenderBox renderBox = _backdropKey.currentContext!.findRenderObject()! as RenderBox;
    return math.max(0.0, renderBox.size.height - _kBackAppBarHeight - _kFrontClosedHeight);
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    _controller!.value -= details.primaryDelta! / _backdropHeight;
  }

  void _handleDragEnd(DragEndDetails details) {
    if (!_controller!.isDismissed) {
      return;
    }

    final double flingVelocity = details.velocity.pixelsPerSecond.dy / _backdropHeight;
    _controller!.fling(
      velocity: switch (flingVelocity) {
        < 0.0 => math.max(2.0, -flingVelocity),
        > 0.0 => math.min(-2.0, -flingVelocity),
        _ => _controller!.value < 0.5 ? -2.0 : 2.0,
      },
    );
  }

  void _toggleFrontLayer() {
    _controller!.fling(velocity: _controller!.isForwardOrCompleted ? -2.0 : 2.0);
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    final Animation<RelativeRect> frontRelativeRect = _controller!.drive(
      RelativeRectTween(
        begin: RelativeRect.fromLTRB(
          0.0,
          constraints.biggest.height - _kFrontClosedHeight,
          0.0,
          0.0,
        ),
        end: const RelativeRect.fromLTRB(0.0, _kBackAppBarHeight, 0.0, 0.0),
      ),
    );
    return Stack(
      key: _backdropKey,
      children: <Widget>[
        // Back layer
        Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            _BackAppBar(
              leading: widget.frontAction!,
              title: _CrossFadeTransition(
                progress: _controller!,
                alignment: AlignmentDirectional.centerStart,
                child0: Semantics(namesRoute: true, child: widget.frontTitle),
                child1: Semantics(namesRoute: true, child: widget.backTitle),
              ),
              trailing: IconButton(
                onPressed: _toggleFrontLayer,
                tooltip: 'Toggle options page',
                icon: AnimatedIcon(icon: AnimatedIcons.close_menu, progress: _controller!),
              ),
            ),
            Expanded(
              child: _TappableWhileStatusIs(
                AnimationStatus.dismissed,
                controller: _controller,
                child: Visibility(
                  visible: !_controller!.isCompleted,
                  maintainState: true,
                  child: widget.backLayer!,
                ),
              ),
            ),
          ],
        ),
        // Front layer
        PositionedTransition(
          rect: frontRelativeRect,
          child: AnimatedBuilder(
            animation: _controller!,
            builder: (BuildContext context, Widget? child) {
              return PhysicalShape(
                elevation: 12.0,
                color: Theme.of(context).canvasColor,
                clipper: ShapeBorderClipper(
                  shape: BeveledRectangleBorder(
                    borderRadius: _kFrontHeadingBevelRadius.transform(_controller!.value)!,
                  ),
                ),
                clipBehavior: Clip.antiAlias,
                child: child,
              );
            },
            child: _TappableWhileStatusIs(
              AnimationStatus.completed,
              controller: _controller,
              child: FadeTransition(opacity: _frontOpacity, child: widget.frontLayer),
            ),
          ),
        ),
        // The front "heading" is a (typically transparent) widget that's stacked on
        // top of, and at the top of, the front layer. It adds support for dragging
        // the front layer up and down and for opening and closing the front layer
        // with a tap. It may obscure part of the front layer's topmost child.
        if (widget.frontHeading != null)
          PositionedTransition(
            rect: frontRelativeRect,
            child: ExcludeSemantics(
              child: Container(
                alignment: Alignment.topLeft,
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTap: _toggleFrontLayer,
                  onVerticalDragUpdate: _handleDragUpdate,
                  onVerticalDragEnd: _handleDragEnd,
                  child: widget.frontHeading,
                ),
              ),
            ),
          ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: _buildStack);
  }
}
