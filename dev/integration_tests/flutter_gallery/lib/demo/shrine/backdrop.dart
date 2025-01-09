// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'login.dart';

const Cubic _kAccelerateCurve = Cubic(0.548, 0.0, 0.757, 0.464);
const Cubic _kDecelerateCurve = Cubic(0.23, 0.94, 0.41, 1.0);
const double _kPeakVelocityTime = 0.248210;
const double _kPeakVelocityProgress = 0.379146;

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

class _FrontLayer extends StatelessWidget {
  const _FrontLayer({this.onTap, this.child});

  final VoidCallback? onTap;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    return Material(
      elevation: 16.0,
      shape: const BeveledRectangleBorder(
        borderRadius: BorderRadius.only(topLeft: Radius.circular(46.0)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: onTap,
            child: Container(height: 40.0, alignment: AlignmentDirectional.centerStart),
          ),
          Expanded(child: child!),
        ],
      ),
    );
  }
}

class _BackdropTitle extends AnimatedWidget {
  const _BackdropTitle({
    required Animation<double> super.listenable,
    this.onPress,
    required this.frontTitle,
    required this.backTitle,
  });

  final void Function()? onPress;
  final Widget frontTitle;
  final Widget backTitle;

  @override
  Widget build(BuildContext context) {
    final Animation<double> animation = CurvedAnimation(
      parent: listenable as Animation<double>,
      curve: const Interval(0.0, 0.78),
    );

    return DefaultTextStyle(
      style: Theme.of(context).primaryTextTheme.titleLarge!,
      softWrap: false,
      overflow: TextOverflow.ellipsis,
      child: Row(
        children: <Widget>[
          // branded icon
          SizedBox(
            width: 72.0,
            child: IconButton(
              padding: const EdgeInsets.only(right: 8.0),
              onPressed: onPress,
              icon: Stack(
                children: <Widget>[
                  Opacity(
                    opacity: animation.value,
                    child: const ImageIcon(AssetImage('packages/shrine_images/slanted_menu.png')),
                  ),
                  FractionalTranslation(
                    translation: Tween<Offset>(
                      begin: Offset.zero,
                      end: const Offset(1.0, 0.0),
                    ).evaluate(animation),
                    child: const ImageIcon(AssetImage('packages/shrine_images/diamond.png')),
                  ),
                ],
              ),
            ),
          ),
          // Here, we do a custom cross fade between backTitle and frontTitle.
          // This makes a smooth animation between the two texts.
          Stack(
            children: <Widget>[
              Opacity(
                opacity:
                    CurvedAnimation(
                      parent: ReverseAnimation(animation),
                      curve: const Interval(0.5, 1.0),
                    ).value,
                child: FractionalTranslation(
                  translation: Tween<Offset>(
                    begin: Offset.zero,
                    end: const Offset(0.5, 0.0),
                  ).evaluate(animation),
                  child: backTitle,
                ),
              ),
              Opacity(
                opacity: CurvedAnimation(parent: animation, curve: const Interval(0.5, 1.0)).value,
                child: FractionalTranslation(
                  translation: Tween<Offset>(
                    begin: const Offset(-0.25, 0.0),
                    end: Offset.zero,
                  ).evaluate(animation),
                  child: frontTitle,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

/// Builds a Backdrop.
///
/// A Backdrop widget has two layers, front and back. The front layer is shown
/// by default, and slides down to show the back layer, from which a user
/// can make a selection. The user can also configure the titles for when the
/// front or back layer is showing.
class Backdrop extends StatefulWidget {
  const Backdrop({
    super.key,
    required this.frontLayer,
    required this.backLayer,
    required this.frontTitle,
    required this.backTitle,
    required this.controller,
  });

  final Widget frontLayer;
  final Widget backLayer;
  final Widget frontTitle;
  final Widget backTitle;
  final AnimationController controller;

  @override
  State<Backdrop> createState() => _BackdropState();
}

class _BackdropState extends State<Backdrop> with SingleTickerProviderStateMixin {
  final GlobalKey _backdropKey = GlobalKey(debugLabel: 'Backdrop');
  AnimationController? _controller;
  late Animation<RelativeRect> _layerAnimation;

  @override
  void initState() {
    super.initState();
    _controller = widget.controller;
  }

  @override
  void dispose() {
    _controller!.dispose();
    super.dispose();
  }

  bool get _frontLayerVisible {
    final AnimationStatus status = _controller!.status;
    return status == AnimationStatus.completed || status == AnimationStatus.forward;
  }

  void _toggleBackdropLayerVisibility() {
    // Call setState here to update layerAnimation if that's necessary
    setState(() {
      _frontLayerVisible ? _controller!.reverse() : _controller!.forward();
    });
  }

  // _layerAnimation animates the front layer between open and close.
  // _getLayerAnimation adjusts the values in the TweenSequence so the
  // curve and timing are correct in both directions.
  Animation<RelativeRect> _getLayerAnimation(Size layerSize, double layerTop) {
    Curve firstCurve; // Curve for first TweenSequenceItem
    Curve secondCurve; // Curve for second TweenSequenceItem
    double firstWeight; // Weight of first TweenSequenceItem
    double secondWeight; // Weight of second TweenSequenceItem
    Animation<double> animation; // Animation on which TweenSequence runs

    if (_frontLayerVisible) {
      firstCurve = _kAccelerateCurve;
      secondCurve = _kDecelerateCurve;
      firstWeight = _kPeakVelocityTime;
      secondWeight = 1.0 - _kPeakVelocityTime;
      animation = CurvedAnimation(parent: _controller!.view, curve: const Interval(0.0, 0.78));
    } else {
      // These values are only used when the controller runs from t=1.0 to t=0.0
      firstCurve = _kDecelerateCurve.flipped;
      secondCurve = _kAccelerateCurve.flipped;
      firstWeight = 1.0 - _kPeakVelocityTime;
      secondWeight = _kPeakVelocityTime;
      animation = _controller!.view;
    }

    return TweenSequence<RelativeRect>(<TweenSequenceItem<RelativeRect>>[
      TweenSequenceItem<RelativeRect>(
        tween: RelativeRectTween(
          begin: RelativeRect.fromLTRB(0.0, layerTop, 0.0, layerTop - layerSize.height),
          end: RelativeRect.fromLTRB(
            0.0,
            layerTop * _kPeakVelocityProgress,
            0.0,
            (layerTop - layerSize.height) * _kPeakVelocityProgress,
          ),
        ).chain(CurveTween(curve: firstCurve)),
        weight: firstWeight,
      ),
      TweenSequenceItem<RelativeRect>(
        tween: RelativeRectTween(
          begin: RelativeRect.fromLTRB(
            0.0,
            layerTop * _kPeakVelocityProgress,
            0.0,
            (layerTop - layerSize.height) * _kPeakVelocityProgress,
          ),
          end: RelativeRect.fill,
        ).chain(CurveTween(curve: secondCurve)),
        weight: secondWeight,
      ),
    ]).animate(animation);
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    const double layerTitleHeight = 48.0;
    final Size layerSize = constraints.biggest;
    final double layerTop = layerSize.height - layerTitleHeight;

    _layerAnimation = _getLayerAnimation(layerSize, layerTop);

    return Stack(
      key: _backdropKey,
      children: <Widget>[
        _TappableWhileStatusIs(
          AnimationStatus.dismissed,
          controller: _controller,
          child: widget.backLayer,
        ),
        PositionedTransition(
          rect: _layerAnimation,
          child: _FrontLayer(
            onTap: _toggleBackdropLayerVisibility,
            child: _TappableWhileStatusIs(
              AnimationStatus.completed,
              controller: _controller,
              child: widget.frontLayer,
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    final AppBar appBar = AppBar(
      elevation: 0.0,
      titleSpacing: 0.0,
      title: _BackdropTitle(
        listenable: _controller!.view,
        onPress: _toggleBackdropLayerVisibility,
        frontTitle: widget.frontTitle,
        backTitle: widget.backTitle,
      ),
      actions: <Widget>[
        IconButton(
          icon: const Icon(Icons.search, semanticLabel: 'login'),
          onPressed: () {
            Navigator.push<void>(
              context,
              MaterialPageRoute<void>(builder: (BuildContext context) => const LoginPage()),
            );
          },
        ),
        IconButton(
          icon: const Icon(Icons.tune, semanticLabel: 'login'),
          onPressed: () {
            Navigator.push<void>(
              context,
              MaterialPageRoute<void>(builder: (BuildContext context) => const LoginPage()),
            );
          },
        ),
      ],
    );
    return Scaffold(appBar: appBar, body: LayoutBuilder(builder: _buildStack));
  }
}
