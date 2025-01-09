// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../../gallery_localizations.dart';
import 'category_menu_page.dart';
import 'page_status.dart';

const Cubic _accelerateCurve = Cubic(0.548, 0, 0.757, 0.464);
const Cubic _decelerateCurve = Cubic(0.23, 0.94, 0.41, 1);
const double _peakVelocityTime = 0.248210;
const double _peakVelocityProgress = 0.379146;

class _FrontLayer extends StatelessWidget {
  const _FrontLayer({this.onTap, required this.child});

  final VoidCallback? onTap;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    // An area at the top of the product page.
    // When the menu page is shown, tapping this area will close the menu
    // page and reveal the product page.
    final Widget pageTopArea = Container(height: 40, alignment: AlignmentDirectional.centerStart);

    return Material(
      elevation: 16,
      shape: const BeveledRectangleBorder(
        borderRadius: BorderRadiusDirectional.only(topStart: Radius.circular(46)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          if (onTap != null)
            MouseRegion(
              cursor: SystemMouseCursors.click,
              child: GestureDetector(
                behavior: HitTestBehavior.opaque,
                excludeFromSemantics:
                    true, // Because there is already a "Close Menu" button on screen.
                onTap: onTap,
                child: pageTopArea,
              ),
            )
          else
            pageTopArea,
          Expanded(child: child),
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
      curve: const Interval(0, 0.78),
    );

    final int textDirectionScalar = Directionality.of(context) == TextDirection.ltr ? 1 : -1;

    const ImageIcon slantedMenuIcon = ImageIcon(
      AssetImage('packages/shrine_images/slanted_menu.png'),
    );

    final Widget directionalSlantedMenuIcon =
        Directionality.of(context) == TextDirection.ltr
            ? slantedMenuIcon
            : Transform(
              alignment: Alignment.center,
              transform: Matrix4.rotationY(pi),
              child: slantedMenuIcon,
            );

    final String? menuButtonTooltip =
        animation.isCompleted
            ? GalleryLocalizations.of(context)!.shrineTooltipOpenMenu
            : animation.isDismissed
            ? GalleryLocalizations.of(context)!.shrineTooltipCloseMenu
            : null;

    return DefaultTextStyle(
      style: Theme.of(context).primaryTextTheme.titleLarge!,
      softWrap: false,
      overflow: TextOverflow.ellipsis,
      child: Row(
        children: <Widget>[
          // branded icon
          SizedBox(
            width: 72,
            child: Semantics(
              container: true,
              child: IconButton(
                padding: const EdgeInsetsDirectional.only(end: 8),
                onPressed: onPress,
                tooltip: menuButtonTooltip,
                icon: Stack(
                  children: <Widget>[
                    Opacity(opacity: animation.value, child: directionalSlantedMenuIcon),
                    FractionalTranslation(
                      translation: Tween<Offset>(
                        begin: Offset.zero,
                        end: Offset(1.0 * textDirectionScalar, 0.0),
                      ).evaluate(animation),
                      child: const ImageIcon(AssetImage('packages/shrine_images/diamond.png')),
                    ),
                  ],
                ),
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
                      curve: const Interval(0.5, 1),
                    ).value,
                child: FractionalTranslation(
                  translation: Tween<Offset>(
                    begin: Offset.zero,
                    end: Offset(0.5 * textDirectionScalar, 0),
                  ).evaluate(animation),
                  child: backTitle,
                ),
              ),
              Opacity(
                opacity: CurvedAnimation(parent: animation, curve: const Interval(0.5, 1)).value,
                child: FractionalTranslation(
                  translation: Tween<Offset>(
                    begin: Offset(-0.25 * textDirectionScalar, 0),
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
  late AnimationController _controller;
  late Animation<RelativeRect> _layerAnimation;

  @override
  void initState() {
    super.initState();
    _controller = widget.controller;
  }

  bool get _frontLayerVisible {
    final AnimationStatus status = _controller.status;
    return status == AnimationStatus.completed || status == AnimationStatus.forward;
  }

  void _toggleBackdropLayerVisibility() {
    // Call setState here to update layerAnimation if that's necessary
    setState(() {
      _frontLayerVisible ? _controller.reverse() : _controller.forward();
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
      firstCurve = _accelerateCurve;
      secondCurve = _decelerateCurve;
      firstWeight = _peakVelocityTime;
      secondWeight = 1 - _peakVelocityTime;
      animation = CurvedAnimation(parent: _controller.view, curve: const Interval(0, 0.78));
    } else {
      // These values are only used when the controller runs from t=1.0 to t=0.0
      firstCurve = _decelerateCurve.flipped;
      secondCurve = _accelerateCurve.flipped;
      firstWeight = 1 - _peakVelocityTime;
      secondWeight = _peakVelocityTime;
      animation = _controller.view;
    }

    return TweenSequence<RelativeRect>(<TweenSequenceItem<RelativeRect>>[
      TweenSequenceItem<RelativeRect>(
        tween: RelativeRectTween(
          begin: RelativeRect.fromLTRB(0, layerTop, 0, layerTop - layerSize.height),
          end: RelativeRect.fromLTRB(
            0,
            layerTop * _peakVelocityProgress,
            0,
            (layerTop - layerSize.height) * _peakVelocityProgress,
          ),
        ).chain(CurveTween(curve: firstCurve)),
        weight: firstWeight,
      ),
      TweenSequenceItem<RelativeRect>(
        tween: RelativeRectTween(
          begin: RelativeRect.fromLTRB(
            0,
            layerTop * _peakVelocityProgress,
            0,
            (layerTop - layerSize.height) * _peakVelocityProgress,
          ),
          end: RelativeRect.fill,
        ).chain(CurveTween(curve: secondCurve)),
        weight: secondWeight,
      ),
    ]).animate(animation);
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    const int layerTitleHeight = 48;
    final Size layerSize = constraints.biggest;
    final double layerTop = layerSize.height - layerTitleHeight;

    _layerAnimation = _getLayerAnimation(layerSize, layerTop);

    return Stack(
      key: _backdropKey,
      children: <Widget>[
        ExcludeSemantics(excluding: _frontLayerVisible, child: widget.backLayer),
        PositionedTransition(
          rect: _layerAnimation,
          child: ExcludeSemantics(
            excluding: !_frontLayerVisible,
            child: AnimatedBuilder(
              animation: PageStatus.of(context)!.cartController,
              builder:
                  (BuildContext context, Widget? child) => AnimatedBuilder(
                    animation: PageStatus.of(context)!.menuController,
                    builder:
                        (BuildContext context, Widget? child) => _FrontLayer(
                          onTap: menuPageIsVisible(context) ? _toggleBackdropLayerVisibility : null,
                          child: widget.frontLayer,
                        ),
                  ),
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    final AppBar appBar = AppBar(
      automaticallyImplyLeading: false,
      systemOverlayStyle: SystemUiOverlayStyle.dark,
      elevation: 0,
      titleSpacing: 0,
      title: _BackdropTitle(
        listenable: _controller.view,
        onPress: _toggleBackdropLayerVisibility,
        frontTitle: widget.frontTitle,
        backTitle: widget.backTitle,
      ),
      actions: <Widget>[
        IconButton(
          icon: const Icon(Icons.search),
          tooltip: GalleryLocalizations.of(context)!.shrineTooltipSearch,
          onPressed: () {},
        ),
        IconButton(
          icon: const Icon(Icons.tune),
          tooltip: GalleryLocalizations.of(context)!.shrineTooltipSettings,
          onPressed: () {},
        ),
      ],
    );
    return AnimatedBuilder(
      animation: PageStatus.of(context)!.cartController,
      builder:
          (BuildContext context, Widget? child) => ExcludeSemantics(
            excluding: cartPageIsVisible(context),
            child: Scaffold(appBar: appBar, body: LayoutBuilder(builder: _buildStack)),
          ),
    );
  }
}

class DesktopBackdrop extends StatelessWidget {
  const DesktopBackdrop({super.key, required this.frontLayer, required this.backLayer});

  final Widget frontLayer;
  final Widget backLayer;

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        backLayer,
        Padding(
          padding: EdgeInsetsDirectional.only(
            start: desktopCategoryMenuPageWidth(context: context),
          ),
          child: Material(elevation: 16, color: Colors.white, child: frontLayer),
        ),
      ],
    );
  }
}
