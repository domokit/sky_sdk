// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

///
library;

import 'package:flutter/widgets.dart';

import 'constants.dart';

/// iOS-style collapsible widget.
///
/// This widget is a wrapper around [AnimatedCrossFade] that animates the height of the child widget,
/// and clips it to fit the parent when collapsed.
///
/// When collapsing/expanding, the height animates between 0 and the height of the child,
/// while the child is still laid out and painted. During the animation,
/// the size of the child doesn't change, it only gets clipped to fit the parent
class CupertinoCollapsible extends StatelessWidget {
  /// Creates an iOS-style collapsible widget.
  const CupertinoCollapsible({
    super.key,
    required this.child,
    this.isExpanded = true,
    this.animationStyle,
  }) : super();

  /// The child widget to be collapsed or expanded.
  final Widget child;

  /// Whether the child is expanded.
  ///
  /// Defaults to true.
  final bool isExpanded;

    /// Used to override the expansion animation curve and duration.
  ///
  /// If [AnimationStyle.duration] is provided, it will be used to override
  /// the expansion animation duration. If it is null, then
  /// [kCupertinoCollapsibleAnimationDuration] will be used.
  ///
  /// If [AnimationStyle.curve] is provided, it will be used to override
  /// the expansion animation curve. If it is null, then
  /// [kCupertinoCollapsibleAnimationCurve] will be used.
  ///
  /// The same curve will be used for expansion and collapse animations.
  ///
  /// To disable the animation, use [AnimationStyle.noAnimation].
  final AnimationStyle? animationStyle;

  @override
  Widget build(BuildContext context) {
    return AnimatedCrossFade(
      crossFadeState: isExpanded ? CrossFadeState.showSecond : CrossFadeState.showFirst,
      duration: animationStyle?.duration ?? kCupertinoCollapsibleAnimationDuration,
      firstCurve: animationStyle?.curve ?? kCupertinoCollapsibleAnimationCurve,
      secondCurve: animationStyle?.curve ?? kCupertinoCollapsibleAnimationCurve,
      sizeCurve: animationStyle?.curve ?? kCupertinoCollapsibleAnimationCurve,
      alignment: Alignment.bottomLeft,
      firstChild: const SizedBox(width: double.infinity, height: 0.0,),
      secondChild: child,
      layoutBuilder: (Widget topChild, Key topChildKey, Widget bottomChild, Key bottomChildKey) => Stack(
        clipBehavior: Clip.none,
        children: <Widget>[
          Positioned(
            key: bottomChildKey,
            left: 0.0,
            right: 0.0,
            bottom: 0.0,
            child: bottomChild,
          ),
          Positioned(
            key: topChildKey,
            child: topChild,
          ),
        ],
      )
    );
  }
}
