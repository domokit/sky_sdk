// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;

import 'basic_types.dart';
import 'borders.dart';

/// A shape with a notch in its outline.
///
/// Typically used as the outline of a 'host' widget to make a notch that
/// accommodates a 'guest' widget. e.g the [BottomAppBar] may have a notch to
/// accommodate the [FloatingActionButton].
///
/// See also:
///
///  * [ShapeBorder], which defines a shaped border without a dynamic notch.
///  * [AutomaticNotchedShape], an adapter from [ShapeBorder] to [NotchedShape].
abstract class NotchedShape {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const NotchedShape();

  /// Creates a [Path] that describes the outline of the shape.
  ///
  /// The `host` is the bounding rectangle of the shape.
  ///
  /// The `guest` is the bounding rectangle of the shape for which a notch will
  /// be made. It is null when there is no guest.
  Path getOuterPath(Rect host, Rect? guest);
}

/// A rectangle with a smooth circular notch.
///
/// See also:
///
///  * [CircleBorder], a [ShapeBorder] that describes a circle.
class CircularNotchedRectangle extends NotchedShape {
  /// Creates a [CircularNotchedRectangle].
  ///
  /// The same object can be used to create multiple shapes.
  const CircularNotchedRectangle({this.inverted = false});

  /// If [inverted] is true, the notch is placed at the bottom of the
  /// rectangle.
  final bool inverted;

  /// Creates a [Path] that describes a rectangle with a smooth circular notch.
  ///
  /// `host` is the bounding box for the returned shape. Conceptually this is
  /// the rectangle to which the notch will be applied.
  ///
  /// `guest` is the bounding box of a circle that the notch accommodates. All
  /// points in the circle bounded by `guest` will be outside of the returned
  /// path.
  ///
  /// The notch is curve that smoothly connects the host's top edge and
  /// the guest circle.
  // TODO(amirh): add an example diagram here.
  @override
  Path getOuterPath(Rect host, Rect? guest) {
    if (guest == null || !host.overlaps(guest)) {
      return Path()..addRect(host);
    }

    // The guest's shape is a circle bounded by the guest rectangle.
    // So the guest's radius is half the guest width.
    final double r = guest.width / 2.0;
    final Radius notchRadius = Radius.circular(r);

    // The variables [p2yA] and [p2yB] need to be inverted
    // when the notch is drawn on the bottom of a path.
    final double invertMultiplier = inverted ? -1.0 : 1.0;

    // We build a path for the notch from 3 segments:
    // Segment A - a Bezier curve from the host's top edge to segment B.
    // Segment B - an arc with radius notchRadius.
    // Segment C - a Bezier curve from segment B back to the host's top edge.
    //
    // A detailed explanation and the derivation of the formulas below is
    // available at: https://goo.gl/Ufzrqn

    const double s1 = 15.0;
    const double s2 = 1.0;

    final double a = -r - s2;
    final double b = (inverted ? host.bottom : host.top) - guest.center.dy;

    final double n2 = math.sqrt(b * b * r * r * (a * a + b * b - r * r));
    final double p2xA = ((a * r * r) - n2) / (a * a + b * b);
    final double p2xB = ((a * r * r) + n2) / (a * a + b * b);
    final double p2yA = math.sqrt(r * r - p2xA * p2xA) * invertMultiplier;
    final double p2yB = math.sqrt(r * r - p2xB * p2xB) * invertMultiplier;

    final List<Offset> p = List<Offset>.filled(6, Offset.zero);

    // p0, p1, and p2 are the control points for segment A.
    p[0] = Offset(a - s1, b);
    p[1] = Offset(a, b);
    final double cmp = b < 0 ? -1.0 : 1.0;
    p[2] = cmp * p2yA > cmp * p2yB ? Offset(p2xA, p2yA) : Offset(p2xB, p2yB);

    // p3, p4, and p5 are the control points for segment B, which is a mirror
    // of segment A around the y axis.
    p[3] = Offset(-1.0 * p[2].dx, p[2].dy);
    p[4] = Offset(-1.0 * p[1].dx, p[1].dy);
    p[5] = Offset(-1.0 * p[0].dx, p[0].dy);

    // translate all points back to the absolute coordinate system.
    for (int i = 0; i < p.length; i += 1) {
      p[i] += guest.center;
    }

    // Use the calculated points to draw out a path object.
    final Path path = Path()..moveTo(host.left, host.top);
    if (!inverted) {
      path
        ..lineTo(p[0].dx, p[0].dy)
        ..quadraticBezierTo(p[1].dx, p[1].dy, p[2].dx, p[2].dy)
        ..arcToPoint(p[3], radius: notchRadius, clockwise: false)
        ..quadraticBezierTo(p[4].dx, p[4].dy, p[5].dx, p[5].dy)
        ..lineTo(host.right, host.top)
        ..lineTo(host.right, host.bottom)
        ..lineTo(host.left, host.bottom);
    } else {
      path
        ..lineTo(host.right, host.top)
        ..lineTo(host.right, host.bottom)
        ..lineTo(p[5].dx, p[5].dy)
        ..quadraticBezierTo(p[4].dx, p[4].dy, p[3].dx, p[3].dy)
        ..arcToPoint(p[2], radius: notchRadius, clockwise: false)
        ..quadraticBezierTo(p[1].dx, p[1].dy, p[0].dx, p[0].dy)
        ..lineTo(host.left, host.bottom);
    }

    return path..close();
  }
}

/// A [NotchedShape] created from [ShapeBorder]s.
///
/// Two shapes can be provided. The [host] is the shape of the widget that
/// uses the [NotchedShape] (typically a [BottomAppBar]). The [guest] is
/// subtracted from the [host] to create the notch (typically to make room
/// for a [FloatingActionButton]).
class AutomaticNotchedShape extends NotchedShape {
  /// Creates a [NotchedShape] that is defined by two [ShapeBorder]s.
  ///
  /// The [guest] may be null, in which case no notch is created even
  /// if a guest rectangle is provided to [getOuterPath].
  const AutomaticNotchedShape(this.host, [this.guest]);

  /// The shape of the widget that uses the [NotchedShape] (typically a
  /// [BottomAppBar]).
  ///
  /// This shape cannot depend on the [TextDirection], as no text direction
  /// is available to [NotchedShape]s.
  final ShapeBorder host;

  /// The shape to subtract from the [host] to make the notch.
  ///
  /// This shape cannot depend on the [TextDirection], as no text direction
  /// is available to [NotchedShape]s.
  ///
  /// If this is null, [getOuterPath] ignores the guest rectangle.
  final ShapeBorder? guest;

  @override
  Path getOuterPath(Rect hostRect, Rect? guestRect) {
    // ignore: avoid_renaming_method_parameters
    // The parameters of this method are renamed over the baseclass because they
    // would clash with properties of this object, and the use of all four of
    // them in the code below is really confusing if they have the same names.
    final Path hostPath = host.getOuterPath(hostRect);
    if (guest != null && guestRect != null) {
      final Path guestPath = guest!.getOuterPath(guestRect);
      return Path.combine(PathOperation.difference, hostPath, guestPath);
    }
    return hostPath;
  }
}
