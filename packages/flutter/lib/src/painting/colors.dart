// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show Color, lerpDouble;

import 'package:flutter/foundation.dart';

double _getHue(double red, double green, double blue, double max, double delta) {
  late double hue;
  if (max == 0.0) {
    hue = 0.0;
  } else if (max == red) {
    hue = 60.0 * (((green - blue) / delta) % 6);
  } else if (max == green) {
    hue = 60.0 * (((blue - red) / delta) + 2);
  } else if (max == blue) {
    hue = 60.0 * (((red - green) / delta) + 4);
  }

  /// Set hue to 0.0 when red == green == blue.
  hue = hue.isNaN ? 0.0 : hue;
  return hue;
}

Color _colorFromHue(double alpha, double hue, double chroma, double secondary, double match) {
  final (double red, double green, double blue) = switch (hue) {
    < 60.0 => (chroma, secondary, 0.0),
    < 120.0 => (secondary, chroma, 0.0),
    < 180.0 => (0.0, chroma, secondary),
    < 240.0 => (0.0, secondary, chroma),
    < 300.0 => (secondary, 0.0, chroma),
    _ => (chroma, 0.0, secondary),
  };
  return Color.fromARGB(
    (alpha * 0xFF).round(),
    ((red + match) * 0xFF).round(),
    ((green + match) * 0xFF).round(),
    ((blue + match) * 0xFF).round(),
  );
}

/// A color represented using [alpha], [hue], [saturation], and [value].
///
/// An [HSVColor] is represented in a parameter space that's based on human
/// perception of color in pigments (e.g. paint and printer's ink). The
/// representation is useful for some color computations (e.g. rotating the hue
/// through the colors), because interpolation and picking of
/// colors as red, green, and blue channels doesn't always produce intuitive
/// results.
///
/// The HSV color space models the way that different pigments are perceived
/// when mixed. The hue describes which pigment is used, the saturation
/// describes which shade of the pigment, and the value resembles mixing the
/// pigment with different amounts of black or white pigment.
///
/// See also:
///
///  * [HSLColor], a color that uses a color space based on human perception of
///    colored light.
///  * [HSV and HSL](https://en.wikipedia.org/wiki/HSL_and_HSV) Wikipedia
///    article, which this implementation is based upon.
@immutable
class HSVColor {
  /// Creates a color.
  ///
  /// All the arguments must be in their respective ranges. See the fields for
  /// each parameter for a description of their ranges.
  const HSVColor.fromAHSV(this.alpha, this.hue, this.saturation, this.value)
    : assert(alpha >= 0.0),
      assert(alpha <= 1.0),
      assert(hue >= 0.0),
      assert(hue <= 360.0),
      assert(saturation >= 0.0),
      assert(saturation <= 1.0),
      assert(value >= 0.0),
      assert(value <= 1.0);

  /// Creates an [HSVColor] from an RGB [Color].
  ///
  /// This constructor does not necessarily round-trip with [toColor] because
  /// of floating point imprecision.
  factory HSVColor.fromColor(Color color) {
    final double red = color.red / 0xFF;
    final double green = color.green / 0xFF;
    final double blue = color.blue / 0xFF;

    final double max = math.max(red, math.max(green, blue));
    final double min = math.min(red, math.min(green, blue));
    final double delta = max - min;

    final double alpha = color.alpha / 0xFF;
    final double hue = _getHue(red, green, blue, max, delta);
    final double saturation = max == 0.0 ? 0.0 : delta / max;

    return HSVColor.fromAHSV(alpha, hue, saturation, max);
  }

  /// Alpha, from 0.0 to 1.0. The describes the transparency of the color.
  /// A value of 0.0 is fully transparent, and 1.0 is fully opaque.
  final double alpha;

  /// Hue, from 0.0 to 360.0. Describes which color of the spectrum is
  /// represented. A value of 0.0 represents red, as does 360.0. Values in
  /// between go through all the hues representable in RGB. You can think of
  /// this as selecting which pigment will be added to a color.
  final double hue;

  /// Saturation, from 0.0 to 1.0. This describes how colorful the color is.
  /// 0.0 implies a shade of grey (i.e. no pigment), and 1.0 implies a color as
  /// vibrant as that hue gets. You can think of this as the equivalent of
  /// how much of a pigment is added.
  final double saturation;

  /// Value, from 0.0 to 1.0. The "value" of a color that, in this context,
  /// describes how bright a color is. A value of 0.0 indicates black, and 1.0
  /// indicates full intensity color. You can think of this as the equivalent of
  /// removing black from the color as value increases.
  final double value;

  /// Returns a copy of this color with the [alpha] parameter replaced with the
  /// given value.
  HSVColor withAlpha(double alpha) {
    return HSVColor.fromAHSV(alpha, hue, saturation, value);
  }

  /// Returns a copy of this color with the [hue] parameter replaced with the
  /// given value.
  HSVColor withHue(double hue) {
    return HSVColor.fromAHSV(alpha, hue, saturation, value);
  }

  /// Returns a copy of this color with the [saturation] parameter replaced with
  /// the given value.
  HSVColor withSaturation(double saturation) {
    return HSVColor.fromAHSV(alpha, hue, saturation, value);
  }

  /// Returns a copy of this color with the [value] parameter replaced with the
  /// given value.
  HSVColor withValue(double value) {
    return HSVColor.fromAHSV(alpha, hue, saturation, value);
  }

  /// Returns this color in RGB.
  Color toColor() {
    final double chroma = saturation * value;
    final double secondary = chroma * (1.0 - (((hue / 60.0) % 2.0) - 1.0).abs());
    final double match = value - chroma;

    return _colorFromHue(alpha, hue, chroma, secondary, match);
  }

  HSVColor _scaleAlpha(double factor) {
    return withAlpha(alpha * factor);
  }

  /// Linearly interpolate between two HSVColors.
  ///
  /// The colors are interpolated by interpolating the [alpha], [hue],
  /// [saturation], and [value] channels separately, which usually leads to a
  /// more pleasing effect than [Color.lerp] (which interpolates the red, green,
  /// and blue channels separately).
  ///
  /// If either color is null, this function linearly interpolates from a
  /// transparent instance of the other color. This is usually preferable to
  /// interpolating from [Colors.transparent] (`const Color(0x00000000)`) since
  /// that will interpolate from a transparent red and cycle through the hues to
  /// match the target color, regardless of what that color's hue is.
  ///
  /// {@macro dart.ui.shadow.lerp}
  ///
  /// Values outside of the valid range for each channel will be clamped.
  static HSVColor? lerp(HSVColor? a, HSVColor? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!._scaleAlpha(t);
    }
    if (b == null) {
      return a._scaleAlpha(1.0 - t);
    }
    return HSVColor.fromAHSV(
      clampDouble(lerpDouble(a.alpha, b.alpha, t)!, 0.0, 1.0),
      lerpDouble(a.hue, b.hue, t)! % 360.0,
      clampDouble(lerpDouble(a.saturation, b.saturation, t)!, 0.0, 1.0),
      clampDouble(lerpDouble(a.value, b.value, t)!, 0.0, 1.0),
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is HSVColor &&
        other.alpha == alpha &&
        other.hue == hue &&
        other.saturation == saturation &&
        other.value == value;
  }

  @override
  int get hashCode => Object.hash(alpha, hue, saturation, value);

  @override
  String toString() => '${objectRuntimeType(this, 'HSVColor')}($alpha, $hue, $saturation, $value)';
}

/// A color represented using [alpha], [hue], [saturation], and [lightness].
///
/// An [HSLColor] is represented in a parameter space that's based up human
/// perception of colored light. The representation is useful for some color
/// computations (e.g., combining colors of light), because interpolation and
/// picking of colors as red, green, and blue channels doesn't always produce
/// intuitive results.
///
/// HSL is a perceptual color model, placing fully saturated colors around a
/// circle (conceptually) at a lightness of ​0.5, with a lightness of 0.0 being
/// completely black, and a lightness of 1.0 being completely white. As the
/// lightness increases or decreases from 0.5, the apparent saturation decreases
/// proportionally (even though the [saturation] parameter hasn't changed).
///
/// See also:
///
///  * [HSVColor], a color that uses a color space based on human perception of
///    pigments (e.g. paint and printer's ink).
///  * [HSV and HSL](https://en.wikipedia.org/wiki/HSL_and_HSV) Wikipedia
///    article, which this implementation is based upon.
@immutable
class HSLColor {
  /// Creates a color.
  ///
  /// All the arguments must be in their respective ranges. See the fields for
  /// each parameter for a description of their ranges.
  const HSLColor.fromAHSL(this.alpha, this.hue, this.saturation, this.lightness)
    : assert(alpha >= 0.0),
      assert(alpha <= 1.0),
      assert(hue >= 0.0),
      assert(hue <= 360.0),
      assert(saturation >= 0.0),
      assert(saturation <= 1.0),
      assert(lightness >= 0.0),
      assert(lightness <= 1.0);

  /// Creates an [HSLColor] from an RGB [Color].
  ///
  /// This constructor does not necessarily round-trip with [toColor] because
  /// of floating point imprecision.
  factory HSLColor.fromColor(Color color) {
    final double red = color.red / 0xFF;
    final double green = color.green / 0xFF;
    final double blue = color.blue / 0xFF;

    final double max = math.max(red, math.max(green, blue));
    final double min = math.min(red, math.min(green, blue));
    final double delta = max - min;

    final double alpha = color.alpha / 0xFF;
    final double hue = _getHue(red, green, blue, max, delta);
    final double lightness = (max + min) / 2.0;
    // Saturation can exceed 1.0 with rounding errors, so clamp it.
    final double saturation =
        lightness == 1.0
            ? 0.0
            : clampDouble(delta / (1.0 - (2.0 * lightness - 1.0).abs()), 0.0, 1.0);
    return HSLColor.fromAHSL(alpha, hue, saturation, lightness);
  }

  /// Alpha, from 0.0 to 1.0. The describes the transparency of the color.
  /// A value of 0.0 is fully transparent, and 1.0 is fully opaque.
  final double alpha;

  /// Hue, from 0.0 to 360.0. Describes which color of the spectrum is
  /// represented. A value of 0.0 represents red, as does 360.0. Values in
  /// between go through all the hues representable in RGB. You can think of
  /// this as selecting which color filter is placed over a light.
  final double hue;

  /// Saturation, from 0.0 to 1.0. This describes how colorful the color is.
  /// 0.0 implies a shade of grey (i.e. no pigment), and 1.0 implies a color as
  /// vibrant as that hue gets. You can think of this as the purity of the
  /// color filter over the light.
  final double saturation;

  /// Lightness, from 0.0 to 1.0. The lightness of a color describes how bright
  /// a color is. A value of 0.0 indicates black, and 1.0 indicates white. You
  /// can think of this as the intensity of the light behind the filter. As the
  /// lightness approaches 0.5, the colors get brighter and appear more
  /// saturated, and over 0.5, the colors start to become less saturated and
  /// approach white at 1.0.
  final double lightness;

  /// Returns a copy of this color with the alpha parameter replaced with the
  /// given value.
  HSLColor withAlpha(double alpha) {
    return HSLColor.fromAHSL(alpha, hue, saturation, lightness);
  }

  /// Returns a copy of this color with the [hue] parameter replaced with the
  /// given value.
  HSLColor withHue(double hue) {
    return HSLColor.fromAHSL(alpha, hue, saturation, lightness);
  }

  /// Returns a copy of this color with the [saturation] parameter replaced with
  /// the given value.
  HSLColor withSaturation(double saturation) {
    return HSLColor.fromAHSL(alpha, hue, saturation, lightness);
  }

  /// Returns a copy of this color with the [lightness] parameter replaced with
  /// the given value.
  HSLColor withLightness(double lightness) {
    return HSLColor.fromAHSL(alpha, hue, saturation, lightness);
  }

  /// Returns this HSL color in RGB.
  Color toColor() {
    final double chroma = (1.0 - (2.0 * lightness - 1.0).abs()) * saturation;
    final double secondary = chroma * (1.0 - (((hue / 60.0) % 2.0) - 1.0).abs());
    final double match = lightness - chroma / 2.0;

    return _colorFromHue(alpha, hue, chroma, secondary, match);
  }

  HSLColor _scaleAlpha(double factor) {
    return withAlpha(alpha * factor);
  }

  /// Linearly interpolate between two HSLColors.
  ///
  /// The colors are interpolated by interpolating the [alpha], [hue],
  /// [saturation], and [lightness] channels separately, which usually leads to
  /// a more pleasing effect than [Color.lerp] (which interpolates the red,
  /// green, and blue channels separately).
  ///
  /// If either color is null, this function linearly interpolates from a
  /// transparent instance of the other color. This is usually preferable to
  /// interpolating from [Colors.transparent] (`const Color(0x00000000)`) since
  /// that will interpolate from a transparent red and cycle through the hues to
  /// match the target color, regardless of what that color's hue is.
  ///
  /// The `t` argument represents position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values between them
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  /// 1.0, so negative values and values greater than 1.0 are valid
  /// (and can easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values outside of the valid range for each channel will be clamped.
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static HSLColor? lerp(HSLColor? a, HSLColor? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!._scaleAlpha(t);
    }
    if (b == null) {
      return a._scaleAlpha(1.0 - t);
    }
    return HSLColor.fromAHSL(
      clampDouble(lerpDouble(a.alpha, b.alpha, t)!, 0.0, 1.0),
      lerpDouble(a.hue, b.hue, t)! % 360.0,
      clampDouble(lerpDouble(a.saturation, b.saturation, t)!, 0.0, 1.0),
      clampDouble(lerpDouble(a.lightness, b.lightness, t)!, 0.0, 1.0),
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is HSLColor &&
        other.alpha == alpha &&
        other.hue == hue &&
        other.saturation == saturation &&
        other.lightness == lightness;
  }

  @override
  int get hashCode => Object.hash(alpha, hue, saturation, lightness);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'HSLColor')}($alpha, $hue, $saturation, $lightness)';
}

/// A color that has a small table of related colors called a "swatch".
///
/// The table is accessed by key values of type `T`.
///
/// See also:
///
///  * [MaterialColor] and [MaterialAccentColor], which define Material Design
///    primary and accent color swatches.
///  * [Colors], which defines all of the standard Material Design
///    colors.
@immutable
class ColorSwatch<T> extends Color {
  /// Creates a color that has a small table of related colors called a "swatch".
  ///
  /// The `primary` argument should be the 32 bit ARGB value of one of the
  /// values in the swatch, as would be passed to the [Color.new] constructor
  /// for that same color, and as is exposed by [value]. (This is distinct from
  /// the key of any color in the swatch.)
  const ColorSwatch(super.primary, this._swatch);

  @protected
  final Map<T, Color> _swatch;

  /// Returns an element of the swatch table.
  Color? operator [](T key) => _swatch[key];

  /// Returns the valid keys for accessing operator[].
  Iterable<T> get keys => _swatch.keys;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return super == other && other is ColorSwatch<T> && mapEquals<T, Color>(other._swatch, _swatch);
  }

  @override
  int get hashCode => Object.hash(runtimeType, value, _swatch);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'ColorSwatch')}(primary value: ${super.toString()})';

  /// Linearly interpolate between two [ColorSwatch]es.
  ///
  /// It delegates to [Color.lerp] to interpolate the different colors of the
  /// swatch.
  ///
  /// If either color is null, this function linearly interpolates from a
  /// transparent instance of the other color.
  ///
  /// The `t` argument represents position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a` (or something
  /// equivalent to `a`), 1.0 meaning that the interpolation has finished,
  /// returning `b` (or something equivalent to `b`), and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  /// 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]). Each channel
  /// will be clamped to the range 0 to 255.
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static ColorSwatch<T>? lerp<T>(ColorSwatch<T>? a, ColorSwatch<T>? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    final Map<T, Color> swatch;
    if (b == null) {
      swatch = a!._swatch.map(
        (T key, Color color) => MapEntry<T, Color>(key, Color.lerp(color, null, t)!),
      );
    } else {
      if (a == null) {
        swatch = b._swatch.map(
          (T key, Color color) => MapEntry<T, Color>(key, Color.lerp(null, color, t)!),
        );
      } else {
        swatch = a._swatch.map(
          (T key, Color color) => MapEntry<T, Color>(key, Color.lerp(color, b[key], t)!),
        );
      }
    }
    return ColorSwatch<T>(Color.lerp(a, b, t)!.value, swatch);
  }
}

/// [DiagnosticsProperty] that has an [Color] as value.
class ColorProperty extends DiagnosticsProperty<Color> {
  /// Create a diagnostics property for [Color].
  ColorProperty(
    String super.name,
    super.value, {
    super.showName,
    super.defaultValue,
    super.style,
    super.level,
  });

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (value != null) {
      json['valueProperties'] = <String, Object>{
        'red': value!.red,
        'green': value!.green,
        'blue': value!.blue,
        'alpha': value!.alpha,
      };
    }
    return json;
  }
}
