// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:collection';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'browser_detection.dart' show isIOS15, isMacOrIOS;
import 'dom.dart';
import 'safe_browser_api.dart';
import 'services.dart';
import 'vector_math.dart';

/// Generic callback signature, used by [_futurize].
typedef Callback<T> = void Function(T result);

/// Signature for a method that receives a [_Callback].
///
/// Return value should be null on success, and a string error message on
/// failure.
typedef Callbacker<T> = String? Function(Callback<T> callback);

/// Converts a method that receives a value-returning callback to a method that
/// returns a Future.
///
/// Return a [String] to cause an [Exception] to be synchronously thrown with
/// that string as a message.
///
/// If the callback is called with null, the future completes with an error.
///
/// Example usage:
///
/// ```dart
/// typedef IntCallback = void Function(int result);
///
/// String _doSomethingAndCallback(IntCallback callback) {
///   Timer(const Duration(seconds: 1), () { callback(1); });
/// }
///
/// Future<int> doSomething() {
///   return futurize(_doSomethingAndCallback);
/// }
/// ```
// Keep this in sync with _futurize in lib/ui/fixtures/ui_test.dart.
Future<T> futurize<T>(Callbacker<T> callbacker) {
  final Completer<T> completer = Completer<T>.sync();
  // If the callback synchronously throws an error, then synchronously
  // rethrow that error instead of adding it to the completer. This
  // prevents the Zone from receiving an uncaught exception.
  bool isSync = true;
  final String? error = callbacker((T? t) {
    if (t == null) {
      if (isSync) {
        throw Exception('operation failed');
      } else {
        completer.completeError(Exception('operation failed'));
      }
    } else {
      completer.complete(t);
    }
  });
  isSync = false;
  if (error != null) {
    throw Exception(error);
  }
  return completer.future;
}

/// Converts [matrix] to CSS transform value.
String matrix4ToCssTransform(Matrix4 matrix) {
  return float64ListToCssTransform(matrix.storage);
}

/// Applies a transform to the [element].
///
/// See [float64ListToCssTransform] for details on how the CSS value is chosen.
void setElementTransform(DomElement element, Float32List matrix4) {
  element.style
    ..transformOrigin = '0 0 0'
    ..transform = float64ListToCssTransform(matrix4);
}

/// Converts [matrix] to CSS transform value.
///
/// To avoid blurry text on some screens this function uses a 2D CSS transform
/// if it detects that [matrix] is a 2D transform. Otherwise, it uses a 3D CSS
/// transform.
///
/// See also:
///  * https://github.com/flutter/flutter/issues/32274
///  * https://bugs.chromium.org/p/chromium/issues/detail?id=1040222
String float64ListToCssTransform(List<double> matrix) {
  assert(matrix.length == 16);
  final TransformKind transformKind = transformKindOf(matrix);
  if (transformKind == TransformKind.transform2d) {
    return float64ListToCssTransform2d(matrix);
  } else if (transformKind == TransformKind.complex) {
    return float64ListToCssTransform3d(matrix);
  } else {
    assert(transformKind == TransformKind.identity);
    return 'none';
  }
}

/// The kind of effect a transform matrix performs.
enum TransformKind {
  /// No effect.
  ///
  /// We do not want to set any CSS properties in this case.
  identity,

  /// A transform that contains only 2d scale, rotation, and translation.
  ///
  /// We prefer to use "matrix" instead of "matrix3d" in this case.
  transform2d,

  /// All other kinds of transforms.
  ///
  /// In this case we will use "matrix3d".
  complex,
}

/// Detects the kind of transform the [matrix] performs.
TransformKind transformKindOf(List<double> matrix) {
  assert(matrix.length == 16);
  final List<double> m = matrix;

  // If matrix contains scaling, rotation, z translation or
  // perspective transform, it is not considered simple.
  final bool isSimple2dTransform =
      m[15] ==
          1.0 && // start reading from the last element to eliminate range checks in subsequent reads.
      m[14] == 0.0 && // z translation is NOT simple
      // m[13] - y translation is simple
      // m[12] - x translation is simple
      m[11] == 0.0 &&
      m[10] == 1.0 &&
      m[9] == 0.0 &&
      m[8] == 0.0 &&
      m[7] == 0.0 &&
      m[6] == 0.0 &&
      // m[5] - scale y is simple
      // m[4] - 2D rotation is simple
      m[3] == 0.0 &&
      m[2] == 0.0;
  // m[1] - 2D rotation is simple
  // m[0] - scale x is simple

  if (!isSimple2dTransform) {
    return TransformKind.complex;
  }

  // From this point on we're sure the transform is 2D, but we don't know if
  // it's identity or not. To check, we need to look at the remaining elements
  // that were not checked above.
  final bool isIdentityTransform =
      m[0] == 1.0 && m[1] == 0.0 && m[4] == 0.0 && m[5] == 1.0 && m[12] == 0.0 && m[13] == 0.0;

  if (isIdentityTransform) {
    return TransformKind.identity;
  } else {
    return TransformKind.transform2d;
  }
}

/// Returns `true` is the [matrix] describes an identity transformation.
bool isIdentityFloat32ListTransform(Float32List matrix) {
  assert(matrix.length == 16);
  return transformKindOf(matrix) == TransformKind.identity;
}

/// Converts [matrix] to CSS transform 2D matrix value.
///
/// The [matrix] must not be a [TransformKind.complex] transform, because CSS
/// `matrix` can only express 2D transforms. [TransformKind.identity] is
/// permitted. However, it is inefficient to construct a matrix for an identity
/// transform. Consider removing the CSS `transform` property from elements
/// that apply identity transform.
String float64ListToCssTransform2d(List<double> matrix) {
  assert(transformKindOf(matrix) != TransformKind.complex);
  return 'matrix(${matrix[0]},${matrix[1]},${matrix[4]},${matrix[5]},${matrix[12]},${matrix[13]})';
}

/// Converts [matrix] to a 3D CSS transform value.
String float64ListToCssTransform3d(List<double> matrix) {
  assert(matrix.length == 16);
  final List<double> m = matrix;
  if (m[0] == 1.0 &&
      m[1] == 0.0 &&
      m[2] == 0.0 &&
      m[3] == 0.0 &&
      m[4] == 0.0 &&
      m[5] == 1.0 &&
      m[6] == 0.0 &&
      m[7] == 0.0 &&
      m[8] == 0.0 &&
      m[9] == 0.0 &&
      m[10] == 1.0 &&
      m[11] == 0.0 &&
      // 12 can be anything
      // 13 can be anything
      m[14] == 0.0 &&
      m[15] == 1.0) {
    final double tx = m[12];
    final double ty = m[13];
    return 'translate3d(${tx}px, ${ty}px, 0px)';
  } else {
    return 'matrix3d(${m[0]},${m[1]},${m[2]},${m[3]},${m[4]},${m[5]},${m[6]},${m[7]},${m[8]},${m[9]},${m[10]},${m[11]},${m[12]},${m[13]},${m[14]},${m[15]})';
  }
}

final Float32List _tempRectData = Float32List(4);

/// Transforms a [ui.Rect] given the effective [transform].
///
/// The resulting rect is aligned to the pixel grid, i.e. two of
/// its sides are vertical and two are horizontal. In the presence of rotations
/// the rectangle is inflated such that it fits the rotated rectangle.
ui.Rect transformRectWithMatrix(Matrix4 transform, ui.Rect rect) {
  _tempRectData[0] = rect.left;
  _tempRectData[1] = rect.top;
  _tempRectData[2] = rect.right;
  _tempRectData[3] = rect.bottom;
  transformLTRB(transform, _tempRectData);
  return ui.Rect.fromLTRB(_tempRectData[0], _tempRectData[1], _tempRectData[2], _tempRectData[3]);
}

/// Temporary storage for intermediate data used by [transformLTRB].
///
/// WARNING: do not use this outside [transformLTRB]. Sharing this variable in
/// other contexts will lead to bugs.
final Float32List _tempPointData = Float32List(16);
final Matrix4 _tempPointMatrix = Matrix4.fromFloat32List(_tempPointData);

/// Transforms a rectangle given the effective [transform].
///
/// This is the same as [transformRect], except that the rect is specified
/// in terms of left, top, right, and bottom edge offsets.
void transformLTRB(Matrix4 transform, Float32List ltrb) {
  // Construct a matrix where each row represents a vector pointing at
  // one of the four corners of the (left, top, right, bottom) rectangle.
  // Using the row-major order allows us to multiply the matrix in-place
  // by the transposed current transformation matrix. The vector_math
  // library has a convenience function `multiplyTranspose` that performs
  // the multiplication without copying. This way we compute the positions
  // of all four points in a single matrix-by-matrix multiplication at the
  // cost of one `Matrix4` instance and one `Float32List` instance.
  //
  // The rejected alternative was to use `Vector3` for each point and
  // multiply by the current transform. However, that would cost us four
  // `Vector3` instances, four `Float32List` instances, and four
  // matrix-by-vector multiplications.
  //
  // `Float32List` initializes the array with zeros, so we do not have to
  // fill in every single element.

  // Row 0: top-left
  _tempPointData[0] = ltrb[0];
  _tempPointData[4] = ltrb[1];
  _tempPointData[8] = 0;
  _tempPointData[12] = 1;

  // Row 1: top-right
  _tempPointData[1] = ltrb[2];
  _tempPointData[5] = ltrb[1];
  _tempPointData[9] = 0;
  _tempPointData[13] = 1;

  // Row 2: bottom-left
  _tempPointData[2] = ltrb[0];
  _tempPointData[6] = ltrb[3];
  _tempPointData[10] = 0;
  _tempPointData[14] = 1;

  // Row 3: bottom-right
  _tempPointData[3] = ltrb[2];
  _tempPointData[7] = ltrb[3];
  _tempPointData[11] = 0;
  _tempPointData[15] = 1;

  _tempPointMatrix.multiplyTranspose(transform);

  // Handle non-homogenous matrices.
  double w = transform[15];
  if (w == 0.0) {
    w = 1.0;
  }

  ltrb[0] =
      math.min(
        math.min(math.min(_tempPointData[0], _tempPointData[1]), _tempPointData[2]),
        _tempPointData[3],
      ) /
      w;
  ltrb[1] =
      math.min(
        math.min(math.min(_tempPointData[4], _tempPointData[5]), _tempPointData[6]),
        _tempPointData[7],
      ) /
      w;
  ltrb[2] =
      math.max(
        math.max(math.max(_tempPointData[0], _tempPointData[1]), _tempPointData[2]),
        _tempPointData[3],
      ) /
      w;
  ltrb[3] =
      math.max(
        math.max(math.max(_tempPointData[4], _tempPointData[5]), _tempPointData[6]),
        _tempPointData[7],
      ) /
      w;
}

/// Returns true if [rect] contains every point that is also contained by the
/// [other] rect.
///
/// Points on the edges of both rectangles are also considered. For example,
/// this returns true when the two rects are equal to each other.
bool rectContainsOther(ui.Rect rect, ui.Rect other) {
  return rect.left <= other.left &&
      rect.top <= other.top &&
      rect.right >= other.right &&
      rect.bottom >= other.bottom;
}

extension CssColor on ui.Color {
  /// Converts color to a css compatible attribute value.
  String toCssString() {
    return colorValueToCssString(value);
  }
}

// Converts a color value (as an int) into a CSS-compatible value.
String colorValueToCssString(int value) {
  if (value == 0xFF000000) {
    return '#000000';
  }
  if ((0xff000000 & value) == 0xff000000) {
    final String hexValue = (value & 0xFFFFFF).toRadixString(16);
    final int hexValueLength = hexValue.length;
    switch (hexValueLength) {
      case 1:
        return '#00000$hexValue';
      case 2:
        return '#0000$hexValue';
      case 3:
        return '#000$hexValue';
      case 4:
        return '#00$hexValue';
      case 5:
        return '#0$hexValue';
      default:
        return '#$hexValue';
    }
  } else {
    final double alpha = ((value >> 24) & 0xFF) / 255.0;
    final StringBuffer sb = StringBuffer();
    sb.write('rgba(');
    sb.write(((value >> 16) & 0xFF).toString());
    sb.write(',');
    sb.write(((value >> 8) & 0xFF).toString());
    sb.write(',');
    sb.write((value & 0xFF).toString());
    sb.write(',');
    sb.write(alpha.toString());
    sb.write(')');
    return sb.toString();
  }
}

/// Converts color components to a CSS compatible attribute value.
String colorComponentsToCssString(int r, int g, int b, int a) {
  if (a == 255) {
    return 'rgb($r,$g,$b)';
  } else {
    final double alphaRatio = a / 255;
    return 'rgba($r,$g,$b,${alphaRatio.toStringAsFixed(2)})';
  }
}

/// Determines if the (dynamic) exception passed in is a NS_ERROR_FAILURE
/// (from Firefox).
///
/// NS_ERROR_FAILURE (0x80004005) is the most general of all the (Firefox)
/// errors and occurs for all errors for which a more specific error code does
/// not apply. (https://developer.mozilla.org/en-US/docs/Mozilla/Errors)
///
/// Other browsers do not throw this exception.
///
/// In Flutter, this exception happens when we try to perform some operations on
/// a Canvas when the application is rendered in a display:none iframe.
///
/// We need this in [BitmapCanvas] and [RecordingCanvas] to swallow this
/// Firefox exception without interfering with others (potentially useful
/// for the programmer).
bool isNsErrorFailureException(Object e) {
  return getJsProperty<dynamic>(e, 'name') == 'NS_ERROR_FAILURE';
}

/// From: https://developer.mozilla.org/en-US/docs/Web/CSS/font-family#Syntax
///
/// Generic font families are a fallback mechanism, a means of preserving some
/// of the style sheet author's intent when none of the specified fonts are
/// available. Generic family names are keywords and must not be quoted. A
/// generic font family should be the last item in the list of font family
/// names.
const Set<String> _genericFontFamilies = <String>{
  'serif',
  'sans-serif',
  'monospace',
  'cursive',
  'fantasy',
  'system-ui',
  'math',
  'emoji',
  'fangsong',
};

/// A default fallback font family in case an unloaded font has been requested.
///
/// -apple-system targets San Francisco in Safari (on Mac OS X and iOS),
/// and it targets Neue Helvetica and Lucida Grande on older versions of
/// Mac OS X. It properly selects between San Francisco Text and
/// San Francisco Display depending on the text’s size.
///
/// For iOS, default to -apple-system, where it should be available, otherwise
/// default to Arial. BlinkMacSystemFont is used for Chrome on iOS.
String get _fallbackFontFamily {
  if (isIOS15) {
    // Remove the "-apple-system" fallback font because it causes a crash in
    // iOS 15.
    //
    // See github issue: https://github.com/flutter/flutter/issues/90705
    // See webkit bug: https://bugs.webkit.org/show_bug.cgi?id=231686
    return 'BlinkMacSystemFont';
  }
  if (isMacOrIOS) {
    return '-apple-system, BlinkMacSystemFont';
  }
  return 'Arial';
}

/// Create a font-family string appropriate for CSS.
///
/// If the given [fontFamily] is a generic font-family, then just return it.
/// Otherwise, wrap the family name in quotes and add a fallback font family.
String? canonicalizeFontFamily(String? fontFamily) {
  if (_genericFontFamilies.contains(fontFamily)) {
    return fontFamily;
  }
  if (isMacOrIOS) {
    // Unlike Safari, Chrome on iOS does not correctly fallback to cupertino
    // on sans-serif.
    // Map to San Francisco Text/Display fonts, use -apple-system,
    // BlinkMacSystemFont.
    if (fontFamily == '.SF Pro Text' ||
        fontFamily == '.SF Pro Display' ||
        fontFamily == '.SF UI Text' ||
        fontFamily == '.SF UI Display') {
      return _fallbackFontFamily;
    }
  }
  return '"$fontFamily", $_fallbackFontFamily, sans-serif';
}

/// Converts a list of [Offset] to a typed array of floats.
Float32List offsetListToFloat32List(List<ui.Offset> offsetList) {
  final int length = offsetList.length;
  final Float32List floatList = Float32List(length * 2);
  for (int i = 0, destIndex = 0; i < length; i++, destIndex += 2) {
    floatList[destIndex] = offsetList[i].dx;
    floatList[destIndex + 1] = offsetList[i].dy;
  }
  return floatList;
}

/// Apply this function to container elements in the HTML render tree (this is
/// not relevant to semantics tree).
///
/// On WebKit browsers this will apply `z-order: 0` to ensure that clips are
/// applied correctly. Otherwise, the browser will refuse to clip its contents.
///
/// Other possible fixes that were rejected:
///
/// * Use 3D transform instead of 2D: this does not work because it causes text
///   blurriness: https://github.com/flutter/flutter/issues/32274
void applyWebkitClipFix(DomElement? containerElement) {
  if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
    containerElement!.style.zIndex = '0';
  }
}

/// Roughly the inverse of [ui.Shadow.convertRadiusToSigma].
///
/// This does not inverse [ui.Shadow.convertRadiusToSigma] exactly, because on
/// the Web the difference between sigma and blur radius is different from
/// Flutter mobile.
double convertSigmaToRadius(double sigma) {
  return sigma * 2.0;
}

int clampInt(int value, int min, int max) {
  assert(min <= max);
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}

/// Prints a warning message to the console.
///
/// This function can be overridden in tests. This could be useful, for example,
/// to verify that warnings are printed under certain circumstances.
void Function(String) printWarning = domWindow.console.warn;

/// Converts a 4x4 matrix into a human-readable String.
String matrixString(List<num> matrix) {
  final StringBuffer sb = StringBuffer();
  for (int i = 0; i < 16; i++) {
    sb.write(matrix[i]);
    if ((i + 1) % 4 == 0) {
      sb.write('\n');
    } else {
      sb.write(' ');
    }
  }
  return sb.toString();
}

/// Determines if lists [a] and [b] are deep equivalent.
///
/// Returns true if the lists are both null, or if they are both non-null, have
/// the same length, and contain the same elements in the same order. Returns
/// false otherwise.
bool listEquals<T>(List<T>? a, List<T>? b) {
  if (a == null) {
    return b == null;
  }
  if (b == null || a.length != b.length) {
    return false;
  }
  for (int index = 0; index < a.length; index += 1) {
    if (a[index] != b[index]) {
      return false;
    }
  }
  return true;
}

// HTML only supports a single radius, but Flutter ImageFilter supports separate
// horizontal and vertical radii. The best approximation we can provide is to
// average the two radii together for a single compromise value.
String blurSigmasToCssString(double sigmaX, double sigmaY) {
  return 'blur(${(sigmaX + sigmaY) * 0.5}px)';
}

/// Extensions to [Map] that make it easier to treat it as a JSON object. The
/// keys are `dynamic` because when JSON is deserialized from method channels
/// it arrives as `Map<dynamic, dynamic>`.
// TODO(yjbanov): use Json typedef when type aliases are shipped
extension JsonExtensions on Map<dynamic, dynamic> {
  Map<String, dynamic> readJson(String propertyName) {
    return this[propertyName] as Map<String, dynamic>;
  }

  Map<String, dynamic>? tryJson(String propertyName) {
    return this[propertyName] as Map<String, dynamic>?;
  }

  Map<dynamic, dynamic> readDynamicJson(String propertyName) {
    return this[propertyName] as Map<dynamic, dynamic>;
  }

  Map<dynamic, dynamic>? tryDynamicJson(String propertyName) {
    return this[propertyName] as Map<dynamic, dynamic>?;
  }

  List<dynamic> readList(String propertyName) {
    return this[propertyName] as List<dynamic>;
  }

  List<dynamic>? tryList(String propertyName) {
    return this[propertyName] as List<dynamic>?;
  }

  List<T> castList<T>(String propertyName) {
    return (this[propertyName] as List<dynamic>).cast<T>();
  }

  List<T>? tryCastList<T>(String propertyName) {
    final List<dynamic>? rawList = tryList(propertyName);
    if (rawList == null) {
      return null;
    }
    return rawList.cast<T>();
  }

  String readString(String propertyName) {
    return this[propertyName] as String;
  }

  String? tryString(String propertyName) {
    return this[propertyName] as String?;
  }

  bool readBool(String propertyName) {
    return this[propertyName] as bool;
  }

  bool? tryBool(String propertyName) {
    return this[propertyName] as bool?;
  }

  int readInt(String propertyName) {
    return (this[propertyName] as num).toInt();
  }

  int? tryInt(String propertyName) {
    return (this[propertyName] as num?)?.toInt();
  }

  double readDouble(String propertyName) {
    return (this[propertyName] as num).toDouble();
  }

  double? tryDouble(String propertyName) {
    return (this[propertyName] as num?)?.toDouble();
  }
}

/// Extracts view ID from the [MethodCall.arguments] map.
///
/// Throws if the view ID is not present or if [arguments] is not a map.
int readViewId(Object? arguments) {
  final int? viewId = tryViewId(arguments);
  if (viewId == null) {
    throw Exception('Could not find a `viewId` in the arguments: $arguments');
  }
  return viewId;
}

/// Extracts view ID from the [MethodCall.arguments] map.
///
/// Returns null if the view ID is not present or if [arguments] is not a map.
int? tryViewId(Object? arguments) {
  if (arguments is Map) {
    return arguments.tryInt('viewId');
  }
  return null;
}

/// Prints a list of bytes in hex format.
///
/// Bytes are separated by one space and are padded on the left to always show
/// two digits.
///
/// Example:
///
///     Input: [0, 1, 2, 3]
///     Output: 0x00 0x01 0x02 0x03
String bytesToHexString(List<int> data) {
  return data.map((int byte) => '0x${byte.toRadixString(16).padLeft(2, '0')}').join(' ');
}

/// Sets a style property on [element].
///
/// [name] is the name of the property. [value] is the value of the property.
/// If [value] is null, removes the style property.
void setElementStyle(DomElement element, String name, String? value) {
  if (value == null) {
    element.style.removeProperty(name);
  } else {
    element.style.setProperty(name, value);
  }
}

void setClipPath(DomElement element, String? value) {
  if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
    if (value == null) {
      element.style.removeProperty('-webkit-clip-path');
    } else {
      element.style.setProperty('-webkit-clip-path', value);
    }
  }
  if (value == null) {
    element.style.removeProperty('clip-path');
  } else {
    element.style.setProperty('clip-path', value);
  }
}

void setThemeColor(ui.Color? color) {
  DomHTMLMetaElement? theme = domDocument.querySelector('#flutterweb-theme') as DomHTMLMetaElement?;

  if (color != null) {
    if (theme == null) {
      theme =
          createDomHTMLMetaElement()
            ..id = 'flutterweb-theme'
            ..name = 'theme-color';
      domDocument.head!.append(theme);
    }
    theme.content = color.toCssString();
  } else {
    theme?.remove();
  }
}

/// Ensure a "meta" tag with [name] and [content] is set on the page.
void ensureMetaTag(String name, String content) {
  final DomElement? existingTag = domDocument.querySelector('meta[name=$name][content=$content]');

  if (existingTag == null) {
    final DomHTMLMetaElement meta =
        createDomHTMLMetaElement()
          ..name = name
          ..content = content;
    domDocument.head!.append(meta);
  }
}

bool? _ellipseFeatureDetected;

/// Draws CanvasElement ellipse with fallback.
void drawEllipse(
  DomCanvasRenderingContext2D context,
  double centerX,
  double centerY,
  double radiusX,
  double radiusY,
  double rotation,
  double startAngle,
  double endAngle,
  bool antiClockwise,
) {
  _ellipseFeatureDetected ??= getJsProperty<Object?>(context, 'ellipse') != null;
  if (_ellipseFeatureDetected!) {
    context.ellipse(
      centerX,
      centerY,
      radiusX,
      radiusY,
      rotation,
      startAngle,
      endAngle,
      antiClockwise,
    );
  } else {
    context.save();
    context.translate(centerX, centerY);
    context.rotate(rotation);
    context.scale(radiusX, radiusY);
    context.arc(0, 0, 1, startAngle, endAngle, antiClockwise);
    context.restore();
  }
}

/// Removes all children of a DOM node.
void removeAllChildren(DomNode node) {
  while (node.lastChild != null) {
    node.lastChild!.remove();
  }
}

/// A helper that finds an element in an iterable that satisfy a predicate, or
/// returns null otherwise.
///
/// This is mostly useful for iterables containing non-null elements.
extension FirstWhereOrNull<T> on Iterable<T> {
  T? firstWhereOrNull(bool Function(T element) test) {
    for (final T element in this) {
      if (test(element)) {
        return element;
      }
    }
    return null;
  }
}

typedef _LruCacheEntry<K extends Object, V extends Object> = ({K key, V value});

/// Caches up to a [maximumSize] key-value pairs.
///
/// Call [cache] to cache a key-value pair.
class LruCache<K extends Object, V extends Object> {
  LruCache(this.maximumSize);

  /// The maximum number of key/value pairs this cache can contain.
  ///
  /// To avoid exceeding this limit the cache remove least recently used items.
  final int maximumSize;

  /// A doubly linked list of the objects in the cache.
  ///
  /// This makes it fast to move a recently used object to the front.
  final DoubleLinkedQueue<_LruCacheEntry<K, V>> _itemQueue =
      DoubleLinkedQueue<_LruCacheEntry<K, V>>();

  @visibleForTesting
  DoubleLinkedQueue<_LruCacheEntry<K, V>> get debugItemQueue => _itemQueue;

  /// A map of objects to their associated node in the [_itemQueue].
  ///
  /// This makes it fast to find the node in the queue when we need to
  /// move the object to the front of the queue.
  final Map<K, DoubleLinkedQueueEntry<_LruCacheEntry<K, V>>> _itemMap =
      <K, DoubleLinkedQueueEntry<_LruCacheEntry<K, V>>>{};

  @visibleForTesting
  Map<K, DoubleLinkedQueueEntry<_LruCacheEntry<K, V>>> get itemMap => _itemMap;

  /// The number of objects in the cache.
  int get length => _itemQueue.length;

  /// Whether or not [object] is in the cache.
  ///
  /// This is only for testing.
  @visibleForTesting
  bool debugContainsValue(V object) {
    return _itemMap.containsValue(object);
  }

  @visibleForTesting
  bool debugContainsKey(K key) {
    return _itemMap.containsKey(key);
  }

  /// Returns the cached value associated with the [key].
  ///
  /// If the value is not in the cache, returns null.
  V? operator [](K key) {
    return _itemMap[key]?.element.value;
  }

  /// Caches the given [key]/[value] pair in this cache.
  ///
  /// If the pair is not already in the cache, adds it to the cache as the most
  /// recently used pair.
  ///
  /// If the [key] is already in the cache, moves it to the most recently used
  /// position. If the [value] corresponding to the [key] is different from
  /// what's in the cache, updates the value.
  void cache(K key, V value) {
    final DoubleLinkedQueueEntry<_LruCacheEntry<K, V>>? item = _itemMap[key];
    if (item == null) {
      // New key-value pair, just add.
      _add(key, value);
    } else if (item.element.value != value) {
      // Key already in the cache, but value is new. Re-add.
      item.remove();
      _add(key, value);
    } else {
      // Key-value pair already in the cache, move to most recently used.
      item.remove();
      _itemQueue.addFirst(item.element);
      _itemMap[key] = _itemQueue.firstEntry()!;
    }
  }

  void clear() {
    _itemQueue.clear();
    _itemMap.clear();
  }

  void _add(K key, V value) {
    _itemQueue.addFirst((key: key, value: value));
    _itemMap[key] = _itemQueue.firstEntry()!;

    if (_itemQueue.length > maximumSize) {
      _removeLeastRecentlyUsedValue();
    }
  }

  void _removeLeastRecentlyUsedValue() {
    final bool didRemove = _itemMap.remove(_itemQueue.last.key) != null;
    assert(didRemove);
    _itemQueue.removeLast();
  }
}

/// Returns the VM-compatible string for the tile mode.
String tileModeString(ui.TileMode? tileMode) {
  switch (tileMode) {
    case ui.TileMode.clamp:
      return 'clamp';
    case ui.TileMode.mirror:
      return 'mirror';
    case ui.TileMode.repeated:
      return 'repeated';
    case ui.TileMode.decal:
      return 'decal';
    case null:
      return 'unspecified';
  }
}

/// A size where both the width and height are integers.
class BitmapSize {
  const BitmapSize(this.width, this.height);

  /// Returns a [BitmapSize] by rounding the width and height of a [ui.Size] to
  /// the nearest integer.
  BitmapSize.fromSize(ui.Size size) : width = size.width.round(), height = size.height.round();

  final int width;
  final int height;

  @override
  bool operator ==(Object other) {
    return other is BitmapSize && other.width == width && other.height == height;
  }

  @override
  int get hashCode => Object.hash(width, height);

  ui.Size toSize() {
    return ui.Size(width.toDouble(), height.toDouble());
  }

  bool get isEmpty => width == 0 || height == 0;

  static const BitmapSize zero = BitmapSize(0, 0);
}
