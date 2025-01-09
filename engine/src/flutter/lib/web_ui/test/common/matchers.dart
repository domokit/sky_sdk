// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Provides utilities for testing engine code.
library matchers;

import 'dart:math' as math;

import 'package:html/dom.dart' as html;
import 'package:html/parser.dart' as html;

import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

/// The epsilon of tolerable double precision error.
///
/// This is used in various places in the framework to allow for floating point
/// precision loss in calculations. Differences below this threshold are safe
/// to disregard.
const double precisionErrorTolerance = 1e-10;

/// Enumerates all persisted surfaces in the tree rooted at [root].
///
/// If [root] is `null` returns all surfaces from the last rendered scene.
///
/// Surfaces are returned in a depth-first order.
Iterable<PersistedSurface> enumerateSurfaces([PersistedSurface? root]) {
  root ??= SurfaceSceneBuilder.debugLastFrameScene;
  final List<PersistedSurface> surfaces = <PersistedSurface>[root!];

  root.visitChildren((PersistedSurface surface) {
    surfaces.addAll(enumerateSurfaces(surface));
  });

  return surfaces;
}

/// Enumerates all pictures nested under [root].
///
/// If [root] is `null` returns all pictures from the last rendered scene.
Iterable<PersistedPicture> enumeratePictures([PersistedSurface? root]) {
  root ??= SurfaceSceneBuilder.debugLastFrameScene;
  return enumerateSurfaces(root).whereType<PersistedPicture>();
}

/// Enumerates all offset surfaces nested under [root].
///
/// If [root] is `null` returns all pictures from the last rendered scene.
Iterable<PersistedOffset> enumerateOffsets([PersistedSurface? root]) {
  root ??= SurfaceSceneBuilder.debugLastFrameScene;
  return enumerateSurfaces(root).whereType<PersistedOffset>();
}

/// Computes the distance between two values.
///
/// The distance should be a metric in a metric space (see
/// https://en.wikipedia.org/wiki/Metric_space). Specifically, if `f` is a
/// distance function then the following conditions should hold:
///
/// - f(a, b) >= 0
/// - f(a, b) == 0 if and only if a == b
/// - f(a, b) == f(b, a)
/// - f(a, c) <= f(a, b) + f(b, c), known as triangle inequality
///
/// This makes it useful for comparing numbers, [Color]s, [Offset]s and other
/// sets of value for which a metric space is defined.
typedef DistanceFunction<T> = double Function(T a, T b);

/// The type of a union of instances of [DistanceFunction<T>] for various types
/// T.
///
/// This type is used to describe a collection of [DistanceFunction<T>]
/// functions which have (potentially) unrelated argument types. Since the
/// argument types of the functions may be unrelated, the only thing that the
/// type system can statically assume about them is that they accept null (since
/// all types in Dart are nullable).
///
/// Calling an instance of this type must either be done dynamically, or by
/// first casting it to a [DistanceFunction<T>] for some concrete T.
typedef AnyDistanceFunction = double Function(Never a, Never b);

const Map<Type, AnyDistanceFunction> _kStandardDistanceFunctions = <Type, AnyDistanceFunction>{
  Color: _maxComponentColorDistance,
  Offset: _offsetDistance,
  int: _intDistance,
  double: _doubleDistance,
  Rect: _rectDistance,
  Size: _sizeDistance,
};

double _intDistance(int a, int b) => (b - a).abs().toDouble();
double _doubleDistance(double a, double b) => (b - a).abs();
double _offsetDistance(Offset a, Offset b) => (b - a).distance;

double _maxComponentColorDistance(Color a, Color b) {
  int delta = math.max<int>((a.red - b.red).abs(), (a.green - b.green).abs());
  delta = math.max<int>(delta, (a.blue - b.blue).abs());
  delta = math.max<int>(delta, (a.alpha - b.alpha).abs());
  return delta.toDouble();
}

double _rectDistance(Rect a, Rect b) {
  double delta = math.max<double>((a.left - b.left).abs(), (a.top - b.top).abs());
  delta = math.max<double>(delta, (a.right - b.right).abs());
  delta = math.max<double>(delta, (a.bottom - b.bottom).abs());
  return delta;
}

double _sizeDistance(Size a, Size b) {
  final Offset delta = (b - a) as Offset; // ignore: unnecessary_parenthesis
  return delta.distance;
}

/// Asserts that two values are within a certain distance from each other.
///
/// The distance is computed by a [DistanceFunction].
///
/// If `distanceFunction` is null, a standard distance function is used for the
/// type `T` . Standard functions are defined for the following types:
///
///  * [Color], whose distance is the maximum component-wise delta.
///  * [Offset], whose distance is the Euclidean distance computed using the
///    method [Offset.distance].
///  * [Rect], whose distance is the maximum component-wise delta.
///  * [Size], whose distance is the [Offset.distance] of the offset computed as
///    the difference between two sizes.
///  * [int], whose distance is the absolute difference between two integers.
///  * [double], whose distance is the absolute difference between two doubles.
///
/// See also:
///
///  * [moreOrLessEquals], which is similar to this function, but specializes in
///    [double]s and has an optional `epsilon` parameter.
///  * [closeTo], which specializes in numbers only.
Matcher within<T>({
  required T from,
  double distance = precisionErrorTolerance,
  DistanceFunction<T>? distanceFunction,
}) {
  distanceFunction ??= _kStandardDistanceFunctions[T] as DistanceFunction<T>?;

  if (distanceFunction == null) {
    throw ArgumentError(
      'The specified distanceFunction was null, and a standard distance '
      'function was not found for type $T of the provided '
      '`from` argument.',
    );
  }

  return _IsWithinDistance<T>(distanceFunction, from, distance);
}

class _IsWithinDistance<T> extends Matcher {
  const _IsWithinDistance(this.distanceFunction, this.value, this.epsilon);

  final DistanceFunction<T> distanceFunction;
  final T value;
  final double epsilon;

  @override
  bool matches(Object? object, Map<dynamic, dynamic> matchState) {
    if (object is! T) {
      return false;
    }
    if (object == value) {
      return true;
    }
    final T test = object;
    final double distance = distanceFunction(test, value);
    if (distance < 0) {
      throw ArgumentError(
        'Invalid distance function was used to compare a ${value.runtimeType} '
        'to a ${object.runtimeType}. The function must return a non-negative '
        'double value, but it returned $distance.',
      );
    }
    matchState['distance'] = distance;
    return distance <= epsilon;
  }

  @override
  Description describe(Description description) => description.add('$value (±$epsilon)');

  @override
  Description describeMismatch(
    Object? object,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    mismatchDescription.add('was ${matchState['distance']} away from the desired value.');
    return mismatchDescription;
  }
}

/// A matcher for functions that throw [AssertionError].
///
/// This is equivalent to `throwsA(isInstanceOf<AssertionError>())`.
///
/// If you are trying to test whether a call to [WidgetTester.pumpWidget]
/// results in an [AssertionError], see
/// [TestWidgetsFlutterBinding.takeException].
///
/// See also:
///
///  * [throwsFlutterError], to test if a function throws a [FlutterError].
///  * [throwsArgumentError], to test if a functions throws an [ArgumentError].
///  * [isAssertionError], to test if any object is any kind of [AssertionError].
final Matcher throwsAssertionError = throwsA(isAssertionError);

/// A matcher for [AssertionError].
///
/// This is equivalent to `isInstanceOf<AssertionError>()`.
///
/// See also:
///
///  * [throwsAssertionError], to test if a function throws any [AssertionError].
///  * [isFlutterError], to test if any object is a [FlutterError].
const Matcher isAssertionError = TypeMatcher<AssertionError>();

/// Matches a [DomElement] against an HTML pattern.
///
/// An HTML pattern is a piece of valid HTML. The expectation is that the DOM
/// element has the exact element structure as the provided [htmlPattern]. The
/// DOM element is expected to have the exact element and style attributes
/// specified in the pattern.
///
/// The DOM element may have additional attributes not specified in the pattern.
/// This allows testing specific features relevant to the test.
///
/// The DOM structure may not have additional elements that are not specified in
/// the pattern.
Matcher hasHtml(String htmlPattern) {
  final html.DocumentFragment originalDom = html.parseFragment(htmlPattern);
  if (originalDom.children.isEmpty) {
    fail(
      'Test HTML pattern is empty.\n'
      'The pattern must contain exacly one top-level element, but was: $htmlPattern',
    );
  }
  if (originalDom.children.length > 1) {
    fail(
      'Test HTML pattern has more than one top-level element.\n'
      'The pattern must contain exacly one top-level element, but was: $htmlPattern',
    );
  }
  return HtmlPatternMatcher(originalDom.children.single);
}

enum _Breadcrumb { root, element, attribute, styleProperty }

class _Breadcrumbs {
  const _Breadcrumbs._(this.parent, this.kind, this.name);

  final _Breadcrumbs? parent;
  final _Breadcrumb kind;
  final String name;

  static const _Breadcrumbs root = _Breadcrumbs._(null, _Breadcrumb.root, '');

  _Breadcrumbs element(String tagName) {
    return _Breadcrumbs._(this, _Breadcrumb.element, tagName);
  }

  _Breadcrumbs attribute(String attributeName) {
    return _Breadcrumbs._(this, _Breadcrumb.attribute, attributeName);
  }

  _Breadcrumbs styleProperty(String propertyName) {
    return _Breadcrumbs._(this, _Breadcrumb.styleProperty, propertyName);
  }

  @override
  String toString() {
    return switch (kind) {
      _Breadcrumb.root => '<root>',
      _Breadcrumb.element => parent!.kind == _Breadcrumb.root ? '@$name' : '$parent > $name',
      _Breadcrumb.attribute => '$parent#$name',
      _Breadcrumb.styleProperty => '$parent#style($name)',
    };
  }
}

class HtmlPatternMatcher extends Matcher {
  const HtmlPatternMatcher(this.pattern);

  final html.Element pattern;

  @override
  bool matches(final Object? object, Map<Object?, Object?> matchState) {
    if (object is! DomElement) {
      return false;
    }

    final List<String> mismatches = <String>[];
    matchState['mismatches'] = mismatches;

    final html.Element element = html.parseFragment(object.outerHTML).children.single;
    matchElements(_Breadcrumbs.root, mismatches, element, pattern);
    return mismatches.isEmpty;
  }

  static bool _areTagsEqual(html.Element a, html.Element b) {
    const Map<String, String> synonyms = <String, String>{
      'sem': 'flt-semantics',
      'sem-c': 'flt-semantics-container',
      'sem-img': 'flt-semantics-img',
      'sem-tf': 'flt-semantics-text-field',
    };

    String aName = a.localName!.toLowerCase();
    String bName = b.localName!.toLowerCase();

    if (synonyms.containsKey(aName)) {
      aName = synonyms[aName]!;
    }

    if (synonyms.containsKey(bName)) {
      bName = synonyms[bName]!;
    }

    return aName == bName;
  }

  void matchElements(
    _Breadcrumbs parent,
    List<String> mismatches,
    html.Element element,
    html.Element pattern,
  ) {
    final _Breadcrumbs breadcrumb = parent.element(pattern.localName!);

    if (!_areTagsEqual(element, pattern)) {
      mismatches.add(
        '$breadcrumb: unexpected tag name <${element.localName}> (expected <${pattern.localName}>).',
      );
      // Don't bother matching anything else. If tags are different, it's likely
      // we're comparing apples to oranges at this point.
      return;
    }

    matchAttributes(breadcrumb, mismatches, element, pattern);
    matchChildren(breadcrumb, mismatches, element, pattern);
  }

  void matchAttributes(
    _Breadcrumbs parent,
    List<String> mismatches,
    html.Element element,
    html.Element pattern,
  ) {
    for (final MapEntry<Object, String> attribute in pattern.attributes.entries) {
      final String expectedName = attribute.key as String;
      final String expectedValue = attribute.value;
      final _Breadcrumbs breadcrumb = parent.attribute(expectedName);

      if (expectedName == 'style') {
        // Style is a complex attribute that deserves a special comparison algorithm.
        matchStyle(parent, mismatches, element, pattern);
      } else {
        if (!element.attributes.containsKey(expectedName)) {
          mismatches.add('$breadcrumb: attribute $expectedName="$expectedValue" missing.');
        } else {
          final String? actualValue = element.attributes[expectedName];
          if (actualValue != expectedValue) {
            mismatches.add(
              '$breadcrumb: expected attribute value $expectedName="$expectedValue", '
              'but found $expectedName="$actualValue".',
            );
          }
        }
      }
    }
  }

  static Map<String, String> parseStyle(html.Element element) {
    final Map<String, String> result = <String, String>{};

    final String rawStyle = element.attributes['style']!;
    for (final String attribute in rawStyle.split(';')) {
      final List<String> parts = attribute.split(':');
      final String name = parts[0].trim();
      final String value = parts.skip(1).join(':').trim();
      result[name] = value;
    }

    return result;
  }

  void matchStyle(
    _Breadcrumbs parent,
    List<String> mismatches,
    html.Element element,
    html.Element pattern,
  ) {
    final Map<String, String> expected = parseStyle(pattern);
    final Map<String, String> actual = parseStyle(element);
    for (final MapEntry<String, String> entry in expected.entries) {
      final _Breadcrumbs breadcrumb = parent.styleProperty(entry.key);
      if (!actual.containsKey(entry.key)) {
        mismatches.add('$breadcrumb: style property ${entry.key}="${entry.value}" missing.');
      } else if (actual[entry.key] != entry.value) {
        mismatches.add(
          '$breadcrumb: expected style property ${entry.key}="${entry.value}", '
          'but found ${entry.key}="${actual[entry.key]}".',
        );
      }
    }
  }

  // Removes nodes that are not interesting for comparison purposes.
  //
  // In particular, removes non-leaf white space Text nodes between elements, as
  // these are typically not interesting to test for. It's strictly not correct
  // to ignore it entirely. For example, in the presence of a <pre> tag or CSS
  // `white-space: pre` white space does matter, but Flutter Web doesn't use
  // them, at least not in tests, so it's OK to ignore.
  List<html.Node> _cleanUpNodeList(html.NodeList nodeList) {
    final List<html.Node> cleanNodes = <html.Node>[];
    for (int i = 0; i < nodeList.length; i++) {
      final html.Node node = nodeList[i];
      assert(
        node is html.Element || node is html.Text,
        'Unsupported node type ${node.runtimeType}. Only Element and Text nodes are supported',
      );

      final bool hasSiblings = nodeList.length > 1;
      final bool isWhitespace = node is html.Text && node.data.trim().isEmpty;

      if (hasSiblings && isWhitespace) {
        // Ignore white space between elements, e.g. <div> <div>   </div> </div>
        //                                                |      |       |
        //                                              ignore   |       |
        //                                                       |       |
        //                                                    compare    |
        //                                                             ignore
        continue;
      }

      cleanNodes.add(node);
    }
    return cleanNodes;
  }

  void matchChildren(
    _Breadcrumbs parent,
    List<String> mismatches,
    html.Element element,
    html.Element pattern,
  ) {
    final List<html.Node> actualChildNodes = _cleanUpNodeList(element.nodes);
    final List<html.Node> expectedChildNodes = _cleanUpNodeList(pattern.nodes);

    if (actualChildNodes.length != expectedChildNodes.length) {
      mismatches.add(
        '$parent: expected ${expectedChildNodes.length} child nodes, but found ${actualChildNodes.length}.',
      );
      return;
    }

    for (int i = 0; i < expectedChildNodes.length; i++) {
      final html.Node expectedChild = expectedChildNodes[i];
      final html.Node actualChild = actualChildNodes[i];

      if (expectedChild is html.Element && actualChild is html.Element) {
        matchElements(parent, mismatches, actualChild, expectedChild);
      } else if (expectedChild is html.Text && actualChild is html.Text) {
        if (expectedChild.data != actualChild.data) {
          mismatches.add(
            '$parent: expected text content "${expectedChild.data}", but found "${actualChild.data}".',
          );
        }
      } else {
        mismatches.add(
          '$parent: expected child type ${expectedChild.runtimeType}, but found ${actualChild.runtimeType}.',
        );
      }
    }
  }

  @override
  Description describe(Description description) {
    description.add('the element to have the following pattern:\n');
    description.add(pattern.outerHtml);
    return description;
  }

  @override
  Description describeMismatch(
    Object? object,
    Description mismatchDescription,
    Map<Object?, Object?> matchState,
    bool verbose,
  ) {
    if (object == null) {
      mismatchDescription.add('Expected a DOM element, but got null.');
      return mismatchDescription;
    }

    mismatchDescription.add('The following DOM structure did not match the expected pattern:\n');
    mismatchDescription.add('${(object as DomElement).outerHTML!}\n\n');
    mismatchDescription.add('Specifically:\n');

    final List<String> mismatches = matchState['mismatches']! as List<String>;
    for (final String mismatch in mismatches) {
      mismatchDescription.add(' - $mismatch\n');
    }

    return mismatchDescription;
  }
}
