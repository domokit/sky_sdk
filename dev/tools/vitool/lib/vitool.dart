// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:math';

import 'package:collection/collection.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:xml/xml.dart';

// String to use for a single indentation.
const String kIndent = '  ';

/// Represents an animation, and provides logic to generate dart code for it.
class Animation {
  const Animation(this.size, this.paths);

  factory Animation.fromFrameData(List<FrameData> frames) {
    _validateFramesData(frames);
    final Point<double> size = frames[0].size;
    final List<PathAnimation> paths = <PathAnimation>[];
    for (int i = 0; i < frames[0].paths.length; i += 1) {
      paths.add(PathAnimation.fromFrameData(frames, i));
    }
    return Animation(size, paths);
  }

  /// The size of the animation (width, height) in pixels.
  final Point<double> size;

  /// List of paths in the animation.
  final List<PathAnimation> paths;

  static void _validateFramesData(List<FrameData> frames) {
    final Point<double> size = frames[0].size;
    final int numPaths = frames[0].paths.length;
    for (int i = 0; i < frames.length; i += 1) {
      final FrameData frame = frames[i];
      if (size != frame.size) {
        throw Exception(
          'All animation frames must have the same size,\n'
          'first frame size was: (${size.x}, ${size.y})\n'
          'frame $i size was: (${frame.size.x}, ${frame.size.y})',
        );
      }
      if (numPaths != frame.paths.length) {
        throw Exception(
          'All animation frames must have the same number of paths,\n'
          'first frame has $numPaths paths\n'
          'frame $i has ${frame.paths.length} paths',
        );
      }
    }
  }

  String toDart(String className, String varName) {
    final StringBuffer sb = StringBuffer();
    sb.write('const $className $varName = const $className(\n');
    sb.write('${kIndent}const Size(${size.x}, ${size.y}),\n');
    sb.write('${kIndent}const <_PathFrames>[\n');
    for (final PathAnimation path in paths) {
      sb.write(path.toDart());
    }
    sb.write('$kIndent],\n');
    sb.write(');');
    return sb.toString();
  }
}

/// Represents the animation of a single path.
class PathAnimation {
  const PathAnimation(this.commands, {required this.opacities});

  factory PathAnimation.fromFrameData(List<FrameData> frames, int pathIdx) {
    if (frames.isEmpty) {
      return const PathAnimation(<PathCommandAnimation>[], opacities: <double>[]);
    }

    final List<PathCommandAnimation> commands = <PathCommandAnimation>[];
    for (
      int commandIdx = 0;
      commandIdx < frames[0].paths[pathIdx].commands.length;
      commandIdx += 1
    ) {
      final int numPointsInCommand = frames[0].paths[pathIdx].commands[commandIdx].points.length;
      final List<List<Point<double>>> points = List<List<Point<double>>>.filled(
        numPointsInCommand,
        <Point<double>>[],
      );
      final String commandType = frames[0].paths[pathIdx].commands[commandIdx].type;
      for (int i = 0; i < frames.length; i += 1) {
        final FrameData frame = frames[i];
        final String currentCommandType = frame.paths[pathIdx].commands[commandIdx].type;
        if (commandType != currentCommandType) {
          throw Exception(
            'Paths must be built from the same commands in all frames '
            "command $commandIdx at frame 0 was of type '$commandType' "
            "command $commandIdx at frame $i was of type '$currentCommandType'",
          );
        }
        for (int j = 0; j < numPointsInCommand; j += 1) {
          points[j].add(frame.paths[pathIdx].commands[commandIdx].points[j]);
        }
      }
      commands.add(PathCommandAnimation(commandType, points));
    }

    final List<double> opacities =
        frames.map<double>((FrameData d) => d.paths[pathIdx].opacity).toList();

    return PathAnimation(commands, opacities: opacities);
  }

  /// List of commands for drawing the path.
  final List<PathCommandAnimation> commands;

  /// The path opacity for each animation frame.
  final List<double> opacities;

  @override
  String toString() {
    return 'PathAnimation(commands: $commands, opacities: $opacities)';
  }

  String toDart() {
    final StringBuffer sb = StringBuffer();
    sb.write('${kIndent * 2}const _PathFrames(\n');
    sb.write('${kIndent * 3}opacities: const <double>[\n');
    for (final double opacity in opacities) {
      sb.write('${kIndent * 4}$opacity,\n');
    }
    sb.write('${kIndent * 3}],\n');
    sb.write('${kIndent * 3}commands: const <_PathCommand>[\n');
    for (final PathCommandAnimation command in commands) {
      sb.write(command.toDart());
    }
    sb.write('${kIndent * 3}],\n');
    sb.write('${kIndent * 2}),\n');
    return sb.toString();
  }
}

/// Represents the animation of a single path command.
class PathCommandAnimation {
  const PathCommandAnimation(this.type, this.points);

  /// The command type.
  final String type;

  /// A matrix with the command's points in different frames.
  ///
  /// points[i][j] is the i-th point of the command at frame j.
  final List<List<Point<double>>> points;

  @override
  String toString() {
    return 'PathCommandAnimation(type: $type, points: $points)';
  }

  String toDart() {
    final String dartCommandClass = switch (type) {
      'M' => '_PathMoveTo',
      'C' => '_PathCubicTo',
      'L' => '_PathLineTo',
      'Z' => '_PathClose',
      _ => throw Exception('unsupported path command: $type'),
    };
    final StringBuffer sb = StringBuffer();
    sb.write('${kIndent * 4}const $dartCommandClass(\n');
    for (final List<Point<double>> pointFrames in points) {
      sb.write('${kIndent * 5}const <Offset>[\n');
      for (final Point<double> point in pointFrames) {
        sb.write('${kIndent * 6}const Offset(${point.x}, ${point.y}),\n');
      }
      sb.write('${kIndent * 5}],\n');
    }
    sb.write('${kIndent * 4}),\n');
    return sb.toString();
  }
}

/// Interprets some subset of an SVG file.
///
/// Recursively goes over the SVG tree, applying transforms and opacities,
/// and build a FrameData which is a flat representation of the paths in the SVG
/// file, after applying transformations and converting relative coordinates to
/// absolute.
///
/// This does not support the SVG specification, but is just built to
/// support SVG files exported by a specific tool the motion design team is
/// using.
FrameData interpretSvg(String svgFilePath) {
  final File file = File(svgFilePath);
  final String fileData = file.readAsStringSync();
  final XmlElement svgElement = _extractSvgElement(XmlDocument.parse(fileData));
  final double width = parsePixels(_extractAttr(svgElement, 'width')).toDouble();
  final double height = parsePixels(_extractAttr(svgElement, 'height')).toDouble();

  final List<SvgPath> paths = _interpretSvgGroup(svgElement.children, _Transform());
  return FrameData(Point<double>(width, height), paths);
}

List<SvgPath> _interpretSvgGroup(List<XmlNode> children, _Transform transform) {
  final List<SvgPath> paths = <SvgPath>[];
  for (final XmlNode node in children) {
    if (node.nodeType != XmlNodeType.ELEMENT) {
      continue;
    }
    final XmlElement element = node as XmlElement;

    if (element.name.local == 'path') {
      paths.add(SvgPath.fromElement(element)._applyTransform(transform));
    }

    if (element.name.local == 'g') {
      double opacity = transform.opacity;
      if (_hasAttr(element, 'opacity')) {
        opacity *= double.parse(_extractAttr(element, 'opacity'));
      }

      Matrix3 transformMatrix = transform.transformMatrix;
      if (_hasAttr(element, 'transform')) {
        transformMatrix = transformMatrix.multiplied(
          _parseSvgTransform(_extractAttr(element, 'transform')),
        );
      }

      final _Transform subtreeTransform = _Transform(
        transformMatrix: transformMatrix,
        opacity: opacity,
      );
      paths.addAll(_interpretSvgGroup(element.children, subtreeTransform));
    }
  }
  return paths;
}

// Given a points list in the form e.g: "25.0, 1.0 12.0, 12.0 23.0, 9.0" matches
// the coordinated of the first point and the rest of the string, for the
// example above:
// group 1 will match "25.0"
// group 2 will match "1.0"
// group 3 will match "12.0, 12.0 23.0, 9.0"
//
// Commas are optional.
final RegExp _pointMatcher = RegExp(r'^ *([\-\.0-9]+) *,? *([\-\.0-9]+)(.*)');

/// Parse a string with a list of points, e.g:
/// '25.0, 1.0 12.0, 12.0 23.0, 9.0' will be parsed to:
/// [Point(25.0, 1.0), Point(12.0, 12.0), Point(23.0, 9.0)].
///
/// Commas are optional.
List<Point<double>> parsePoints(String points) {
  String unParsed = points;
  final List<Point<double>> result = <Point<double>>[];
  while (unParsed.isNotEmpty && _pointMatcher.hasMatch(unParsed)) {
    final Match m = _pointMatcher.firstMatch(unParsed)!;
    result.add(Point<double>(double.parse(m.group(1)!), double.parse(m.group(2)!)));
    unParsed = m.group(3)!;
  }
  return result;
}

/// Data for a single animation frame.
@immutable
class FrameData {
  const FrameData(this.size, this.paths);

  final Point<double> size;
  final List<SvgPath> paths;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FrameData &&
        other.size == size &&
        const ListEquality<SvgPath>().equals(other.paths, paths);
  }

  @override
  int get hashCode => Object.hash(size, Object.hashAll(paths));

  @override
  String toString() {
    return 'FrameData(size: $size, paths: $paths)';
  }
}

/// Represents an SVG path element.
@immutable
class SvgPath {
  const SvgPath(this.id, this.commands, {this.opacity = 1.0});

  final String id;
  final List<SvgPathCommand> commands;
  final double opacity;

  static const String _pathCommandAtom = r' *([a-zA-Z]) *([\-\.0-9 ,]*)';
  static final RegExp _pathCommandValidator = RegExp('^($_pathCommandAtom)*\$');
  static final RegExp _pathCommandMatcher = RegExp(_pathCommandAtom);

  static SvgPath fromElement(XmlElement pathElement) {
    assert(pathElement.name.local == 'path');
    final String id = _extractAttr(pathElement, 'id');
    final String dAttr = _extractAttr(pathElement, 'd');
    final List<SvgPathCommand> commands = <SvgPathCommand>[];
    final SvgPathCommandBuilder commandsBuilder = SvgPathCommandBuilder();
    if (!_pathCommandValidator.hasMatch(dAttr)) {
      throw Exception('illegal or unsupported path d expression: $dAttr');
    }
    for (final Match match in _pathCommandMatcher.allMatches(dAttr)) {
      final String commandType = match.group(1)!;
      final String pointStr = match.group(2)!;
      commands.add(commandsBuilder.build(commandType, parsePoints(pointStr)));
    }
    return SvgPath(id, commands);
  }

  SvgPath _applyTransform(_Transform transform) {
    final List<SvgPathCommand> transformedCommands =
        commands.map<SvgPathCommand>((SvgPathCommand c) => c._applyTransform(transform)).toList();
    return SvgPath(id, transformedCommands, opacity: opacity * transform.opacity);
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SvgPath &&
        other.id == id &&
        other.opacity == opacity &&
        const ListEquality<SvgPathCommand>().equals(other.commands, commands);
  }

  @override
  int get hashCode => Object.hash(id, Object.hashAll(commands), opacity);

  @override
  String toString() {
    return 'SvgPath(id: $id, opacity: $opacity, commands: $commands)';
  }
}

/// Represents a single SVG path command from an SVG d element.
///
/// This class normalizes all the 'd' commands into a single type, that has
/// a command type and a list of points.
///
/// Some examples of how d commands translated to SvgPathCommand:
///   * "M 0.0, 1.0" => SvgPathCommand('M', [Point(0.0, 1.0)])
///   * "Z" => SvgPathCommand('Z', [])
///   * "C 1.0, 1.0 2.0, 2.0 3.0, 3.0" SvgPathCommand('C', [Point(1.0, 1.0),
///      Point(2.0, 2.0), Point(3.0, 3.0)])
@immutable
class SvgPathCommand {
  const SvgPathCommand(this.type, this.points);

  /// The command type.
  final String type;

  /// List of points used by this command.
  final List<Point<double>> points;

  SvgPathCommand _applyTransform(_Transform transform) {
    final List<Point<double>> transformedPoints = _vector3ArrayToPoints(
      transform.transformMatrix.applyToVector3Array(_pointsToVector3Array(points)),
    );
    return SvgPathCommand(type, transformedPoints);
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SvgPathCommand &&
        other.type == type &&
        const ListEquality<Point<double>>().equals(other.points, points);
  }

  @override
  int get hashCode => Object.hash(type, Object.hashAll(points));

  @override
  String toString() {
    return 'SvgPathCommand(type: $type, points: $points)';
  }
}

class SvgPathCommandBuilder {
  static const Map<String, void> kRelativeCommands = <String, void>{
    'c': null,
    'l': null,
    'm': null,
    't': null,
    's': null,
  };

  Point<double> lastPoint = const Point<double>(0.0, 0.0);
  Point<double> subPathStartPoint = const Point<double>(0.0, 0.0);

  SvgPathCommand build(String type, List<Point<double>> points) {
    List<Point<double>> absPoints = points;
    if (_isRelativeCommand(type)) {
      absPoints = points.map<Point<double>>((Point<double> p) => p + lastPoint).toList();
    }

    if (type == 'M' || type == 'm') {
      subPathStartPoint = absPoints.last;
    }

    if (type == 'Z' || type == 'z') {
      lastPoint = subPathStartPoint;
    } else {
      lastPoint = absPoints.last;
    }

    return SvgPathCommand(type.toUpperCase(), absPoints);
  }

  static bool _isRelativeCommand(String type) {
    return kRelativeCommands.containsKey(type);
  }
}

List<double> _pointsToVector3Array(List<Point<double>> points) {
  final List<double> result = List<double>.filled(points.length * 3, 0.0);
  for (int i = 0; i < points.length; i += 1) {
    result[i * 3] = points[i].x;
    result[i * 3 + 1] = points[i].y;
    result[i * 3 + 2] = 1.0;
  }
  return result;
}

List<Point<double>> _vector3ArrayToPoints(List<double> vector) {
  final int numPoints = (vector.length / 3).floor();
  final List<Point<double>> points = <Point<double>>[
    for (int i = 0; i < numPoints; i += 1) Point<double>(vector[i * 3], vector[i * 3 + 1]),
  ];
  return points;
}

/// Represents a transformation to apply on an SVG subtree.
///
/// This includes more transforms than the ones described by the SVG transform
/// attribute, e.g opacity.
class _Transform {
  /// Constructs a new _Transform, default arguments create a no-op transform.
  _Transform({Matrix3? transformMatrix, this.opacity = 1.0})
    : transformMatrix = transformMatrix ?? Matrix3.identity();

  final Matrix3 transformMatrix;
  final double opacity;

  _Transform applyTransform(_Transform transform) {
    return _Transform(
      transformMatrix: transform.transformMatrix.multiplied(transformMatrix),
      opacity: transform.opacity * opacity,
    );
  }
}

const String _transformCommandAtom = r' *([^(]+)\(([^)]*)\)';
final RegExp _transformValidator = RegExp('^($_transformCommandAtom)*\$');
final RegExp _transformCommand = RegExp(_transformCommandAtom);

Matrix3 _parseSvgTransform(String transform) {
  if (!_transformValidator.hasMatch(transform)) {
    throw Exception('illegal or unsupported transform: $transform');
  }
  final Iterable<Match> matches = _transformCommand.allMatches(transform).toList().reversed;
  Matrix3 result = Matrix3.identity();
  for (final Match m in matches) {
    final String command = m.group(1)!;
    final String params = m.group(2)!;
    if (command == 'translate') {
      result = _parseSvgTranslate(params).multiplied(result);
      continue;
    }
    if (command == 'scale') {
      result = _parseSvgScale(params).multiplied(result);
      continue;
    }
    if (command == 'rotate') {
      result = _parseSvgRotate(params).multiplied(result);
      continue;
    }
    throw Exception('unimplemented transform: $command');
  }
  return result;
}

final RegExp _valueSeparator = RegExp('( *, *| +)');

Matrix3 _parseSvgTranslate(String paramsStr) {
  final List<String> params = paramsStr.split(_valueSeparator);
  assert(params.isNotEmpty);
  assert(params.length <= 2);
  final double x = double.parse(params[0]);
  final double y = params.length < 2 ? 0 : double.parse(params[1]);
  return _matrix(1.0, 0.0, 0.0, 1.0, x, y);
}

Matrix3 _parseSvgScale(String paramsStr) {
  final List<String> params = paramsStr.split(_valueSeparator);
  assert(params.isNotEmpty);
  assert(params.length <= 2);
  final double x = double.parse(params[0]);
  final double y = params.length < 2 ? 0 : double.parse(params[1]);
  return _matrix(x, 0.0, 0.0, y, 0.0, 0.0);
}

Matrix3 _parseSvgRotate(String paramsStr) {
  final List<String> params = paramsStr.split(_valueSeparator);
  assert(params.length == 1);
  final double a = radians(double.parse(params[0]));
  return _matrix(cos(a), sin(a), -sin(a), cos(a), 0.0, 0.0);
}

Matrix3 _matrix(double a, double b, double c, double d, double e, double f) {
  return Matrix3(a, b, 0.0, c, d, 0.0, e, f, 1.0);
}

// Matches a pixels expression e.g "14px".
// First group is just the number.
final RegExp _pixelsExp = RegExp(r'^([0-9]+)px$');

/// Parses a pixel expression, e.g "14px", and returns the number.
/// Throws an [ArgumentError] if the given string doesn't match the pattern.
int parsePixels(String pixels) {
  if (!_pixelsExp.hasMatch(pixels)) {
    throw ArgumentError(
      "illegal pixels expression: '$pixels'"
      ' (the tool currently only support pixel units).',
    );
  }
  return int.parse(_pixelsExp.firstMatch(pixels)!.group(1)!);
}

String _extractAttr(XmlElement element, String name) {
  try {
    return element.attributes.singleWhere((XmlAttribute x) => x.name.local == name).value;
  } catch (e) {
    throw ArgumentError(
      "Can't find a single '$name' attributes in ${element.name}, "
      'attributes were: ${element.attributes}',
    );
  }
}

bool _hasAttr(XmlElement element, String name) {
  return element.attributes.where((XmlAttribute a) => a.name.local == name).isNotEmpty;
}

XmlElement _extractSvgElement(XmlDocument document) {
  return document.children.singleWhere(
        (XmlNode node) =>
            node.nodeType == XmlNodeType.ELEMENT && _asElement(node).name.local == 'svg',
      )
      as XmlElement;
}

XmlElement _asElement(XmlNode node) => node as XmlElement;
