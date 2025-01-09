// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// For member documentation see https://api.flutter.dev/flutter/dart-ui/Canvas-class.html

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'dom.dart';
import 'html/painting.dart';
import 'html/render_vertices.dart';
import 'text/canvas_paragraph.dart';
import 'util.dart';
import 'vector_math.dart';

/// Defines canvas interface common across canvases that the [SceneBuilder]
/// renders to.
///
/// This can be used either as an interface or super-class.
abstract class EngineCanvas {
  /// The element that is attached to the DOM.
  DomElement get rootElement;

  void dispose() {
    clear();
  }

  void clear();

  void save();

  void restore();

  void translate(double dx, double dy);

  void scale(double sx, double sy);

  void rotate(double radians);

  void skew(double sx, double sy);

  void transform(Float32List matrix4);

  void clipRect(ui.Rect rect, ui.ClipOp clipOp);

  void clipRRect(ui.RRect rrect);

  void clipPath(ui.Path path);

  void drawColor(ui.Color color, ui.BlendMode blendMode);

  void drawLine(ui.Offset p1, ui.Offset p2, SurfacePaintData paint);

  void drawPaint(SurfacePaintData paint);

  void drawRect(ui.Rect rect, SurfacePaintData paint);

  void drawRRect(ui.RRect rrect, SurfacePaintData paint);

  void drawDRRect(ui.RRect outer, ui.RRect inner, SurfacePaintData paint);

  void drawOval(ui.Rect rect, SurfacePaintData paint);

  void drawCircle(ui.Offset c, double radius, SurfacePaintData paint);

  void drawPath(ui.Path path, SurfacePaintData paint);

  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder);

  void drawImage(ui.Image image, ui.Offset p, SurfacePaintData paint);

  void drawImageRect(ui.Image image, ui.Rect src, ui.Rect dst, SurfacePaintData paint);

  void drawParagraph(CanvasParagraph paragraph, ui.Offset offset);

  void drawVertices(SurfaceVertices vertices, ui.BlendMode blendMode, SurfacePaintData paint);

  void drawPoints(ui.PointMode pointMode, Float32List points, SurfacePaintData paint);

  /// Extension of Canvas API to mark the end of a stream of painting commands
  /// to enable re-use/dispose optimizations.
  void endOfPaint();
}

/// Adds an [offset] transformation to a [transform] matrix and returns the
/// combined result.
///
/// If the given offset is zero, returns [transform] matrix as is. Otherwise,
/// returns a new [Matrix4] object representing the combined transformation.
Matrix4 transformWithOffset(Matrix4 transform, ui.Offset offset) {
  if (offset == ui.Offset.zero) {
    return transform;
  }

  // Clone to avoid mutating transform.
  final Matrix4 effectiveTransform = transform.clone();
  effectiveTransform.translate(offset.dx, offset.dy);
  return effectiveTransform;
}

class SaveStackEntry {
  SaveStackEntry({required this.transform, required this.clipStack});

  final Matrix4 transform;
  final List<SaveClipEntry>? clipStack;
}

/// Tagged union of clipping parameters used for canvas.
class SaveClipEntry {
  SaveClipEntry.rect(this.rect, this.currentTransform) : rrect = null, path = null;
  SaveClipEntry.rrect(this.rrect, this.currentTransform) : rect = null, path = null;
  SaveClipEntry.path(this.path, this.currentTransform) : rect = null, rrect = null;

  final ui.Rect? rect;
  final ui.RRect? rrect;
  final ui.Path? path;
  final Matrix4 currentTransform;
}

/// Provides save stack tracking functionality to implementations of
/// [EngineCanvas].
mixin SaveStackTracking on EngineCanvas {
  final List<SaveStackEntry> _saveStack = <SaveStackEntry>[];

  /// The stack that maintains clipping operations used when text is painted
  /// onto bitmap canvas but is composited as separate element.
  List<SaveClipEntry>? _clipStack;

  /// Returns whether there are active clipping regions on the canvas.
  bool get isClipped => _clipStack != null;

  /// Empties the save stack and the element stack, and resets the transform
  /// and clip parameters.
  ///
  /// Classes that override this method must call `super.clear()`.
  @override
  void clear() {
    _saveStack.clear();
    _clipStack = null;
    _currentTransform = Matrix4.identity();
  }

  /// The current transformation matrix.
  Matrix4 get currentTransform => _currentTransform;
  Matrix4 _currentTransform = Matrix4.identity();

  /// Saves current clip and transform on the save stack.
  ///
  /// Classes that override this method must call `super.save()`.
  @override
  void save() {
    _saveStack.add(
      SaveStackEntry(
        transform: _currentTransform.clone(),
        clipStack: _clipStack == null ? null : List<SaveClipEntry>.from(_clipStack!),
      ),
    );
  }

  /// Restores current clip and transform from the save stack.
  ///
  /// Classes that override this method must call `super.restore()`.
  @override
  void restore() {
    if (_saveStack.isEmpty) {
      return;
    }
    final SaveStackEntry entry = _saveStack.removeLast();
    _currentTransform = entry.transform;
    _clipStack = entry.clipStack;
  }

  /// Multiplies the [currentTransform] matrix by a translation.
  ///
  /// Classes that override this method must call `super.translate()`.
  @override
  void translate(double dx, double dy) {
    _currentTransform.translate(dx, dy);
  }

  /// Scales the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.scale()`.
  @override
  void scale(double sx, double sy) {
    _currentTransform.scale(sx, sy);
  }

  /// Rotates the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.rotate()`.
  @override
  void rotate(double radians) {
    _currentTransform.rotate(kUnitZ, radians);
  }

  /// Skews the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.skew()`.
  @override
  void skew(double sx, double sy) {
    final Matrix4 skewMatrix = Matrix4.identity();
    final Float32List storage = skewMatrix.storage;
    storage[1] = sy;
    storage[4] = sx;
    _currentTransform.multiply(skewMatrix);
  }

  /// Multiplies the [currentTransform] matrix by another matrix.
  ///
  /// Classes that override this method must call `super.transform()`.
  @override
  void transform(Float32List matrix4) {
    _currentTransform.multiply(Matrix4.fromFloat32List(matrix4));
  }

  /// Adds a rectangle to clipping stack.
  ///
  /// Classes that override this method must call `super.clipRect()`.
  @override
  void clipRect(ui.Rect rect, ui.ClipOp op) {
    _clipStack ??= <SaveClipEntry>[];
    _clipStack!.add(SaveClipEntry.rect(rect, _currentTransform.clone()));
  }

  /// Adds a round rectangle to clipping stack.
  ///
  /// Classes that override this method must call `super.clipRRect()`.
  @override
  void clipRRect(ui.RRect rrect) {
    _clipStack ??= <SaveClipEntry>[];
    _clipStack!.add(SaveClipEntry.rrect(rrect, _currentTransform.clone()));
  }

  /// Adds a path to clipping stack.
  ///
  /// Classes that override this method must call `super.clipPath()`.
  @override
  void clipPath(ui.Path path) {
    _clipStack ??= <SaveClipEntry>[];
    _clipStack!.add(SaveClipEntry.path(path, _currentTransform.clone()));
  }
}

DomElement drawParagraphElement(CanvasParagraph paragraph, ui.Offset offset, {Matrix4? transform}) {
  assert(paragraph.isLaidOut);

  final DomElement paragraphElement = paragraph.toDomElement();

  if (transform != null) {
    setElementTransform(paragraphElement, transformWithOffset(transform, offset).storage);
  }
  return paragraphElement;
}

class _SaveElementStackEntry {
  _SaveElementStackEntry({required this.savedElement, required this.transform});

  final DomElement savedElement;
  final Matrix4 transform;
}

/// Provides save stack tracking functionality to implementations of
/// [EngineCanvas].
mixin SaveElementStackTracking on EngineCanvas {
  final List<_SaveElementStackEntry> _saveStack = <_SaveElementStackEntry>[];

  /// The element at the top of the element stack, or [rootElement] if the stack
  /// is empty.
  DomElement get currentElement => _elementStack.isEmpty ? rootElement : _elementStack.last;

  /// The stack that maintains the DOM elements used to express certain paint
  /// operations, such as clips.
  final List<DomElement> _elementStack = <DomElement>[];

  /// Pushes the [element] onto the element stack for the purposes of applying
  /// a paint effect using a DOM element, e.g. for clipping.
  ///
  /// The [restore] method automatically pops the element off the stack.
  void pushElement(DomElement element) {
    _elementStack.add(element);
  }

  /// Empties the save stack and the element stack, and resets the transform
  /// and clip parameters.
  ///
  /// Classes that override this method must call `super.clear()`.
  @override
  void clear() {
    _saveStack.clear();
    _elementStack.clear();
    _currentTransform = Matrix4.identity();
  }

  /// The current transformation matrix.
  Matrix4 get currentTransform => _currentTransform;
  Matrix4 _currentTransform = Matrix4.identity();

  /// Saves current clip and transform on the save stack.
  ///
  /// Classes that override this method must call `super.save()`.
  @override
  void save() {
    _saveStack.add(
      _SaveElementStackEntry(savedElement: currentElement, transform: _currentTransform.clone()),
    );
  }

  /// Restores current clip and transform from the save stack.
  ///
  /// Classes that override this method must call `super.restore()`.
  @override
  void restore() {
    if (_saveStack.isEmpty) {
      return;
    }
    final _SaveElementStackEntry entry = _saveStack.removeLast();
    _currentTransform = entry.transform;

    // Pop out of any clips.
    while (currentElement != entry.savedElement) {
      _elementStack.removeLast();
    }
  }

  /// Multiplies the [currentTransform] matrix by a translation.
  ///
  /// Classes that override this method must call `super.translate()`.
  @override
  void translate(double dx, double dy) {
    _currentTransform.translate(dx, dy);
  }

  /// Scales the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.scale()`.
  @override
  void scale(double sx, double sy) {
    _currentTransform.scale(sx, sy);
  }

  /// Rotates the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.rotate()`.
  @override
  void rotate(double radians) {
    _currentTransform.rotate(kUnitZ, radians);
  }

  /// Skews the [currentTransform] matrix.
  ///
  /// Classes that override this method must call `super.skew()`.
  @override
  void skew(double sx, double sy) {
    // DO NOT USE Matrix4.skew(sx, sy)! It treats sx and sy values as radians,
    // but in our case they are transform matrix values.
    final Matrix4 skewMatrix = Matrix4.identity();
    final Float32List storage = skewMatrix.storage;
    storage[1] = sy;
    storage[4] = sx;
    _currentTransform.multiply(skewMatrix);
  }

  /// Multiplies the [currentTransform] matrix by another matrix.
  ///
  /// Classes that override this method must call `super.transform()`.
  @override
  void transform(Float32List matrix4) {
    _currentTransform.multiply(Matrix4.fromFloat32List(matrix4));
  }
}
