// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

class EngineRootLayer with PictureEngineLayer {
  @override
  final NoopOperation operation = const NoopOperation();

  @override
  EngineRootLayer emptyClone() => EngineRootLayer();
}

class NoopOperation implements LayerOperation {
  const NoopOperation();

  @override
  PlatformViewStyling createPlatformViewStyling() => const PlatformViewStyling();

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect;

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
  }

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'NoopOperation()';
}

class BackdropFilterLayer with PictureEngineLayer implements ui.BackdropFilterEngineLayer {
  BackdropFilterLayer(this.operation);

  @override
  final LayerOperation operation;

  @override
  BackdropFilterLayer emptyClone() => BackdropFilterLayer(operation);
}

class BackdropFilterOperation implements LayerOperation {
  BackdropFilterOperation(this.filter, this.mode);

  final ui.ImageFilter filter;
  final ui.BlendMode mode;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect;

  @override
  void pre(SceneCanvas canvas) {
    canvas.saveLayerWithFilter(ui.Rect.largest, ui.Paint()..blendMode = mode, filter);
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() => const PlatformViewStyling();

  @override
  bool get affectsBackdrop => true;

  @override
  String toString() => 'BackdropFilterOperation(filter: $filter, mode: $mode)';
}

class ClipPathLayer with PictureEngineLayer implements ui.ClipPathEngineLayer {
  ClipPathLayer(this.operation);

  @override
  final ClipPathOperation operation;

  @override
  ClipPathLayer emptyClone() => ClipPathLayer(operation);
}

class ClipPathOperation implements LayerOperation {
  ClipPathOperation(this.path, this.clip);

  final ScenePath path;
  final ui.Clip clip;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect.intersect(path.getBounds());

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
    canvas.clipPath(path, doAntiAlias: clip != ui.Clip.hardEdge);
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.saveLayer(path.getBounds(), ui.Paint());
    }
  }

  @override
  void post(SceneCanvas canvas) {
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.restore();
    }
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() {
    return PlatformViewStyling(clip: PlatformViewPathClip(path));
  }

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'ClipPathOperation(path: $path, clip: $clip)';
}

class ClipRectLayer with PictureEngineLayer implements ui.ClipRectEngineLayer {
  ClipRectLayer(this.operation);

  @override
  final ClipRectOperation operation;

  @override
  ClipRectLayer emptyClone() => ClipRectLayer(operation);
}

class ClipRectOperation implements LayerOperation {
  const ClipRectOperation(this.rect, this.clip);

  final ui.Rect rect;
  final ui.Clip clip;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect.intersect(rect);

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
    canvas.clipRect(rect, doAntiAlias: clip != ui.Clip.hardEdge);
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.saveLayer(rect, ui.Paint());
    }
  }

  @override
  void post(SceneCanvas canvas) {
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.restore();
    }
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() {
    return PlatformViewStyling(clip: PlatformViewRectClip(rect));
  }

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'ClipRectOperation(rect: $rect, clip: $clip)';
}

class ClipRRectLayer with PictureEngineLayer implements ui.ClipRRectEngineLayer {
  ClipRRectLayer(this.operation);

  @override
  final ClipRRectOperation operation;

  @override
  ClipRRectLayer emptyClone() => ClipRRectLayer(operation);
}

class ClipRRectOperation implements LayerOperation {
  const ClipRRectOperation(this.rrect, this.clip);

  final ui.RRect rrect;
  final ui.Clip clip;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect.intersect(rrect.outerRect);

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
    canvas.clipRRect(rrect, doAntiAlias: clip != ui.Clip.hardEdge);
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.saveLayer(rrect.outerRect, ui.Paint());
    }
  }

  @override
  void post(SceneCanvas canvas) {
    if (clip == ui.Clip.antiAliasWithSaveLayer) {
      canvas.restore();
    }
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() {
    return PlatformViewStyling(clip: PlatformViewRRectClip(rrect));
  }

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'ClipRRectOperation(rrect: $rrect, clip: $clip)';
}

class ColorFilterLayer with PictureEngineLayer implements ui.ColorFilterEngineLayer {
  ColorFilterLayer(this.operation);

  @override
  final ColorFilterOperation operation;

  @override
  ColorFilterLayer emptyClone() => ColorFilterLayer(operation);
}

class ColorFilterOperation implements LayerOperation {
  ColorFilterOperation(this.filter);

  final ui.ColorFilter filter;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect;

  @override
  void pre(SceneCanvas canvas) {
    canvas.saveLayer(ui.Rect.largest, ui.Paint()..colorFilter = filter);
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() => const PlatformViewStyling();

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'ColorFilterOperation(filter: $filter)';
}

class ImageFilterLayer with PictureEngineLayer implements ui.ImageFilterEngineLayer {
  ImageFilterLayer(this.operation);

  @override
  final ImageFilterOperation operation;

  @override
  ImageFilterLayer emptyClone() => ImageFilterLayer(operation);
}

class ImageFilterOperation implements LayerOperation {
  ImageFilterOperation(this.filter, this.offset);

  final SceneImageFilter filter;
  final ui.Offset offset;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => filter.filterBounds(contentRect);

  @override
  void pre(SceneCanvas canvas) {
    if (offset != ui.Offset.zero) {
      canvas.save();
      canvas.translate(offset.dx, offset.dy);
    }
    canvas.saveLayer(ui.Rect.largest, ui.Paint()..imageFilter = filter);
  }

  @override
  void post(SceneCanvas canvas) {
    if (offset != ui.Offset.zero) {
      canvas.restore();
    }
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() {
    PlatformViewStyling styling = const PlatformViewStyling();
    if (offset != ui.Offset.zero) {
      styling = PlatformViewStyling(position: PlatformViewPosition.offset(offset));
    }
    final Matrix4? transform = filter.transform;
    if (transform != null) {
      styling = PlatformViewStyling.combine(
        styling,
        PlatformViewStyling(position: PlatformViewPosition.transform(transform)),
      );
    }
    return const PlatformViewStyling();
  }

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'ImageFilterOperation(filter: $filter)';
}

class OffsetLayer with PictureEngineLayer implements ui.OffsetEngineLayer {
  OffsetLayer(this.operation);

  @override
  final OffsetOperation operation;

  @override
  OffsetLayer emptyClone() => OffsetLayer(operation);
}

class OffsetOperation implements LayerOperation {
  OffsetOperation(this.dx, this.dy);

  final double dx;
  final double dy;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect.shift(ui.Offset(dx, dy));

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
    canvas.translate(dx, dy);
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() =>
      PlatformViewStyling(position: PlatformViewPosition.offset(ui.Offset(dx, dy)));

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'OffsetOperation(dx: $dx, dy: $dy)';
}

class OpacityLayer with PictureEngineLayer implements ui.OpacityEngineLayer {
  OpacityLayer(this.operation);

  @override
  final OpacityOperation operation;

  @override
  OpacityLayer emptyClone() => OpacityLayer(operation);
}

class OpacityOperation implements LayerOperation {
  OpacityOperation(this.alpha, this.offset);

  final int alpha;
  final ui.Offset offset;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect.shift(offset);

  @override
  void pre(SceneCanvas canvas) {
    if (offset != ui.Offset.zero) {
      canvas.save();
      canvas.translate(offset.dx, offset.dy);
    }
    canvas.saveLayer(ui.Rect.largest, ui.Paint()..color = ui.Color.fromARGB(alpha, 0, 0, 0));
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
    if (offset != ui.Offset.zero) {
      canvas.restore();
    }
  }

  @override
  PlatformViewStyling createPlatformViewStyling() => PlatformViewStyling(
    position:
        offset != ui.Offset.zero
            ? PlatformViewPosition.offset(offset)
            : const PlatformViewPosition.zero(),
    opacity: alpha.toDouble() / 255.0,
  );

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'OpacityOperation(offset: $offset, alpha: $alpha)';
}

class TransformLayer with PictureEngineLayer implements ui.TransformEngineLayer {
  TransformLayer(this.operation);

  @override
  final TransformOperation operation;

  @override
  TransformLayer emptyClone() => TransformLayer(operation);
}

class TransformOperation implements LayerOperation {
  TransformOperation(this.transform);

  final Float64List transform;

  Matrix4? _memoizedMatrix;
  Matrix4 get matrix =>
      _memoizedMatrix ?? (_memoizedMatrix = Matrix4.fromFloat32List(toMatrix32(transform)));

  @override
  ui.Rect mapRect(ui.Rect contentRect) => matrix.transformRect(contentRect);

  @override
  void pre(SceneCanvas canvas) {
    canvas.save();
    canvas.transform(transform);
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() =>
      PlatformViewStyling(position: PlatformViewPosition.transform(matrix));

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() => 'TransformOperation(matrix: $matrix)';
}

class ShaderMaskLayer with PictureEngineLayer implements ui.ShaderMaskEngineLayer {
  ShaderMaskLayer(this.operation);

  @override
  final ShaderMaskOperation operation;

  @override
  ShaderMaskLayer emptyClone() => ShaderMaskLayer(operation);
}

class ShaderMaskOperation implements LayerOperation {
  ShaderMaskOperation(this.shader, this.maskRect, this.blendMode);

  final ui.Shader shader;
  final ui.Rect maskRect;
  final ui.BlendMode blendMode;

  @override
  ui.Rect mapRect(ui.Rect contentRect) => contentRect;

  @override
  void pre(SceneCanvas canvas) {
    canvas.saveLayer(ui.Rect.largest, ui.Paint());
  }

  @override
  void post(SceneCanvas canvas) {
    canvas.save();
    canvas.translate(maskRect.left, maskRect.top);
    canvas.drawRect(
      ui.Rect.fromLTWH(0, 0, maskRect.width, maskRect.height),
      ui.Paint()
        ..blendMode = blendMode
        ..shader = shader,
    );
    canvas.restore();
    canvas.restore();
  }

  @override
  PlatformViewStyling createPlatformViewStyling() => const PlatformViewStyling();

  @override
  bool get affectsBackdrop => false;

  @override
  String toString() =>
      'ShaderMaskOperation(shader: $shader, maskRect: $maskRect, blendMode: $blendMode)';
}

class PlatformView {
  PlatformView(this.viewId, this.bounds, this.styling);

  final int viewId;

  // The bounds of this platform view, in the layer's local coordinate space.
  final ui.Rect bounds;

  final PlatformViewStyling styling;

  @override
  String toString() {
    return 'PlatformView(viewId: $viewId, bounds: $bounds, styling: $styling)';
  }
}

class LayerSlice {
  LayerSlice(this.picture, this.platformViews);

  // The picture of native flutter content to be rendered
  ScenePicture picture;

  // Platform views to be placed on top of the flutter content.
  final List<PlatformView> platformViews;

  void dispose() {
    picture.dispose();
  }
}

mixin PictureEngineLayer implements ui.EngineLayer {
  // Each layer is represented as a series of "slices" which contain flutter content
  // with platform views on top. This is ordered from bottommost to topmost.
  List<LayerSlice?> slices = [];

  List<LayerDrawCommand> drawCommands = [];
  PlatformViewStyling platformViewStyling = const PlatformViewStyling();

  LayerOperation get operation;

  PictureEngineLayer emptyClone();

  @override
  void dispose() {
    for (final LayerSlice? slice in slices) {
      slice?.dispose();
    }
  }

  @override
  String toString() {
    return 'PictureEngineLayer($operation)';
  }

  bool get isSimple {
    if (slices.length > 1) {
      return false;
    }
    final LayerSlice? singleSlice = slices.firstOrNull;
    if (singleSlice == null || singleSlice.platformViews.isEmpty) {
      return true;
    }
    return false;
  }
}

abstract class LayerOperation {
  const LayerOperation();

  // Given an input content rectangle, this returns a conservative estimate of
  // the covering rectangle of the content after it has been processed by the
  // layer operation.
  ui.Rect mapRect(ui.Rect contentRect);

  void pre(SceneCanvas canvas);
  void post(SceneCanvas canvas);

  PlatformViewStyling createPlatformViewStyling();

  /// Indicates whether this operation's `pre` and `post` methods should be
  /// invoked even if it contains no pictures. (Most operations don't need to
  /// actually be performed at all if they don't contain any pictures.)
  bool get affectsBackdrop;
}

sealed class LayerDrawCommand {}

class PictureDrawCommand extends LayerDrawCommand {
  PictureDrawCommand(this.offset, this.picture, this.sliceIndex);

  final int sliceIndex;
  final ui.Offset offset;
  final ScenePicture picture;
}

class PlatformViewDrawCommand extends LayerDrawCommand {
  PlatformViewDrawCommand(this.viewId, this.bounds, this.sliceIndex);

  final int sliceIndex;
  final int viewId;
  final ui.Rect bounds;
}

class RetainedLayerDrawCommand extends LayerDrawCommand {
  RetainedLayerDrawCommand(this.layer);

  final PictureEngineLayer layer;
}

// Represents how a platform view should be positioned in the scene.
// This object is immutable, so it can be reused across different platform
// views that have the same positioning.
class PlatformViewPosition {
  // No transformation at all. We leave both fields null.
  const PlatformViewPosition.zero() : offset = null, transform = null;

  // A simple offset is the most common scenario. In those cases, we only
  // store the offset and leave the transform as null
  const PlatformViewPosition.offset(this.offset) : transform = null;

  // In more complex cases, we store the transform. In those cases, the offset
  // is left as null.
  const PlatformViewPosition.transform(this.transform) : offset = null;

  bool get isZero => (offset == null) && (transform == null);

  ui.Rect mapLocalToGlobal(ui.Rect rect) {
    if (offset != null) {
      return rect.shift(offset!);
    }
    if (transform != null) {
      return transform!.transformRect(rect);
    }
    return rect;
  }

  // Note that by construction only one of these can be set at any given time, not both.
  final ui.Offset? offset;
  final Matrix4? transform;

  static PlatformViewPosition combine(PlatformViewPosition outer, PlatformViewPosition inner) {
    // We try to reuse existing objects if possible, if they are immutable.
    if (outer.isZero) {
      return inner;
    }
    if (inner.isZero) {
      return outer;
    }
    final ui.Offset? outerOffset = outer.offset;
    final ui.Offset? innerOffset = inner.offset;
    if (outerOffset != null && innerOffset != null) {
      // Both positions are simple offsets, so they can be combined cheaply
      // into another offset.
      return PlatformViewPosition.offset(outerOffset + innerOffset);
    }

    // Otherwise, at least one of the positions involves a matrix transform.
    final Matrix4 innerTransform;
    final Matrix4 outerTransform;
    if (innerOffset != null) {
      innerTransform = Matrix4.translationValues(innerOffset.dx, innerOffset.dy, 0);
    } else {
      innerTransform = inner.transform!;
    }
    if (outerOffset != null) {
      outerTransform = Matrix4.translationValues(outerOffset.dx, outerOffset.dy, 0);
    } else {
      outerTransform = outer.transform!;
    }
    return PlatformViewPosition.transform(outerTransform.multiplied(innerTransform));
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other is! PlatformViewPosition) {
      return false;
    }
    return (offset == other.offset) && (transform == other.transform);
  }

  @override
  int get hashCode {
    return Object.hash(offset, transform);
  }

  @override
  String toString() {
    if (offset != null) {
      return 'PlatformViewPosition(offset: $offset)';
    }
    if (transform != null) {
      return 'PlatformViewPosition(transform: $transform)';
    }
    return 'PlatformViewPosition(zero)';
  }
}

// Represents the styling to be performed on a platform view when it is
// composited. This object is immutable so that it can be reused with different
// platform views that have the same styling.
class PlatformViewStyling {
  const PlatformViewStyling({
    this.position = const PlatformViewPosition.zero(),
    this.clip = const PlatformViewNoClip(),
    this.opacity = 1.0,
  });

  bool get isDefault => position.isZero && (opacity == 1.0) && clip is PlatformViewNoClip;

  final PlatformViewPosition position;
  final double opacity;
  final PlatformViewClip clip;

  ui.Rect mapLocalToGlobal(ui.Rect rect) {
    return position.mapLocalToGlobal(rect.intersect(clip.outerRect));
  }

  static PlatformViewStyling combine(PlatformViewStyling outer, PlatformViewStyling inner) {
    // Attempt to reuse one of the existing immutable objects.
    if (outer.isDefault) {
      return inner;
    }
    if (inner.isDefault) {
      return outer;
    }
    return PlatformViewStyling(
      position: PlatformViewPosition.combine(outer.position, inner.position),
      clip: PlatformViewClip.combine(outer.clip, inner.clip.positioned(outer.position)),
      opacity: outer.opacity * inner.opacity,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other is! PlatformViewStyling) {
      return false;
    }
    return (position == other.position) && (opacity == other.opacity) && (clip == other.clip);
  }

  @override
  int get hashCode {
    return Object.hash(position, opacity, clip);
  }

  @override
  String toString() {
    return 'PlatformViewStyling(position: $position, clip: $clip, opacity: $opacity)';
  }
}

sealed class PlatformViewClip {
  PlatformViewClip positioned(PlatformViewPosition position);

  /// The largest rectangle that is entirely inside the clip region. All
  /// inside of this region is unclipped.
  ui.Rect get innerRect;

  /// The bounding rectangle of the clip region. All content outside of this
  /// region is clipped.
  ui.Rect get outerRect;

  ScenePath get toPath;

  static bool rectCovers(ui.Rect covering, ui.Rect covered) {
    return covering.left <= covered.left &&
        covering.right >= covered.right &&
        covering.top <= covered.top &&
        covering.bottom >= covered.bottom;
  }

  static PlatformViewClip combine(PlatformViewClip outer, PlatformViewClip inner) {
    if (outer is PlatformViewNoClip) {
      return inner;
    }
    if (inner is PlatformViewNoClip) {
      return outer;
    }

    if (rectCovers(outer.innerRect, inner.outerRect)) {
      return inner;
    }

    if (rectCovers(inner.innerRect, outer.outerRect)) {
      return outer;
    }

    return PlatformViewPathClip(
      ui.Path.combine(ui.PathOperation.intersect, outer.toPath, inner.toPath) as ScenePath,
    );
  }
}

class PlatformViewNoClip implements PlatformViewClip {
  const PlatformViewNoClip();

  @override
  PlatformViewClip positioned(PlatformViewPosition positioned) {
    return this;
  }

  @override
  ScenePath get toPath => ui.Path() as ScenePath;

  @override
  bool operator ==(Object other) {
    return identical(this, other) || (other.runtimeType == PlatformViewNoClip);
  }

  @override
  int get hashCode => runtimeType.hashCode;

  @override
  ui.Rect get innerRect => ui.Rect.zero;

  @override
  ui.Rect get outerRect => ui.Rect.largest;
}

class PlatformViewRectClip implements PlatformViewClip {
  PlatformViewRectClip(this.rect);

  final ui.Rect rect;

  @override
  PlatformViewClip positioned(PlatformViewPosition position) {
    if (position.isZero) {
      return this;
    }
    final ui.Offset? offset = position.offset;
    if (offset != null) {
      return PlatformViewRectClip(rect.shift(offset));
    } else {
      return PlatformViewPathClip(toPath.transform(position.transform!.toFloat64()) as ScenePath);
    }
  }

  @override
  ScenePath get toPath => (ui.Path() as ScenePath)..addRect(rect);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is PlatformViewRectClip && rect == other.rect;
  }

  @override
  int get hashCode => Object.hash(runtimeType, rect);

  @override
  ui.Rect get innerRect => rect;

  @override
  ui.Rect get outerRect => rect;
}

class PlatformViewRRectClip implements PlatformViewClip {
  PlatformViewRRectClip(this.rrect);

  final ui.RRect rrect;

  @override
  PlatformViewClip positioned(PlatformViewPosition position) {
    if (position.isZero) {
      return this;
    }
    final ui.Offset? offset = position.offset;
    if (offset != null) {
      return PlatformViewRRectClip(rrect.shift(offset));
    } else {
      return PlatformViewPathClip(toPath.transform(position.transform!.toFloat64()) as ScenePath);
    }
  }

  @override
  ScenePath get toPath => (ui.Path() as ScenePath)..addRRect(rrect);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is PlatformViewRRectClip && rrect == other.rrect;
  }

  @override
  int get hashCode => Object.hash(runtimeType, rrect);

  @override
  ui.Rect get innerRect => rrect.safeInnerRect;

  @override
  ui.Rect get outerRect => rrect.outerRect;
}

class PlatformViewPathClip implements PlatformViewClip {
  PlatformViewPathClip(this.path);

  final ScenePath path;

  @override
  PlatformViewClip positioned(PlatformViewPosition position) {
    if (position.isZero) {
      return this;
    }

    final ui.Offset? offset = position.offset;
    if (offset != null) {
      return PlatformViewPathClip(path.shift(offset) as ScenePath);
    } else {
      return PlatformViewPathClip(path.transform(position.transform!.toFloat64()) as ScenePath);
    }
  }

  @override
  ScenePath get toPath => path;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is PlatformViewPathClip && path == other.path;
  }

  @override
  int get hashCode => Object.hash(runtimeType, path);

  @override
  ui.Rect get innerRect => ui.Rect.zero;

  @override
  ui.Rect get outerRect => path.getBounds();
}

class LayerSliceBuilder {
  @visibleForTesting
  static (ui.PictureRecorder, SceneCanvas) Function(ui.Rect)? debugRecorderFactory;

  static (ui.PictureRecorder, SceneCanvas) defaultRecorderFactory(ui.Rect rect) {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final SceneCanvas canvas = ui.Canvas(recorder, rect) as SceneCanvas;
    return (recorder, canvas);
  }

  void addPicture(ui.Offset offset, ScenePicture picture) {
    pictures.add((picture, offset));
    final ui.Rect pictureRect = picture.cullRect.shift(offset);
    cullRect = cullRect?.expandToInclude(pictureRect) ?? pictureRect;
  }

  (ui.PictureRecorder, SceneCanvas) createRecorder(ui.Rect rect) =>
      debugRecorderFactory != null ? debugRecorderFactory!(rect) : defaultRecorderFactory(rect);

  LayerSlice buildWithOperation(LayerOperation operation, ui.Rect? backdropRect) {
    final ui.Rect effectiveRect;
    if (backdropRect != null && cullRect != null) {
      effectiveRect = cullRect!.expandToInclude(backdropRect);
    } else {
      effectiveRect = backdropRect ?? cullRect ?? ui.Rect.zero;
    }
    final (recorder, canvas) = createRecorder(operation.mapRect(effectiveRect));
    operation.pre(canvas);
    for (final (picture, offset) in pictures) {
      if (offset != ui.Offset.zero) {
        canvas.save();
        canvas.translate(offset.dx, offset.dy);
        canvas.drawPicture(picture);
        canvas.restore();
      } else {
        canvas.drawPicture(picture);
      }
    }
    operation.post(canvas);
    final ui.Picture picture = recorder.endRecording();
    return LayerSlice(picture as ScenePicture, platformViews);
  }

  final List<(ScenePicture, ui.Offset)> pictures = [];
  ui.Rect? cullRect;
  final List<PlatformView> platformViews = <PlatformView>[];
}

class LayerBuilder {
  factory LayerBuilder.rootLayer() {
    return LayerBuilder._(null, EngineRootLayer());
  }

  factory LayerBuilder.childLayer({
    required LayerBuilder parent,
    required PictureEngineLayer layer,
  }) {
    return LayerBuilder._(parent, layer);
  }

  LayerBuilder._(this.parent, this.layer);

  final LayerBuilder? parent;
  final PictureEngineLayer layer;

  final List<LayerSliceBuilder?> sliceBuilders = <LayerSliceBuilder?>[];
  final List<LayerDrawCommand> drawCommands = <LayerDrawCommand>[];

  ui.Rect? getCurrentBackdropRectAtSliceIndex(int index) {
    final parentRect = parent?.getCurrentBackdropRectAtSliceIndex(index);
    final sliceBuilder = index < sliceBuilders.length ? sliceBuilders[index] : null;
    final sliceRect = sliceBuilder?.cullRect;
    final ui.Rect? combinedRect;
    if (sliceRect != null && parentRect != null) {
      combinedRect = parentRect.expandToInclude(sliceRect);
    } else {
      combinedRect = parentRect ?? sliceRect;
    }
    return combinedRect == null ? null : layer.operation.mapRect(combinedRect);
  }

  int getCurrentSliceCount() {
    final parentSliceCount = parent?.getCurrentSliceCount();
    if (parentSliceCount != null) {
      return math.max(parentSliceCount, sliceBuilders.length);
    } else {
      return sliceBuilders.length;
    }
  }

  PlatformViewStyling? _memoizedPlatformViewStyling;
  PlatformViewStyling get platformViewStyling {
    return _memoizedPlatformViewStyling ??= layer.operation.createPlatformViewStyling();
  }

  PlatformViewStyling? _memoizedGlobalPlatformViewStyling;
  PlatformViewStyling get globalPlatformViewStyling {
    if (_memoizedGlobalPlatformViewStyling != null) {
      return _memoizedGlobalPlatformViewStyling!;
    }
    if (parent != null) {
      return _memoizedGlobalPlatformViewStyling ??= PlatformViewStyling.combine(
        parent!.globalPlatformViewStyling,
        platformViewStyling,
      );
    }
    return _memoizedGlobalPlatformViewStyling ??= platformViewStyling;
  }

  LayerSliceBuilder getOrCreateSliceBuilderAtIndex(int index) {
    while (sliceBuilders.length <= index) {
      sliceBuilders.add(null);
    }
    final LayerSliceBuilder? existingSliceBuilder = sliceBuilders[index];
    if (existingSliceBuilder != null) {
      return existingSliceBuilder;
    }
    final LayerSliceBuilder newSliceBuilder = LayerSliceBuilder();
    sliceBuilders[index] = newSliceBuilder;
    return newSliceBuilder;
  }

  void addPicture(ui.Offset offset, ui.Picture picture, {required int sliceIndex}) {
    final LayerSliceBuilder sliceBuilder = getOrCreateSliceBuilderAtIndex(sliceIndex);
    sliceBuilder.addPicture(offset, picture as ScenePicture);
    drawCommands.add(PictureDrawCommand(offset, picture, sliceIndex));
  }

  void addPlatformView(int viewId, {required ui.Rect bounds, required int sliceIndex}) {
    final LayerSliceBuilder sliceBuilder = getOrCreateSliceBuilderAtIndex(sliceIndex);
    sliceBuilder.platformViews.add(PlatformView(viewId, bounds, platformViewStyling));
    drawCommands.add(PlatformViewDrawCommand(viewId, bounds, sliceIndex));
  }

  void mergeLayer(PictureEngineLayer layer) {
    for (int i = 0; i < layer.slices.length; i++) {
      final LayerSlice? slice = layer.slices[i];
      if (slice != null) {
        final LayerSliceBuilder sliceBuilder = getOrCreateSliceBuilderAtIndex(i);
        sliceBuilder.addPicture(ui.Offset.zero, slice.picture);
        sliceBuilder.platformViews.addAll(
          slice.platformViews.map((PlatformView view) {
            return PlatformView(
              view.viewId,
              view.bounds,
              PlatformViewStyling.combine(platformViewStyling, view.styling),
            );
          }),
        );
      }
    }
    drawCommands.add(RetainedLayerDrawCommand(layer));
  }

  PictureEngineLayer sliceUp() {
    final int sliceCount =
        layer.operation.affectsBackdrop ? getCurrentSliceCount() : sliceBuilders.length;
    final slices = <LayerSlice?>[];
    for (int i = 0; i < sliceCount; i++) {
      final ui.Rect? backdropRect;
      if (layer.operation.affectsBackdrop) {
        backdropRect = getCurrentBackdropRectAtSliceIndex(i);
      } else {
        backdropRect = null;
      }
      final LayerSliceBuilder? builder;
      if (backdropRect != null) {
        builder = getOrCreateSliceBuilderAtIndex(i);
      } else {
        builder = i < sliceBuilders.length ? sliceBuilders[i] : null;
      }
      slices.add(builder?.buildWithOperation(layer.operation, backdropRect));
    }
    layer.slices = slices;
    return layer;
  }

  PictureEngineLayer build() {
    layer.drawCommands = drawCommands;
    layer.platformViewStyling = platformViewStyling;
    return sliceUp();
  }
}
