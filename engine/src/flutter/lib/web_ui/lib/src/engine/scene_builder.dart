// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

// This file implements a SceneBuilder and Scene that works with any renderer
// implementation that provides:
//   * A `ui.Canvas` that conforms to `SceneCanvas`
//   * A `ui.Picture` that conforms to `ScenePicture`
//   * A `ui.ImageFilter` that conforms to `SceneImageFilter`
//
// These contain a few augmentations to the normal `dart:ui` API that provide
// additional sizing information that the scene builder uses to determine how
// these object might occlude one another.

class EngineScene implements ui.Scene {
  EngineScene(this.rootLayer);

  final EngineRootLayer rootLayer;

  // We keep a refcount here because this can be asynchronously rendered, so we
  // don't necessarily want to dispose immediately when the user calls dispose.
  // Instead, we need to stay alive until we're done rendering.
  int _refCount = 1;

  void beginRender() {
    assert(_refCount > 0);
    _refCount++;
  }

  void endRender() {
    _refCount--;
    _disposeIfNeeded();
  }

  @override
  void dispose() {
    _refCount--;
    _disposeIfNeeded();
  }

  void _disposeIfNeeded() {
    assert(_refCount >= 0);
    if (_refCount == 0) {
      rootLayer.dispose();
    }
  }

  @override
  Future<ui.Image> toImage(int width, int height) async {
    return toImageSync(width, height);
  }

  @override
  ui.Image toImageSync(int width, int height) {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Rect canvasRect = ui.Rect.fromLTWH(0, 0, width.toDouble(), height.toDouble());
    final ui.Canvas canvas = ui.Canvas(recorder, canvasRect);

    // Only rasterizes the pictures.
    for (final LayerSlice? slice in rootLayer.slices) {
      if (slice != null) {
        canvas.drawPicture(slice.picture);
      }
    }
    return recorder.endRecording().toImageSync(width, height);
  }
}

sealed class OcclusionMapNode {
  bool overlaps(ui.Rect rect);
  OcclusionMapNode insert(ui.Rect rect);
  ui.Rect get boundingBox;
}

class OcclusionMapEmpty implements OcclusionMapNode {
  @override
  ui.Rect get boundingBox => ui.Rect.zero;

  @override
  OcclusionMapNode insert(ui.Rect rect) => OcclusionMapLeaf(rect);

  @override
  bool overlaps(ui.Rect rect) => false;
}

class OcclusionMapLeaf implements OcclusionMapNode {
  OcclusionMapLeaf(this.rect);

  final ui.Rect rect;

  @override
  ui.Rect get boundingBox => rect;

  @override
  OcclusionMapNode insert(ui.Rect other) => OcclusionMapBranch(this, OcclusionMapLeaf(other));

  @override
  bool overlaps(ui.Rect other) => rect.overlaps(other);
}

class OcclusionMapBranch implements OcclusionMapNode {
  OcclusionMapBranch(this.left, this.right)
    : boundingBox = left.boundingBox.expandToInclude(right.boundingBox);

  final OcclusionMapNode left;
  final OcclusionMapNode right;

  @override
  final ui.Rect boundingBox;

  double _areaOfUnion(ui.Rect first, ui.Rect second) {
    return (math.max(first.right, second.right) - math.min(first.left, second.left)) *
        (math.max(first.bottom, second.bottom) - math.max(first.top, second.top));
  }

  @override
  OcclusionMapNode insert(ui.Rect other) {
    // Try to create nodes with the smallest possible area
    final double leftOtherArea = _areaOfUnion(left.boundingBox, other);
    final double rightOtherArea = _areaOfUnion(right.boundingBox, other);
    final double leftRightArea = boundingBox.width * boundingBox.height;
    if (leftOtherArea < rightOtherArea) {
      if (leftOtherArea < leftRightArea) {
        return OcclusionMapBranch(left.insert(other), right);
      }
    } else {
      if (rightOtherArea < leftRightArea) {
        return OcclusionMapBranch(left, right.insert(other));
      }
    }
    return OcclusionMapBranch(this, OcclusionMapLeaf(other));
  }

  @override
  bool overlaps(ui.Rect rect) {
    if (!boundingBox.overlaps(rect)) {
      return false;
    }
    return left.overlaps(rect) || right.overlaps(rect);
  }
}

class OcclusionMap {
  OcclusionMapNode root = OcclusionMapEmpty();

  void addRect(ui.Rect rect) => root = root.insert(rect);

  bool overlaps(ui.Rect rect) => root.overlaps(rect);
}

class SceneSlice {
  final OcclusionMap pictureOcclusionMap = OcclusionMap();
  final OcclusionMap platformViewOcclusionMap = OcclusionMap();
}

class EngineSceneBuilder implements ui.SceneBuilder {
  LayerBuilder currentBuilder = LayerBuilder.rootLayer();

  final List<SceneSlice> sceneSlices = <SceneSlice>[SceneSlice()];

  // This represents the simplest case with no platform views, which is a fast path
  // that allows us to avoid work tracking the pictures themselves.
  bool _isSimple = true;

  @override
  void addPerformanceOverlay(int enabledOptions, ui.Rect bounds) {
    // We don't plan to implement this on the web.
    throw UnimplementedError();
  }

  @override
  void addPicture(
    ui.Offset offset,
    ui.Picture picture, {
    bool isComplexHint = false,
    bool willChangeHint = false,
  }) {
    final int sliceIndex = _placePicture(
      offset,
      picture as ScenePicture,
      currentBuilder.globalPlatformViewStyling,
    );
    currentBuilder.addPicture(offset, picture, sliceIndex: sliceIndex);
  }

  // This function determines the lowest scene slice that this picture can be placed
  // into and adds it to that slice's occlusion map.
  //
  // The picture is placed in the last slice where it either intersects with a picture
  // in the slice or it intersects with a platform view in the preceding slice. If the
  // picture intersects with a platform view in the last slice, a new slice is added at
  // the end and the picture goes in there.
  int _placePicture(ui.Offset offset, ScenePicture picture, PlatformViewStyling styling) {
    if (_isSimple) {
      // This is the fast path where there are no platform views. The picture should
      // just be placed on the bottom (and only) slice.
      return 0;
    }
    final ui.Rect cullRect = picture.cullRect.shift(offset);
    final ui.Rect mappedCullRect = styling.mapLocalToGlobal(cullRect);
    int sliceIndex = sceneSlices.length;
    while (sliceIndex > 0) {
      final SceneSlice sliceBelow = sceneSlices[sliceIndex - 1];
      if (sliceBelow.platformViewOcclusionMap.overlaps(mappedCullRect)) {
        break;
      }
      sliceIndex--;
      if (sliceBelow.pictureOcclusionMap.overlaps(mappedCullRect)) {
        break;
      }
    }
    if (sliceIndex == 0) {
      // Don't bother to populate the lowest occlusion map with pictures, since
      // we never hit test against pictures in the bottom slice.
      return sliceIndex;
    }
    if (sliceIndex == sceneSlices.length) {
      // Insert a new slice.
      sceneSlices.add(SceneSlice());
    }
    final SceneSlice slice = sceneSlices[sliceIndex];
    slice.pictureOcclusionMap.addRect(mappedCullRect);
    return sliceIndex;
  }

  @override
  void addPlatformView(
    int viewId, {
    ui.Offset offset = ui.Offset.zero,
    double width = 0.0,
    double height = 0.0,
  }) {
    final ui.Rect platformViewRect = ui.Rect.fromLTWH(offset.dx, offset.dy, width, height);
    final int sliceIndex = _placePlatformView(
      viewId,
      platformViewRect,
      currentBuilder.globalPlatformViewStyling,
    );
    currentBuilder.addPlatformView(viewId, bounds: platformViewRect, sliceIndex: sliceIndex);
  }

  // This function determines the lowest scene slice this platform view can be placed
  // into and adds it to that slice's occlusion map.
  //
  // The platform view is placed into the last slice where it intersects with a picture
  // or a platform view.
  int _placePlatformView(int viewId, ui.Rect rect, PlatformViewStyling styling) {
    // Once we add a platform view, we actually have to do proper occlusion tracking.
    _isSimple = false;

    final ui.Rect globalPlatformViewRect = styling.mapLocalToGlobal(rect);
    int sliceIndex = sceneSlices.length - 1;
    while (sliceIndex > 0) {
      final SceneSlice slice = sceneSlices[sliceIndex];
      if (slice.platformViewOcclusionMap.overlaps(globalPlatformViewRect) ||
          slice.pictureOcclusionMap.overlaps(globalPlatformViewRect)) {
        break;
      }
      sliceIndex--;
    }
    sliceIndex = 0;
    final SceneSlice slice = sceneSlices[sliceIndex];
    slice.platformViewOcclusionMap.addRect(globalPlatformViewRect);
    return sliceIndex;
  }

  @override
  void addRetained(ui.EngineLayer retainedLayer) {
    final PictureEngineLayer placedEngineLayer = _placeRetainedLayer(
      retainedLayer as PictureEngineLayer,
      currentBuilder.globalPlatformViewStyling,
    );
    currentBuilder.mergeLayer(placedEngineLayer);
  }

  PictureEngineLayer _placeRetainedLayer(
    PictureEngineLayer retainedLayer,
    PlatformViewStyling styling,
  ) {
    if (_isSimple && retainedLayer.isSimple) {
      // There are no platform views, so we don't need to do any occlusion tracking
      // and can simply merge the layer.
      return retainedLayer;
    }
    bool needsRebuild = false;
    final List<LayerDrawCommand> revisedDrawCommands = [];
    final PlatformViewStyling combinedStyling = PlatformViewStyling.combine(
      styling,
      retainedLayer.platformViewStyling,
    );
    for (final LayerDrawCommand command in retainedLayer.drawCommands) {
      switch (command) {
        case PictureDrawCommand(
          offset: final ui.Offset offset,
          picture: final ScenePicture picture,
        ):
          final int sliceIndex = _placePicture(offset, picture, combinedStyling);
          if (command.sliceIndex != sliceIndex) {
            needsRebuild = true;
          }
          revisedDrawCommands.add(PictureDrawCommand(offset, picture, sliceIndex));
        case PlatformViewDrawCommand(viewId: final int viewId, bounds: final ui.Rect bounds):
          final int sliceIndex = _placePlatformView(viewId, bounds, combinedStyling);
          if (command.sliceIndex != sliceIndex) {
            needsRebuild = true;
          }
          revisedDrawCommands.add(PlatformViewDrawCommand(viewId, bounds, sliceIndex));
        case RetainedLayerDrawCommand(layer: final PictureEngineLayer sublayer):
          final PictureEngineLayer revisedSublayer = _placeRetainedLayer(sublayer, combinedStyling);
          if (sublayer != revisedSublayer) {
            needsRebuild = true;
          }
          revisedDrawCommands.add(RetainedLayerDrawCommand(revisedSublayer));
      }
    }

    if (!needsRebuild) {
      // No elements changed which slice position they are in, so we can simply
      // merge the existing layer down and don't have to redraw individual elements.
      return retainedLayer;
    }

    // Otherwise, we replace the commands of the layer to create a new one.
    currentBuilder = LayerBuilder.childLayer(
      parent: currentBuilder,
      layer: retainedLayer.emptyClone(),
    );
    for (final LayerDrawCommand command in revisedDrawCommands) {
      switch (command) {
        case PictureDrawCommand(
          offset: final ui.Offset offset,
          picture: final ScenePicture picture,
        ):
          currentBuilder.addPicture(offset, picture, sliceIndex: command.sliceIndex);
        case PlatformViewDrawCommand(viewId: final int viewId, bounds: final ui.Rect bounds):
          currentBuilder.addPlatformView(viewId, bounds: bounds, sliceIndex: command.sliceIndex);
        case RetainedLayerDrawCommand(layer: final PictureEngineLayer layer):
          currentBuilder.mergeLayer(layer);
      }
    }
    final PictureEngineLayer newLayer = currentBuilder.build();
    currentBuilder = currentBuilder.parent!;
    return newLayer;
  }

  @override
  void addTexture(
    int textureId, {
    ui.Offset offset = ui.Offset.zero,
    double width = 0.0,
    double height = 0.0,
    bool freeze = false,
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) {
    // addTexture is not implemented on web.
  }

  @override
  ui.BackdropFilterEngineLayer pushBackdropFilter(
    ui.ImageFilter filter, {
    ui.BlendMode blendMode = ui.BlendMode.srcOver,
    ui.BackdropFilterEngineLayer? oldLayer,
    int? backdropId,
  }) => pushLayer<BackdropFilterLayer>(
    BackdropFilterLayer(BackdropFilterOperation(filter, blendMode)),
  );

  @override
  ui.ClipPathEngineLayer pushClipPath(
    ui.Path path, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.ClipPathEngineLayer? oldLayer,
  }) => pushLayer<ClipPathLayer>(ClipPathLayer(ClipPathOperation(path as ScenePath, clipBehavior)));

  @override
  ui.ClipRRectEngineLayer pushClipRRect(
    ui.RRect rrect, {
    required ui.Clip clipBehavior,
    ui.ClipRRectEngineLayer? oldLayer,
  }) => pushLayer<ClipRRectLayer>(ClipRRectLayer(ClipRRectOperation(rrect, clipBehavior)));

  @override
  ui.ClipRectEngineLayer pushClipRect(
    ui.Rect rect, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.ClipRectEngineLayer? oldLayer,
  }) {
    return pushLayer<ClipRectLayer>(ClipRectLayer(ClipRectOperation(rect, clipBehavior)));
  }

  @override
  ui.ColorFilterEngineLayer pushColorFilter(
    ui.ColorFilter filter, {
    ui.ColorFilterEngineLayer? oldLayer,
  }) => pushLayer<ColorFilterLayer>(ColorFilterLayer(ColorFilterOperation(filter)));

  @override
  ui.ImageFilterEngineLayer pushImageFilter(
    ui.ImageFilter filter, {
    ui.Offset offset = ui.Offset.zero,
    ui.ImageFilterEngineLayer? oldLayer,
  }) => pushLayer<ImageFilterLayer>(
    ImageFilterLayer(ImageFilterOperation(filter as SceneImageFilter, offset)),
  );

  @override
  ui.OffsetEngineLayer pushOffset(double dx, double dy, {ui.OffsetEngineLayer? oldLayer}) =>
      pushLayer<OffsetLayer>(OffsetLayer(OffsetOperation(dx, dy)));

  @override
  ui.OpacityEngineLayer pushOpacity(
    int alpha, {
    ui.Offset offset = ui.Offset.zero,
    ui.OpacityEngineLayer? oldLayer,
  }) => pushLayer<OpacityLayer>(OpacityLayer(OpacityOperation(alpha, offset)));

  @override
  ui.ShaderMaskEngineLayer pushShaderMask(
    ui.Shader shader,
    ui.Rect maskRect,
    ui.BlendMode blendMode, {
    ui.ShaderMaskEngineLayer? oldLayer,
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) =>
      pushLayer<ShaderMaskLayer>(ShaderMaskLayer(ShaderMaskOperation(shader, maskRect, blendMode)));

  @override
  ui.TransformEngineLayer pushTransform(Float64List matrix4, {ui.TransformEngineLayer? oldLayer}) =>
      pushLayer<TransformLayer>(TransformLayer(TransformOperation(matrix4)));

  @override
  void setProperties(
    double width,
    double height,
    double insetTop,
    double insetRight,
    double insetBottom,
    double insetLeft,
    bool focusable,
  ) {
    // Not implemented on web
  }

  @override
  ui.Scene build() {
    while (currentBuilder.parent != null) {
      pop();
    }
    final PictureEngineLayer rootLayer = currentBuilder.build();
    return EngineScene(rootLayer as EngineRootLayer);
  }

  @override
  void pop() {
    final PictureEngineLayer layer = currentBuilder.build();
    final LayerBuilder? parentBuilder = currentBuilder.parent;
    if (parentBuilder == null) {
      throw StateError('Popped too many times.');
    }
    currentBuilder = parentBuilder;
    currentBuilder.mergeLayer(layer);
  }

  T pushLayer<T extends PictureEngineLayer>(T layer) {
    currentBuilder = LayerBuilder.childLayer(parent: currentBuilder, layer: layer);
    return layer;
  }
}
