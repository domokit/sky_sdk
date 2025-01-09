// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../vector_math.dart';
import 'layer.dart';
import 'layer_tree.dart';
import 'path.dart';
import 'picture.dart';

class LayerScene implements ui.Scene {
  LayerScene(RootLayer rootLayer) : layerTree = LayerTree(rootLayer);

  final LayerTree layerTree;

  @override
  void dispose() {}

  @override
  Future<ui.Image> toImage(int width, int height) {
    final ui.Picture picture = layerTree.flatten(ui.Size(width.toDouble(), height.toDouble()));
    return picture.toImage(width, height);
  }

  @override
  ui.Image toImageSync(int width, int height) {
    final ui.Picture picture = layerTree.flatten(ui.Size(width.toDouble(), height.toDouble()));
    return picture.toImageSync(width, height);
  }
}

class LayerSceneBuilder implements ui.SceneBuilder {
  LayerSceneBuilder() : rootLayer = RootLayer() {
    currentLayer = rootLayer;
  }

  final RootLayer rootLayer;
  late ContainerLayer currentLayer;

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
    currentLayer.add(PictureLayer(picture as CkPicture, offset, isComplexHint, willChangeHint));
  }

  @override
  void addRetained(ui.EngineLayer retainedLayer) {
    currentLayer.add(retainedLayer as Layer);
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
    // TODO(hterkelsen): implement addTexture, b/128315641
  }

  @override
  void addPlatformView(
    int viewId, {
    ui.Offset offset = ui.Offset.zero,
    double width = 0.0,
    double height = 0.0,
  }) {
    currentLayer.add(PlatformViewLayer(viewId, offset, width, height));
  }

  @override
  LayerScene build() {
    return LayerScene(rootLayer);
  }

  @override
  void pop() {
    if (currentLayer == rootLayer) {
      // Don't pop the root layer. It must always be there.
      return;
    }
    currentLayer = currentLayer.parent!;
  }

  @override
  BackdropFilterEngineLayer pushBackdropFilter(
    ui.ImageFilter filter, {
    ui.BlendMode blendMode = ui.BlendMode.srcOver,
    ui.EngineLayer? oldLayer,
    int? backdropId,
  }) {
    return pushLayer<BackdropFilterEngineLayer>(BackdropFilterEngineLayer(filter, blendMode));
  }

  @override
  ClipPathEngineLayer pushClipPath(
    ui.Path path, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.EngineLayer? oldLayer,
  }) {
    return pushLayer<ClipPathEngineLayer>(ClipPathEngineLayer(path as CkPath, clipBehavior));
  }

  @override
  ClipRRectEngineLayer pushClipRRect(
    ui.RRect rrect, {
    ui.Clip? clipBehavior,
    ui.EngineLayer? oldLayer,
  }) {
    return pushLayer<ClipRRectEngineLayer>(ClipRRectEngineLayer(rrect, clipBehavior));
  }

  @override
  ClipRectEngineLayer pushClipRect(
    ui.Rect rect, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.EngineLayer? oldLayer,
  }) {
    return pushLayer<ClipRectEngineLayer>(ClipRectEngineLayer(rect, clipBehavior));
  }

  @override
  ColorFilterEngineLayer pushColorFilter(
    ui.ColorFilter filter, {
    ui.ColorFilterEngineLayer? oldLayer,
  }) {
    return pushLayer<ColorFilterEngineLayer>(ColorFilterEngineLayer(filter));
  }

  @override
  ImageFilterEngineLayer pushImageFilter(
    ui.ImageFilter filter, {
    ui.ImageFilterEngineLayer? oldLayer,
    ui.Offset offset = ui.Offset.zero,
  }) {
    return pushLayer<ImageFilterEngineLayer>(ImageFilterEngineLayer(filter, offset));
  }

  @override
  OffsetEngineLayer pushOffset(double dx, double dy, {ui.EngineLayer? oldLayer}) {
    return pushLayer<OffsetEngineLayer>(OffsetEngineLayer(dx, dy));
  }

  @override
  OpacityEngineLayer pushOpacity(
    int alpha, {
    ui.EngineLayer? oldLayer,
    ui.Offset offset = ui.Offset.zero,
  }) {
    return pushLayer<OpacityEngineLayer>(OpacityEngineLayer(alpha, offset));
  }

  @override
  ShaderMaskEngineLayer pushShaderMask(
    ui.Shader shader,
    ui.Rect maskRect,
    ui.BlendMode blendMode, {
    ui.EngineLayer? oldLayer,
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) {
    return pushLayer<ShaderMaskEngineLayer>(
      ShaderMaskEngineLayer(shader, maskRect, blendMode, filterQuality),
    );
  }

  @override
  TransformEngineLayer pushTransform(Float64List matrix4, {ui.EngineLayer? oldLayer}) {
    final Matrix4 matrix = Matrix4.fromFloat32List(toMatrix32(matrix4));
    return pushLayer<TransformEngineLayer>(TransformEngineLayer(matrix));
  }

  T pushLayer<T extends ContainerLayer>(T layer) {
    currentLayer.add(layer);
    currentLayer = layer;
    return layer;
  }

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
    throw UnimplementedError();
  }
}
