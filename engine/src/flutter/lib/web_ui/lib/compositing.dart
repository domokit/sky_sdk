// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of ui;

abstract class Scene {
  Future<Image> toImage(int width, int height);
  Image toImageSync(int width, int height);
  void dispose();
}

abstract class TransformEngineLayer implements EngineLayer {}

abstract class OffsetEngineLayer implements EngineLayer {}

abstract class ClipRectEngineLayer implements EngineLayer {}

abstract class ClipRRectEngineLayer implements EngineLayer {}

abstract class ClipPathEngineLayer implements EngineLayer {}

abstract class OpacityEngineLayer implements EngineLayer {}

abstract class ColorFilterEngineLayer implements EngineLayer {}

abstract class ImageFilterEngineLayer implements EngineLayer {}

abstract class BackdropFilterEngineLayer implements EngineLayer {}

abstract class ShaderMaskEngineLayer implements EngineLayer {}

abstract class SceneBuilder {
  factory SceneBuilder() => engine.renderer.createSceneBuilder();

  OffsetEngineLayer pushOffset(double dx, double dy, {OffsetEngineLayer? oldLayer});
  TransformEngineLayer pushTransform(Float64List matrix4, {TransformEngineLayer? oldLayer});
  ClipRectEngineLayer pushClipRect(
    Rect rect, {
    Clip clipBehavior = Clip.antiAlias,
    ClipRectEngineLayer? oldLayer,
  });
  ClipRRectEngineLayer pushClipRRect(
    RRect rrect, {
    required Clip clipBehavior,
    ClipRRectEngineLayer? oldLayer,
  });
  ClipPathEngineLayer pushClipPath(
    Path path, {
    Clip clipBehavior = Clip.antiAlias,
    ClipPathEngineLayer? oldLayer,
  });
  OpacityEngineLayer pushOpacity(
    int alpha, {
    Offset offset = Offset.zero,
    OpacityEngineLayer? oldLayer,
  });
  ColorFilterEngineLayer pushColorFilter(ColorFilter filter, {ColorFilterEngineLayer? oldLayer});
  ImageFilterEngineLayer pushImageFilter(
    ImageFilter filter, {
    Offset offset = Offset.zero,
    ImageFilterEngineLayer? oldLayer,
  });
  BackdropFilterEngineLayer pushBackdropFilter(
    ImageFilter filter, {
    BlendMode blendMode = BlendMode.srcOver,
    BackdropFilterEngineLayer? oldLayer,
    int? backdropId,
  });
  ShaderMaskEngineLayer pushShaderMask(
    Shader shader,
    Rect maskRect,
    BlendMode blendMode, {
    ShaderMaskEngineLayer? oldLayer,
    FilterQuality filterQuality = FilterQuality.low,
  });
  void addRetained(EngineLayer retainedLayer);
  void pop();
  void addPerformanceOverlay(int enabledOptions, Rect bounds);
  void addPicture(
    Offset offset,
    Picture picture, {
    bool isComplexHint = false,
    bool willChangeHint = false,
  });
  void addTexture(
    int textureId, {
    Offset offset = Offset.zero,
    double width = 0.0,
    double height = 0.0,
    bool freeze = false,
    FilterQuality filterQuality = FilterQuality.low,
  });
  void addPlatformView(
    int viewId, {
    Offset offset = Offset.zero,
    double width = 0.0,
    double height = 0.0,
  });
  Scene build();
  void setProperties(
    double width,
    double height,
    double insetTop,
    double insetRight,
    double insetBottom,
    double insetLeft,
    bool focusable,
  );
}

class EngineLayer {
  void dispose() {}
}
