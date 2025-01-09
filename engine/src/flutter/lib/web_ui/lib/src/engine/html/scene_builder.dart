// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../engine.dart' show FrameTimingRecorder, kProfileApplyFrame, kProfilePrerollFrame;
import '../display.dart';
import '../dom.dart';
import '../profiler.dart';
import '../util.dart';
import '../vector_math.dart';
import 'backdrop_filter.dart';
import 'clip.dart';
import 'color_filter.dart';
import 'image_filter.dart';
import 'offset.dart';
import 'opacity.dart';
import 'path_to_svg_clip.dart';
import 'picture.dart';
import 'platform_view.dart';
import 'scene.dart';
import 'shader_mask.dart';
import 'surface.dart';
import 'transform.dart';

class SurfaceSceneBuilder implements ui.SceneBuilder {
  SurfaceSceneBuilder() {
    _surfaceStack.add(PersistedScene(_lastFrameScene));
  }

  final List<PersistedContainerSurface> _surfaceStack = <PersistedContainerSurface>[];

  /// The scene built by this scene builder.
  ///
  /// This getter should only be called after all surfaces are built.
  PersistedScene get _persistedScene {
    return _surfaceStack.first as PersistedScene;
  }

  /// The surface currently being built.
  PersistedContainerSurface get _currentSurface => _surfaceStack.last;

  T _pushSurface<T extends PersistedContainerSurface>(T surface) {
    // Only attempt to update if the update is requested and the surface is in
    // the live tree.
    if (surface.oldLayer != null) {
      assert(surface.oldLayer!.runtimeType == surface.runtimeType);
      assert(debugAssertSurfaceState(surface.oldLayer!, PersistedSurfaceState.active));
      surface.oldLayer!.state = PersistedSurfaceState.pendingUpdate;
    }
    _adoptSurface(surface);
    _surfaceStack.add(surface);
    return surface;
  }

  /// Adds [surface] to the surface tree.
  ///
  /// This is used by tests.
  void debugAddSurface(PersistedSurface surface) {
    assert(() {
      _addSurface(surface);
      return true;
    }());
  }

  void _addSurface(PersistedSurface surface) {
    _adoptSurface(surface);
  }

  void _adoptSurface(PersistedSurface surface) {
    _currentSurface.appendChild(surface);
  }

  /// Pushes an offset operation onto the operation stack.
  ///
  /// This is equivalent to [pushTransform] with a matrix with only translation.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.OffsetEngineLayer pushOffset(double dx, double dy, {ui.OffsetEngineLayer? oldLayer}) {
    return _pushSurface<PersistedOffset>(PersistedOffset(oldLayer as PersistedOffset?, dx, dy));
  }

  /// Pushes a transform operation onto the operation stack.
  ///
  /// The objects are transformed by the given matrix before rasterization.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.TransformEngineLayer pushTransform(Float64List matrix4, {ui.TransformEngineLayer? oldLayer}) {
    if (matrix4.length != 16) {
      throw ArgumentError('"matrix4" must have 16 entries.');
    }

    final Float32List matrix;
    if (_surfaceStack.length == 1) {
      // Top level transform contains view configuration to scale
      // scene to devicepixelratio. Use identity instead since CSS uses
      // logical device pixels.
      if (!ui_web.debugEmulateFlutterTesterEnvironment) {
        assert(
          matrix4[0] == EngineFlutterDisplay.instance.devicePixelRatio &&
              matrix4[5] == EngineFlutterDisplay.instance.devicePixelRatio,
        );
      }
      matrix = Matrix4.identity().storage;
    } else {
      matrix = toMatrix32(matrix4);
    }
    return _pushSurface<PersistedTransform>(
      PersistedTransform(oldLayer as PersistedTransform?, matrix),
    );
  }

  /// Pushes a rectangular clip operation onto the operation stack.
  ///
  /// Rasterization outside the given rectangle is discarded.
  ///
  /// See [pop] for details about the operation stack, and [Clip] for different clip modes.
  /// By default, the clip will be anti-aliased (clip = [Clip.antiAlias]).
  @override
  ui.ClipRectEngineLayer pushClipRect(
    ui.Rect rect, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.ClipRectEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedClipRect>(
      PersistedClipRect(oldLayer as PersistedClipRect?, rect, clipBehavior),
    );
  }

  /// Pushes a rounded-rectangular clip operation onto the operation stack.
  ///
  /// Rasterization outside the given rounded rectangle is discarded.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.ClipRRectEngineLayer pushClipRRect(
    ui.RRect rrect, {
    ui.Clip? clipBehavior,
    ui.ClipRRectEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedClipRRect>(PersistedClipRRect(oldLayer, rrect, clipBehavior));
  }

  /// Pushes a path clip operation onto the operation stack.
  ///
  /// Rasterization outside the given path is discarded.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.ClipPathEngineLayer pushClipPath(
    ui.Path path, {
    ui.Clip clipBehavior = ui.Clip.antiAlias,
    ui.ClipPathEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedClipPath>(
      PersistedClipPath(oldLayer as PersistedClipPath?, path, clipBehavior),
    );
  }

  /// Pushes an opacity operation onto the operation stack.
  ///
  /// The given alpha value is blended into the alpha value of the objects'
  /// rasterization. An alpha value of 0 makes the objects entirely invisible.
  /// An alpha value of 255 has no effect (i.e., the objects retain the current
  /// opacity).
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.OpacityEngineLayer pushOpacity(
    int alpha, {
    ui.Offset offset = ui.Offset.zero,
    ui.OpacityEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedOpacity>(
      PersistedOpacity(oldLayer as PersistedOpacity?, alpha, offset),
    );
  }

  /// Pushes a color filter operation onto the operation stack.
  ///
  /// The given color is applied to the objects' rasterization using the given
  /// blend mode.
  ///
  /// {@macro dart.ui.sceneBuilder.oldLayer}
  ///
  /// {@macro dart.ui.sceneBuilder.oldLayerVsRetained}
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.ColorFilterEngineLayer pushColorFilter(
    ui.ColorFilter filter, {
    ui.ColorFilterEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedColorFilter>(
      PersistedColorFilter(oldLayer as PersistedColorFilter?, filter),
    );
  }

  /// Pushes an image filter operation onto the operation stack.
  ///
  /// The given filter is applied to the children's rasterization before compositing them into
  /// the scene.
  ///
  /// {@macro dart.ui.sceneBuilder.oldLayer}
  ///
  /// {@macro dart.ui.sceneBuilder.oldLayerVsRetained}
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.ImageFilterEngineLayer pushImageFilter(
    ui.ImageFilter filter, {
    ui.Offset offset = ui.Offset.zero,
    ui.ImageFilterEngineLayer? oldLayer,
  }) {
    return _pushSurface<PersistedImageFilter>(
      PersistedImageFilter(oldLayer as PersistedImageFilter?, filter, offset),
    );
  }

  /// Pushes a backdrop filter operation onto the operation stack.
  ///
  /// The given filter is applied to the current contents of the scene prior to
  /// rasterizing the child layers.
  ///
  /// The [blendMode] argument is required for [ui.SceneBuilder] compatibility, but is
  /// ignored by the DOM renderer.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.BackdropFilterEngineLayer pushBackdropFilter(
    ui.ImageFilter filter, {
    ui.BlendMode blendMode = ui.BlendMode.srcOver,
    ui.BackdropFilterEngineLayer? oldLayer,
    int? backdropId,
  }) {
    return _pushSurface<PersistedBackdropFilter>(
      PersistedBackdropFilter(oldLayer as PersistedBackdropFilter?, filter),
    );
  }

  /// Pushes a shader mask operation onto the operation stack.
  ///
  /// The given shader is applied to the object's rasterization in the given
  /// rectangle using the given blend mode.
  ///
  /// See [pop] for details about the operation stack.
  @override
  ui.ShaderMaskEngineLayer pushShaderMask(
    ui.Shader shader,
    ui.Rect maskRect,
    ui.BlendMode blendMode, {
    ui.ShaderMaskEngineLayer? oldLayer,
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) {
    return _pushSurface<PersistedShaderMask>(
      PersistedShaderMask(
        oldLayer as PersistedShaderMask?,
        shader,
        maskRect,
        blendMode,
        filterQuality,
      ),
    );
  }

  /// Add a retained engine layer subtree from previous frames.
  ///
  /// All the engine layers that are in the subtree of the retained layer will
  /// be automatically appended to the current engine layer tree.
  ///
  /// Therefore, when implementing a subclass of the [Layer] concept defined in
  /// the rendering layer of Flutter's framework, once this is called, there's
  /// no need to call [addToScene] for its children layers.
  @override
  void addRetained(ui.EngineLayer retainedLayer) {
    final PersistedContainerSurface retainedSurface = retainedLayer as PersistedContainerSurface;
    assert(
      debugAssertSurfaceState(
        retainedSurface,
        PersistedSurfaceState.active,
        PersistedSurfaceState.released,
      ),
    );
    retainedSurface.tryRetain();
    _adoptSurface(retainedSurface);
  }

  /// Ends the effect of the most recently pushed operation.
  ///
  /// Internally the scene builder maintains a stack of operations. Each of the
  /// operations in the stack applies to each of the objects added to the scene.
  /// Calling this function removes the most recently added operation from the
  /// stack.
  @override
  void pop() {
    assert(_surfaceStack.isNotEmpty);
    _surfaceStack.removeLast();
  }

  /// Adds an object to the scene that displays performance statistics.
  ///
  /// Useful during development to assess the performance of the application.
  /// The enabledOptions controls which statistics are displayed. The bounds
  /// controls where the statistics are displayed.
  ///
  /// enabledOptions is a bit field with the following bits defined:
  ///  - 0x01: displayRasterizerStatistics - show raster thread frame time
  ///  - 0x02: visualizeRasterizerStatistics - graph raster thread frame times
  ///  - 0x04: displayEngineStatistics - show UI thread frame time
  ///  - 0x08: visualizeEngineStatistics - graph UI thread frame times
  /// Set enabledOptions to 0x0F to enable all the currently defined features.
  ///
  /// The "UI thread" is the thread that includes all the execution of the main
  /// Dart isolate (the isolate that can call [FlutterView.render]). The UI
  /// thread frame time is the total time spent executing the
  /// [FlutterView.onBeginFrame] callback. The "raster thread" is the thread
  /// (running on the CPU) that subsequently processes the [Scene] provided by
  /// the Dart code to turn it into GPU commands and send it to the GPU.
  ///
  /// See also the [PerformanceOverlayOption] enum in the rendering library.
  /// for more details.
  @override
  void addPerformanceOverlay(int enabledOptions, ui.Rect bounds) {
    _addPerformanceOverlay(enabledOptions, bounds.left, bounds.right, bounds.top, bounds.bottom);
  }

  /// Whether we've already warned the user about the lack of the performance
  /// overlay or not.
  ///
  /// We use this to avoid spamming the console with redundant warning messages.
  static bool _didWarnAboutPerformanceOverlay = false;

  void _addPerformanceOverlay(
    int enabledOptions,
    double left,
    double right,
    double top,
    double bottom,
  ) {
    if (!_didWarnAboutPerformanceOverlay) {
      _didWarnAboutPerformanceOverlay = true;
      printWarning("The performance overlay isn't supported on the web");
    }
  }

  /// Adds a [Picture] to the scene.
  ///
  /// The picture is rasterized at the given offset.
  @override
  void addPicture(
    ui.Offset offset,
    ui.Picture picture, {
    bool isComplexHint = false,
    bool willChangeHint = false,
  }) {
    int hints = 0;
    if (isComplexHint) {
      hints |= 1;
    }
    if (willChangeHint) {
      hints |= 2;
    }
    _addSurface(PersistedPicture(offset.dx, offset.dy, picture as EnginePicture, hints));
  }

  /// Adds a backend texture to the scene.
  ///
  /// The texture is scaled to the given size and rasterized at the given
  /// offset.
  @override
  void addTexture(
    int textureId, {
    ui.Offset offset = ui.Offset.zero,
    double width = 0.0,
    double height = 0.0,
    bool freeze = false,
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) {
    _addTexture(offset.dx, offset.dy, width, height, textureId, filterQuality);
  }

  void _addTexture(
    double dx,
    double dy,
    double width,
    double height,
    int textureId,
    ui.FilterQuality filterQuality,
  ) {
    // In test mode, allow this to be a no-op.
    if (!ui_web.debugEmulateFlutterTesterEnvironment) {
      throw UnimplementedError('Textures are not supported in Flutter Web');
    }
  }

  /// Adds a platform view (e.g an iOS UIView) to the scene.
  ///
  /// Only supported on iOS, this is currently a no-op on other platforms.
  ///
  /// On iOS this layer splits the current output surface into two surfaces, one for the scene nodes
  /// preceding the platform view, and one for the scene nodes following the platform view.
  ///
  /// ## Performance impact
  ///
  /// Adding an additional surface doubles the amount of graphics memory directly used by Flutter
  /// for output buffers. Quartz might allocated extra buffers for compositing the Flutter surfaces
  /// and the platform view.
  ///
  /// With a platform view in the scene, Quartz has to composite the two Flutter surfaces and the
  /// embedded UIView. In addition to that, on iOS versions greater than 9, the Flutter frames are
  /// synchronized with the UIView frames adding additional performance overhead.
  @override
  void addPlatformView(
    int viewId, {
    ui.Offset offset = ui.Offset.zero,
    double width = 0.0,
    double height = 0.0,
  }) {
    _addPlatformView(offset.dx, offset.dy, width, height, viewId);
  }

  void _addPlatformView(double dx, double dy, double width, double height, int viewId) {
    _addSurface(PersistedPlatformView(viewId, dx, dy, width, height));
  }

  /// The scene recorded in the last frame.
  ///
  /// This is a surface tree that holds onto the DOM elements that can be reused
  /// on the next frame.
  static PersistedScene? _lastFrameScene;

  /// Returns the computed persisted scene graph recorded in the last frame.
  ///
  /// This is only available in debug mode. It returns `null` in profile and
  /// release modes.
  static PersistedScene? get debugLastFrameScene {
    PersistedScene? result;
    assert(() {
      result = _lastFrameScene;
      return true;
    }());
    return result;
  }

  /// Discards information about previously rendered frames, including DOM
  /// elements and cached canvases.
  ///
  /// After calling this function new canvases will be created for the
  /// subsequent scene. This is useful when tests need predictable canvas
  /// sizes. If the cache is not cleared, then canvases allocated in one test
  /// may be reused in another test.
  static void debugForgetFrameScene() {
    _lastFrameScene?.rootElement?.remove();
    _lastFrameScene = null;
    resetSvgClipIds();
    recycledCanvases.clear();
  }

  /// Finishes building the scene.
  ///
  /// Returns a [Scene] containing the objects that have been added to
  /// this scene builder. The [Scene] can then be displayed on the
  /// screen with [FlutterView.render].
  ///
  /// After calling this function, the scene builder object is invalid and
  /// cannot be used further.
  @override
  SurfaceScene build() {
    // "Build finish" and "raster start" happen back-to-back because we
    // render on the same thread, so there's no overhead from hopping to
    // another thread.
    //
    // In the HTML renderer we time the beginning of the rasterization phase
    // (counter-intuitively) in SceneBuilder.build because DOM updates happen
    // here. This is different from CanvasKit.
    final FrameTimingRecorder? recorder =
        FrameTimingRecorder.frameTimingsEnabled ? FrameTimingRecorder() : null;
    recorder?.recordBuildFinish();
    recorder?.recordRasterStart();
    timeAction<void>(kProfilePrerollFrame, () {
      while (_surfaceStack.length > 1) {
        // Auto-pop layers that were pushed without a corresponding pop.
        pop();
      }
      _persistedScene.preroll(PrerollSurfaceContext());
    });
    return timeAction<SurfaceScene>(kProfileApplyFrame, () {
      if (_lastFrameScene == null) {
        _persistedScene.build();
      } else {
        _persistedScene.update(_lastFrameScene!);
      }
      commitScene(_persistedScene);
      _lastFrameScene = _persistedScene;
      return SurfaceScene(_persistedScene.rootElement, timingRecorder: recorder);
    });
  }

  /// Set properties on the linked scene.  These properties include its bounds,
  /// as well as whether it can be the target of focus events or not.
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
