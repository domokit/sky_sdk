// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

const String kCanvasContainerTag = 'flt-canvas-container';

typedef RenderResult =
    ({List<DomImageBitmap> imageBitmaps, int rasterStartMicros, int rasterEndMicros});

// This is an interface that renders a `ScenePicture` as a `DomImageBitmap`.
// It is optionally asynchronous. It is required for the `EngineSceneView` to
// composite pictures into the canvases in the DOM tree it builds.
abstract class PictureRenderer {
  FutureOr<RenderResult> renderPictures(List<ScenePicture> picture);
  ScenePicture clipPicture(ScenePicture picture, ui.Rect clip);
}

class _SceneRender {
  _SceneRender(this.scene, this._completer, {this.recorder}) {
    scene.beginRender();
  }

  final EngineScene scene;
  final Completer<void> _completer;
  final FrameTimingRecorder? recorder;

  void done() {
    scene.endRender();
    _completer.complete();
  }
}

// This class builds a DOM tree that composites an `EngineScene`.
class EngineSceneView {
  factory EngineSceneView(PictureRenderer pictureRenderer, EngineFlutterView flutterView) {
    final DomElement sceneElement = createDomElement('flt-scene');
    return EngineSceneView._(pictureRenderer, flutterView, sceneElement);
  }

  EngineSceneView._(this.pictureRenderer, this.flutterView, this.sceneElement);

  final PictureRenderer pictureRenderer;
  final DomElement sceneElement;
  final EngineFlutterView flutterView;

  List<SliceContainer> containers = <SliceContainer>[];

  _SceneRender? _currentRender;
  _SceneRender? _nextRender;

  Future<void> renderScene(EngineScene scene, FrameTimingRecorder? recorder) {
    if (_currentRender != null) {
      // If a scene is already queued up, drop it and queue this one up instead
      // so that the scene view always displays the most recently requested scene.
      _nextRender?.done();
      final Completer<void> completer = Completer<void>();
      _nextRender = _SceneRender(scene, completer, recorder: recorder);
      return completer.future;
    }
    final Completer<void> completer = Completer<void>();
    _currentRender = _SceneRender(scene, completer, recorder: recorder);
    _kickRenderLoop();
    return completer.future;
  }

  Future<void> _kickRenderLoop() async {
    final _SceneRender current = _currentRender!;
    await _renderScene(current.scene, current.recorder);
    current.done();
    _currentRender = _nextRender;
    _nextRender = null;
    if (_currentRender == null) {
      return;
    } else {
      return _kickRenderLoop();
    }
  }

  Future<void> _renderScene(EngineScene scene, FrameTimingRecorder? recorder) async {
    final ui.Rect screenBounds = ui.Rect.fromLTWH(
      0,
      0,
      flutterView.physicalSize.width,
      flutterView.physicalSize.height,
    );
    final List<LayerSlice?> slices = scene.rootLayer.slices;
    final List<ScenePicture> picturesToRender = <ScenePicture>[];
    final List<ScenePicture> originalPicturesToRender = <ScenePicture>[];
    for (final LayerSlice? slice in slices) {
      if (slice == null) {
        continue;
      }
      final ui.Rect clippedRect = slice.picture.cullRect.intersect(screenBounds);
      if (clippedRect.isEmpty) {
        // This picture is completely offscreen, so don't render it at all
        continue;
      } else if (clippedRect == slice.picture.cullRect) {
        // The picture doesn't need to be clipped, just render the original
        originalPicturesToRender.add(slice.picture);
        picturesToRender.add(slice.picture);
      } else {
        originalPicturesToRender.add(slice.picture);
        picturesToRender.add(pictureRenderer.clipPicture(slice.picture, clippedRect));
      }
    }
    final Map<ScenePicture, DomImageBitmap> renderMap;
    if (picturesToRender.isNotEmpty) {
      final RenderResult renderResult = await pictureRenderer.renderPictures(picturesToRender);
      renderMap = <ScenePicture, DomImageBitmap>{
        for (int i = 0; i < picturesToRender.length; i++)
          originalPicturesToRender[i]: renderResult.imageBitmaps[i],
      };
      recorder?.recordRasterStart(renderResult.rasterStartMicros);
      recorder?.recordRasterFinish(renderResult.rasterEndMicros);
    } else {
      renderMap = <ScenePicture, DomImageBitmap>{};
      recorder?.recordRasterStart();
      recorder?.recordRasterFinish();
    }
    recorder?.submitTimings();

    final List<SliceContainer?> reusableContainers = List<SliceContainer?>.from(containers);
    final List<SliceContainer> newContainers = <SliceContainer>[];
    for (final LayerSlice? slice in slices) {
      if (slice == null) {
        continue;
      }
      final DomImageBitmap? bitmap = renderMap[slice.picture];
      if (bitmap != null) {
        PictureSliceContainer? container;
        for (int j = 0; j < reusableContainers.length; j++) {
          final SliceContainer? candidate = reusableContainers[j];
          if (candidate is PictureSliceContainer) {
            container = candidate;
            reusableContainers[j] = null;
            break;
          }
        }

        final ui.Rect clippedBounds = slice.picture.cullRect.intersect(screenBounds);
        if (container != null) {
          container.bounds = clippedBounds;
        } else {
          container = PictureSliceContainer(clippedBounds);
        }
        container.updateContents();
        container.renderBitmap(bitmap);
        newContainers.add(container);
      }

      for (final PlatformView view in slice.platformViews) {
        // Ensure the contents of the platform view are injected into the DOM.
        flutterView.dom.injectPlatformView(view.viewId);

        // Attempt to reuse a container for the existing view
        PlatformViewContainer? container;
        for (int j = 0; j < reusableContainers.length; j++) {
          final SliceContainer? candidate = reusableContainers[j];
          if (candidate is PlatformViewContainer && candidate.viewId == view.viewId) {
            container = candidate;
            reusableContainers[j] = null;
            break;
          }
        }
        container ??= PlatformViewContainer(view.viewId);
        container.bounds = view.bounds;
        container.styling = view.styling;
        container.updateContents();
        newContainers.add(container);
      }
    }

    for (final SliceContainer? container in reusableContainers) {
      if (container != null) {
        sceneElement.removeChild(container.container);
      }
    }

    containers = newContainers;

    DomElement? currentElement = sceneElement.firstElementChild;
    for (final SliceContainer container in containers) {
      if (currentElement == null) {
        sceneElement.appendChild(container.container);
      } else if (currentElement == container.container) {
        currentElement = currentElement.nextElementSibling;
      } else {
        sceneElement.insertBefore(container.container, currentElement);
      }
    }
  }
}

sealed class SliceContainer {
  DomElement get container;

  void updateContents();
}

final class PictureSliceContainer extends SliceContainer {
  factory PictureSliceContainer(ui.Rect bounds) {
    final DomElement container = domDocument.createElement(kCanvasContainerTag);
    final DomCanvasElement canvas = createDomCanvasElement(
      width: bounds.width.toInt(),
      height: bounds.height.toInt(),
    );
    container.appendChild(canvas);
    return PictureSliceContainer._(bounds, container, canvas);
  }

  PictureSliceContainer._(this._bounds, this.container, this.canvas);

  ui.Rect _bounds;
  bool _dirty = true;

  ui.Rect get bounds => _bounds;
  set bounds(ui.Rect bounds) {
    if (_bounds != bounds) {
      _bounds = bounds;
      _dirty = true;
    }
  }

  @override
  void updateContents() {
    if (_dirty) {
      _dirty = false;

      final ui.Rect roundedOutBounds = ui.Rect.fromLTRB(
        bounds.left.floorToDouble(),
        bounds.top.floorToDouble(),
        bounds.right.ceilToDouble(),
        bounds.bottom.ceilToDouble(),
      );
      final DomCSSStyleDeclaration style = canvas.style;
      final double devicePixelRatio = EngineFlutterDisplay.instance.devicePixelRatio;
      final double logicalWidth = roundedOutBounds.width / devicePixelRatio;
      final double logicalHeight = roundedOutBounds.height / devicePixelRatio;
      final double logicalLeft = roundedOutBounds.left / devicePixelRatio;
      final double logicalTop = roundedOutBounds.top / devicePixelRatio;
      style.width = '${logicalWidth}px';
      style.height = '${logicalHeight}px';
      style.position = 'absolute';
      style.left = '${logicalLeft}px';
      style.top = '${logicalTop}px';
      canvas.width = roundedOutBounds.width.ceilToDouble();
      canvas.height = roundedOutBounds.height.ceilToDouble();
    }
  }

  void renderBitmap(DomImageBitmap bitmap) {
    final DomCanvasRenderingContextBitmapRenderer ctx = canvas.contextBitmapRenderer;
    ctx.transferFromImageBitmap(bitmap);
  }

  @override
  final DomElement container;
  final DomCanvasElement canvas;
}

final class PlatformViewContainer extends SliceContainer {
  PlatformViewContainer(this.viewId)
    : container = createDomElement('flt-clip'),
      slot = createPlatformViewSlot(viewId) {
    container.appendChild(slot);
  }

  final int viewId;
  PlatformViewStyling? _styling;
  ui.Rect? _bounds;
  bool _dirty = false;

  ui.Path? _clipPath;
  String? _clipPathString;

  @override
  final DomElement container;

  final DomElement slot;

  set styling(PlatformViewStyling styling) {
    if (_styling != styling) {
      _styling = styling;
      _dirty = true;
    }
  }

  set bounds(ui.Rect bounds) {
    if (_bounds != bounds) {
      _bounds = bounds;
      _dirty = true;
    }
  }

  set clipPath(ScenePath? path) {
    if (_clipPath == path) {
      return;
    }

    _clipPath = path;
    _clipPathString = path?.toSvgString();
  }

  String cssStringForClip(PlatformViewClip clip, double devicePixelRatio) {
    switch (clip) {
      case PlatformViewNoClip():
        clipPath = null;
        return '';
      case PlatformViewRectClip():
        clipPath = null;
        final double top = clip.rect.top / devicePixelRatio;
        final double right = clip.rect.right / devicePixelRatio;
        final double bottom = clip.rect.bottom / devicePixelRatio;
        final double left = clip.rect.left / devicePixelRatio;
        return 'rect(${top}px ${right}px ${bottom}px ${left}px)';
      case PlatformViewRRectClip():
        clipPath = null;
        final double top = clip.rrect.top / devicePixelRatio;
        final double right = clip.rrect.right / devicePixelRatio;
        final double bottom = clip.rrect.bottom / devicePixelRatio;
        final double left = clip.rrect.left / devicePixelRatio;
        final double tlRadiusX = clip.rrect.tlRadiusX / devicePixelRatio;
        final double tlRadiusY = clip.rrect.tlRadiusY / devicePixelRatio;
        final double trRadiusX = clip.rrect.trRadiusX / devicePixelRatio;
        final double trRadiusY = clip.rrect.trRadiusY / devicePixelRatio;
        final double brRadiusX = clip.rrect.brRadiusX / devicePixelRatio;
        final double brRadiusY = clip.rrect.brRadiusY / devicePixelRatio;
        final double blRadiusX = clip.rrect.blRadiusX / devicePixelRatio;
        final double blRadiusY = clip.rrect.blRadiusY / devicePixelRatio;
        return 'rect(${top}px ${right}px ${bottom}px ${left}px round ${tlRadiusX}px ${trRadiusX}px ${brRadiusX}px ${blRadiusX}px / ${tlRadiusY}px ${trRadiusY}px ${brRadiusY}px ${blRadiusY}px)';
      case PlatformViewPathClip():
        clipPath = clip.path;
        return "path('$_clipPathString')";
    }
  }

  @override
  void updateContents() {
    assert(_styling != null);
    assert(_bounds != null);
    if (_dirty) {
      final DomCSSStyleDeclaration style = slot.style;
      style.position = 'absolute';
      style.width = '${_bounds!.width}px';
      style.height = '${_bounds!.height}px';

      final double devicePixelRatio = EngineFlutterDisplay.instance.devicePixelRatio;
      final PlatformViewPosition position = _styling!.position;

      final Matrix4 transform;
      if (position.transform != null) {
        transform = position.transform!.clone()..translate(_bounds!.left, _bounds!.top);
      } else {
        final ui.Offset offset = position.offset ?? ui.Offset.zero;
        transform = Matrix4.translationValues(
          _bounds!.left + offset.dx,
          _bounds!.top + offset.dy,
          0,
        );
      }
      final double inverseScale = 1.0 / devicePixelRatio;
      final Matrix4 scaleMatrix = Matrix4.diagonal3Values(inverseScale, inverseScale, 1);
      scaleMatrix.multiply(transform);
      style.transform = float64ListToCssTransform(scaleMatrix.storage);
      style.transformOrigin = '0 0 0';
      style.opacity = _styling!.opacity != 1.0 ? '${_styling!.opacity}' : '';

      final DomCSSStyleDeclaration containerStyle = container.style;
      containerStyle.position = 'absolute';
      containerStyle.width = '100%';
      containerStyle.height = '100%';

      final String clipPathString = cssStringForClip(_styling!.clip, devicePixelRatio);
      containerStyle.clipPath = clipPathString;

      _dirty = false;
    }
  }
}
