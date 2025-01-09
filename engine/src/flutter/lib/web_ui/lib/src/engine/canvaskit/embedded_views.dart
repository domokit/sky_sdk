// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'dart:math' as math;

import 'package:ui/ui.dart' as ui;

import '../../engine.dart' show PlatformViewManager, configuration, longestIncreasingSubsequence;
import '../display.dart';
import '../dom.dart';
import '../html/path_to_svg_clip.dart';
import '../platform_views/slots.dart';
import '../svg.dart';
import '../util.dart';
import '../vector_math.dart';
import 'canvas.dart';
import 'layer.dart';
import 'overlay_scene_optimizer.dart';
import 'painting.dart';
import 'path.dart';
import 'picture.dart';
import 'picture_recorder.dart';
import 'rasterizer.dart';

/// This composites HTML views into the [ui.Scene].
class HtmlViewEmbedder {
  HtmlViewEmbedder(this.sceneHost, this.rasterizer);

  final DomElement sceneHost;
  final ViewRasterizer rasterizer;

  /// The context for the current frame.
  EmbedderFrameContext _context = EmbedderFrameContext();

  /// The most recent composition parameters for a given view id.
  ///
  /// If we receive a request to composite a view, but the composition
  /// parameters haven't changed, we can avoid having to recompute the
  /// element stack that correctly composites the view into the scene.
  final Map<int, EmbeddedViewParams> _currentCompositionParams = <int, EmbeddedViewParams>{};

  /// The clip chain for a view Id.
  ///
  /// This contains:
  /// * The root view in the stack of mutator elements for the view id.
  /// * The slot view in the stack (what shows the actual platform view contents).
  /// * The number of clipping elements used last time the view was composited.
  final Map<int, ViewClipChain> _viewClipChains = <int, ViewClipChain>{};

  /// The maximum number of render canvases to create. Too many canvases can
  /// cause a performance burden.
  static int get maximumCanvases => configuration.canvasKitMaximumSurfaces;

  /// The views that need to be recomposited into the scene on the next frame.
  final Set<int> _viewsToRecomposite = <int>{};

  /// The list of view ids that should be composited, in order.
  final List<int> _compositionOrder = <int>[];

  /// The most recent composition order.
  final List<int> _activeCompositionOrder = <int>[];

  /// The most recent rendering.
  Rendering _activeRendering = Rendering();

  /// Returns the most recent rendering. Only used in tests.
  Rendering get debugActiveRendering => _activeRendering;

  /// If [debugOverlayOptimizationBounds] is true, this canvas will draw
  /// semitransparent rectangles showing the computed bounds of the platform
  /// views and pictures in the scene.
  DisplayCanvas? debugBoundsCanvas;

  /// The size of the frame, in physical pixels.
  late BitmapSize _frameSize;

  set frameSize(BitmapSize size) {
    _frameSize = size;
  }

  /// Returns a list of canvases for the optimized rendering. These are used in
  /// the paint step.
  Iterable<CkCanvas> getOptimizedCanvases() {
    return _context.optimizedCanvasRecorders!.map((CkPictureRecorder r) => r.recordingCanvas!);
  }

  void prerollCompositeEmbeddedView(int viewId, EmbeddedViewParams params) {
    // Do nothing if the params didn't change.
    if (_currentCompositionParams[viewId] == params) {
      // If the view was prerolled but not composited, then it needs to be
      // recomposited.
      if (!_activeCompositionOrder.contains(viewId)) {
        _viewsToRecomposite.add(viewId);
      }
      return;
    }
    _currentCompositionParams[viewId] = params;
    _viewsToRecomposite.add(viewId);
  }

  /// Adds the picture recorder associated with [picture] to the unoptimized
  /// scene.
  void addPictureToUnoptimizedScene(PictureLayer picture) {
    _context.sceneElements.add(PictureSceneElement(picture));
  }

  /// Prepares to composite [viewId].
  void compositeEmbeddedView(int viewId) {
    // Ensure platform view with `viewId` is injected into the `rasterizer.view`.
    rasterizer.view.dom.injectPlatformView(viewId);

    _compositionOrder.add(viewId);
    _context.sceneElements.add(PlatformViewSceneElement(viewId));

    if (_viewsToRecomposite.contains(viewId)) {
      _compositeWithParams(viewId, _currentCompositionParams[viewId]!);
      _viewsToRecomposite.remove(viewId);
    }
  }

  void _compositeWithParams(int platformViewId, EmbeddedViewParams params) {
    // If we haven't seen this viewId yet, cache it for clips/transforms.
    final ViewClipChain clipChain = _viewClipChains.putIfAbsent(platformViewId, () {
      return ViewClipChain(view: createPlatformViewSlot(platformViewId));
    });

    final DomElement slot = clipChain.slot;

    // See `apply()` in the PersistedPlatformView class for the HTML version
    // of this code.
    slot.style
      ..width = '${params.size.width}px'
      ..height = '${params.size.height}px'
      ..position = 'absolute';

    // Recompute the position in the DOM of the `slot` element...
    final int currentClippingCount = _countClips(params.mutators);
    final int previousClippingCount = clipChain.clipCount;
    if (currentClippingCount != previousClippingCount) {
      final DomElement oldPlatformViewRoot = clipChain.root;
      final DomElement newPlatformViewRoot = _reconstructClipViewsChain(
        currentClippingCount,
        slot,
        oldPlatformViewRoot,
      );
      // Store the updated root element, and clip count
      clipChain.updateClipChain(root: newPlatformViewRoot, clipCount: currentClippingCount);
    }

    // Apply mutators to the slot
    _applyMutators(params, slot, platformViewId);
  }

  int _countClips(MutatorsStack mutators) {
    int clipCount = 0;
    for (final Mutator mutator in mutators) {
      if (mutator.isClipType) {
        clipCount++;
      }
    }
    return clipCount;
  }

  DomElement _reconstructClipViewsChain(
    int numClips,
    DomElement platformView,
    DomElement headClipView,
  ) {
    DomNode? headClipViewNextSibling;
    bool headClipViewWasAttached = false;
    if (headClipView.parentNode != null) {
      headClipViewWasAttached = true;
      headClipViewNextSibling = headClipView.nextSibling;
      headClipView.remove();
    }
    DomElement head = platformView;
    int clipIndex = 0;
    // Re-use as much existing clip views as needed.
    while (head != headClipView && clipIndex < numClips) {
      head = head.parent!;
      clipIndex++;
    }
    // If there weren't enough existing clip views, add more.
    while (clipIndex < numClips) {
      final DomElement clippingView = createDomElement('flt-clip');
      clippingView.append(head);
      head = clippingView;
      clipIndex++;
    }
    head.remove();

    // If the chain was previously attached, attach it to the same position.
    if (headClipViewWasAttached) {
      sceneHost.insertBefore(head, headClipViewNextSibling);
    }
    return head;
  }

  /// Clean up the old SVG clip definitions, as this platform view is about to
  /// be recomposited.
  void _cleanUpClipDefs(int viewId) {
    if (_svgClipDefs.containsKey(viewId)) {
      final DomElement clipDefs = _svgPathDefs!.querySelector('#sk_path_defs')!;
      final List<DomElement> nodesToRemove = <DomElement>[];
      final Set<String> oldDefs = _svgClipDefs[viewId]!;
      for (final DomElement child in clipDefs.children) {
        if (oldDefs.contains(child.id)) {
          nodesToRemove.add(child);
        }
      }
      for (final DomElement node in nodesToRemove) {
        node.remove();
      }
      _svgClipDefs[viewId]!.clear();
    }
  }

  void _applyMutators(EmbeddedViewParams params, DomElement embeddedView, int viewId) {
    final MutatorsStack mutators = params.mutators;
    DomElement head = embeddedView;
    Matrix4 headTransform =
        params.offset == ui.Offset.zero
            ? Matrix4.identity()
            : Matrix4.translationValues(params.offset.dx, params.offset.dy, 0);
    double embeddedOpacity = 1.0;
    _resetAnchor(head);
    _cleanUpClipDefs(viewId);

    for (final Mutator mutator in mutators) {
      switch (mutator.type) {
        case MutatorType.transform:
          headTransform = mutator.matrix!.multiplied(headTransform);
          head.style.transform = float64ListToCssTransform(headTransform.storage);
        case MutatorType.clipRect:
        case MutatorType.clipRRect:
        case MutatorType.clipPath:
          final DomElement clipView = head.parent!;
          clipView.style.clip = '';
          clipView.style.clipPath = '';
          headTransform = Matrix4.identity();
          clipView.style.transform = '';
          // We need to set width and height for the clipView to cover the
          // bounds of the path since Safari seem to incorrectly intersect
          // the element bounding rect with the clip path.
          clipView.style.width = '100%';
          clipView.style.height = '100%';
          if (mutator.rect != null) {
            final ui.Rect rect = mutator.rect!;
            clipView.style.clip =
                'rect(${rect.top}px, ${rect.right}px, '
                '${rect.bottom}px, ${rect.left}px)';
          } else if (mutator.rrect != null) {
            final CkPath path = CkPath();
            path.addRRect(mutator.rrect!);
            _ensureSvgPathDefs();
            final DomElement pathDefs = _svgPathDefs!.querySelector('#sk_path_defs')!;
            _clipPathCount += 1;
            final String clipId = 'svgClip$_clipPathCount';
            final SVGClipPathElement newClipPath = createSVGClipPathElement();
            newClipPath.id = clipId;
            newClipPath.append(createSVGPathElement()..setAttribute('d', path.toSvgString()));

            pathDefs.append(newClipPath);
            // Store the id of the node instead of [newClipPath] directly. For
            // some reason, calling `newClipPath.remove()` doesn't remove it
            // from the DOM.
            _svgClipDefs.putIfAbsent(viewId, () => <String>{}).add(clipId);
            clipView.style.clipPath = 'url(#$clipId)';
          } else if (mutator.path != null) {
            final CkPath path = mutator.path! as CkPath;
            _ensureSvgPathDefs();
            final DomElement pathDefs = _svgPathDefs!.querySelector('#sk_path_defs')!;
            _clipPathCount += 1;
            final String clipId = 'svgClip$_clipPathCount';
            final SVGClipPathElement newClipPath = createSVGClipPathElement();
            newClipPath.id = clipId;
            newClipPath.append(createSVGPathElement()..setAttribute('d', path.toSvgString()));
            pathDefs.append(newClipPath);
            // Store the id of the node instead of [newClipPath] directly. For
            // some reason, calling `newClipPath.remove()` doesn't remove it
            // from the DOM.
            _svgClipDefs.putIfAbsent(viewId, () => <String>{}).add(clipId);
            clipView.style.clipPath = 'url(#$clipId)';
          }
          _resetAnchor(clipView);
          head = clipView;
        case MutatorType.opacity:
          embeddedOpacity *= mutator.alphaFloat;
      }
    }

    embeddedView.style.opacity = embeddedOpacity.toString();

    // Reverse scale based on screen scale.
    //
    // HTML elements use logical (CSS) pixels, but we have been using physical
    // pixels, so scale down the head element to match the logical resolution.
    final double scale = EngineFlutterDisplay.instance.devicePixelRatio;
    final double inverseScale = 1 / scale;
    final Matrix4 scaleMatrix = Matrix4.diagonal3Values(inverseScale, inverseScale, 1);
    headTransform = scaleMatrix.multiplied(headTransform);
    head.style.transform = float64ListToCssTransform(headTransform.storage);
  }

  /// Sets the transform origin to the top-left corner of the element.
  ///
  /// By default, the transform origin is the center of the element, but
  /// Flutter assumes the transform origin is the top-left point.
  void _resetAnchor(DomElement element) {
    element.style.transformOrigin = '0 0 0';
    element.style.position = 'absolute';
  }

  int _clipPathCount = 0;

  DomElement? _svgPathDefs;

  /// The nodes containing the SVG clip definitions needed to clip this view.
  final Map<int, Set<String>> _svgClipDefs = <int, Set<String>>{};

  /// Ensures we add a container of SVG path defs to the DOM so they can
  /// be referred to in clip-path: url(#blah).
  void _ensureSvgPathDefs() {
    if (_svgPathDefs != null) {
      return;
    }
    _svgPathDefs = kSvgResourceHeader.cloneNode(false) as SVGElement;
    _svgPathDefs!.append(createSVGDefsElement()..id = 'sk_path_defs');
    sceneHost.append(_svgPathDefs!);
  }

  /// Optimizes the scene to use the fewest possible canvases. This sets up
  /// the final paint pass to paint the pictures into the optimized canvases.
  void optimizeRendering() {
    Rendering rendering = createOptimizedRendering(
      _context.sceneElements,
      _currentCompositionParams,
    );
    rendering = _modifyRenderingForMaxCanvases(rendering);
    _context.optimizedRendering = rendering;
    // Create new picture recorders for the optimized render canvases and record
    // which pictures go in which canvas.
    final List<CkPictureRecorder> optimizedCanvasRecorders = <CkPictureRecorder>[];
    final Map<PictureLayer, CkPictureRecorder> pictureToOptimizedCanvasMap =
        <PictureLayer, CkPictureRecorder>{};
    for (final RenderingRenderCanvas renderCanvas in rendering.canvases) {
      final CkPictureRecorder pictureRecorder = CkPictureRecorder();
      pictureRecorder.beginRecording(ui.Offset.zero & _frameSize.toSize());
      optimizedCanvasRecorders.add(pictureRecorder);
      for (final PictureLayer picture in renderCanvas.pictures) {
        pictureToOptimizedCanvasMap[picture] = pictureRecorder;
      }
    }
    _context.optimizedCanvasRecorders = optimizedCanvasRecorders;
    _context.pictureToOptimizedCanvasMap = pictureToOptimizedCanvasMap;
  }

  /// Returns the canvas that this picture layer should draw into in the
  /// optimized scene.
  CkCanvas getOptimizedCanvasFor(PictureLayer picture) {
    assert(_context.optimizedRendering != null);
    return _context.pictureToOptimizedCanvasMap![picture]!.recordingCanvas!;
  }

  Future<void> submitFrame() async {
    final Rendering rendering = _context.optimizedRendering!;
    _updateDomForNewRendering(rendering);
    if (rendering.equalsForRendering(_activeRendering)) {
      // Copy the display canvases to the new rendering.
      for (int i = 0; i < rendering.canvases.length; i++) {
        rendering.canvases[i].displayCanvas = _activeRendering.canvases[i].displayCanvas;
        _activeRendering.canvases[i].displayCanvas = null;
      }
    }
    _activeRendering = rendering;

    final List<RenderingRenderCanvas> renderCanvases = rendering.canvases;
    int renderCanvasIndex = 0;
    for (final RenderingRenderCanvas renderCanvas in renderCanvases) {
      final CkPicture renderPicture =
          _context.optimizedCanvasRecorders![renderCanvasIndex++].endRecording();
      await rasterizer.rasterizeToCanvas(renderCanvas.displayCanvas!, <CkPicture>[renderPicture]);
      renderPicture.dispose();
    }

    for (final CkPictureRecorder recorder in _context.measuringPictureRecorders.values) {
      if (recorder.isRecording) {
        recorder.endRecording();
      }
    }

    // Draw the computed bounds for pictures and platform views if overlay
    // optimization debugging is enabled.
    if (debugOverlayOptimizationBounds) {
      debugBoundsCanvas ??= rasterizer.displayFactory.getCanvas();
      final CkPictureRecorder boundsRecorder = CkPictureRecorder();
      final CkCanvas boundsCanvas = boundsRecorder.beginRecording(
        ui.Rect.fromLTWH(0, 0, _frameSize.width.toDouble(), _frameSize.height.toDouble()),
      );
      final CkPaint platformViewBoundsPaint =
          CkPaint()..color = const ui.Color.fromARGB(100, 0, 255, 0);
      final CkPaint pictureBoundsPaint = CkPaint()..color = const ui.Color.fromARGB(100, 0, 0, 255);
      for (final RenderingEntity entity in _activeRendering.entities) {
        if (entity is RenderingPlatformView) {
          if (entity.debugComputedBounds != null) {
            boundsCanvas.drawRect(entity.debugComputedBounds!, platformViewBoundsPaint);
          }
        } else if (entity is RenderingRenderCanvas) {
          for (final PictureLayer picture in entity.pictures) {
            boundsCanvas.drawRect(picture.sceneBounds!, pictureBoundsPaint);
          }
        }
      }
      await rasterizer.rasterizeToCanvas(debugBoundsCanvas!, <CkPicture>[
        boundsRecorder.endRecording(),
      ]);
      sceneHost.append(debugBoundsCanvas!.hostElement);
    }

    // Reset the context.
    _context = EmbedderFrameContext();
    if (listEquals(_compositionOrder, _activeCompositionOrder)) {
      _compositionOrder.clear();
      return;
    }

    final Set<int> unusedViews = Set<int>.from(_activeCompositionOrder);
    _activeCompositionOrder.clear();

    List<int>? debugInvalidViewIds;

    for (int i = 0; i < _compositionOrder.length; i++) {
      final int viewId = _compositionOrder[i];

      bool isViewInvalid = false;
      assert(() {
        isViewInvalid = !PlatformViewManager.instance.knowsViewId(viewId);
        if (isViewInvalid) {
          debugInvalidViewIds ??= <int>[];
          debugInvalidViewIds!.add(viewId);
        }
        return true;
      }());
      if (isViewInvalid) {
        continue;
      }

      _activeCompositionOrder.add(viewId);
      unusedViews.remove(viewId);
    }

    _compositionOrder.clear();

    unusedViews.forEach(disposeView);

    assert(
      debugInvalidViewIds == null || debugInvalidViewIds!.isEmpty,
      'Cannot render platform views: ${debugInvalidViewIds!.join(', ')}. '
      'These views have not been created, or they have been deleted.',
    );
  }

  void disposeView(int viewId) {
    final ViewClipChain? clipChain = _viewClipChains.remove(viewId);
    clipChain?.root.remove();
    // More cleanup
    _currentCompositionParams.remove(viewId);
    _viewsToRecomposite.remove(viewId);
    _cleanUpClipDefs(viewId);
    _svgClipDefs.remove(viewId);
  }

  /// Modify the given rendering by removing canvases until the number of
  /// canvases is less than or equal to the maximum number of canvases.
  Rendering _modifyRenderingForMaxCanvases(Rendering rendering) {
    final Rendering result = Rendering();
    final int numCanvases = rendering.canvases.length;
    if (numCanvases <= maximumCanvases) {
      return rendering;
    }
    int numCanvasesToDelete = numCanvases - maximumCanvases;
    final List<PictureLayer> picturesForLastCanvas = <PictureLayer>[];
    final List<RenderingEntity> modifiedEntities = List<RenderingEntity>.from(rendering.entities);
    bool sawLastCanvas = false;
    for (int i = rendering.entities.length - 1; i >= 0; i--) {
      final RenderingEntity entity = modifiedEntities[i];
      if (entity is RenderingRenderCanvas) {
        if (!sawLastCanvas) {
          sawLastCanvas = true;
          continue;
        }
        modifiedEntities.removeAt(i);
        picturesForLastCanvas.insertAll(0, entity.pictures);
        numCanvasesToDelete--;
        if (numCanvasesToDelete == 0) {
          break;
        }
      }
    }

    // Add all the pictures from the deleted canvases to the second-to-last
    // canvas (or the last canvas if there is only one).
    sawLastCanvas = (maximumCanvases == 1);
    for (int i = modifiedEntities.length - 1; i > 0; i--) {
      final RenderingEntity entity = modifiedEntities[i];
      if (entity is RenderingRenderCanvas) {
        if (sawLastCanvas) {
          entity.pictures.addAll(picturesForLastCanvas);
          break;
        }
        sawLastCanvas = true;
      }
    }

    result.entities.addAll(modifiedEntities);
    return result;
  }

  void _updateDomForNewRendering(Rendering rendering) {
    if (rendering.equalsForRendering(_activeRendering)) {
      // The rendering has not changed, so no DOM manipulation is needed.
      return;
    }
    final List<int> indexMap = _getIndexMapFromPreviousRendering(_activeRendering, rendering);
    final List<int> existingIndexMap = indexMap.where((int index) => index != -1).toList();

    final List<int> staticElements = longestIncreasingSubsequence(existingIndexMap);
    // Convert longest increasing subsequence from subsequence of indices of
    // `existingIndexMap` to a subsequence of indices in previous rendering.
    for (int i = 0; i < staticElements.length; i++) {
      staticElements[i] = existingIndexMap[staticElements[i]];
    }

    // Remove elements which are in the active rendering, but not in the new
    // rendering.
    for (int i = 0; i < _activeRendering.entities.length; i++) {
      if (indexMap.contains(i)) {
        continue;
      }
      final RenderingEntity entity = _activeRendering.entities[i];
      if (entity is RenderingPlatformView) {
        disposeView(entity.viewId);
      } else if (entity is RenderingRenderCanvas) {
        assert(
          entity.displayCanvas != null,
          'RenderCanvas in previous rendering was '
          'not assigned a DisplayCanvas',
        );
        rasterizer.releaseOverlay(entity.displayCanvas!);
        entity.displayCanvas = null;
      }
    }

    // Updates [renderCanvas] (located in [index] in the next rendering) to have
    // a display canvas, either taken from the associated render canvas in the
    // previous rendering, or newly created.
    void updateRenderCanvasWithDisplay(RenderingRenderCanvas renderCanvas, int index) {
      // Does [nextEntity] correspond with a render canvas in the previous
      // rendering? If so, then the render canvas in the previous rendering
      // had an associated display canvas. Use this display canvas for
      // [nextEntity].
      if (indexMap[index] != -1) {
        final RenderingEntity previousEntity = _activeRendering.entities[indexMap[index]];
        assert(previousEntity is RenderingRenderCanvas && previousEntity.displayCanvas != null);
        renderCanvas.displayCanvas = (previousEntity as RenderingRenderCanvas).displayCanvas;
        previousEntity.displayCanvas = null;
      } else {
        // There is no corresponding render canvas in the previous
        // rendering. So this render canvas needs a display canvas.
        renderCanvas.displayCanvas = rasterizer.getOverlay();
      }
    }

    // At this point, the DOM contains the static elements and the elements from
    // the previous rendering which need to move. We iterate over the static
    // elements and insert the elements which come before them into the DOM.
    int staticElementIndex = 0;
    int nextRenderingIndex = 0;
    while (staticElementIndex < staticElements.length) {
      final int staticElementIndexInActiveRendering = staticElements[staticElementIndex];
      final DomElement staticDomElement = _getElement(
        _activeRendering.entities[staticElementIndexInActiveRendering],
      );
      // Go through next rendering elements until we reach the static element.
      while (indexMap[nextRenderingIndex] != staticElementIndexInActiveRendering) {
        final RenderingEntity nextEntity = rendering.entities[nextRenderingIndex];
        if (nextEntity is RenderingRenderCanvas) {
          updateRenderCanvasWithDisplay(nextEntity, nextRenderingIndex);
        }
        sceneHost.insertBefore(_getElement(nextEntity), staticDomElement);
        nextRenderingIndex++;
      }
      if (rendering.entities[nextRenderingIndex] is RenderingRenderCanvas) {
        updateRenderCanvasWithDisplay(
          rendering.entities[nextRenderingIndex] as RenderingRenderCanvas,
          nextRenderingIndex,
        );
      }
      // Also increment the next rendering index because this is the static
      // element.
      nextRenderingIndex++;
      staticElementIndex++;
    }

    // Add the leftover entities.
    while (nextRenderingIndex < rendering.entities.length) {
      final RenderingEntity nextEntity = rendering.entities[nextRenderingIndex];
      if (nextEntity is RenderingRenderCanvas) {
        updateRenderCanvasWithDisplay(nextEntity, nextRenderingIndex);
      }
      sceneHost.append(_getElement(nextEntity));
      nextRenderingIndex++;
    }
  }

  DomElement _getElement(RenderingEntity entity) {
    switch (entity) {
      case RenderingRenderCanvas():
        return entity.displayCanvas!.hostElement;
      case RenderingPlatformView():
        return _viewClipChains[entity.viewId]!.root;
    }
  }

  /// Returns a [List] of ints mapping elements from the [next] rendering to
  /// elements of the [previous] rendering. If there is no matching element in
  /// the previous rendering, then the index map for that element is `-1`.
  List<int> _getIndexMapFromPreviousRendering(Rendering previous, Rendering next) {
    assert(
      !previous.equalsForRendering(next),
      'Should not be in this method if the Renderings are equal',
    );
    final List<int> result = <int>[];
    int index = 0;

    final int maxUnchangedLength = math.min(previous.entities.length, next.entities.length);

    // A canvas in the previous rendering can only be used once in the next
    // rendering. So if it is matched with one in the next rendering, mark it
    // here so it is only matched once.
    final Set<int> alreadyClaimedCanvases = <int>{};

    // Add the unchanged elements from the beginning of the list.
    while (index < maxUnchangedLength &&
        previous.entities[index].equalsForRendering(next.entities[index])) {
      result.add(index);
      if (previous.entities[index] is RenderingRenderCanvas) {
        alreadyClaimedCanvases.add(index);
      }
      index += 1;
    }

    while (index < next.entities.length) {
      bool foundForIndex = false;
      for (int oldIndex = 0; oldIndex < previous.entities.length; oldIndex += 1) {
        if (previous.entities[oldIndex].equalsForRendering(next.entities[index]) &&
            !alreadyClaimedCanvases.contains(oldIndex)) {
          result.add(oldIndex);
          if (previous.entities[oldIndex] is RenderingRenderCanvas) {
            alreadyClaimedCanvases.add(oldIndex);
          }
          foundForIndex = true;
          break;
        }
      }
      if (!foundForIndex) {
        result.add(-1);
      }
      index += 1;
    }

    assert(result.length == next.entities.length);
    return result;
  }

  /// Deletes SVG clip paths, useful for tests.
  void debugCleanupSvgClipPaths() {
    final DomElement? parent = _svgPathDefs?.children.single;
    if (parent != null) {
      for (DomNode? child = parent.lastChild; child != null; child = parent.lastChild) {
        parent.removeChild(child);
      }
    }
    _svgClipDefs.clear();
  }

  static void removeElement(DomElement element) {
    element.remove();
  }

  /// Disposes the state of this view embedder.
  void dispose() {
    _viewClipChains.keys.toList().forEach(disposeView);
    _context = EmbedderFrameContext();
    _currentCompositionParams.clear();
    debugCleanupSvgClipPaths();
    _currentCompositionParams.clear();
    _viewClipChains.clear();
    _viewsToRecomposite.clear();
    _activeCompositionOrder.clear();
    _compositionOrder.clear();
    _activeRendering = Rendering();
  }

  /// Clears the state. Used in tests.
  void debugClear() {
    dispose();
    rasterizer.removeOverlaysFromDom();
  }
}

/// Represents a Clip Chain (for a view).
///
/// Objects of this class contain:
/// * The root view in the stack of mutator elements for the view id.
/// * The slot view in the stack (the actual contents of the platform view).
/// * The number of clipping elements used last time the view was composited.
class ViewClipChain {
  ViewClipChain({required DomElement view}) : _root = view, _slot = view;

  DomElement _root;
  final DomElement _slot;
  int _clipCount = -1;

  DomElement get root => _root;
  DomElement get slot => _slot;
  int get clipCount => _clipCount;

  void updateClipChain({required DomElement root, required int clipCount}) {
    _root = root;
    _clipCount = clipCount;
  }
}

/// The parameters passed to the view embedder.
class EmbeddedViewParams {
  EmbeddedViewParams(this.offset, this.size, MutatorsStack mutators)
    : mutators = MutatorsStack._copy(mutators);

  final ui.Offset offset;
  final ui.Size size;
  final MutatorsStack mutators;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is EmbeddedViewParams &&
        other.offset == offset &&
        other.size == size &&
        other.mutators == mutators;
  }

  @override
  int get hashCode => Object.hash(offset, size, mutators);
}

enum MutatorType { clipRect, clipRRect, clipPath, transform, opacity }

/// Stores mutation information like clipping or transform.
class Mutator {
  const Mutator.clipRect(ui.Rect rect) : this._(MutatorType.clipRect, rect, null, null, null, null);
  const Mutator.clipRRect(ui.RRect rrect)
    : this._(MutatorType.clipRRect, null, rrect, null, null, null);
  const Mutator.clipPath(ui.Path path) : this._(MutatorType.clipPath, null, null, path, null, null);
  const Mutator.transform(Matrix4 matrix)
    : this._(MutatorType.transform, null, null, null, matrix, null);
  const Mutator.opacity(int alpha) : this._(MutatorType.opacity, null, null, null, null, alpha);

  const Mutator._(this.type, this.rect, this.rrect, this.path, this.matrix, this.alpha);

  final MutatorType type;
  final ui.Rect? rect;
  final ui.RRect? rrect;
  final ui.Path? path;
  final Matrix4? matrix;
  final int? alpha;

  bool get isClipType =>
      type == MutatorType.clipRect || type == MutatorType.clipRRect || type == MutatorType.clipPath;

  double get alphaFloat => alpha! / 255.0;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other is! Mutator) {
      return false;
    }

    final Mutator typedOther = other;
    if (type != typedOther.type) {
      return false;
    }

    switch (type) {
      case MutatorType.clipRect:
        return rect == typedOther.rect;
      case MutatorType.clipRRect:
        return rrect == typedOther.rrect;
      case MutatorType.clipPath:
        return path == typedOther.path;
      case MutatorType.transform:
        return matrix == typedOther.matrix;
      case MutatorType.opacity:
        return alpha == typedOther.alpha;
    }
  }

  @override
  int get hashCode => Object.hash(type, rect, rrect, path, matrix, alpha);
}

/// A stack of mutators that can be applied to an embedded view.
class MutatorsStack extends Iterable<Mutator> {
  MutatorsStack() : _mutators = <Mutator>[];

  MutatorsStack._copy(MutatorsStack original) : _mutators = List<Mutator>.from(original._mutators);

  final List<Mutator> _mutators;

  void pushClipRect(ui.Rect rect) {
    _mutators.add(Mutator.clipRect(rect));
  }

  void pushClipRRect(ui.RRect rrect) {
    _mutators.add(Mutator.clipRRect(rrect));
  }

  void pushClipPath(ui.Path path) {
    _mutators.add(Mutator.clipPath(path));
  }

  void pushTransform(Matrix4 matrix) {
    _mutators.add(Mutator.transform(matrix));
  }

  void pushOpacity(int alpha) {
    _mutators.add(Mutator.opacity(alpha));
  }

  void pop() {
    _mutators.removeLast();
  }

  @override
  bool operator ==(Object other) {
    if (identical(other, this)) {
      return true;
    }
    return other is MutatorsStack && listEquals<Mutator>(other._mutators, _mutators);
  }

  @override
  int get hashCode => Object.hashAll(_mutators);

  @override
  Iterator<Mutator> get iterator => _mutators.reversed.iterator;

  /// Iterate over the mutators in reverse.
  Iterable<Mutator> get reversed => _mutators;
}

sealed class SceneElement {}

class PictureSceneElement extends SceneElement {
  PictureSceneElement(this.picture);

  final PictureLayer picture;
}

class PlatformViewSceneElement extends SceneElement {
  PlatformViewSceneElement(this.viewId);

  final int viewId;
}

/// The state for the current frame.
class EmbedderFrameContext {
  /// Picture recorders which were created d the final bounds of the picture in the scene.
  final Map<PictureLayer, CkPictureRecorder> measuringPictureRecorders =
      <PictureLayer, CkPictureRecorder>{};

  /// List of picture recorders and platform view ids in the order they were
  /// painted.
  final List<SceneElement> sceneElements = <SceneElement>[];

  /// The optimized rendering for this frame. This is set by calling
  /// [optimizeRendering].
  Rendering? optimizedRendering;

  /// The picture recorders for the optimized rendering. This is set by calling
  /// [optimizeRendering].
  List<CkPictureRecorder>? optimizedCanvasRecorders;

  /// A map from the original PictureLayer to the picture recorder it should go
  /// into in the optimized rendering. This is set by calling
  /// [optimizedRendering].
  Map<PictureLayer, CkPictureRecorder>? pictureToOptimizedCanvasMap;
}
