// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:ui/src/engine/util.dart';
import 'package:ui/ui.dart' as ui;

import '../../engine.dart' show PictureLayer, PlatformViewManager;
import '../vector_math.dart';
import 'embedded_views.dart';
import 'rasterizer.dart';

/// If `true`, draws the computed bounds for platform views and pictures to
/// help debug issues with the overlay optimization.
bool debugOverlayOptimizationBounds = false;

/// A [Rendering] is a concrete description of how a Flutter scene will be
/// rendered in a web browser.
///
/// A [Rendering] is a sequence containing two types of entities:
///   * Render canvases: which contain rasterized CkPictures, and
///   * Platform views: being HTML content that is to be composited along with
///     the Flutter content.
class Rendering {
  final List<RenderingEntity> entities = <RenderingEntity>[];

  void add(RenderingEntity entity) {
    entities.add(entity);
  }

  /// Returns [true] if this is equibalent to [other] for use in rendering.
  bool equalsForRendering(Rendering other) {
    if (other.entities.length != entities.length) {
      return false;
    }
    for (int i = 0; i < entities.length; i++) {
      if (!entities[i].equalsForRendering(other.entities[i])) {
        return false;
      }
    }
    return true;
  }

  /// A list of just the canvases in the rendering.
  List<RenderingRenderCanvas> get canvases => entities.whereType<RenderingRenderCanvas>().toList();

  @override
  String toString() => entities.toString();
}

/// An element of a [Rendering]. Either a render canvas or a platform view.
sealed class RenderingEntity {
  /// Returns [true] if this entity is equal to [other] for use in a rendering.
  ///
  /// For example, all [RenderingRenderCanvas] objects are equal to each other
  /// for purposes of rendering since any canvas in that place in the rendering
  /// will be equivalent. Platform views are only equal if they are for the same
  /// view id.
  bool equalsForRendering(RenderingEntity other);
}

class RenderingRenderCanvas extends RenderingEntity {
  RenderingRenderCanvas();

  /// The [pictures] which should be rendered in this canvas.
  final List<PictureLayer> pictures = <PictureLayer>[];

  /// The [DisplayCanvas] that will be used to display [pictures].
  ///
  /// This is set by the view embedder.
  DisplayCanvas? displayCanvas;

  /// Adds the [picture] to the pictures that should be rendered in this canvas.
  void add(PictureLayer picture) {
    pictures.add(picture);
  }

  @override
  bool equalsForRendering(RenderingEntity other) {
    return other is RenderingRenderCanvas;
  }

  @override
  String toString() {
    return '$RenderingRenderCanvas(${pictures.length} pictures)';
  }
}

/// A platform view to be rendered.
class RenderingPlatformView extends RenderingEntity {
  RenderingPlatformView(this.viewId);

  /// The [viewId] of the platform view to render.
  final int viewId;

  @override
  bool equalsForRendering(RenderingEntity other) {
    return other is RenderingPlatformView && other.viewId == viewId;
  }

  @override
  String toString() {
    return '$RenderingPlatformView($viewId)';
  }

  /// The bounds that were computed for this platform view when creating the
  /// optimized rendering. This is only set in debug mode.
  ui.Rect? debugComputedBounds;
}

// Computes the bounds of the platform view from its associated parameters.
@visibleForTesting
ui.Rect computePlatformViewBounds(EmbeddedViewParams params) {
  ui.Rect currentClipBounds = ui.Rect.largest;

  Matrix4 currentTransform = Matrix4.identity();
  for (final Mutator mutator in params.mutators.reversed) {
    switch (mutator.type) {
      case MutatorType.clipRect:
        final ui.Rect transformedClipBounds = transformRectWithMatrix(
          currentTransform,
          mutator.rect!,
        );
        currentClipBounds = currentClipBounds.intersect(transformedClipBounds);
      case MutatorType.clipRRect:
        final ui.Rect transformedClipBounds = transformRectWithMatrix(
          currentTransform,
          mutator.rrect!.outerRect,
        );
        currentClipBounds = currentClipBounds.intersect(transformedClipBounds);
      case MutatorType.clipPath:
        final ui.Rect transformedClipBounds = transformRectWithMatrix(
          currentTransform,
          mutator.path!.getBounds(),
        );
        currentClipBounds.intersect(transformedClipBounds);
      case MutatorType.transform:
        currentTransform = currentTransform.multiplied(mutator.matrix!);
      case MutatorType.opacity:
        // Doesn't effect bounds.
        continue;
    }
  }

  // The width and height are in physical pixels already, so apply the inverse
  // scale since the transform already applied the scaling.
  final ui.Rect rawBounds = ui.Rect.fromLTWH(
    params.offset.dx,
    params.offset.dy,
    params.size.width,
    params.size.height,
  );
  final ui.Rect transformedBounds = transformRectWithMatrix(currentTransform, rawBounds);
  return transformedBounds.intersect(currentClipBounds);
}

/// Returns the optimized [Rendering] for a sequence of [pictures] and
/// [platformViews].
///
/// [paramsForViews] is required to compute the bounds of the platform views.
Rendering createOptimizedRendering(
  Iterable<SceneElement> renderObjects,
  Map<int, EmbeddedViewParams> paramsForViews,
) {
  final Map<int, ui.Rect> cachedComputedRects = <int, ui.Rect>{};

  final Rendering result = Rendering();

  // The first picture is added to the rendering in a new render canvas.
  RenderingRenderCanvas tentativeCanvas = RenderingRenderCanvas();

  for (final SceneElement renderObject in renderObjects) {
    if (renderObject is PlatformViewSceneElement) {
      final int viewId = renderObject.viewId;
      final RenderingPlatformView platformView = RenderingPlatformView(viewId);
      if (PlatformViewManager.instance.isVisible(viewId)) {
        final ui.Rect platformViewBounds =
            cachedComputedRects[viewId] = computePlatformViewBounds(paramsForViews[viewId]!);

        if (debugOverlayOptimizationBounds) {
          platformView.debugComputedBounds = platformViewBounds;
        }

        // If the platform view intersects with any pictures in the tentative canvas
        // then add the tentative canvas to the rendering.
        for (final PictureLayer picture in tentativeCanvas.pictures) {
          if (!picture.sceneBounds!.intersect(platformViewBounds).isEmpty) {
            result.add(tentativeCanvas);
            tentativeCanvas = RenderingRenderCanvas();
            break;
          }
        }
      }
      result.add(platformView);
    } else if (renderObject is PictureSceneElement) {
      final PictureLayer picture = renderObject.picture;
      if (picture.isCulled) {
        continue;
      }

      // Find the first render canvas which comes after the last entity (picture
      // or platform view) that the next picture intersects with, and add the
      // picture to that render canvas, or create a new render canvas.

      // First check if the picture intersects with any pictures in the
      // tentative  canvas, as this will be the last canvas in the rendering
      // when it is eventually added.
      bool addedToTentativeCanvas = false;
      for (final PictureLayer canvasPicture in tentativeCanvas.pictures) {
        if (!canvasPicture.sceneBounds!.intersect(picture.sceneBounds!).isEmpty) {
          tentativeCanvas.add(picture);
          addedToTentativeCanvas = true;
          break;
        }
      }
      if (addedToTentativeCanvas) {
        continue;
      }

      RenderingRenderCanvas? lastCanvasSeen;
      bool addedPictureToRendering = false;
      for (final RenderingEntity entity in result.entities.reversed) {
        if (entity is RenderingPlatformView) {
          if (PlatformViewManager.instance.isVisible(entity.viewId)) {
            final ui.Rect platformViewBounds = cachedComputedRects[entity.viewId]!;
            if (!platformViewBounds.intersect(picture.sceneBounds!).isEmpty) {
              // The next picture intersects with a platform view already in the
              // result. Add this picture to the first render canvas which comes
              // after this platform view or create one if none exists.
              if (lastCanvasSeen != null) {
                lastCanvasSeen.add(picture);
              } else {
                tentativeCanvas.add(picture);
              }
              addedPictureToRendering = true;
              break;
            }
          }
        } else if (entity is RenderingRenderCanvas) {
          lastCanvasSeen = entity;
          // Check if we intersect with any pictures in this render canvas.
          for (final PictureLayer canvasPicture in entity.pictures) {
            if (!canvasPicture.sceneBounds!.intersect(picture.sceneBounds!).isEmpty) {
              lastCanvasSeen.add(picture);
              addedPictureToRendering = true;
              break;
            }
          }
        }
      }
      if (!addedPictureToRendering) {
        if (lastCanvasSeen != null) {
          // Add it to the last canvas seen in the rendering, if any.
          lastCanvasSeen.add(picture);
        } else {
          tentativeCanvas.add(picture);
        }
      }
    }
  }

  if (tentativeCanvas.pictures.isNotEmpty) {
    result.add(tentativeCanvas);
  }

  return result;
}
