// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome || firefox')
library;

import 'dart:async';
import 'dart:js_interop';
import 'dart:js_util' as js_util;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../../common/matchers.dart';
import '../../common/rendering.dart';
import '../../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpAll(() async {
    await bootstrapAndRunApp(withImplicitView: true);
    setUpRenderingForTests();
  });

  group('SceneBuilder', () {
    test('pushOffset implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushOffset(10, 20, oldLayer: oldLayer as ui.OffsetEngineLayer?);
        },
        () {
          return '''<flt-scene><flt-offset></flt-offset></flt-scene>''';
        },
      );
    });

    test('pushTransform implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushTransform(
            (Matrix4.identity()..scale(EngineFlutterDisplay.instance.browserDevicePixelRatio))
                .toFloat64(),
          );
        },
        () {
          return '''<flt-scene><flt-transform></flt-transform></flt-scene>''';
        },
      );
    });

    test('pushClipRect implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushClipRect(
            const ui.Rect.fromLTRB(10, 20, 30, 40),
            oldLayer: oldLayer as ui.ClipRectEngineLayer?,
          );
        },
        () {
          return '''
<flt-scene>
  <flt-clip><flt-clip-interior></flt-clip-interior></flt-clip>
</flt-scene>
''';
        },
      );
    });

    test('pushClipRRect implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushClipRRect(
            ui.RRect.fromLTRBR(10, 20, 30, 40, const ui.Radius.circular(3)),
            oldLayer: oldLayer as ui.ClipRRectEngineLayer?,
            clipBehavior: ui.Clip.none,
          );
        },
        () {
          return '''
<flt-scene>
  <flt-clip clip-type="rrect">
    <flt-clip-interior></flt-clip-interior>
  </flt-clip>
</flt-scene>
''';
        },
      );
    });

    test('pushClipPath implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          final ui.Path path = ui.Path()..addRect(const ui.Rect.fromLTRB(10, 20, 30, 40));
          return sceneBuilder.pushClipPath(path, oldLayer: oldLayer as ui.ClipPathEngineLayer?);
        },
        () {
          return '''
<flt-scene>
  <flt-clippath>
    <svg><defs><clipPath><path></path></clipPath></defs></svg>
  </flt-clippath>
</flt-scene>
''';
        },
      );
    });

    test('pushOpacity implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushOpacity(10, oldLayer: oldLayer as ui.OpacityEngineLayer?);
        },
        () {
          return '''<flt-scene><flt-opacity></flt-opacity></flt-scene>''';
        },
      );
    });
    test('pushBackdropFilter implements surface lifecycle', () {
      testLayerLifeCycle(
        (ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer) {
          return sceneBuilder.pushBackdropFilter(
            ui.ImageFilter.blur(sigmaX: 1.0, sigmaY: 1.0),
            oldLayer: oldLayer as ui.BackdropFilterEngineLayer?,
          );
        },
        () {
          return '''
<flt-scene>
  <flt-backdrop>
    <flt-backdrop-filter></flt-backdrop-filter>
    <flt-backdrop-interior></flt-backdrop-interior>
  </flt-backdrop>
</flt-scene>''';
        },
      );
    });
  });

  group('parent child lifecycle', () {
    test('build, retain, update, and applyPaint are called the right number of times', () {
      final PersistedScene scene1 = PersistedScene(null);
      final PersistedClipRect clip1 = PersistedClipRect(
        null,
        const ui.Rect.fromLTRB(10, 10, 20, 20),
        ui.Clip.antiAlias,
      );
      final PersistedOpacity opacity = PersistedOpacity(null, 100, ui.Offset.zero);
      final MockPersistedPicture picture = MockPersistedPicture();

      scene1.appendChild(clip1);
      clip1.appendChild(opacity);
      opacity.appendChild(picture);

      expect(picture.retainCount, 0);
      expect(picture.buildCount, 0);
      expect(picture.updateCount, 0);
      expect(picture.applyPaintCount, 0);

      scene1.preroll(PrerollSurfaceContext());
      scene1.build();
      commitScene(scene1);
      expect(picture.retainCount, 0);
      expect(picture.buildCount, 1);
      expect(picture.updateCount, 0);
      expect(picture.applyPaintCount, 1);

      // The second scene graph retains the opacity, but not the clip. However,
      // because the clip didn't change no repaints should happen.
      final PersistedScene scene2 = PersistedScene(scene1);
      final PersistedClipRect clip2 = PersistedClipRect(
        clip1,
        const ui.Rect.fromLTRB(10, 10, 20, 20),
        ui.Clip.antiAlias,
      );
      clip1.state = PersistedSurfaceState.pendingUpdate;
      scene2.appendChild(clip2);
      opacity.state = PersistedSurfaceState.pendingRetention;
      clip2.appendChild(opacity);

      scene2.preroll(PrerollSurfaceContext());
      scene2.update(scene1);
      commitScene(scene2);
      expect(picture.retainCount, 1);
      expect(picture.buildCount, 1);
      expect(picture.updateCount, 0);
      expect(picture.applyPaintCount, 1);

      // The third scene graph retains the opacity, and produces a new clip.
      // This should cause the picture to repaint despite being retained.
      final PersistedScene scene3 = PersistedScene(scene2);
      final PersistedClipRect clip3 = PersistedClipRect(
        clip2,
        const ui.Rect.fromLTRB(10, 10, 50, 50),
        ui.Clip.antiAlias,
      );
      clip2.state = PersistedSurfaceState.pendingUpdate;
      scene3.appendChild(clip3);
      opacity.state = PersistedSurfaceState.pendingRetention;
      clip3.appendChild(opacity);

      scene3.preroll(PrerollSurfaceContext());
      scene3.update(scene2);
      commitScene(scene3);
      expect(picture.retainCount, 2);
      expect(picture.buildCount, 1);
      expect(picture.updateCount, 0);
      expect(picture.applyPaintCount, 2);
    });
  });

  group('Compositing order', () {
    // Regression test for https://github.com/flutter/flutter/issues/55058
    //
    // When BitmapCanvas uses multiple elements to paint, the very first
    // canvas needs to have a -1 zIndex so it can preserve compositing order.
    test('Canvas element should retain -1 zIndex after update', () async {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      final ui.Picture picture1 = _drawPicture();
      final ui.ClipRectEngineLayer oldLayer = builder.pushClipRect(
        const ui.Rect.fromLTRB(10, 10, 300, 300),
      );
      builder.addPicture(ui.Offset.zero, picture1);
      builder.pop();

      final DomElement content = builder.build().webOnlyRootElement!;
      expect(content.querySelector('canvas')!.style.zIndex, '-1');

      // Force update to scene which will utilize reuse code path.
      final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
      builder2.pushClipRect(const ui.Rect.fromLTRB(5, 10, 300, 300), oldLayer: oldLayer);
      final ui.Picture picture2 = _drawPicture();
      builder2.addPicture(ui.Offset.zero, picture2);
      builder2.pop();

      final DomElement contentAfterReuse = builder2.build().webOnlyRootElement!;
      expect(contentAfterReuse.querySelector('canvas')!.style.zIndex, '-1');
    });

    test('Multiple canvas elements should retain zIndex after update', () async {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      final ui.Picture picture1 = _drawPathImagePath();
      final ui.ClipRectEngineLayer oldLayer = builder.pushClipRect(
        const ui.Rect.fromLTRB(10, 10, 300, 300),
      );
      builder.addPicture(ui.Offset.zero, picture1);
      builder.pop();

      final DomElement content = builder.build().webOnlyRootElement!;
      domDocument.body!.append(content);
      expect(content.querySelector('canvas')!.style.zIndex, '-1');

      // Force update to scene which will utilize reuse code path.
      final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
      builder2.pushClipRect(const ui.Rect.fromLTRB(5, 10, 300, 300), oldLayer: oldLayer);
      final ui.Picture picture2 = _drawPathImagePath();
      builder2.addPicture(ui.Offset.zero, picture2);
      builder2.pop();

      final DomElement contentAfterReuse = builder2.build().webOnlyRootElement!;
      final List<DomCanvasElement> list =
          contentAfterReuse.querySelectorAll('canvas').cast<DomCanvasElement>().toList();
      expect(list[0].style.zIndex, '-1');
      expect(list[1].style.zIndex, '');
    });
  });

  /// Verify elementCache is passed during update to reuse existing
  /// image elements.
  test('Should retain same image element', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final ui.Picture picture1 = _drawPathImagePath();
    final ui.ClipRectEngineLayer oldLayer = builder.pushClipRect(
      const ui.Rect.fromLTRB(10, 10, 300, 300),
    );
    builder.addPicture(ui.Offset.zero, picture1);
    builder.pop();

    final DomElement content = builder.build().webOnlyRootElement!;
    domDocument.body!.append(content);
    List<DomHTMLImageElement> list =
        content.querySelectorAll('img').cast<DomHTMLImageElement>().toList();
    for (final DomHTMLImageElement image in list) {
      image.alt = 'marked';
    }

    // Force update to scene which will utilize reuse code path.
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    builder2.pushClipRect(const ui.Rect.fromLTRB(5, 10, 300, 300), oldLayer: oldLayer);
    final ui.Picture picture2 = _drawPathImagePath();
    builder2.addPicture(ui.Offset.zero, picture2);
    builder2.pop();

    final DomElement contentAfterReuse = builder2.build().webOnlyRootElement!;
    list = contentAfterReuse.querySelectorAll('img').cast<DomHTMLImageElement>().toList();
    for (final DomHTMLImageElement image in list) {
      expect(image.alt, 'marked');
    }
    expect(list.length, 1);
  });

  PersistedPicture? findPictureSurfaceChild(PersistedContainerSurface parent) {
    PersistedPicture? pictureSurface;
    parent.visitChildren((PersistedSurface child) {
      pictureSurface = child as PersistedPicture;
    });
    return pictureSurface;
  }

  test('skips painting picture when picture fully clipped out', () async {
    final ui.Picture picture = _drawPicture();

    // Picture not clipped out, so we should see a `<flt-canvas>`
    {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      builder.pushOffset(0, 0);
      builder.addPicture(ui.Offset.zero, picture);
      builder.pop();
      final DomElement content = builder.build().webOnlyRootElement!;
      expect(content.querySelectorAll('flt-picture').single.children, isNotEmpty);
    }

    // Picture fully clipped out, so we should not see a `<flt-canvas>`
    {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      builder.pushOffset(0, 0);
      final PersistedContainerSurface clip =
          builder.pushClipRect(const ui.Rect.fromLTRB(1000, 1000, 2000, 2000))
              as PersistedContainerSurface;
      builder.addPicture(ui.Offset.zero, picture);
      builder.pop();
      builder.pop();
      final DomElement content = builder.build().webOnlyRootElement!;
      expect(content.querySelectorAll('flt-picture').single.children, isEmpty);
      expect(findPictureSurfaceChild(clip)!.canvas, isNull);
    }
  });

  test('does not skip painting picture when picture is '
      'inside transform with offset', () async {
    final ui.Picture picture = _drawPicture();
    // Picture should not be clipped out since transform will offset it to 500,500
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    builder.pushOffset(0, 0);
    builder.pushClipRect(const ui.Rect.fromLTRB(0, 0, 1000, 1000)) as PersistedContainerSurface;
    builder.pushTransform((Matrix4.identity()..scale(0.5, 0.5)).toFloat64());
    builder.addPicture(const ui.Offset(1000, 1000), picture);
    builder.pop();
    builder.pop();
    builder.pop();
    final DomElement content = builder.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-picture').single.children, isNotEmpty);
  });

  test('does not skip painting picture when picture is '
      'inside transform', () async {
    final ui.Picture picture = _drawPicture();
    // Picture should not be clipped out since transform will offset it to 500,500
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    builder.pushOffset(0, 0);
    builder.pushClipRect(const ui.Rect.fromLTRB(0, 0, 1000, 1000)) as PersistedContainerSurface;
    builder.pushTransform((Matrix4.identity()..scale(0.5, 0.5)).toFloat64());
    builder.pushOffset(1000, 1000);
    builder.addPicture(ui.Offset.zero, picture);
    builder.pop();
    builder.pop();
    builder.pop();
    final DomElement content = builder.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-picture').single.children, isNotEmpty);
  });

  test('skips painting picture when picture fully clipped out with'
      ' transform and offset', () async {
    final ui.Picture picture = _drawPicture();
    // Picture should be clipped out since transform will offset it to 500,500
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    builder.pushOffset(50, 50);
    builder.pushClipRect(const ui.Rect.fromLTRB(0, 0, 1000, 1000)) as PersistedContainerSurface;
    builder.pushTransform((Matrix4.identity()..scale(2, 2)).toFloat64());
    builder.pushOffset(500, 500);
    builder.addPicture(ui.Offset.zero, picture);
    builder.pop();
    builder.pop();
    builder.pop();
    builder.pop();
    final DomElement content = builder.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-picture').single.children, isEmpty);
  });

  test('releases old canvas when picture is fully clipped out after addRetained', () async {
    final ui.Picture picture = _drawPicture();

    // Frame 1: picture visible
    final SurfaceSceneBuilder builder1 = SurfaceSceneBuilder();
    final PersistedOffset offset1 = builder1.pushOffset(0, 0) as PersistedOffset;
    builder1.addPicture(ui.Offset.zero, picture);
    builder1.pop();
    final DomElement content1 = builder1.build().webOnlyRootElement!;
    expect(content1.querySelectorAll('flt-picture').single.children, isNotEmpty);
    expect(findPictureSurfaceChild(offset1)!.canvas, isNotNull);

    // Frame 2: picture is clipped out after an update
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    final PersistedOffset offset2 =
        builder2.pushOffset(-10000, -10000, oldLayer: offset1) as PersistedOffset;
    builder2.addPicture(ui.Offset.zero, picture);
    builder2.pop();
    final DomElement content = builder2.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-picture').single.children, isEmpty);
    expect(findPictureSurfaceChild(offset2)!.canvas, isNull);
  });

  test('releases old canvas when picture is fully clipped out after addRetained', () async {
    final ui.Picture picture = _drawPicture();

    // Frame 1: picture visible
    final SurfaceSceneBuilder builder1 = SurfaceSceneBuilder();
    final PersistedOffset offset1 = builder1.pushOffset(0, 0) as PersistedOffset;
    final PersistedOffset subOffset1 = builder1.pushOffset(0, 0) as PersistedOffset;
    builder1.addPicture(ui.Offset.zero, picture);
    builder1.pop();
    builder1.pop();
    final DomElement content1 = builder1.build().webOnlyRootElement!;
    expect(content1.querySelectorAll('flt-picture').single.children, isNotEmpty);
    expect(findPictureSurfaceChild(subOffset1)!.canvas, isNotNull);

    // Frame 2: picture is clipped out after addRetained
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    builder2.pushOffset(-10000, -10000, oldLayer: offset1);

    // Even though the child offset is added as retained, the parent
    // is updated with a value that causes the picture to move out of
    // the clipped area. We should see the canvas being released.
    builder2.addRetained(subOffset1);
    builder2.pop();
    final DomElement content = builder2.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-picture').single.children, isEmpty);
    expect(findPictureSurfaceChild(subOffset1)!.canvas, isNull);
  });

  test('auto-pops pushed layers', () async {
    final ui.Picture picture = _drawPicture();
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    builder.pushOffset(0, 0);
    builder.pushOffset(0, 0);
    builder.pushOffset(0, 0);
    builder.pushOffset(0, 0);
    builder.pushOffset(0, 0);
    builder.addPicture(ui.Offset.zero, picture);

    // Intentionally pop fewer layers than we pushed
    builder.pop();
    builder.pop();
    builder.pop();

    // Expect as many layers as we pushed (not popped).
    final DomElement content = builder.build().webOnlyRootElement!;
    expect(content.querySelectorAll('flt-offset'), hasLength(5));
  });

  test('updates child lists efficiently', () async {
    // Pushes a single child that renders one character.
    //
    // If the character is a number, pushes an offset layer. Otherwise, pushes
    // an offset layer. Test cases use this to control how layers are reused.
    // Layers of the same type can be reused even if they are not explicitly
    // updated. Conversely, layers of different types are never reused.
    ui.EngineLayer pushChild(SurfaceSceneBuilder builder, String char, {ui.EngineLayer? oldLayer}) {
      // Numbers use opacity layers, letters use offset layers. This is used to
      // control DOM reuse. Layers of the same type can reuse DOM nodes from other
      // dropped layers.
      final bool useOffset = int.tryParse(char) == null;
      final EnginePictureRecorder recorder = EnginePictureRecorder();
      final RecordingCanvas canvas = recorder.beginRecording(
        const ui.Rect.fromLTRB(0, 0, 400, 400),
      );
      final ui.Paragraph paragraph =
          (ui.ParagraphBuilder(ui.ParagraphStyle())
                ..pushStyle(ui.TextStyle(decoration: ui.TextDecoration.lineThrough))
                ..addText(char))
              .build();
      paragraph.layout(const ui.ParagraphConstraints(width: 1000));
      canvas.drawParagraph(paragraph, ui.Offset.zero);
      final ui.EngineLayer newLayer =
          useOffset
              ? builder.pushOffset(
                0,
                0,
                oldLayer: oldLayer == null ? null : oldLayer as ui.OffsetEngineLayer,
              )
              : builder.pushOpacity(
                100,
                oldLayer: oldLayer == null ? null : oldLayer as ui.OpacityEngineLayer,
              );
      builder.addPicture(ui.Offset.zero, recorder.endRecording());
      builder.pop();
      return newLayer;
    }

    // Maps letters to layers used to render them in the last frame, used to
    // supply `oldLayer` to guarantee update.
    final Map<String, ui.EngineLayer> renderedLayers = <String, ui.EngineLayer>{};

    // Pump an empty scene to reset it, otherwise the first frame will attempt
    // to diff left-overs from a previous test, which results in unpredictable
    // DOM mutations.
    await renderScene(SurfaceSceneBuilder().build());

    // Renders a `string` by breaking it up into individual characters and
    // rendering each character into its own layer.
    Future<void> testCase(
      String string,
      String description, {
      int deletions = 0,
      int additions = 0,
      int moves = 0,
    }) {
      final Set<DomNode> actualDeletions = <DomNode>{};
      final Set<DomNode> actualAdditions = <DomNode>{};

      // Watches DOM mutations and counts deletions and additions to the child
      // list of the `<flt-scene>` element.
      final DomMutationObserver observer = createDomMutationObserver((
        JSArray<JSAny?> mutations,
        _,
      ) {
        for (final DomMutationRecord record in mutations.toDart.cast<DomMutationRecord>()) {
          actualDeletions.addAll(record.removedNodes!);
          actualAdditions.addAll(record.addedNodes!);
        }
      });
      observer.observe(SurfaceSceneBuilder.debugLastFrameScene!.rootElement!, childList: true);

      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      for (int i = 0; i < string.length; i++) {
        final String char = string[i];
        renderedLayers[char] = pushChild(builder, char, oldLayer: renderedLayers[char]);
      }
      final SurfaceScene scene = builder.build();
      final List<DomElement> pTags =
          scene.webOnlyRootElement!.querySelectorAll('flt-paragraph').toList();
      expect(pTags, hasLength(string.length));
      expect(
        scene.webOnlyRootElement!
            .querySelectorAll('flt-paragraph')
            .map((DomElement p) => p.innerText)
            .join(),
        string,
      );
      renderedLayers.removeWhere((String key, ui.EngineLayer value) => !string.contains(key));

      // Inject a zero-duration timer to allow mutation observers to receive notification.
      return Future<void>.delayed(Duration.zero).then((_) {
        observer.disconnect();

        // Nodes that are removed then added are classified as "moves".
        final int actualMoves = actualAdditions.intersection(actualDeletions).length;
        // Compare all at once instead of one by one because when it fails, it's
        // much more useful to see all numbers, not just the one that failed to
        // match.
        expect(
          <String, int>{
            'additions': actualAdditions.length - actualMoves,
            'deletions': actualDeletions.length - actualMoves,
            'moves': actualMoves,
          },
          <String, int>{'additions': additions, 'deletions': deletions, 'moves': moves},
        );
      });
    }

    // Adding
    await testCase('', 'noop');
    await testCase('', 'noop');
    await testCase('be', 'zero-to-many', additions: 2);
    await testCase('bcde', 'insert in the middle', additions: 2);
    await testCase('abcde', 'prepend', additions: 1);
    await testCase('abcdef', 'append', additions: 1);

    // Moving
    await testCase('fbcdea', 'swap at ends', moves: 2);
    await testCase('fecdba', 'swap in the middle', moves: 2);
    await testCase('fedcba', 'swap adjacent in one move', moves: 1);
    await testCase('fedcba', 'non-empty noop');
    await testCase('afedcb', 'shift right by 1', moves: 1);
    await testCase('fedcba', 'shift left by 1', moves: 1);
    await testCase('abcdef', 'reverse', moves: 5);
    await testCase('efabcd', 'shift right by 2', moves: 2);
    await testCase('abcdef', 'shift left by 2', moves: 2);

    // Scrolling without DOM reuse (numbers and letters use different types of layers)
    await testCase('9abcde', 'scroll right by 1', additions: 1, deletions: 1);
    await testCase('789abc', 'scroll right by 2', additions: 2, deletions: 2);
    await testCase('89abcd', 'scroll left by 1', additions: 1, deletions: 1);
    await testCase('abcdef', 'scroll left by 2', additions: 2, deletions: 2);

    // Scrolling with DOM reuse
    await testCase('zabcde', 'scroll right by 1', moves: 1);
    await testCase('xyzabc', 'scroll right by 2', moves: 2);
    await testCase('yzabcd', 'scroll left by 1', moves: 1);
    await testCase('abcdef', 'scroll left by 2', moves: 2);

    // Removing
    await testCase('bcdef', 'remove as start', deletions: 1);
    await testCase('bcde', 'remove as end', deletions: 1);
    await testCase('be', 'remove in the middle', deletions: 2);
    await testCase('', 'remove all', deletions: 2);
  });

  test('Canvas should allocate fewer pixels when zoomed out', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final ui.Picture picture1 = _drawPicture();
    builder.pushClipRect(const ui.Rect.fromLTRB(10, 10, 300, 300));
    builder.addPicture(ui.Offset.zero, picture1);
    builder.pop();

    final DomElement content = builder.build().webOnlyRootElement!;
    final DomCanvasElement canvas = content.querySelector('canvas')! as DomCanvasElement;
    final int unscaledWidth = canvas.width!.toInt();
    final int unscaledHeight = canvas.height!.toInt();

    // Force update to scene which will utilize reuse code path.
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    builder2.pushOffset(0, 0);
    builder2.pushTransform(Matrix4.identity().scaled(0.5, 0.5).toFloat64());
    builder2.pushClipRect(const ui.Rect.fromLTRB(10, 10, 300, 300));
    builder2.addPicture(ui.Offset.zero, picture1);
    builder2.pop();
    builder2.pop();
    builder2.pop();

    final DomElement contentAfterScale = builder2.build().webOnlyRootElement!;
    final DomCanvasElement canvas2 = contentAfterScale.querySelector('canvas')! as DomCanvasElement;
    // Although we are drawing same picture, due to scaling the new canvas
    // should have fewer pixels.
    expect(canvas2.width! < unscaledWidth, isTrue);
    expect(canvas2.height! < unscaledHeight, isTrue);
  });

  test('Canvas should allocate more pixels when zoomed in', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final ui.Picture picture1 = _drawPicture();
    builder.pushClipRect(const ui.Rect.fromLTRB(10, 10, 300, 300));
    builder.addPicture(ui.Offset.zero, picture1);
    builder.pop();

    final DomElement content = builder.build().webOnlyRootElement!;
    final DomCanvasElement canvas = content.querySelector('canvas')! as DomCanvasElement;
    final int unscaledWidth = canvas.width!.toInt();
    final int unscaledHeight = canvas.height!.toInt();

    // Force update to scene which will utilize reuse code path.
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    builder2.pushOffset(0, 0);
    builder2.pushTransform(Matrix4.identity().scaled(2, 2).toFloat64());
    builder2.pushClipRect(const ui.Rect.fromLTRB(10, 10, 300, 300));
    builder2.addPicture(ui.Offset.zero, picture1);
    builder2.pop();
    builder2.pop();
    builder2.pop();

    final DomElement contentAfterScale = builder2.build().webOnlyRootElement!;
    final DomCanvasElement canvas2 = contentAfterScale.querySelector('canvas')! as DomCanvasElement;
    // Although we are drawing same picture, due to scaling the new canvas
    // should have more pixels.
    expect(canvas2.width! > unscaledWidth, isTrue);
    expect(canvas2.height! > unscaledHeight, isTrue);
  });

  test('Should recycle canvas once', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final ui.Picture picture1 = _drawPicture();
    final ui.ClipRectEngineLayer oldLayer = builder.pushClipRect(
      const ui.Rect.fromLTRB(10, 10, 300, 300),
    );
    builder.addPicture(ui.Offset.zero, picture1);
    builder.pop();
    builder.build();

    // Force update to scene which will utilize reuse code path.
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    final ui.ClipRectEngineLayer oldLayer2 = builder2.pushClipRect(
      const ui.Rect.fromLTRB(5, 10, 300, 300),
      oldLayer: oldLayer,
    );
    builder2.addPicture(ui.Offset.zero, _drawEmptyPicture());
    builder2.pop();

    final DomElement contentAfterReuse = builder2.build().webOnlyRootElement!;
    expect(contentAfterReuse, isNotNull);

    final SurfaceSceneBuilder builder3 = SurfaceSceneBuilder();
    builder3.pushClipRect(const ui.Rect.fromLTRB(25, 10, 300, 300), oldLayer: oldLayer2);
    builder3.addPicture(ui.Offset.zero, _drawEmptyPicture());
    builder3.pop();
    // This build will crash if canvas gets recycled twice.
    final DomElement contentAfterReuse2 = builder3.build().webOnlyRootElement!;
    expect(contentAfterReuse2, isNotNull);
  });
}

typedef TestLayerBuilder =
    ui.EngineLayer Function(ui.SceneBuilder sceneBuilder, ui.EngineLayer? oldLayer);
typedef ExpectedHtmlGetter = String Function();

void testLayerLifeCycle(TestLayerBuilder layerBuilder, ExpectedHtmlGetter expectedHtmlGetter) {
  // Force scene builder to start from scratch. This guarantees that the first
  // scene starts from the "build" phase.
  SurfaceSceneBuilder.debugForgetFrameScene();

  // Build: builds a brand new layer.
  SurfaceSceneBuilder sceneBuilder = SurfaceSceneBuilder();
  final ui.EngineLayer layer1 = layerBuilder(sceneBuilder, null);
  final Type surfaceType = layer1.runtimeType;
  sceneBuilder.pop();

  SceneTester tester = SceneTester(sceneBuilder.build());
  tester.expectSceneHtml(expectedHtmlGetter());

  PersistedSurface findSurface() {
    return enumerateSurfaces().where((PersistedSurface s) => s.runtimeType == surfaceType).single;
  }

  final PersistedSurface surface1 = findSurface();
  final DomElement surfaceElement1 = surface1.rootElement!;

  // Retain: reuses a layer as is along with its DOM elements.
  sceneBuilder = SurfaceSceneBuilder();
  sceneBuilder.addRetained(layer1);

  tester = SceneTester(sceneBuilder.build());
  tester.expectSceneHtml(expectedHtmlGetter());

  final PersistedSurface surface2 = findSurface();
  final DomElement surfaceElement2 = surface2.rootElement!;

  expect(surface2, same(surface1));
  expect(surfaceElement2, same(surfaceElement1));

  // Reuse: reuses a layer's DOM elements by matching it.
  sceneBuilder = SurfaceSceneBuilder();
  final ui.EngineLayer layer3 = layerBuilder(sceneBuilder, layer1);
  sceneBuilder.pop();
  expect(layer3, isNot(same(layer1)));
  tester = SceneTester(sceneBuilder.build());
  tester.expectSceneHtml(expectedHtmlGetter());

  final PersistedSurface surface3 = findSurface();
  expect(surface3, same(layer3));
  final DomElement surfaceElement3 = surface3.rootElement!;
  expect(surface3, isNot(same(surface2)));
  expect(surfaceElement3, isNotNull);
  expect(surfaceElement3, same(surfaceElement2));

  // Recycle: discards all the layers.
  sceneBuilder = SurfaceSceneBuilder();
  tester = SceneTester(sceneBuilder.build());
  tester.expectSceneHtml('<flt-scene></flt-scene>');

  expect(surface3.rootElement, isNull); // offset3 should be recycled.

  // Retain again: the framework should be able to request that a layer is added
  //               as retained even after it has been recycled. In this case the
  //               engine would "rehydrate" the layer with new DOM elements.
  sceneBuilder = SurfaceSceneBuilder();
  sceneBuilder.addRetained(layer3);
  tester = SceneTester(sceneBuilder.build());
  tester.expectSceneHtml(expectedHtmlGetter());
  expect(surface3.rootElement, isNotNull); // offset3 should be rehydrated.

  // Make sure we clear retained surface list.
  expect(retainedSurfaces, isEmpty);
}

class MockPersistedPicture extends PersistedPicture {
  factory MockPersistedPicture() {
    final EnginePictureRecorder recorder = EnginePictureRecorder();
    // Use the largest cull rect so that layer clips are effective. The tests
    // rely on this.
    recorder.beginRecording(ui.Rect.largest).drawPaint(SurfacePaint());
    return MockPersistedPicture._(recorder.endRecording());
  }

  MockPersistedPicture._(EnginePicture picture) : super(0, 0, picture, 0);

  int retainCount = 0;
  int buildCount = 0;
  int updateCount = 0;
  int applyPaintCount = 0;

  final BitmapCanvas _fakeCanvas = BitmapCanvas(
    const ui.Rect.fromLTRB(0, 0, 10, 10),
    RenderStrategy(),
  );

  @override
  EngineCanvas get canvas {
    return _fakeCanvas;
  }

  @override
  double matchForUpdate(PersistedPicture existingSurface) {
    return identical(existingSurface.picture, picture) ? 0.0 : 1.0;
  }

  @override
  Matrix4 get localTransformInverse => Matrix4.identity();

  @override
  void build() {
    super.build();
    buildCount++;
  }

  @override
  void retain() {
    super.retain();
    retainCount++;
  }

  @override
  void applyPaint(EngineCanvas? oldCanvas) {
    applyPaintCount++;
  }

  @override
  void update(PersistedPicture oldSurface) {
    super.update(oldSurface);
    updateCount++;
  }

  @override
  int get bitmapPixelCount => 0;
}

/// Draw 4 circles within 50, 50, 120, 120 bounds
ui.Picture _drawPicture() {
  const double offsetX = 50;
  const double offsetY = 50;
  final EnginePictureRecorder recorder = EnginePictureRecorder();
  final RecordingCanvas canvas = recorder.beginRecording(const ui.Rect.fromLTRB(0, 0, 400, 400));
  final ui.Shader gradient = ui.Gradient.radial(const ui.Offset(100, 100), 50, const <ui.Color>[
    ui.Color.fromARGB(255, 0, 0, 0),
    ui.Color.fromARGB(255, 0, 0, 255),
  ]);
  canvas.drawCircle(
    const ui.Offset(offsetX + 10, offsetY + 10),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..shader = gradient,
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 60, offsetY + 10),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(255, 0, 0, 1),
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 10, offsetY + 60),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(0, 255, 0, 1),
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 60, offsetY + 60),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(0, 0, 255, 1),
  );
  return recorder.endRecording();
}

EnginePicture _drawEmptyPicture() {
  final EnginePictureRecorder recorder = EnginePictureRecorder();
  recorder.beginRecording(const ui.Rect.fromLTRB(0, 0, 400, 400));
  return recorder.endRecording();
}

EnginePicture _drawPathImagePath() {
  const double offsetX = 50;
  const double offsetY = 50;
  final EnginePictureRecorder recorder = EnginePictureRecorder();
  final RecordingCanvas canvas = recorder.beginRecording(const ui.Rect.fromLTRB(0, 0, 400, 400));
  final ui.Shader gradient = ui.Gradient.radial(const ui.Offset(100, 100), 50, const <ui.Color>[
    ui.Color.fromARGB(255, 0, 0, 0),
    ui.Color.fromARGB(255, 0, 0, 255),
  ]);
  canvas.drawCircle(
    const ui.Offset(offsetX + 10, offsetY + 10),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..shader = gradient,
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 60, offsetY + 10),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(255, 0, 0, 1),
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 10, offsetY + 60),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(0, 255, 0, 1),
  );
  canvas.drawImage(createTestImage(), ui.Offset.zero, SurfacePaint());
  canvas.drawCircle(
    const ui.Offset(offsetX + 10, offsetY + 10),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..shader = gradient,
  );
  canvas.drawCircle(
    const ui.Offset(offsetX + 60, offsetY + 60),
    10,
    SurfacePaint()
      ..style = ui.PaintingStyle.fill
      ..color = const ui.Color.fromRGBO(0, 0, 255, 1),
  );
  return recorder.endRecording();
}

HtmlImage createTestImage({int width = 100, int height = 50}) {
  final DomCanvasElement canvas = createDomCanvasElement(width: width, height: height);
  final DomCanvasRenderingContext2D ctx = canvas.context2D;
  ctx.fillStyle = '#E04040';
  ctx.fillRect(0, 0, 33, 50);
  ctx.fill();
  ctx.fillStyle = '#40E080';
  ctx.fillRect(33, 0, 33, 50);
  ctx.fill();
  ctx.fillStyle = '#2040E0';
  ctx.fillRect(66, 0, 33, 50);
  ctx.fill();
  final DomHTMLImageElement imageElement = createDomHTMLImageElement();
  imageElement.src = js_util.callMethod<String>(canvas, 'toDataURL', <dynamic>[]);
  return HtmlImage(imageElement, width, height);
}

class SceneTester {
  SceneTester(this.scene);

  final SurfaceScene scene;

  void expectSceneHtml(String expectedHtml) {
    expect(scene.webOnlyRootElement, hasHtml(expectedHtml));
  }
}
