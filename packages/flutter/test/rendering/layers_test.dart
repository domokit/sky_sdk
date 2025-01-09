// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('non-painted layers are detached', () {
    RenderObject boundary, inner;
    final RenderOpacity root = RenderOpacity(
      child:
          boundary = RenderRepaintBoundary(
            child: inner = RenderDecoratedBox(decoration: const BoxDecoration()),
          ),
    );
    layout(root, phase: EnginePhase.paint);
    expect(inner.isRepaintBoundary, isFalse);
    expect(inner.debugLayer, null);
    expect(boundary.isRepaintBoundary, isTrue);
    expect(boundary.debugLayer, isNotNull);
    expect(boundary.debugLayer!.attached, isTrue); // this time it painted...

    root.opacity = 0.0;
    pumpFrame(phase: EnginePhase.paint);
    expect(inner.isRepaintBoundary, isFalse);
    expect(inner.debugLayer, null);
    expect(boundary.isRepaintBoundary, isTrue);
    expect(boundary.debugLayer, isNotNull);
    expect(boundary.debugLayer!.attached, isFalse); // this time it did not.

    root.opacity = 0.5;
    pumpFrame(phase: EnginePhase.paint);
    expect(inner.isRepaintBoundary, isFalse);
    expect(inner.debugLayer, null);
    expect(boundary.isRepaintBoundary, isTrue);
    expect(boundary.debugLayer, isNotNull);
    expect(boundary.debugLayer!.attached, isTrue); // this time it did again!
  });

  test('updateSubtreeNeedsAddToScene propagates Layer.alwaysNeedsAddToScene up the tree', () {
    final ContainerLayer a = ContainerLayer();
    final ContainerLayer b = ContainerLayer();
    final ContainerLayer c = ContainerLayer();
    final _TestAlwaysNeedsAddToSceneLayer d = _TestAlwaysNeedsAddToSceneLayer();
    final ContainerLayer e = ContainerLayer();
    final ContainerLayer f = ContainerLayer();

    // Tree structure:
    //        a
    //       / \
    //      b   c
    //     / \
    // (x)d   e
    //   /
    //  f
    a.append(b);
    a.append(c);
    b.append(d);
    b.append(e);
    d.append(f);

    a.debugMarkClean();
    b.debugMarkClean();
    c.debugMarkClean();
    d.debugMarkClean();
    e.debugMarkClean();
    f.debugMarkClean();

    expect(a.debugSubtreeNeedsAddToScene, false);
    expect(b.debugSubtreeNeedsAddToScene, false);
    expect(c.debugSubtreeNeedsAddToScene, false);
    expect(d.debugSubtreeNeedsAddToScene, false);
    expect(e.debugSubtreeNeedsAddToScene, false);
    expect(f.debugSubtreeNeedsAddToScene, false);

    a.updateSubtreeNeedsAddToScene();

    expect(a.debugSubtreeNeedsAddToScene, true);
    expect(b.debugSubtreeNeedsAddToScene, true);
    expect(c.debugSubtreeNeedsAddToScene, false);
    expect(d.debugSubtreeNeedsAddToScene, true);
    expect(e.debugSubtreeNeedsAddToScene, false);
    expect(f.debugSubtreeNeedsAddToScene, false);
  });

  test('updateSubtreeNeedsAddToScene propagates Layer._needsAddToScene up the tree', () {
    final ContainerLayer a = ContainerLayer();
    final ContainerLayer b = ContainerLayer();
    final ContainerLayer c = ContainerLayer();
    final ContainerLayer d = ContainerLayer();
    final ContainerLayer e = ContainerLayer();
    final ContainerLayer f = ContainerLayer();
    final ContainerLayer g = ContainerLayer();
    final List<ContainerLayer> allLayers = <ContainerLayer>[a, b, c, d, e, f, g];

    // The tree is like the following where b and j are dirty:
    //        a____
    //       /     \
    //   (x)b___    c
    //     / \  \   |
    //    d   e  f  g(x)
    a.append(b);
    a.append(c);
    b.append(d);
    b.append(e);
    b.append(f);
    c.append(g);

    for (final ContainerLayer layer in allLayers) {
      expect(layer.debugSubtreeNeedsAddToScene, true);
    }

    for (final ContainerLayer layer in allLayers) {
      layer.debugMarkClean();
    }

    for (final ContainerLayer layer in allLayers) {
      expect(layer.debugSubtreeNeedsAddToScene, false);
    }

    b.markNeedsAddToScene();
    a.updateSubtreeNeedsAddToScene();

    expect(a.debugSubtreeNeedsAddToScene, true);
    expect(b.debugSubtreeNeedsAddToScene, true);
    expect(c.debugSubtreeNeedsAddToScene, false);
    expect(d.debugSubtreeNeedsAddToScene, false);
    expect(e.debugSubtreeNeedsAddToScene, false);
    expect(f.debugSubtreeNeedsAddToScene, false);
    expect(g.debugSubtreeNeedsAddToScene, false);

    g.markNeedsAddToScene();
    a.updateSubtreeNeedsAddToScene();

    expect(a.debugSubtreeNeedsAddToScene, true);
    expect(b.debugSubtreeNeedsAddToScene, true);
    expect(c.debugSubtreeNeedsAddToScene, true);
    expect(d.debugSubtreeNeedsAddToScene, false);
    expect(e.debugSubtreeNeedsAddToScene, false);
    expect(f.debugSubtreeNeedsAddToScene, false);
    expect(g.debugSubtreeNeedsAddToScene, true);

    a.buildScene(SceneBuilder());
    for (final ContainerLayer layer in allLayers) {
      expect(layer.debugSubtreeNeedsAddToScene, false);
    }
  });

  test('follower layers are always dirty', () {
    final LayerLink link = LayerLink();
    final LeaderLayer leaderLayer = LeaderLayer(link: link);
    final FollowerLayer followerLayer = FollowerLayer(link: link);
    leaderLayer.debugMarkClean();
    followerLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    followerLayer.updateSubtreeNeedsAddToScene();
    expect(followerLayer.debugSubtreeNeedsAddToScene, true);
  });

  test('switching layer link of an attached leader layer should not crash', () {
    final LayerLink link = LayerLink();
    final LeaderLayer leaderLayer = LeaderLayer(link: link);
    final FlutterView flutterView = RendererBinding.instance.platformDispatcher.views.single;
    final RenderView view = RenderView(
      configuration: ViewConfiguration.fromView(flutterView),
      view: flutterView,
    );
    leaderLayer.attach(view);
    final LayerLink link2 = LayerLink();
    leaderLayer.link = link2;
    // This should not crash.
    leaderLayer.detach();
    expect(leaderLayer.link, link2);
  });

  test('layer link attach/detach order should not crash app.', () {
    final LayerLink link = LayerLink();
    final LeaderLayer leaderLayer1 = LeaderLayer(link: link);
    final LeaderLayer leaderLayer2 = LeaderLayer(link: link);
    final FlutterView flutterView = RendererBinding.instance.platformDispatcher.views.single;
    final RenderView view = RenderView(
      configuration: ViewConfiguration.fromView(flutterView),
      view: flutterView,
    );
    leaderLayer1.attach(view);
    leaderLayer2.attach(view);
    leaderLayer2.detach();
    leaderLayer1.detach();
    expect(link.leader, isNull);
  });

  test('leader layers not dirty when connected to follower layer', () {
    final ContainerLayer root = ContainerLayer()..attach(Object());

    final LayerLink link = LayerLink();
    final LeaderLayer leaderLayer = LeaderLayer(link: link);
    final FollowerLayer followerLayer = FollowerLayer(link: link);

    root.append(leaderLayer);
    root.append(followerLayer);

    leaderLayer.debugMarkClean();
    followerLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    followerLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);
  });

  test('leader layers are not dirty when all followers disconnects', () {
    final ContainerLayer root = ContainerLayer()..attach(Object());
    final LayerLink link = LayerLink();
    final LeaderLayer leaderLayer = LeaderLayer(link: link);
    root.append(leaderLayer);

    // Does not need add to scene when nothing is connected to link.
    leaderLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);

    // Connecting a follower does not require adding to scene
    final FollowerLayer follower1 = FollowerLayer(link: link);
    root.append(follower1);
    leaderLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);

    final FollowerLayer follower2 = FollowerLayer(link: link);
    root.append(follower2);
    leaderLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);

    // Disconnecting one follower, still does not needs add to scene.
    follower2.remove();
    leaderLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);

    // Disconnecting all followers goes back to not requiring add to scene.
    follower1.remove();
    leaderLayer.debugMarkClean();
    leaderLayer.updateSubtreeNeedsAddToScene();
    expect(leaderLayer.debugSubtreeNeedsAddToScene, false);
  });

  test('LeaderLayer.applyTransform can be called after retained rendering', () {
    void expectTransform(RenderObject leader) {
      final LeaderLayer leaderLayer = leader.debugLayer! as LeaderLayer;
      final Matrix4 expected =
          Matrix4.identity()..translate(leaderLayer.offset.dx, leaderLayer.offset.dy);
      final Matrix4 transformed = Matrix4.identity();
      leaderLayer.applyTransform(null, transformed);
      expect(transformed, expected);
    }

    final LayerLink link = LayerLink();
    late RenderLeaderLayer leader;
    final RenderRepaintBoundary root = RenderRepaintBoundary(
      child: RenderRepaintBoundary(child: leader = RenderLeaderLayer(link: link)),
    );
    layout(root, phase: EnginePhase.composite);

    expectTransform(leader);

    // Causes a repaint, but the LeaderLayer of RenderLeaderLayer will be added
    // as retained and LeaderLayer.addChildrenToScene will not be called.
    root.markNeedsPaint();
    pumpFrame(phase: EnginePhase.composite);

    // The LeaderLayer.applyTransform call shouldn't crash.
    expectTransform(leader);
  });

  test('depthFirstIterateChildren', () {
    final ContainerLayer a = ContainerLayer();
    final ContainerLayer b = ContainerLayer();
    final ContainerLayer c = ContainerLayer();
    final ContainerLayer d = ContainerLayer();
    final ContainerLayer e = ContainerLayer();
    final ContainerLayer f = ContainerLayer();
    final ContainerLayer g = ContainerLayer();

    final PictureLayer h = PictureLayer(Rect.zero);
    final PictureLayer i = PictureLayer(Rect.zero);
    final PictureLayer j = PictureLayer(Rect.zero);

    // The tree is like the following:
    //        a____
    //       /     \
    //      b___    c
    //     / \  \   |
    //    d   e  f  g
    //   / \        |
    //  h   i       j
    a.append(b);
    a.append(c);
    b.append(d);
    b.append(e);
    b.append(f);
    d.append(h);
    d.append(i);
    c.append(g);
    g.append(j);

    expect(a.depthFirstIterateChildren(), <Layer>[b, d, h, i, e, f, c, g, j]);

    d.remove();
    //        a____
    //       /     \
    //      b___    c
    //       \  \   |
    //        e  f  g
    //              |
    //              j
    expect(a.depthFirstIterateChildren(), <Layer>[b, e, f, c, g, j]);
  });

  void checkNeedsAddToScene(Layer layer, void Function() mutateCallback) {
    layer.debugMarkClean();
    layer.updateSubtreeNeedsAddToScene();
    expect(layer.debugSubtreeNeedsAddToScene, false);
    mutateCallback();
    layer.updateSubtreeNeedsAddToScene();
    expect(layer.debugSubtreeNeedsAddToScene, true);
  }

  List<String> getDebugInfo(Layer layer) {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    layer.debugFillProperties(builder);
    return builder.properties
        .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
        .map((DiagnosticsNode node) => node.toString())
        .toList();
  }

  test('ClipRectLayer prints clipBehavior in debug info', () {
    expect(getDebugInfo(ClipRectLayer()), contains('clipBehavior: Clip.hardEdge'));
    expect(
      getDebugInfo(ClipRectLayer(clipBehavior: Clip.antiAliasWithSaveLayer)),
      contains('clipBehavior: Clip.antiAliasWithSaveLayer'),
    );
  });

  test('ClipRRectLayer prints clipBehavior in debug info', () {
    expect(getDebugInfo(ClipRRectLayer()), contains('clipBehavior: Clip.antiAlias'));
    expect(
      getDebugInfo(ClipRRectLayer(clipBehavior: Clip.antiAliasWithSaveLayer)),
      contains('clipBehavior: Clip.antiAliasWithSaveLayer'),
    );
  });

  test('ClipPathLayer prints clipBehavior in debug info', () {
    expect(getDebugInfo(ClipPathLayer()), contains('clipBehavior: Clip.antiAlias'));
    expect(
      getDebugInfo(ClipPathLayer(clipBehavior: Clip.antiAliasWithSaveLayer)),
      contains('clipBehavior: Clip.antiAliasWithSaveLayer'),
    );
  });

  test('BackdropFilterLayer prints filter and blendMode in debug info', () {
    final ImageFilter filter = ImageFilter.blur(
      sigmaX: 1.0,
      sigmaY: 1.0,
      tileMode: TileMode.repeated,
    );
    final BackdropFilterLayer layer = BackdropFilterLayer(
      filter: filter,
      blendMode: BlendMode.clear,
    );
    final List<String> info = getDebugInfo(layer);

    expect(info, contains('filter: ImageFilter.blur(${1.0}, ${1.0}, repeated)'));
    expect(info, contains('blendMode: clear'));
  });

  test('PictureLayer prints picture, raster cache hints in debug info', () {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint());
    final Picture picture = recorder.endRecording();
    final PictureLayer layer = PictureLayer(const Rect.fromLTRB(0, 0, 1, 1));
    layer.picture = picture;
    layer.isComplexHint = true;
    layer.willChangeHint = false;
    final List<String> info = getDebugInfo(layer);
    expect(info, contains('picture: ${describeIdentity(picture)}'));
    expect(info, isNot(contains('engine layer: ${describeIdentity(null)}')));
    expect(info, contains('raster cache hints: isComplex = true, willChange = false'));
  });

  test('Layer prints engineLayer if it is not null in debug info', () {
    final ConcreteLayer layer = ConcreteLayer();
    List<String> info = getDebugInfo(layer);
    expect(info, isNot(contains('engine layer: ${describeIdentity(null)}')));

    layer.engineLayer = FakeEngineLayer();
    info = getDebugInfo(layer);
    expect(info, contains('engine layer: ${describeIdentity(layer.engineLayer)}'));
  });

  test('mutating PictureLayer fields triggers needsAddToScene', () {
    final PictureLayer pictureLayer = PictureLayer(Rect.zero);
    checkNeedsAddToScene(pictureLayer, () {
      final PictureRecorder recorder = PictureRecorder();
      Canvas(recorder);
      pictureLayer.picture = recorder.endRecording();
    });

    pictureLayer.isComplexHint = false;
    checkNeedsAddToScene(pictureLayer, () {
      pictureLayer.isComplexHint = true;
    });

    pictureLayer.willChangeHint = false;
    checkNeedsAddToScene(pictureLayer, () {
      pictureLayer.willChangeHint = true;
    });
  });

  const Rect unitRect = Rect.fromLTRB(0, 0, 1, 1);

  test('mutating PerformanceOverlayLayer fields triggers needsAddToScene', () {
    final PerformanceOverlayLayer layer = PerformanceOverlayLayer(
      overlayRect: Rect.zero,
      optionsMask: 0,
    );
    checkNeedsAddToScene(layer, () {
      layer.overlayRect = unitRect;
    });
  });

  test('mutating OffsetLayer fields triggers needsAddToScene', () {
    final OffsetLayer layer = OffsetLayer();
    checkNeedsAddToScene(layer, () {
      layer.offset = const Offset(1, 1);
    });
  });

  test('mutating ClipRectLayer fields triggers needsAddToScene', () {
    final ClipRectLayer layer = ClipRectLayer(clipRect: Rect.zero);
    checkNeedsAddToScene(layer, () {
      layer.clipRect = unitRect;
    });
    checkNeedsAddToScene(layer, () {
      layer.clipBehavior = Clip.antiAliasWithSaveLayer;
    });
  });

  test('mutating ClipRRectLayer fields triggers needsAddToScene', () {
    final ClipRRectLayer layer = ClipRRectLayer(clipRRect: RRect.zero);
    checkNeedsAddToScene(layer, () {
      layer.clipRRect = RRect.fromRectAndRadius(unitRect, Radius.zero);
    });
    checkNeedsAddToScene(layer, () {
      layer.clipBehavior = Clip.antiAliasWithSaveLayer;
    });
  });

  test('mutating ClipPath fields triggers needsAddToScene', () {
    final ClipPathLayer layer = ClipPathLayer(clipPath: Path());
    checkNeedsAddToScene(layer, () {
      final Path newPath = Path();
      newPath.addRect(unitRect);
      layer.clipPath = newPath;
    });
    checkNeedsAddToScene(layer, () {
      layer.clipBehavior = Clip.antiAliasWithSaveLayer;
    });
  });

  test('mutating OpacityLayer fields triggers needsAddToScene', () {
    final OpacityLayer layer = OpacityLayer(alpha: 0);
    checkNeedsAddToScene(layer, () {
      layer.alpha = 1;
    });
    checkNeedsAddToScene(layer, () {
      layer.offset = const Offset(1, 1);
    });
  });

  test('mutating ColorFilterLayer fields triggers needsAddToScene', () {
    final ColorFilterLayer layer = ColorFilterLayer(
      colorFilter: const ColorFilter.mode(Color(0xFFFF0000), BlendMode.color),
    );
    checkNeedsAddToScene(layer, () {
      layer.colorFilter = const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color);
    });
  });

  test('mutating ShaderMaskLayer fields triggers needsAddToScene', () {
    const Gradient gradient = RadialGradient(colors: <Color>[Color(0x00000000), Color(0x00000001)]);
    final Shader shader = gradient.createShader(Rect.zero);
    final ShaderMaskLayer layer = ShaderMaskLayer(
      shader: shader,
      maskRect: Rect.zero,
      blendMode: BlendMode.clear,
    );
    checkNeedsAddToScene(layer, () {
      layer.maskRect = unitRect;
    });
    checkNeedsAddToScene(layer, () {
      layer.blendMode = BlendMode.color;
    });
    checkNeedsAddToScene(layer, () {
      layer.shader = gradient.createShader(unitRect);
    });
  });

  test('mutating BackdropFilterLayer fields triggers needsAddToScene', () {
    final BackdropFilterLayer layer = BackdropFilterLayer(filter: ImageFilter.blur());
    checkNeedsAddToScene(layer, () {
      layer.filter = ImageFilter.blur(sigmaX: 1.0);
    });
  });

  test(
    'ContainerLayer.toImage can render interior layer',
    () {
      final OffsetLayer parent = OffsetLayer();
      final OffsetLayer child = OffsetLayer();
      final OffsetLayer grandChild = OffsetLayer();
      child.append(grandChild);
      parent.append(child);

      // This renders the layers and generates engine layers.
      parent.buildScene(SceneBuilder());

      // Causes grandChild to pass its engine layer as `oldLayer`
      grandChild.toImage(const Rect.fromLTRB(0, 0, 10, 10));

      // Ensure we can render the same scene again after rendering an interior
      // layer.
      parent.buildScene(SceneBuilder());
    },
    // TODO(yjbanov): `toImage` doesn't work in HTML: https://github.com/flutter/flutter/issues/49857
    skip: isBrowser && !isSkiaWeb,
  );

  test(
    'ContainerLayer.toImageSync can render interior layer',
    () {
      final OffsetLayer parent = OffsetLayer();
      final OffsetLayer child = OffsetLayer();
      final OffsetLayer grandChild = OffsetLayer();
      child.append(grandChild);
      parent.append(child);

      // This renders the layers and generates engine layers.
      parent.buildScene(SceneBuilder());

      // Causes grandChild to pass its engine layer as `oldLayer`
      grandChild.toImageSync(const Rect.fromLTRB(0, 0, 10, 10));

      // Ensure we can render the same scene again after rendering an interior
      // layer.
      parent.buildScene(SceneBuilder());
    },
    // TODO(yjbanov): `toImage` doesn't work in HTML: https://github.com/flutter/flutter/issues/49857
    skip: isBrowser && !isSkiaWeb,
  );

  test('PictureLayer does not let you call dispose unless refcount is 0', () {
    PictureLayer layer = PictureLayer(Rect.zero);
    expect(layer.debugHandleCount, 0);
    layer.dispose();
    expect(layer.debugDisposed, true);

    layer = PictureLayer(Rect.zero);
    final LayerHandle<PictureLayer> handle = LayerHandle<PictureLayer>(layer);
    expect(layer.debugHandleCount, 1);
    expect(() => layer.dispose(), throwsAssertionError);
    handle.layer = null;
    expect(layer.debugHandleCount, 0);
    expect(layer.debugDisposed, true);
    expect(() => layer.dispose(), throwsAssertionError); // already disposed.
  });

  test('Layer append/remove increases/decreases handle count', () {
    final PictureLayer layer = PictureLayer(Rect.zero);
    final ContainerLayer parent = ContainerLayer();
    expect(layer.debugHandleCount, 0);
    expect(layer.debugDisposed, false);

    parent.append(layer);
    expect(layer.debugHandleCount, 1);
    expect(layer.debugDisposed, false);

    layer.remove();
    expect(layer.debugHandleCount, 0);
    expect(layer.debugDisposed, true);
  });

  test('Layer.dispose disposes the engineLayer', () {
    final Layer layer = ConcreteLayer();
    final FakeEngineLayer engineLayer = FakeEngineLayer();
    layer.engineLayer = engineLayer;
    expect(engineLayer.disposed, false);
    layer.dispose();
    expect(engineLayer.disposed, true);
    expect(layer.engineLayer, null);
  });

  test('Layer.engineLayer (set) disposes the engineLayer', () {
    final Layer layer = ConcreteLayer();
    final FakeEngineLayer engineLayer = FakeEngineLayer();
    layer.engineLayer = engineLayer;
    expect(engineLayer.disposed, false);
    layer.engineLayer = null;
    expect(engineLayer.disposed, true);
  });

  test('PictureLayer.picture (set) disposes the picture', () {
    final PictureLayer layer = PictureLayer(Rect.zero);
    final FakePicture picture = FakePicture();
    layer.picture = picture;
    expect(picture.disposed, false);
    layer.picture = null;
    expect(picture.disposed, true);
  });

  test('PictureLayer disposes the picture', () {
    final PictureLayer layer = PictureLayer(Rect.zero);
    final FakePicture picture = FakePicture();
    layer.picture = picture;
    expect(picture.disposed, false);
    layer.dispose();
    expect(picture.disposed, true);
  });

  test('LayerHandle disposes the layer', () {
    final ConcreteLayer layer = ConcreteLayer();
    final ConcreteLayer layer2 = ConcreteLayer();

    expect(layer.debugHandleCount, 0);
    expect(layer2.debugHandleCount, 0);

    final LayerHandle<ConcreteLayer> holder = LayerHandle<ConcreteLayer>(layer);
    expect(layer.debugHandleCount, 1);
    expect(layer.debugDisposed, false);
    expect(layer2.debugHandleCount, 0);
    expect(layer2.debugDisposed, false);

    holder.layer = layer;
    expect(layer.debugHandleCount, 1);
    expect(layer.debugDisposed, false);
    expect(layer2.debugHandleCount, 0);
    expect(layer2.debugDisposed, false);

    holder.layer = layer2;
    expect(layer.debugHandleCount, 0);
    expect(layer.debugDisposed, true);
    expect(layer2.debugHandleCount, 1);
    expect(layer2.debugDisposed, false);

    holder.layer = null;
    expect(layer.debugHandleCount, 0);
    expect(layer.debugDisposed, true);
    expect(layer2.debugHandleCount, 0);
    expect(layer2.debugDisposed, true);

    expect(() => holder.layer = layer, throwsAssertionError);
  });

  test('OpacityLayer does not push an OffsetLayer if there are no children', () {
    final OpacityLayer layer = OpacityLayer(alpha: 128);
    final FakeSceneBuilder builder = FakeSceneBuilder();
    layer.addToScene(builder);
    expect(builder.pushedOpacity, false);
    expect(builder.pushedOffset, false);
    expect(builder.addedPicture, false);
    expect(layer.engineLayer, null);

    layer.append(PictureLayer(Rect.largest)..picture = FakePicture());

    builder.reset();
    layer.addToScene(builder);

    expect(builder.pushedOpacity, true);
    expect(builder.pushedOffset, false);
    expect(builder.addedPicture, true);
    expect(layer.engineLayer, isA<FakeOpacityEngineLayer>());

    builder.reset();

    layer.alpha = 200;
    expect(layer.engineLayer, isA<FakeOpacityEngineLayer>());

    layer.alpha = 255;
    expect(layer.engineLayer, null);

    builder.reset();
    layer.addToScene(builder);

    expect(builder.pushedOpacity, false);
    expect(builder.pushedOffset, true);
    expect(builder.addedPicture, true);
    expect(layer.engineLayer, isA<FakeOffsetEngineLayer>());

    layer.alpha = 200;
    expect(layer.engineLayer, null);

    builder.reset();
    layer.addToScene(builder);

    expect(builder.pushedOpacity, true);
    expect(builder.pushedOffset, false);
    expect(builder.addedPicture, true);
    expect(layer.engineLayer, isA<FakeOpacityEngineLayer>());
  });

  test('OpacityLayer dispose its engineLayer if there are no children', () {
    final OpacityLayer layer = OpacityLayer(alpha: 128);
    final FakeSceneBuilder builder = FakeSceneBuilder();
    layer.addToScene(builder);
    expect(layer.engineLayer, null);

    layer.append(PictureLayer(Rect.largest)..picture = FakePicture());
    layer.addToScene(builder);
    expect(layer.engineLayer, isA<FakeOpacityEngineLayer>());

    layer.removeAllChildren();
    layer.addToScene(builder);
    expect(layer.engineLayer, null);
  });

  test('Layers describe clip bounds', () {
    ContainerLayer layer = ContainerLayer();
    expect(layer.describeClipBounds(), null);

    const Rect bounds = Rect.fromLTRB(10, 10, 20, 20);
    final RRect rbounds = RRect.fromRectXY(bounds, 2, 2);
    layer = ClipRectLayer(clipRect: bounds);
    expect(layer.describeClipBounds(), bounds);

    layer = ClipRRectLayer(clipRRect: rbounds);
    expect(layer.describeClipBounds(), rbounds.outerRect);

    layer = ClipPathLayer(clipPath: Path()..addRect(bounds));
    expect(layer.describeClipBounds(), bounds);
  });

  test('Subtree has composition callbacks', () {
    final ContainerLayer root = ContainerLayer();
    expect(root.subtreeHasCompositionCallbacks, false);

    final List<VoidCallback> cancellationCallbacks = <VoidCallback>[];

    cancellationCallbacks.add(root.addCompositionCallback((_) {}));
    expect(root.subtreeHasCompositionCallbacks, true);

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, false);
    expect(a2.subtreeHasCompositionCallbacks, false);
    expect(b1.subtreeHasCompositionCallbacks, false);
    cancellationCallbacks.add(b1.addCompositionCallback((_) {}));

    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, true);
    expect(a2.subtreeHasCompositionCallbacks, false);
    expect(b1.subtreeHasCompositionCallbacks, true);

    cancellationCallbacks.removeAt(0)();

    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, true);
    expect(a2.subtreeHasCompositionCallbacks, false);
    expect(b1.subtreeHasCompositionCallbacks, true);

    cancellationCallbacks.removeAt(0)();

    expect(root.subtreeHasCompositionCallbacks, false);
    expect(a1.subtreeHasCompositionCallbacks, false);
    expect(a2.subtreeHasCompositionCallbacks, false);
    expect(b1.subtreeHasCompositionCallbacks, false);
  });

  test('Subtree has composition callbacks - removeChild', () {
    final ContainerLayer root = ContainerLayer();
    expect(root.subtreeHasCompositionCallbacks, false);

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    expect(b1.subtreeHasCompositionCallbacks, false);
    expect(a1.subtreeHasCompositionCallbacks, false);
    expect(root.subtreeHasCompositionCallbacks, false);
    expect(a2.subtreeHasCompositionCallbacks, false);

    b1.addCompositionCallback((_) {});

    expect(b1.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, true);
    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a2.subtreeHasCompositionCallbacks, false);

    b1.remove();

    expect(b1.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, false);
    expect(root.subtreeHasCompositionCallbacks, false);
    expect(a2.subtreeHasCompositionCallbacks, false);
  });

  test('No callback if removed', () {
    final ContainerLayer root = ContainerLayer();

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    // Add and immediately remove the callback.
    b1.addCompositionCallback((Layer layer) {
      fail('Should not have called back');
    })();

    root.buildScene(SceneBuilder()).dispose();
  });

  test('Observe layer tree composition - not retained', () {
    final ContainerLayer root = ContainerLayer();

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    bool compositedB1 = false;

    b1.addCompositionCallback((Layer layer) {
      expect(layer, b1);
      compositedB1 = true;
    });

    expect(compositedB1, false);

    root.buildScene(SceneBuilder()).dispose();

    expect(compositedB1, true);
  });

  test('Observe layer tree composition - retained', () {
    final ContainerLayer root = ContainerLayer();

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    // Actually build the retained layer so that the engine sees it as real and
    // reusable.
    SceneBuilder builder = SceneBuilder();
    b1.engineLayer = builder.pushOffset(0, 0);
    builder.build().dispose();
    builder = SceneBuilder();

    // Force the layer to appear clean and have an engine layer for retained
    // rendering.
    expect(b1.engineLayer, isNotNull);
    b1.debugMarkClean();
    expect(b1.debugSubtreeNeedsAddToScene, false);

    bool compositedB1 = false;

    b1.addCompositionCallback((Layer layer) {
      expect(layer, b1);
      compositedB1 = true;
    });

    expect(compositedB1, false);

    root.buildScene(builder).dispose();

    expect(compositedB1, true);
  });

  test('Observe layer tree composition - asserts on mutation', () {
    final ContainerLayer root = ContainerLayer();

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    bool compositedB1 = false;

    b1.addCompositionCallback((Layer layer) {
      expect(layer, b1);
      expect(() => layer.remove(), throwsAssertionError);
      expect(() => layer.dispose(), throwsAssertionError);
      expect(() => layer.markNeedsAddToScene(), throwsAssertionError);
      expect(() => layer.debugMarkClean(), throwsAssertionError);
      expect(() => layer.updateSubtreeNeedsAddToScene(), throwsAssertionError);
      expect(() => layer.remove(), throwsAssertionError);
      expect(() => (layer as ContainerLayer).append(ContainerLayer()), throwsAssertionError);
      expect(() => layer.engineLayer = null, throwsAssertionError);
      compositedB1 = true;
    });

    expect(compositedB1, false);

    root.buildScene(SceneBuilder()).dispose();

    expect(compositedB1, true);
  });

  test('Observe layer tree composition - detach triggers callback', () {
    final ContainerLayer root = ContainerLayer();

    final ContainerLayer a1 = ContainerLayer();
    final ContainerLayer a2 = ContainerLayer();
    final ContainerLayer b1 = ContainerLayer();
    root.append(a1);
    root.append(a2);
    a1.append(b1);

    bool compositedB1 = false;

    b1.addCompositionCallback((Layer layer) {
      expect(layer, b1);
      compositedB1 = true;
    });

    root.attach(Object());
    expect(compositedB1, false);
    root.detach();
    expect(compositedB1, true);
  });

  test('Observe layer tree composition - observer count correctly maintained', () {
    final ContainerLayer root = ContainerLayer();
    final ContainerLayer a1 = ContainerLayer();
    root.append(a1);

    expect(root.subtreeHasCompositionCallbacks, false);
    expect(a1.subtreeHasCompositionCallbacks, false);

    final VoidCallback remover1 = a1.addCompositionCallback((_) {});
    final VoidCallback remover2 = a1.addCompositionCallback((_) {});

    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, true);

    remover1();

    expect(root.subtreeHasCompositionCallbacks, true);
    expect(a1.subtreeHasCompositionCallbacks, true);

    remover2();

    expect(root.subtreeHasCompositionCallbacks, false);
    expect(a1.subtreeHasCompositionCallbacks, false);
  });

  test('Double removing a observe callback throws', () {
    final ContainerLayer root = ContainerLayer();
    final VoidCallback callback = root.addCompositionCallback((_) {});
    callback();

    expect(() => callback(), throwsAssertionError);
  });

  test('Removing an observe callback on a disposed layer does not throw', () {
    final ContainerLayer root = ContainerLayer();
    final VoidCallback callback = root.addCompositionCallback((_) {});
    root.dispose();
    expect(() => callback(), returnsNormally);
  });

  test('Layer types that support rasterization', () {
    // Supported.
    final OffsetLayer offsetLayer = OffsetLayer();
    final OpacityLayer opacityLayer = OpacityLayer();
    final ClipRectLayer clipRectLayer = ClipRectLayer();
    final ClipRRectLayer clipRRectLayer = ClipRRectLayer();
    final ImageFilterLayer imageFilterLayer = ImageFilterLayer();
    final BackdropFilterLayer backdropFilterLayer = BackdropFilterLayer();
    final ColorFilterLayer colorFilterLayer = ColorFilterLayer();
    final ShaderMaskLayer shaderMaskLayer = ShaderMaskLayer();
    final TextureLayer textureLayer = TextureLayer(rect: Rect.zero, textureId: 1);
    expect(offsetLayer.supportsRasterization(), true);
    expect(opacityLayer.supportsRasterization(), true);
    expect(clipRectLayer.supportsRasterization(), true);
    expect(clipRRectLayer.supportsRasterization(), true);
    expect(imageFilterLayer.supportsRasterization(), true);
    expect(backdropFilterLayer.supportsRasterization(), true);
    expect(colorFilterLayer.supportsRasterization(), true);
    expect(shaderMaskLayer.supportsRasterization(), true);
    expect(textureLayer.supportsRasterization(), true);

    // Unsupported.
    final PlatformViewLayer platformViewLayer = PlatformViewLayer(rect: Rect.zero, viewId: 1);

    expect(platformViewLayer.supportsRasterization(), false);
  });
}

class FakeEngineLayer extends Fake implements EngineLayer {
  bool disposed = false;

  @override
  void dispose() {
    assert(!disposed);
    disposed = true;
  }
}

class FakePicture extends Fake implements Picture {
  bool disposed = false;

  @override
  void dispose() {
    assert(!disposed);
    disposed = true;
  }
}

class ConcreteLayer extends Layer {
  @override
  void addToScene(SceneBuilder builder) {}
}

class _TestAlwaysNeedsAddToSceneLayer extends ContainerLayer {
  @override
  bool get alwaysNeedsAddToScene => true;
}

class FakeSceneBuilder extends Fake implements SceneBuilder {
  void reset() {
    pushedOpacity = false;
    pushedOffset = false;
    addedPicture = false;
  }

  bool pushedOpacity = false;
  bool pushedOffset = false;
  bool addedPicture = false;

  @override
  dynamic noSuchMethod(Invocation invocation) {
    // Use noSuchMethod forwarding instead of override these methods to make it easier
    // for these methods to add new optional arguments in the future.
    switch (invocation.memberName) {
      case #pushOpacity:
        pushedOpacity = true;
        return FakeOpacityEngineLayer();
      case #pushOffset:
        pushedOffset = true;
        return FakeOffsetEngineLayer();
      case #addPicture:
        addedPicture = true;
        return;
      case #pop:
        return;
    }
    super.noSuchMethod(invocation);
  }
}

class FakeOpacityEngineLayer extends FakeEngineLayer implements OpacityEngineLayer {}

class FakeOffsetEngineLayer extends FakeEngineLayer implements OffsetEngineLayer {}
