// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Texture with freeze set to true', (WidgetTester tester) async {
    await tester.pumpWidget(const Center(child: Texture(textureId: 1, freeze: true)));

    final Texture texture = tester.firstWidget(find.byType(Texture));
    expect(texture, isNotNull);
    expect(texture.textureId, 1);
    expect(texture.freeze, true);

    final RenderObject renderObject = tester.firstRenderObject(find.byType(Texture));
    expect(renderObject, isNotNull);
    final TextureBox textureBox = renderObject as TextureBox;
    expect(textureBox, isNotNull);
    expect(textureBox.textureId, 1);
    expect(textureBox.freeze, true);

    final ContainerLayer containerLayer = ContainerLayer();
    addTearDown(containerLayer.dispose);
    final PaintingContext paintingContext = PaintingContext(containerLayer, Rect.zero);
    textureBox.paint(paintingContext, Offset.zero);
    final Layer layer = containerLayer.lastChild!;
    expect(layer, isNotNull);
    final TextureLayer textureLayer = layer as TextureLayer;
    expect(textureLayer, isNotNull);
    expect(textureLayer.textureId, 1);
    expect(textureLayer.freeze, true);
  });

  testWidgets('Texture with default FilterQuality', (WidgetTester tester) async {
    await tester.pumpWidget(const Center(child: Texture(textureId: 1)));

    final Texture texture = tester.firstWidget(find.byType(Texture));
    expect(texture, isNotNull);
    expect(texture.textureId, 1);
    expect(texture.filterQuality, FilterQuality.low);

    final RenderObject renderObject = tester.firstRenderObject(find.byType(Texture));
    expect(renderObject, isNotNull);
    final TextureBox textureBox = renderObject as TextureBox;
    expect(textureBox, isNotNull);
    expect(textureBox.textureId, 1);
    expect(textureBox.filterQuality, FilterQuality.low);

    final ContainerLayer containerLayer = ContainerLayer();
    addTearDown(containerLayer.dispose);
    final PaintingContext paintingContext = PaintingContext(containerLayer, Rect.zero);
    textureBox.paint(paintingContext, Offset.zero);
    final Layer layer = containerLayer.lastChild!;
    expect(layer, isNotNull);
    final TextureLayer textureLayer = layer as TextureLayer;
    expect(textureLayer, isNotNull);
    expect(textureLayer.textureId, 1);
    expect(textureLayer.filterQuality, FilterQuality.low);
  });

  testWidgets('Texture with FilterQuality.none', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(child: Texture(textureId: 1, filterQuality: FilterQuality.none)),
    );

    final Texture texture = tester.firstWidget(find.byType(Texture));
    expect(texture, isNotNull);
    expect(texture.textureId, 1);
    expect(texture.filterQuality, FilterQuality.none);

    final RenderObject renderObject = tester.firstRenderObject(find.byType(Texture));
    expect(renderObject, isNotNull);
    final TextureBox textureBox = renderObject as TextureBox;
    expect(textureBox, isNotNull);
    expect(textureBox.textureId, 1);
    expect(textureBox.filterQuality, FilterQuality.none);

    final ContainerLayer containerLayer = ContainerLayer();
    addTearDown(containerLayer.dispose);
    final PaintingContext paintingContext = PaintingContext(containerLayer, Rect.zero);
    textureBox.paint(paintingContext, Offset.zero);
    final Layer layer = containerLayer.lastChild!;
    expect(layer, isNotNull);
    final TextureLayer textureLayer = layer as TextureLayer;
    expect(textureLayer, isNotNull);
    expect(textureLayer.textureId, 1);
    expect(textureLayer.filterQuality, FilterQuality.none);
  });

  testWidgets('Texture with FilterQuality.low', (WidgetTester tester) async {
    await tester.pumpWidget(const Center(child: Texture(textureId: 1)));

    final Texture texture = tester.firstWidget(find.byType(Texture));
    expect(texture, isNotNull);
    expect(texture.textureId, 1);
    expect(texture.filterQuality, FilterQuality.low);

    final RenderObject renderObject = tester.firstRenderObject(find.byType(Texture));
    expect(renderObject, isNotNull);
    final TextureBox textureBox = renderObject as TextureBox;
    expect(textureBox, isNotNull);
    expect(textureBox.textureId, 1);
    expect(textureBox.filterQuality, FilterQuality.low);

    final ContainerLayer containerLayer = ContainerLayer();
    addTearDown(containerLayer.dispose);
    final PaintingContext paintingContext = PaintingContext(containerLayer, Rect.zero);
    textureBox.paint(paintingContext, Offset.zero);
    final Layer layer = containerLayer.lastChild!;
    expect(layer, isNotNull);
    final TextureLayer textureLayer = layer as TextureLayer;
    expect(textureLayer, isNotNull);
    expect(textureLayer.textureId, 1);
    expect(textureLayer.filterQuality, FilterQuality.low);
  });
}
