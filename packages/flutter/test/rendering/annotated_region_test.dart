// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('$AnnotatedRegion find', () {
    test('finds the first value in a OffsetLayer when sized', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<OffsetLayer> layers = <OffsetLayer>[
        OffsetLayer(),
        OffsetLayer(offset: const Offset(0.0, 100.0)),
        OffsetLayer(offset: const Offset(0.0, 200.0)),
      ];
      int i = 0;
      for (final OffsetLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i, size: const Size(200.0, 100.0)));
        containerLayer.append(layer);
        i += 1;
      }

      expect(containerLayer.find<int>(const Offset(0.0, 1.0)), 0);
      expect(containerLayer.find<int>(const Offset(0.0, 101.0)), 1);
      expect(containerLayer.find<int>(const Offset(0.0, 201.0)), 2);
    });

    test('finds a value within the clip in a ClipRectLayer', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<ClipRectLayer> layers = <ClipRectLayer>[
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 0.0, 100.0, 100.0)),
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 100.0, 100.0, 200.0)),
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 200.0, 100.0, 300.0)),
      ];
      int i = 0;
      for (final ClipRectLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i));
        containerLayer.append(layer);
        i += 1;
      }

      expect(containerLayer.find<int>(const Offset(0.0, 1.0)), 0);
      expect(containerLayer.find<int>(const Offset(0.0, 101.0)), 1);
      expect(containerLayer.find<int>(const Offset(0.0, 201.0)), 2);
    });

    test('finds a value within the clip in a ClipRRectLayer', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<ClipRRectLayer> layers = <ClipRRectLayer>[
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 0.0, 100.0, 100.0, const Radius.circular(4.0)),
        ),
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 100.0, 100.0, 200.0, const Radius.circular(4.0)),
        ),
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 200.0, 100.0, 300.0, const Radius.circular(4.0)),
        ),
      ];
      int i = 0;
      for (final ClipRRectLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i));
        containerLayer.append(layer);
        i += 1;
      }

      expect(containerLayer.find<int>(const Offset(5.0, 5.0)), 0);
      expect(containerLayer.find<int>(const Offset(5.0, 105.0)), 1);
      expect(containerLayer.find<int>(const Offset(5.0, 205.0)), 2);
    });

    test('finds a value under a TransformLayer', () {
      final Matrix4 transform = Matrix4(
        2.625,
        0.0,
        0.0,
        0.0,
        0.0,
        2.625,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
      );
      final TransformLayer transformLayer = TransformLayer(transform: transform);
      final List<OffsetLayer> layers = <OffsetLayer>[
        OffsetLayer(),
        OffsetLayer(offset: const Offset(0.0, 100.0)),
        OffsetLayer(offset: const Offset(0.0, 200.0)),
      ];
      int i = 0;
      for (final OffsetLayer layer in layers) {
        final AnnotatedRegionLayer<int> annotatedRegionLayer = AnnotatedRegionLayer<int>(
          i,
          size: const Size(100.0, 100.0),
        );
        layer.append(annotatedRegionLayer);
        transformLayer.append(layer);
        i += 1;
      }

      expect(transformLayer.find<int>(const Offset(0.0, 100.0)), 0);
      expect(transformLayer.find<int>(const Offset(0.0, 200.0)), 0);
      expect(transformLayer.find<int>(const Offset(0.0, 270.0)), 1);
      expect(transformLayer.find<int>(const Offset(0.0, 400.0)), 1);
      expect(transformLayer.find<int>(const Offset(0.0, 530.0)), 2);
    });

    test('looks for child AnnotatedRegions before parents', () {
      final AnnotatedRegionLayer<int> parent = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<int> child = AnnotatedRegionLayer<int>(2);
      final ContainerLayer layer = ContainerLayer();
      parent.append(child);
      layer.append(parent);

      expect(parent.find<int>(Offset.zero), 2);
    });

    test('looks for correct type', () {
      final AnnotatedRegionLayer<int> child1 = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<String> child2 = AnnotatedRegionLayer<String>('hello');
      final ContainerLayer layer = ContainerLayer();
      layer.append(child2);
      layer.append(child1);

      expect(layer.find<String>(Offset.zero), 'hello');
    });

    test('does not clip Layer.find on an AnnotatedRegion with an unrelated type', () {
      final AnnotatedRegionLayer<int> child = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<String> parent = AnnotatedRegionLayer<String>(
        'hello',
        size: const Size(10.0, 10.0),
      );
      final ContainerLayer layer = ContainerLayer();
      parent.append(child);
      layer.append(parent);

      expect(layer.find<int>(const Offset(100.0, 100.0)), 1);
    });

    test('handles non-invertible transforms', () {
      final AnnotatedRegionLayer<int> child = AnnotatedRegionLayer<int>(1);
      final TransformLayer parent = TransformLayer(
        transform: Matrix4.diagonal3Values(0.0, 1.0, 1.0),
      );
      parent.append(child);

      expect(parent.find<int>(Offset.zero), null);

      parent.transform = Matrix4.diagonal3Values(1.0, 1.0, 1.0);

      expect(parent.find<int>(Offset.zero), 1);
    });
  });
  group('$AnnotatedRegion findAllAnnotations', () {
    test('finds the first value in a OffsetLayer when sized', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<OffsetLayer> layers = <OffsetLayer>[
        OffsetLayer(),
        OffsetLayer(offset: const Offset(0.0, 100.0)),
        OffsetLayer(offset: const Offset(0.0, 200.0)),
      ];
      int i = 0;
      for (final OffsetLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i, size: const Size(200.0, 100.0)));
        containerLayer.append(layer);
        i += 1;
      }

      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 1.0)).annotations.toList(),
        equals(<int>[0]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 101.0)).annotations.toList(),
        equals(<int>[1]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 201.0)).annotations.toList(),
        equals(<int>[2]),
      );
    });

    test('finds a value within the clip in a ClipRectLayer', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<ClipRectLayer> layers = <ClipRectLayer>[
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 0.0, 100.0, 100.0)),
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 100.0, 100.0, 200.0)),
        ClipRectLayer(clipRect: const Rect.fromLTRB(0.0, 200.0, 100.0, 300.0)),
      ];
      int i = 0;
      for (final ClipRectLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i));
        containerLayer.append(layer);
        i += 1;
      }

      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 1.0)).annotations.toList(),
        equals(<int>[0]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 101.0)).annotations.toList(),
        equals(<int>[1]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(0.0, 201.0)).annotations.toList(),
        equals(<int>[2]),
      );
    });

    test('finds a value within the clip in a ClipRRectLayer', () {
      final ContainerLayer containerLayer = ContainerLayer();
      final List<ClipRRectLayer> layers = <ClipRRectLayer>[
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 0.0, 100.0, 100.0, const Radius.circular(4.0)),
        ),
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 100.0, 100.0, 200.0, const Radius.circular(4.0)),
        ),
        ClipRRectLayer(
          clipRRect: RRect.fromLTRBR(0.0, 200.0, 100.0, 300.0, const Radius.circular(4.0)),
        ),
      ];
      int i = 0;
      for (final ClipRRectLayer layer in layers) {
        layer.append(AnnotatedRegionLayer<int>(i));
        containerLayer.append(layer);
        i += 1;
      }

      expect(
        containerLayer.findAllAnnotations<int>(const Offset(5.0, 5.0)).annotations.toList(),
        equals(<int>[0]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(5.0, 105.0)).annotations.toList(),
        equals(<int>[1]),
      );
      expect(
        containerLayer.findAllAnnotations<int>(const Offset(5.0, 205.0)).annotations.toList(),
        equals(<int>[2]),
      );
    });

    test('finds a value under a TransformLayer', () {
      final Matrix4 transform = Matrix4(
        2.625,
        0.0,
        0.0,
        0.0,
        0.0,
        2.625,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
      );
      final TransformLayer transformLayer = TransformLayer(transform: transform);
      final List<OffsetLayer> layers = <OffsetLayer>[
        OffsetLayer(),
        OffsetLayer(offset: const Offset(0.0, 100.0)),
        OffsetLayer(offset: const Offset(0.0, 200.0)),
      ];
      int i = 0;
      for (final OffsetLayer layer in layers) {
        final AnnotatedRegionLayer<int> annotatedRegionLayer = AnnotatedRegionLayer<int>(
          i,
          size: const Size(100.0, 100.0),
        );
        layer.append(annotatedRegionLayer);
        transformLayer.append(layer);
        i += 1;
      }

      expect(
        transformLayer.findAllAnnotations<int>(const Offset(0.0, 100.0)).annotations.toList(),
        equals(<int>[0]),
      );
      expect(
        transformLayer.findAllAnnotations<int>(const Offset(0.0, 200.0)).annotations.toList(),
        equals(<int>[0]),
      );
      expect(
        transformLayer.findAllAnnotations<int>(const Offset(0.0, 270.0)).annotations.toList(),
        equals(<int>[1]),
      );
      expect(
        transformLayer.findAllAnnotations<int>(const Offset(0.0, 400.0)).annotations.toList(),
        equals(<int>[1]),
      );
      expect(
        transformLayer.findAllAnnotations<int>(const Offset(0.0, 530.0)).annotations.toList(),
        equals(<int>[2]),
      );
    });

    test('finds multiple nested, overlapping regions', () {
      final ContainerLayer parent = ContainerLayer();

      int index = 0;
      final List<AnnotatedRegionLayer<int>> layers = <AnnotatedRegionLayer<int>>[
        AnnotatedRegionLayer<int>(index++, size: const Size(100.0, 100.0)),
        AnnotatedRegionLayer<int>(index++, size: const Size(100.0, 100.0)),
      ];
      for (final ContainerLayer layer in layers) {
        final AnnotatedRegionLayer<int> annotatedRegionLayer = AnnotatedRegionLayer<int>(
          index++,
          size: const Size(100.0, 100.0),
        );
        layer.append(annotatedRegionLayer);
        parent.append(layer);
      }

      expect(
        parent.findAllAnnotations<int>(Offset.zero).annotations.toList(),
        equals(<int>[3, 1, 2, 0]),
      );
    });

    test('looks for child AnnotatedRegions before parents', () {
      final AnnotatedRegionLayer<int> parent = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<int> child1 = AnnotatedRegionLayer<int>(2);
      final AnnotatedRegionLayer<int> child2 = AnnotatedRegionLayer<int>(3);
      final AnnotatedRegionLayer<int> child3 = AnnotatedRegionLayer<int>(4);
      final ContainerLayer layer = ContainerLayer();
      parent.append(child1);
      parent.append(child2);
      parent.append(child3);
      layer.append(parent);

      expect(
        parent.findAllAnnotations<int>(Offset.zero).annotations.toList(),
        equals(<int>[4, 3, 2, 1]),
      );
    });

    test('looks for correct type', () {
      final AnnotatedRegionLayer<int> child1 = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<String> child2 = AnnotatedRegionLayer<String>('hello');
      final ContainerLayer layer = ContainerLayer();
      layer.append(child2);
      layer.append(child1);

      expect(
        layer.findAllAnnotations<String>(Offset.zero).annotations.toList(),
        equals(<String>['hello']),
      );
    });

    test('does not clip Layer.find on an AnnotatedRegion with an unrelated type', () {
      final AnnotatedRegionLayer<int> child = AnnotatedRegionLayer<int>(1);
      final AnnotatedRegionLayer<String> parent = AnnotatedRegionLayer<String>(
        'hello',
        size: const Size(10.0, 10.0),
      );
      final ContainerLayer layer = ContainerLayer();
      parent.append(child);
      layer.append(parent);

      expect(
        layer.findAllAnnotations<int>(const Offset(100.0, 100.0)).annotations.toList(),
        equals(<int>[1]),
      );
    });

    test('handles non-invertible transforms', () {
      final AnnotatedRegionLayer<int> child = AnnotatedRegionLayer<int>(1);
      final TransformLayer parent = TransformLayer(
        transform: Matrix4.diagonal3Values(0.0, 1.0, 1.0),
      );
      parent.append(child);

      expect(parent.findAllAnnotations<int>(Offset.zero).annotations.toList(), equals(<int>[]));

      parent.transform = Matrix4.diagonal3Values(1.0, 1.0, 1.0);

      expect(parent.findAllAnnotations<int>(Offset.zero).annotations.toList(), equals(<int>[1]));
    });
  });
}
