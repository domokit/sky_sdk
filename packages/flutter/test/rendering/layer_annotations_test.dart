// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('ContainerLayer.findAllAnnotations returns all results from its children', () {
    final Layer root =
        _Layers(
          ContainerLayer(),
          children: <Object>[
            _TestAnnotatedLayer(1, opaque: false),
            _TestAnnotatedLayer(2, opaque: false),
            _TestAnnotatedLayer(3, opaque: false),
          ],
        ).build();

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 3, localPosition: Offset.zero),
        const AnnotationEntry<int>(annotation: 2, localPosition: Offset.zero),
        const AnnotationEntry<int>(annotation: 1, localPosition: Offset.zero),
      ]),
    );
  });

  test('ContainerLayer.find returns the first result from its children', () {
    final Layer root =
        _Layers(
          ContainerLayer(),
          children: <Object>[
            _TestAnnotatedLayer(1, opaque: false),
            _TestAnnotatedLayer(2, opaque: false),
            _TestAnnotatedLayer(3, opaque: false),
          ],
        ).build();

    final int result = root.find<int>(Offset.zero)!;
    expect(result, 3);
  });

  test('ContainerLayer.findAllAnnotations returns empty result when finding nothing', () {
    final Layer root =
        _Layers(
          ContainerLayer(),
          children: <Object>[
            _TestAnnotatedLayer(1, opaque: false),
            _TestAnnotatedLayer(2, opaque: false),
            _TestAnnotatedLayer(3, opaque: false),
          ],
        ).build();

    expect(root.findAllAnnotations<double>(Offset.zero).entries.isEmpty, isTrue);
  });

  test('ContainerLayer.find returns null when finding nothing', () {
    final Layer root =
        _Layers(
          ContainerLayer(),
          children: <Object>[
            _TestAnnotatedLayer(1, opaque: false),
            _TestAnnotatedLayer(2, opaque: false),
            _TestAnnotatedLayer(3, opaque: false),
          ],
        ).build();

    expect(root.find<double>(Offset.zero), isNull);
  });

  test('ContainerLayer.findAllAnnotations stops at the first opaque child', () {
    final Layer root =
        _Layers(
          ContainerLayer(),
          children: <Object>[
            _TestAnnotatedLayer(1, opaque: false),
            _TestAnnotatedLayer(2, opaque: true),
            _TestAnnotatedLayer(3, opaque: false),
          ],
        ).build();

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 3, localPosition: Offset.zero),
        const AnnotationEntry<int>(annotation: 2, localPosition: Offset.zero),
      ]),
    );
  });

  test("ContainerLayer.findAllAnnotations returns children's opacity (true)", () {
    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(ContainerLayer(), children: <Object>[_TestAnnotatedLayer(2, opaque: true)]).build(),
    );

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: Offset.zero),
      ]),
    );
  });

  test("ContainerLayer.findAllAnnotations returns children's opacity (false)", () {
    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(ContainerLayer(), children: <Object>[_TestAnnotatedLayer(2, opaque: false)]).build(),
    );

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: Offset.zero),
        const AnnotationEntry<int>(annotation: 1000, localPosition: Offset.zero),
      ]),
    );
  });

  test('ContainerLayer.findAllAnnotations returns false as opacity when finding nothing', () {
    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        ContainerLayer(),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false, size: Size.zero)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: Offset.zero),
      ]),
    );
  });

  test('OffsetLayer.findAllAnnotations respects offset', () {
    const Offset insidePosition = Offset(-5, 5);
    const Offset outsidePosition = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        OffsetLayer(offset: const Offset(-10, 0)),
        children: <Object>[_TestAnnotatedLayer(1, opaque: true, size: const Size(10, 10))],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: Offset(5, 5)),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: Offset(5, 5)),
      ]),
    );
  });

  test('ClipRectLayer.findAllAnnotations respects clipRect', () {
    const Offset insidePosition = Offset(11, 11);
    const Offset outsidePosition = Offset(19, 19);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        ClipRectLayer(clipRect: const Offset(10, 10) & const Size(5, 5)),
        children: <Object>[
          _TestAnnotatedLayer(
            1,
            opaque: true,
            size: const Size(10, 10),
            offset: const Offset(10, 10),
          ),
        ],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: insidePosition),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: outsidePosition),
      ]),
    );
  });

  test('ClipRRectLayer.findAllAnnotations respects clipRRect', () {
    // For a curve of radius 4 centered at (4, 4),
    // location (1, 1) is outside, while (2, 2) is inside.
    // Here we shift this RRect by (10, 10).
    final RRect rrect = RRect.fromRectAndRadius(
      const Offset(10, 10) & const Size(10, 10),
      const Radius.circular(4),
    );
    const Offset insidePosition = Offset(12, 12);
    const Offset outsidePosition = Offset(11, 11);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        ClipRRectLayer(clipRRect: rrect),
        children: <Object>[
          _TestAnnotatedLayer(
            1,
            opaque: true,
            size: const Size(10, 10),
            offset: const Offset(10, 10),
          ),
        ],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: insidePosition),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: outsidePosition),
      ]),
    );
  });

  test('ClipPathLayer.findAllAnnotations respects clipPath', () {
    // For this triangle, location (1, 1) is inside, while (2, 2) is outside.
    //         2
    //    —————
    //    |  /
    //    | /
    // 2  |/
    final Path originalPath = Path();
    originalPath.lineTo(2, 0);
    originalPath.lineTo(0, 2);
    originalPath.close();
    // Shift this clip path by (10, 10).
    final Path path = originalPath.shift(const Offset(10, 10));
    const Offset insidePosition = Offset(11, 11);
    const Offset outsidePosition = Offset(12, 12);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        ClipPathLayer(clipPath: path),
        children: <Object>[
          _TestAnnotatedLayer(
            1,
            opaque: true,
            size: const Size(10, 10),
            offset: const Offset(10, 10),
          ),
        ],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: insidePosition),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: outsidePosition),
      ]),
    );
  });

  test('TransformLayer.findAllAnnotations respects transform', () {
    // Matrix `transform` enlarges the target by (2x, 4x), then shift it by
    // (10, 20).
    final Matrix4 transform = Matrix4.diagonal3Values(2, 4, 1)..setTranslation(Vector3(10, 20, 0));
    // The original region is Offset(10, 10) & Size(10, 10)
    // The transformed region is Offset(30, 60) & Size(20, 40)
    const Offset insidePosition = Offset(40, 80);
    const Offset outsidePosition = Offset(20, 40);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        TransformLayer(transform: transform),
        children: <Object>[
          _TestAnnotatedLayer(
            1,
            opaque: true,
            size: const Size(10, 10),
            offset: const Offset(10, 10),
          ),
        ],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: Offset(15, 15)),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: outsidePosition),
      ]),
    );
  });

  test('TransformLayer.findAllAnnotations correctly transforms with perspective', () {
    // Test the 4 corners of a transformed annotated region.
    final Matrix4 transform =
        Matrix4.identity()
          ..setEntry(3, 2, 0.005)
          ..rotateX(-0.2)
          ..rotateY(0.2);

    final Layer root = _withBackgroundAnnotation(
      0,
      _Layers(
        TransformLayer(transform: transform),
        children: <Object>[
          _TestAnnotatedLayer(
            1,
            opaque: true,
            size: const Size(30, 40),
            offset: const Offset(10, 20),
          ),
        ],
      ).build(),
    );

    void expectOneAnnotation({
      required Offset globalPosition,
      required int value,
      required Offset localPosition,
    }) {
      expect(
        root.findAllAnnotations<int>(globalPosition).entries.toList(),
        _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
          AnnotationEntry<int>(annotation: value, localPosition: localPosition),
        ], maxCoordinateRelativeDiff: 0.005),
      );
    }

    expectOneAnnotation(
      globalPosition: const Offset(10.0, 19.7),
      value: 0,
      localPosition: const Offset(10.0, 19.7),
    );
    expectOneAnnotation(
      globalPosition: const Offset(10.1, 19.8),
      value: 1,
      localPosition: const Offset(10.0, 20.0),
    );

    expectOneAnnotation(
      globalPosition: const Offset(10.5, 62.8),
      value: 0,
      localPosition: const Offset(10.5, 62.8),
    );
    expectOneAnnotation(
      globalPosition: const Offset(10.6, 62.7),
      value: 1,
      localPosition: const Offset(10.1, 59.9),
    );

    expectOneAnnotation(
      globalPosition: const Offset(42.6, 40.8),
      value: 0,
      localPosition: const Offset(42.6, 40.8),
    );
    expectOneAnnotation(
      globalPosition: const Offset(42.5, 40.9),
      value: 1,
      localPosition: const Offset(39.9, 40.0),
    );

    expectOneAnnotation(
      globalPosition: const Offset(43.5, 63.5),
      value: 0,
      localPosition: const Offset(43.5, 63.5),
    );
    expectOneAnnotation(
      globalPosition: const Offset(43.4, 63.4),
      value: 1,
      localPosition: const Offset(39.9, 59.9),
    );
  });

  test('TransformLayer.findAllAnnotations skips when transform is irreversible', () {
    final Matrix4 transform = Matrix4.diagonal3Values(1, 0, 1);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        TransformLayer(transform: transform),
        children: <Object>[_TestAnnotatedLayer(1, opaque: true)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(Offset.zero).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: Offset.zero),
      ]),
    );
  });

  test('LeaderLayer.findAllAnnotations respects offset', () {
    const Offset insidePosition = Offset(-5, 5);
    const Offset outsidePosition = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        LeaderLayer(link: LayerLink(), offset: const Offset(-10, 0)),
        children: <Object>[_TestAnnotatedLayer(1, opaque: true, size: const Size(10, 10))],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(insidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1, localPosition: Offset(5, 5)),
      ]),
    );
    expect(
      root.findAllAnnotations<int>(outsidePosition).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 1000, localPosition: outsidePosition),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should append to the list '
      'and return the given opacity (false) during a successful hit', () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: position),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should append to the list '
      'and return the given opacity (true) during a successful hit', () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, opaque: true),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations has default opacity as false', () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: position),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should still check children and return '
      "children's opacity (false) during a failed hit", () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, opaque: true, size: Size.zero),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should still check children and return '
      "children's opacity (true) during a failed hit", () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, size: Size.zero),
        children: <Object>[_TestAnnotatedLayer(2, opaque: true)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
      ]),
    );
  });

  test("AnnotatedRegionLayer.findAllAnnotations should not add to children's opacity "
      'during a successful hit if it is not opaque', () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: position),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });

  test("AnnotatedRegionLayer.findAllAnnotations should add to children's opacity "
      'during a successful hit if it is opaque', () {
    const Offset position = Offset(5, 5);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, opaque: true),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false)],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should clip its annotation '
      'using size and offset (positive)', () {
    // The target position would have fallen outside if not for the offset.
    const Offset position = Offset(100, 100);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, size: const Size(20, 20), offset: const Offset(90, 90)),
        children: <Object>[
          _TestAnnotatedLayer(
            2,
            opaque: false,
            // Use this offset to make sure AnnotatedRegionLayer's offset
            // does not affect its children.
            offset: const Offset(20, 20),
            size: const Size(110, 110),
          ),
        ],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1, localPosition: Offset(10, 10)),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });

  test('AnnotatedRegionLayer.findAllAnnotations should clip its annotation '
      'using size and offset (negative)', () {
    // The target position would have fallen inside if not for the offset.
    const Offset position = Offset(10, 10);

    final Layer root = _withBackgroundAnnotation(
      1000,
      _Layers(
        AnnotatedRegionLayer<int>(1, size: const Size(20, 20), offset: const Offset(90, 90)),
        children: <Object>[_TestAnnotatedLayer(2, opaque: false, size: const Size(110, 110))],
      ).build(),
    );

    expect(
      root.findAllAnnotations<int>(position).entries.toList(),
      _equalToAnnotationResult<int>(<AnnotationEntry<int>>[
        const AnnotationEntry<int>(annotation: 2, localPosition: position),
        const AnnotationEntry<int>(annotation: 1000, localPosition: position),
      ]),
    );
  });
}

/// A [ContainerLayer] that contains a stack of layers: `layer` in the front,
/// and another layer annotated with `value` in the back.
///
/// It is a utility function that helps checking the opacity returned by
/// [Layer.findAnnotations].
Layer _withBackgroundAnnotation(int value, Layer layer) {
  return _Layers(
    ContainerLayer(),
    children: <Object>[_TestAnnotatedLayer(value, opaque: false), layer],
  ).build();
}

// A utility class that helps building a layer tree.
class _Layers {
  _Layers(this.root, {this.children});

  final ContainerLayer root;
  // Each element must be instance of Layer or _Layers.
  final List<Object>? children;
  bool _assigned = false;

  // Build the layer tree by calling each child's `build`, then append children
  // to [root]. Returns the root.
  Layer build() {
    assert(!_assigned);
    _assigned = true;
    if (children != null) {
      for (final Object child in children!) {
        late Layer layer;
        if (child is Layer) {
          layer = child;
        } else if (child is _Layers) {
          layer = child.build();
        } else {
          assert(false, 'Element of _Layers.children must be instance of Layer or _Layers');
        }
        root.append(layer);
      }
    }
    return root;
  }
}

// This layer's [findAnnotation] can be controlled by the given arguments.
class _TestAnnotatedLayer extends Layer {
  _TestAnnotatedLayer(this.value, {required this.opaque, this.offset = Offset.zero, this.size});

  // The value added to result in [findAnnotations] during a successful hit.
  final int value;

  // The return value of [findAnnotations] during a successful hit.
  final bool opaque;

  /// The [offset] is optionally used to translate the clip region for the
  /// hit-testing of [find] by [offset].
  ///
  /// If not provided, offset defaults to [Offset.zero].
  ///
  /// Ignored if [size] is not set.
  final Offset offset;

  /// The [size] is optionally used to clip the hit-testing of [find].
  ///
  /// If not provided, all offsets are considered to be contained within this
  /// layer, unless an ancestor layer applies a clip.
  ///
  /// If [offset] is set, then the offset is applied to the size region before
  /// hit testing in [find].
  final Size? size;

  @override
  EngineLayer? addToScene(SceneBuilder builder) {
    return null;
  }

  // This implementation is hit when the type is `int` and position is within
  // [offset] & [size]. If it is hit, it adds [value] to result and returns
  // [opaque]; otherwise it directly returns false.
  @override
  bool findAnnotations<S extends Object>(
    AnnotationResult<S> result,
    Offset localPosition, {
    required bool onlyFirst,
  }) {
    if (S != int) {
      return false;
    }
    if (size != null && !(offset & size!).contains(localPosition)) {
      return false;
    }
    final Object untypedValue = value;
    final S typedValue = untypedValue as S;
    result.add(AnnotationEntry<S>(annotation: typedValue, localPosition: localPosition));
    return opaque;
  }
}

bool _almostEqual(double a, double b, double maxRelativeDiff) {
  assert(maxRelativeDiff >= 0);
  assert(maxRelativeDiff < 1);
  return (a - b).abs() <= a.abs() * maxRelativeDiff;
}

Matcher _equalToAnnotationResult<T>(
  List<AnnotationEntry<int>> list, {
  double maxCoordinateRelativeDiff = 0,
}) {
  return pairwiseCompare<AnnotationEntry<int>, AnnotationEntry<int>>(list, (
    AnnotationEntry<int> a,
    AnnotationEntry<int> b,
  ) {
    return a.annotation == b.annotation &&
        _almostEqual(a.localPosition.dx, b.localPosition.dx, maxCoordinateRelativeDiff) &&
        _almostEqual(a.localPosition.dy, b.localPosition.dy, maxCoordinateRelativeDiff);
  }, 'equal to');
}
