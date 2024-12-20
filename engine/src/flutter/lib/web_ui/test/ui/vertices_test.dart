// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('Vertices', () {
    setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

    test('can be constructed, drawn, and disposed of', () {
      final ui.Vertices vertices = _testVertices();
      expect(vertices.debugDisposed, isFalse);

      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder, const ui.Rect.fromLTRB(0, 0, 100, 100));
      canvas.drawVertices(vertices, ui.BlendMode.srcOver, ui.Paint());
      vertices.dispose();
      expect(vertices.debugDisposed, isTrue);
    });
  });

  test('Vertices are not anti-aliased by default', () async {
    const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 500);
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder, region);
    final ui.Vertices vertices = ui.Vertices.raw(
      ui.VertexMode.triangles,
      Float32List.fromList(_circularVertices),
      indices: Uint16List.fromList(_circularVertexIndices),
    );
    canvas.scale(3.0, 3.0);
    canvas.drawVertices(
      vertices,
      ui.BlendMode.srcOver,
      ui.Paint()..color = const ui.Color(0xFFFF0000),
    );

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_vertices_antialiased.png', region: region);
  }, skip: isHtml); // https://github.com/flutter/flutter/issues/127454
}

ui.Vertices _testVertices() {
  return ui.Vertices(
    ui.VertexMode.triangles,
    const <ui.Offset>[ui.Offset.zero, ui.Offset(10, 10), ui.Offset(0, 20)],
    textureCoordinates: const <ui.Offset>[ui.Offset.zero, ui.Offset(10, 10), ui.Offset(0, 20)],
    colors: const <ui.Color>[
      ui.Color.fromRGBO(255, 0, 0, 1.0),
      ui.Color.fromRGBO(0, 255, 0, 1.0),
      ui.Color.fromRGBO(0, 0, 255, 1.0),
    ],
    indices: <int>[0, 1, 2],
  );
}

const List<double> _circularVertices = <double>[
  14.530723571777344,
  18.18381690979004,
  14.530723571777344,
  18.624177932739258,
  14.538220405578613,
  18.40399742126465,
  14.445072174072266,
  17.765277862548828,
  14.445072174072266,
  19.04271697998047,
  14.282388687133789,
  17.38067626953125,
  14.282388687133789,
  19.427318572998047,
  14.051292419433594,
  17.038639068603516,
  14.051292419433594,
  19.76935577392578,
  13.76040267944336,
  16.747783660888672,
  13.76040267944336,
  20.06021499633789,
  13.418340682983398,
  16.5167236328125,
  13.418340682983398,
  20.291275024414062,
  13.033727645874023,
  20.45391845703125,
  13.03372573852539,
  16.354080200195312,
  12.615180015563965,
  20.539527893066406,
  12.6151762008667,
  16.268470764160156,
  12.395000457763672,
  20.547000885009766,
  12.394994735717773,
  16.260997772216797,
  12.394994735717773,
  20.546998977661133,
  12.174847602844238,
  16.268516540527344,
  12.174847602844238,
  20.539480209350586,
  11.75637149810791,
  16.354188919067383,
  11.75637149810791,
  20.45380973815918,
  11.371832847595215,
  16.516868591308594,
  11.371832847595215,
  20.291126251220703,
  11.029844284057617,
  16.747943878173828,
  11.029844284057617,
  20.06005096435547,
  10.739023208618164,
  17.038795471191406,
  10.739023208618164,
  19.76919937133789,
  10.507984161376953,
  17.380809783935547,
  10.507984161376953,
  19.427188873291016,
  10.345342636108398,
  17.765365600585938,
  10.345342636108398,
  19.042633056640625,
  10.259716033935547,
  18.183849334716797,
  10.259716033935547,
  18.6241455078125,
  10.25222110748291,
  18.40399742126465,
];

const List<int> _circularVertexIndices = <int>[
  0,
  1,
  2,
  3,
  1,
  0,
  3,
  4,
  1,
  5,
  4,
  3,
  5,
  6,
  4,
  7,
  6,
  5,
  7,
  8,
  6,
  9,
  8,
  7,
  9,
  10,
  8,
  11,
  10,
  9,
  11,
  12,
  10,
  11,
  13,
  12,
  14,
  13,
  11,
  14,
  15,
  13,
  16,
  15,
  14,
  16,
  17,
  15,
  18,
  17,
  16,
  18,
  19,
  17,
  20,
  19,
  18,
  20,
  21,
  19,
  22,
  21,
  20,
  22,
  23,
  21,
  24,
  23,
  22,
  24,
  25,
  23,
  26,
  25,
  24,
  26,
  27,
  25,
  28,
  27,
  26,
  28,
  29,
  27,
  30,
  29,
  28,
  30,
  31,
  29,
  32,
  31,
  30,
  32,
  33,
  31,
  34,
  33,
  32,
  34,
  35,
  33,
  35,
  34,
  36,
];
