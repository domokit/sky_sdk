// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_util' as js_util;
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide ImageShader, TextStyle;

import '../../common/test_initialization.dart';
import '../screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);

  setUpUnitTests(setUpTestViewDimensions: false);

  setUp(() {
    GlContextCache.dispose();
    glRenderer = null;
  });

  Future<void> testVertices(
    String fileName,
    Vertices vertices,
    BlendMode blendMode,
    Paint paint,
  ) async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.drawVertices(vertices as SurfaceVertices, blendMode, paint as SurfacePaint);
    await canvasScreenshot(rc, fileName, canvasRect: screenRect);
  }

  test('Should draw green hairline triangles when colors array is null.', () async {
    final Vertices vertices = Vertices.raw(
      VertexMode.triangles,
      Float32List.fromList(<double>[
        20.0,
        20.0,
        220.0,
        10.0,
        110.0,
        220.0,
        220.0,
        320.0,
        20.0,
        310.0,
        200.0,
        420.0,
      ]),
    );
    await testVertices(
      'draw_vertices_hairline_triangle',
      vertices,
      BlendMode.srcOver,
      Paint()..color = const Color.fromARGB(255, 0, 128, 0),
    );
  });

  test('Should draw black hairline triangles when colors array is null'
      ' and Paint() has no color.', () async {
    // ignore: unused_local_variable
    final Int32List colors = Int32List.fromList(<int>[
      0xFFFF0000,
      0xFF00FF00,
      0xFF0000FF,
      0xFFFF0000,
      0xFF00FF00,
      0xFF0000FF,
      0xFFFF0000,
      0xFF00FF00,
      0xFF0000FF,
      0xFFFF0000,
      0xFF00FF00,
      0xFF0000FF,
    ]);
    final Vertices vertices = Vertices.raw(
      VertexMode.triangles,
      Float32List.fromList(<double>[
        20.0,
        20.0,
        220.0,
        10.0,
        110.0,
        220.0,
        220.0,
        320.0,
        20.0,
        310.0,
        200.0,
        420.0,
      ]),
    );
    await testVertices(
      'draw_vertices_hairline_triangle_black',
      vertices,
      BlendMode.srcOver,
      Paint(),
    );
  });

  /// Regression test for https://github.com/flutter/flutter/issues/71442.
  test(
    'Should draw filled triangles when colors array is null'
    ' and Paint() has color.',
    () async {
      // ignore: unused_local_variable
      final Int32List colors = Int32List.fromList(<int>[
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
      ]);
      final Vertices vertices = Vertices.raw(
        VertexMode.triangles,
        Float32List.fromList(<double>[
          20.0,
          20.0,
          220.0,
          10.0,
          110.0,
          220.0,
          220.0,
          320.0,
          20.0,
          310.0,
          200.0,
          420.0,
        ]),
      );
      await testVertices(
        'draw_vertices_triangle_green_filled',
        vertices,
        BlendMode.srcOver,
        Paint()
          ..style = PaintingStyle.fill
          ..color = const Color(0xFF00FF00),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test('Should draw hairline triangleFan.', () async {
    final Vertices vertices = Vertices.raw(
      VertexMode.triangleFan,
      Float32List.fromList(<double>[
        150.0,
        150.0,
        20.0,
        10.0,
        80.0,
        20.0,
        220.0,
        15.0,
        280.0,
        30.0,
        300.0,
        420.0,
      ]),
    );

    await testVertices(
      'draw_vertices_hairline_triangle_fan',
      vertices,
      BlendMode.srcOver,
      Paint()..color = const Color.fromARGB(255, 0, 128, 0),
    );
  });

  test('Should draw hairline triangleStrip.', () async {
    final Vertices vertices = Vertices.raw(
      VertexMode.triangleStrip,
      Float32List.fromList(<double>[
        20.0,
        20.0,
        220.0,
        10.0,
        110.0,
        220.0,
        220.0,
        320.0,
        20.0,
        310.0,
        200.0,
        420.0,
      ]),
    );
    await testVertices(
      'draw_vertices_hairline_triangle_strip',
      vertices,
      BlendMode.srcOver,
      Paint()..color = const Color.fromARGB(255, 0, 128, 0),
    );
  });

  test(
    'Should draw triangles with colors.',
    () async {
      final Int32List colors = Int32List.fromList(<int>[
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
      ]);
      final Vertices vertices = Vertices.raw(
        VertexMode.triangles,
        Float32List.fromList(<double>[
          150.0,
          150.0,
          20.0,
          10.0,
          80.0,
          20.0,
          220.0,
          15.0,
          280.0,
          30.0,
          300.0,
          420.0,
        ]),
        colors: colors,
      );

      await testVertices(
        'draw_vertices_triangles',
        vertices,
        BlendMode.srcOver,
        Paint()..color = const Color.fromARGB(255, 0, 128, 0),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw triangles with colors and indices.',
    () async {
      final Int32List colors = Int32List.fromList(<int>[
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF0000FF,
      ]);
      final Uint16List indices = Uint16List.fromList(<int>[0, 1, 2, 3, 4, 0]);

      final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

      final Vertices vertices = Vertices.raw(
        VertexMode.triangles,
        Float32List.fromList(<double>[
          210.0,
          150.0,
          30.0,
          110.0,
          80.0,
          30.0,
          220.0,
          15.0,
          280.0,
          30.0,
        ]),
        colors: colors,
        indices: indices,
      );

      rc.drawVertices(vertices as SurfaceVertices, BlendMode.srcOver, SurfacePaint());

      await canvasScreenshot(rc, 'draw_vertices_triangles_indexed', canvasRect: screenRect);
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw triangleFan with colors.',
    () async {
      final Int32List colors = Int32List.fromList(<int>[
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
      ]);
      final Vertices vertices = Vertices.raw(
        VertexMode.triangleFan,
        Float32List.fromList(<double>[
          150.0,
          150.0,
          20.0,
          10.0,
          80.0,
          20.0,
          220.0,
          15.0,
          280.0,
          30.0,
          300.0,
          420.0,
        ]),
        colors: colors,
      );

      await testVertices(
        'draw_vertices_triangle_fan',
        vertices,
        BlendMode.srcOver,
        Paint()..color = const Color.fromARGB(255, 0, 128, 0),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw triangleStrip with colors.',
    () async {
      final Int32List colors = Int32List.fromList(<int>[
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
        0xFFFF0000,
        0xFF00FF00,
        0xFF0000FF,
      ]);
      final Vertices vertices = Vertices.raw(
        VertexMode.triangleStrip,
        Float32List.fromList(<double>[
          20.0,
          20.0,
          220.0,
          10.0,
          110.0,
          220.0,
          220.0,
          320.0,
          20.0,
          310.0,
          200.0,
          420.0,
        ]),
        colors: colors,
      );
      await testVertices(
        'draw_vertices_triangle_strip',
        vertices,
        BlendMode.srcOver,
        Paint()..color = const Color.fromARGB(255, 0, 128, 0),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  Future<void> testTexture(TileMode tileMode, String filename) async {
    final Uint16List indices = Uint16List.fromList(<int>[0, 1, 2, 3, 4, 0]);

    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

    final Vertices vertices = Vertices.raw(
      VertexMode.triangles,
      Float32List.fromList(<double>[210.0, 150.0, 0.0, 0.0, 80.0, 30.0, 220.0, 15.0, 280.0, 30.0]),
      indices: indices,
    );

    final Float32List matrix4 = Matrix4.identity().storage;

    final HtmlImage img = await createTestImage();
    final SurfacePaint paint = SurfacePaint();

    final EngineImageShader imgShader = EngineImageShader(
      img,
      tileMode,
      tileMode,
      Float64List.fromList(matrix4),
      FilterQuality.high,
    );

    paint.shader = imgShader;

    rc.drawVertices(vertices as SurfaceVertices, BlendMode.srcOver, paint);
    await canvasScreenshot(rc, filename, canvasRect: screenRect);

    expect(imgShader.debugDisposed, false);
    imgShader.dispose();
    expect(imgShader.debugDisposed, true);
  }

  test(
    'Should draw triangle with texture and indices',
    () async {
      await testTexture(TileMode.clamp, 'draw_vertices_texture');
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw triangle with texture and indices',
    () async {
      await testTexture(TileMode.mirror, 'draw_vertices_texture_mirror');
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw triangle with texture and indices',
    () async {
      await testTexture(TileMode.repeated, 'draw_vertices_texture_repeated');
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );
}

Future<HtmlImage> createTestImage({int width = 50, int height = 40}) {
  final DomCanvasElement canvas = createDomCanvasElement(width: width, height: height);
  final DomCanvasRenderingContext2D ctx = canvas.context2D;
  ctx.fillStyle = '#E04040';
  ctx.fillRect(0, 0, width / 3, height);
  ctx.fill();
  ctx.fillStyle = '#40E080';
  ctx.fillRect(width / 3, 0, width / 3, height);
  ctx.fill();
  ctx.fillStyle = '#2040E0';
  ctx.fillRect(2 * width / 3, 0, width / 3, height);
  ctx.fill();
  final DomHTMLImageElement imageElement = createDomHTMLImageElement();
  final Completer<HtmlImage> completer = Completer<HtmlImage>();
  imageElement.addEventListener(
    'load',
    createDomEventListener((DomEvent event) {
      completer.complete(HtmlImage(imageElement, width, height));
    }),
  );
  imageElement.src = js_util.callMethod<String>(canvas, 'toDataURL', <dynamic>[]);
  return completer.future;
}
