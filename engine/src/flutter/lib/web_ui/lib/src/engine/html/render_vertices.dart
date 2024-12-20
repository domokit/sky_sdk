// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../browser_detection.dart';
import '../dom.dart';
import '../safe_browser_api.dart';
import '../util.dart';
import '../vector_math.dart';
import 'painting.dart';
import 'shaders/image_shader.dart';
import 'shaders/normalized_gradient.dart';
import 'shaders/shader_builder.dart';
import 'shaders/vertex_shaders.dart';

GlRenderer? glRenderer;

class SurfaceVertices implements ui.Vertices {
  SurfaceVertices(
    this.mode,
    List<ui.Offset> positions, {
    List<ui.Color>? colors,
    List<int>? indices,
  }) : colors = colors != null ? _int32ListFromColors(colors) : null,
       indices = indices != null ? Uint16List.fromList(indices) : null,
       positions = offsetListToFloat32List(positions) {
    initWebGl();
  }

  SurfaceVertices.raw(this.mode, this.positions, {this.colors, this.indices}) {
    initWebGl();
  }

  final ui.VertexMode mode;
  final Float32List positions;
  final Int32List? colors;
  final Uint16List? indices;

  static Int32List _int32ListFromColors(List<ui.Color> colors) {
    final Int32List list = Int32List(colors.length);
    final int len = colors.length;
    for (int i = 0; i < len; i++) {
      list[i] = colors[i].value;
    }
    return list;
  }

  bool _disposed = false;

  @override
  void dispose() {
    _disposed = true;
  }

  @override
  bool get debugDisposed {
    bool? result;
    assert(() {
      result = _disposed;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw StateError('Vertices.debugDisposed is only available when asserts are enabled.');
  }
}

/// Lazily initializes web gl.
///
/// Used to treeshake WebGlRenderer when user doesn't create Vertices object
/// to use the API.
void initWebGl() {
  glRenderer ??= _WebGlRenderer();
}

abstract class GlRenderer {
  void drawVertices(
    DomCanvasRenderingContext2D? context,
    int canvasWidthInPixels,
    int canvasHeightInPixels,
    Matrix4 transform,
    SurfaceVertices vertices,
    ui.BlendMode blendMode,
    SurfacePaintData paint,
  );

  Object? drawRect(
    ui.Rect targetRect,
    GlContext gl,
    GlProgram glProgram,
    NormalizedGradient gradient,
    int widthInPixels,
    int heightInPixels,
  );

  String drawRectToImageUrl(
    ui.Rect targetRect,
    GlContext gl,
    GlProgram glProgram,
    NormalizedGradient gradient,
    int widthInPixels,
    int heightInPixels,
  );

  void drawHairline(DomCanvasRenderingContext2D? ctx, Float32List positions);
}

/// Treeshakeable backend for rendering webgl on canvas.
///
/// This class gets instantiated on demand by Vertices constructor. For apps
/// that don't use Vertices WebGlRenderer will be removed from release binary.
class _WebGlRenderer implements GlRenderer {
  @override
  void drawVertices(
    DomCanvasRenderingContext2D? context,
    int canvasWidthInPixels,
    int canvasHeightInPixels,
    Matrix4 transform,
    SurfaceVertices vertices,
    ui.BlendMode blendMode,
    SurfacePaintData paint,
  ) {
    // Compute bounds of vertices.
    final Float32List positions = vertices.positions;
    final ui.Rect bounds = _computeVerticesBounds(positions, transform);
    final double minValueX = bounds.left;
    final double minValueY = bounds.top;
    final double maxValueX = bounds.right;
    final double maxValueY = bounds.bottom;
    double offsetX = 0;
    double offsetY = 0;
    int widthInPixels = canvasWidthInPixels;
    int heightInPixels = canvasHeightInPixels;
    // If vertices fall outside the bitmap area, cull.
    if (maxValueX < 0 || maxValueY < 0) {
      return;
    }
    if (minValueX > widthInPixels || minValueY > heightInPixels) {
      return;
    }
    // If Vertices are is smaller than hosting canvas, allocate minimal
    // offscreen canvas to reduce readPixels data size.
    if ((maxValueX - minValueX) < widthInPixels && (maxValueY - minValueY) < heightInPixels) {
      widthInPixels = maxValueX.ceil() - minValueX.floor();
      heightInPixels = maxValueY.ceil() - minValueY.floor();
      offsetX = minValueX.floor().toDouble();
      offsetY = minValueY.floor().toDouble();
    }
    if (widthInPixels == 0 || heightInPixels == 0) {
      return;
    }

    final bool isWebGl2 = webGLVersion == WebGLVersion.webgl2;

    final EngineImageShader? imageShader =
        paint.shader == null ? null : paint.shader! as EngineImageShader;

    final String vertexShader =
        imageShader == null
            ? VertexShaders.writeBaseVertexShader()
            : VertexShaders.writeTextureVertexShader();
    final String fragmentShader =
        imageShader == null
            ? _writeVerticesFragmentShader()
            : FragmentShaders.writeTextureFragmentShader(
              isWebGl2,
              imageShader.tileModeX,
              imageShader.tileModeY,
            );

    final GlContext gl = GlContextCache.createGlContext(widthInPixels, heightInPixels)!;

    final GlProgram glProgram = gl.cacheProgram(vertexShader, fragmentShader);
    gl.useProgram(glProgram);

    final Object positionAttributeLocation = gl.getAttributeLocation(glProgram.program, 'position');

    setupVertexTransforms(
      gl,
      glProgram,
      offsetX,
      offsetY,
      widthInPixels.toDouble(),
      heightInPixels.toDouble(),
      transform,
    );

    if (imageShader != null) {
      /// To map from vertex position to texture coordinate in 0..1 range,
      /// we setup scalar to be used in vertex shader.
      setupTextureTransform(
        gl,
        glProgram,
        0.0,
        0.0,
        1.0 / imageShader.image.width.toDouble(),
        1.0 / imageShader.image.height.toDouble(),
      );
    }

    // Setup geometry.
    //
    // Create buffer for vertex coordinates.
    final Object positionsBuffer = gl.createBuffer()!;

    Object? vao;
    if (imageShader != null) {
      if (isWebGl2) {
        // Create a vertex array object.
        vao = gl.createVertexArray();
        // Set vertex array object as active one.
        gl.bindVertexArray(vao!);
      }
    }
    // Turn on position attribute.
    gl.enableVertexAttribArray(positionAttributeLocation);
    // Bind buffer as position buffer and transfer data.
    gl.bindArrayBuffer(positionsBuffer);
    bufferVertexData(gl, positions, 1.0);

    // Setup data format for attribute.
    vertexAttribPointerGlContext(
      gl.glContext,
      positionAttributeLocation,
      2,
      gl.kFloat,
      false,
      0,
      0,
    );

    final int vertexCount = positions.length ~/ 2;
    Object? texture;

    if (imageShader == null) {
      // Setup color buffer.
      final Object? colorsBuffer = gl.createBuffer();
      gl.bindArrayBuffer(colorsBuffer);

      // Buffer kBGRA_8888.
      if (vertices.colors == null) {
        final Uint32List vertexColors = Uint32List(vertexCount);
        for (int i = 0; i < vertexCount; i++) {
          vertexColors[i] = paint.color;
        }
        gl.bufferData(vertexColors, gl.kStaticDraw);
      } else {
        gl.bufferData(vertices.colors, gl.kStaticDraw);
      }
      final Object colorLoc = gl.getAttributeLocation(glProgram.program, 'color');
      vertexAttribPointerGlContext(gl.glContext, colorLoc, 4, gl.kUnsignedByte, true, 0, 0);
      gl.enableVertexAttribArray(colorLoc);
    } else {
      // Copy image it to the texture.
      texture = gl.createTexture();
      // Texture units are a global array of references to the textures.
      // By setting activeTexture, we associate the bound texture to a unit.
      // Every time we call a texture function such as texImage2D with a target
      // like TEXTURE_2D, it looks up texture by using the currently active
      // unit.
      // In our case we have a single texture unit 0.
      gl.activeTexture(gl.kTexture0);
      gl.bindTexture(gl.kTexture2D, texture);

      gl.texImage2D(
        gl.kTexture2D,
        0,
        gl.kRGBA,
        gl.kRGBA,
        gl.kUnsignedByte,
        imageShader.image.imgElement,
      );

      if (isWebGl2) {
        // Texture REPEAT and MIRROR is only supported in WebGL 2, for
        // WebGL 1.0 we let shader compute correct uv coordinates.
        gl.texParameteri(
          gl.kTexture2D,
          gl.kTextureWrapS,
          tileModeToGlWrapping(gl, imageShader.tileModeX),
        );

        gl.texParameteri(
          gl.kTexture2D,
          gl.kTextureWrapT,
          tileModeToGlWrapping(gl, imageShader.tileModeY),
        );

        // Mipmapping saves your texture in different resolutions
        // so the graphics card can choose which resolution is optimal
        // without artifacts.
        gl.generateMipmap(gl.kTexture2D);
      } else {
        // For webgl1, if a texture is not mipmap complete, then the return
        // value of a texel fetch is (0, 0, 0, 1), so we have to set
        // minifying function to filter.
        // See https://www.khronos.org/registry/webgl/specs/1.0.0/#5.13.8.
        gl.texParameteri(gl.kTexture2D, gl.kTextureWrapS, gl.kClampToEdge);
        gl.texParameteri(gl.kTexture2D, gl.kTextureWrapT, gl.kClampToEdge);
        gl.texParameteri(gl.kTexture2D, gl.kTextureMinFilter, gl.kLinear);
      }
    }

    // Finally render triangles.
    gl.clear();

    final Uint16List? indices = vertices.indices;
    if (indices == null) {
      gl.drawTriangles(vertexCount, vertices.mode);
    } else {
      /// If indices are specified to use shared vertices to reduce vertex
      /// data transfer, use drawElements to map from vertex indices to
      /// triangles.
      final Object? indexBuffer = gl.createBuffer();
      gl.bindElementArrayBuffer(indexBuffer);
      gl.bufferElementData(indices, gl.kStaticDraw);
      gl.drawElements(gl.kTriangles, indices.length, gl.kUnsignedShort);
    }

    if (vao != null) {
      gl.unbindVertexArray();
    }

    context!.save();
    context.resetTransform();
    gl.drawImage(context, offsetX, offsetY);
    context.restore();
  }

  /// Renders a rectangle using given program into an image resource.
  ///
  /// Browsers that support OffscreenCanvas and the transferToImageBitmap api
  /// will return ImageBitmap, otherwise will return CanvasElement.
  @override
  Object? drawRect(
    ui.Rect targetRect,
    GlContext gl,
    GlProgram glProgram,
    NormalizedGradient gradient,
    int widthInPixels,
    int heightInPixels,
  ) {
    drawRectToGl(targetRect, gl, glProgram, gradient, widthInPixels, heightInPixels);
    final Object? image = gl.readPatternData(gradient.isOpaque);
    gl.bindArrayBuffer(null);
    gl.bindElementArrayBuffer(null);
    return image;
  }

  /// Renders a rectangle using given program into an image resource and returns
  /// url.
  @override
  String drawRectToImageUrl(
    ui.Rect targetRect,
    GlContext gl,
    GlProgram glProgram,
    NormalizedGradient gradient,
    int widthInPixels,
    int heightInPixels,
  ) {
    drawRectToGl(targetRect, gl, glProgram, gradient, widthInPixels, heightInPixels);
    final String imageUrl = gl.toImageUrl();
    // Cleanup buffers.
    gl.bindArrayBuffer(null);
    gl.bindElementArrayBuffer(null);
    return imageUrl;
  }

  /// Renders a rectangle using given program into [GlContext].
  ///
  /// Caller has to cleanup gl array and element array buffers.
  void drawRectToGl(
    ui.Rect targetRect,
    GlContext gl,
    GlProgram glProgram,
    NormalizedGradient gradient,
    int widthInPixels,
    int heightInPixels,
  ) {
    // Setup rectangle coordinates.
    final double left = targetRect.left;
    final double top = targetRect.top;
    final double right = targetRect.right;
    final double bottom = targetRect.bottom;
    // Form 2 triangles for rectangle.
    final Float32List vertices = Float32List(8);
    vertices[0] = left;
    vertices[1] = top;
    vertices[2] = right;
    vertices[3] = top;
    vertices[4] = right;
    vertices[5] = bottom;
    vertices[6] = left;
    vertices[7] = bottom;

    final Object transformUniform = gl.getUniformLocation(glProgram.program, 'u_ctransform');
    gl.setUniformMatrix4fv(transformUniform, false, Matrix4.identity().storage);

    // Set uniform to scale 0..width/height pixels coordinates to -1..1
    // clipspace range and flip the Y axis.
    final Object resolution = gl.getUniformLocation(glProgram.program, 'u_scale');
    gl.setUniform4f(
      resolution,
      2.0 / widthInPixels.toDouble(),
      -2.0 / heightInPixels.toDouble(),
      1,
      1,
    );
    final Object shift = gl.getUniformLocation(glProgram.program, 'u_shift');
    gl.setUniform4f(shift, -1, 1, 0, 0);

    // Setup geometry.
    final Object positionsBuffer = gl.createBuffer()!;
    gl.bindArrayBuffer(positionsBuffer);
    gl.bufferData(vertices, gl.kStaticDraw);
    // Point an attribute to the currently bound vertex buffer object.
    vertexAttribPointerGlContext(gl.glContext, 0, 2, gl.kFloat, false, 0, 0);
    gl.enableVertexAttribArray(0);

    // Setup color buffer.
    final Object? colorsBuffer = gl.createBuffer();
    gl.bindArrayBuffer(colorsBuffer);
    // Buffer kBGRA_8888.
    final Int32List colors = Int32List.fromList(<int>[
      0xFF00FF00,
      0xFF0000FF,
      0xFFFFFF00,
      0xFF00FFFF,
    ]);
    gl.bufferData(colors, gl.kStaticDraw);
    vertexAttribPointerGlContext(gl.glContext, 1, 4, gl.kUnsignedByte, true, 0, 0);
    gl.enableVertexAttribArray(1);

    final Object? indexBuffer = gl.createBuffer();
    gl.bindElementArrayBuffer(indexBuffer);
    gl.bufferElementData(VertexShaders.vertexIndicesForRect, gl.kStaticDraw);

    if (gl.containsUniform(glProgram.program, 'u_resolution')) {
      final Object uRes = gl.getUniformLocation(glProgram.program, 'u_resolution');
      gl.setUniform2f(uRes, widthInPixels.toDouble(), heightInPixels.toDouble());
    }

    gl.clear();
    gl.viewport(0, 0, widthInPixels.toDouble(), heightInPixels.toDouble());

    gl.drawElements(gl.kTriangles, VertexShaders.vertexIndicesForRect.length, gl.kUnsignedShort);
  }

  /// This fragment shader enables Int32List of colors to be passed directly
  /// to gl context buffer for rendering by decoding RGBA8888.
  ///     #version 300 es
  ///     precision mediump float;
  ///     in vec4 vColor;
  ///     out vec4 fragColor;
  ///     void main() {
  ///       fragColor = vColor;
  ///     }
  String _writeVerticesFragmentShader() {
    final ShaderBuilder builder = ShaderBuilder.fragment(webGLVersion);
    builder.floatPrecision = ShaderPrecision.kMedium;
    builder.addIn(ShaderType.kVec4, name: 'v_color');
    final ShaderMethod method = builder.addMethod('main');
    method.addStatement('${builder.fragmentColor.name} = v_color;');
    return builder.build();
  }

  @override
  void drawHairline(DomCanvasRenderingContext2D? ctx, Float32List positions) {
    final int pointCount = positions.length ~/ 2;
    ctx!.lineWidth = 1.0;
    ctx.beginPath();
    final int len = pointCount * 2;
    for (int i = 0; i < len;) {
      for (int triangleVertexIndex = 0; triangleVertexIndex < 3; triangleVertexIndex++, i += 2) {
        final double dx = positions[i];
        final double dy = positions[i + 1];
        switch (triangleVertexIndex) {
          case 0:
            ctx.moveTo(dx, dy);
          case 1:
            ctx.lineTo(dx, dy);
          case 2:
            ctx.lineTo(dx, dy);
            ctx.closePath();
            ctx.stroke();
        }
      }
    }
  }
}

ui.Rect _computeVerticesBounds(Float32List positions, Matrix4 transform) {
  double minValueX, maxValueX, minValueY, maxValueY;
  minValueX = maxValueX = positions[0];
  minValueY = maxValueY = positions[1];
  final int len = positions.length;
  for (int i = 2; i < len; i += 2) {
    final double x = positions[i];
    final double y = positions[i + 1];
    if (x.isNaN || y.isNaN) {
      // Follows skia implementation that sets bounds to empty
      // and aborts.
      return ui.Rect.zero;
    }
    minValueX = math.min(minValueX, x);
    maxValueX = math.max(maxValueX, x);
    minValueY = math.min(minValueY, y);
    maxValueY = math.max(maxValueY, y);
  }
  return _transformBounds(transform, minValueX, minValueY, maxValueX, maxValueY);
}

ui.Rect _transformBounds(Matrix4 transform, double left, double top, double right, double bottom) {
  final Float32List storage = transform.storage;
  final double m0 = storage[0];
  final double m1 = storage[1];
  final double m4 = storage[4];
  final double m5 = storage[5];
  final double m12 = storage[12];
  final double m13 = storage[13];
  final double x0 = (m0 * left) + (m4 * top) + m12;
  final double y0 = (m1 * left) + (m5 * top) + m13;
  final double x1 = (m0 * right) + (m4 * top) + m12;
  final double y1 = (m1 * right) + (m5 * top) + m13;
  final double x2 = (m0 * right) + (m4 * bottom) + m12;
  final double y2 = (m1 * right) + (m5 * bottom) + m13;
  final double x3 = (m0 * left) + (m4 * bottom) + m12;
  final double y3 = (m1 * left) + (m5 * bottom) + m13;
  return ui.Rect.fromLTRB(
    math.min(x0, math.min(x1, math.min(x2, x3))),
    math.min(y0, math.min(y1, math.min(y2, y3))),
    math.max(x0, math.max(x1, math.max(x2, x3))),
    math.max(y0, math.max(y1, math.max(y2, y3))),
  );
}

/// Converts from [VertexMode] triangleFan and triangleStrip to triangles.
Float32List convertVertexPositions(ui.VertexMode mode, Float32List positions) {
  assert(mode != ui.VertexMode.triangles);
  if (mode == ui.VertexMode.triangleFan) {
    final int coordinateCount = positions.length ~/ 2;
    final int triangleCount = coordinateCount - 2;
    final Float32List triangleList = Float32List(triangleCount * 3 * 2);
    final double centerX = positions[0];
    final double centerY = positions[1];
    int destIndex = 0;
    int positionIndex = 2;
    for (
      int triangleIndex = 0;
      triangleIndex < triangleCount;
      triangleIndex++, positionIndex += 2
    ) {
      triangleList[destIndex++] = centerX;
      triangleList[destIndex++] = centerY;
      triangleList[destIndex++] = positions[positionIndex];
      triangleList[destIndex++] = positions[positionIndex + 1];
      triangleList[destIndex++] = positions[positionIndex + 2];
      triangleList[destIndex++] = positions[positionIndex + 3];
    }
    return triangleList;
  } else {
    assert(mode == ui.VertexMode.triangleStrip);
    // Set of connected triangles. Each triangle shares 2 last vertices.
    final int vertexCount = positions.length ~/ 2;
    final int triangleCount = vertexCount - 2;
    double x0 = positions[0];
    double y0 = positions[1];
    double x1 = positions[2];
    double y1 = positions[3];
    final Float32List triangleList = Float32List(triangleCount * 3 * 2);
    int destIndex = 0;
    for (int i = 0, positionIndex = 4; i < triangleCount; i++) {
      final double x2 = positions[positionIndex++];
      final double y2 = positions[positionIndex++];
      triangleList[destIndex++] = x0;
      triangleList[destIndex++] = y0;
      triangleList[destIndex++] = x1;
      triangleList[destIndex++] = y1;
      triangleList[destIndex++] = x2;
      triangleList[destIndex++] = y2;
      x0 = x1;
      y0 = y1;
      x1 = x2;
      y1 = y2;
    }
    return triangleList;
  }
}
