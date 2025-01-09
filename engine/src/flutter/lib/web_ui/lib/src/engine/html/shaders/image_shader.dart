// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../../browser_detection.dart';
import '../../dom.dart';
import '../../safe_browser_api.dart';
import '../../vector_math.dart';
import '../image.dart';
import '../render_vertices.dart';
import 'vertex_shaders.dart';

class EngineImageShader implements ui.ImageShader {
  EngineImageShader(
    ui.Image image,
    this.tileModeX,
    this.tileModeY,
    Float64List matrix4,
    this.filterQuality,
  ) : image = image as HtmlImage,
      matrix4 = Float32List.fromList(matrix4);

  final ui.TileMode tileModeX;
  final ui.TileMode tileModeY;
  final Float32List matrix4;
  final ui.FilterQuality? filterQuality;
  final HtmlImage image;

  /// Whether fill pattern requires transform to shift tiling offset.
  bool requiresTileOffset = false;

  Object createPaintStyle(
    DomCanvasRenderingContext2D context,
    ui.Rect? shaderBounds,
    double density,
  ) {
    /// Creates a canvas rendering context pattern based on image and tile modes.
    final ui.TileMode tileX = tileModeX;
    final ui.TileMode tileY = tileModeY;
    if (tileX != ui.TileMode.clamp && tileY != ui.TileMode.clamp) {
      return context.createPattern(
        _resolveTiledImageSource(image, tileX, tileY)!,
        _tileModeToHtmlRepeatAttribute(tileX, tileY),
      )!;
    } else {
      initWebGl();
      return _createGlShader(context, shaderBounds!, density);
    }
  }

  /// Converts tilemode to CSS repeat attribute.
  ///
  /// CSS and Canvas2D createPattern apis only support repeated tiles.
  /// For mirroring we create a new image with mirror builtin so both
  /// repeated and mirrored modes can be supported when applied to
  /// html element background-image or used by createPattern api.
  String _tileModeToHtmlRepeatAttribute(ui.TileMode tileModeX, ui.TileMode tileModeY) {
    final bool repeatX = tileModeX == ui.TileMode.repeated || tileModeX == ui.TileMode.mirror;
    final bool repeatY = tileModeY == ui.TileMode.repeated || tileModeY == ui.TileMode.mirror;
    return repeatX ? (repeatY ? 'repeat' : 'repeat-x') : (repeatY ? 'repeat-y' : 'no-repeat');
  }

  /// Tiles the image and returns an image or canvas element to be used as
  /// source for a repeated pattern.
  ///
  /// Other alternative was to create a webgl shader for the area and
  /// tile in the shader, but that will generate a much larger image footprint
  /// when the pattern is small. So we opt here for mirroring by
  /// redrawing the image 2 or 4 times into a new bitmap.
  Object? _resolveTiledImageSource(HtmlImage image, ui.TileMode tileX, ui.TileMode tileY) {
    final int mirrorX = tileX == ui.TileMode.mirror ? 2 : 1;
    final int mirrorY = tileY == ui.TileMode.mirror ? 2 : 1;

    /// If we have no mirror, we can use image directly as pattern.
    if (mirrorX == 1 && mirrorY == 1) {
      return image.imgElement;
    }

    /// Create a new image by mirroring.
    final int imageWidth = image.width;
    final int imageHeight = image.height;
    final int newWidth = imageWidth * mirrorX;
    final int newHeight = imageHeight * mirrorY;
    final OffScreenCanvas offscreenCanvas = OffScreenCanvas(newWidth, newHeight);
    final Object renderContext = offscreenCanvas.getContext2d()!;
    for (int y = 0; y < mirrorY; y++) {
      for (int x = 0; x < mirrorX; x++) {
        final int flipX = x != 0 ? -1 : 1;
        final int flipY = y != 0 ? -1 : 1;

        /// To draw image flipped we set translate and scale and pass
        /// negative width/height to drawImage.
        if (flipX != 1 || flipY != 1) {
          scaleCanvas2D(renderContext, flipX, flipY);
        }
        drawImageCanvas2D(
          renderContext,
          image.imgElement,
          x == 0 ? 0 : -2 * imageWidth,
          y == 0 ? 0 : -2 * imageHeight,
        );
        if (flipX != 1 || flipY != 1) {
          /// Restore transform. This is faster than save/restore on context.
          scaleCanvas2D(renderContext, flipX, flipY);
        }
      }
    }
    // When using OffscreenCanvas and transferToImageBitmap is supported by
    // browser create ImageBitmap otherwise use more expensive canvas
    // allocation.
    if (OffScreenCanvas.supported && offscreenCanvas.transferToImageBitmapSupported) {
      return offscreenCanvas.transferToImageBitmap();
    } else {
      final DomCanvasElement canvas = createDomCanvasElement(width: newWidth, height: newHeight);
      final DomCanvasRenderingContext2D ctx = canvas.context2D;
      offscreenCanvas.transferImage(ctx);
      return canvas;
    }
  }

  /// Creates an image with tiled/transformed images.
  DomCanvasPattern _createGlShader(
    DomCanvasRenderingContext2D? context,
    ui.Rect shaderBounds,
    double density,
  ) {
    final Matrix4 transform = Matrix4.fromFloat32List(matrix4);
    final double dpr = ui.window.devicePixelRatio;

    final int widthInPixels = (shaderBounds.width * dpr).ceil();
    final int heightInPixels = (shaderBounds.height * dpr).ceil();

    assert(widthInPixels > 0 && heightInPixels > 0);

    /// Render tiles into a bitmap and create a canvas pattern.
    final bool isWebGl2 = webGLVersion == WebGLVersion.webgl2;

    final String vertexShader = VertexShaders.writeTextureVertexShader();
    final String fragmentShader = FragmentShaders.writeTextureFragmentShader(
      isWebGl2,
      tileModeX,
      tileModeY,
    );

    /// Render gradient into a bitmap and create a canvas pattern.
    final OffScreenCanvas offScreenCanvas = OffScreenCanvas(widthInPixels, heightInPixels);
    final GlContext gl = GlContext(offScreenCanvas);
    gl.setViewportSize(widthInPixels, heightInPixels);

    final GlProgram glProgram = gl.cacheProgram(vertexShader, fragmentShader);
    gl.useProgram(glProgram);

    const int vertexCount = 6;
    final Float32List vertices = Float32List(vertexCount * 2);
    final ui.Rect vRect = shaderBounds.translate(-shaderBounds.left, -shaderBounds.top);
    vertices[0] = vRect.left;
    vertices[1] = vRect.top;
    vertices[2] = vRect.right;
    vertices[3] = vRect.top;
    vertices[4] = vRect.right;
    vertices[5] = vRect.bottom;
    vertices[6] = vRect.right;
    vertices[7] = vRect.bottom;
    vertices[8] = vRect.left;
    vertices[9] = vRect.bottom;
    vertices[10] = vRect.left;
    vertices[11] = vRect.top;

    final Object positionAttributeLocation = gl.getAttributeLocation(glProgram.program, 'position');

    setupVertexTransforms(
      gl,
      glProgram,
      0,
      0,
      widthInPixels.toDouble(),
      heightInPixels.toDouble(),
      transform,
    );

    requiresTileOffset = shaderBounds.left != 0 || shaderBounds.top != 0;

    /// To map from vertex position to texture coordinate in 0..1 range,
    /// we setup scalar to be used in vertex shader.
    setupTextureTransform(
      gl,
      glProgram,
      shaderBounds.left,
      shaderBounds.top,
      1.0 / image.width.toDouble(),
      1.0 / image.height.toDouble(),
    );

    /// Setup geometry.
    ///
    /// Create buffer for vertex coordinates.
    final Object positionsBuffer = gl.createBuffer()!;

    Object? vao;
    if (isWebGl2) {
      /// Create a vertex array object.
      vao = gl.createVertexArray();

      /// Set vertex array object as active one.
      gl.bindVertexArray(vao!);
    }

    /// Turn on position attribute.
    gl.enableVertexAttribArray(positionAttributeLocation);

    /// Bind buffer as position buffer and transfer data.
    gl.bindArrayBuffer(positionsBuffer);
    bufferVertexData(gl, vertices, ui.window.devicePixelRatio);

    /// Setup data format for attribute.
    vertexAttribPointerGlContext(
      gl.glContext,
      positionAttributeLocation,
      2,
      gl.kFloat,
      false,
      0,
      0,
    );

    /// Copy image to the texture.
    final Object? texture = gl.createTexture();

    /// Texture units are a global array of references to the textures.
    /// By setting activeTexture, we associate the bound texture to a unit.
    /// Every time we call a texture function such as texImage2D with a target
    /// like TEXTURE_2D, it looks up texture by using the currently active
    /// unit.
    /// In our case we have a single texture unit 0.
    gl.activeTexture(gl.kTexture0);
    gl.bindTexture(gl.kTexture2D, texture);

    gl.texImage2D(gl.kTexture2D, 0, gl.kRGBA, gl.kRGBA, gl.kUnsignedByte, image.imgElement);

    if (isWebGl2) {
      /// Texture REPEAT and MIRROR is only supported in WebGL 2, for
      /// WebGL 1.0 we let shader compute correct uv coordinates.
      gl.texParameteri(gl.kTexture2D, gl.kTextureWrapS, tileModeToGlWrapping(gl, tileModeX));

      gl.texParameteri(gl.kTexture2D, gl.kTextureWrapT, tileModeToGlWrapping(gl, tileModeY));

      /// Mipmapping saves your texture in different resolutions
      /// so the graphics card can choose which resolution is optimal
      /// without artifacts.
      gl.generateMipmap(gl.kTexture2D);
    } else {
      /// For webgl1, if a texture is not mipmap complete, then the return
      /// value of a texel fetch is (0, 0, 0, 1), so we have to set
      /// minifying function to filter.
      /// See https://www.khronos.org/registry/webgl/specs/1.0.0/#5.13.8.
      gl.texParameteri(gl.kTexture2D, gl.kTextureWrapS, gl.kClampToEdge);
      gl.texParameteri(gl.kTexture2D, gl.kTextureWrapT, gl.kClampToEdge);
      gl.texParameteri(gl.kTexture2D, gl.kTextureMinFilter, gl.kLinear);
    }

    /// Finally render triangles.
    gl.clear();

    gl.drawTriangles(vertexCount, ui.VertexMode.triangles);

    if (vao != null) {
      gl.unbindVertexArray();
    }

    final Object? bitmapImage = gl.readPatternData(false);
    gl.bindArrayBuffer(null);
    gl.bindElementArrayBuffer(null);
    return context!.createPattern(bitmapImage!, 'no-repeat')!;
  }

  bool _disposed = false;

  @override
  bool get debugDisposed {
    late bool disposed;
    assert(() {
      disposed = _disposed;
      return true;
    }());
    return disposed;
  }

  @override
  void dispose() {
    assert(() {
      _disposed = true;
      return true;
    }());
    image.dispose();
  }
}
