// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Flutter GPU API tests.
// The flutter_gpu package is located at //flutter/impeller/lib/gpu.

// ignore_for_file: avoid_relative_lib_imports

import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../lib/gpu/lib/gpu.dart' as gpu;

import 'goldens.dart';
import 'impeller_enabled.dart';

ByteData float32(List<double> values) {
  return Float32List.fromList(values).buffer.asByteData();
}

ByteData unlitUBO(Matrix4 mvp, Vector4 color) {
  return float32(<double>[
    mvp[0], mvp[1], mvp[2], mvp[3], //
    mvp[4], mvp[5], mvp[6], mvp[7], //
    mvp[8], mvp[9], mvp[10], mvp[11], //
    mvp[12], mvp[13], mvp[14], mvp[15], //
    color.r, color.g, color.b, color.a,
  ]);
}

gpu.RenderPipeline createUnlitRenderPipeline() {
  final gpu.ShaderLibrary? library = gpu.ShaderLibrary.fromAsset('test.shaderbundle');
  assert(library != null);
  final gpu.Shader? vertex = library!['UnlitVertex'];
  assert(vertex != null);
  final gpu.Shader? fragment = library['UnlitFragment'];
  assert(fragment != null);
  return gpu.gpuContext.createRenderPipeline(vertex!, fragment!);
}

class RenderPassState {
  RenderPassState(this.renderTexture, this.commandBuffer, this.renderPass);

  final gpu.Texture renderTexture;
  final gpu.CommandBuffer commandBuffer;
  final gpu.RenderPass renderPass;
}

/// Create a simple RenderPass with simple color and depth-stencil attachments.
RenderPassState createSimpleRenderPass({Vector4? clearColor}) {
  final gpu.Texture renderTexture = gpu.gpuContext.createTexture(
    gpu.StorageMode.devicePrivate,
    100,
    100,
  );

  final gpu.Texture depthStencilTexture = gpu.gpuContext.createTexture(
    gpu.StorageMode.deviceTransient,
    100,
    100,
    format: gpu.gpuContext.defaultDepthStencilFormat,
  );

  final gpu.CommandBuffer commandBuffer = gpu.gpuContext.createCommandBuffer();

  final gpu.RenderTarget renderTarget = gpu.RenderTarget.singleColor(
    gpu.ColorAttachment(texture: renderTexture, clearValue: clearColor),
    depthStencilAttachment: gpu.DepthStencilAttachment(texture: depthStencilTexture),
  );

  final gpu.RenderPass renderPass = commandBuffer.createRenderPass(renderTarget);

  return RenderPassState(renderTexture, commandBuffer, renderPass);
}

RenderPassState createSimpleRenderPassWithMSAA() {
  // Create transient MSAA attachments, which will live entirely in tile memory
  // for most GPUs.

  final gpu.Texture renderTexture = gpu.gpuContext.createTexture(
    gpu.StorageMode.deviceTransient,
    100,
    100,
    format: gpu.gpuContext.defaultColorFormat,
    sampleCount: 4,
  );

  final gpu.Texture depthStencilTexture = gpu.gpuContext.createTexture(
    gpu.StorageMode.deviceTransient,
    100,
    100,
    format: gpu.gpuContext.defaultDepthStencilFormat,
    sampleCount: 4,
  );

  // Create the single-sample resolve texture that live in DRAM and will be
  // drawn to the screen.

  final gpu.Texture resolveTexture = gpu.gpuContext.createTexture(
    gpu.StorageMode.devicePrivate,
    100,
    100,
    format: gpu.gpuContext.defaultColorFormat,
  );

  final gpu.CommandBuffer commandBuffer = gpu.gpuContext.createCommandBuffer();

  final gpu.RenderTarget renderTarget = gpu.RenderTarget.singleColor(
    gpu.ColorAttachment(
      texture: renderTexture,
      resolveTexture: resolveTexture,
      storeAction: gpu.StoreAction.multisampleResolve,
    ),
    depthStencilAttachment: gpu.DepthStencilAttachment(texture: depthStencilTexture),
  );

  final gpu.RenderPass renderPass = commandBuffer.createRenderPass(renderTarget);

  return RenderPassState(resolveTexture, commandBuffer, renderPass);
}

void drawTriangle(RenderPassState state, Vector4 color) {
  final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();

  state.renderPass.bindPipeline(pipeline);

  final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
  final gpu.BufferView vertices = transients.emplace(
    float32(<double>[
      -0.5, 0.5, //
      0.0, -0.5, //
      0.5, 0.5, //
    ]),
  );
  final gpu.BufferView vertInfoData = transients.emplace(unlitUBO(Matrix4.identity(), color));
  state.renderPass.bindVertexBuffer(vertices, 3);

  final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
  state.renderPass.bindUniform(vertInfo, vertInfoData);

  state.renderPass.draw();
}

void main() async {
  final ImageComparer comparer = await ImageComparer.create();

  test('gpu.context throws exception for incompatible embedders', () async {
    try {
      // ignore: unnecessary_statements
      gpu.gpuContext; // Force the context to instantiate.
      if (!impellerEnabled) {
        fail('Exception not thrown, but no Impeller context available.');
      }
    } catch (e) {
      if (impellerEnabled) {
        fail('Exception thrown even though Impeller is enabled.');
      }
      expect(
        e.toString(),
        contains('Flutter GPU requires the Impeller rendering backend to be enabled.'),
      );
    }
  });

  test('GpuContext.minimumUniformByteAlignment', () async {
    final int alignment = gpu.gpuContext.minimumUniformByteAlignment;
    expect(alignment, greaterThanOrEqualTo(16));
  }, skip: !impellerEnabled);

  test('HostBuffer.emplace', () async {
    final gpu.HostBuffer hostBuffer = gpu.gpuContext.createHostBuffer();

    final gpu.BufferView view0 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );
    expect(view0.offsetInBytes, 0);
    expect(view0.lengthInBytes, 4);

    final gpu.BufferView view1 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );
    expect(view1.offsetInBytes, equals(gpu.gpuContext.minimumUniformByteAlignment));
    expect(view1.lengthInBytes, 4);
  }, skip: !impellerEnabled);

  test('HostBuffer.reset', () async {
    final gpu.HostBuffer hostBuffer = gpu.gpuContext.createHostBuffer();

    final gpu.BufferView view0 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );
    expect(view0.offsetInBytes, 0);
    expect(view0.lengthInBytes, 4);

    hostBuffer.reset();

    final gpu.BufferView view1 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );
    expect(view1.offsetInBytes, 0);
    expect(view1.lengthInBytes, 4);
  }, skip: !impellerEnabled);

  test('HostBuffer reuses DeviceBuffers after N frames', () async {
    final gpu.HostBuffer hostBuffer = gpu.gpuContext.createHostBuffer();

    final gpu.BufferView view0 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );

    for (int i = 0; i < hostBuffer.frameCount; i++) {
      hostBuffer.reset();
    }
    final gpu.BufferView view1 = hostBuffer.emplace(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );

    expect(view0.buffer, equals(view1.buffer));
  }, skip: !impellerEnabled);

  test('GpuContext.createDeviceBuffer', () async {
    final gpu.DeviceBuffer deviceBuffer = gpu.gpuContext.createDeviceBuffer(
      gpu.StorageMode.hostVisible,
      4,
    );

    expect(deviceBuffer.sizeInBytes, 4);
  }, skip: !impellerEnabled);

  test('DeviceBuffer.overwrite', () async {
    final gpu.DeviceBuffer deviceBuffer = gpu.gpuContext.createDeviceBuffer(
      gpu.StorageMode.hostVisible,
      4,
    );

    final bool success = deviceBuffer.overwrite(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
    );
    deviceBuffer.flush();
    expect(success, true);
  }, skip: !impellerEnabled);

  test('DeviceBuffer.overwrite fails when out of bounds', () async {
    final gpu.DeviceBuffer deviceBuffer = gpu.gpuContext.createDeviceBuffer(
      gpu.StorageMode.hostVisible,
      4,
    );

    final bool success = deviceBuffer.overwrite(
      Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
      destinationOffsetInBytes: 1,
    );
    deviceBuffer.flush();
    expect(success, false);
  }, skip: !impellerEnabled);

  test('DeviceBuffer.overwrite throws for negative destination offset', () async {
    final gpu.DeviceBuffer deviceBuffer = gpu.gpuContext.createDeviceBuffer(
      gpu.StorageMode.hostVisible,
      4,
    );

    try {
      deviceBuffer.overwrite(
        Int8List.fromList(<int>[0, 1, 2, 3]).buffer.asByteData(),
        destinationOffsetInBytes: -1,
      );
      deviceBuffer.flush();
      fail('Exception not thrown for negative destination offset.');
    } catch (e) {
      expect(e.toString(), contains('destinationOffsetInBytes must be positive'));
    }
  }, skip: !impellerEnabled);

  test('GpuContext.createTexture', () async {
    final gpu.Texture texture = gpu.gpuContext.createTexture(gpu.StorageMode.hostVisible, 100, 100);

    // Check the defaults.
    expect(texture.coordinateSystem, gpu.TextureCoordinateSystem.renderToTexture);
    expect(texture.width, 100);
    expect(texture.height, 100);
    expect(texture.storageMode, gpu.StorageMode.hostVisible);
    expect(texture.sampleCount, 1);
    expect(texture.format, gpu.PixelFormat.r8g8b8a8UNormInt);
    expect(texture.enableRenderTargetUsage, true);
    expect(texture.enableShaderReadUsage, true);
    expect(!texture.enableShaderWriteUsage, true);
    expect(texture.bytesPerTexel, 4);
    expect(texture.getBaseMipLevelSizeInBytes(), 40000);
  }, skip: !impellerEnabled);

  test('Texture.overwrite', () async {
    final gpu.Texture texture = gpu.gpuContext.createTexture(gpu.StorageMode.hostVisible, 2, 2);

    const ui.Color red = ui.Color.fromARGB(0xFF, 0xFF, 0, 0);
    const ui.Color green = ui.Color.fromARGB(0xFF, 0, 0xFF, 0);
    texture.overwrite(
      Int32List.fromList(<int>[red.value, green.value, green.value, red.value]).buffer.asByteData(),
    );
  }, skip: !impellerEnabled);

  test('Texture.overwrite throws for wrong buffer size', () async {
    final gpu.Texture texture = gpu.gpuContext.createTexture(gpu.StorageMode.hostVisible, 100, 100);

    const ui.Color red = ui.Color.fromARGB(0xFF, 0xFF, 0, 0);
    try {
      texture.overwrite(
        Int32List.fromList(<int>[red.value, red.value, red.value, red.value]).buffer.asByteData(),
      );
      fail('Exception not thrown for wrong buffer size.');
    } catch (e) {
      expect(
        e.toString(),
        contains(
          'The length of sourceBytes (bytes: 16) must exactly match the size of the base mip level (bytes: 40000)',
        ),
      );
    }
  }, skip: !impellerEnabled);

  test('Texture.asImage returns a valid ui.Image handle', () async {
    final gpu.Texture texture = gpu.gpuContext.createTexture(gpu.StorageMode.hostVisible, 100, 100);

    final ui.Image image = texture.asImage();
    expect(image.width, 100);
    expect(image.height, 100);
  }, skip: !impellerEnabled);

  test('Texture.asImage throws when not shader readable', () async {
    final gpu.Texture texture = gpu.gpuContext.createTexture(
      gpu.StorageMode.hostVisible,
      100,
      100,
      enableShaderReadUsage: false,
    );

    try {
      texture.asImage();
      fail('Exception not thrown when not shader readable.');
    } catch (e) {
      expect(
        e.toString(),
        contains('Only shader readable Flutter GPU textures can be used as UI Images'),
      );
    }
  }, skip: !impellerEnabled);

  test('RenderPass.setStencilReference doesnt throw for valid values', () async {
    final state = createSimpleRenderPass();

    state.renderPass.setStencilReference(0);
    state.renderPass.setStencilReference(2 << 30);
  }, skip: !impellerEnabled);

  test('RenderPass.setStencilReference throws for invalid values', () async {
    final state = createSimpleRenderPass();

    try {
      state.renderPass.setStencilReference(-1);
      fail('Exception not thrown for out of bounds stencil reference.');
    } catch (e) {
      expect(e.toString(), contains('The stencil reference value must be in the range'));
    }

    try {
      state.renderPass.setStencilReference(2 << 31);
      fail('Exception not thrown for out of bounds stencil reference.');
    } catch (e) {
      expect(e.toString(), contains('The stencil reference value must be in the range'));
    }
  }, skip: !impellerEnabled);

  test('RenderPass.setStencilConfig doesnt throw for valid values', () async {
    final state = createSimpleRenderPass();

    state.renderPass.setStencilConfig(gpu.StencilConfig());
    state.renderPass.setStencilConfig(
      gpu.StencilConfig(
        compareFunction: gpu.CompareFunction.notEqual,
        depthFailureOperation: gpu.StencilOperation.decrementWrap,
        depthStencilPassOperation: gpu.StencilOperation.incrementWrap,
        stencilFailureOperation: gpu.StencilOperation.invert,
        readMask: 0,
        writeMask: 0,
      ),
      targetFace: gpu.StencilFace.back,
    );
  }, skip: !impellerEnabled);

  test('RenderPass.setStencilConfig throws for invalid masks', () async {
    final state = createSimpleRenderPass();

    try {
      state.renderPass.setStencilConfig(gpu.StencilConfig(readMask: -1));
      fail('Exception not thrown for invalid stencil read mask.');
    } catch (e) {
      expect(e.toString(), contains('The stencil read mask must be in the range'));
    }
    try {
      state.renderPass.setStencilConfig(gpu.StencilConfig(readMask: 0xFFFFFFFF + 1));
      fail('Exception not thrown for invalid stencil read mask.');
    } catch (e) {
      expect(e.toString(), contains('The stencil read mask must be in the range'));
    }

    try {
      state.renderPass.setStencilConfig(gpu.StencilConfig(writeMask: -1));
      fail('Exception not thrown for invalid stencil write mask.');
    } catch (e) {
      expect(e.toString(), contains('The stencil write mask must be in the range'));
    }
    try {
      state.renderPass.setStencilConfig(gpu.StencilConfig(writeMask: 0xFFFFFFFF + 1));
      fail('Exception not thrown for invalid stencil write mask.');
    } catch (e) {
      expect(e.toString(), contains('The stencil write mask must be in the range'));
    }
  }, skip: !impellerEnabled);

  test('RenderPass.bindTexture throws for deviceTransient Textures', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    // Although this is a non-texture uniform slot, it'll work fine for the
    // purposes of testing this error.
    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');

    final gpu.Texture texture = gpu.gpuContext.createTexture(
      gpu.StorageMode.deviceTransient,
      100,
      100,
    );

    try {
      state.renderPass.bindTexture(vertInfo, texture);
      fail('Exception not thrown when binding a transient texture.');
    } catch (e) {
      expect(
        e.toString(),
        contains('Textures with StorageMode.deviceTransient cannot be bound to a RenderPass'),
      );
    }
  }, skip: !impellerEnabled);

  // Performs no draw calls. Just clears the render target to a solid green color.
  test('Can render clear color', () async {
    final state = createSimpleRenderPass(clearColor: Colors.lime);

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_clear_color.png');
  }, skip: !impellerEnabled);

  // Regression test for https://github.com/flutter/flutter/issues/157324
  test('Can bind uniforms in range', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');

    final ByteData vertInfoData = float32(<double>[
      1, 0, 0, 0, // mvp
      0, 1, 0, 0, // mvp
      0, 0, 1, 0, // mvp
      0, 0, 0, 1, // mvp
      0, 1, 0, 1, // color
    ]);
    final uniformBuffer = gpu.gpuContext.createDeviceBufferWithCopy(vertInfoData);
    final gooduniformBufferView = gpu.BufferView(
      uniformBuffer,
      offsetInBytes: 0,
      lengthInBytes: uniformBuffer.sizeInBytes,
    );
    state.renderPass.bindUniform(vertInfo, gooduniformBufferView);

    final badUniformBufferView = gpu.BufferView(
      uniformBuffer,
      offsetInBytes: 0,
      lengthInBytes: uniformBuffer.sizeInBytes + 1,
    );
    try {
      state.renderPass.bindUniform(vertInfo, badUniformBufferView);
      fail('Exception not thrown for bad buffer view range.');
    } catch (e) {
      expect(e.toString(), contains('Failed to bind uniform'));
    }
  }, skip: !impellerEnabled);

  // Renders a green triangle pointing downwards.
  test('Can render triangle', () async {
    final state = createSimpleRenderPass();
    drawTriangle(state, Colors.lime);
    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_triangle.png');
  }, skip: !impellerEnabled);

  // Renders a green triangle pointing downwards using polygon mode line.
  test('Can render triangle with polygon mode line.', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    // Configure blending with defaults (just to test the bindings).
    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    // Set polygon mode.
    state.renderPass.setPolygonMode(gpu.PolygonMode.line);

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    final gpu.BufferView vertices = transients.emplace(
      float32(<double>[
        -0.5, 0.5, //
        0.0, -0.5, //
        0.5, 0.5, //
      ]),
    );
    final gpu.BufferView vertInfoData = transients.emplace(
      float32(<double>[
        1, 0, 0, 0, // mvp
        0, 1, 0, 0, // mvp
        0, 0, 1, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    state.renderPass.bindVertexBuffer(vertices, 3);

    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
    state.renderPass.bindUniform(vertInfo, vertInfoData);
    state.renderPass.draw();

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_triangle_polygon_mode.png');
  }, skip: !impellerEnabled);

  // Renders a green triangle pointing downwards, with 4xMSAA.
  test(
    'Can render triangle with MSAA',
    () async {
      final state = createSimpleRenderPassWithMSAA();
      drawTriangle(state, Colors.lime);
      state.commandBuffer.submit();

      final ui.Image image = state.renderTexture.asImage();
      await comparer.addGoldenImage(image, 'flutter_gpu_test_triangle_msaa.png');
    },
    skip: !(impellerEnabled && gpu.gpuContext.doesSupportOffscreenMSAA),
  );

  test(
    'Rendering with MSAA throws exception when offscreen MSAA is not supported',
    () async {
      try {
        final state = createSimpleRenderPassWithMSAA();
        drawTriangle(state, Colors.lime);
        state.commandBuffer.submit();
        fail('Exception not thrown when offscreen MSAA is not supported.');
      } catch (e) {
        expect(
          e.toString(),
          contains(
            'The backend does not support multisample anti-aliasing for offscreen color and stencil attachments',
          ),
        );
      }
    },
    skip: !(impellerEnabled && !gpu.gpuContext.doesSupportOffscreenMSAA),
  );

  // Renders a hollow green triangle pointing downwards.
  test('Can render hollowed out triangle using stencil ops', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    // Configure blending with defaults (just to test the bindings).
    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    final gpu.BufferView vertices = transients.emplace(
      float32(<double>[
        -0.5, 0.5, //
        0.0, -0.5, //
        0.5, 0.5, //
      ]),
    );
    final gpu.BufferView innerClipVertInfo = transients.emplace(
      float32(<double>[
        0.5, 0, 0, 0, // mvp
        0, 0.5, 0, 0, // mvp
        0, 0, 0.5, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    final gpu.BufferView outerGreenVertInfo = transients.emplace(
      float32(<double>[
        1, 0, 0, 0, // mvp
        0, 1, 0, 0, // mvp
        0, 0, 1, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    state.renderPass.bindVertexBuffer(vertices, 3);

    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');

    // First, punch out a scaled down triangle in the stencil buffer.
    // Since the stencil buffer is initialized to 0, we set the stencil ref to 1
    // and the compare to `equial`, which will result in the stencil test
    // failing. But on failure, we increment the stencil in order to punch out
    // the triangle.

    state.renderPass.bindUniform(vertInfo, innerClipVertInfo);
    state.renderPass.setStencilReference(1);
    state.renderPass.setStencilConfig(
      gpu.StencilConfig(
        compareFunction: gpu.CompareFunction.equal,
        stencilFailureOperation: gpu.StencilOperation.incrementClamp,
      ),
    );
    state.renderPass.draw();

    // Next, render the outer triangle with the stencil ref set to zero, so that
    // the stencil test passes everywhere except where the inner triangle was
    // punched out.

    state.renderPass.setStencilReference(0);
    // Set the stencil config to turn off the increment. For this golden test
    // we technically don't need to do this, but we do it here just to exercise
    // the API.
    state.renderPass.setStencilConfig(
      gpu.StencilConfig(compareFunction: gpu.CompareFunction.equal),
    );
    state.renderPass.bindUniform(vertInfo, outerGreenVertInfo);
    state.renderPass.draw();

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_triangle_stencil.png');
  }, skip: !impellerEnabled);

  test('Drawing respects cull mode', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    // Counter-clockwise triangle.
    final List<double> triangle = [
      -0.5, 0.5, //
      0.0, -0.5, //
      0.5, 0.5, //
    ];
    final gpu.BufferView vertices = transients.emplace(float32(triangle));

    void drawTriangle(Vector4 color) {
      final gpu.BufferView vertInfoUboFront = transients.emplace(
        unlitUBO(Matrix4.identity(), color),
      );

      final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
      state.renderPass.bindVertexBuffer(vertices, 3);
      state.renderPass.bindUniform(vertInfo, vertInfoUboFront);
      state.renderPass.draw();
    }

    // Draw the green rectangle.
    // Defaults to clockwise winding order. So frontface culling should not
    // impact the green triangle.
    state.renderPass.setCullMode(gpu.CullMode.frontFace);
    drawTriangle(Colors.lime);

    // Backface cull a red triangle.
    state.renderPass.setCullMode(gpu.CullMode.backFace);
    drawTriangle(Colors.red);

    // Invert the winding mode and frontface cull a red rectangle.
    state.renderPass.setWindingOrder(gpu.WindingOrder.counterClockwise);
    state.renderPass.setCullMode(gpu.CullMode.frontFace);
    drawTriangle(Colors.red);

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_cull_mode.png');
  }, skip: !impellerEnabled);

  // Renders a hexagon using line strip primitive type.
  test('Can render hollow hexagon using line strip primitive type', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    // Configure blending with defaults (just to test the bindings).
    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    // Set primitive type
    state.renderPass.setPrimitiveType(gpu.PrimitiveType.lineStrip);

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    final gpu.BufferView vertices = transients.emplace(
      float32(<double>[1.0, 0.0, 0.5, 0.8, -0.5, 0.8, -1.0, 0.0, -0.5, -0.8, 0.5, -0.8, 1.0, 0.0]),
    );
    final gpu.BufferView vertInfoData = transients.emplace(
      float32(<double>[
        1, 0, 0, 0, // mvp
        0, 1, 0, 0, // mvp
        0, 0, 1, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    state.renderPass.bindVertexBuffer(vertices, 7);

    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
    state.renderPass.bindUniform(vertInfo, vertInfoData);
    state.renderPass.draw();

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_hexgon_line_strip.png');
  }, skip: !impellerEnabled);

  // Renders the middle part triangle using scissor.
  test('Can render portion of the triangle using scissor', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    // Configure blending with defaults (just to test the bindings).
    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    // Set primitive type.
    state.renderPass.setPrimitiveType(gpu.PrimitiveType.triangle);

    // Set scissor.
    state.renderPass.setScissor(gpu.Scissor(x: 25, width: 50, height: 100));

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    final gpu.BufferView vertices = transients.emplace(
      float32(<double>[-1.0, -1.0, 0.0, 1.0, 1.0, -1.0]),
    );
    final gpu.BufferView vertInfoData = transients.emplace(
      float32(<double>[
        1, 0, 0, 0, // mvp
        0, 1, 0, 0, // mvp
        0, 0, 1, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    state.renderPass.bindVertexBuffer(vertices, 3);

    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
    state.renderPass.bindUniform(vertInfo, vertInfoData);
    state.renderPass.draw();

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_scissor.png');
  }, skip: !impellerEnabled);

  test('RenderPass.setScissor doesnt throw for valid values', () async {
    final state = createSimpleRenderPass();

    state.renderPass.setScissor(gpu.Scissor(x: 25, width: 50, height: 100));
    state.renderPass.setScissor(gpu.Scissor(width: 50, height: 100));
  }, skip: !impellerEnabled);

  test('RenderPass.setScissor throws for invalid values', () async {
    final state = createSimpleRenderPass();

    try {
      state.renderPass.setScissor(gpu.Scissor(x: -1, width: 50, height: 100));
      fail('Exception not thrown for invalid scissor.');
    } catch (e) {
      expect(e.toString(), contains('Invalid values for scissor. All values should be positive.'));
    }

    try {
      state.renderPass.setScissor(gpu.Scissor(width: 50, height: -100));
      fail('Exception not thrown for invalid scissor.');
    } catch (e) {
      expect(e.toString(), contains('Invalid values for scissor. All values should be positive.'));
    }
  }, skip: !impellerEnabled);

  test('RenderPass.setViewport doesnt throw for valid values', () async {
    final state = createSimpleRenderPass();

    state.renderPass.setViewport(gpu.Viewport(x: 25, width: 50, height: 100));
    state.renderPass.setViewport(gpu.Viewport(width: 50, height: 100));
  }, skip: !impellerEnabled);

  test('RenderPass.setViewport throws for invalid values', () async {
    final state = createSimpleRenderPass();

    try {
      state.renderPass.setViewport(gpu.Viewport(x: -1, width: 50, height: 100));
      fail('Exception not thrown for invalid viewport.');
    } catch (e) {
      expect(e.toString(), contains('Invalid values for viewport. All values should be positive.'));
    }

    try {
      state.renderPass.setViewport(gpu.Viewport(width: 50, height: -100));
      fail('Exception not thrown for invalid viewport.');
    } catch (e) {
      expect(e.toString(), contains('Invalid values for viewport. All values should be positive.'));
    }
  }, skip: !impellerEnabled);

  // Renders the middle part triangle using viewport.
  test('Can render portion of the triangle using viewport', () async {
    final state = createSimpleRenderPass();

    final gpu.RenderPipeline pipeline = createUnlitRenderPipeline();
    state.renderPass.bindPipeline(pipeline);

    // Configure blending with defaults (just to test the bindings).
    state.renderPass.setColorBlendEnable(true);
    state.renderPass.setColorBlendEquation(gpu.ColorBlendEquation());

    // Set primitive type.
    state.renderPass.setPrimitiveType(gpu.PrimitiveType.triangle);

    // Set viewport.
    state.renderPass.setViewport(gpu.Viewport(x: 25, width: 50, height: 100));

    final gpu.HostBuffer transients = gpu.gpuContext.createHostBuffer();
    final gpu.BufferView vertices = transients.emplace(
      float32(<double>[-1.0, -1.0, 0.0, 1.0, 1.0, -1.0]),
    );
    final gpu.BufferView vertInfoData = transients.emplace(
      float32(<double>[
        1, 0, 0, 0, // mvp
        0, 1, 0, 0, // mvp
        0, 0, 1, 0, // mvp
        0, 0, 0, 1, // mvp
        0, 1, 0, 1, // color
      ]),
    );
    state.renderPass.bindVertexBuffer(vertices, 3);

    final gpu.UniformSlot vertInfo = pipeline.vertexShader.getUniformSlot('VertInfo');
    state.renderPass.bindUniform(vertInfo, vertInfoData);
    state.renderPass.draw();

    state.commandBuffer.submit();

    final ui.Image image = state.renderTexture.asImage();
    await comparer.addGoldenImage(image, 'flutter_gpu_test_viewport.png');
  }, skip: !impellerEnabled);
}
