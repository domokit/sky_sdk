// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:developer';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';

import 'debug.dart';

/// Interface for drawing an image to warm up Skia shader compilations.
///
/// When Skia first sees a certain type of draw operation on the GPU, it needs
/// to compile the corresponding shader. The compilation can be slow (20ms-
/// 200ms). Having that time as startup latency is often better than having
/// jank in the middle of an animation.
///
/// Therefore, we use this during the [PaintingBinding.initInstances] call to
/// move common shader compilations from animation time to startup time. If
/// needed, app developers can create a custom [ShaderWarmUp] subclass and
/// hand it to [PaintingBinding.shaderWarmUp] before
/// [PaintingBinding.initInstances] is called. Usually, that can be done before
/// calling [runApp].
///
/// To determine whether a draw operation is useful for warming up shaders,
/// check whether it improves the slowest frame rasterization time. Also,
/// tracing with `flutter run --profile --trace-skia` may reveal whether there
/// is shader-compilation-related jank. If there is such jank, some long
/// `GrGLProgramBuilder::finalize` calls would appear in the middle of an
/// animation. Their parent calls, which look like `XyzOp` (e.g., `FillRecOp`,
/// `CircularRRectOp`) would suggest Xyz draw operations are causing the shaders
/// to be compiled. A useful shader warm-up draw operation would eliminate such
/// long compilation calls in the animation. To double-check the warm-up, trace
/// with `flutter run --profile --trace-skia --start-paused`. The
/// `GrGLProgramBuilder` with the associated `XyzOp` should appear during
/// startup rather than in the middle of a later animation.
///
/// This warm-up needs to be run on each individual device because the shader
/// compilation depends on the specific GPU hardware and driver a device has. It
/// can't be pre-computed during the Flutter engine compilation as the engine is
/// device-agnostic.
///
/// If no warm-up is desired (e.g., when the startup latency is crucial), set
/// [PaintingBinding.shaderWarmUp] either to a custom ShaderWarmUp with an empty
/// [warmUpOnCanvas] or null.
///
/// See also:
///
///  * [PaintingBinding.shaderWarmUp], the actual instance of [ShaderWarmUp]
///    that's used to warm up the shaders.
///  * <https://docs.flutter.dev/perf/shader>
abstract class ShaderWarmUp {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ShaderWarmUp();

  /// The size of the warm up image.
  ///
  /// The exact size shouldn't matter much as long as all draws are onscreen.
  /// 100x100 is an arbitrary small size that's easy to fit significant draw
  /// calls onto.
  ///
  /// A custom shader warm up can override this based on targeted devices.
  ui.Size get size => const ui.Size(100.0, 100.0);

  /// Trigger draw operations on a given canvas to warm up GPU shader
  /// compilation cache.
  ///
  /// To decide which draw operations to be added to your custom warm up
  /// process, consider capturing an skp using `flutter screenshot
  /// --vm-service-uri=<uri> --type=skia` and analyzing it with
  /// <https://debugger.skia.org/>. Alternatively, one may run the app with
  /// `flutter run --trace-skia` and then examine the raster thread in the
  /// Flutter DevTools timeline to see which Skia draw operations are commonly used,
  /// and which shader compilations are causing jank.
  @protected
  Future<void> warmUpOnCanvas(ui.Canvas canvas);

  /// Construct an offscreen image of [size], and execute [warmUpOnCanvas] on a
  /// canvas associated with that image.
  ///
  /// Currently, this has no effect when [kIsWeb] is true.
  Future<void> execute() async {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    await warmUpOnCanvas(canvas);
    final ui.Picture picture = recorder.endRecording();
    assert(debugCaptureShaderWarmUpPicture(picture));
    if (!kIsWeb || isSkiaWeb) {
      // Picture.toImage is not implemented on the html renderer.
      TimelineTask? debugShaderWarmUpTask;
      if (!kReleaseMode) {
        debugShaderWarmUpTask = TimelineTask()..start('Warm-up shader');
      }
      try {
        final ui.Image image = await picture.toImage(size.width.ceil(), size.height.ceil());
        assert(debugCaptureShaderWarmUpImage(image));
        image.dispose();
      } finally {
        if (!kReleaseMode) {
          debugShaderWarmUpTask!.finish();
        }
      }
    }
    picture.dispose();
  }
}
