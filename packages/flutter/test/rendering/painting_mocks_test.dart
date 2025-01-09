// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

final List<String> log = <String>[];

void main() {
  final PaintingMocksTestRenderingFlutterBinding binding =
      PaintingMocksTestRenderingFlutterBinding.ensureInitialized();

  test('createSceneBuilder et al', () async {
    final RenderView root = RenderView(
      view: binding.platformDispatcher.views.single,
      configuration: const ViewConfiguration(),
    );
    root.attach(PipelineOwner());
    root.prepareInitialFrame();
    expect(log, isEmpty);
    root.compositeFrame();
    expect(log, <String>['createSceneBuilder']);
    log.clear();
    final PaintingContext context = PaintingContext(ContainerLayer(), Rect.zero);
    expect(log, isEmpty);
    context.canvas;
    expect(log, <String>['createPictureRecorder', 'createCanvas']);
    log.clear();
    context.addLayer(ContainerLayer());
    expect(log, isEmpty);
    context.canvas;
    expect(log, <String>['createPictureRecorder', 'createCanvas']);
    log.clear();
  });
}

class PaintingMocksTestRenderingFlutterBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        PaintingBinding,
        SemanticsBinding,
        RendererBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  static PaintingMocksTestRenderingFlutterBinding get instance =>
      BindingBase.checkInstance(_instance);
  static PaintingMocksTestRenderingFlutterBinding? _instance;

  static PaintingMocksTestRenderingFlutterBinding ensureInitialized() {
    if (PaintingMocksTestRenderingFlutterBinding._instance == null) {
      PaintingMocksTestRenderingFlutterBinding();
    }
    return PaintingMocksTestRenderingFlutterBinding.instance;
  }

  @override
  ui.SceneBuilder createSceneBuilder() {
    log.add('createSceneBuilder');
    return super.createSceneBuilder();
  }

  @override
  ui.PictureRecorder createPictureRecorder() {
    log.add('createPictureRecorder');
    return super.createPictureRecorder();
  }

  @override
  Canvas createCanvas(ui.PictureRecorder recorder) {
    log.add('createCanvas');
    return super.createCanvas(recorder);
  }
}
