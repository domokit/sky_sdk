// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/rendering.dart';

/// An extension of [RenderingFlutterBinding] that owns and manages a
/// [renderView].
///
/// Unlike [RenderingFlutterBinding], this binding also creates and owns a
/// [renderView] to simplify bootstrapping for apps that have a dedicated main
/// view.
class ViewRenderingFlutterBinding extends RenderingFlutterBinding {
  /// Creates a binding for the rendering layer.
  ///
  /// The `root` render box is attached directly to the [renderView] and is
  /// given constraints that require it to fill the window. The [renderView]
  /// itself is attached to the [rootPipelineOwner].
  ///
  /// This binding does not automatically schedule any frames. Callers are
  /// responsible for deciding when to first call [scheduleFrame].
  ViewRenderingFlutterBinding({RenderBox? root}) : _root = root;

  @override
  void initInstances() {
    super.initInstances();
    // TODO(goderbauer): Create window if embedder doesn't provide an implicit view.
    assert(PlatformDispatcher.instance.implicitView != null);
    _renderView = initRenderView(PlatformDispatcher.instance.implicitView!);
    _renderView.child = _root;
    _root = null;
  }

  RenderBox? _root;

  @override
  RenderView get renderView => _renderView;
  late RenderView _renderView;

  /// Creates a [RenderView] object to be the root of the
  /// [RenderObject] rendering tree, and initializes it so that it
  /// will be rendered when the next frame is requested.
  ///
  /// Called automatically when the binding is created.
  RenderView initRenderView(FlutterView view) {
    final RenderView renderView = RenderView(view: view);
    rootPipelineOwner.rootNode = renderView;
    addRenderView(renderView);
    renderView.prepareInitialFrame();
    return renderView;
  }

  @override
  PipelineOwner createRootPipelineOwner() {
    return PipelineOwner(
      onSemanticsOwnerCreated: () {
        renderView.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {
        renderView.updateSemantics(update);
      },
      onSemanticsOwnerDisposed: () {
        renderView.clearSemantics();
      },
    );
  }
}
