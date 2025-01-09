// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

/// Flutter code sample for [BuildOwner].

void main() {
  runApp(const BuildOwnerExample());
}

class BuildOwnerExample extends StatefulWidget {
  const BuildOwnerExample({super.key});

  @override
  State<BuildOwnerExample> createState() => _BuildOwnerExampleState();
}

class _BuildOwnerExampleState extends State<BuildOwnerExample> {
  late final Size size;

  @override
  void initState() {
    super.initState();
    size = measureWidget(const SizedBox(width: 640, height: 480));
  }

  @override
  Widget build(BuildContext context) {
    // Just displays the size calculated above.
    return WidgetsApp(
      title: 'BuildOwner Sample',
      color: const Color(0xff000000),
      builder: (BuildContext context, Widget? child) {
        return Scaffold(body: Center(child: Text(size.toString())));
      },
    );
  }
}

Size measureWidget(Widget widget) {
  final PipelineOwner pipelineOwner = PipelineOwner();
  final MeasurementView rootView = pipelineOwner.rootNode = MeasurementView();
  final BuildOwner buildOwner = BuildOwner(focusManager: FocusManager());
  final RenderObjectToWidgetElement<RenderBox> element = RenderObjectToWidgetAdapter<RenderBox>(
    container: rootView,
    debugShortDescription: '[root]',
    child: widget,
  ).attachToRenderTree(buildOwner);
  try {
    rootView.scheduleInitialLayout();
    pipelineOwner.flushLayout();
    return rootView.size;
  } finally {
    // Clean up.
    element.update(RenderObjectToWidgetAdapter<RenderBox>(container: rootView));
    buildOwner.finalizeTree();
  }
}

class MeasurementView extends RenderBox with RenderObjectWithChildMixin<RenderBox> {
  @override
  void performLayout() {
    assert(child != null);
    child!.layout(const BoxConstraints(), parentUsesSize: true);
    size = child!.size;
  }

  @override
  void debugAssertDoesMeetConstraints() => true;
}
