// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

import '../rendering/src/solid_color_box.dart';

// Solid color, RenderObject version
void addFlexChildSolidColor(RenderFlex parent, Color backgroundColor, {int flex = 0}) {
  final RenderSolidColorBox child = RenderSolidColorBox(backgroundColor);
  parent.add(child);
  final FlexParentData childParentData = child.parentData! as FlexParentData;
  childParentData.flex = flex;
}

// Solid color, Widget version
class Rectangle extends StatelessWidget {
  const Rectangle(this.color, {super.key});

  final Color color;

  @override
  Widget build(BuildContext context) {
    return Expanded(child: Container(color: color));
  }
}

double? value;
RenderObjectToWidgetElement<RenderBox>? element;
void attachWidgetTreeToRenderTree(RenderProxyBox container) {
  element = RenderObjectToWidgetAdapter<RenderBox>(
    container: container,
    child: Directionality(
      textDirection: TextDirection.ltr,
      child: SizedBox(
        height: 300.0,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            const Rectangle(Color(0xFF00FFFF)),
            Material(
              child: Container(
                padding: const EdgeInsets.all(10.0),
                margin: const EdgeInsets.all(10.0),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceAround,
                  children: <Widget>[
                    ElevatedButton(
                      child: const Row(children: <Widget>[FlutterLogo(), Text('PRESS ME')]),
                      onPressed: () {
                        value = value == null ? 0.1 : (value! + 0.1) % 1.0;
                        attachWidgetTreeToRenderTree(container);
                      },
                    ),
                    CircularProgressIndicator(value: value),
                  ],
                ),
              ),
            ),
            const Rectangle(Color(0xFFFFFF00)),
          ],
        ),
      ),
    ),
  ).attachToRenderTree(WidgetsBinding.instance.buildOwner!, element);
}

Duration? timeBase;
late RenderTransform transformBox;

void rotate(Duration timeStamp) {
  timeBase ??= timeStamp;
  final double delta =
      (timeStamp - timeBase!).inMicroseconds.toDouble() / Duration.microsecondsPerSecond; // radians

  transformBox.setIdentity();
  transformBox.rotateZ(delta);

  WidgetsBinding.instance.buildOwner!.buildScope(element!);
}

void main() {
  final WidgetsBinding binding = WidgetsFlutterBinding.ensureInitialized();
  final RenderProxyBox proxy = RenderProxyBox();
  attachWidgetTreeToRenderTree(proxy);

  final RenderFlex flexRoot = RenderFlex(direction: Axis.vertical);
  addFlexChildSolidColor(flexRoot, const Color(0xFFFF00FF), flex: 1);
  flexRoot.add(proxy);
  addFlexChildSolidColor(flexRoot, const Color(0xFF0000FF), flex: 1);

  transformBox = RenderTransform(
    child: flexRoot,
    transform: Matrix4.identity(),
    alignment: Alignment.center,
  );
  final RenderPadding root = RenderPadding(
    padding: const EdgeInsets.all(80.0),
    child: transformBox,
  );

  // TODO(goderbauer): Create a window if embedder doesn't provide an implicit view to draw into.
  assert(binding.platformDispatcher.implicitView != null);
  final RenderView view = RenderView(view: binding.platformDispatcher.implicitView!, child: root);
  final PipelineOwner pipelineOwner = PipelineOwner()..rootNode = view;
  binding.rootPipelineOwner.adoptChild(pipelineOwner);
  binding.addRenderView(view);
  view.prepareInitialFrame();

  binding.addPersistentFrameCallback(rotate);
}
