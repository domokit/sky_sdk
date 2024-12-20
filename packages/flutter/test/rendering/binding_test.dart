// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();

  test(
    'handleMetricsChanged does not scheduleForcedFrame unless there a registered renderView with a child',
    () async {
      expect(SchedulerBinding.instance.hasScheduledFrame, false);
      RendererBinding.instance.handleMetricsChanged();
      expect(SchedulerBinding.instance.hasScheduledFrame, false);

      RendererBinding.instance.addRenderView(RendererBinding.instance.renderView);
      RendererBinding.instance.handleMetricsChanged();
      expect(SchedulerBinding.instance.hasScheduledFrame, false);

      RendererBinding.instance.renderView.child = RenderLimitedBox();
      RendererBinding.instance.handleMetricsChanged();
      expect(SchedulerBinding.instance.hasScheduledFrame, true);

      RendererBinding.instance.removeRenderView(RendererBinding.instance.renderView);
    },
  );

  test('debugDumpSemantics prints explanation when semantics are unavailable', () {
    RendererBinding.instance.addRenderView(RendererBinding.instance.renderView);
    final List<String?> log = <String?>[];
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add(message);
    };
    debugDumpSemanticsTree();
    expect(log, hasLength(1));
    expect(log.single, startsWith('Semantics not generated'));
    expect(
      log.single,
      endsWith(
        'For performance reasons, the framework only generates semantics when asked to do so by the platform.\n'
        'Usually, platforms only ask for semantics when assistive technologies (like screen readers) are running.\n'
        'To generate semantics, try turning on an assistive technology (like VoiceOver or TalkBack) on your device.',
      ),
    );
    RendererBinding.instance.removeRenderView(RendererBinding.instance.renderView);
  });

  test('root pipeline owner cannot manage root node', () {
    final RenderObject rootNode = RenderProxyBox();
    expect(
      () => RendererBinding.instance.rootPipelineOwner.rootNode = rootNode,
      throwsA(
        isFlutterError.having(
          (FlutterError e) => e.message,
          'message',
          contains('Cannot set a rootNode on the default root pipeline owner.'),
        ),
      ),
    );
  });
}
