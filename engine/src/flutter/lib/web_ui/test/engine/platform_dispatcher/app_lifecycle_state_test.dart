// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group(AppLifecycleState, () {
    test('listens to changes in view manager', () {
      final FlutterViewManager viewManager = FlutterViewManager(EnginePlatformDispatcher.instance);
      final AppLifecycleState state = AppLifecycleState.create(viewManager);

      ui.AppLifecycleState? currentState;
      void listener(ui.AppLifecycleState newState) {
        currentState = newState;
      }

      state.addListener(listener);

      final view1 = EngineFlutterView(EnginePlatformDispatcher.instance, createDomHTMLDivElement());
      viewManager.registerView(view1);
      expect(currentState, ui.AppLifecycleState.resumed);
      currentState = null;

      final view2 = EngineFlutterView(EnginePlatformDispatcher.instance, createDomHTMLDivElement());
      viewManager.registerView(view2);
      // The listener should not be called again. The view manager is still not empty.
      expect(currentState, isNull);

      viewManager.disposeAndUnregisterView(view1.viewId);
      // The listener should not be called again. The view manager is still not empty.
      expect(currentState, isNull);

      viewManager.disposeAndUnregisterView(view2.viewId);
      expect(currentState, ui.AppLifecycleState.detached);
      currentState = null;

      final view3 = EngineFlutterView(EnginePlatformDispatcher.instance, createDomHTMLDivElement());
      viewManager.registerView(view3);
      // The state should go back to `resumed` after a new view is registered.
      expect(currentState, ui.AppLifecycleState.resumed);

      viewManager.dispose();
      state.removeListener(listener);
    });
  });
}
