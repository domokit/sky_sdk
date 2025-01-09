// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:js/js_util.dart' as js_util;
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' as engine;
import 'package:ui/src/engine/dom.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

@JS('_flutter')
external set _loader(JSAny? loader);
set loader(Object? l) => _loader = l?.toJSAnyShallow;

@JS('_flutter.loader.didCreateEngineInitializer')
external set didCreateEngineInitializer(JSFunction? callback);

void main() {
  // Prepare _flutter.loader.didCreateEngineInitializer, so it's ready in the page ASAP.
  loader = js_util.jsify(<String, Object>{
    'loader': <String, Object>{
      'didCreateEngineInitializer':
          () {
            print('not mocked');
          }.toJS,
    },
  });
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test(
    'bootstrapEngine calls _flutter.loader.didCreateEngineInitializer callback',
    () async {
      JSAny? engineInitializer;

      void didCreateEngineInitializerMock(JSAny? obj) {
        print('obj: $obj');
        engineInitializer = obj;
      }

      // Prepare the DOM for: _flutter.loader.didCreateEngineInitializer
      didCreateEngineInitializer = didCreateEngineInitializerMock.toJS;

      // Reset the engine
      engine.debugResetEngineInitializationState();

      await ui_web.bootstrapEngine(registerPlugins: () {}, runApp: () {});

      // Check that the object we captured is actually a loader
      expect(engineInitializer, isNotNull);
      expect(
        js_util.hasProperty(engineInitializer!, 'initializeEngine'),
        isTrue,
        reason: 'Missing FlutterEngineInitializer method: initializeEngine.',
      );
      expect(
        js_util.hasProperty(engineInitializer!, 'autoStart'),
        isTrue,
        reason: 'Missing FlutterEngineInitializer method: autoStart.',
      );
      // https://github.com/flutter/flutter/issues/160096
    },
    skip: ui_web.browser.isFirefox,
  );

  test(
    'bootstrapEngine does auto-start when _flutter.loader.didCreateEngineInitializer does not exist',
    () async {
      loader = null;

      bool pluginsRegistered = false;
      bool appRan = false;
      void registerPluginsMock() {
        pluginsRegistered = true;
      }

      void runAppMock() {
        appRan = true;
      }

      // Reset the engine
      engine.debugResetEngineInitializationState();

      await ui_web.bootstrapEngine(registerPlugins: registerPluginsMock, runApp: runAppMock);

      // Check that the object we captured is actually a loader
      expect(
        pluginsRegistered,
        isTrue,
        reason: 'Plugins should be immediately registered in autoStart mode.',
      );
      expect(appRan, isTrue, reason: 'App should run immediately in autoStart mode');

      // After starting the engine, the meta-generator tag should be on the page
      final DomElement? meta = domDocument.querySelector('meta[name=generator][content=Flutter]');
      expect(
        meta,
        isNotNull,
        reason: 'The generator meta-tag should be added when Flutter initializes its UI.',
      );
      // https://github.com/flutter/flutter/issues/160096
    },
    skip: ui_web.browser.isFirefox,
  );

  // We cannot test anymore, because by now the engine has registered some stuff that can't be rewound back.
  // Like the `ext.flutter.disassemble` developer extension.
}
