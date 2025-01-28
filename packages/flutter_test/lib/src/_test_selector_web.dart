// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:ui' as ui;
import 'dart:ui_web' as ui_web;

import 'package:stream_channel/stream_channel.dart';
import 'package:test_api/backend.dart';

import '_goldens_web.dart';
import 'goldens.dart';
import 'web.dart' as web;

// This file contains APIs that are used by the generated test harness for
// running flutter unit tests.

/// A `main` entry point for a test.
typedef EntryPoint = FutureOr<void> Function();

/// An entry point runner provided by a test config file
typedef EntryPointRunner = Future<void> Function(EntryPoint);

/// Metadata about a web test to run
typedef WebTest = ({EntryPoint entryPoint, EntryPointRunner? entryPointRunner, Uri goldensUri});

/// Gets the test selector set by the test bootstrapping logic
String get testSelector {
  final JSString? jsTestSelector = web.window.testSelector;
  if (jsTestSelector == null) {
    throw Exception('Test selector not set');
  }
  return jsTestSelector.toDart;
}

/// Runs a specific web test
Future<void> runWebTest(WebTest test) async {
  ui_web.debugEmulateFlutterTesterEnvironment = true;
  final Completer<void> completer = Completer<void>();
  await ui_web.bootstrapEngine(runApp: () => completer.complete());
  await completer.future;

  goldenFileComparator = HttpProxyGoldenComparator(test.goldensUri);

  /// This hard-codes the device pixel ratio to 3.0 and a 2400 x 1800 window
  /// size for the purposes of testing.
  ui_web.debugOverrideDevicePixelRatio(3.0);
  ui.window.debugPhysicalSizeOverride = const ui.Size(2400, 1800);

  final EntryPointRunner? entryPointRunner = test.entryPointRunner;
  final EntryPoint entryPoint = test.entryPoint;
  _internalBootstrapBrowserTest(() {
    return entryPointRunner != null ? () => entryPointRunner(entryPoint) : entryPoint;
  });
}

void _internalBootstrapBrowserTest(EntryPoint Function() getMain) {
  final StreamChannel<Object?> channel = _serializeSuite(getMain, hidePrints: false);
  _postMessageChannel().pipe(channel);
}

StreamChannel<Object?> _serializeSuite(EntryPoint Function() getMain, {bool hidePrints = true}) =>
    RemoteListener.start(getMain, hidePrints: hidePrints);

StreamChannel<Object?> _postMessageChannel() {
  final StreamChannelController<Object?> controller = StreamChannelController<Object?>(sync: true);
  final web.MessageChannel channel = web.MessageChannel();
  web.window.parent!.postMessage(
    'port'.toJS,
    web.window.location.origin,
    <JSObject>[channel.port2].toJS,
  );

  final JSFunction eventCallback =
      (web.Event event) {
        controller.local.sink.add(event.data.dartify());
      }.toJS;
  channel.port1.addEventListener('message'.toJS, eventCallback);
  channel.port1.start();
  controller.local.stream.listen(
    (Object? message) => channel.port1.postMessage(message.jsify()),
    onDone: () => channel.port1.removeEventListener('message'.toJS, eventCallback),
  );

  return controller.foreign;
}
