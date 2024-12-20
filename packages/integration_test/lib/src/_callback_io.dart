// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show Platform;
import 'dart:ui';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../common.dart';
import 'channel.dart';

/// The dart:io implementation of [CallbackManager].
///
/// See also:
///
///  * `_callback_web.dart`, which has the web implementation
CallbackManager get callbackManager => _singletonCallbackManager;

/// IOCallbackManager singleton.
final _IOCallbackManager _singletonCallbackManager = _IOCallbackManager();

/// Manages communication between `integration_tests` and the `driver_tests`.
///
/// This is the dart:io implementation.
class _IOCallbackManager implements CallbackManager {
  @override
  Future<Map<String, dynamic>> callback(
    Map<String, String> params,
    IntegrationTestResults testRunner,
  ) async {
    final String command = params['command']!;
    Map<String, String> response;
    switch (command) {
      case 'request_data':
        final bool allTestsPassed = await testRunner.allTestsPassed.future;
        response = <String, String>{
          'message':
              allTestsPassed
                  ? Response.allTestsPassed(data: testRunner.reportData).toJson()
                  : Response.someTestsFailed(
                    testRunner.failureMethodsDetails,
                    data: testRunner.reportData,
                  ).toJson(),
        };
      case 'get_health':
        response = <String, String>{'status': 'ok'};
      default:
        throw UnimplementedError('$command is not implemented');
    }
    return <String, dynamic>{'isError': false, 'response': response};
  }

  @override
  void cleanup() {
    // no-op.
    // Add any IO platform specific Completer/Future cleanups to here if any
    // comes up in the future. For example: `WebCallbackManager.cleanup`.
  }

  // [convertFlutterSurfaceToImage] has been called and [takeScreenshot] is ready to capture the surface (Android only).
  bool _isSurfaceRendered = false;

  @override
  Future<void> convertFlutterSurfaceToImage() async {
    if (!Platform.isAndroid) {
      // No-op on other platforms.
      return;
    }
    assert(!_isSurfaceRendered, 'Surface already converted to an image');
    await integrationTestChannel.invokeMethod<void>('convertFlutterSurfaceToImage');
    _isSurfaceRendered = true;

    addTearDown(() async {
      assert(_isSurfaceRendered, 'Surface is not an image');
      await integrationTestChannel.invokeMethod<void>('revertFlutterImage');
      _isSurfaceRendered = false;
    });
  }

  @override
  Future<Map<String, dynamic>> takeScreenshot(
    String screenshot, [
    Map<String, Object?>? args,
  ]) async {
    assert(args == null, '[args] handling has not been implemented for this platform');
    if (Platform.isAndroid && !_isSurfaceRendered) {
      throw StateError('Call convertFlutterSurfaceToImage() before taking a screenshot');
    }
    integrationTestChannel.setMethodCallHandler(_onMethodChannelCall);
    final List<int>? rawBytes = await integrationTestChannel.invokeMethod<List<int>>(
      'captureScreenshot',
      <String, dynamic>{'name': screenshot},
    );
    if (rawBytes == null) {
      throw StateError('Expected a list of bytes, but instead captureScreenshot returned null');
    }
    return <String, dynamic>{'screenshotName': screenshot, 'bytes': rawBytes};
  }

  Future<dynamic> _onMethodChannelCall(MethodCall call) async {
    switch (call.method) {
      case 'scheduleFrame':
        PlatformDispatcher.instance.scheduleFrame();
    }
    return null;
  }
}
