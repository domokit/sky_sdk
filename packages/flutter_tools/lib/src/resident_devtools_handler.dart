// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(bkonyi): remove this file when ready to serve DevTools from DDS.
//
// See https://github.com/flutter/flutter/issues/150044

import 'dart:async';

import 'package:browser_launcher/browser_launcher.dart';
import 'package:meta/meta.dart';

import 'base/logger.dart';
import 'build_info.dart';
import 'resident_runner.dart';
import 'vmservice.dart';

typedef ResidentDevtoolsHandlerFactory =
    ResidentDevtoolsHandler Function(DevtoolsLauncher?, ResidentRunner, Logger);

ResidentDevtoolsHandler createDefaultHandler(
  DevtoolsLauncher? launcher,
  ResidentRunner runner,
  Logger logger,
) {
  return FlutterResidentDevtoolsHandler(launcher, runner, logger);
}

/// Helper class to manage the life-cycle of devtools and its interaction with
/// the resident runner.
abstract class ResidentDevtoolsHandler {
  /// The current devtools server, or null if one is not running.
  DevToolsServerAddress? get activeDevToolsServer;

  /// The Dart Tooling Daemon (DTD) URI for the DTD instance being hosted by
  /// DevTools server.
  ///
  /// This will be null if the DevTools server is not served through Flutter
  /// tools (e.g. if it is served from an IDE).
  Uri? get dtdUri;

  /// Whether to print the Dart Tooling Daemon URI.
  ///
  /// This will always return false when there is not a DTD instance being
  /// served from the DevTools server.
  bool get printDtdUri;

  /// Whether it's ok to announce the [activeDevToolsServer].
  ///
  /// This should only return true once all the devices have been notified
  /// of the DevTools.
  bool get readyToAnnounce;

  Future<void> hotRestart(List<FlutterDevice?> flutterDevices);

  Future<void> serveAndAnnounceDevTools({
    Uri? devToolsServerAddress,
    required List<FlutterDevice?> flutterDevices,
    bool isStartPaused = false,
  });

  bool launchDevToolsInBrowser({required List<FlutterDevice?> flutterDevices});

  Future<void> shutdown();
}

class FlutterResidentDevtoolsHandler implements ResidentDevtoolsHandler {
  FlutterResidentDevtoolsHandler(this._devToolsLauncher, this._residentRunner, this._logger);

  static const Duration launchInBrowserTimeout = Duration(seconds: 15);

  final DevtoolsLauncher? _devToolsLauncher;
  final ResidentRunner _residentRunner;
  final Logger _logger;
  bool _shutdown = false;

  @visibleForTesting
  bool launchedInBrowser = false;

  @override
  DevToolsServerAddress? get activeDevToolsServer {
    assert(!_readyToAnnounce || _devToolsLauncher?.activeDevToolsServer != null);
    return _devToolsLauncher?.activeDevToolsServer;
  }

  @override
  Uri? get dtdUri => _devToolsLauncher?.dtdUri;

  @override
  bool get printDtdUri => _devToolsLauncher?.printDtdUri ?? false;

  @override
  bool get readyToAnnounce => _readyToAnnounce;
  bool _readyToAnnounce = false;

  // This must be guaranteed not to return a Future that fails.
  @override
  Future<void> serveAndAnnounceDevTools({
    Uri? devToolsServerAddress,
    required List<FlutterDevice?> flutterDevices,
    bool isStartPaused = false,
  }) async {
    assert(!_readyToAnnounce);
    if (!_residentRunner.supportsServiceProtocol || _devToolsLauncher == null) {
      return;
    }
    if (devToolsServerAddress != null) {
      _devToolsLauncher.devToolsUrl = devToolsServerAddress;
    } else {
      await _devToolsLauncher.serve();
    }
    await _devToolsLauncher.ready;
    // Do not attempt to print debugger list if the connection has failed or if we're shutting down.
    if (_devToolsLauncher.activeDevToolsServer == null || _shutdown) {
      assert(!_readyToAnnounce);
      return;
    }

    Future<void> callServiceExtensions() async {
      final List<FlutterDevice?> devicesWithExtension = await _devicesWithExtensions(
        flutterDevices,
      );
      await Future.wait(<Future<void>>[
        _maybeCallDevToolsUriServiceExtension(devicesWithExtension),
        _callConnectedVmServiceUriExtension(devicesWithExtension),
      ]);
    }

    // If the application is starting paused, we can't invoke service extensions
    // as they're handled on the target app's paused isolate. Since invoking
    // service extensions will block in this situation, we should wait to invoke
    // them until after we've output the DevTools connection details.
    if (!isStartPaused) {
      await callServiceExtensions();
    }

    // This check needs to happen after the possible asynchronous call above,
    // otherwise a shutdown event might be missed and the DevTools launcher may
    // no longer be initialized.
    if (_shutdown) {
      // If we're shutting down, no point reporting the debugger list.
      return;
    }

    _readyToAnnounce = true;
    assert(_devToolsLauncher.activeDevToolsServer != null);
    if (_residentRunner.reportedDebuggers) {
      // Since the DevTools only just became available, we haven't had a chance to
      // report their URLs yet. Do so now.
      _residentRunner.printDebuggerList(includeVmService: false);
    }

    if (isStartPaused) {
      await callServiceExtensions();
    }
  }

  // This must be guaranteed not to return a Future that fails.
  @override
  bool launchDevToolsInBrowser({required List<FlutterDevice?> flutterDevices}) {
    if (!_residentRunner.supportsServiceProtocol || _devToolsLauncher == null) {
      return false;
    }
    if (_devToolsLauncher.devToolsUrl == null) {
      _logger.startProgress('Waiting for Flutter DevTools to be served...');
      unawaited(
        _devToolsLauncher.ready.then((_) {
          _launchDevToolsForDevices(flutterDevices);
        }),
      );
    } else {
      _launchDevToolsForDevices(flutterDevices);
    }
    return true;
  }

  void _launchDevToolsForDevices(List<FlutterDevice?> flutterDevices) {
    assert(activeDevToolsServer != null);
    for (final FlutterDevice? device in flutterDevices) {
      final String devToolsUrl =
          activeDevToolsServer!.uri!
              .replace(
                queryParameters: <String, dynamic>{'uri': '${device!.vmService!.httpAddress}'},
              )
              .toString();
      _logger.printStatus('Launching Flutter DevTools for ${device.device!.name} at $devToolsUrl');
      unawaited(Chrome.start(<String>[devToolsUrl]));
    }
    launchedInBrowser = true;
  }

  Future<void> _maybeCallDevToolsUriServiceExtension(List<FlutterDevice?> flutterDevices) async {
    if (_devToolsLauncher?.activeDevToolsServer == null) {
      return;
    }
    await Future.wait(<Future<void>>[
      for (final FlutterDevice? device in flutterDevices)
        if (device?.vmService != null) _callDevToolsUriExtension(device!),
    ]);
  }

  Future<void> _callDevToolsUriExtension(FlutterDevice device) async {
    try {
      await _invokeRpcOnFirstView(
        'ext.flutter.activeDevToolsServerAddress',
        device: device,
        params: <String, dynamic>{'value': _devToolsLauncher!.activeDevToolsServer!.uri.toString()},
      );
    } on Exception catch (e) {
      if (!_shutdown) {
        _logger.printError(
          'Failed to set DevTools server address: $e. Deep links to'
          ' DevTools will not show in Flutter errors.',
        );
      }
    }
  }

  Future<List<FlutterDevice?>> _devicesWithExtensions(List<FlutterDevice?> flutterDevices) async {
    return Future.wait(<Future<FlutterDevice?>>[
      for (final FlutterDevice? device in flutterDevices) _waitForExtensionsForDevice(device!),
    ]);
  }

  /// Returns null if the service extension cannot be found on the device.
  Future<FlutterDevice?> _waitForExtensionsForDevice(FlutterDevice flutterDevice) async {
    const String extension = 'ext.flutter.connectedVmServiceUri';
    try {
      await flutterDevice.vmService?.findExtensionIsolate(extension);
      return flutterDevice;
    } on VmServiceDisappearedException {
      _logger.printTrace(
        'The VM Service for ${flutterDevice.device} disappeared while trying to'
        ' find the $extension service extension. Skipping subsequent DevTools '
        'setup for this device.',
      );
      return null;
    }
  }

  Future<void> _callConnectedVmServiceUriExtension(List<FlutterDevice?> flutterDevices) async {
    await Future.wait(<Future<void>>[
      for (final FlutterDevice? device in flutterDevices)
        if (device?.vmService != null) _callConnectedVmServiceExtension(device!),
    ]);
  }

  Future<void> _callConnectedVmServiceExtension(FlutterDevice device) async {
    final Uri? uri = device.vmService!.httpAddress ?? device.vmService!.wsAddress;
    if (uri == null) {
      return;
    }
    try {
      await _invokeRpcOnFirstView(
        'ext.flutter.connectedVmServiceUri',
        device: device,
        params: <String, dynamic>{'value': uri.toString()},
      );
    } on Exception catch (e) {
      if (!_shutdown) {
        _logger.printError(e.toString());
        _logger.printError(
          'Failed to set vm service URI: $e. Deep links to DevTools'
          ' will not show in Flutter errors.',
        );
      }
    }
  }

  Future<void> _invokeRpcOnFirstView(
    String method, {
    required FlutterDevice device,
    required Map<String, dynamic> params,
  }) async {
    if (device.targetPlatform == TargetPlatform.web_javascript) {
      await device.vmService!.callMethodWrapper(method, args: params);
      return;
    }
    final List<FlutterView> views = await device.vmService!.getFlutterViews();
    if (views.isEmpty) {
      return;
    }
    await device.vmService!.invokeFlutterExtensionRpcRaw(
      method,
      args: params,
      isolateId: views.first.uiIsolate!.id!,
    );
  }

  @override
  Future<void> hotRestart(List<FlutterDevice?> flutterDevices) async {
    final List<FlutterDevice?> devicesWithExtension = await _devicesWithExtensions(flutterDevices);
    await Future.wait(<Future<void>>[
      _maybeCallDevToolsUriServiceExtension(devicesWithExtension),
      _callConnectedVmServiceUriExtension(devicesWithExtension),
    ]);
  }

  @override
  Future<void> shutdown() async {
    _shutdown = true;
    if (_devToolsLauncher == null) {
      return;
    }
    _readyToAnnounce = false;
    await _devToolsLauncher.close();
  }
}

@visibleForTesting
NoOpDevtoolsHandler createNoOpHandler(
  DevtoolsLauncher? launcher,
  ResidentRunner runner,
  Logger logger,
) {
  return NoOpDevtoolsHandler();
}

@visibleForTesting
class NoOpDevtoolsHandler implements ResidentDevtoolsHandler {
  bool wasShutdown = false;

  @override
  DevToolsServerAddress? get activeDevToolsServer => null;

  @override
  bool get readyToAnnounce => false;

  @override
  Future<void> hotRestart(List<FlutterDevice?> flutterDevices) async {
    return;
  }

  @override
  Future<void> serveAndAnnounceDevTools({
    Uri? devToolsServerAddress,
    List<FlutterDevice?>? flutterDevices,
    bool isStartPaused = false,
  }) async {
    return;
  }

  @override
  bool launchDevToolsInBrowser({List<FlutterDevice?>? flutterDevices}) {
    return false;
  }

  @override
  Future<void> shutdown() async {
    wasShutdown = true;
    return;
  }

  @override
  Uri? get dtdUri => null;

  @override
  bool get printDtdUri => false;
}

/// Convert a [URI] with query parameters into a display format instead
/// of the default URI encoding.
String urlToDisplayString(Uri uri) {
  final StringBuffer base = StringBuffer(
    uri.replace(queryParameters: <String, String>{}).toString(),
  );
  base.write(
    uri.queryParameters.keys.map((String key) => '$key=${uri.queryParameters[key]}').join('&'),
  );
  return base.toString();
}
