// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:dds/dds.dart';
import 'package:dds/dds_launcher.dart';
import 'package:meta/meta.dart';

import '../artifacts.dart';
import '../device.dart';
import '../globals.dart' as globals;
import 'io.dart' as io;
import 'logger.dart';

export 'package:dds/dds.dart'
    show DartDevelopmentServiceException, ExistingDartDevelopmentServiceException;

typedef DDSLauncherCallback =
    Future<DartDevelopmentServiceLauncher> Function({
      required Uri remoteVmServiceUri,
      Uri? serviceUri,
      bool enableAuthCodes,
      bool serveDevTools,
      Uri? devToolsServerAddress,
      bool enableServicePortFallback,
      List<String> cachedUserTags,
      String? dartExecutable,
      String? google3WorkspaceRoot,
    });

// TODO(fujino): This should be direct injected, rather than mutable global state.
/// Used by tests to override the DDS spawn behavior for mocking purposes.
@visibleForTesting
DDSLauncherCallback ddsLauncherCallback = DartDevelopmentServiceLauncher.start;

/// Helper class to launch a [dds.DartDevelopmentService]. Allows for us to
/// mock out this functionality for testing purposes.
class DartDevelopmentService with DartDevelopmentServiceLocalOperationsMixin {
  DartDevelopmentService({required Logger logger}) : _logger = logger;

  DartDevelopmentServiceLauncher? _ddsInstance;

  Uri? get uri => _ddsInstance?.uri ?? _existingDdsUri;
  Uri? _existingDdsUri;

  Future<void> get done => _completer.future;
  final Completer<void> _completer = Completer<void>();

  final Logger _logger;

  @override
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    int? ddsPort,
    bool? disableServiceAuthCodes,
    bool? ipv6,
    bool enableDevTools = true,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  }) async {
    assert(_ddsInstance == null);
    final Uri ddsUri = Uri(
      scheme: 'http',
      host:
          ((ipv6 ?? false) ? io.InternetAddress.loopbackIPv6 : io.InternetAddress.loopbackIPv4)
              .host,
      port: ddsPort ?? 0,
    );
    _logger.printTrace(
      'Launching a Dart Developer Service (DDS) instance at $ddsUri, '
      'connecting to VM service at $vmServiceUri.',
    );
    void completeFuture() {
      if (!_completer.isCompleted) {
        _completer.complete();
      }
    }

    try {
      _ddsInstance = await ddsLauncherCallback(
        remoteVmServiceUri: vmServiceUri,
        serviceUri: ddsUri,
        enableAuthCodes: disableServiceAuthCodes != true,
        // Enables caching of CPU samples collected during application startup.
        cachedUserTags: cacheStartupProfile ? const <String>['AppStartUp'] : const <String>[],
        devToolsServerAddress: devToolsServerAddress,
        google3WorkspaceRoot: google3WorkspaceRoot,
        dartExecutable: globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
      );
      unawaited(_ddsInstance!.done.whenComplete(completeFuture));
    } on DartDevelopmentServiceException catch (e) {
      _logger.printTrace('Warning: Failed to start DDS: ${e.message}');
      if (e is ExistingDartDevelopmentServiceException) {
        _existingDdsUri = e.ddsUri;
      }
      completeFuture();
      rethrow;
    }
  }

  void shutdown() => _ddsInstance?.shutdown();
}

/// Contains common functionality that can be used with any implementation of
/// [DartDevelopmentService].
mixin DartDevelopmentServiceLocalOperationsMixin {
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    int? ddsPort,
    bool? disableServiceAuthCodes,
    bool? ipv6,
    bool enableDevTools = true,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  });

  /// A convenience method used to create a [DartDevelopmentService] instance
  /// from a [DebuggingOptions] instance.
  Future<void> startDartDevelopmentServiceFromDebuggingOptions(
    Uri vmServiceUri, {
    required DebuggingOptions debuggingOptions,
  }) => startDartDevelopmentService(
    vmServiceUri,
    ddsPort: debuggingOptions.ddsPort,
    disableServiceAuthCodes: debuggingOptions.disableServiceAuthCodes,
    ipv6: debuggingOptions.ipv6,
    enableDevTools: debuggingOptions.enableDevTools,
    cacheStartupProfile: debuggingOptions.cacheStartupProfile,
    google3WorkspaceRoot: debuggingOptions.google3WorkspaceRoot,
    devToolsServerAddress: debuggingOptions.devToolsServerAddress,
  );
}
