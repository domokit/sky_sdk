// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:http/http.dart';
import 'package:path/path.dart' as path;

import 'common.dart';
import 'environment.dart';
import 'package_lock.dart';

/// Returns the installation of Edge.
///
/// Currently uses the Edge version installed on the operating system.
///
/// As explained on the Microsoft help page: `Microsoft Edge comes
/// exclusively with Windows 10 and cannot be downloaded or installed separately.`
/// See: https://support.microsoft.com/en-us/help/17171/microsoft-edge-get-to-know
///
// TODO(yjbanov): Investigate running tests for the tech preview downloads
// from the beta channel.
Future<BrowserInstallation> getEdgeInstallation(
  String requestedVersion, {
  StringSink? infoLog,
}) async {
  // For now these tests are aimed to run only on Windows machines local or on LUCI/CI.
  // In the future we can investigate to run them on Android or on MacOS.
  if (!io.Platform.isWindows) {
    throw UnimplementedError(
      'Tests for Edge on ${io.Platform.operatingSystem} is'
      ' not supported.',
    );
  }

  infoLog ??= io.stdout;

  if (requestedVersion == 'system') {
    // Since Edge is included in Windows, always assume there will be one on the
    // system.
    infoLog.writeln('Using the system version that is already installed.');
    final EdgeLauncher edgeLauncher = EdgeLauncher();
    if (edgeLauncher.isInstalled) {
      infoLog.writeln('Launcher installation was skipped, already installed.');
    } else {
      infoLog.writeln('Installing MicrosoftEdgeLauncher');
      await edgeLauncher.install();
      infoLog.writeln('Installations complete. To launch it run ${edgeLauncher.executable}');
    }

    return BrowserInstallation(
      version: 'system',
      executable:
          io.Directory(
            path.join(
              edgeLauncher.launcherInstallationDir.path,
              PlatformBinding.instance.getCommandToRunEdge(),
            ),
          ).path,
    );
  } else {
    infoLog.writeln('Unsupported version $requestedVersion.');
    throw UnimplementedError();
  }
}

/// `MicrosoftEdgeLauncher` is an executable for launching Edge.
///
/// It is useful for starting Edge from comand line or from a
/// batch script.
///
/// See: https://github.com/MicrosoftEdge/edge-launcher
class EdgeLauncher {
  EdgeLauncher();

  /// Path to the directory that contains `MicrosoftEdgeLauncher.exe`.
  io.Directory get launcherInstallationDir =>
      io.Directory(path.join(environment.webUiDartToolDir.path, 'microsoftedgelauncher', version));

  io.File get executable =>
      io.File(path.join(launcherInstallationDir.path, 'MicrosoftEdgeLauncher.exe'));

  bool get isInstalled => executable.existsSync();

  /// Version number launcher executable  `MicrosoftEdgeLauncher`.
  String get version => packageLock.edgeLock.launcherVersion;

  /// Url for downloading  `MicrosoftEdgeLauncher`.
  ///
  /// Only useful in Windows, hence not added to [PlatformBinding].
  String get windowsEdgeLauncherDownloadUrl =>
      'https://github.com/MicrosoftEdge/edge-launcher/releases/download/$version/MicrosoftEdgeLauncher.exe';

  /// Install the launcher if it does not exist in this system.
  Future<void> install() async {
    // Checks if the  `MicrosoftEdgeLauncher` executable exists.
    if (isInstalled) {
      return;
    }

    // Create directory for download.
    if (!launcherInstallationDir.existsSync()) {
      launcherInstallationDir.createSync(recursive: true);
    }

    final Client client = Client();

    try {
      // Download executable from Github.
      final StreamedResponse download = await client.send(
        Request('GET', Uri.parse(windowsEdgeLauncherDownloadUrl)),
      );

      await download.stream.pipe(executable.openWrite());
    } finally {
      client.close();
    }
  }
}
