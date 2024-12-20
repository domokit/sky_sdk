// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;
import 'dart:typed_data';

import 'package:archive/archive.dart';
import 'package:archive/archive_io.dart';
import 'package:http/http.dart';
import 'package:path/path.dart' as path;

import 'common.dart';
import 'environment.dart';
import 'exceptions.dart';

const String _chromeExecutableVar = 'CHROME_EXECUTABLE';

/// Returns the installation of Chrome, installing it if necessary.
///
/// If [requestedVersion] is null, uses the version specified on the
/// command-line. If not specified on the command-line, uses the version
/// specified in the "package_lock.yaml" file.
///
/// If [requestedVersion] is not null, installs that version. The value
/// may be "latest" (the latest available build of Chrome), "system"
/// (manually installed Chrome on the current operating system), or an
/// exact build nuber, such as 695653. Build numbers can be found here:
///
/// https://commondatastorage.googleapis.com/chromium-browser-snapshots/index.html?prefix=Linux_x64/
Future<BrowserInstallation> getOrInstallChrome(
  String requestedVersion, {
  StringSink? infoLog,
}) async {
  infoLog ??= io.stdout;

  // When running on LUCI, if we specify the "chrome_and_driver" dependency,
  // then the bot will download Chrome from CIPD and place it in a cache and
  // set the environment variable CHROME_EXECUTABLE.
  if (io.Platform.environment.containsKey(_chromeExecutableVar)) {
    infoLog.writeln(
      'Using Chrome from $_chromeExecutableVar variable: '
      '${io.Platform.environment[_chromeExecutableVar]}',
    );
    return BrowserInstallation(
      version: 'cipd',
      executable: io.Platform.environment[_chromeExecutableVar]!,
    );
  }

  if (requestedVersion == 'system') {
    return BrowserInstallation(version: 'system', executable: await _findSystemChromeExecutable());
  }

  ChromeInstaller? installer;
  try {
    installer =
        requestedVersion == 'latest'
            ? await ChromeInstaller.latest()
            : ChromeInstaller(version: requestedVersion);

    if (installer.isInstalled) {
      infoLog.writeln(
        'Installation was skipped because Chrome version ${installer.version} is already installed.',
      );
    } else {
      infoLog.writeln('Installing Chrome version: ${installer.version}');
      await installer.install();
      final BrowserInstallation installation = installer.getInstallation()!;
      infoLog.writeln('Installations complete. To launch it run ${installation.executable}');
    }
    return installer.getInstallation()!;
  } finally {
    installer?.close();
  }
}

Future<String> _findSystemChromeExecutable() async {
  final io.ProcessResult which = await io.Process.run('which', <String>['google-chrome']);

  if (which.exitCode != 0) {
    throw BrowserInstallerException('Failed to locate system Chrome installation.');
  }

  return which.stdout as String;
}

/// Manages the installation of a particular [version] of Chrome.
class ChromeInstaller {
  factory ChromeInstaller({required String version}) {
    if (version == 'system') {
      throw BrowserInstallerException(
        'Cannot install system version of Chrome. System Chrome must be installed manually.',
      );
    }
    if (version == 'latest') {
      throw BrowserInstallerException(
        'Expected a concrete Chromer version, but got $version. Maybe use ChromeInstaller.latest()?',
      );
    }
    final io.Directory chromeInstallationDir = io.Directory(
      path.join(environment.webUiDartToolDir.path, 'chrome'),
    );
    final io.Directory versionDir = io.Directory(path.join(chromeInstallationDir.path, version));
    return ChromeInstaller._(
      version: version,
      chromeInstallationDir: chromeInstallationDir,
      versionDir: versionDir,
    );
  }

  ChromeInstaller._({
    required this.version,
    required this.chromeInstallationDir,
    required this.versionDir,
  });

  static Future<ChromeInstaller> latest() async {
    final String latestVersion = await fetchLatestChromeVersion();
    return ChromeInstaller(version: latestVersion);
  }

  /// Chrome version managed by this installer.
  final String version;

  /// HTTP client used to download Chrome.
  final Client client = Client();

  /// Root directory that contains Chrome versions.
  final io.Directory chromeInstallationDir;

  /// Installation directory for Chrome of the requested [version].
  final io.Directory versionDir;

  bool get isInstalled {
    return versionDir.existsSync();
  }

  BrowserInstallation? getInstallation() {
    if (!isInstalled) {
      return null;
    }

    return BrowserInstallation(
      version: version,
      executable: PlatformBinding.instance.getChromeExecutablePath(versionDir),
    );
  }

  Future<void> install() async {
    if (isLuci) {
      throw StateError(
        'Rejecting attempt to install Chromium on LUCI. LUCI is expected to '
        'provide Chromium as a CIPD dependency managed using .ci.yaml.',
      );
    }

    if (versionDir.existsSync()) {
      versionDir.deleteSync(recursive: true);
    }
    versionDir.createSync(recursive: true);

    final String url = PlatformBinding.instance.getChromeDownloadUrl(version);
    print('Downloading Chrome from $url');

    final StreamedResponse download = await client.send(Request('GET', Uri.parse(url)));

    final io.File downloadedFile = io.File(path.join(versionDir.path, 'chrome.zip'));
    await download.stream.pipe(downloadedFile.openWrite());

    /// Windows LUCI bots does not have a `unzip`. Instead we are
    /// using `archive` pub package.
    ///
    /// We didn't use `archive` on Mac/Linux since the new files have
    /// permission issues. For now we are not able change file permissions
    /// from dart.
    /// See: https://github.com/dart-lang/sdk/issues/15078.
    if (io.Platform.isWindows) {
      final Stopwatch stopwatch = Stopwatch()..start();

      // Read the Zip file from disk.
      final Uint8List bytes = downloadedFile.readAsBytesSync();

      final Archive archive = ZipDecoder().decodeBytes(bytes);

      // Extract the contents of the Zip archive to disk.
      for (final ArchiveFile file in archive) {
        final String filename = file.name;
        if (file.isFile) {
          final List<int> data = file.content as List<int>;
          io.File(
              path.joinAll(<String>[
                versionDir.path,
                // Remove the "chrome-win/" path prefix, which is the Windows
                // convention for Chromium directory structure.
                ...path.split(filename).skip(1),
              ]),
            )
            ..createSync(recursive: true)
            ..writeAsBytesSync(data);
        }
      }

      stopwatch.stop();
      print('The unzip took ${stopwatch.elapsedMilliseconds ~/ 1000} seconds.');
    } else {
      // We have to unzip into a temporary directory and then copy the files
      // out because our tests expect the files to be direct children of the
      // version directory. However, the zip file contains a top-level directory
      // named e.g. 'chrome-linux'. We need to copy the files out of that
      // directory and into the version directory.
      final io.Directory tmpDir = await chromeInstallationDir.createTemp();
      final io.Directory unzipDir = tmpDir;
      final io.ProcessResult unzipResult = await io.Process.run('unzip', <String>[
        downloadedFile.path,
        '-d',
        unzipDir.path,
      ]);
      if (unzipResult.exitCode != 0) {
        throw BrowserInstallerException(
          'Failed to unzip the downloaded Chrome archive ${downloadedFile.path}.\n'
          'With the version path ${versionDir.path}\n'
          'The unzip process exited with code ${unzipResult.exitCode}.',
        );
      }

      final io.Directory topLevelDir = await tmpDir.list().single as io.Directory;
      await for (final io.FileSystemEntity entity in topLevelDir.list()) {
        await entity.rename(path.join(versionDir.path, path.basename(entity.path)));
      }
      await tmpDir.delete(recursive: true);
    }

    downloadedFile.deleteSync();
  }

  void close() {
    client.close();
  }
}

/// Fetches the latest available Chrome build version.
Future<String> fetchLatestChromeVersion() async {
  final Client client = Client();
  try {
    final Response response = await client.get(
      Uri.parse(
        'https://www.googleapis.com/download/storage/v1/b/chromium-browser-snapshots/o/Linux_x64%2FLAST_CHANGE?alt=media',
      ),
    );
    if (response.statusCode != 200) {
      throw BrowserInstallerException(
        'Failed to fetch latest Chrome version. Server returned status code ${response.statusCode}',
      );
    }
    return response.body;
  } finally {
    client.close();
  }
}
