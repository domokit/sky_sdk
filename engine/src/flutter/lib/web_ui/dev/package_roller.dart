// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:http/http.dart';
import 'package:path/path.dart' as path;

import 'cipd.dart';
import 'common.dart';
import 'package_lock.dart';
import 'utils.dart';

final ArgParser _argParser =
    ArgParser(allowTrailingOptions: false)
      ..addFlag(
        'dry-run',
        help:
            'Whether or not to push changes to CIPD. When --dry-run is set, the '
            'script will download everything and attempt to prepare the bundle '
            'but will stop before publishing. When not set, the bundle will be '
            'published.',
        negatable: false,
      )
      ..addFlag('verbose', abbr: 'v', help: 'Enable verbose output.', negatable: false);

late final bool dryRun;
late final bool verbose;

final Client _client = Client();

/// Rolls browser CIPD packages to the version specified in `package_lock.yaml`.
///
/// Currently only rolls Chrome.
///
/// Chrome rolls are consumed by the "chrome_and_driver" and "chrome" LUCI recipes, here:
/// * https://cs.opensource.google/flutter/recipes/+/main:recipe_modules/flutter_deps/api.py;l=146
/// * https://cs.opensource.google/flutter/recipes/+/master:recipe_modules/web_util/api.py;l=22
///
/// Chromedriver is consumed by the same "chrome_and_driver" LUCI recipe, but also "chrome_driver":
/// * https://cs.opensource.google/flutter/recipes/+/master:recipe_modules/web_util/api.py;l=48
///
/// There's a small difference in the layout of the zip file coming from CIPD for the
/// Mac platform. In `Linux` and `Windows`, the chrome(.exe) executable is expected
/// to be placed directly in the root of the zip file.
///
/// However in `Mac`, the `Chromium.app` is expected to be placed inside of a
/// `chrome-mac` directory in the resulting zip file.
///
/// This script respects that historical quirk when building the CIPD packages.
/// In order for all the packages to be the same, the recipes listed above should
/// be made slightly smarter, so they can find the CHROME_EXECUTABLE in the right
/// place.
///
/// All platforms expect the "chromedriver" executable to be placed in the root
/// of the CIPD zip.
Future<void> main(List<String> args) async {
  try {
    processArgs(_argParser.parse(args));
    await _PackageRoller().roll();
    io.exitCode = 0;
  } on FormatException catch (e) {
    print('''
Error! ${e.message}

Available options:

${_argParser.usage}
''');
    io.exitCode = 1;
  } finally {
    _client.close();
  }
}

// Initialize globals from the parsed command-line arguments.
void processArgs(ArgResults args) {
  dryRun = args['dry-run'] as bool;
  verbose = args['verbose'] as bool;
}

class _Platform {
  _Platform(this.os, this.arch, this.binding);

  final String os;
  final String arch;
  final PlatformBinding binding;

  String get name => '$os-$arch';
}

class _PackageRoller {
  _PackageRoller();

  final io.Directory _rollDir = io.Directory.systemTemp.createTempSync('browser-roll-');

  final List<_Platform> _platforms = <_Platform>[
    _Platform('linux', 'amd64', LinuxPlatformBinding()),
    _Platform('mac', 'amd64', Macx64PlatformBinding()),
    _Platform('mac', 'arm64', MacArmPlatformBinding()),
    _Platform('windows', 'amd64', WindowsPlatformBinding()),
  ];

  final PackageLock _lock = PackageLock();

  // Prints output when --verbose is set.
  void vprint(String out) {
    if (verbose) {
      print(out);
    }
  }

  // Roll Chromium and ChromeDriver for each of the Platforms.
  Future<void> roll() async {
    for (final _Platform platform in _platforms) {
      await _rollChromium(platform);
      await _rollChromeDriver(platform);
      // For now, we only test Firefox on Linux.
      if (platform.os == 'linux') {
        await _rollFirefox(platform);
      }
      await _rollEsbuild(platform);
    }
    if (dryRun) {
      print('\nDry Run Done!\nNon-published roll artifacts kept here: ${_rollDir.path}\n');
    } else {
      // Clean-up
      vprint('\nDeleting temporary directory: ${_rollDir.path}');
      await _rollDir.delete(recursive: true);
      print('\nDone.\n');
    }
  }

  // Download a file from the internet, and put it in a temporary location.
  Future<io.File> _downloadTemporaryFile(String url) async {
    // Use the hash of the Url to temporarily store a file under tmp
    final io.File downloadedFile = io.File(
      path.join(io.Directory.systemTemp.path, 'download_${url.hashCode.toRadixString(16)}'),
    );
    vprint('  Downloading [$url] into [${downloadedFile.path}]');
    final StreamedResponse download = await _client.send(Request('GET', Uri.parse(url)));
    await download.stream.pipe(downloadedFile.openWrite());
    return downloadedFile;
  }

  // Unzips a `file` into a `destination` Directory (must exist).
  Future<void> _unzipAndDeleteFile(io.File zipFile, io.Directory destination) async {
    vprint('  Unzipping [${zipFile.path}] into [$destination]');
    await runProcess('unzip', <String>[
      if (!verbose) ...<String>['-q'],
      zipFile.path,
      '-d',
      destination.path,
    ]);
    vprint('  Deleting [${zipFile.path}]');
    await zipFile.delete();
  }

  // Uncompresses a `file` into a `destination` Directory (must exist).
  Future<void> _uncompressAndDeleteFile(io.File tarFile, io.Directory destination) async {
    vprint('  Uncompressing [${tarFile.path}] into [$destination]');
    final io.ProcessResult unzipResult = await io.Process.run('tar', <String>[
      '-x',
      '-f',
      tarFile.path,
      '-C',
      destination.path,
    ]);

    if (unzipResult.exitCode != 0) {
      throw StateError(
        'Failed to unzip the downloaded archive ${tarFile.path}.\n'
        'The unzip process exited with code ${unzipResult.exitCode}.',
      );
    }
    vprint('  Deleting [${tarFile.path}]');
    await tarFile.delete();
  }

  // Locate the first subdirectory that contains more than one file under `root`.
  // (or one ".app" bundle for mac)
  //
  // When uncompressing files, unzip might create some extra directories, but it
  // seems that our scripts want our CIPD packages to contain everything in the root.
  Future<io.Directory?> _locateContentRoot(io.Directory root) async {
    final List<io.FileSystemEntity> children = root.listSync(followLinks: false);
    assert(children.isNotEmpty);
    if (root.path.toLowerCase().endsWith('.app')) {
      // We've gone inside the .app bundle of the mac version!
      return root.parent;
    }
    if (children.length == 1) {
      if (children.first is io.Directory) {
        return _locateContentRoot(children.first as io.Directory);
      } else {
        return root;
      }
    }
    return root;
  }

  // Downloads Chromium from the internet, packs it in the directory structure
  // that the LUCI script wants. The result of this will be then uploaded to CIPD.
  Future<void> _rollChromium(_Platform platform) async {
    final String version = _lock.chromeLock.version;
    final String url = platform.binding.getChromeDownloadUrl(version);
    final String cipdPackageName = 'flutter_internal/browsers/chrome/${platform.name}';
    final io.Directory platformDir = io.Directory(path.join(_rollDir.path, platform.name));
    print('\nRolling Chromium for ${platform.name} (version:$version)');
    // Bail out if CIPD already has version:$majorVersion for this package!
    if (!dryRun &&
        await cipdKnowsPackageVersion(
          package: cipdPackageName,
          versionTag: version,
          isVerbose: verbose,
        )) {
      print('  Skipping $cipdPackageName version:$version. Already uploaded to CIPD!');
      vprint('  Update  package_lock.yaml  and use a different version value.');
      return;
    }

    await platformDir.create(recursive: true);
    vprint('  Created target directory [${platformDir.path}]');

    final io.File chromeDownload = await _downloadTemporaryFile(url);

    await _unzipAndDeleteFile(chromeDownload, platformDir);

    final io.Directory? actualContentRoot = await _locateContentRoot(platformDir);
    assert(actualContentRoot != null);
    final String relativePlatformDirPath = path.relative(
      actualContentRoot!.path,
      from: _rollDir.path,
    );

    vprint('  Uploading Chromium (${platform.name}) to CIPD...');
    await uploadDirectoryToCipd(
      directory: _rollDir,
      packageName: cipdPackageName,
      configFileName: 'cipd.chromium.${platform.name}.yaml',
      description: 'Chromium $version used for testing',
      version: version,
      root: relativePlatformDirPath,
      isDryRun: dryRun,
      isVerbose: verbose,
    );
  }

  // Downloads Chromedriver from the internet, packs it in the directory structure
  // that the LUCI script wants. The result of this will be then uploaded to CIPD.
  Future<void> _rollChromeDriver(_Platform platform) async {
    final String version = _lock.chromeLock.version;
    final String url = platform.binding.getChromeDriverDownloadUrl(version);
    final String cipdPackageName = 'flutter_internal/browser-drivers/chrome/${platform.name}';
    final io.Directory platformDir = io.Directory(
      path.join(_rollDir.path, '${platform.name}_driver'),
    );
    print('\nRolling Chromedriver for ${platform.os}-${platform.arch} (version:$version)');
    // Bail out if CIPD already has version:$majorVersion for this package!
    if (!dryRun &&
        await cipdKnowsPackageVersion(
          package: cipdPackageName,
          versionTag: version,
          isVerbose: verbose,
        )) {
      print('  Skipping $cipdPackageName version:$version. Already uploaded to CIPD!');
      vprint('  Update  package_lock.yaml  and use a different version value.');
      return;
    }

    await platformDir.create(recursive: true);
    vprint('  Created target directory [${platformDir.path}]');

    final io.File chromedriverDownload = await _downloadTemporaryFile(url);

    await _unzipAndDeleteFile(chromedriverDownload, platformDir);

    // Ensure the chromedriver executable is placed in the root of the bundle.
    final io.Directory? actualContentRoot = await _locateContentRoot(platformDir);
    assert(actualContentRoot != null);
    final String relativePlatformDirPath = path.relative(
      actualContentRoot!.path,
      from: _rollDir.path,
    );

    vprint('  Uploading Chromedriver (${platform.name}) to CIPD...');
    await uploadDirectoryToCipd(
      directory: _rollDir,
      packageName: cipdPackageName,
      configFileName: 'cipd.chromedriver.${platform.name}.yaml',
      description: 'Chromedriver for Chromium $version used for testing',
      version: version,
      root: relativePlatformDirPath,
      isDryRun: dryRun,
      isVerbose: verbose,
    );
  }

  // Downloads Firefox from the internet, packs it in the directory structure
  // that the LUCI script wants. The result of this will be then uploaded to CIPD.
  Future<void> _rollFirefox(_Platform platform) async {
    final String version = _lock.firefoxLock.version;
    final String url = platform.binding.getFirefoxDownloadUrl(version);
    final String cipdPackageName = 'flutter_internal/browsers/firefox/${platform.name}';
    final io.Directory platformDir = io.Directory(path.join(_rollDir.path, platform.name));
    print('\nRolling Firefox for ${platform.name} (version:$version)');
    // Bail out if CIPD already has version:$majorVersion for this package!
    if (!dryRun &&
        await cipdKnowsPackageVersion(
          package: cipdPackageName,
          versionTag: version,
          isVerbose: verbose,
        )) {
      print('  Skipping $cipdPackageName version:$version. Already uploaded to CIPD!');
      vprint('  Update  package_lock.yaml  and use a different version value.');
      return;
    }

    await platformDir.create(recursive: true);
    vprint('  Created target directory [${platformDir.path}]');

    final io.File firefoxDownload = await _downloadTemporaryFile(url);

    await _uncompressAndDeleteFile(firefoxDownload, platformDir);

    final io.Directory? actualContentRoot = await _locateContentRoot(platformDir);
    assert(actualContentRoot != null);
    final String relativePlatformDirPath = path.relative(
      actualContentRoot!.path,
      from: _rollDir.path,
    );

    vprint('  Uploading Firefox (${platform.name}) to CIPD...');
    await uploadDirectoryToCipd(
      directory: _rollDir,
      packageName: cipdPackageName,
      configFileName: 'cipd.firefox.${platform.name}.yaml',
      description: 'Firefox $version used for testing',
      version: version,
      root: relativePlatformDirPath,
      isDryRun: dryRun,
      isVerbose: verbose,
    );
  }

  Future<void> _rollEsbuild(_Platform platform) async {
    final String version = _lock.esbuildLock.version;
    final String url = platform.binding.getEsbuildDownloadUrl(version);
    final String cipdPackageName = 'flutter/tools/esbuild/${platform.name}';
    final io.Directory platformDir = io.Directory(path.join(_rollDir.path, platform.name));
    print('\nRolling esbuild for ${platform.name} (version:$version)');
    // Bail out if CIPD already has version:$majorVersion for this package!
    if (!dryRun &&
        await cipdKnowsPackageVersion(
          package: cipdPackageName,
          versionTag: version,
          isVerbose: verbose,
        )) {
      print('  Skipping $cipdPackageName version:$version. Already uploaded to CIPD!');
      vprint('  Update  package_lock.yaml  and use a different version value.');
      return;
    }

    await platformDir.create(recursive: true);
    vprint('  Created target directory [${platformDir.path}]');

    final io.File esbuildDownload = await _downloadTemporaryFile(url);

    await _uncompressAndDeleteFile(esbuildDownload, platformDir);
    final String packageDir = path.join(platformDir.path, 'package');

    // Write out the license file from the github repo.
    // Copied from https://github.com/evanw/esbuild/blob/main/LICENSE.md
    final io.File licenseFile = io.File(path.join(packageDir, 'LICENSE.md'));
    licenseFile
      ..createSync()
      ..writeAsStringSync('''
MIT License

Copyright (c) 2020 Evan Wallace

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
''');

    vprint('  Uploading esbuild (${platform.name}) to CIPD...');
    await uploadDirectoryToCipd(
      directory: _rollDir,
      packageName: cipdPackageName,
      configFileName: 'cipd.esbuild.${platform.name}.yaml',
      description: 'esbuild used by the flutter engine for bundling JavaScript',
      version: version,
      root: packageDir,
      isDryRun: dryRun,
      isVerbose: verbose,
    );
  }
}
