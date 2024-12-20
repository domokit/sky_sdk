// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:path/path.dart' as path;

import 'browser.dart';
import 'chrome.dart';
import 'edge.dart';
import 'environment.dart';
import 'exceptions.dart';
import 'felt_config.dart';
import 'firefox.dart';
import 'safari_macos.dart';

/// The port number for debugging.
const int kDevtoolsPort = 12345;
const int kMaxScreenshotWidth = 1024;
const int kMaxScreenshotHeight = 1024;

abstract class PlatformBinding {
  static PlatformBinding get instance {
    return _instance ??= _createInstance();
  }

  static PlatformBinding? _instance;

  static PlatformBinding _createInstance() {
    if (io.Platform.isLinux) {
      return LinuxPlatformBinding();
    }
    if (io.Platform.isMacOS) {
      if (environment.isMacosArm) {
        return MacArmPlatformBinding();
      }
      return Macx64PlatformBinding();
    }
    if (io.Platform.isWindows) {
      return WindowsPlatformBinding();
    }
    throw UnsupportedError('${io.Platform.operatingSystem} is not supported');
  }

  String get chromePlatformString;

  String getChromeDownloadUrl(String version) =>
      'https://storage.googleapis.com/chrome-for-testing-public/$version/$chromePlatformString/chrome-$chromePlatformString.zip';

  String getChromeDriverDownloadUrl(String version) =>
      'https://storage.googleapis.com/chrome-for-testing-public/$version/$chromePlatformString/chromedriver-$chromePlatformString.zip';

  String getFirefoxDownloadUrl(String version);
  String getFirefoxDownloadFilename(String version);
  String getChromeExecutablePath(io.Directory versionDir);
  String getFirefoxExecutablePath(io.Directory versionDir);
  String getFirefoxLatestVersionUrl();
  String getMacApplicationLauncher();
  String getCommandToRunEdge();

  String getEsbuildDownloadUrl(String version) =>
      'https://registry.npmjs.org/@esbuild/$esbuildPlatformName/-/$esbuildPlatformName-$version.tgz';
  String get esbuildPlatformName;
}

class WindowsPlatformBinding extends PlatformBinding {
  @override
  String get chromePlatformString => 'win64';

  @override
  String getChromeExecutablePath(io.Directory versionDir) =>
      path.join(versionDir.path, 'chrome.exe');

  @override
  String getFirefoxDownloadUrl(String version) =>
      'https://download-installer.cdn.mozilla.net/pub/firefox/releases/$version/win64/en-US/'
      '${getFirefoxDownloadFilename(version)}';

  @override
  String getFirefoxDownloadFilename(String version) => 'firefox-$version.exe';

  @override
  String getFirefoxExecutablePath(io.Directory versionDir) =>
      path.join(versionDir.path, 'firefox', 'firefox');

  @override
  String getFirefoxLatestVersionUrl() =>
      'https://download.mozilla.org/?product=firefox-latest&os=win&lang=en-US';

  @override
  String getMacApplicationLauncher() =>
      throw UnsupportedError('Safari is not supported on Windows');

  @override
  String getCommandToRunEdge() => 'MicrosoftEdgeLauncher';

  @override
  String get esbuildPlatformName => 'win32-x64';
}

class LinuxPlatformBinding extends PlatformBinding {
  @override
  String get chromePlatformString => 'linux64';

  @override
  String getChromeExecutablePath(io.Directory versionDir) => path.join(versionDir.path, 'chrome');

  @override
  String getFirefoxDownloadUrl(String version) =>
      'https://download-installer.cdn.mozilla.net/pub/firefox/releases/$version/linux-x86_64/en-US/'
      '${getFirefoxDownloadFilename(version)}';

  @override
  String getFirefoxDownloadFilename(String version) => 'firefox-$version.tar.bz2';

  @override
  String getFirefoxExecutablePath(io.Directory versionDir) =>
      path.join(versionDir.path, 'firefox', 'firefox');

  @override
  String getFirefoxLatestVersionUrl() =>
      'https://download.mozilla.org/?product=firefox-latest&os=linux64&lang=en-US';

  @override
  String getMacApplicationLauncher() => throw UnsupportedError('Safari is not supported on Linux');

  @override
  String getCommandToRunEdge() => throw UnsupportedError('Edge is not supported on Linux');

  @override
  String get esbuildPlatformName => 'linux-x64';
}

abstract class MacPlatformBinding extends PlatformBinding {
  @override
  String getChromeExecutablePath(io.Directory versionDir) => path.join(
    versionDir.path,
    'Google Chrome for Testing.app',
    'Contents',
    'MacOS',
    'Google Chrome for Testing',
  );

  @override
  String getFirefoxDownloadUrl(String version) =>
      'https://download-installer.cdn.mozilla.net/pub/firefox/releases/$version/mac/en-US/'
      '${getFirefoxDownloadFilename(version)}';

  @override
  String getFirefoxDownloadFilename(String version) => 'Firefox $version.dmg';

  @override
  String getFirefoxExecutablePath(io.Directory versionDir) =>
      path.join(versionDir.path, 'Firefox.app', 'Contents', 'MacOS', 'firefox');

  @override
  String getFirefoxLatestVersionUrl() =>
      'https://download.mozilla.org/?product=firefox-latest&os=osx&lang=en-US';

  @override
  String getMacApplicationLauncher() => 'open';

  @override
  String getCommandToRunEdge() =>
      throw UnimplementedError('Tests for Edge are not implemented for MacOS.');
}

class MacArmPlatformBinding extends MacPlatformBinding {
  @override
  String get chromePlatformString => 'mac-arm64';

  @override
  String get esbuildPlatformName => 'darwin-arm64';
}

class Macx64PlatformBinding extends MacPlatformBinding {
  @override
  String get chromePlatformString => 'mac-x64';

  @override
  String get esbuildPlatformName => 'darwin-x64';
}

class BrowserInstallation {
  const BrowserInstallation({required this.version, required this.executable});

  /// Browser version.
  final String version;

  /// Path the browser executable.
  final String executable;
}

/// A string sink that swallows all input.
class DevNull implements StringSink {
  @override
  void write(Object? obj) {}

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {}

  @override
  void writeCharCode(int charCode) {}

  @override
  void writeln([Object? obj = '']) {}
}

enum LuciRealm { Prod, Staging, Try, Unknown }

class LuciConfig {
  LuciConfig(this.realm);

  factory LuciConfig.fromJson(String contextJson) {
    final json = jsonDecode(contextJson) as Map<String, Object?>;
    final LuciRealm realm = switch ((json['realm'] as Map<String, Object?>?)?['name']) {
      'flutter:prod' => LuciRealm.Prod,
      'flutter:staging' => LuciRealm.Staging,
      'flutter:try' => LuciRealm.Try,
      _ => LuciRealm.Unknown,
    };
    return LuciConfig(realm);
  }

  final LuciRealm realm;
}

final LuciConfig? luciConfig = () {
  final String? contextPath = io.Platform.environment['LUCI_CONTEXT'];
  if (contextPath == null) {
    return null;
  }
  return LuciConfig.fromJson(io.File(contextPath).readAsStringSync());
}();

/// Whether the felt command is running on LUCI.
bool get isLuci => io.Platform.environment['LUCI_CONTEXT'] != null;

/// Whether the felt command is running on one of the Continuous Integration
/// environements.
bool get isCi => isLuci;

final String gitRevision = () {
  final result = io.Process.runSync(
    'git',
    <String>['rev-parse', 'HEAD'],
    workingDirectory: path.join(environment.engineSrcDir.path, 'flutter'),
    stderrEncoding: utf8,
    stdoutEncoding: utf8,
  );
  if (result.exitCode != 0) {
    throw ToolExit(
      'Failed to get git revision. Exit code: ${result.exitCode} Error: ${result.stderr}',
    );
  }
  return (result.stdout as String).trim();
}();

const String kChrome = 'chrome';
const String kEdge = 'edge';
const String kFirefox = 'firefox';
const String kSafari = 'safari';

const List<String> kAllBrowserNames = <String>[kChrome, kEdge, kFirefox, kSafari];

/// Creates an environment for a browser.
///
/// The [browserName] matches the browser name passed as the `--browser` option.
BrowserEnvironment getBrowserEnvironment(BrowserName browserName, {required bool useDwarf}) {
  switch (browserName) {
    case BrowserName.chrome:
      return ChromeEnvironment(useDwarf: useDwarf);
    case BrowserName.edge:
      return EdgeEnvironment();
    case BrowserName.firefox:
      return FirefoxEnvironment();
    case BrowserName.safari:
      return SafariMacOsEnvironment();
  }
}
