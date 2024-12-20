// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:webkit_inspection_protocol/webkit_inspection_protocol.dart' hide StackTrace;

import '../base/async_guard.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../convert.dart';

/// An environment variable used to override the location of Google Chrome.
const String kChromeEnvironment = 'CHROME_EXECUTABLE';

/// An environment variable used to override the location of Microsoft Edge.
const String kEdgeEnvironment = 'EDGE_ENVIRONMENT';

/// The expected executable name on linux.
const String kLinuxExecutable = 'google-chrome';

/// The expected executable name on macOS.
const String kMacOSExecutable = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

/// The expected Chrome executable name on Windows.
const String kWindowsExecutable = r'Google\Chrome\Application\chrome.exe';

/// The expected Edge executable name on Windows.
const String kWindowsEdgeExecutable = r'Microsoft\Edge\Application\msedge.exe';

/// Used by [ChromiumLauncher] to detect a glibc bug and retry launching the
/// browser.
///
/// Once every few thousands of launches we hit this glibc bug:
///
/// https://sourceware.org/bugzilla/show_bug.cgi?id=19329.
///
/// When this happens Chrome spits out something like the following then exits with code 127:
///
///     Inconsistency detected by ld.so: ../elf/dl-tls.c: 493: _dl_allocate_tls_init: Assertion `listp->slotinfo[cnt].gen <= GL(dl_tls_generation)' failed!
const String _kGlibcError = 'Inconsistency detected by ld.so';

typedef BrowserFinder = String Function(Platform, FileSystem);

/// Find the chrome executable on the current platform.
///
/// Does not verify whether the executable exists.
String findChromeExecutable(Platform platform, FileSystem fileSystem) {
  if (platform.environment.containsKey(kChromeEnvironment)) {
    return platform.environment[kChromeEnvironment]!;
  }
  if (platform.isLinux) {
    return kLinuxExecutable;
  }
  if (platform.isMacOS) {
    return kMacOSExecutable;
  }
  if (platform.isWindows) {
    /// The possible locations where the chrome executable can be located on windows.
    final List<String> kWindowsPrefixes = <String>[
      if (platform.environment.containsKey('LOCALAPPDATA')) platform.environment['LOCALAPPDATA']!,
      if (platform.environment.containsKey('PROGRAMFILES')) platform.environment['PROGRAMFILES']!,
      if (platform.environment.containsKey('PROGRAMFILES(X86)'))
        platform.environment['PROGRAMFILES(X86)']!,
    ];
    final String windowsPrefix = kWindowsPrefixes.firstWhere((String prefix) {
      final String path = fileSystem.path.join(prefix, kWindowsExecutable);
      return fileSystem.file(path).existsSync();
    }, orElse: () => '.');
    return fileSystem.path.join(windowsPrefix, kWindowsExecutable);
  }
  throwToolExit('Platform ${platform.operatingSystem} is not supported.');
}

/// Find the Microsoft Edge executable on the current platform.
///
/// Does not verify whether the executable exists.
String findEdgeExecutable(Platform platform, FileSystem fileSystem) {
  if (platform.environment.containsKey(kEdgeEnvironment)) {
    return platform.environment[kEdgeEnvironment]!;
  }
  if (platform.isWindows) {
    /// The possible locations where the Edge executable can be located on windows.
    final List<String> kWindowsPrefixes = <String>[
      if (platform.environment.containsKey('LOCALAPPDATA')) platform.environment['LOCALAPPDATA']!,
      if (platform.environment.containsKey('PROGRAMFILES')) platform.environment['PROGRAMFILES']!,
      if (platform.environment.containsKey('PROGRAMFILES(X86)'))
        platform.environment['PROGRAMFILES(X86)']!,
    ];
    final String windowsPrefix = kWindowsPrefixes.firstWhere((String prefix) {
      final String path = fileSystem.path.join(prefix, kWindowsEdgeExecutable);
      return fileSystem.file(path).existsSync();
    }, orElse: () => '.');
    return fileSystem.path.join(windowsPrefix, kWindowsEdgeExecutable);
  }
  // Not yet supported for macOS and Linux.
  return '';
}

/// A launcher for Chromium browsers with devtools configured.
class ChromiumLauncher {
  ChromiumLauncher({
    required FileSystem fileSystem,
    required Platform platform,
    required ProcessManager processManager,
    required OperatingSystemUtils operatingSystemUtils,
    required BrowserFinder browserFinder,
    required Logger logger,
  }) : _fileSystem = fileSystem,
       _platform = platform,
       _processManager = processManager,
       _operatingSystemUtils = operatingSystemUtils,
       _browserFinder = browserFinder,
       _logger = logger;

  final FileSystem _fileSystem;
  final Platform _platform;
  final ProcessManager _processManager;
  final OperatingSystemUtils _operatingSystemUtils;
  final BrowserFinder _browserFinder;
  final Logger _logger;

  bool get hasChromeInstance => currentCompleter.isCompleted;

  @visibleForTesting
  Completer<Chromium> currentCompleter = Completer<Chromium>();

  /// Whether we can locate the chrome executable.
  bool canFindExecutable() {
    final String chrome = _browserFinder(_platform, _fileSystem);
    try {
      return _processManager.canRun(chrome);
    } on ArgumentError {
      return false;
    }
  }

  /// The executable this launcher will use.
  String findExecutable() => _browserFinder(_platform, _fileSystem);

  /// Launch a Chromium browser to a particular `host` page.
  ///
  /// [headless] defaults to false, and controls whether we open a headless or
  /// a "headful" browser.
  ///
  /// [debugPort] is Chrome's debugging protocol port. If null, a random free
  /// port is picked automatically.
  ///
  /// [skipCheck] does not attempt to make a devtools connection before returning.
  ///
  /// [webBrowserFlags] add arbitrary browser flags.
  Future<Chromium> launch(
    String url, {
    bool headless = false,
    int? debugPort,
    bool skipCheck = false,
    Directory? cacheDir,
    List<String> webBrowserFlags = const <String>[],
  }) async {
    if (currentCompleter.isCompleted) {
      throwToolExit('Only one instance of chrome can be started.');
    }

    if (_logger.isVerbose) {
      _logger.printTrace(
        'Launching Chromium (url = $url, headless = $headless, skipCheck = $skipCheck, debugPort = $debugPort)',
      );
    }

    final String chromeExecutable = _browserFinder(_platform, _fileSystem);

    if (_logger.isVerbose) {
      _logger.printTrace('Will use Chromium executable at $chromeExecutable');

      if (!_platform.isWindows) {
        // The "--version" argument is not supported on Windows.
        final ProcessResult versionResult = await _processManager.run(<String>[
          chromeExecutable,
          '--version',
        ]);
        _logger.printTrace('Using ${versionResult.stdout}');
      }
    }

    final Directory userDataDir = _fileSystem.systemTempDirectory.createTempSync(
      'flutter_tools_chrome_device.',
    );

    if (cacheDir != null) {
      // Seed data dir with previous state.
      _restoreUserSessionInformation(cacheDir, userDataDir);
    }

    final int port = debugPort ?? await _operatingSystemUtils.findFreePort();
    final List<String> args = <String>[
      chromeExecutable,
      // Using a tmp directory ensures that a new instance of chrome launches
      // allowing for the remote debug port to be enabled.
      '--user-data-dir=${userDataDir.path}',
      '--remote-debugging-port=$port',
      // When the DevTools has focus we don't want to slow down the application.
      '--disable-background-timer-throttling',
      // Since we are using a temp profile, disable features that slow the
      // Chrome launch.
      '--disable-extensions',
      '--disable-popup-blocking',
      '--bwsi',
      '--no-first-run',
      '--no-default-browser-check',
      '--disable-default-apps',
      '--disable-translate',

      // Remove the search engine choice screen. It's irrelevant for app
      // debugging purposes.
      // See: https://github.com/flutter/flutter/issues/153928
      '--disable-search-engine-choice-screen',

      if (headless) ...<String>[
        '--headless',
        '--disable-gpu',
        '--no-sandbox',
        '--window-size=2400,1800',
      ],
      ...webBrowserFlags,
      url,
    ];

    final Process process = await _spawnChromiumProcess(args, chromeExecutable);

    // When the process exits, copy the user settings back to the provided data-dir.
    if (cacheDir != null) {
      unawaited(
        process.exitCode.whenComplete(() {
          _cacheUserSessionInformation(userDataDir, cacheDir);
          // cleanup temp dir
          try {
            userDataDir.deleteSync(recursive: true);
          } on FileSystemException {
            // ignore
          }
        }),
      );
    }
    return connect(
      Chromium(
        port,
        ChromeConnection('localhost', port),
        url: url,
        process: process,
        chromiumLauncher: this,
        logger: _logger,
      ),
      skipCheck,
    );
  }

  Future<Process> _spawnChromiumProcess(List<String> args, String chromeExecutable) async {
    if (_operatingSystemUtils.hostPlatform == HostPlatform.darwin_arm64) {
      final ProcessResult result = _processManager.runSync(<String>['file', chromeExecutable]);
      // Check if ARM Chrome is installed.
      // Mach-O 64-bit executable arm64
      if ((result.stdout as String).contains('arm64')) {
        _logger.printTrace(
          'Found ARM Chrome installation at $chromeExecutable, forcing native launch.',
        );
        // If so, force Chrome to launch natively.
        args.insertAll(0, <String>['/usr/bin/arch', '-arm64']);
      }
    }

    // Keep attempting to launch the browser until one of:
    // - Chrome launched successfully, in which case we just return from the loop.
    // - The tool reached the maximum retry count, in which case we throw ToolExit.
    const int kMaxRetries = 3;
    int retry = 0;
    while (true) {
      final Process process = await _processManager.start(args);

      process.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((String line) {
        _logger.printTrace('[CHROME]: $line');
      });

      // Wait until the DevTools are listening before trying to connect. This is
      // only required for flutter_test --platform=chrome and not flutter run.
      bool hitGlibcBug = false;
      bool shouldRetry = false;
      final List<String> errors = <String>[];
      await process.stderr
          .transform(utf8.decoder)
          .transform(const LineSplitter())
          .map((String line) {
            _logger.printTrace('[CHROME]: $line');
            errors.add('[CHROME]:$line');
            if (line.contains(_kGlibcError)) {
              hitGlibcBug = true;
              shouldRetry = true;
            }
            return line;
          })
          .firstWhere(
            (String line) => line.startsWith('DevTools listening'),
            orElse: () {
              if (hitGlibcBug) {
                _logger.printTrace(
                  'Encountered glibc bug https://sourceware.org/bugzilla/show_bug.cgi?id=19329. '
                  'Will try launching browser again.',
                );
                // Return value unused.
                return '';
              }
              if (retry >= kMaxRetries) {
                errors.forEach(_logger.printError);
                _logger.printError(
                  'Failed to launch browser after $kMaxRetries tries. Command used to launch it: ${args.join(' ')}',
                );
                throwToolExit(
                  'Failed to launch browser. Make sure you are using an up-to-date '
                  'Chrome or Edge. Otherwise, consider using -d web-server instead '
                  'and filing an issue at https://github.com/flutter/flutter/issues.',
                );
              }
              shouldRetry = true;
              return '';
            },
          );

      if (!hitGlibcBug && !shouldRetry) {
        return process;
      }
      retry += 1;

      // A precaution that avoids accumulating browser processes, in case the
      // glibc bug doesn't cause the browser to quit and we keep looping and
      // launching more processes.
      unawaited(
        process.exitCode.timeout(
          const Duration(seconds: 1),
          onTimeout: () {
            process.kill();
            // sigterm
            return 15;
          },
        ),
      );
    }
  }

  // This is a directory which Chrome uses to store cookies, preferences and
  // other session data.
  String get _chromeDefaultPath => _fileSystem.path.join('Default');

  // This is a JSON file which contains configuration from the browser session,
  // such as window position. It is located under the Chrome data-dir folder.
  String get _preferencesPath => _fileSystem.path.join('Default', 'preferences');

  /// Copy Chrome user information from a Chrome session into a per-project
  /// cache.
  ///
  /// More detailed docs of the Chrome user preferences store exists here:
  /// https://www.chromium.org/developers/design-documents/preferences.
  ///
  /// This intentionally skips the Cache, Code Cache, and GPUCache directories.
  /// While we're not sure exactly what is in them, this constitutes nearly 1 GB
  /// of data for a fresh flutter run and adds significant overhead to all startups.
  /// For workflows that may require this data, using the start-paused flag and
  /// dart debug extension with a user controlled browser profile will lead to a
  /// better experience.
  void _cacheUserSessionInformation(Directory userDataDir, Directory cacheDir) {
    final Directory targetChromeDefault = _fileSystem.directory(
      _fileSystem.path.join(cacheDir.path, _chromeDefaultPath),
    );
    final Directory sourceChromeDefault = _fileSystem.directory(
      _fileSystem.path.join(userDataDir.path, _chromeDefaultPath),
    );
    if (sourceChromeDefault.existsSync()) {
      targetChromeDefault.createSync(recursive: true);
      try {
        copyDirectory(
          sourceChromeDefault,
          targetChromeDefault,
          shouldCopyDirectory: _isNotCacheDirectory,
        );
      } on FileSystemException catch (err) {
        // This is a best-effort update. Display the message in case the failure is relevant.
        // one possible example is a file lock due to multiple running chrome instances.
        _logger.printError('Failed to save Chrome preferences: $err');
      }
    }

    final File targetPreferencesFile = _fileSystem.file(
      _fileSystem.path.join(cacheDir.path, _preferencesPath),
    );
    final File sourcePreferencesFile = _fileSystem.file(
      _fileSystem.path.join(userDataDir.path, _preferencesPath),
    );

    if (sourcePreferencesFile.existsSync()) {
      targetPreferencesFile.parent.createSync(recursive: true);
      // If the file contains a crash string, remove it to hide the popup on next run.
      final String contents = sourcePreferencesFile.readAsStringSync();
      targetPreferencesFile.writeAsStringSync(
        contents.replaceFirst('"exit_type":"Crashed"', '"exit_type":"Normal"'),
      );
    }
  }

  /// Restore Chrome user information from a per-project cache into Chrome's
  /// user data directory.
  void _restoreUserSessionInformation(Directory cacheDir, Directory userDataDir) {
    final Directory sourceChromeDefault = _fileSystem.directory(
      _fileSystem.path.join(cacheDir.path, _chromeDefaultPath),
    );
    final Directory targetChromeDefault = _fileSystem.directory(
      _fileSystem.path.join(userDataDir.path, _chromeDefaultPath),
    );
    try {
      if (sourceChromeDefault.existsSync()) {
        targetChromeDefault.createSync(recursive: true);
        copyDirectory(
          sourceChromeDefault,
          targetChromeDefault,
          shouldCopyDirectory: _isNotCacheDirectory,
        );
      }
    } on FileSystemException catch (err) {
      _logger.printError('Failed to restore Chrome preferences: $err');
    }
  }

  // Cache, Code Cache, and GPUCache are nearly 1GB of data
  bool _isNotCacheDirectory(Directory directory) {
    return !directory.path.endsWith('Cache') &&
        !directory.path.endsWith('Code Cache') &&
        !directory.path.endsWith('GPUCache');
  }

  /// Connect to the [chrome] instance, testing the connection if
  /// [skipCheck] is set to false.
  @visibleForTesting
  Future<Chromium> connect(Chromium chrome, bool skipCheck) async {
    // The connection is lazy. Try a simple call to make sure the provided
    // connection is valid.
    if (!skipCheck) {
      try {
        await chrome._validateChromeConnection();
      } on Exception catch (error, stackTrace) {
        _logger.printError('$error', stackTrace: stackTrace);
        await chrome.close();
        throwToolExit('Unable to connect to Chrome debug port: ${chrome.debugPort}\n $error');
      }
    }
    currentCompleter.complete(chrome);
    return chrome;
  }

  Future<Chromium> get connectedInstance => currentCompleter.future;
}

/// A class for managing an instance of a Chromium browser.
class Chromium {
  Chromium(
    this.debugPort,
    this.chromeConnection, {
    this.url,
    required Process process,
    required ChromiumLauncher chromiumLauncher,
    required Logger logger,
  }) : _process = process,
       _chromiumLauncher = chromiumLauncher,
       _logger = logger;

  final String? url;
  final int debugPort;
  final Process _process;
  final ChromeConnection chromeConnection;
  final ChromiumLauncher _chromiumLauncher;
  final Logger _logger;
  bool _hasValidChromeConnection = false;

  /// Resolves to browser's main process' exit code, when the browser exits.
  Future<int> get onExit async => _process.exitCode;

  /// The main Chromium process that represents this instance of Chromium.
  ///
  /// Killing this process should result in the browser exiting.
  @visibleForTesting
  Process get process => _process;

  /// Gets the first [chrome] tab in order to verify that the connection to
  /// the Chrome debug protocol is working properly.
  ///
  /// Retries getting tabs from Chrome for a few seconds and retries finding
  /// the tab a few times. This reduces flakes caused by Chrome not returning
  /// correct output if the call was too close to the start.
  //
  // TODO(ianh): remove the timeouts here, they violate our style guide.
  // (We should just keep waiting forever, and print a warning when it's
  // taking too long.)
  Future<void> _validateChromeConnection() async {
    const Duration retryFor = Duration(seconds: 2);
    const int attempts = 5;

    for (int i = 1; i <= attempts; i++) {
      try {
        final List<ChromeTab> tabs = await chromeConnection.getTabs(retryFor: retryFor);

        if (tabs.isNotEmpty) {
          _hasValidChromeConnection = true;
          return;
        }
        if (i == attempts) {
          return;
        }
      } on ConnectionException catch (_) {
        if (i == attempts) {
          rethrow;
        }
      } on IOException {
        if (i == attempts) {
          rethrow;
        }
      }
      await Future<void>.delayed(const Duration(milliseconds: 25));
    }
  }

  /// Closes all connections to the browser and asks the browser to exit.
  Future<void> close() async {
    if (_logger.isVerbose) {
      _logger.printTrace('Shutting down Chromium.');
    }
    if (_chromiumLauncher.hasChromeInstance) {
      _chromiumLauncher.currentCompleter = Completer<Chromium>();
    }

    // Send a command to shut down the browser cleanly.
    Duration sigtermDelay = Duration.zero;
    if (_hasValidChromeConnection) {
      try {
        final ChromeTab? tab = await getChromeTabGuarded(
          chromeConnection,
          (_) => true,
          retryFor: const Duration(seconds: 1),
        );
        if (tab != null) {
          final WipConnection wipConnection = await tab.connect();
          await wipConnection.sendCommand('Browser.close');
          await wipConnection.close();
          sigtermDelay = const Duration(seconds: 1);
        }
      } on IOException {
        // Chrome is not responding to the debug protocol and probably has
        // already been closed.
      }
    }
    chromeConnection.close();
    _hasValidChromeConnection = false;

    // If the browser close command did not shut down the process, then try to
    // exit Chromium using SIGTERM.
    await _process.exitCode.timeout(
      sigtermDelay,
      onTimeout: () {
        ProcessSignal.sigterm.kill(_process);
        return 0;
      },
    );
    // If the process still has not ended, then use SIGKILL. Wait up to 5
    // seconds for Chromium to exit before falling back to SIGKILL and then to
    // a warning message.
    await _process.exitCode.timeout(
      const Duration(seconds: 5),
      onTimeout: () {
        _logger.printWarning(
          'Failed to exit Chromium (pid: ${_process.pid}) using SIGTERM. Will try '
          'sending SIGKILL instead.',
        );
        ProcessSignal.sigkill.kill(_process);
        return _process.exitCode.timeout(
          const Duration(seconds: 5),
          onTimeout: () async {
            _logger.printWarning(
              'Failed to exit Chromium (pid: ${_process.pid}) using SIGKILL. Giving '
              'up. Will continue, assuming Chromium has exited successfully, but '
              'it is possible that this left a dangling Chromium process running '
              'on the system.',
            );
            return 0;
          },
        );
      },
    );
  }
}

/// Wrapper for [ChromeConnection.getTab] that will catch any [IOException] or
/// [StateError], delegate it to the [onIoError] callback, and return null.
///
/// This is useful for callers who are want to retrieve a [ChromeTab], but
/// are okay with the operation failing (e.g. due to an network IO issue or
/// the Chrome process no longer existing).
Future<ChromeTab?> getChromeTabGuarded(
  ChromeConnection chromeConnection,
  bool Function(ChromeTab tab) accept, {
  Duration? retryFor,
  void Function(Object error, StackTrace stackTrace)? onIoError,
}) async {
  try {
    return await asyncGuard(() => chromeConnection.getTab(accept, retryFor: retryFor));
  } on IOException catch (error, stackTrace) {
    if (onIoError != null) {
      onIoError(error, stackTrace);
    }
    return null;
    // The underlying HttpClient will throw a StateError when it tries to
    // perform a request despite the connection already being closed.
  } on StateError catch (error, stackTrace) {
    if (onIoError != null) {
      onIoError(error, stackTrace);
    }
    return null;
  }
}
