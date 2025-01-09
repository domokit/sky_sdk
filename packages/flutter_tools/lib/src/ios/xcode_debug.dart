// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:xml/xml.dart';
import 'package:xml/xpath.dart';

import '../base/common.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../base/template.dart';
import '../convert.dart';
import '../macos/xcode.dart';
import '../template.dart';

/// A class to handle interacting with Xcode via OSA (Open Scripting Architecture)
/// Scripting to debug Flutter applications.
class XcodeDebug {
  XcodeDebug({
    required Logger logger,
    required ProcessManager processManager,
    required Xcode xcode,
    required FileSystem fileSystem,
  }) : _logger = logger,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       _xcode = xcode,
       _fileSystem = fileSystem;

  final ProcessUtils _processUtils;
  final Logger _logger;
  final Xcode _xcode;
  final FileSystem _fileSystem;

  /// Process to start Xcode's debug action.
  @visibleForTesting
  Process? startDebugActionProcess;

  /// Information about the project that is currently being debugged.
  @visibleForTesting
  XcodeDebugProject? currentDebuggingProject;

  /// Whether the debug action has been started.
  bool get debugStarted => currentDebuggingProject != null;

  /// Install, launch, and start a debug session for app through Xcode interface,
  /// automated by OSA scripting. First checks if the project is opened in
  /// Xcode. If it isn't, open it with the `open` command.
  ///
  /// The OSA script waits until the project is opened and the debug action
  /// has started. It does not wait for the app to install, launch, or start
  /// the debug session.
  Future<bool> debugApp({
    required XcodeDebugProject project,
    required String deviceId,
    required List<String> launchArguments,
  }) async {
    // If project is not already opened in Xcode, open it.
    if (!await _isProjectOpenInXcode(project: project)) {
      final bool openResult = await _openProjectInXcode(xcodeWorkspace: project.xcodeWorkspace);
      if (!openResult) {
        return openResult;
      }
    }

    currentDebuggingProject = project;
    StreamSubscription<String>? stdoutSubscription;
    StreamSubscription<String>? stderrSubscription;
    try {
      startDebugActionProcess = await _processUtils.start(<String>[
        ..._xcode.xcrunCommand(),
        'osascript',
        '-l',
        'JavaScript',
        _xcode.xcodeAutomationScriptPath,
        'debug',
        '--xcode-path',
        _xcode.xcodeAppPath,
        '--project-path',
        project.xcodeProject.path,
        '--workspace-path',
        project.xcodeWorkspace.path,
        '--project-name',
        project.hostAppProjectName,
        if (project.expectedConfigurationBuildDir != null) ...<String>[
          '--expected-configuration-build-dir',
          project.expectedConfigurationBuildDir!,
        ],
        '--device-id',
        deviceId,
        '--scheme',
        project.scheme,
        '--skip-building',
        '--launch-args',
        json.encode(launchArguments),
        if (project.verboseLogging) '--verbose',
      ]);

      final StringBuffer stdoutBuffer = StringBuffer();
      stdoutSubscription = startDebugActionProcess!.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            _logger.printTrace(line);
            stdoutBuffer.write(line);
          });

      final StringBuffer stderrBuffer = StringBuffer();
      bool permissionWarningPrinted = false;
      // console.log from the script are found in the stderr
      stderrSubscription = startDebugActionProcess!.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            _logger.printTrace('stderr: $line');
            stderrBuffer.write(line);

            // This error may occur if Xcode automation has not been allowed.
            // Example: Failed to get workspace: Error: An error occurred.
            if (!permissionWarningPrinted &&
                line.contains('Failed to get workspace') &&
                line.contains('An error occurred')) {
              _logger.printError(
                'There was an error finding the project in Xcode. Ensure permission '
                'has been given to control Xcode in Settings > Privacy & Security > Automation.',
              );
              permissionWarningPrinted = true;
            }
          });

      final int exitCode = await startDebugActionProcess!.exitCode.whenComplete(() async {
        await stdoutSubscription?.cancel();
        await stderrSubscription?.cancel();
        startDebugActionProcess = null;
      });

      if (exitCode != 0) {
        _logger.printError('Error executing osascript: $exitCode\n$stderrBuffer');
        return false;
      }

      final XcodeAutomationScriptResponse? response = parseScriptResponse(stdoutBuffer.toString());
      if (response == null) {
        return false;
      }
      if (response.status == false) {
        _logger.printError('Error starting debug session in Xcode: ${response.errorMessage}');
        return false;
      }
      if (response.debugResult == null) {
        _logger.printError('Unable to get debug results from response: $stdoutBuffer');
        return false;
      }
      if (response.debugResult?.status != 'running') {
        _logger.printError(
          'Unexpected debug results: \n'
          '  Status: ${response.debugResult?.status}\n'
          '  Completed: ${response.debugResult?.completed}\n'
          '  Error Message: ${response.debugResult?.errorMessage}\n',
        );
        return false;
      }
      return true;
    } on ProcessException catch (exception) {
      _logger.printError('Error executing osascript: $exitCode\n$exception');
      await stdoutSubscription?.cancel();
      await stderrSubscription?.cancel();
      startDebugActionProcess = null;

      return false;
    }
  }

  /// Kills [startDebugActionProcess] if it's still running. If [force] is true, it
  /// will kill all Xcode app processes. Otherwise, it will stop the debug
  /// session in Xcode. If the project is temporary, it will close the Xcode
  /// window of the project and then delete the project.
  Future<bool> exit({bool force = false, @visibleForTesting bool skipDelay = false}) async {
    final bool success = (startDebugActionProcess == null) || startDebugActionProcess!.kill();

    if (force) {
      await _forceExitXcode();
      if (currentDebuggingProject != null) {
        final XcodeDebugProject project = currentDebuggingProject!;
        if (project.isTemporaryProject) {
          // Only delete if it exists. This is to prevent crashes when racing
          // with shutdown hooks to delete temporary files.
          ErrorHandlingFileSystem.deleteIfExists(project.xcodeProject.parent, recursive: true);
        }
        currentDebuggingProject = null;
      }
    }

    if (currentDebuggingProject != null) {
      final XcodeDebugProject project = currentDebuggingProject!;
      await stopDebuggingApp(project: project, closeXcode: project.isTemporaryProject);

      if (project.isTemporaryProject) {
        // Wait a couple seconds before deleting the project. If project is
        // still opened in Xcode and it's deleted, it will prompt the user to
        // restore it.
        if (!skipDelay) {
          await Future<void>.delayed(const Duration(seconds: 2));
        }

        try {
          project.xcodeProject.parent.deleteSync(recursive: true);
        } on FileSystemException {
          _logger.printError(
            'Failed to delete temporary Xcode project: ${project.xcodeProject.parent.path}',
          );
        }
      }
      currentDebuggingProject = null;
    }

    return success;
  }

  /// Kill all opened Xcode applications.
  Future<bool> _forceExitXcode() async {
    final RunResult result = await _processUtils.run(<String>['killall', '-9', 'Xcode']);

    if (result.exitCode != 0) {
      _logger.printError('Error killing Xcode: ${result.exitCode}\n${result.stderr}');
      return false;
    }
    return true;
  }

  Future<bool> _isProjectOpenInXcode({required XcodeDebugProject project}) async {
    final RunResult result = await _processUtils.run(<String>[
      ..._xcode.xcrunCommand(),
      'osascript',
      '-l',
      'JavaScript',
      _xcode.xcodeAutomationScriptPath,
      'check-workspace-opened',
      '--xcode-path',
      _xcode.xcodeAppPath,
      '--project-path',
      project.xcodeProject.path,
      '--workspace-path',
      project.xcodeWorkspace.path,
      if (project.verboseLogging) '--verbose',
    ]);

    if (result.exitCode != 0) {
      _logger.printError('Error executing osascript: ${result.exitCode}\n${result.stderr}');
      return false;
    }

    final XcodeAutomationScriptResponse? response = parseScriptResponse(result.stdout);
    if (response == null) {
      return false;
    }
    if (response.status == false) {
      _logger.printTrace('Error checking if project opened in Xcode: ${response.errorMessage}');
      return false;
    }
    return true;
  }

  @visibleForTesting
  XcodeAutomationScriptResponse? parseScriptResponse(String results) {
    try {
      final Object decodeResult = json.decode(results) as Object;
      if (decodeResult is Map<String, Object?>) {
        final XcodeAutomationScriptResponse response = XcodeAutomationScriptResponse.fromJson(
          decodeResult,
        );
        // Status should always be found
        if (response.status != null) {
          return response;
        }
      }
      _logger.printError('osascript returned unexpected JSON response: $results');
      return null;
    } on FormatException {
      _logger.printError('osascript returned non-JSON response: $results');
      return null;
    }
  }

  Future<bool> _openProjectInXcode({required Directory xcodeWorkspace}) async {
    try {
      await _processUtils.run(<String>[
        'open',
        '-a',
        _xcode.xcodeAppPath,
        '-g', // Do not bring the application to the foreground.
        '-j', // Launches the app hidden.
        '-F', // Open "fresh", without restoring windows.
        xcodeWorkspace.path,
      ], throwOnError: true);
      return true;
    } on ProcessException catch (error, stackTrace) {
      _logger.printError('$error', stackTrace: stackTrace);
    }
    return false;
  }

  /// Using OSA Scripting, stop the debug session in Xcode.
  ///
  /// If [closeXcode] is true, it will close the Xcode window that has the
  /// project opened. If [promptToSaveOnClose] is true, it will ask the user if
  /// they want to save any changes before it closes.
  Future<bool> stopDebuggingApp({
    required XcodeDebugProject project,
    bool closeXcode = false,
    bool promptToSaveOnClose = false,
  }) async {
    final RunResult result = await _processUtils.run(<String>[
      ..._xcode.xcrunCommand(),
      'osascript',
      '-l',
      'JavaScript',
      _xcode.xcodeAutomationScriptPath,
      'stop',
      '--xcode-path',
      _xcode.xcodeAppPath,
      '--project-path',
      project.xcodeProject.path,
      '--workspace-path',
      project.xcodeWorkspace.path,
      if (closeXcode) '--close-window',
      if (promptToSaveOnClose) '--prompt-to-save',
      if (project.verboseLogging) '--verbose',
    ]);

    if (result.exitCode != 0) {
      _logger.printError('Error executing osascript: ${result.exitCode}\n${result.stderr}');
      return false;
    }

    final XcodeAutomationScriptResponse? response = parseScriptResponse(result.stdout);
    if (response == null) {
      return false;
    }
    if (response.status == false) {
      _logger.printError('Error stopping app in Xcode: ${response.errorMessage}');
      return false;
    }
    return true;
  }

  /// Create a temporary empty Xcode project with the application bundle
  /// location explicitly set.
  Future<XcodeDebugProject> createXcodeProjectWithCustomBundle(
    String deviceBundlePath, {
    required TemplateRenderer templateRenderer,
    @visibleForTesting Directory? projectDestination,
    bool verboseLogging = false,
  }) async {
    final Directory tempXcodeProject =
        projectDestination ??
        _fileSystem.systemTempDirectory.createTempSync('flutter_empty_xcode.');

    final Template template = await Template.fromName(
      _fileSystem.path.join('xcode', 'ios', 'custom_application_bundle'),
      fileSystem: _fileSystem,
      templateManifest: null,
      logger: _logger,
      templateRenderer: templateRenderer,
    );

    template.render(tempXcodeProject, <String, Object>{
      'applicationBundlePath': deviceBundlePath,
    }, printStatusWhenWriting: false);

    return XcodeDebugProject(
      scheme: 'Runner',
      hostAppProjectName: 'Runner',
      xcodeProject: tempXcodeProject.childDirectory('Runner.xcodeproj'),
      xcodeWorkspace: tempXcodeProject.childDirectory('Runner.xcworkspace'),
      isTemporaryProject: true,
      verboseLogging: verboseLogging,
    );
  }

  /// Ensure the Xcode project is set up to launch an LLDB debugger. If these
  /// settings are not set, the launch will fail with a "Cannot create a
  /// FlutterEngine instance in debug mode without Flutter tooling or Xcode."
  /// error message. These settings should be correct by default, but some users
  /// reported them not being so after upgrading to Xcode 15.
  void ensureXcodeDebuggerLaunchAction(File schemeFile) {
    if (!schemeFile.existsSync()) {
      _logger.printError('Failed to find ${schemeFile.path}');
      return;
    }

    final String schemeXml = schemeFile.readAsStringSync();
    try {
      final XmlDocument document = XmlDocument.parse(schemeXml);
      final Iterable<XmlNode> nodes = document.xpath('/Scheme/LaunchAction');
      if (nodes.isEmpty) {
        _logger.printError('Failed to find LaunchAction for the Scheme in ${schemeFile.path}.');
        return;
      }
      final XmlNode launchAction = nodes.first;
      final XmlAttribute? debuggerIdentifier =
          launchAction.attributes
              .where(
                (XmlAttribute attribute) => attribute.localName == 'selectedDebuggerIdentifier',
              )
              .firstOrNull;
      final XmlAttribute? launcherIdentifier =
          launchAction.attributes
              .where(
                (XmlAttribute attribute) => attribute.localName == 'selectedLauncherIdentifier',
              )
              .firstOrNull;
      if (debuggerIdentifier == null ||
          launcherIdentifier == null ||
          !debuggerIdentifier.value.contains('LLDB') ||
          !launcherIdentifier.value.contains('LLDB')) {
        throwToolExit('''
Your Xcode project is not setup to start a debugger. To fix this, launch Xcode
and select "Product > Scheme > Edit Scheme", select "Run" in the sidebar,
and ensure "Debug executable" is checked in the "Info" tab.
''');
      }
    } on XmlException catch (exception) {
      _logger.printError('Failed to parse ${schemeFile.path}: $exception');
    }
  }
}

@visibleForTesting
class XcodeAutomationScriptResponse {
  XcodeAutomationScriptResponse._({this.status, this.errorMessage, this.debugResult});

  factory XcodeAutomationScriptResponse.fromJson(Map<String, Object?> data) {
    XcodeAutomationScriptDebugResult? debugResult;
    if (data case {'debugResult': final Map<String, Object?> resultData}) {
      debugResult = XcodeAutomationScriptDebugResult.fromJson(resultData);
    }
    return XcodeAutomationScriptResponse._(
      status: data['status'] is bool? ? data['status'] as bool? : null,
      errorMessage: data['errorMessage']?.toString(),
      debugResult: debugResult,
    );
  }

  final bool? status;
  final String? errorMessage;
  final XcodeAutomationScriptDebugResult? debugResult;
}

@visibleForTesting
class XcodeAutomationScriptDebugResult {
  XcodeAutomationScriptDebugResult._({
    required this.completed,
    required this.status,
    required this.errorMessage,
  });

  factory XcodeAutomationScriptDebugResult.fromJson(Map<String, Object?> data) {
    return XcodeAutomationScriptDebugResult._(
      completed: data['completed'] is bool? ? data['completed'] as bool? : null,
      status: data['status']?.toString(),
      errorMessage: data['errorMessage']?.toString(),
    );
  }

  /// Whether this scheme action has completed (successfully or otherwise). Will
  /// be false if still running.
  final bool? completed;

  /// The status of the debug action. Potential statuses include:
  /// `not yet started`, `‌running`, `‌cancelled`, `‌failed`, `‌error occurred`,
  /// and `‌succeeded`.
  ///
  /// Only the status of `‌running` indicates the debug action has started successfully.
  /// For example, `‌succeeded` often does not indicate success as if the action fails,
  /// it will sometimes return `‌succeeded`.
  final String? status;

  /// When [status] is `‌error occurred`, an error message is provided.
  /// Otherwise, this will be null.
  final String? errorMessage;
}

class XcodeDebugProject {
  XcodeDebugProject({
    required this.scheme,
    required this.xcodeWorkspace,
    required this.xcodeProject,
    required this.hostAppProjectName,
    this.expectedConfigurationBuildDir,
    this.isTemporaryProject = false,
    this.verboseLogging = false,
  });

  final String scheme;
  final Directory xcodeWorkspace;
  final Directory xcodeProject;
  final String hostAppProjectName;
  final String? expectedConfigurationBuildDir;
  final bool isTemporaryProject;

  /// When [verboseLogging] is true, the xcode_debug.js script will log
  /// additional information via console.log, which is sent to stderr.
  final bool verboseLogging;
}
