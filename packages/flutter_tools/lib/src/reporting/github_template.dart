// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:intl/intl.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../build_system/exceptions.dart';
import '../convert.dart';
import '../devfs.dart';
import '../flutter_manifest.dart';
import '../flutter_project_metadata.dart';
import '../project.dart';
import '../version.dart';

/// Provide suggested GitHub issue templates to user when Flutter encounters an error.
class GitHubTemplateCreator {
  GitHubTemplateCreator({
    required FileSystem fileSystem,
    required Logger logger,
    required FlutterProjectFactory flutterProjectFactory,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _flutterProjectFactory = flutterProjectFactory;

  final FileSystem _fileSystem;
  final Logger _logger;
  final FlutterProjectFactory _flutterProjectFactory;

  static String toolCrashSimilarIssuesURL(String errorString) {
    return 'https://github.com/flutter/flutter/issues?q=is%3Aissue+${Uri.encodeQueryComponent(errorString)}';
  }

  /// Restricts exception object strings to contain only information about tool internals.
  static String sanitizedCrashException(Object error) {
    if (error is ProcessException) {
      // Suppress args.
      return 'ProcessException: ${error.message} Command: ${error.executable}, OS error code: ${error.errorCode}';
    } else if (error is FileSystemException) {
      // Suppress path.
      return 'FileSystemException: ${error.message}, ${error.osError}';
    } else if (error is SocketException) {
      // Suppress address and port.
      return 'SocketException: ${error.message}, ${error.osError}';
    } else if (error is DevFSException) {
      // Suppress underlying error.
      return 'DevFSException: ${error.message}';
    } else if (error is NoSuchMethodError ||
        error is ArgumentError ||
        error is VersionCheckError ||
        error is MissingDefineException ||
        error is UnsupportedError ||
        error is UnimplementedError ||
        error is StateError ||
        error is ProcessExit ||
        error is OSError) {
      // These exception objects only reference tool internals, print the whole error.
      return '${error.runtimeType}: $error';
    } else if (error is Error) {
      return '${error.runtimeType}: ${LineSplitter.split(error.stackTrace.toString()).take(1)}';
    } else if (error is String) {
      // Force comma separator to standardize.
      return 'String: <${NumberFormat(null, 'en_US').format(error.length)} characters>';
    }
    // Exception, other.
    return error.runtimeType.toString();
  }

  /// GitHub URL to present to the user containing encoded suggested template.
  ///
  /// Shorten the URL, if possible.
  Future<String> toolCrashIssueTemplateGitHubURL(
    String command,
    Object error,
    StackTrace stackTrace,
    String doctorText,
  ) async {
    final String errorString = sanitizedCrashException(error);
    final String title = '[tool_crash] $errorString';
    final String body = '''
## Command
```sh
$command
```

## Steps to Reproduce
1. ...
2. ...
3. ...

## Logs
$errorString
```console
${LineSplitter.split(stackTrace.toString()).take(25).join('\n')}
```
```console
$doctorText
```

## Flutter Application Metadata
${_projectMetadataInformation()}
''';

    return 'https://github.com/flutter/flutter/issues'
        '/new' // We split this here to appease our lint that looks for bad "new bug" links.
        '?title=${Uri.encodeQueryComponent(title)}'
        '&body=${Uri.encodeQueryComponent(body)}'
        '&labels=${Uri.encodeQueryComponent('tool,severe: crash')}';
  }

  /// Provide information about the Flutter project in the working directory, if present.
  String _projectMetadataInformation() {
    FlutterProject project;
    try {
      project = _flutterProjectFactory.fromDirectory(_fileSystem.currentDirectory);
    } on Exception catch (exception) {
      // pubspec may be malformed.
      return exception.toString();
    }
    try {
      final FlutterManifest manifest = project.manifest;
      if (manifest.isEmpty) {
        return 'No pubspec in working directory.';
      }
      final FlutterProjectMetadata metadata = FlutterProjectMetadata(project.metadataFile, _logger);
      final FlutterProjectType? projectType = metadata.projectType;
      final StringBuffer description =
          StringBuffer()
            ..writeln('**Type**: ${projectType == null ? 'malformed' : projectType.cliName}')
            ..writeln('**Version**: ${manifest.appVersion}')
            ..writeln('**Material**: ${manifest.usesMaterialDesign}')
            ..writeln('**Android X**: ${manifest.usesAndroidX}')
            ..writeln('**Module**: ${manifest.isModule}')
            ..writeln('**Plugin**: ${manifest.isPlugin}')
            ..writeln('**Android package**: ${manifest.androidPackage}')
            ..writeln('**iOS bundle identifier**: ${manifest.iosBundleIdentifier}')
            ..writeln('**Creation channel**: ${metadata.versionChannel}')
            ..writeln('**Creation framework version**: ${metadata.versionRevision}');

      final File file = project.flutterPluginsFile;
      if (file.existsSync()) {
        description.writeln('### Plugins');
        // Format is:
        // camera=/path/to/.pub-cache/hosted/pub.dartlang.org/camera-0.5.7+2/
        for (final String plugin in project.flutterPluginsFile.readAsLinesSync()) {
          final List<String> pluginParts = plugin.split('=');
          if (pluginParts.length != 2) {
            continue;
          }
          // Write the last part of the path, which includes the plugin name and version.
          // Example: camera-0.5.7+2
          final List<String> pathParts = _fileSystem.path.split(pluginParts[1]);
          description.writeln(pathParts.isEmpty ? pluginParts.first : pathParts.last);
        }
      }

      return description.toString();
    } on Exception catch (exception) {
      return exception.toString();
    }
  }
}
