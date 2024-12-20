// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:process/process.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../convert.dart';

/// An interface to the Dart analysis server.
class AnalysisServer {
  AnalysisServer(
    this.sdkPath,
    this.directories, {
    required FileSystem fileSystem,
    required ProcessManager processManager,
    required Logger logger,
    required Platform platform,
    required Terminal terminal,
    required this.suppressAnalytics,
    String? protocolTrafficLog,
  }) : _fileSystem = fileSystem,
       _processManager = processManager,
       _logger = logger,
       _platform = platform,
       _terminal = terminal,
       _protocolTrafficLog = protocolTrafficLog;

  final String sdkPath;
  final List<String> directories;
  final FileSystem _fileSystem;
  final ProcessManager _processManager;
  final Logger _logger;
  final Platform _platform;
  final Terminal _terminal;
  final String? _protocolTrafficLog;
  final bool suppressAnalytics;

  Process? _process;
  final StreamController<bool> _analyzingController = StreamController<bool>.broadcast();
  final StreamController<FileAnalysisErrors> _errorsController =
      StreamController<FileAnalysisErrors>.broadcast();
  bool _didServerErrorOccur = false;

  int _id = 0;

  Future<void> start() async {
    final String snapshot = _fileSystem.path.join(
      sdkPath,
      'bin',
      'snapshots',
      'analysis_server.dart.snapshot',
    );
    final List<String> command = <String>[
      _fileSystem.path.join(sdkPath, 'bin', 'dart'),
      snapshot,
      '--disable-server-feature-completion',
      '--disable-server-feature-search',
      '--sdk',
      sdkPath,
      if (suppressAnalytics) '--suppress-analytics',
      if (_protocolTrafficLog != null) '--protocol-traffic-log=$_protocolTrafficLog',
    ];

    _logger.printTrace('dart ${command.skip(1).join(' ')}');
    _process = await _processManager.start(command);
    // This callback hookup can't throw.
    unawaited(_process!.exitCode.whenComplete(() => _process = null));

    final Stream<String> errorStream = _process!.stderr
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter());
    errorStream.listen(_handleError);

    final Stream<String> inStream = _process!.stdout
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter());
    inStream.listen(_handleServerResponse);

    _sendCommand('server.setSubscriptions', <String, dynamic>{
      'subscriptions': <String>['STATUS'],
    });

    _sendCommand('analysis.setAnalysisRoots', <String, dynamic>{
      'included': directories,
      'excluded': <String>[],
    });
  }

  final List<String> _logs = <String>[];

  /// Aggregated STDOUT and STDERR logs from the server.
  ///
  /// This can be surfaced to the user if the server crashes. If [tail] is null,
  /// returns all logs, else only the last [tail] lines.
  String getLogs([int? tail]) {
    if (tail == null) {
      return _logs.join('\n');
    }
    // Since List doesn't implement a .tail() method, we reverse it then use
    // .take()
    final Iterable<String> reversedLogs = _logs.reversed;
    final List<String> firstTailLogs = reversedLogs.take(tail).toList();
    return firstTailLogs.reversed.join('\n');
  }

  void _handleError(String message) {
    _logs.add('[stderr] $message');
    _logger.printError(message);
  }

  bool get didServerErrorOccur => _didServerErrorOccur;

  Stream<bool> get onAnalyzing => _analyzingController.stream;

  Stream<FileAnalysisErrors> get onErrors => _errorsController.stream;

  Future<int?> get onExit async => _process?.exitCode;

  void _sendCommand(String method, Map<String, dynamic> params) {
    final String message = json.encode(<String, dynamic>{
      'id': (++_id).toString(),
      'method': method,
      'params': params,
    });
    _process?.stdin.writeln(message);
    _logger.printTrace('==> $message');
  }

  void _handleServerResponse(String line) {
    _logs.add('[stdout] $line');
    _logger.printTrace('<== $line');

    final dynamic response = json.decode(line);

    if (response is Map<String, dynamic>) {
      if (response['event'] != null) {
        final String event = response['event'] as String;
        final dynamic params = response['params'];
        Map<String, dynamic>? paramsMap;
        if (params is Map<String, dynamic>) {
          paramsMap = castStringKeyedMap(params);
        }

        if (paramsMap != null) {
          switch (event) {
            case 'server.status':
              _handleStatus(paramsMap);
            case 'analysis.errors':
              _handleAnalysisIssues(paramsMap);
            case 'server.error':
              _handleServerError(paramsMap);
          }
        }
      } else if (response['error'] != null) {
        // Fields are 'code', 'message', and 'stackTrace'.
        final Map<String, dynamic> error = castStringKeyedMap(response['error'])!;
        _logger.printError('Error response from the server: ${error['code']} ${error['message']}');
        if (error['stackTrace'] != null) {
          _logger.printError(error['stackTrace'] as String);
        }
      }
    }
  }

  void _handleStatus(Map<String, dynamic> statusInfo) {
    // {"event":"server.status","params":{"analysis":{"isAnalyzing":true}}}
    if (statusInfo['analysis'] != null && !_analyzingController.isClosed) {
      final bool isAnalyzing =
          (statusInfo['analysis'] as Map<String, dynamic>)['isAnalyzing'] as bool;
      _analyzingController.add(isAnalyzing);
    }
  }

  void _handleServerError(Map<String, dynamic> error) {
    // Fields are 'isFatal', 'message', and 'stackTrace'.
    _logger.printError('Error from the analysis server: ${error['message']}');
    if (error['stackTrace'] != null) {
      _logger.printError(error['stackTrace'] as String);
    }
    _didServerErrorOccur = true;
  }

  void _handleAnalysisIssues(Map<String, dynamic> issueInfo) {
    // {"event":"analysis.errors","params":{"file":"/Users/.../lib/main.dart","errors":[]}}
    final String file = issueInfo['file'] as String;
    final List<dynamic> errorsList = issueInfo['errors'] as List<dynamic>;
    final List<AnalysisError> errors =
        errorsList
            .map<Map<String, dynamic>>((dynamic e) => castStringKeyedMap(e) ?? <String, dynamic>{})
            .map<AnalysisError>((Map<String, dynamic> json) {
              return AnalysisError(
                WrittenError.fromJson(json),
                fileSystem: _fileSystem,
                platform: _platform,
                terminal: _terminal,
              );
            })
            .toList();
    if (!_errorsController.isClosed) {
      _errorsController.add(FileAnalysisErrors(file, errors));
    }
  }

  Future<bool?> dispose() async {
    await _analyzingController.close();
    await _errorsController.close();
    return _process?.kill();
  }
}

enum AnalysisSeverity { error, warning, info, none }

/// [AnalysisError] with command line style.
class AnalysisError implements Comparable<AnalysisError> {
  AnalysisError(
    this.writtenError, {
    required Platform platform,
    required Terminal terminal,
    required FileSystem fileSystem,
  }) : _platform = platform,
       _terminal = terminal,
       _fileSystem = fileSystem;

  final WrittenError writtenError;
  final Platform _platform;
  final Terminal _terminal;
  final FileSystem _fileSystem;

  String get _separator => _platform.isWindows ? '-' : '•';

  String get colorSeverity => switch (writtenError.severityLevel) {
    AnalysisSeverity.error => _terminal.color(writtenError.severity, TerminalColor.red),
    AnalysisSeverity.warning => _terminal.color(writtenError.severity, TerminalColor.yellow),
    AnalysisSeverity.info || AnalysisSeverity.none => writtenError.severity,
  };

  String get type => writtenError.type;
  String get code => writtenError.code;

  @override
  int compareTo(AnalysisError other) {
    // Sort in order of file path, error location, severity, and message.
    if (writtenError.file != other.writtenError.file) {
      return writtenError.file.compareTo(other.writtenError.file);
    }

    if (writtenError.offset != other.writtenError.offset) {
      return writtenError.offset - other.writtenError.offset;
    }

    final int diff = other.writtenError.severityLevel.index - writtenError.severityLevel.index;
    if (diff != 0) {
      return diff;
    }

    return writtenError.message.compareTo(other.writtenError.message);
  }

  @override
  String toString() {
    // Can't use "padLeft" because of ANSI color sequences in the colorized
    // severity.
    final String padding = ' ' * math.max(0, 7 - writtenError.severity.length);
    return '$padding${colorSeverity.toLowerCase()} $_separator '
        '${writtenError.messageSentenceFragment} $_separator '
        '${_fileSystem.path.relative(writtenError.file)}:${writtenError.startLine}:${writtenError.startColumn} $_separator '
        '$code';
  }

  String toLegacyString() {
    return writtenError.toString();
  }
}

/// [AnalysisError] in plain text content.
class WrittenError {
  WrittenError._({
    required this.severity,
    required this.type,
    required this.message,
    required this.code,
    required this.file,
    required this.startLine,
    required this.startColumn,
    required this.offset,
  });

  ///  {
  ///      "severity":"INFO",
  ///      "type":"TODO",
  ///      "location":{
  ///          "file":"/Users/.../lib/test.dart",
  ///          "offset":362,
  ///          "length":72,
  ///          "startLine":15,
  ///         "startColumn":4
  ///      },
  ///      "message":"...",
  ///      "hasFix":false
  ///  }
  static WrittenError fromJson(Map<String, dynamic> json) {
    final Map<String, dynamic> location = json['location'] as Map<String, dynamic>;
    return WrittenError._(
      severity: json['severity'] as String,
      type: json['type'] as String,
      message: json['message'] as String,
      code: json['code'] as String,
      file: location['file'] as String,
      startLine: location['startLine'] as int,
      startColumn: location['startColumn'] as int,
      offset: location['offset'] as int,
    );
  }

  final String severity;
  final String type;
  final String message;
  final String code;

  final String file;
  final int startLine;
  final int startColumn;
  final int offset;

  static final Map<String, AnalysisSeverity> _severityMap = <String, AnalysisSeverity>{
    'INFO': AnalysisSeverity.info,
    'WARNING': AnalysisSeverity.warning,
    'ERROR': AnalysisSeverity.error,
  };

  AnalysisSeverity get severityLevel => _severityMap[severity] ?? AnalysisSeverity.none;

  String get messageSentenceFragment {
    if (message.endsWith('.')) {
      return message.substring(0, message.length - 1);
    }
    return message;
  }

  @override
  String toString() {
    return '[${severity.toLowerCase()}] $messageSentenceFragment ($file:$startLine:$startColumn)';
  }
}

class FileAnalysisErrors {
  FileAnalysisErrors(this.file, this.errors);

  final String file;
  final List<AnalysisError> errors;
}
