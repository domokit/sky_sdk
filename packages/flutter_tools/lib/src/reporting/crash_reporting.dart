// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:http/http.dart' as http;
import 'package:unified_analytics/unified_analytics.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../doctor.dart';
import '../project.dart';
import 'github_template.dart';

/// Tells crash backend that the error is from the Flutter CLI.
const String _kProductId = 'Flutter_Tools';

/// Tells crash backend that this is a Dart error as opposed to, say, Java.
const String _kDartTypeId = 'DartError';

/// Crash backend host.
const String _kCrashServerHost = 'clients2.google.com';

/// Path to the crash servlet.
const String _kCrashEndpointPath = '/cr/report';

/// The field corresponding to the multipart/form-data file attachment where
/// crash backend expects to find the Dart stack trace.
const String _kStackTraceFileField = 'DartError';

/// The name of the file attached as [_kStackTraceFileField].
///
/// The precise value is not important. It is ignored by the crash back end, but
/// it must be supplied in the request.
const String _kStackTraceFilename = 'stacktrace_file';

class CrashDetails {
  CrashDetails({
    required this.command,
    required this.error,
    required this.stackTrace,
    required this.doctorText,
  });

  final String command;
  final Object error;
  final StackTrace stackTrace;
  final DoctorText doctorText;
}

/// Reports information about the crash to the user.
class CrashReporter {
  CrashReporter({
    required FileSystem fileSystem,
    required Logger logger,
    required FlutterProjectFactory flutterProjectFactory,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _flutterProjectFactory = flutterProjectFactory;

  final FileSystem _fileSystem;
  final Logger _logger;
  final FlutterProjectFactory _flutterProjectFactory;

  /// Prints instructions for filing a bug about the crash.
  Future<void> informUser(CrashDetails details, File crashFile) async {
    _logger.printError('A crash report has been written to ${crashFile.path}');
    _logger.printStatus(
      'This crash may already be reported. Check GitHub for similar crashes.',
      emphasis: true,
    );

    final String similarIssuesURL = GitHubTemplateCreator.toolCrashSimilarIssuesURL(
      details.error.toString(),
    );
    _logger.printStatus('$similarIssuesURL\n', wrap: false);
    _logger.printStatus(
      'To report your crash to the Flutter team, first read the guide to filing a bug.',
      emphasis: true,
    );
    _logger.printStatus('https://flutter.dev/to/report-bugs\n', wrap: false);

    _logger.printStatus(
      'Create a new GitHub issue by pasting this link into your browser and completing the issue template. Thank you!',
      emphasis: true,
    );

    final GitHubTemplateCreator gitHubTemplateCreator = GitHubTemplateCreator(
      fileSystem: _fileSystem,
      logger: _logger,
      flutterProjectFactory: _flutterProjectFactory,
    );

    final String gitHubTemplateURL = await gitHubTemplateCreator.toolCrashIssueTemplateGitHubURL(
      details.command,
      details.error,
      details.stackTrace,
      await details.doctorText.piiStrippedText,
    );
    _logger.printStatus('$gitHubTemplateURL\n', wrap: false);
  }
}

/// Sends crash reports to Google.
///
/// To override the behavior of this class, define a
/// `FLUTTER_CRASH_SERVER_BASE_URL` environment variable that points to a custom
/// crash reporting server. This is useful if your development environment is
/// behind a firewall and unable to send crash reports to Google, or when you
/// wish to use your own server for collecting crash reports from Flutter Tools.
class CrashReportSender {
  CrashReportSender({
    http.Client? client,
    required Platform platform,
    required Logger logger,
    required OperatingSystemUtils operatingSystemUtils,
    required Analytics analytics,
  }) : _client = client ?? http.Client(),
       _platform = platform,
       _logger = logger,
       _operatingSystemUtils = operatingSystemUtils,
       _analytics = analytics;

  final http.Client _client;
  final Platform _platform;
  final Logger _logger;
  final OperatingSystemUtils _operatingSystemUtils;
  final Analytics _analytics;

  bool _crashReportSent = false;

  Uri get _baseUrl {
    final String? overrideUrl = _platform.environment['FLUTTER_CRASH_SERVER_BASE_URL'];

    if (overrideUrl != null) {
      return Uri.parse(overrideUrl);
    }
    return Uri(scheme: 'https', host: _kCrashServerHost, port: 443, path: _kCrashEndpointPath);
  }

  /// Sends one crash report.
  ///
  /// The report is populated from data in [error] and [stackTrace].
  Future<void> sendReport({
    required Object error,
    required StackTrace stackTrace,
    required String Function() getFlutterVersion,
    required String command,
  }) async {
    // Only send one crash report per run.
    if (_crashReportSent) {
      return;
    }
    try {
      final String flutterVersion = getFlutterVersion();

      // We don't need to report exceptions happening on user branches
      if (!_analytics.okToSend || RegExp(r'^\[user-branch\]\/').hasMatch(flutterVersion)) {
        return;
      }

      _logger.printTrace('Sending crash report to Google.');

      final Uri uri = _baseUrl.replace(
        queryParameters: <String, String>{'product': _kProductId, 'version': flutterVersion},
      );

      final http.MultipartRequest req = http.MultipartRequest('POST', uri);
      req.fields['uuid'] = _analytics.clientId;
      req.fields['product'] = _kProductId;
      req.fields['version'] = flutterVersion;
      req.fields['osName'] = _platform.operatingSystem;
      req.fields['osVersion'] = _operatingSystemUtils.name; // this actually includes version
      req.fields['type'] = _kDartTypeId;
      req.fields['error_runtime_type'] = '${error.runtimeType}';
      req.fields['error_message'] = '$error';
      req.fields['comments'] = command;

      req.files.add(
        http.MultipartFile.fromString(
          _kStackTraceFileField,
          stackTrace.toString(),
          filename: _kStackTraceFilename,
        ),
      );

      final http.StreamedResponse resp = await _client.send(req);

      if (resp.statusCode == HttpStatus.ok) {
        final String reportId = await http.ByteStream(resp.stream).bytesToString();
        _logger.printTrace('Crash report sent (report ID: $reportId)');
        _crashReportSent = true;
      } else {
        _logger.printError(
          'Failed to send crash report. Server responded with HTTP status code ${resp.statusCode}',
        );
      }

      // Catch all exceptions to print the message that makes clear that the
      // crash logger crashed.
      // ignore: avoid_catches_without_on_clauses
    } catch (sendError, sendStackTrace) {
      if (sendError is SocketException ||
          sendError is HttpException ||
          sendError is http.ClientException) {
        _logger.printError('Failed to send crash report due to a network error: $sendError');
      } else {
        // If the sender itself crashes, just print. We did our best.
        _logger.printError('Crash report sender itself crashed: $sendError\n$sendStackTrace');
      }
    }
  }
}
