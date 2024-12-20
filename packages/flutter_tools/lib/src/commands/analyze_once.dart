// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../dart/analysis.dart';
import 'analyze_base.dart';

class AnalyzeOnce extends AnalyzeBase {
  AnalyzeOnce(
    super.argResults,
    List<Directory> repoPackages, {
    required super.fileSystem,
    required super.logger,
    required super.platform,
    required super.processManager,
    required super.terminal,
    required super.artifacts,
    required super.suppressAnalytics,
    this.workingDirectory,
  }) : super(repoPackages: repoPackages);

  /// The working directory for testing analysis using dartanalyzer.
  final Directory? workingDirectory;

  @override
  Future<void> analyze() async {
    final String currentDirectory = (workingDirectory ?? fileSystem.currentDirectory).path;
    final Set<String> items = findDirectories(argResults, fileSystem);

    if (isFlutterRepo) {
      // check for conflicting dependencies
      final PackageDependencyTracker dependencies = PackageDependencyTracker();
      dependencies.checkForConflictingDependencies(repoPackages, dependencies);
      items.add(flutterRoot);
      if (argResults.wasParsed('current-package') && (argResults['current-package'] as bool)) {
        items.add(currentDirectory);
      }
    } else {
      if ((argResults['current-package'] as bool) && items.isEmpty) {
        items.add(currentDirectory);
      }
    }

    if (items.isEmpty) {
      throwToolExit('Nothing to analyze.', exitCode: 0);
    }

    final Completer<void> analysisCompleter = Completer<void>();
    final List<AnalysisError> errors = <AnalysisError>[];

    final AnalysisServer server = AnalysisServer(
      sdkPath,
      items.toList(),
      fileSystem: fileSystem,
      platform: platform,
      logger: logger,
      processManager: processManager,
      terminal: terminal,
      protocolTrafficLog: protocolTrafficLog,
      suppressAnalytics: suppressAnalytics,
    );

    Stopwatch? timer;
    Status? progress;
    try {
      StreamSubscription<bool>? subscription;

      void handleAnalysisStatus(bool isAnalyzing) {
        if (!isAnalyzing) {
          analysisCompleter.complete();
          subscription?.cancel();
          subscription = null;
        }
      }

      subscription = server.onAnalyzing.listen(
        (bool isAnalyzing) => handleAnalysisStatus(isAnalyzing),
      );

      void handleAnalysisErrors(FileAnalysisErrors fileErrors) {
        fileErrors.errors.removeWhere((AnalysisError error) => error.type == 'TODO');

        errors.addAll(fileErrors.errors);
      }

      server.onErrors.listen(handleAnalysisErrors);

      await server.start();
      // Completing the future in the callback can't fail.
      unawaited(
        server.onExit.then<void>((int? exitCode) {
          if (!analysisCompleter.isCompleted) {
            analysisCompleter.completeError(
              // Include the last 20 lines of server output in exception message
              Exception(
                'analysis server exited with code $exitCode and output:\n${server.getLogs(20)}',
              ),
            );
          }
        }),
      );

      // collect results
      timer = Stopwatch()..start();
      final String message =
          items.length > 1
              ? '${items.length} ${items.length == 1 ? 'item' : 'items'}'
              : fileSystem.path.basename(items.first);
      progress =
          argResults['preamble'] == true ? logger.startProgress('Analyzing $message...') : null;

      await analysisCompleter.future;
    } finally {
      await server.dispose();
      progress?.cancel();
      timer?.stop();
    }

    // emit benchmarks
    if (isBenchmarking) {
      writeBenchmark(timer, errors.length);
    }

    // --write
    dumpErrors(errors.map<String>((AnalysisError error) => error.toLegacyString()));

    // report errors
    if (errors.isNotEmpty && (argResults['preamble'] as bool)) {
      logger.printStatus('');
    }
    errors.sort();
    for (final AnalysisError error in errors) {
      logger.printStatus(error.toString(), hangingIndent: 7);
    }

    final int errorCount = errors.length;
    final String seconds = (timer.elapsedMilliseconds / 1000.0).toStringAsFixed(1);
    final String errorsMessage = AnalyzeBase.generateErrorsMessage(
      issueCount: errorCount,
      seconds: seconds,
    );

    if (errorCount > 0) {
      logger.printStatus('');
      throwToolExit(errorsMessage, exitCode: _isFatal(errors) ? 1 : 0);
    }

    if (argResults['congratulate'] as bool) {
      logger.printStatus(errorsMessage);
    }

    if (server.didServerErrorOccur) {
      throwToolExit('Server error(s) occurred. (ran in ${seconds}s)');
    }
  }

  bool _isFatal(List<AnalysisError> errors) {
    for (final AnalysisError error in errors) {
      final AnalysisSeverity severityLevel = error.writtenError.severityLevel;
      if (severityLevel == AnalysisSeverity.error) {
        return true;
      }
      if (severityLevel == AnalysisSeverity.warning && argResults['fatal-warnings'] as bool) {
        return true;
      }
      if (severityLevel == AnalysisSeverity.info && argResults['fatal-infos'] as bool) {
        return true;
      }
    }
    return false;
  }
}
