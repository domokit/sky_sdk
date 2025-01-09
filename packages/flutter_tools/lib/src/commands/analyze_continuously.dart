// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../dart/analysis.dart';
import 'analyze_base.dart';

class AnalyzeContinuously extends AnalyzeBase {
  AnalyzeContinuously(
    super.argResults,
    List<Directory> repoPackages, {
    required super.fileSystem,
    required super.logger,
    required super.terminal,
    required super.platform,
    required super.processManager,
    required super.artifacts,
    required super.suppressAnalytics,
  }) : super(repoPackages: repoPackages);

  String? analysisTarget;
  bool firstAnalysis = true;
  Set<String> analyzedPaths = <String>{};
  Map<String, List<AnalysisError>> analysisErrors = <String, List<AnalysisError>>{};
  final Stopwatch analysisTimer = Stopwatch();
  int lastErrorCount = 0;
  Status? analysisStatus;

  @override
  Future<void> analyze() async {
    List<String> directories;

    if (isFlutterRepo) {
      final PackageDependencyTracker dependencies = PackageDependencyTracker();
      dependencies.checkForConflictingDependencies(repoPackages, dependencies);

      directories = <String>[flutterRoot];
      analysisTarget = 'Flutter repository';

      logger.printTrace('Analyzing Flutter repository:');
    } else {
      directories = <String>[fileSystem.currentDirectory.path];
      analysisTarget = fileSystem.currentDirectory.path;
    }

    final AnalysisServer server = AnalysisServer(
      sdkPath,
      directories,
      fileSystem: fileSystem,
      logger: logger,
      platform: platform,
      processManager: processManager,
      terminal: terminal,
      protocolTrafficLog: protocolTrafficLog,
      suppressAnalytics: suppressAnalytics,
    );
    server.onAnalyzing.listen((bool isAnalyzing) => _handleAnalysisStatus(server, isAnalyzing));
    server.onErrors.listen(_handleAnalysisErrors);

    await server.start();
    final int? exitCode = await server.onExit;

    final String message = 'Analysis server exited with code $exitCode.';
    if (exitCode != 0) {
      throwToolExit(message, exitCode: exitCode);
    }
    logger.printStatus(message);

    if (server.didServerErrorOccur) {
      throwToolExit('Server error(s) occurred.');
    }
  }

  void _handleAnalysisStatus(AnalysisServer server, bool isAnalyzing) {
    if (isAnalyzing) {
      analysisStatus?.cancel();
      if (!firstAnalysis) {
        logger.printStatus('\n');
      }
      analysisStatus = logger.startProgress('Analyzing $analysisTarget...');
      analyzedPaths.clear();
      analysisTimer.start();
    } else {
      analysisStatus?.stop();
      analysisStatus = null;
      analysisTimer.stop();

      logger.printStatus(terminal.clearScreen(), newline: false);

      // Remove errors for deleted files, sort, and print errors.
      final List<AnalysisError> sortedErrors = <AnalysisError>[];
      final List<String> pathsToRemove = <String>[];
      analysisErrors.forEach((String path, List<AnalysisError> errors) {
        if (fileSystem.isFileSync(path)) {
          sortedErrors.addAll(errors);
        } else {
          pathsToRemove.add(path);
        }
      });
      analysisErrors.removeWhere((String path, _) => pathsToRemove.contains(path));

      sortedErrors.sort();

      for (final AnalysisError error in sortedErrors) {
        logger.printStatus(error.toString());
        logger.printTrace('error code: ${error.code}');
      }

      dumpErrors(sortedErrors.map<String>((AnalysisError error) => error.toLegacyString()));

      final int issueCount = sortedErrors.length;
      final int issueDiff = issueCount - lastErrorCount;
      lastErrorCount = issueCount;
      final String seconds = (analysisTimer.elapsedMilliseconds / 1000.0).toStringAsFixed(2);
      final String errorsMessage = AnalyzeBase.generateErrorsMessage(
        issueCount: issueCount,
        issueDiff: issueDiff,
        files: analyzedPaths.length,
        seconds: seconds,
      );

      logger.printStatus(errorsMessage);

      if (firstAnalysis && isBenchmarking) {
        writeBenchmark(analysisTimer, issueCount);
        server.dispose().whenComplete(() {
          exit(issueCount > 0 ? 1 : 0);
        });
      }

      firstAnalysis = false;
    }
  }

  void _handleAnalysisErrors(FileAnalysisErrors fileErrors) {
    fileErrors.errors.removeWhere((AnalysisError error) => error.type == 'TODO');

    analyzedPaths.add(fileErrors.file);
    analysisErrors[fileErrors.file] = fileErrors.errors;
  }
}
