// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

// ignore: implementation_imports
import 'package:_fe_analyzer_shared/src/base/syntactic_entity.dart';
import 'package:analyzer/dart/analysis/analysis_context.dart';
import 'package:analyzer/dart/analysis/analysis_context_collection.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/file_system/physical_file_system.dart';
import 'package:watcher/watcher.dart';

import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/utils.dart';
import 'preview_code_generator.dart';

typedef PreviewMapping = Map<String, List<String>>;

class PreviewDetector {
  PreviewDetector({required this.fs, required this.logger, required this.onChangeDetected});

  final FileSystem fs;
  final Logger logger;
  final void Function(PreviewMapping) onChangeDetected;
  StreamSubscription<WatchEvent>? _fileWatcher;
  late final PreviewMapping _pathToPreviews;

  /// Starts listening for changes to Dart sources under [projectRoot] and returns
  /// the initial [PreviewMapping] for the project.
  Future<PreviewMapping> initialize(Directory projectRoot) async {
    // Find the initial set of previews.
    _pathToPreviews = findPreviewFunctions(projectRoot);

    final Watcher watcher = Watcher(projectRoot.path);
    // TODO(bkonyi): watch for changes to pubspec.yaml
    _fileWatcher = watcher.events.listen((WatchEvent event) async {
      final String eventPath = Uri.file(event.path).toString();
      // Only trigger a reload when changes to Dart sources are detected. We
      // ignore the generated preview file to avoid getting stuck in a loop.
      if (!eventPath.endsWith('.dart') ||
          eventPath.endsWith(PreviewCodeGenerator.generatedPreviewFilePath)) {
        return;
      }
      logger.printStatus('Detected change in $eventPath.');
      final PreviewMapping filePreviewsMapping = findPreviewFunctions(
        fs.file(Uri.file(event.path)),
      );
      if (filePreviewsMapping.isEmpty && !_pathToPreviews.containsKey(eventPath)) {
        // No previews found or removed, nothing to do.
        return;
      }
      if (filePreviewsMapping.length > 1) {
        logger.printWarning('Previews from more than one file were detected!');
        logger.printWarning('Previews: $filePreviewsMapping');
      }
      if (filePreviewsMapping.isNotEmpty) {
        // The set of previews has changed, but there are still previews in the file.
        final MapEntry<String, List<String>>(key: String uri, value: List<String> filePreviews) =
            filePreviewsMapping.entries.first;
        assert(uri == eventPath);
        logger.printStatus('Updated previews for $eventPath: $filePreviews');
        if (filePreviews.isNotEmpty) {
          final List<String>? currentPreviewsForFile = _pathToPreviews[eventPath];
          if (filePreviews != currentPreviewsForFile) {
            _pathToPreviews[eventPath] = filePreviews;
          }
        }
      } else {
        // The file previously had previews that were removed.
        logger.printStatus('Previews removed from $eventPath');
        _pathToPreviews.remove(eventPath);
      }
      onChangeDetected(_pathToPreviews);
    });
    // Wait for file watcher to finish initializing, otherwise we might miss changes and cause
    // tests to flake.
    await watcher.ready;
    return _pathToPreviews;
  }

  Future<void> dispose() async {
    await _fileWatcher?.cancel();
  }

  /// Search for functions annotated with `@Preview` in the current project.
  PreviewMapping findPreviewFunctions(FileSystemEntity entity) {
    final AnalysisContextCollection collection = AnalysisContextCollection(
      includedPaths: <String>[entity.absolute.path],
      resourceProvider: PhysicalResourceProvider.INSTANCE,
    );

    final PreviewMapping previews = PreviewMapping();
    for (final AnalysisContext context in collection.contexts) {
      logger.printStatus('Finding previews in ${context.contextRoot.root.path}...');

      for (final String filePath in context.contextRoot.analyzedFiles()) {
        logger.printTrace('Checking file: $filePath');
        if (!filePath.endsWith('.dart')) {
          continue;
        }

        final SomeParsedLibraryResult lib = context.currentSession.getParsedLibrary(filePath);
        if (lib is ParsedLibraryResult) {
          for (final ParsedUnitResult unit in lib.units) {
            final List<String> previewEntries = previews[unit.uri.toString()] ?? <String>[];
            for (final SyntacticEntity entity in unit.unit.childEntities) {
              if (entity is FunctionDeclaration && !entity.name.toString().startsWith('_')) {
                bool foundPreview = false;
                for (final Annotation annotation in entity.metadata) {
                  if (annotation.name.name == 'Preview') {
                    // What happens if the annotation is applied multiple times?
                    foundPreview = true;
                    break;
                  }
                }
                if (foundPreview) {
                  logger.printStatus('Found preview at:');
                  logger.printStatus('File path: ${unit.uri}');
                  logger.printStatus('Preview function: ${entity.name}');
                  logger.printStatus('');
                  previewEntries.add(entity.name.toString());
                }
              }
            }
            if (previewEntries.isNotEmpty) {
              previews[unit.uri.toString()] = previewEntries;
            }
          }
        } else {
          logger.printWarning('Unknown library type at $filePath: $lib');
        }
      }
    }
    final int previewCount = previews.values.fold<int>(
      0,
      (int count, List<String> value) => count + value.length,
    );
    logger.printStatus('Found $previewCount ${pluralize('preview', previewCount)}.');
    return previews;
  }
}
