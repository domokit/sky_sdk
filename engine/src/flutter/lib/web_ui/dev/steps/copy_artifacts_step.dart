// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show JsonEncoder;
import 'dart:io' as io;

import 'package:archive/archive_io.dart';
import 'package:http/http.dart' as http;
import 'package:path/path.dart' as pathlib;

import '../common.dart';
import '../environment.dart';
import '../exceptions.dart';
import '../felt_config.dart';
import '../pipeline.dart';
import '../utils.dart';

sealed class ArtifactSource {}

class LocalArtifactSource implements ArtifactSource {
  LocalArtifactSource({required this.mode});

  final RuntimeMode mode;
}

class GcsArtifactSource implements ArtifactSource {
  GcsArtifactSource({required this.realm});

  final LuciRealm realm;
}

class CopyArtifactsStep implements PipelineStep {
  CopyArtifactsStep(this.artifactDeps, {required this.source});

  final ArtifactDependencies artifactDeps;
  final ArtifactSource source;

  @override
  String get description => 'copy_artifacts';

  @override
  bool get isSafeToInterrupt => true;

  @override
  Future<void> interrupt() async {
    await cleanup();
  }

  Future<io.Directory> _downloadArtifacts(LuciRealm realm) async {
    final String realmComponent = switch (realm) {
      LuciRealm.Prod || LuciRealm.Staging => '',
      LuciRealm.Try => 'flutter_archives_v2/',
      LuciRealm.Unknown =>
        throw ToolExit('Could not generate artifact bucket url for unknown realm.'),
    };
    final Uri url = Uri.https(
      'storage.googleapis.com',
      '${realmComponent}flutter_infra_release/flutter/$gitRevision/flutter-web-sdk.zip',
    );
    final http.Response response = await http.Client().get(url);
    if (response.statusCode != 200) {
      throw ToolExit(
        'Could not download flutter-web-sdk.zip from cloud bucket at URL: $url. Response status code: ${response.statusCode}',
      );
    }
    final Archive archive = ZipDecoder().decodeBytes(response.bodyBytes);
    final io.Directory tempDirectory = await io.Directory.systemTemp.createTemp();
    await extractArchiveToDisk(archive, tempDirectory.absolute.path);
    return tempDirectory;
  }

  @override
  Future<void> run() async {
    final String flutterJsSourceDirectory;
    final String canvaskitSourceDirectory;
    final String canvaskitChromiumSourceDirectory;
    final String skwasmSourceDirectory;
    final String skwasmStSourceDirectory;
    switch (source) {
      case LocalArtifactSource(:final mode):
        final buildDirectory = getBuildDirectoryForRuntimeMode(mode).path;
        flutterJsSourceDirectory = pathlib.join(buildDirectory, 'flutter_web_sdk', 'flutter_js');
        canvaskitSourceDirectory = pathlib.join(buildDirectory, 'canvaskit');
        canvaskitChromiumSourceDirectory = pathlib.join(buildDirectory, 'canvaskit_chromium');
        skwasmSourceDirectory = pathlib.join(buildDirectory, 'skwasm');
        skwasmStSourceDirectory = pathlib.join(buildDirectory, 'skwasm_st');

      case GcsArtifactSource(:final realm):
        final artifactsDirectory = (await _downloadArtifacts(realm)).path;
        flutterJsSourceDirectory = pathlib.join(artifactsDirectory, 'flutter_js');
        canvaskitSourceDirectory = pathlib.join(artifactsDirectory, 'canvaskit');
        canvaskitChromiumSourceDirectory = pathlib.join(
          artifactsDirectory,
          'canvaskit',
          'chromium',
        );
        skwasmSourceDirectory = pathlib.join(artifactsDirectory, 'canvaskit');
        skwasmStSourceDirectory = pathlib.join(artifactsDirectory, 'canvaskit');
    }

    await environment.webTestsArtifactsDir.create(recursive: true);
    await buildHostPage();
    await copyTestFonts();
    await copySkiaTestImages();
    await copyFlutterJsFiles(flutterJsSourceDirectory);
    if (artifactDeps.canvasKit) {
      print('Copying CanvasKit...');
      await copyWasmLibrary('canvaskit', canvaskitSourceDirectory, 'canvaskit');
    }
    if (artifactDeps.canvasKitChromium) {
      print('Copying CanvasKit (Chromium)...');
      await copyWasmLibrary('canvaskit', canvaskitChromiumSourceDirectory, 'canvaskit/chromium');
    }
    if (artifactDeps.skwasm) {
      print('Copying Skwasm...');
      await copyWasmLibrary('skwasm', skwasmSourceDirectory, 'canvaskit');
      await copyWasmLibrary('skwasm_st', skwasmStSourceDirectory, 'canvaskit');
    }
  }

  Future<void> copyTestFonts() async {
    const Map<String, String> testFonts = <String, String>{
      'Ahem': 'ahem.ttf',
      'Roboto': 'Roboto-Regular.ttf',
      'RobotoVariable': 'RobotoSlab-VariableFont_wght.ttf',
      'Noto Naskh Arabic UI': 'NotoNaskhArabic-Regular.ttf',
      'Noto Color Emoji': 'NotoColorEmoji.ttf',
    };

    final String fontsPath = pathlib.join(
      environment.flutterDirectory.path,
      'third_party',
      'txt',
      'third_party',
      'fonts',
    );

    final List<dynamic> fontManifest = <dynamic>[];
    for (final MapEntry<String, String> fontEntry in testFonts.entries) {
      final String family = fontEntry.key;
      final String fontFile = fontEntry.value;

      fontManifest.add(<String, dynamic>{
        'family': family,
        'fonts': <dynamic>[
          <String, String>{'asset': 'fonts/$fontFile'},
        ],
      });

      final io.File sourceTtf = io.File(pathlib.join(fontsPath, fontFile));
      final io.File destinationTtf = io.File(
        pathlib.join(environment.webTestsArtifactsDir.path, 'assets', 'fonts', fontFile),
      );
      await destinationTtf.create(recursive: true);
      await sourceTtf.copy(destinationTtf.path);
    }

    final io.File fontManifestFile = io.File(
      pathlib.join(environment.webTestsArtifactsDir.path, 'assets', 'FontManifest.json'),
    );
    await fontManifestFile.create(recursive: true);
    await fontManifestFile.writeAsString(const JsonEncoder.withIndent('  ').convert(fontManifest));

    final io.Directory fallbackFontsSource = io.Directory(
      pathlib.join(
        environment.engineSrcDir.path,
        'flutter',
        'third_party',
        'google_fonts_for_unit_tests',
      ),
    );
    final String fallbackFontsDestinationPath = pathlib.join(
      environment.webTestsArtifactsDir.path,
      'assets',
      'fallback_fonts',
    );
    for (final io.File file in fallbackFontsSource.listSync(recursive: true).whereType<io.File>()) {
      final String relativePath = pathlib.relative(file.path, from: fallbackFontsSource.path);
      final io.File destinationFile = io.File(
        pathlib.join(fallbackFontsDestinationPath, relativePath),
      );
      if (!destinationFile.parent.existsSync()) {
        destinationFile.parent.createSync(recursive: true);
      }
      file.copySync(destinationFile.path);
    }
  }

  Future<void> copySkiaTestImages() async {
    final io.Directory testImagesDir = io.Directory(
      pathlib.join(
        environment.engineSrcDir.path,
        'flutter',
        'third_party',
        'skia',
        'resources',
        'images',
      ),
    );

    for (final io.File imageFile in testImagesDir.listSync(recursive: true).whereType<io.File>()) {
      // Skip files that are used by Skia to test handling of invalid input.
      if (pathlib.basename(imageFile.path).contains('invalid')) {
        continue;
      }
      final io.File destination = io.File(
        pathlib.join(
          environment.webTestsArtifactsDir.path,
          'test_images',
          pathlib.relative(imageFile.path, from: testImagesDir.path),
        ),
      );
      destination.createSync(recursive: true);
      await imageFile.copy(destination.path);
    }
  }

  Future<void> copyFlutterJsFiles(String sourcePath) async {
    final io.Directory flutterJsInputDirectory = io.Directory(sourcePath);
    final String targetDirectoryPath = pathlib.join(
      environment.webTestsArtifactsDir.path,
      'flutter_js',
    );

    for (final io.File sourceFile
        in flutterJsInputDirectory.listSync(recursive: true).whereType<io.File>()) {
      final String relativePath = pathlib.relative(
        sourceFile.path,
        from: flutterJsInputDirectory.path,
      );
      final String targetPath = pathlib.join(targetDirectoryPath, relativePath);
      final io.File targetFile = io.File(targetPath);
      if (!targetFile.parent.existsSync()) {
        targetFile.parent.createSync(recursive: true);
      }
      sourceFile.copySync(targetPath);
    }
  }

  Future<void> copyWasmLibrary(
    String libraryName,
    String sourcePath,
    String destinationPath,
  ) async {
    final String targetDirectoryPath = pathlib.join(
      environment.webTestsArtifactsDir.path,
      destinationPath,
    );

    for (final String filename in <String>[
      '$libraryName.js',
      '$libraryName.wasm',
      '$libraryName.wasm.map',
    ]) {
      final io.File sourceFile = io.File(pathlib.join(sourcePath, filename));
      final io.File targetFile = io.File(pathlib.join(targetDirectoryPath, filename));
      if (!sourceFile.existsSync()) {
        if (filename.endsWith('.map')) {
          // Sourcemaps are only generated under certain build conditions, so
          // they are optional.
          continue;
        }
        {
          throw ToolExit('Built artifact not found at path "$sourceFile".');
        }
      }
      await targetFile.create(recursive: true);
      await sourceFile.copy(targetFile.path);
    }
  }

  Future<void> buildHostPage() async {
    final String hostDartPath = pathlib.join('lib', 'static', 'host.dart');
    final io.File hostDartFile = io.File(
      pathlib.join(environment.webEngineTesterRootDir.path, hostDartPath),
    );
    final String targetDirectoryPath = pathlib.join(environment.webTestsArtifactsDir.path, 'host');
    io.Directory(targetDirectoryPath).createSync(recursive: true);
    final String targetFilePath = pathlib.join(targetDirectoryPath, 'host.dart');

    const List<String> staticFiles = <String>['favicon.ico', 'host.css', 'index.html'];
    for (final String staticFilePath in staticFiles) {
      final io.File source = io.File(
        pathlib.join(environment.webEngineTesterRootDir.path, 'lib', 'static', staticFilePath),
      );
      final io.File destination = io.File(pathlib.join(targetDirectoryPath, staticFilePath));
      await source.copy(destination.path);
    }

    final io.File timestampFile = io.File(
      pathlib.join(environment.webEngineTesterRootDir.path, '$targetFilePath.js.timestamp'),
    );

    final String timestamp = hostDartFile.statSync().modified.millisecondsSinceEpoch.toString();
    if (timestampFile.existsSync()) {
      final String lastBuildTimestamp = timestampFile.readAsStringSync();
      if (lastBuildTimestamp == timestamp) {
        // The file is still fresh. No need to rebuild.
        return;
      } else {
        // Record new timestamp, but don't return. We need to rebuild.
        print('${hostDartFile.path} timestamp changed. Rebuilding.');
      }
    } else {
      print('Building ${hostDartFile.path}.');
    }

    int exitCode = await runProcess(environment.dartExecutable, <String>[
      'pub',
      'get',
    ], workingDirectory: environment.webEngineTesterRootDir.path);

    if (exitCode != 0) {
      throw ToolExit(
        'Failed to run pub get for web_engine_tester, exit code $exitCode',
        exitCode: exitCode,
      );
    }

    exitCode = await runProcess(environment.dartExecutable, <String>[
      'compile',
      'js',
      hostDartPath,
      '-o',
      '$targetFilePath.js',
    ], workingDirectory: environment.webEngineTesterRootDir.path);

    if (exitCode != 0) {
      throw ToolExit(
        'Failed to compile ${hostDartFile.path}. Compiler '
        'exited with exit code $exitCode',
        exitCode: exitCode,
      );
    }

    // Record the timestamp to avoid rebuilding unless the file changes.
    timestampFile.writeAsStringSync(timestamp);
  }
}
