// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/io.dart';
import '../base/process.dart';
import '../build_info.dart';
import '../cache.dart';
import '../dart/package_map.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../runner/flutter_command.dart' show FlutterCommandResult;
import '../windows/build_windows.dart';
import 'build.dart';

class BuildPreviewCommand extends BuildSubCommand {
  BuildPreviewCommand({
    required super.logger,
    required super.verboseHelp,
    required this.fs,
    required this.flutterRoot,
    required this.processUtils,
    required this.artifacts,
  });

  @override
  final String name = '_preview';

  @override
  final bool hidden = true;

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => <DevelopmentArtifact>{
    DevelopmentArtifact.windows,
  };

  @override
  final String description = 'Build Flutter preview (desktop) app.';

  final FileSystem fs;
  final String flutterRoot;
  final ProcessUtils processUtils;
  final Artifacts artifacts;

  @override
  void requiresPubspecYaml() {}

  static const String appName = 'flutter_preview';

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (!globals.platform.isWindows) {
      throwToolExit('"build _preview" is currently only supported on Windows hosts.');
    }
    final Directory targetDir = fs.systemTempDirectory.createTempSync('flutter-build-preview');
    try {
      final FlutterProject flutterProject = await _createProject(targetDir);

      final File packageConfigFile = findPackageConfigFileOrDefault(flutterProject.directory);

      final BuildInfo buildInfo = BuildInfo(
        BuildMode.debug,
        null, // no flavor
        // users may add icons later
        packageConfigPath: packageConfigFile.path,
        packageConfig: await loadPackageConfigWithLogging(packageConfigFile, logger: logger),
        treeShakeIcons: false,
      );

      // TODO(loic-sharma): Support windows-arm64 preview device, https://github.com/flutter/flutter/issues/139949.
      await buildWindows(flutterProject.windows, buildInfo, TargetPlatform.windows_x64);

      final File previewDevice = targetDir
          .childDirectory(getWindowsBuildDirectory(TargetPlatform.windows_x64))
          .childDirectory('runner')
          .childDirectory('Debug')
          .childFile('$appName.exe');
      if (!previewDevice.existsSync()) {
        throw StateError('Preview device not found at ${previewDevice.absolute.path}');
      }
      final String newPath = artifacts.getArtifactPath(Artifact.flutterPreviewDevice);
      fs.file(newPath).parent.createSync(recursive: true);
      previewDevice.copySync(newPath);
      return FlutterCommandResult.success();
    } finally {
      try {
        targetDir.deleteSync(recursive: true);
      } on FileSystemException catch (exception) {
        logger.printError('Failed to delete ${targetDir.path}\n\n$exception');
      }
    }
  }

  Future<FlutterProject> _createProject(Directory targetDir) async {
    final List<String> cmd = <String>[
      fs.path.join(flutterRoot, 'bin', 'flutter.bat'),
      'create',
      '--empty',
      '--project-name',
      'flutter_preview',
      targetDir.path,
    ];
    final RunResult result = await processUtils.run(cmd, allowReentrantFlutter: true);
    if (result.exitCode != 0) {
      final StringBuffer buffer = StringBuffer(
        '${cmd.join(' ')} exited with code ${result.exitCode}\n',
      );
      buffer.writeln('stdout:\n${result.stdout}\n');
      buffer.writeln('stderr:\n${result.stderr}');
      throw ProcessException(cmd.first, cmd.sublist(1), buffer.toString(), result.exitCode);
    }
    return FlutterProject.fromDirectory(targetDir);
  }
}
