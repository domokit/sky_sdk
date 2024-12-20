// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/file_system.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';
import '../template.dart';

class IdeConfigCommand extends FlutterCommand {
  IdeConfigCommand() {
    argParser.addFlag('overwrite', help: 'When performing operations, overwrite existing files.');
    argParser.addFlag(
      'update-templates',
      negatable: false,
      help:
          'Update the templates in the template directory from the current '
          'configuration files. This is the opposite of what $name usually does. '
          'Will search the flutter tree for *.iml files and copy any missing ones '
          'into the template directory. If "--overwrite" is also specified, it will '
          'update any out-of-date files, and remove any deleted files from the '
          'template directory.',
    );
    argParser.addFlag(
      'with-root-module',
      defaultsTo: true,
      help:
          'Also create module that corresponds to the root of Flutter tree. '
          'This makes the entire Flutter tree browsable and searchable in IDE. '
          'Without this flag, only the child modules will be visible in IDE.',
    );
  }

  @override
  final String name = 'ide-config';

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => const <DevelopmentArtifact>{};

  @override
  final String description =
      'Configure the IDE for use in the Flutter tree.\n\n'
      'If run on a Flutter tree that is already configured for the IDE, this '
      'command will add any new configurations, recreate any files that are '
      'missing. If --overwrite is specified, will revert existing files to '
      'the template versions, reset the module list, and return configuration '
      'settings to the template versions.\n\n'
      'This command is intended for Flutter developers to help them set up the '
      "Flutter tree for development in an IDE. It doesn't affect other projects.\n\n"
      'Currently, IntelliJ is the default (and only) IDE that may be configured.';

  @override
  final bool hidden = true;

  @override
  String get invocation => '${runner?.executableName} $name';

  static const String _ideName = 'intellij';
  Directory get _templateDirectory {
    return globals.fs.directory(
      globals.fs.path.join(
        Cache.flutterRoot!,
        'packages',
        'flutter_tools',
        'ide_templates',
        _ideName,
      ),
    );
  }

  Directory get _createTemplatesDirectory {
    return globals.fs.directory(
      globals.fs.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools', 'templates'),
    );
  }

  Directory get _flutterRoot => globals.fs.directory(globals.fs.path.absolute(Cache.flutterRoot!));

  // Returns true if any entire path element is equal to dir.
  bool _hasDirectoryInPath(FileSystemEntity entity, String dir) {
    String path = entity.absolute.path;
    while (path.isNotEmpty && globals.fs.path.dirname(path) != path) {
      if (globals.fs.path.basename(path) == dir) {
        return true;
      }
      path = globals.fs.path.dirname(path);
    }
    return false;
  }

  // Returns true if child is anywhere underneath parent.
  bool _isChildDirectoryOf(FileSystemEntity parent, FileSystemEntity child) {
    return child.absolute.path.startsWith(parent.absolute.path);
  }

  // Checks the contents of the two files to see if they have changes.
  bool _fileIsIdentical(File src, File dest) {
    if (src.lengthSync() != dest.lengthSync()) {
      return false;
    }

    // Test byte by byte. We're assuming that these are small files.
    final List<int> srcBytes = src.readAsBytesSync();
    final List<int> destBytes = dest.readAsBytesSync();
    for (int i = 0; i < srcBytes.length; ++i) {
      if (srcBytes[i] != destBytes[i]) {
        return false;
      }
    }
    return true;
  }

  // Discovers and syncs with existing configuration files in the Flutter tree.
  void _handleTemplateUpdate() {
    if (!_flutterRoot.existsSync()) {
      return;
    }

    final Set<String> manifest = <String>{};
    final Iterable<File> flutterFiles = _flutterRoot.listSync(recursive: true).whereType<File>();
    for (final File srcFile in flutterFiles) {
      final String relativePath = globals.fs.path.relative(
        srcFile.path,
        from: _flutterRoot.absolute.path,
      );

      // Skip template files in both the ide_templates and templates
      // directories to avoid copying onto themselves.
      if (_isChildDirectoryOf(_templateDirectory, srcFile) ||
          _isChildDirectoryOf(_createTemplatesDirectory, srcFile)) {
        continue;
      }

      // Skip files we aren't interested in.
      final RegExp trackedIdeaFileRegExp = RegExp(r'(\.name|modules.xml|vcs.xml)$');
      final bool isATrackedIdeaFile =
          _hasDirectoryInPath(srcFile, '.idea') &&
          (trackedIdeaFileRegExp.hasMatch(relativePath) ||
              _hasDirectoryInPath(srcFile, 'runConfigurations'));
      final bool isAnImlOutsideIdea = !isATrackedIdeaFile && srcFile.path.endsWith('.iml');
      if (!isATrackedIdeaFile && !isAnImlOutsideIdea) {
        continue;
      }

      final File finalDestinationFile = globals.fs.file(
        globals.fs.path.absolute(
          _templateDirectory.absolute.path,
          '$relativePath${Template.copyTemplateExtension}',
        ),
      );
      final String relativeDestination = globals.fs.path.relative(
        finalDestinationFile.path,
        from: _flutterRoot.absolute.path,
      );
      if (finalDestinationFile.existsSync()) {
        if (_fileIsIdentical(srcFile, finalDestinationFile)) {
          globals.printTrace('  $relativeDestination (identical)');
          manifest.add('$relativePath${Template.copyTemplateExtension}');
          continue;
        }
        if (boolArg('overwrite')) {
          finalDestinationFile.deleteSync();
          globals.printStatus('  $relativeDestination (overwritten)');
        } else {
          globals.printTrace('  $relativeDestination (existing - skipped)');
          manifest.add('$relativePath${Template.copyTemplateExtension}');
          continue;
        }
      } else {
        globals.printStatus('  $relativeDestination (added)');
      }
      final Directory finalDestinationDir = globals.fs.directory(finalDestinationFile.dirname);
      if (!finalDestinationDir.existsSync()) {
        globals.printTrace("  ${finalDestinationDir.path} doesn't exist, creating.");
        finalDestinationDir.createSync(recursive: true);
      }
      srcFile.copySync(finalDestinationFile.path);
      manifest.add('$relativePath${Template.copyTemplateExtension}');
    }

    // If we're not overwriting, then we're not going to remove missing items either.
    if (!boolArg('overwrite')) {
      return;
    }

    // Look for any files under the template dir that don't exist in the manifest and remove
    // them.
    final Iterable<File> templateFiles =
        _templateDirectory.listSync(recursive: true).whereType<File>();
    for (final File templateFile in templateFiles) {
      final String relativePath = globals.fs.path.relative(
        templateFile.absolute.path,
        from: _templateDirectory.absolute.path,
      );
      if (!manifest.contains(relativePath)) {
        templateFile.deleteSync();
        final String relativeDestination = globals.fs.path.relative(
          templateFile.path,
          from: _flutterRoot.absolute.path,
        );
        globals.printStatus('  $relativeDestination (removed)');
      }
      // If the directory is now empty, then remove it, and do the same for its parent,
      // until we escape to the template directory.
      Directory parentDir = globals.fs.directory(templateFile.dirname);
      while (parentDir.listSync().isEmpty) {
        parentDir.deleteSync();
        globals.printTrace(
          '  ${globals.fs.path.relative(parentDir.absolute.path)} (empty directory - removed)',
        );
        parentDir = globals.fs.directory(parentDir.dirname);
        if (globals.fs.path.isWithin(_templateDirectory.absolute.path, parentDir.absolute.path)) {
          break;
        }
      }
    }
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final List<String> rest = argResults?.rest ?? <String>[];
    if (rest.isNotEmpty) {
      throwToolExit('Currently, the only supported IDE is IntelliJ\n$usage', exitCode: 2);
    }

    if (boolArg('update-templates')) {
      _handleTemplateUpdate();
      return FlutterCommandResult.success();
    }

    final String flutterRoot = globals.fs.path.absolute(Cache.flutterRoot!);
    final String dirPath = globals.fs.path.normalize(
      globals.fs.directory(globals.fs.path.absolute(Cache.flutterRoot!)).absolute.path,
    );

    final String? error = _validateFlutterDir(dirPath, flutterRoot: flutterRoot);
    if (error != null) {
      throwToolExit(error);
    }

    globals.printStatus('Updating IDE configuration for Flutter tree at $dirPath...');
    int generatedCount = 0;
    generatedCount += _renderTemplate(_ideName, dirPath, <String, Object>{
      'withRootModule': boolArg('with-root-module'),
      'android': true,
    });

    globals.printStatus('Wrote $generatedCount files.');
    globals.printStatus('');
    globals.printStatus(
      'Your IntelliJ configuration is now up to date. It is prudent to '
      'restart IntelliJ, if running.',
    );

    return FlutterCommandResult.success();
  }

  int _renderTemplate(String templateName, String dirPath, Map<String, Object> context) {
    final Template template = Template(
      _templateDirectory,
      null,
      fileSystem: globals.fs,
      logger: globals.logger,
      templateRenderer: globals.templateRenderer,
    );
    return template.render(
      globals.fs.directory(dirPath),
      context,
      overwriteExisting: boolArg('overwrite'),
    );
  }
}

/// Return null if the flutter root directory is a valid destination. Return a
/// validation message if we should disallow the directory.
String? _validateFlutterDir(String dirPath, {String? flutterRoot}) {
  final FileSystemEntityType type = globals.fs.typeSync(dirPath);

  // ignore: exhaustive_cases, https://github.com/dart-lang/linter/issues/3017
  switch (type) {
    case FileSystemEntityType.link:
      // Do not overwrite links.
      return "Invalid project root dir: '$dirPath' - refers to a link.";
    case FileSystemEntityType.file:
    case FileSystemEntityType.directory:
    case FileSystemEntityType.notFound:
      return null;
  }
  // In the case of any other [FileSystemEntityType]s, like the deprecated ones, return null.
  return null;
}
