// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:package_config/package_config.dart';
import 'package:package_config/package_config_types.dart';

import 'base/common.dart';
import 'base/context.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/template.dart';
import 'cache.dart';
import 'dart/package_map.dart';

/// The Kotlin keywords which are not Java keywords.
/// They are escaped in Kotlin files.
///
/// https://kotlinlang.org/docs/keyword-reference.html
const List<String> kReservedKotlinKeywords = <String>['when', 'in', 'is'];

/// Provides the path where templates used by flutter_tools are stored.
class TemplatePathProvider {
  const TemplatePathProvider();

  /// Returns the directory containing the 'name' template directory.
  Directory directoryInPackage(String name, FileSystem fileSystem) {
    final String templatesDir = fileSystem.path.join(
      Cache.flutterRoot!,
      'packages',
      'flutter_tools',
      'templates',
    );
    return fileSystem.directory(fileSystem.path.join(templatesDir, name));
  }

  /// Returns the directory containing the 'name' template directory in
  /// flutter_template_images, to resolve image placeholder against.
  /// if 'name' is null, return the parent template directory.
  Future<Directory> imageDirectory(String? name, FileSystem fileSystem, Logger logger) async {
    final String toolPackagePath = fileSystem.path.join(
      Cache.flutterRoot!,
      'packages',
      'flutter_tools',
    );
    final String packageFilePath = fileSystem.path.join(
      toolPackagePath,
      '.dart_tool',
      'package_config.json',
    );
    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      fileSystem.file(packageFilePath),
      logger: logger,
    );
    final Uri? imagePackageLibDir = packageConfig['flutter_template_images']?.packageUriRoot;
    final Directory templateDirectory = fileSystem
        .directory(imagePackageLibDir)
        .parent
        .childDirectory('templates');
    return name == null ? templateDirectory : templateDirectory.childDirectory(name);
  }
}

TemplatePathProvider get templatePathProvider =>
    context.get<TemplatePathProvider>() ?? const TemplatePathProvider();

/// Expands templates in a directory to a destination. All files that must
/// undergo template expansion should end with the '.tmpl' extension. All files
/// that should be replaced with the corresponding image from
/// flutter_template_images should end with the '.img.tmpl' extension. All other
/// files are ignored. In case the contents of entire directories must be copied
/// as is, the directory itself can end with '.tmpl' extension. Files within
/// such a directory may also contain the '.tmpl' or '.img.tmpl' extensions and
/// will be considered for expansion. In case certain files need to be copied
/// but without template expansion (data files, etc.), the '.copy.tmpl'
/// extension may be used. Furthermore, templates may contain additional
/// test files intended to run on the CI. Test files must end in `.test.tmpl`
/// and are only included when the --implementation-tests flag is enabled.
///
/// Folders with platform/language-specific content must be named
/// '<platform>-<language>.tmpl'.
///
/// Files in the destination will contain none of the '.tmpl', '.copy.tmpl',
/// 'img.tmpl', or '-<language>.tmpl' extensions.
class Template {
  factory Template(
    Directory templateSource,
    Directory? imageSourceDir, {
    required FileSystem fileSystem,
    required Logger logger,
    required TemplateRenderer templateRenderer,
    Set<Uri>? templateManifest,
  }) {
    return Template._(
      <Directory>[templateSource],
      imageSourceDir != null ? <Directory>[imageSourceDir] : <Directory>[],
      fileSystem: fileSystem,
      logger: logger,
      templateRenderer: templateRenderer,
      templateManifest: templateManifest,
    );
  }

  Template._(
    List<Directory> templateSources,
    this.imageSourceDirectories, {
    required FileSystem fileSystem,
    required Logger logger,
    required TemplateRenderer templateRenderer,
    required Set<Uri>? templateManifest,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _templateRenderer = templateRenderer,
       _templateManifest = templateManifest ?? <Uri>{} {
    for (final Directory sourceDirectory in templateSources) {
      if (!sourceDirectory.existsSync()) {
        throwToolExit('Template source directory does not exist: ${sourceDirectory.absolute.path}');
      }
    }

    final Map<FileSystemEntity, Directory> templateFiles = <FileSystemEntity, Directory>{
      for (final Directory sourceDirectory in templateSources)
        for (final FileSystemEntity entity in sourceDirectory.listSync(recursive: true))
          entity: sourceDirectory,
    };
    for (final FileSystemEntity entity in templateFiles.keys.whereType<File>()) {
      if (_templateManifest.isNotEmpty &&
          !_templateManifest.contains(Uri.file(entity.absolute.path))) {
        _logger.printTrace('Skipping ${entity.absolute.path}, missing from the template manifest.');
        // Skip stale files in the flutter_tools directory.
        continue;
      }

      final String relativePath = fileSystem.path.relative(
        entity.path,
        from: templateFiles[entity]!.absolute.path,
      );
      if (relativePath.contains(templateExtension)) {
        // If '.tmpl' appears anywhere within the path of this entity, it is
        // a candidate for rendering. This catches cases where the folder
        // itself is a template.
        _templateFilePaths[relativePath] = fileSystem.path.absolute(entity.path);
      }
    }
  }

  static Future<Template> fromName(
    String name, {
    required FileSystem fileSystem,
    required Set<Uri>? templateManifest,
    required Logger logger,
    required TemplateRenderer templateRenderer,
  }) async {
    // All named templates are placed in the 'templates' directory
    final Directory templateDir = templatePathProvider.directoryInPackage(name, fileSystem);
    final Directory imageDir = await templatePathProvider.imageDirectory(name, fileSystem, logger);
    return Template._(
      <Directory>[templateDir],
      <Directory>[imageDir],
      fileSystem: fileSystem,
      logger: logger,
      templateRenderer: templateRenderer,
      templateManifest: templateManifest,
    );
  }

  static Future<Template> merged(
    List<String> names,
    Directory directory, {
    required FileSystem fileSystem,
    required Set<Uri> templateManifest,
    required Logger logger,
    required TemplateRenderer templateRenderer,
  }) async {
    // All named templates are placed in the 'templates' directory
    return Template._(
      <Directory>[
        for (final String name in names) templatePathProvider.directoryInPackage(name, fileSystem),
      ],
      <Directory>[
        for (final String name in names)
          if ((await templatePathProvider.imageDirectory(name, fileSystem, logger)).existsSync())
            await templatePathProvider.imageDirectory(name, fileSystem, logger),
      ],
      fileSystem: fileSystem,
      logger: logger,
      templateRenderer: templateRenderer,
      templateManifest: templateManifest,
    );
  }

  final FileSystem _fileSystem;
  final Logger _logger;
  final Set<Uri> _templateManifest;
  final TemplateRenderer _templateRenderer;

  static const String templateExtension = '.tmpl';
  static const String copyTemplateExtension = '.copy.tmpl';
  static const String imageTemplateExtension = '.img.tmpl';
  static const String testTemplateExtension = '.test.tmpl';
  final Pattern _kTemplateLanguageVariant = RegExp(r'(\w+)-(\w+)\.tmpl.*');
  final List<Directory> imageSourceDirectories;

  final Map<String /* relative */, String /* absolute source */> _templateFilePaths =
      <String, String>{};

  /// Render the template into [directory].
  ///
  /// May throw a [ToolExit] if the directory is not writable.
  int render(
    Directory destination,
    Map<String, Object?> context, {
    bool overwriteExisting = true,
    bool printStatusWhenWriting = true,
  }) {
    try {
      destination.createSync(recursive: true);
    } on FileSystemException catch (err) {
      _logger.printError(err.toString());
      throwToolExit('Failed to flutter create at ${destination.path}.');
    }
    int fileCount = 0;
    final bool implementationTests = (context['implementationTests'] as bool?) ?? false;

    /// Returns the resolved destination path corresponding to the specified
    /// raw destination path, after performing language filtering and template
    /// expansion on the path itself.
    ///
    /// Returns null if the given raw destination path has been filtered.
    String? renderPath(String relativeDestinationPath) {
      final Match? match = _kTemplateLanguageVariant.matchAsPrefix(relativeDestinationPath);
      if (match != null) {
        final String platform = match.group(1)!;
        final String? language = context['${platform}Language'] as String?;
        if (language != match.group(2)) {
          return null;
        }
        relativeDestinationPath = relativeDestinationPath.replaceAll(
          '$platform-$language.tmpl',
          platform,
        );
      }

      final bool android = (context['android'] as bool?) ?? false;
      if (relativeDestinationPath.contains('android') && !android) {
        return null;
      }

      final bool ios = (context['ios'] as bool?) ?? false;
      if (relativeDestinationPath.contains('ios') && !ios) {
        return null;
      }

      // Only build a web project if explicitly asked.
      final bool web = (context['web'] as bool?) ?? false;
      if (relativeDestinationPath.contains('web') && !web) {
        return null;
      }
      // Only build a Linux project if explicitly asked.
      final bool linux = (context['linux'] as bool?) ?? false;
      if (relativeDestinationPath.startsWith('linux.tmpl') && !linux) {
        return null;
      }
      // Only build a macOS project if explicitly asked.
      final bool macOS = (context['macos'] as bool?) ?? false;
      if (relativeDestinationPath.startsWith('macos.tmpl') && !macOS) {
        return null;
      }
      // Only build a Windows project if explicitly asked.
      final bool windows = (context['windows'] as bool?) ?? false;
      if (relativeDestinationPath.startsWith('windows.tmpl') && !windows) {
        return null;
      }

      final String? projectName = context['projectName'] as String?;
      final String? androidIdentifier = context['androidIdentifier'] as String?;
      final String? pluginClass = context['pluginClass'] as String?;
      final String? pluginClassSnakeCase = context['pluginClassSnakeCase'] as String?;
      final String destinationDirPath = destination.absolute.path;
      final String pathSeparator = _fileSystem.path.separator;
      String finalDestinationPath = _fileSystem.path
          .join(destinationDirPath, relativeDestinationPath)
          .replaceAll(copyTemplateExtension, '')
          .replaceAll(imageTemplateExtension, '')
          .replaceAll(testTemplateExtension, '')
          .replaceAll(templateExtension, '');

      if (android && androidIdentifier != null) {
        finalDestinationPath = finalDestinationPath.replaceAll(
          'androidIdentifier',
          androidIdentifier.replaceAll('.', pathSeparator),
        );
      }
      if (projectName != null) {
        finalDestinationPath = finalDestinationPath.replaceAll('projectName', projectName);
      }
      // This must be before the pluginClass replacement step.
      if (pluginClassSnakeCase != null) {
        finalDestinationPath = finalDestinationPath.replaceAll(
          'pluginClassSnakeCase',
          pluginClassSnakeCase,
        );
      }
      if (pluginClass != null) {
        finalDestinationPath = finalDestinationPath.replaceAll('pluginClass', pluginClass);
      }
      return finalDestinationPath;
    }

    _templateFilePaths.forEach((String relativeDestinationPath, String absoluteSourcePath) {
      final bool withRootModule = context['withRootModule'] as bool? ?? false;
      if (!withRootModule && absoluteSourcePath.contains('flutter_root')) {
        return;
      }

      if (!implementationTests && absoluteSourcePath.contains(testTemplateExtension)) {
        return;
      }

      final String? finalDestinationPath = renderPath(relativeDestinationPath);
      if (finalDestinationPath == null) {
        return;
      }
      final File finalDestinationFile = _fileSystem.file(finalDestinationPath);
      final String relativePathForLogging = _fileSystem.path.relative(finalDestinationFile.path);

      // Step 1: Check if the file needs to be overwritten.

      if (finalDestinationFile.existsSync()) {
        if (overwriteExisting) {
          finalDestinationFile.deleteSync(recursive: true);
          if (printStatusWhenWriting) {
            _logger.printStatus('  $relativePathForLogging (overwritten)');
          }
        } else {
          // The file exists but we cannot overwrite it, move on.
          if (printStatusWhenWriting) {
            _logger.printTrace('  $relativePathForLogging (existing - skipped)');
          }
          return;
        }
      } else {
        if (printStatusWhenWriting) {
          _logger.printStatus('  $relativePathForLogging (created)');
        }
      }

      fileCount += 1;

      finalDestinationFile.createSync(recursive: true);
      final File sourceFile = _fileSystem.file(absoluteSourcePath);

      // Step 2: If the absolute paths ends with a '.copy.tmpl', this file does
      //         not need mustache rendering but needs to be directly copied.

      if (sourceFile.path.endsWith(copyTemplateExtension)) {
        sourceFile.copySync(finalDestinationFile.path);

        return;
      }

      // Step 3: If the absolute paths ends with a '.img.tmpl', this file needs
      //         to be copied from the template image package.

      if (sourceFile.path.endsWith(imageTemplateExtension)) {
        final List<File> potentials = <File>[
          for (final Directory imageSourceDir in imageSourceDirectories)
            _fileSystem.file(
              _fileSystem.path.join(
                imageSourceDir.path,
                relativeDestinationPath.replaceAll(imageTemplateExtension, ''),
              ),
            ),
        ];

        if (potentials.any((File file) => file.existsSync())) {
          final File imageSourceFile = potentials.firstWhere((File file) => file.existsSync());

          imageSourceFile.copySync(finalDestinationFile.path);
        } else {
          throwToolExit('Image File not found ${finalDestinationFile.path}');
        }

        return;
      }

      // Step 4: If the absolute path ends with a '.tmpl', this file needs
      //         rendering via mustache.

      if (sourceFile.path.endsWith(templateExtension)) {
        final String templateContents = sourceFile.readAsStringSync();
        final String? androidIdentifier = context['androidIdentifier'] as String?;
        if (finalDestinationFile.path.endsWith('.kt') && androidIdentifier != null) {
          context['androidIdentifier'] = _escapeKotlinKeywords(androidIdentifier);
        }

        // Use a copy of the context,
        // since the original is used in rendering other templates.
        final Map<String, Object?> localContext =
            finalDestinationFile.path.endsWith('.yaml')
                ? _createEscapedContextCopy(context)
                : context;

        final String renderedContents = _templateRenderer.renderString(
          templateContents,
          localContext,
        );

        finalDestinationFile.writeAsStringSync(renderedContents);

        return;
      }

      // Step 5: This file does not end in .tmpl but is in a directory that
      //         does. Directly copy the file to the destination.
      sourceFile.copySync(finalDestinationFile.path);
    });

    return fileCount;
  }
}

/// Create a copy of the given [context], escaping its values when necessary.
///
/// Returns the copied context.
Map<String, Object?> _createEscapedContextCopy(Map<String, Object?> context) {
  final Map<String, Object?> localContext = Map<String, Object?>.of(context);

  final String? description = localContext['description'] as String?;

  if (description != null && description.isNotEmpty) {
    localContext['description'] = escapeYamlString(description);
  }

  return localContext;
}

String _escapeKotlinKeywords(String androidIdentifier) {
  final List<String> segments = androidIdentifier.split('.');
  final List<String> correctedSegments =
      segments
          .map(
            (String segment) => kReservedKotlinKeywords.contains(segment) ? '`$segment`' : segment,
          )
          .toList();
  return correctedSegments.join('.');
}

String escapeYamlString(String value) {
  final StringBuffer result = StringBuffer();
  result.write('"');
  for (final int rune in value.runes) {
    result.write(switch (rune) {
      0x00 => r'\0',
      0x09 => r'\t',
      0x0A => r'\n',
      0x0D => r'\r',
      0x22 => r'\"',
      0x5C => r'\\',
      < 0x20 => '\\x${rune.toRadixString(16).padLeft(2, "0")}',
      _ => String.fromCharCode(rune),
    });
  }
  result.write('"');
  return result.toString();
}
