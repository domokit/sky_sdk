// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/user_messages.dart';
import '../base/version.dart';
import '../convert.dart';
import '../doctor_validator.dart';
import '../ios/plist_parser.dart';
import 'intellij.dart';

const String _ultimateEditionTitle = 'IntelliJ IDEA Ultimate Edition';
const String _ultimateEditionId = 'IntelliJIdea';
const String _communityEditionTitle = 'IntelliJ IDEA Community Edition';
const String _communityEditionId = 'IdeaIC';

/// A doctor validator for both Intellij and Android Studio.
abstract class IntelliJValidator extends DoctorValidator {
  IntelliJValidator(
    super.title,
    this.installPath, {
    required FileSystem fileSystem,
    required UserMessages userMessages,
  }) : _fileSystem = fileSystem,
       _userMessages = userMessages;

  final String installPath;
  final FileSystem _fileSystem;
  final UserMessages _userMessages;

  String get version;

  String? get pluginsPath;

  static const Map<String, String> _idToTitle = <String, String>{
    _ultimateEditionId: _ultimateEditionTitle,
    _communityEditionId: _communityEditionTitle,
  };

  static final Version kMinIdeaVersion = Version(2017, 1, 0);

  /// Create a [DoctorValidator] for each installation of Intellij.
  ///
  /// On platforms other than macOS, Linux, and Windows this returns an
  /// empty list.
  static Iterable<DoctorValidator> installedValidators({
    required FileSystem fileSystem,
    required Platform platform,
    required Logger logger,
    required UserMessages userMessages,
    required PlistParser plistParser,
    required ProcessManager processManager,
  }) {
    final FileSystemUtils fileSystemUtils = FileSystemUtils(
      fileSystem: fileSystem,
      platform: platform,
    );
    if (platform.isWindows) {
      return IntelliJValidatorOnWindows.installed(
        fileSystem: fileSystem,
        fileSystemUtils: fileSystemUtils,
        platform: platform,
        userMessages: userMessages,
      );
    }
    if (platform.isLinux) {
      return IntelliJValidatorOnLinux.installed(
        fileSystem: fileSystem,
        fileSystemUtils: fileSystemUtils,
        userMessages: userMessages,
      );
    }
    if (platform.isMacOS) {
      return IntelliJValidatorOnMac.installed(
        fileSystem: fileSystem,
        fileSystemUtils: fileSystemUtils,
        userMessages: userMessages,
        plistParser: plistParser,
        processManager: processManager,
        logger: logger,
      );
    }
    return <DoctorValidator>[];
  }

  @override
  Future<ValidationResult> validateImpl() async {
    final List<ValidationMessage> messages = <ValidationMessage>[];

    if (pluginsPath == null) {
      messages.add(const ValidationMessage.error('Invalid IntelliJ version number.'));
    } else {
      messages.add(ValidationMessage(_userMessages.intellijLocation(installPath)));

      final IntelliJPlugins plugins = IntelliJPlugins(pluginsPath!, fileSystem: _fileSystem);
      plugins.validatePackage(
        messages,
        <String>['flutter-intellij', 'flutter-intellij.jar'],
        'Flutter',
        IntelliJPlugins.kIntellijFlutterPluginUrl,
        minVersion: IntelliJPlugins.kMinFlutterPluginVersion,
      );
      plugins.validatePackage(
        messages,
        <String>['Dart', 'dart'],
        'Dart',
        IntelliJPlugins.kIntellijDartPluginUrl,
      );

      if (_hasIssues(messages)) {
        messages.add(ValidationMessage(_userMessages.intellijPluginInfo));
      }

      _validateIntelliJVersion(messages, kMinIdeaVersion);
    }

    return ValidationResult(
      _hasIssues(messages) ? ValidationType.partial : ValidationType.success,
      messages,
      statusInfo: _userMessages.intellijStatusInfo(version),
    );
  }

  bool _hasIssues(List<ValidationMessage> messages) {
    return messages.any((ValidationMessage message) => message.isError);
  }

  void _validateIntelliJVersion(List<ValidationMessage> messages, Version minVersion) {
    final Version? installedVersion = Version.parse(version);
    if (installedVersion == null) {
      return;
    }

    if (installedVersion < minVersion) {
      messages.add(
        ValidationMessage.error(_userMessages.intellijMinimumVersion(minVersion.toString())),
      );
    }
  }
}

/// A windows specific implementation of the intellij validator.
class IntelliJValidatorOnWindows extends IntelliJValidator {
  IntelliJValidatorOnWindows(
    String title,
    this.version,
    String installPath,
    this.pluginsPath, {
    required FileSystem fileSystem,
    required UserMessages userMessages,
  }) : super(title, installPath, fileSystem: fileSystem, userMessages: userMessages);

  @override
  final String version;

  @override
  final String pluginsPath;

  static Iterable<DoctorValidator> installed({
    required FileSystem fileSystem,
    required FileSystemUtils fileSystemUtils,
    required Platform platform,
    required UserMessages userMessages,
  }) {
    final List<DoctorValidator> validators = <DoctorValidator>[];
    if (fileSystemUtils.homeDirPath == null) {
      return validators;
    }

    void addValidator(String title, String version, String installPath, String pluginsPath) {
      final IntelliJValidatorOnWindows validator = IntelliJValidatorOnWindows(
        title,
        version,
        installPath,
        pluginsPath,
        fileSystem: fileSystem,
        userMessages: userMessages,
      );
      for (int index = 0; index < validators.length; index += 1) {
        final DoctorValidator other = validators[index];
        if (other is IntelliJValidatorOnWindows && validator.installPath == other.installPath) {
          if (validator.version.compareTo(other.version) > 0) {
            validators[index] = validator;
          }
          return;
        }
      }
      validators.add(validator);
    }

    // before IntelliJ 2019
    final Directory homeDir = fileSystem.directory(fileSystemUtils.homeDirPath);
    for (final Directory dir in homeDir.listSync().whereType<Directory>()) {
      final String name = fileSystem.path.basename(dir.path);
      IntelliJValidator._idToTitle.forEach((String id, String title) {
        if (name.startsWith('.$id')) {
          final String version = name.substring(id.length + 1);
          String? installPath;
          try {
            installPath =
                fileSystem
                    .file(fileSystem.path.join(dir.path, 'system', '.home'))
                    .readAsStringSync();
          } on FileSystemException {
            // ignored
          }
          if (installPath != null && fileSystem.isDirectorySync(installPath)) {
            final String pluginsPath = fileSystem.path.join(dir.path, 'config', 'plugins');
            addValidator(title, version, installPath, pluginsPath);
          }
        }
      });
    }

    // after IntelliJ 2020
    if (!platform.environment.containsKey('LOCALAPPDATA')) {
      return validators;
    }
    final Directory cacheDir = fileSystem.directory(
      fileSystem.path.join(platform.environment['LOCALAPPDATA']!, 'JetBrains'),
    );
    if (!cacheDir.existsSync()) {
      return validators;
    }
    for (final Directory dir in cacheDir.listSync().whereType<Directory>()) {
      final String name = fileSystem.path.basename(dir.path);
      IntelliJValidator._idToTitle.forEach((String id, String title) {
        if (name.startsWith(id)) {
          final String version = name.substring(id.length);
          String? installPath;
          try {
            installPath =
                fileSystem.file(fileSystem.path.join(dir.path, '.home')).readAsStringSync();
          } on FileSystemException {
            // ignored
          }
          if (installPath != null && fileSystem.isDirectorySync(installPath)) {
            String pluginsPath;
            if (fileSystem.isDirectorySync('$installPath.plugins')) {
              // IntelliJ 2020.3
              pluginsPath = '$installPath.plugins';
              addValidator(title, version, installPath, pluginsPath);
            } else if (platform.environment.containsKey('APPDATA')) {
              final String pluginsPathInAppData = fileSystem.path.join(
                platform.environment['APPDATA']!,
                'JetBrains',
                name,
                'plugins',
              );
              if (fileSystem.isDirectorySync(pluginsPathInAppData)) {
                // IntelliJ 2020.1 ~ 2020.2
                pluginsPath = pluginsPathInAppData;
                addValidator(title, version, installPath, pluginsPath);
              }
            }
          }
        }
      });
    }
    return validators;
  }
}

/// A linux specific implementation of the intellij validator.
class IntelliJValidatorOnLinux extends IntelliJValidator {
  IntelliJValidatorOnLinux(
    String title,
    this.version,
    String installPath,
    this.pluginsPath, {
    required FileSystem fileSystem,
    required UserMessages userMessages,
  }) : super(title, installPath, fileSystem: fileSystem, userMessages: userMessages);

  @override
  final String version;

  @override
  final String pluginsPath;

  static Iterable<DoctorValidator> installed({
    required FileSystem fileSystem,
    required FileSystemUtils fileSystemUtils,
    required UserMessages userMessages,
  }) {
    final List<DoctorValidator> validators = <DoctorValidator>[];
    final String? homeDirPath = fileSystemUtils.homeDirPath;
    if (homeDirPath == null) {
      return validators;
    }

    void addValidator(String title, String version, String installPath, String pluginsPath) {
      final IntelliJValidatorOnLinux validator = IntelliJValidatorOnLinux(
        title,
        version,
        installPath,
        pluginsPath,
        fileSystem: fileSystem,
        userMessages: userMessages,
      );
      for (int index = 0; index < validators.length; index += 1) {
        final DoctorValidator other = validators[index];
        if (other is IntelliJValidatorOnLinux && validator.installPath == other.installPath) {
          if (validator.version.compareTo(other.version) > 0) {
            validators[index] = validator;
          }
          return;
        }
      }
      validators.add(validator);
    }

    // before IntelliJ 2019
    final Directory homeDir = fileSystem.directory(homeDirPath);
    for (final Directory dir in homeDir.listSync().whereType<Directory>()) {
      final String name = fileSystem.path.basename(dir.path);
      IntelliJValidator._idToTitle.forEach((String id, String title) {
        if (name.startsWith('.$id')) {
          final String version = name.substring(id.length + 1);
          String? installPath;
          try {
            installPath =
                fileSystem
                    .file(fileSystem.path.join(dir.path, 'system', '.home'))
                    .readAsStringSync();
          } on FileSystemException {
            // ignored
          }
          if (installPath != null && fileSystem.isDirectorySync(installPath)) {
            final String pluginsPath = fileSystem.path.join(dir.path, 'config', 'plugins');
            addValidator(title, version, installPath, pluginsPath);
          }
        }
      });
    }
    // after IntelliJ 2020 ~
    final Directory cacheDir = fileSystem.directory(
      fileSystem.path.join(homeDirPath, '.cache', 'JetBrains'),
    );
    if (!cacheDir.existsSync()) {
      return validators;
    }
    for (final Directory dir in cacheDir.listSync().whereType<Directory>()) {
      final String name = fileSystem.path.basename(dir.path);
      IntelliJValidator._idToTitle.forEach((String id, String title) {
        if (name.startsWith(id)) {
          final String version = name.substring(id.length);
          String? installPath;
          try {
            installPath =
                fileSystem.file(fileSystem.path.join(dir.path, '.home')).readAsStringSync();
          } on FileSystemException {
            // ignored
          }
          if (installPath != null && fileSystem.isDirectorySync(installPath)) {
            final String pluginsPathInUserHomeDir = fileSystem.path.join(
              homeDirPath,
              '.local',
              'share',
              'JetBrains',
              name,
            );
            if (installPath.contains(fileSystem.path.join('JetBrains', 'Toolbox', 'apps'))) {
              // via JetBrains ToolBox app
              final String pluginsPathInInstallDir = '$installPath.plugins';
              if (fileSystem.isDirectorySync(pluginsPathInUserHomeDir)) {
                // after 2020.2.x
                final String pluginsPath = pluginsPathInUserHomeDir;
                addValidator(title, version, installPath, pluginsPath);
              } else if (fileSystem.isDirectorySync(pluginsPathInInstallDir)) {
                // only 2020.1.X
                final String pluginsPath = pluginsPathInInstallDir;
                addValidator(title, version, installPath, pluginsPath);
              }
            } else {
              // via tar.gz
              final String pluginsPath = pluginsPathInUserHomeDir;
              addValidator(title, version, installPath, pluginsPath);
            }
          }
        }
      });
    }
    return validators;
  }
}

/// A macOS specific implementation of the intellij validator.
class IntelliJValidatorOnMac extends IntelliJValidator {
  IntelliJValidatorOnMac(
    String title,
    this.id,
    String installPath, {
    required FileSystem fileSystem,
    required UserMessages userMessages,
    required PlistParser plistParser,
    required String? homeDirPath,
  }) : _plistParser = plistParser,
       _homeDirPath = homeDirPath,
       super(title, installPath, fileSystem: fileSystem, userMessages: userMessages);

  final String id;
  final PlistParser _plistParser;
  final String? _homeDirPath;

  static const Map<String, String> _dirNameToId = <String, String>{
    'IntelliJ IDEA.app': _ultimateEditionId,
    'IntelliJ IDEA Ultimate.app': _ultimateEditionId,
    'IntelliJ IDEA CE.app': _communityEditionId,
    'IntelliJ IDEA Community Edition.app': _communityEditionId,
  };

  static Iterable<DoctorValidator> installed({
    required FileSystem fileSystem,
    required FileSystemUtils fileSystemUtils,
    required Logger logger,
    required UserMessages userMessages,
    required PlistParser plistParser,
    required ProcessManager processManager,
  }) {
    final List<DoctorValidator> validators = <DoctorValidator>[];
    final String? homeDirPath = fileSystemUtils.homeDirPath;
    final List<String> installPaths = <String>[
      '/Applications',
      if (homeDirPath != null) fileSystem.path.join(homeDirPath, 'Applications'),
    ];

    void checkForIntelliJ(Directory dir) {
      final String name = fileSystem.path.basename(dir.path);
      _dirNameToId.forEach((String dirName, String id) {
        if (name == dirName) {
          assert(IntelliJValidator._idToTitle.containsKey(id));
          final String title = IntelliJValidator._idToTitle[id]!;
          validators.add(
            IntelliJValidatorOnMac(
              title,
              id,
              dir.path,
              fileSystem: fileSystem,
              userMessages: userMessages,
              plistParser: plistParser,
              homeDirPath: homeDirPath,
            ),
          );
        }
      });
    }

    try {
      final Iterable<Directory> installDirs =
          installPaths
              .map(fileSystem.directory)
              .map<List<FileSystemEntity>>(
                (Directory dir) => dir.existsSync() ? dir.listSync() : <FileSystemEntity>[],
              )
              .expand<FileSystemEntity>((List<FileSystemEntity> mappedDirs) => mappedDirs)
              .whereType<Directory>();
      for (final Directory dir in installDirs) {
        checkForIntelliJ(dir);
        if (!dir.path.endsWith('.app')) {
          for (final FileSystemEntity subdirectory in dir.listSync()) {
            if (subdirectory is Directory) {
              checkForIntelliJ(subdirectory);
            }
          }
        }
      }

      // Query Spotlight for unexpected installation locations.
      String ceSpotlightResult = '';
      String ultimateSpotlightResult = '';
      try {
        final ProcessResult ceQueryResult = processManager.runSync(<String>[
          'mdfind',
          'kMDItemCFBundleIdentifier="com.jetbrains.intellij.ce"',
        ]);
        ceSpotlightResult = ceQueryResult.stdout as String;
        final ProcessResult ultimateQueryResult = processManager.runSync(<String>[
          'mdfind',
          'kMDItemCFBundleIdentifier="com.jetbrains.intellij*"',
        ]);
        ultimateSpotlightResult = ultimateQueryResult.stdout as String;
      } on ProcessException {
        // The Spotlight query is a nice-to-have, continue checking known installation locations.
      }

      for (final String installPath in LineSplitter.split(ceSpotlightResult)) {
        if (!validators.whereType<IntelliJValidatorOnMac>().any(
          (IntelliJValidatorOnMac e) => e.installPath == installPath,
        )) {
          validators.add(
            IntelliJValidatorOnMac(
              _communityEditionTitle,
              _communityEditionId,
              installPath,
              fileSystem: fileSystem,
              userMessages: userMessages,
              plistParser: plistParser,
              homeDirPath: homeDirPath,
            ),
          );
        }
      }

      for (final String installPath in LineSplitter.split(ultimateSpotlightResult)) {
        if (!validators.whereType<IntelliJValidatorOnMac>().any(
          (IntelliJValidatorOnMac e) => e.installPath == installPath,
        )) {
          validators.add(
            IntelliJValidatorOnMac(
              _ultimateEditionTitle,
              _ultimateEditionId,
              installPath,
              fileSystem: fileSystem,
              userMessages: userMessages,
              plistParser: plistParser,
              homeDirPath: homeDirPath,
            ),
          );
        }
      }
    } on FileSystemException catch (e) {
      validators.add(
        ValidatorWithResult(
          userMessages.intellijMacUnknownResult,
          ValidationResult(ValidationType.missing, <ValidationMessage>[
            ValidationMessage.error(e.message),
          ]),
        ),
      );
    }

    // Remove JetBrains Toolbox link apps. These tiny apps just
    // link to the full app, will get detected elsewhere in our search.
    validators.removeWhere((DoctorValidator validator) {
      if (validator is! IntelliJValidatorOnMac) {
        return false;
      }
      final String? identifierKey = plistParser.getValueFromFile<String>(
        validator.plistFile,
        PlistParser.kCFBundleIdentifierKey,
      );
      if (identifierKey == null) {
        logger.printTrace(
          'Android Studio/IntelliJ installation at '
          '${validator.installPath} has a null CFBundleIdentifierKey, '
          'which is a required field.',
        );
        return false;
      }
      return identifierKey.contains('com.jetbrains.toolbox.linkapp');
    });

    return validators;
  }

  @visibleForTesting
  String get plistFile {
    _plistFile ??= _fileSystem.path.join(installPath, 'Contents', 'Info.plist');
    return _plistFile!;
  }

  String? _plistFile;

  @override
  String get version {
    return _version ??=
        _plistParser.getValueFromFile<String>(
          plistFile,
          PlistParser.kCFBundleShortVersionStringKey,
        ) ??
        'unknown';
  }

  String? _version;

  @override
  String? get pluginsPath {
    if (_pluginsPath != null) {
      return _pluginsPath!;
    }

    final String? altLocation = _plistParser.getValueFromFile<String>(
      plistFile,
      'JetBrainsToolboxApp',
    );

    if (altLocation != null) {
      _pluginsPath = '$altLocation.plugins';
      return _pluginsPath!;
    }

    final List<String> split = version.split('.');
    if (split.length < 2) {
      return null;
    }
    final String major = split[0];
    final String minor = split[1];

    final String? homeDirPath = _homeDirPath;
    if (homeDirPath != null) {
      String pluginsPath = _fileSystem.path.join(
        homeDirPath,
        'Library',
        'Application Support',
        'JetBrains',
        '$id$major.$minor',
        'plugins',
      );
      // Fallback to legacy location from < 2020.
      if (!_fileSystem.isDirectorySync(pluginsPath)) {
        pluginsPath = _fileSystem.path.join(
          homeDirPath,
          'Library',
          'Application Support',
          '$id$major.$minor',
        );
      }
      _pluginsPath = pluginsPath;
    }

    return _pluginsPath;
  }

  String? _pluginsPath;
}
