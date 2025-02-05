// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../features.dart';
import '../localizations/gen_l10n.dart';
import '../localizations/localizations_utils.dart';
import '../runner/flutter_command.dart';

/// A command to generate localizations source files for a Flutter project.
///
/// It generates Dart localization source files from arb files.
///
/// For a more comprehensive tutorial on the tool, please see the
/// [internationalization guide](https://flutter.dev/to/internationalization).
class GenerateLocalizationsCommand extends FlutterCommand {
  GenerateLocalizationsCommand({
    required FileSystem fileSystem,
    required Logger logger,
    required Artifacts artifacts,
    required ProcessManager processManager,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _artifacts = artifacts,
       _processManager = processManager {
    argParser.addOption(
      'arb-dir',
      help: 'The directory where the template and translated arb files are located.',
    );
    argParser.addOption(
      'output-dir',
      help:
          'The directory where the generated localization classes will be written '
          'if the synthetic-package flag is set to false.\n'
          '\n'
          'If output-dir is specified and the synthetic-package flag is enabled, '
          'this option will be ignored by the tool.\n'
          '\n'
          'The app must import the file specified in the "--output-localization-file" '
          'option from this directory. If unspecified, this defaults to the same '
          'directory as the input directory specified in "--arb-dir".',
    );
    argParser.addOption(
      'template-arb-file',
      help:
          'The template arb file that will be used as the basis for '
          'generating the Dart localization and messages files.',
    );
    argParser.addOption(
      'output-localization-file',
      help:
          'The filename for the output localization and localizations '
          'delegate classes.',
    );
    argParser.addOption(
      'untranslated-messages-file',
      help:
          'The location of a file that describes the localization '
          'messages have not been translated yet. Using this option will create '
          'a JSON file at the target location, in the following format:\n'
          '\n'
          '    "locale": ["message_1", "message_2" ... "message_n"]\n'
          '\n'
          'If this option is not specified, a summary of the messages that '
          'have not been translated will be printed on the command line.',
    );
    argParser.addOption(
      'output-class',
      defaultsTo: 'AppLocalizations',
      help:
          'The Dart class name to use for the output localization and '
          'localizations delegate classes.',
    );
    argParser.addMultiOption(
      'preferred-supported-locales',
      valueHelp: 'locale',
      help:
          'The list of preferred supported locales for the application. '
          'By default, the tool will generate the supported locales list in '
          'alphabetical order. Use this flag if you would like to default to '
          'a different locale. '
          'For example, pass in "en_US" if you would like your app to '
          'default to American English on devices that support it. '
          'Pass this option multiple times to define multiple items.',
    );
    argParser.addOption(
      'header',
      help:
          'The header to prepend to the generated Dart localizations '
          'files. This option takes in a string.\n'
          '\n'
          'For example, pass in "/// All localized files." if you would '
          'like this string prepended to the generated Dart file.\n'
          '\n'
          'Alternatively, see the "--header-file" option to pass in a text '
          'file for longer headers.',
    );
    argParser.addOption(
      'header-file',
      help:
          'The header to prepend to the generated Dart localizations '
          'files. The value of this option is the name of the file that '
          'contains the header text which will be inserted at the top '
          'of each generated Dart file.\n'
          '\n'
          'Alternatively, see the "--header" option to pass in a string '
          'for a simpler header.\n'
          '\n'
          'This file should be placed in the directory specified in "--arb-dir".',
    );
    argParser.addFlag(
      'use-deferred-loading',
      help:
          'Whether to generate the Dart localization file with locales imported '
          'as deferred, allowing for lazy loading of each locale in Flutter web.\n'
          '\n'
          'This can reduce a web app’s initial startup time by decreasing the '
          'size of the JavaScript bundle. When this flag is set to true, the '
          'messages for a particular locale are only downloaded and loaded by the '
          'Flutter app as they are needed. For projects with a lot of different '
          'locales and many localization strings, it can be an performance '
          'improvement to have deferred loading. For projects with a small number '
          'of locales, the difference is negligible, and might slow down the start '
          'up compared to bundling the localizations with the rest of the '
          'application.\n'
          '\n'
          'This flag does not affect other platforms such as mobile or desktop.',
    );
    argParser.addOption(
      'gen-inputs-and-outputs-list',
      valueHelp: 'path-to-output-directory',
      help:
          'When specified, the tool generates a JSON file containing the '
          "tool's inputs and outputs named gen_l10n_inputs_and_outputs.json.\n"
          '\n'
          'This can be useful for keeping track of which files of the Flutter '
          'project were used when generating the latest set of localizations. '
          "For example, the Flutter tool's build system uses this file to "
          'keep track of when to call gen_l10n during hot reload.\n'
          '\n'
          'The value of this option is the directory where the JSON file will be '
          'generated.\n'
          '\n'
          'When null, the JSON file will not be generated.',
    );
    argParser.addFlag(
      'synthetic-package',
      defaultsTo: !featureFlags.isExplicitPackageDependenciesEnabled,
      help:
          'Determines whether or not the generated output files will be '
          'generated as a synthetic package or at a specified directory in '
          'the Flutter project.\n'
          '\n'
          'DEPRECATED: https://flutter.dev/to/flutter-gen-deprecation.\n'
          '\n'
          'When synthetic-package is set to false, it will generate the '
          'localizations files in the directory specified by arb-dir by default.\n',
    );
    argParser.addOption(
      'project-dir',
      valueHelp: 'absolute/path/to/flutter/project',
      help:
          'When specified, the tool uses the path passed into this option '
          'as the directory of the root Flutter project.\n'
          '\n'
          'When null, the relative path to the present working directory will be used.',
    );
    argParser.addFlag(
      'required-resource-attributes',
      help:
          'Requires all resource ids to contain a corresponding resource attribute.\n'
          '\n'
          'By default, simple messages will not require metadata, but it is highly '
          'recommended as this provides context for the meaning of a message to '
          'readers.\n'
          '\n'
          'Resource attributes are still required for plural messages.',
    );
    argParser.addFlag(
      'nullable-getter',
      defaultsTo: true,
      help:
          'Whether or not the localizations class getter is nullable.\n'
          '\n'
          'By default, this value is set to true so that '
          'Localizations.of(context) returns a nullable value '
          'for backwards compatibility. If this value is set to false, then '
          'a null check is performed on the returned value of '
          'Localizations.of(context), removing the need for null checking in '
          'user code.',
    );
    argParser.addFlag(
      'format',
      help:
          'When specified, the "dart format" command is run after generating the localization files.',
    );
    argParser.addFlag(
      'use-escaping',
      help:
          'Whether or not to use escaping for messages.\n'
          '\n'
          'By default, this value is set to false for backwards compatibility. '
          'Turning this flag on will cause the parser to treat any special characters '
          'contained within pairs of single quotes as normal strings and treat all '
          'consecutive pairs of single quotes as a single quote character.',
    );
    argParser.addFlag(
      'suppress-warnings',
      help: 'When specified, all warnings will be suppressed.\n',
    );
    argParser.addFlag(
      'relax-syntax',
      help:
          'When specified, the syntax will be relaxed so that the special character '
          '"{" is treated as a string if it is not followed by a valid placeholder '
          'and "}" is treated as a string if it does not close any previous "{" '
          'that is treated as a special character.',
    );
    argParser.addFlag(
      'use-named-parameters',
      help: 'Whether or not to use named parameters for the generated localization methods.',
    );
  }

  final FileSystem _fileSystem;
  final Logger _logger;
  final Artifacts _artifacts;
  final ProcessManager _processManager;

  @override
  String get description => 'Generate localizations for the current project.';

  @override
  String get name => 'gen-l10n';

  @override
  String get category => FlutterCommandCategory.project;

  @override
  Future<FlutterCommandResult> runCommand() async {
    // Validate the rest of the args.
    if (argResults!.rest.isNotEmpty) {
      throwToolExit('Unexpected positional argument "${argResults!.rest.first}".');
    }
    // Keep in mind that this is also defined in the following locations:
    // 1. flutter_tools/lib/src/build_system/targets/localizations.dart
    // 2. flutter_tools/test/general.shard/build_system/targets/localizations_test.dart
    // Keep the value consistent in all three locations to ensure behavior is the
    // same across "flutter gen-l10n" and "flutter run".
    final String defaultArbDir = _fileSystem.path.join('lib', 'l10n');
    // Get all options associated with gen-l10n.
    final LocalizationOptions options;
    if (_fileSystem.file('l10n.yaml').existsSync()) {
      options = parseLocalizationsOptionsFromYAML(
        file: _fileSystem.file('l10n.yaml'),
        logger: _logger,
        defaultArbDir: defaultArbDir,
        defaultSyntheticPackage: !featureFlags.isExplicitPackageDependenciesEnabled,
      );
      _logger.printStatus(
        'Because l10n.yaml exists, the options defined there will be used '
        'instead.\n'
        'To use the command line arguments, delete the l10n.yaml file in the '
        'Flutter project.\n\n',
      );
    } else {
      options = parseLocalizationsOptionsFromCommand(command: this, defaultArbDir: defaultArbDir);
    }

    // Run the localizations generator.
    await generateLocalizations(
      logger: _logger,
      options: options,
      projectDir: _fileSystem.currentDirectory,
      fileSystem: _fileSystem,
      artifacts: _artifacts,
      processManager: _processManager,
    );

    return FlutterCommandResult.success();
  }
}
