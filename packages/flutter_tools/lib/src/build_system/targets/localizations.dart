// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../convert.dart';
import '../../features.dart';
import '../../localizations/gen_l10n.dart';
import '../../localizations/localizations_utils.dart';
import '../build_system.dart';
import '../depfile.dart';

const String _kDependenciesFileName = 'gen_l10n_inputs_and_outputs.json';

/// A build step that runs the generate localizations script from
/// dev/tool/localizations.
class GenerateLocalizationsTarget extends Target {
  const GenerateLocalizationsTarget();

  @override
  List<Target> get dependencies => <Target>[];

  @override
  List<Source> get inputs => <Source>[
    // This is added as a convenience for developing the tool.
    const Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/localizations.dart',
    ),
  ];

  @override
  String get name => 'gen_localizations';

  @override
  List<Source> get outputs => <Source>[];

  @override
  List<String> get depfiles => <String>['gen_localizations.d'];

  @override
  bool canSkip(Environment environment) {
    final File configFile = environment.projectDir.childFile('l10n.yaml');
    return !configFile.existsSync();
  }

  @override
  Future<void> build(Environment environment) async {
    final File configFile = environment.projectDir.childFile('l10n.yaml');
    assert(configFile.existsSync());

    // Keep in mind that this is also defined in the following locations:
    // 1. flutter_tools/lib/src/commands/generate_localizations.dart
    // 2. flutter_tools/test/general.shard/build_system/targets/localizations_test.dart
    // Keep the value consistent in all three locations to ensure behavior is the
    // same across "flutter gen-l10n" and "flutter run".
    final String defaultArbDir = environment.fileSystem.path.join('lib', 'l10n');

    final LocalizationOptions options = parseLocalizationsOptionsFromYAML(
      file: configFile,
      logger: environment.logger,
      defaultArbDir: defaultArbDir,
      defaultSyntheticPackage: !featureFlags.isExplicitPackageDependenciesEnabled,
    );
    await generateLocalizations(
      logger: environment.logger,
      options: options,
      projectDir: environment.projectDir,
      dependenciesDir: environment.buildDir,
      fileSystem: environment.fileSystem,
      artifacts: environment.artifacts,
      processManager: environment.processManager,
    );

    final Map<String, Object?> dependencies =
        json.decode(environment.buildDir.childFile(_kDependenciesFileName).readAsStringSync())
            as Map<String, Object?>;
    final List<Object?>? inputs = dependencies['inputs'] as List<Object?>?;
    final List<Object?>? outputs = dependencies['outputs'] as List<Object?>?;
    final Depfile depfile = Depfile(
      <File>[
        configFile,
        if (inputs != null)
          for (final Object inputFile in inputs.whereType<Object>())
            environment.fileSystem.file(inputFile),
      ],
      <File>[
        if (outputs != null)
          for (final Object outputFile in outputs.whereType<Object>())
            environment.fileSystem.file(outputFile),
      ],
    );
    environment.depFileService.writeToFile(
      depfile,
      environment.buildDir.childFile('gen_localizations.d'),
    );
  }
}
