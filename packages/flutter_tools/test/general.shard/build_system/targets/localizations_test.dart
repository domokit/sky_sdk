// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/localizations.dart';
import 'package:flutter_tools/src/localizations/localizations_utils.dart';

import '../../../src/common.dart';
import '../../../src/fake_process_manager.dart';

void main() {
  testWithoutContext('generateLocalizations is skipped if l10n.yaml does not exist.', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      artifacts: Artifacts.test(),
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.any(),
    );

    expect(const GenerateLocalizationsTarget().canSkip(environment), true);

    environment.projectDir.childFile('l10n.yaml').createSync();

    expect(const GenerateLocalizationsTarget().canSkip(environment), false);
  });

  testWithoutContext('parseLocalizationsOptions handles valid yaml configuration', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')
      ..writeAsStringSync('''
arb-dir: arb
template-arb-file: example.arb
template-locale: en
output-localization-file: bar
untranslated-messages-file: untranslated
output-class: Foo
header-file: header
header: HEADER
use-deferred-loading: true
preferred-supported-locales: en_US
synthetic-package: false
required-resource-attributes: false
nullable-getter: false
''');

    final LocalizationOptions options = parseLocalizationsOptionsFromYAML(
      file: configFile,
      logger: BufferLogger.test(),
      defaultArbDir: fileSystem.path.join('lib', 'l10n'),
      defaultSyntheticPackage: true,
    );

    expect(options.arbDir, Uri.parse('arb').path);
    expect(options.templateArbFile, Uri.parse('example.arb').path);
    expect(options.templateLocale, 'en');
    expect(options.outputLocalizationFile, Uri.parse('bar').path);
    expect(options.untranslatedMessagesFile, Uri.parse('untranslated').path);
    expect(options.outputClass, 'Foo');
    expect(options.headerFile, Uri.parse('header').path);
    expect(options.header, 'HEADER');
    expect(options.useDeferredLoading, true);
    expect(options.preferredSupportedLocales, <String>['en_US']);
    expect(options.syntheticPackage, false);
    expect(options.requiredResourceAttributes, false);
    expect(options.nullableGetter, false);
  });

  testWithoutContext(
      'parseLocalizationsOptions uses defaultSyntheticPackage = true', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')..writeAsStringSync('''
arb-dir: arb
template-arb-file: example.arb
output-localization-file: bar
untranslated-messages-file: untranslated
output-class: Foo
header-file: header
header: HEADER
use-deferred-loading: true
preferred-supported-locales: en_US
# Intentionally omitted
# synthetic-package: ...
required-resource-attributes: false
nullable-getter: false
''');

    final LocalizationOptions options = parseLocalizationsOptionsFromYAML(
      file: configFile,
      logger: BufferLogger.test(),
      defaultArbDir: fileSystem.path.join('lib', 'l10n'),
      defaultSyntheticPackage: true,
    );

    expect(options.syntheticPackage, true);
  });

  testWithoutContext(
      'parseLocalizationsOptions uses defaultSyntheticPackage = false',
      () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')..writeAsStringSync('''
arb-dir: arb
template-arb-file: example.arb
output-localization-file: bar
untranslated-messages-file: untranslated
output-class: Foo
header-file: header
header: HEADER
use-deferred-loading: true
preferred-supported-locales: en_US
# Intentionally omitted
# synthetic-package: ...
required-resource-attributes: false
nullable-getter: false
''');

    final LocalizationOptions options = parseLocalizationsOptionsFromYAML(
      file: configFile,
      logger: BufferLogger.test(),
      defaultArbDir: fileSystem.path.join('lib', 'l10n'),
      defaultSyntheticPackage: false,
    );

    expect(options.syntheticPackage, false);
  });

  testWithoutContext('parseLocalizationsOptions handles preferredSupportedLocales as list', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')..writeAsStringSync('''
preferred-supported-locales: ['en_US', 'de']
''');

    final LocalizationOptions options = parseLocalizationsOptionsFromYAML(
      file: configFile,
      logger: BufferLogger.test(),
      defaultArbDir: fileSystem.path.join('lib', 'l10n'),
      defaultSyntheticPackage: true,
    );

    expect(options.preferredSupportedLocales, <String>['en_US', 'de']);
  });

  testWithoutContext(
      'parseLocalizationsOptions throws exception on invalid yaml configuration',
      () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')..writeAsStringSync('''
use-deferred-loading: string
''');

    expect(
      () => parseLocalizationsOptionsFromYAML(
        file: configFile,
        logger: BufferLogger.test(),
        defaultArbDir: fileSystem.path.join('lib', 'l10n'),
        defaultSyntheticPackage: true,
      ),
      throwsException,
    );
  });

  testWithoutContext('parseLocalizationsOptions tool exits on malformed Yaml', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File configFile = fileSystem.file('l10n.yaml')..writeAsStringSync('''
template-arb-file: {name}_en.arb
''');

    expect(
      () => parseLocalizationsOptionsFromYAML(
        file: configFile,
        logger: BufferLogger.test(),
        defaultArbDir: fileSystem.path.join('lib', 'l10n'),
        defaultSyntheticPackage: true,
      ),
      throwsToolExit(),
    );
  });
}
