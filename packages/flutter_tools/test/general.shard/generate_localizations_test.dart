// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/localizations/gen_l10n.dart';
import 'package:flutter_tools/src/localizations/gen_l10n_types.dart';
import 'package:flutter_tools/src/localizations/localizations_utils.dart';
import 'package:yaml/yaml.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';

const String defaultTemplateLocale = 'en';
const String defaultTemplateArbFileName = 'app_en.arb';
const String defaultNamespace = 'home';
const String defaultOutputFileString = 'output-localization-file.dart';
const String defaultClassNameString = 'AppLocalizations';
const String singleMessageArbFileString = '''
{
  "title": "Title",
  "@title": {
    "description": "Title for the application."
  }
}''';
const String twoMessageArbFileString = '''
{
  "title": "Title",
  "@title": {
    "description": "Title for the application."
  },
  "subtitle": "Subtitle",
  "@subtitle": {
    "description": "Subtitle for the application."
  }
}''';
const String esArbFileName = 'app_es.arb';
const String singleEsMessageArbFileString = '''
{
  "title": "Título"
}''';
const String singleZhMessageArbFileString = '''
{
  "title": "标题"
}''';
const String intlImportDartCode = '''
import 'package:intl/intl.dart' as intl;
''';
const String foundationImportDartCode = '''
import 'package:flutter/foundation.dart';
''';

void _standardFlutterDirectoryL10nSetup(FileSystem fs) {
  final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
    ..createSync(recursive: true);
  l10nDirectory.childFile(defaultTemplateArbFileName)
    .writeAsStringSync(singleMessageArbFileString);
  l10nDirectory.childFile(esArbFileName)
    .writeAsStringSync(singleEsMessageArbFileString);
  fs.file('pubspec.yaml')
    ..createSync(recursive: true)
    ..writeAsStringSync('''
flutter:
  generate: true
''');
}

void main() {
  late MemoryFileSystem fs;
  late BufferLogger logger;
  late Artifacts artifacts;
  late ProcessManager processManager;
  late String defaultL10nPathString;
  late String syntheticPackagePath;
  late String syntheticL10nPackagePath;

  LocalizationsGenerator setupLocalizations(
    Map<String, String> localeToArbFile,
    {
      String? yamlFile,
      String? outputPathString,
      String? outputFileString,
      String? headerString,
      String? headerFile,
      String? untranslatedMessagesFile,
      String? templateLocale,
      String? templateArbFileName,
      String? namespace,
      bool useSyntheticPackage = true,
      bool isFromYaml = false,
      bool usesNullableGetter = true,
      String? inputsAndOutputsListPath,
      List<String>? preferredSupportedLocales,
      bool useDeferredLoading = false,
      bool useEscaping = false,
      bool areResourceAttributeRequired = false,
      bool suppressWarnings = false,
      bool relaxSyntax = false,
      bool useNamedParameters = false,
      void Function(Directory)? setup,
    }
  ) {
    final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n');
    final Directory arbFilesDirectory = namespace == null
      ? l10nDirectory
      : l10nDirectory.childDirectory(namespace);
    arbFilesDirectory.createSync(recursive: true);

    for (final String locale in localeToArbFile.keys) {
      arbFilesDirectory.childFile('app_$locale.arb')
        .writeAsStringSync(localeToArbFile[locale]!);
    }
    if (setup != null) {
      setup(l10nDirectory);
    }
    return LocalizationsGenerator(
      fileSystem: fs,
      inputPathString: l10nDirectory.path,
      outputPathString: outputPathString ?? l10nDirectory.path,
      templateLocale: templateLocale ?? defaultTemplateLocale,
      templateArbFileName: templateArbFileName ?? defaultTemplateArbFileName,
      outputFileString: outputFileString ?? defaultOutputFileString,
      classNameString: defaultClassNameString,
      headerString: headerString,
      headerFile: headerFile,
      logger: logger,
      untranslatedMessagesFile: untranslatedMessagesFile,
      useSyntheticPackage: useSyntheticPackage,
      inputsAndOutputsListPath: inputsAndOutputsListPath,
      usesNullableGetter: usesNullableGetter,
      preferredSupportedLocales: preferredSupportedLocales,
      useDeferredLoading: useDeferredLoading,
      useEscaping: useEscaping,
      areResourceAttributesRequired: areResourceAttributeRequired,
      suppressWarnings: suppressWarnings,
      useRelaxedSyntax: relaxSyntax,
      useNamedParameters: useNamedParameters,
    )
      ..loadResources()
      ..writeOutputFiles(isFromYaml: isFromYaml);
  }

  String getGeneratedFileContent({String? locale}) {
    final String fileName = locale == null ? 'output-localization-file.dart' : 'output-localization-file_$locale.dart';
    return fs.file(
      fs.path.join(syntheticL10nPackagePath, fileName)
    ).readAsStringSync();
  }

  setUp(() {
    fs = MemoryFileSystem.test();
    logger = BufferLogger.test();
    artifacts = Artifacts.test();
    processManager = FakeProcessManager.empty();

    defaultL10nPathString = fs.path.join('lib', 'l10n');
    syntheticPackagePath = fs.path.join('.dart_tool', 'flutter_gen');
    syntheticL10nPackagePath = fs.path.join(syntheticPackagePath, 'gen_l10n');
    precacheLanguageAndRegionTags();
  });

  group('Setters', () {
    testWithoutContext('setInputDirectory fails if the directory does not exist', () {
      expect(
        () => LocalizationsGenerator.inputDirectoryFromPath(fs, 'lib', fs.directory('bogus')),
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('Make sure that the correct path was provided'),
        )),
      );
    });

    testWithoutContext('setting className fails if input string is empty', () {
      _standardFlutterDirectoryL10nSetup(fs);
      expect(
        () => LocalizationsGenerator.classNameFromString(''),
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('cannot be empty'),
        )),
      );
    });

    testWithoutContext('sets absolute path of the target Flutter project', () {
      // Set up project directory.
      final Directory l10nDirectory = fs.currentDirectory
        .childDirectory('absolute')
        .childDirectory('path')
        .childDirectory('to')
        .childDirectory('flutter_project')
        .childDirectory('lib')
        .childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile(defaultTemplateArbFileName)
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile(esArbFileName)
        .writeAsStringSync(singleEsMessageArbFileString);

      // Run localizations generator in specified absolute path.
      final String flutterProjectPath = fs.path.join('absolute', 'path', 'to', 'flutter_project');
      LocalizationsGenerator(
        fileSystem: fs,
        projectPathString: flutterProjectPath,
        inputPathString: defaultL10nPathString,
        outputPathString: defaultL10nPathString,
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        logger: logger,
      )
        ..loadResources()
        ..writeOutputFiles();

      // Output files should be generated in the provided absolute path.
      expect(
        fs.isFileSync(fs.path.join(
          flutterProjectPath,
          '.dart_tool',
          'flutter_gen',
          'gen_l10n',
          'output-localization-file_en.dart',
        )),
        true,
      );
      expect(
        fs.isFileSync(fs.path.join(
          flutterProjectPath,
          '.dart_tool',
          'flutter_gen',
          'gen_l10n',
          'output-localization-file_es.dart',
        )),
        true,
      );
    });

    testWithoutContext('throws error when directory at absolute path does not exist', () {
      // Set up project directory.
      final Directory l10nDirectory = fs.currentDirectory
        .childDirectory('lib')
        .childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile(defaultTemplateArbFileName)
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile(esArbFileName)
        .writeAsStringSync(singleEsMessageArbFileString);

      // Project path should be intentionally a directory that does not exist.
      expect(
        () => LocalizationsGenerator(
          fileSystem: fs,
          projectPathString: 'absolute/path/to/flutter_project',
          inputPathString: defaultL10nPathString,
          outputPathString: defaultL10nPathString,
          templateArbFileName: defaultTemplateArbFileName,
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          logger: logger,
        ),
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('Directory does not exist'),
        )),
      );
    });

    testWithoutContext('throws error when arb file does not exist', () {
      // Set up project directory.
      fs.currentDirectory
        .childDirectory('lib')
        .childDirectory('l10n')
        .createSync(recursive: true);

      // Arb file should be nonexistent in the l10n directory.
      expect(
        () => LocalizationsGenerator(
          fileSystem: fs,
          projectPathString: './',
          inputPathString: defaultL10nPathString,
          outputPathString: defaultL10nPathString,
          templateArbFileName: defaultTemplateArbFileName,
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          logger: logger,
        ),
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains(', does not exist.'),
        )),
      );
    });

    group('className should only take valid Dart class names', () {
      setUp(() {
        _standardFlutterDirectoryL10nSetup(fs);
      });

      testWithoutContext('fails on string with spaces', () {
        expect(
          () => LocalizationsGenerator.classNameFromString('String with spaces'),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('is not a valid public Dart class name'),
          )),
        );
      });

      testWithoutContext('fails on non-alphanumeric symbols', () {
        expect(
          () => LocalizationsGenerator.classNameFromString('TestClass@123'),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('is not a valid public Dart class name'),
          )),
        );
      });

      testWithoutContext('fails on camel-case', () {
        expect(
          () => LocalizationsGenerator.classNameFromString('camelCaseClassName'),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('is not a valid public Dart class name'),
          )),
        );
      });

      testWithoutContext('fails when starting with a number', () {
        expect(
          () => LocalizationsGenerator.classNameFromString('123ClassName'),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('is not a valid public Dart class name'),
          )),
        );
      });
    });
  });

  testWithoutContext('correctly adds a headerString when it is set', () {
    final LocalizationsGenerator generator = setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
      'es': singleEsMessageArbFileString,
    }, headerString: '/// Sample header');
    expect(generator.header, '/// Sample header');
  });

  testWithoutContext('correctly adds a headerFile when it is set', () {
    final LocalizationsGenerator generator = setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
      'es': singleEsMessageArbFileString,
    }, headerFile: 'header.txt', setup: (Directory l10nDirectory) {
      l10nDirectory.childFile('header.txt').writeAsStringSync('/// Sample header in a text file');
    });
    expect(generator.header, '/// Sample header in a text file');
  });

  testWithoutContext('sets templateArbFileName with more than one underscore correctly', () {
    setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
      'es': singleEsMessageArbFileString,
    });
    final Directory outputDirectory = fs.directory(syntheticL10nPackagePath);
    expect(outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
    expect(outputDirectory.childFile('output-localization-file_en.dart').existsSync(), isTrue);
    expect(outputDirectory.childFile('output-localization-file_es.dart').existsSync(), isTrue);
  });

  testWithoutContext('filenames with invalid locales should not be recognized', () {
    expect(
      () {
        // This attempts to create 'app_localizations_en_CA_foo.arb'.
        setupLocalizations(<String, String>{
          'en': singleMessageArbFileString,
          'en_CA_foo': singleMessageArbFileString,
        });
      },
      throwsA(isA<L10nException>().having(
        (L10nException e) => e.message,
        'message',
        contains("The following .arb file's locale could not be determined"),
      )),
    );
  });

  testWithoutContext('correctly creates an untranslated messages file (useSyntheticPackage = true)', () {
    final String untranslatedMessagesFilePath = fs.path.join('lib', 'l10n', 'unimplemented_message_translations.json');
    setupLocalizations(<String, String>{
      'en': twoMessageArbFileString,
      'es': singleEsMessageArbFileString,
    }, untranslatedMessagesFile: untranslatedMessagesFilePath);
    final String unimplementedOutputString = fs.file(untranslatedMessagesFilePath).readAsStringSync();
    try {
      // Since ARB file is essentially JSON, decoding it should not fail.
      json.decode(unimplementedOutputString);
    } on Exception {
      fail('Parsing arb file should not fail');
    }
    expect(unimplementedOutputString, contains('es'));
    expect(unimplementedOutputString, contains('subtitle'));
  });

  testWithoutContext('correctly creates an untranslated messages file (useSyntheticPackage = false)', () {
    final String untranslatedMessagesFilePath = fs.path.join('lib', 'l10n', 'unimplemented_message_translations.json');
    setupLocalizations(<String, String>{
      'en': twoMessageArbFileString,
      'es': singleMessageArbFileString,
    }, useSyntheticPackage: false, untranslatedMessagesFile: untranslatedMessagesFilePath);
    final String unimplementedOutputString = fs.file(untranslatedMessagesFilePath).readAsStringSync();
    try {
      // Since ARB file is essentially JSON, decoding it should not fail.
      json.decode(unimplementedOutputString);
    } on Exception {
      fail('Parsing arb file should not fail');
    }
    expect(unimplementedOutputString, contains('es'));
    expect(unimplementedOutputString, contains('subtitle'));
  });

  testWithoutContext(
    'untranslated messages suggestion is printed when translation is missing: '
    'command line message',
    () {
      setupLocalizations(<String, String>{
        'en': twoMessageArbFileString,
        'es': singleEsMessageArbFileString,
      });
      expect(
        logger.statusText,
        contains('To see a detailed report, use the --untranslated-messages-file'),
      );
      expect(
        logger.statusText,
        contains('flutter gen-l10n --untranslated-messages-file=desiredFileName.txt'),
      );
    },
  );

  testWithoutContext(
    'untranslated messages suggestion is printed when translation is missing: '
    'l10n.yaml message',
    () {
      setupLocalizations(<String, String>{
        'en': twoMessageArbFileString,
        'es': singleEsMessageArbFileString,
      }, isFromYaml: true);
      expect(
        logger.statusText,
        contains('To see a detailed report, use the untranslated-messages-file'),
      );
      expect(
        logger.statusText,
        contains('untranslated-messages-file: desiredFileName.txt'),
      );
    },
  );

  testWithoutContext(
    'unimplemented messages suggestion is not printed when all messages '
    'are fully translated',
    () {
      setupLocalizations(<String, String>{
        'en': twoMessageArbFileString,
        'es': twoMessageArbFileString,
      });
      expect(logger.statusText, equals(''));
    },
  );

  testWithoutContext('untranslated messages file included in generated JSON list of outputs', () {
    final String untranslatedMessagesFilePath = fs.path.join('lib', 'l10n', 'unimplemented_message_translations.json');
    setupLocalizations(
      <String, String>{
        'en': twoMessageArbFileString,
        'es': singleEsMessageArbFileString,
      },
      untranslatedMessagesFile: untranslatedMessagesFilePath,
      inputsAndOutputsListPath: syntheticL10nPackagePath,
    );
    final File inputsAndOutputsList = fs.file(
      fs.path.join(syntheticL10nPackagePath, 'gen_l10n_inputs_and_outputs.json')
    );
    expect(inputsAndOutputsList.existsSync(), isTrue);
    final Map<String, dynamic> jsonResult = json.decode(
      inputsAndOutputsList.readAsStringSync(),
    ) as Map<String, dynamic>;
    expect(jsonResult.containsKey('outputs'), isTrue);
    final List<dynamic> outputList = jsonResult['outputs'] as List<dynamic>;
    expect(outputList, contains(contains('unimplemented_message_translations.json')));
  });

  testWithoutContext(
    'uses inputPathString as outputPathString when the outputPathString is '
    'null while not using the synthetic package option',
    () {
      _standardFlutterDirectoryL10nSetup(fs);
      LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        // outputPathString is intentionally not defined
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        useSyntheticPackage: false,
        logger: logger,
      )
        ..loadResources()
        ..writeOutputFiles();

      final Directory outputDirectory = fs.directory('lib').childDirectory('l10n');
      expect(outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_en.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_es.dart').existsSync(), isTrue);
    },
  );

  testWithoutContext(
    'correctly generates output files in non-default output directory if it '
    'already exists while not using the synthetic package option',
    () {
      final Directory l10nDirectory = fs.currentDirectory
        .childDirectory('lib')
        .childDirectory('l10n')
        ..createSync(recursive: true);
      // Create the directory 'lib/l10n/output'.
      l10nDirectory.childDirectory('output');

      l10nDirectory
        .childFile(defaultTemplateArbFileName)
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory
        .childFile(esArbFileName)
        .writeAsStringSync(singleEsMessageArbFileString);

      LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        outputPathString: fs.path.join('lib', 'l10n', 'output'),
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        useSyntheticPackage: false,
        logger: logger,
      )
        ..loadResources()
        ..writeOutputFiles();

      final Directory outputDirectory = fs.directory('lib').childDirectory('l10n').childDirectory('output');
      expect(outputDirectory.existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_en.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_es.dart').existsSync(), isTrue);
    },
  );

  testWithoutContext(
    'correctly creates output directory if it does not exist and writes files '
    'in it while not using the synthetic package option',
    () {
      _standardFlutterDirectoryL10nSetup(fs);

      LocalizationsGenerator(
        fileSystem: fs,
          inputPathString: defaultL10nPathString,
          outputPathString: fs.path.join('lib', 'l10n', 'output'),
          templateArbFileName: defaultTemplateArbFileName,
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          useSyntheticPackage: false,
          logger: logger,
        )
        ..loadResources()
        ..writeOutputFiles();

      final Directory outputDirectory = fs.directory('lib').childDirectory('l10n').childDirectory('output');
      expect(outputDirectory.existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_en.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_es.dart').existsSync(), isTrue);
    },
  );

  testWithoutContext(
    'generates nullable localizations class getter via static `of` method '
    'by default',
    () {
      final LocalizationsGenerator generator = setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      });
      expect(generator.outputDirectory.existsSync(), isTrue);
      expect(generator.outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(
        generator.outputDirectory.childFile('output-localization-file.dart').readAsStringSync(),
        contains('static AppLocalizations? of(BuildContext context)'),
      );
      expect(
        generator.outputDirectory.childFile('output-localization-file.dart').readAsStringSync(),
        contains('return Localizations.of<AppLocalizations>(context, AppLocalizations);'),
      );
    },
  );

  testWithoutContext(
    'can generate non-nullable localizations class getter via static `of` method ',
    () {
      final LocalizationsGenerator generator = setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      }, usesNullableGetter: false);
      expect(generator.outputDirectory.existsSync(), isTrue);
      expect(generator.outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(
        generator.outputDirectory.childFile('output-localization-file.dart').readAsStringSync(),
        contains('static AppLocalizations of(BuildContext context)'),
      );
      expect(
        generator.outputDirectory.childFile('output-localization-file.dart').readAsStringSync(),
        contains('return Localizations.of<AppLocalizations>(context, AppLocalizations)!;'),
      );
    },
  );

  testWithoutContext('creates list of inputs and outputs when file path is specified', () {
    setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
      'es': singleEsMessageArbFileString,
    }, inputsAndOutputsListPath: syntheticL10nPackagePath);
    final File inputsAndOutputsList = fs.file(
      fs.path.join(syntheticL10nPackagePath, 'gen_l10n_inputs_and_outputs.json'),
    );
    expect(inputsAndOutputsList.existsSync(), isTrue);

    final Map<String, dynamic> jsonResult = json.decode(inputsAndOutputsList.readAsStringSync()) as Map<String, dynamic>;
    expect(jsonResult.containsKey('inputs'), isTrue);
    final List<dynamic> inputList = jsonResult['inputs'] as List<dynamic>;
    expect(inputList, contains(fs.path.absolute('lib', 'l10n', 'app_en.arb')));
    expect(inputList, contains(fs.path.absolute('lib', 'l10n', 'app_es.arb')));

    expect(jsonResult.containsKey('outputs'), isTrue);
    final List<dynamic> outputList = jsonResult['outputs'] as List<dynamic>;
    expect(outputList, contains(fs.path.absolute(syntheticL10nPackagePath, 'output-localization-file.dart')));
    expect(outputList, contains(fs.path.absolute(syntheticL10nPackagePath, 'output-localization-file_en.dart')));
    expect(outputList, contains(fs.path.absolute(syntheticL10nPackagePath, 'output-localization-file_es.dart')));
  });

  testWithoutContext('setting both a headerString and a headerFile should fail', () {
    expect(
      () {
        setupLocalizations(
          <String, String>{
            'en': singleMessageArbFileString,
            'es': singleEsMessageArbFileString,
          },
          headerString: '/// Sample header in a text file',
          headerFile: 'header.txt',
          setup: (Directory l10nDirectory) {
            l10nDirectory.childFile('header.txt').writeAsStringSync('/// Sample header in a text file');
          },
        );
      },
      throwsA(isA<L10nException>().having(
        (L10nException e) => e.message,
        'message',
        contains('Cannot accept both header and header file arguments'),
      )),
    );
  });

  testWithoutContext('setting a headerFile that does not exist should fail', () {
    expect(
      () {
        setupLocalizations(<String, String>{
          'en': singleMessageArbFileString,
          'es': singleEsMessageArbFileString,
        }, headerFile: 'header.txt');
      },
      throwsA(isA<L10nException>().having(
        (L10nException e) => e.message,
        'message',
        contains('Failed to read header file'),
      )),
    );
  });

  group('generateLocalizations', () {
    testWithoutContext('works even if CWD does not have a pubspec.yaml', () async {
      final Directory projectDir = fs.currentDirectory.childDirectory('project')..createSync(recursive: true);
      final Directory l10nDirectory = projectDir.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile(defaultTemplateArbFileName)
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile(esArbFileName)
        .writeAsStringSync(singleEsMessageArbFileString);
      projectDir.childFile('pubspec.yaml')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
flutter:
  generate: true
''');

      final Logger logger = BufferLogger.test();
      logger.printError('An error output from a different tool in flutter_tools');

      // Should run without error.
      await generateLocalizations(
        fileSystem: fs,
        options: LocalizationOptions(
          arbDir: Uri.directory(defaultL10nPathString).path,
          outputDir: Uri.directory(defaultL10nPathString, windows: false).path,
          templateArbFile: Uri.file(defaultTemplateArbFileName, windows: false).path,
          syntheticPackage: false,
        ),
        logger: logger,
        projectDir: projectDir,
        dependenciesDir: fs.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
      );
    });

    testWithoutContext('other logs from flutter_tools does not affect gen-l10n', () async {
      _standardFlutterDirectoryL10nSetup(fs);

      final Logger logger = BufferLogger.test();
      logger.printError('An error output from a different tool in flutter_tools');

      // Should run without error.
      await generateLocalizations(
        fileSystem: fs,
        options: LocalizationOptions(
          arbDir: Uri.directory(defaultL10nPathString).path,
          outputDir: Uri.directory(defaultL10nPathString, windows: false).path,
          templateArbFile: Uri.file(defaultTemplateArbFileName, windows: false).path,
          syntheticPackage: false,
        ),
        logger: logger,
        projectDir: fs.currentDirectory,
        dependenciesDir: fs.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
      );
    });

    testWithoutContext('forwards arguments correctly', () async {
      _standardFlutterDirectoryL10nSetup(fs);
      final LocalizationOptions options = LocalizationOptions(
        header: 'HEADER',
        arbDir: Uri.directory(defaultL10nPathString).path,
        useDeferredLoading: true,
        outputClass: 'Foo',
        outputLocalizationFile: Uri.file('bar.dart', windows: false).path,
        outputDir: Uri.directory(defaultL10nPathString, windows: false).path,
        preferredSupportedLocales: <String>['es'],
        templateArbFile: Uri.file(defaultTemplateArbFileName, windows: false).path,
        untranslatedMessagesFile: Uri.file('untranslated', windows: false).path,
        syntheticPackage: false,
        requiredResourceAttributes: true,
        nullableGetter: false,
      );

      // Verify that values are correctly passed through the localizations target.
      final LocalizationsGenerator generator = await generateLocalizations(
        fileSystem: fs,
        options: options,
        logger: logger,
        projectDir: fs.currentDirectory,
        dependenciesDir: fs.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
      );

      expect(generator.inputDirectory.path, '/lib/l10n/');
      expect(generator.outputDirectory.path, '/lib/l10n/');
      expect(generator.templateLocale, LocaleInfo.fromString('en'));
      expect(generator.baseOutputFile.path, '/lib/l10n/bar.dart');
      expect(generator.className, 'Foo');
      expect(generator.preferredSupportedLocales.single, LocaleInfo.fromString('es'));
      expect(generator.header, 'HEADER');
      expect(generator.useDeferredLoading, isTrue);
      expect(generator.inputsAndOutputsListFile?.path, '/gen_l10n_inputs_and_outputs.json');
      expect(generator.useSyntheticPackage, isFalse);
      expect(generator.projectDirectory?.path, '/');
      expect(generator.areResourceAttributesRequired, isTrue);
      expect(generator.untranslatedMessagesFile?.path, 'untranslated');
      expect(generator.usesNullableGetter, isFalse);

      // Just validate one file.
      expect(fs.file('/lib/l10n/bar_en.dart').readAsStringSync(), '''
HEADER

// ignore: unused_import
import 'package:intl/intl.dart' as intl;
import 'bar.dart';

// ignore_for_file: type=lint

/// The translations for English (`en`).
class FooEn extends Foo {
  FooEn([String locale = 'en']) : super(locale);

  @override
  String get title => 'Title';
}
''');
    });

    testWithoutContext('throws exception on missing flutter: generate: true flag', () async {
      _standardFlutterDirectoryL10nSetup(fs);

      // Missing flutter: generate: true should throw exception.
      fs.file('pubspec.yaml')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
flutter:
  uses-material-design: true
''');

      final LocalizationOptions options = LocalizationOptions(
        header: 'HEADER',
        headerFile: Uri.file('header', windows: false).path,
        arbDir: Uri.file('arb', windows: false).path,
        useDeferredLoading: true,
        outputClass: 'Foo',
        outputLocalizationFile: Uri.file('bar', windows: false).path,
        preferredSupportedLocales: <String>['en_US'],
        templateArbFile: Uri.file('example.arb', windows: false).path,
        untranslatedMessagesFile: Uri.file('untranslated', windows: false).path,
      );

      expect(
        () => generateLocalizations(
          fileSystem: fs,
          options: options,
          logger: BufferLogger.test(),
          projectDir: fs.currentDirectory,
          dependenciesDir: fs.currentDirectory,
          artifacts: artifacts,
          processManager: processManager,
        ),
        throwsToolExit(
          message: 'Attempted to generate localizations code without having the '
              'flutter: generate flag turned on.',
        ),
      );
    });

    testWithoutContext('uses the same line terminator as pubspec.yaml', () async {
      _standardFlutterDirectoryL10nSetup(fs);

      fs.file('pubspec.yaml')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
flutter:\r
  generate: true\r
''');

      final LocalizationOptions options = LocalizationOptions(
        arbDir: fs.path.join('lib', 'l10n'),
        outputClass: defaultClassNameString,
        outputLocalizationFile: defaultOutputFileString,
      );
      await generateLocalizations(
        fileSystem: fs,
        options: options,
        logger: BufferLogger.test(),
        projectDir: fs.currentDirectory,
        dependenciesDir: fs.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
      );
      final String content = getGeneratedFileContent(locale: 'en');
      expect(content, contains('\r\n'));
    });

    testWithoutContext('blank lines generated nicely', () async {
      _standardFlutterDirectoryL10nSetup(fs);

      // Test without headers.
      await generateLocalizations(
        fileSystem: fs,
        options: LocalizationOptions(
          arbDir: Uri.directory(defaultL10nPathString).path,
          outputDir: Uri.directory(defaultL10nPathString, windows: false).path,
          templateArbFile: Uri.file(defaultTemplateArbFileName, windows: false).path,
          syntheticPackage: false,
        ),
        logger: BufferLogger.test(),
        projectDir: fs.currentDirectory,
        dependenciesDir: fs.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
      );

      expect(fs.file('/lib/l10n/app_localizations_en.dart').readAsStringSync(), '''
// ignore: unused_import
import 'package:intl/intl.dart' as intl;
import 'app_localizations.dart';

// ignore_for_file: type=lint

/// The translations for English (`en`).
class AppLocalizationsEn extends AppLocalizations {
  AppLocalizationsEn([String locale = 'en']) : super(locale);

  @override
  String get title => 'Title';
}
''');

    // Test with headers.
    await generateLocalizations(
      fileSystem: fs,
      options: LocalizationOptions(
        header: 'HEADER',
        arbDir: Uri.directory(defaultL10nPathString).path,
        outputDir: Uri.directory(defaultL10nPathString, windows: false).path,
        templateArbFile: Uri.file(defaultTemplateArbFileName, windows: false).path,
        syntheticPackage: false,
      ),
      logger: logger,
      projectDir: fs.currentDirectory,
      dependenciesDir: fs.currentDirectory,
      artifacts: artifacts,
      processManager: processManager,
    );

    expect(fs.file('/lib/l10n/app_localizations_en.dart').readAsStringSync(), '''
HEADER

// ignore: unused_import
import 'package:intl/intl.dart' as intl;
import 'app_localizations.dart';

// ignore_for_file: type=lint

/// The translations for English (`en`).
class AppLocalizationsEn extends AppLocalizations {
  AppLocalizationsEn([String locale = 'en']) : super(locale);

  @override
  String get title => 'Title';
}
''');
    });
  });

  group('loadResources', () {
    testWithoutContext('correctly initializes supportedLocales and supportedLanguageCodes properties', () {
      _standardFlutterDirectoryL10nSetup(fs);

      final LocalizationsGenerator generator = LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        outputPathString: defaultL10nPathString,
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        logger: logger,
      )
        ..loadResources();

      expect(generator.supportedLocales.contains(LocaleInfo.fromString('en')), true);
      expect(generator.supportedLocales.contains(LocaleInfo.fromString('es')), true);
    });

    testWithoutContext('correctly sorts supportedLocales and supportedLanguageCodes alphabetically', () {
      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      // Write files in non-alphabetical order so that read performs in that order
      l10nDirectory.childFile('app_zh.arb')
        .writeAsStringSync(singleZhMessageArbFileString);
      l10nDirectory.childFile('app_es.arb')
        .writeAsStringSync(singleEsMessageArbFileString);
      l10nDirectory.childFile('app_en.arb')
        .writeAsStringSync(singleMessageArbFileString);

      final LocalizationsGenerator generator = LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        outputPathString: defaultL10nPathString,
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        logger: logger,
      )
        ..loadResources();

      expect(generator.supportedLocales.first, LocaleInfo.fromString('en'));
      expect(generator.supportedLocales.elementAt(1), LocaleInfo.fromString('es'));
      expect(generator.supportedLocales.elementAt(2), LocaleInfo.fromString('zh'));
    });

    testWithoutContext('adds preferred locales to the top of supportedLocales and supportedLanguageCodes', () {
      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_en.arb')
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile('app_es.arb')
        .writeAsStringSync(singleEsMessageArbFileString);
      l10nDirectory.childFile('app_zh.arb')
        .writeAsStringSync(singleZhMessageArbFileString);

      const List<String> preferredSupportedLocale = <String>['zh', 'es'];
      final LocalizationsGenerator generator = LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        outputPathString: defaultL10nPathString,
        templateArbFileName: defaultTemplateArbFileName,
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        preferredSupportedLocales: preferredSupportedLocale,
        logger: logger,
      )
        ..loadResources();

      expect(generator.supportedLocales.first, LocaleInfo.fromString('zh'));
      expect(generator.supportedLocales.elementAt(1), LocaleInfo.fromString('es'));
      expect(generator.supportedLocales.elementAt(2), LocaleInfo.fromString('en'));
    });

    testWithoutContext(
      'throws an error attempting to add preferred locales when there is no corresponding arb file for that locale',
      () {
        final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
          ..createSync(recursive: true);
        l10nDirectory.childFile('app_en.arb')
          .writeAsStringSync(singleMessageArbFileString);
        l10nDirectory.childFile('app_es.arb')
          .writeAsStringSync(singleEsMessageArbFileString);
        l10nDirectory.childFile('app_zh.arb')
          .writeAsStringSync(singleZhMessageArbFileString);

        const List<String> preferredSupportedLocale = <String>['am', 'es'];
        expect(
          () {
            LocalizationsGenerator(
              fileSystem: fs,
              inputPathString: defaultL10nPathString,
              outputPathString: defaultL10nPathString,
              templateArbFileName: defaultTemplateArbFileName,
              outputFileString: defaultOutputFileString,
              classNameString: defaultClassNameString,
              preferredSupportedLocales: preferredSupportedLocale,
              logger: logger,
            ).loadResources();
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains("The preferred supported locale, 'am', cannot be added."),
          )),
        );
      },
    );

    testWithoutContext('correctly parses @@locale property in arb file', () {
      const String arbFileWithEnLocale = '''
{
  "@@locale": "en",
  "title": "Title",
  "@title": {
    "description": "Title for the application"
  }
}''';

      const String arbFileWithZhLocale = '''
{
  "@@locale": "zh",
  "title": "标题",
  "@title": {
    "description": "Title for the application"
  }
}''';

      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('first_file.arb')
        .writeAsStringSync(arbFileWithEnLocale);
      l10nDirectory.childFile('second_file.arb')
        .writeAsStringSync(arbFileWithZhLocale);

      final LocalizationsGenerator generator = LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: defaultL10nPathString,
        outputPathString: defaultL10nPathString,
        templateArbFileName: 'first_file.arb',
        outputFileString: defaultOutputFileString,
        classNameString: defaultClassNameString,
        logger: logger,
      )
        ..loadResources();

      expect(generator.supportedLocales.contains(LocaleInfo.fromString('en')), true);
      expect(generator.supportedLocales.contains(LocaleInfo.fromString('zh')), true);
    });

    testWithoutContext('correctly requires @@locale property in arb file to match the filename locale suffix', () {
      const String arbFileWithEnLocale = '''
{
  "@@locale": "en",
  "title": "Stocks",
  "@title": {
    "description": "Title for the Stocks application"
  }
}''';

      const String arbFileWithZhLocale = '''
{
  "@@locale": "zh",
  "title": "标题",
  "@title": {
    "description": "Title for the Stocks application"
  }
}''';

      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_es.arb')
        .writeAsStringSync(arbFileWithEnLocale);
      l10nDirectory.childFile('app_am.arb')
        .writeAsStringSync(arbFileWithZhLocale);

      expect(
        () {
          LocalizationsGenerator(
            fileSystem: fs,
            inputPathString: defaultL10nPathString,
            outputPathString: defaultL10nPathString,
            templateArbFileName: 'app_es.arb',
            outputFileString: defaultOutputFileString,
            classNameString: defaultClassNameString,
            logger: logger,
          ).loadResources();
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('The locale specified in @@locale and the arb filename do not match.'),
        )),
      );
    });

    testWithoutContext("throws when arb file's locale could not be determined", () {
      fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true)
        ..childFile('app.arb')
        .writeAsStringSync(singleMessageArbFileString);
      expect(
        () {
          LocalizationsGenerator(
            fileSystem: fs,
            inputPathString: defaultL10nPathString,
            outputPathString: defaultL10nPathString,
            templateArbFileName: 'app.arb',
            outputFileString: defaultOutputFileString,
            classNameString: defaultClassNameString,
            logger: logger,
          ).loadResources();
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('locale could not be determined'),
        )),
      );
    });

    testWithoutContext('throws when an empty string is used as a key', () {
      const String arbFileStringWithEmptyResourceId = '''
{
  "market": "MARKET",
  "": {
    "description": "This key is invalid"
  }
}''';

      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_en.arb')
        .writeAsStringSync(arbFileStringWithEmptyResourceId);

      expect(
        () => LocalizationsGenerator(
          fileSystem: fs,
          inputPathString: defaultL10nPathString,
          outputPathString: defaultL10nPathString,
          templateArbFileName: 'app_en.arb',
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          logger: logger,
        ).loadResources(),
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('Invalid ARB resource name ""'),
        )),
      );
    });

    testWithoutContext('throws when the same locale is detected more than once', () {
      const String secondMessageArbFileString = '''
{
  "market": "MARKET",
  "@market": {
    "description": "Label for the Market tab"
  }
}''';

      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_en.arb')
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile('app2_en.arb')
        .writeAsStringSync(secondMessageArbFileString);

      expect(
        () {
          LocalizationsGenerator(
            fileSystem: fs,
            inputPathString: defaultL10nPathString,
            outputPathString: defaultL10nPathString,
            templateArbFileName: 'app_en.arb',
            outputFileString: defaultOutputFileString,
            classNameString: defaultClassNameString,
            logger: logger,
          ).loadResources();
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains("Multiple arb files with the same 'en' locale detected"),
        )),
      );
    });

    testWithoutContext('throws when the base locale does not exist', () {
      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_en_US.arb')
        .writeAsStringSync(singleMessageArbFileString);

      expect(
        () {
          LocalizationsGenerator(
            fileSystem: fs,
            inputPathString: defaultL10nPathString,
            outputPathString: defaultL10nPathString,
            templateArbFileName: 'app_en_US.arb',
            outputFileString: defaultOutputFileString,
            classNameString: defaultClassNameString,
            logger: logger,
          ).loadResources();
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('Arb file for a fallback, en, does not exist'),
        )),
      );
    });

    testWithoutContext('AppResourceBundle throws if file contains non-string value', () {
      const String inputPathString = 'lib/l10n';
      const String templateArbFileName = 'app_en.arb';
      const String outputFileString = 'app_localizations.dart';
      const String classNameString = 'AppLocalizations';

      fs.file(fs.path.join(inputPathString, templateArbFileName))
        ..createSync(recursive: true)
        ..writeAsStringSync('{ "helloWorld": "Hello World!" }');
      fs.file(fs.path.join(inputPathString, 'app_es.arb'))
        ..createSync(recursive: true)
        ..writeAsStringSync('{ "helloWorld": {} }');

      final LocalizationsGenerator generator = LocalizationsGenerator(
        fileSystem: fs,
        inputPathString: inputPathString,
        templateArbFileName: templateArbFileName,
        outputFileString: outputFileString,
        classNameString: classNameString,
        logger: logger,
      );
      expect(
          () => generator.loadResources(),
          throwsToolExit(message: 'Localized message for key "helloWorld" in '
            '"lib/l10n/app_es.arb" is not a string.'),
      );
    });
  });

  group('writeOutputFiles', () {
    testWithoutContext('multiple messages with syntax error all log their errors', () {
      try {
        setupLocalizations(<String, String>{
          'en': r'''
{
  "msg1": "{",
  "msg2": "{ {"
}'''});
      } on L10nException catch (error) {
        expect(error.message, equals('Found syntax errors.'));
        expect(logger.errorText, contains('''
[app_en.arb:msg1] ICU Syntax Error: Expected "identifier" but found no tokens.
    {
      ^
[app_en.arb:msg2] ICU Syntax Error: Expected "identifier" but found "{".
    { {
      ^'''));
      }
    });

    testWithoutContext('no description generates generic comment', () {
      setupLocalizations(<String, String>{
        'en': r'''
{
  "helloWorld": "Hello world!"
}'''
      });
      expect(getGeneratedFileContent(), contains('/// No description provided for @helloWorld.'));
    });

    testWithoutContext('multiline descriptions are correctly formatted as comments', () {
      setupLocalizations(<String, String>{
        'en': r'''
{
  "helloWorld": "Hello world!",
  "@helloWorld": {
    "description": "The generic example string in every language.\nUse this for tests!"
  }
}'''});
      expect(getGeneratedFileContent(), contains('''
  /// The generic example string in every language.
  /// Use this for tests!'''));
    });

    testWithoutContext('message without placeholders - should generate code comment with description and template message translation', () {
      setupLocalizations(<String, String> {
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      });
      final String content = getGeneratedFileContent();
      expect(content, contains('/// Title for the application.'));
      expect(content, contains('''
  /// In en, this message translates to:
  /// **'Title'**'''));
    });

    testWithoutContext('template message translation handles newline characters', () {
      setupLocalizations(<String, String>{
        'en': r'''
{
  "title": "Title \n of the application",
  "@title": {
    "description": "Title for the application."
  }
}''',
        'es': singleEsMessageArbFileString
      });
      final String content = getGeneratedFileContent();
      expect(content, contains('/// Title for the application.'));
      expect(content, contains(r'''
  /// In en, this message translates to:
  /// **'Title \n of the application'**'''));
    });

    testWithoutContext('message with placeholders - should generate code comment with description and template message translation', () {
      setupLocalizations(<String, String>{
        'en': r'''
{
  "price": "The price of this item is: ${price}",
  "@price": {
    "description": "The price of an online shopping cart item.",
    "placeholders": {
      "price": {
        "type": "double",
        "format": "decimalPattern"
      }
    }
  }
}''',
        'es': r'''
{
  "price": "El precio de este artículo es: ${price}"
}'''
      });
      final String content = getGeneratedFileContent();
      expect(content, contains('/// The price of an online shopping cart item.'));
      expect(content, contains(r'''
  /// In en, this message translates to:
  /// **'The price of this item is: \${price}'**'''));
    });

    testWithoutContext('should generate a file per language', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'en_CA': '''
{
  "title": "Canadian Title"
}'''
      });
      expect(getGeneratedFileContent(locale: 'en'), contains('class AppLocalizationsEn extends AppLocalizations'));
      expect(getGeneratedFileContent(locale: 'en'), contains('class AppLocalizationsEnCa extends AppLocalizationsEn'));
      expect(() => getGeneratedFileContent(locale: 'en_US'), throwsException);
    });

    testWithoutContext('language imports are sorted when preferredSupportedLocaleString is given', () {
      const List<String> preferredSupportedLocales = <String>['zh'];
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'zh': singleZhMessageArbFileString,
        'es': singleEsMessageArbFileString,
      }, preferredSupportedLocales: preferredSupportedLocales);
      final String content = getGeneratedFileContent();
      expect(content, contains(
'''
import 'output-localization-file_en.dart';
import 'output-localization-file_es.dart';
import 'output-localization-file_zh.dart';
'''));
    });

    // Regression test for https://github.com/flutter/flutter/issues/88356
    testWithoutContext('full output file suffix is retained', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
      }, outputFileString: 'output-localization-file.g.dart');
      final String baseLocalizationsFile = fs.file(
        fs.path.join(syntheticL10nPackagePath, 'output-localization-file.g.dart'),
      ).readAsStringSync();
      expect(baseLocalizationsFile, contains(
'''
import 'output-localization-file_en.g.dart';
'''));

      final String englishLocalizationsFile = fs.file(
        fs.path.join(syntheticL10nPackagePath, 'output-localization-file_en.g.dart'),
      ).readAsStringSync();
      expect(englishLocalizationsFile, contains(
'''
import 'output-localization-file.g.dart';
'''));
    });

    testWithoutContext('throws an exception when invalid output file name is passed in', () {
      expect(
        () {
          setupLocalizations(<String, String>{
            'en': singleMessageArbFileString,
          }, outputFileString: 'asdf');
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          allOf(
            contains('output-localization-file'),
            contains('asdf'),
            contains('is invalid'),
            contains('The file name must have a .dart extension.'),
          ),
        )),
      );
      expect(
        () {
          setupLocalizations(<String, String>{
            'en': singleMessageArbFileString,
          }, outputFileString: '.g.dart');
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          allOf(
            contains('output-localization-file'),
            contains('.g.dart'),
            contains('is invalid'),
            contains('The base name cannot be empty.'),
          ),
        )),
      );
    });

    testWithoutContext('imports are deferred and loaded when useDeferredImports are set', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
      }, useDeferredLoading: true);
      final String content = getGeneratedFileContent();
      expect(content, contains(
'''
import 'output-localization-file_en.dart' deferred as output-localization-file_en;
'''));
      expect(content, contains('output-localization-file_en.loadLibrary()'));
    });

    group('placeholder tests', () {
      testWithoutContext('should automatically infer placeholders that are not explicitly defined', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "helloWorld": "Hello {name}"
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains('String helloWorld(Object name) {'));
      });

      testWithoutContext('placeholder parameter list should be consistent between languages', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "helloWorld": "Hello {name}",
  "@helloWorld": {
    "placeholders": {
      "name": {}
    }
  }
}''',
          'es': '''
{
  "helloWorld": "Hola"
}
''',
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('String helloWorld(Object name) {'));
        expect(getGeneratedFileContent(locale: 'es'), contains('String helloWorld(Object name) {'));
      });

      testWithoutContext('braces are ignored as special characters if placeholder does not exist', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "helloWorld": "Hello {name}",
  "@@helloWorld": {
    "placeholders": {
      "names": {}
    }
  }
}'''
        }, relaxSyntax: true);
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains("String get helloWorld => 'Hello {name}'"));
      });
    });

    group('DateTime tests', () {
      testWithoutContext('imports package:intl', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "yMd"
      }
    }
  }
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains(intlImportDartCode));
      });

      testWithoutContext('throws an exception when improperly formatted date is passed in', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "asdf"
      }
    }
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('asdf'),
              contains('springStartDate'),
              contains('does not have a corresponding DateFormat'),
            ),
          )),
        );
      });

      testWithoutContext('use standard date format whenever possible', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "yMd",
        "isCustomDateFormat": "true"
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains('DateFormat.yMd(localeName)'));
      });

      testWithoutContext('handle arbitrary formatted date', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "asdf o'clock",
        "isCustomDateFormat": "true"
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains(r"DateFormat('asdf o\'clock', localeName)"));
      });

      testWithoutContext('handle arbitrary formatted date with actual boolean', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "asdf o'clock",
        "isCustomDateFormat": true
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains(r"DateFormat('asdf o\'clock', localeName)"));
      });

      testWithoutContext('handles adding two valid formats', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "loggedIn": "Last logged in on {lastLoginDate}",
  "@loggedIn": {
    "placeholders": {
      "lastLoginDate": {
        "type": "DateTime",
        "format": "yMd+jms"
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains(r'DateFormat.yMd(localeName).add_jms()'));
      });

      testWithoutContext('handles adding three valid formats', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "loggedIn": "Last logged in on {lastLoginDate}",
  "@loggedIn": {
    "placeholders": {
      "lastLoginDate": {
        "type": "DateTime",
        "format": "yMMMMEEEEd+QQQQ+Hm"
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains(r'DateFormat.yMMMMEEEEd(localeName).add_QQQQ().add_Hm()'));
      });

      testWithoutContext('throws an exception when adding invalid formats', (){
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "loggedIn": "Last logged in on {lastLoginDate}",
  "@loggedIn": {
    "placeholders": {
      "lastLoginDate": {
        "type": "DateTime",
        "format": "foo+bar+baz"
      }
    }
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('"foo+bar+baz"'),
              contains('lastLoginDate'),
              contains('contains at least one invalid date format.'),
            ),
          )),
        );
      });

      testWithoutContext('throws an exception when adding formats and trailing plus sign', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "loggedIn": "Last logged in on {lastLoginDate}",
  "@loggedIn": {
    "placeholders": {
      "lastLoginDate": {
        "type": "DateTime",
        "format": "yMd+Hm+"
      }
    }
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('"yMd+Hm+"'),
              contains('lastLoginDate'),
              contains('contains at least one invalid date format.'),
            ),
          )),
        );
      });

      testWithoutContext('throws an exception when no format attribute is passed in', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime"
      }
    }
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('the "format" attribute needs to be set'),
          )),
        );
      });

      testWithoutContext('handle date with multiple locale', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "MMMd"
      }
    }
  }
}''',
          'ja': '''
{
  "@@locale": "ja",
  "springBegins": "春が始まるのは{springStartDate}",
  "@springBegins": {
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "MMMMd"
      }
    }
  }
}'''
        });

        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('intl.DateFormat.MMMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'en'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('String springBegins(DateTime springStartDate)'));
      });

      testWithoutContext('handle date with multiple locale when only template has placeholders', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "MMMd"
      }
    }
  }
}''',
          'ja': '''
{
  "@@locale": "ja",
  "springBegins": "春が始まるのは{springStartDate}"
}'''
        });

        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'en'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('String springBegins(DateTime springStartDate)'));
      });

      testWithoutContext('handle date with multiple locale when there is unused placeholder', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "MMMd"
      }
    }
  }
}''',
          'ja': '''
{
  "@@locale": "ja",
  "springBegins": "春が始まるのは{springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "notUsed": {
        "type": "DateTime",
        "format": "MMMMd"
      }
    }
  }
}'''
        });

        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'en'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), isNot(contains('notUsed')));
      });

      testWithoutContext('handle date with multiple locale when placeholders are incompatible', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
    {
      "@@locale": "en",
      "springBegins": "Spring begins on {springStartDate}",
      "@springBegins": {
        "description": "The first day of spring",
        "placeholders": {
          "springStartDate": {
            "type": "DateTime",
            "format": "MMMd"
          }
        }
      }
    }''',
              'ja': '''
    {
      "@@locale": "ja",
      "springBegins": "春が始まるのは{springStartDate}",
      "@springBegins": {
        "description": "The first day of spring",
        "placeholders": {
          "springStartDate": {
            "type": "String"
          }
        }
      }
    }'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('The placeholder, springStartDate, has its "type" resource attribute set to the "String" type in locale "ja", but it is "DateTime" in the template placeholder.'),
          )),
        );
      });

      testWithoutContext('handle date with multiple locale when non-template placeholder does not specify type', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
    {
      "@@locale": "en",
      "springBegins": "Spring begins on {springStartDate}",
      "@springBegins": {
        "description": "The first day of spring",
        "placeholders": {
          "springStartDate": {
            "type": "DateTime",
            "format": "MMMd"
          }
        }
      }
    }''',
              'ja': '''
    {
      "@@locale": "ja",
      "springBegins": "春が始まるのは{springStartDate}",
      "@springBegins": {
        "description": "The first day of spring",
        "placeholders": {
          "springStartDate": {
            "format": "MMMMd"
          }
        }
      }
    }'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('The placeholder, springStartDate, has its "type" resource attribute set to the "null" type in locale "ja", but it is "DateTime" in the template placeholder.'),
          )),
        );
      });

      testWithoutContext('handle ordinary formatted date and arbitrary formatted date', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "MMMd"
      }
    }
  }
}''',
          'ja': '''
{
  "@@locale": "ja",
  "springBegins": "春が始まるのは{springStartDate}",
  "@springBegins": {
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "立春",
        "isCustomDateFormat": "true"
      }
    }
  }
}'''
        });

        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.MMMd(localeName)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains(r"DateFormat('立春', localeName)"));
        expect(getGeneratedFileContent(locale: 'en'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('String springBegins(DateTime springStartDate)'));
      });

      testWithoutContext('handle arbitrary formatted date with multiple locale', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "@@locale": "en",
  "springBegins": "Spring begins on {springStartDate}",
  "@springBegins": {
    "description": "The first day of spring",
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "asdf o'clock",
        "isCustomDateFormat": "true"
      }
    }
  }
}''',
          'ja': '''
{
  "@@locale": "ja",
  "springBegins": "春が始まるのは{springStartDate}",
  "@springBegins": {
    "placeholders": {
      "springStartDate": {
        "type": "DateTime",
        "format": "立春",
        "isCustomDateFormat": "true"
      }
    }
  }
}'''
        });

        expect(getGeneratedFileContent(locale: 'en'), contains(r"DateFormat('asdf o\'clock', localeName)"));
        expect(getGeneratedFileContent(locale: 'ja'), contains(r"DateFormat('立春', localeName)"));
        expect(getGeneratedFileContent(locale: 'en'), contains('String springBegins(DateTime springStartDate)'));
        expect(getGeneratedFileContent(locale: 'ja'), contains('String springBegins(DateTime springStartDate)'));
      });
    });

    group('NumberFormat tests', () {
      testWithoutContext('imports package:intl', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "courseCompletion": "You have completed {progress} of the course.",
  "@courseCompletion": {
    "description": "The amount of progress the student has made in their class.",
    "placeholders": {
      "progress": {
        "type": "double",
        "format": "percentPattern"
      }
    }
  }
}'''
        });
        final String content = getGeneratedFileContent(locale: 'en');
        expect(content, contains(intlImportDartCode));
      });

      testWithoutContext('throws an exception when improperly formatted number is passed in', () {
       expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "courseCompletion": "You have completed {progress} of the course.",
  "@courseCompletion": {
    "description": "The amount of progress the student has made in their class.",
    "placeholders": {
      "progress": {
        "type": "double",
        "format": "asdf"
      }
    }
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('asdf'),
              contains('progress'),
              contains('does not have a corresponding NumberFormat'),
            ),
          )),
        );
      });
    });

    group('plural messages', () {
      testWithoutContext('intl package import should be omitted in subclass files when no plurals are included', () {
        setupLocalizations(<String, String>{
          'en': singleMessageArbFileString,
          'es': singleEsMessageArbFileString,
        });
        expect(getGeneratedFileContent(locale: 'es'), contains(intlImportDartCode));
      });

      testWithoutContext('warnings are generated when plural parts are repeated', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "helloWorlds": "{count,plural, =0{Hello}zero{hello} other{hi}}",
  "@helloWorlds": {
    "description": "Properly formatted but has redundant zero cases."
  }
}'''
        });
        expect(logger.hadWarningOutput, isTrue);
        expect(logger.warningText, contains('''
[app_en.arb:helloWorlds] ICU Syntax Warning: The plural part specified below is overridden by a later plural part.
    {count,plural, =0{Hello}zero{hello} other{hi}}
                   ^'''));
      });

      testWithoutContext('undefined plural cases throws syntax error', () {
        try {
          setupLocalizations(<String, String>{
            'en': '''
{
  "count": "{count,plural, =0{None} =1{One} =2{Two} =3{Undefined Behavior!} other{Hmm...}}"
}'''
          });
        } on L10nException catch (error) {
          expect(error.message, contains('Found syntax errors.'));
          expect(logger.hadErrorOutput, isTrue);
          expect(logger.errorText, contains('''
[app_en.arb:count] The plural cases must be one of "=0", "=1", "=2", "zero", "one", "two", "few", "many", or "other.
    3 is not a valid plural case.
    {count,plural, =0{None} =1{One} =2{Two} =3{Undefined Behavior!} other{Hmm...}}
                                            ^'''));
        }
      });

      testWithoutContext('should automatically infer plural placeholders that are not explicitly defined', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "helloWorlds": "{count,plural, =0{Hello}=1{Hello World}=2{Hello two worlds}few{Hello {count} worlds}many{Hello all {count} worlds}other{Hello other {count} worlds}}",
  "@helloWorlds": {
    "description": "Improperly formatted since it has no placeholder attribute."
  }
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('String helloWorlds(num count) {'));
      });

      testWithoutContext('should throw attempting to generate a plural message with incorrect format for placeholders', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "helloWorlds": "{count,plural, =0{Hello}=1{Hello World}=2{Hello two worlds}few{Hello {count} worlds}many{Hello all {count} worlds}other{Hello other {count} worlds}}",
  "@helloWorlds": {
    "placeholders": "Incorrectly a string, should be a map."
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('is not properly formatted'),
              contains('Ensure that it is a map with string valued keys'),
            ),
          )),
        );
      });
    });

    group('select messages', () {
      testWithoutContext('should automatically infer select placeholders that are not explicitly defined', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "genderSelect": "{gender, select, female {She} male {He} other {they} }",
  "@genderSelect": {
    "description": "Improperly formatted since it has no placeholder attribute."
  }
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('String genderSelect(String gender) {'));
      });

      testWithoutContext('should throw attempting to generate a select message with incorrect format for placeholders', () {
        expect(
          () {
            setupLocalizations(<String, String>{
              'en': '''
{
  "genderSelect": "{gender, select, female {She} male {He} other {they} }",
  "@genderSelect": {
    "placeholders": "Incorrectly a string, should be a map."
  }
}'''
            });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('is not properly formatted'),
              contains('Ensure that it is a map with string valued keys'),
            ),
          )),
        );
      });

      testWithoutContext('should throw attempting to generate a select message with an incorrect message', () {
        try {
          setupLocalizations(<String, String>{
            'en': '''
{
  "genderSelect": "{gender, select,}",
  "@genderSelect": {
    "placeholders": {
      "gender": {}
    }
  }
}'''
          });
        } on L10nException {
          expect(logger.errorText, contains('''
[app_en.arb:genderSelect] ICU Syntax Error: Select expressions must have an "other" case.
    {gender, select,}
                    ^''')
          );
        }
      });
    });

    group('argument messages', () {
      testWithoutContext('should generate proper calls to intl.DateFormat', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "datetime": "{today, date, ::yMd}"
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.yMd(localeName).format(today)'));
      });

      testWithoutContext('should generate proper calls to intl.DateFormat when using time', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "datetime": "{current, time, ::jms}"
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('intl.DateFormat.jms(localeName).format(current)'));
      });

      testWithoutContext('should not complain when placeholders are explicitly typed to DateTime', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "datetime": "{today, date, ::yMd}",
  "@datetime": {
    "placeholders": {
      "today": { "type": "DateTime" }
    }
  }
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('String datetime(DateTime today) {'));
      });

      testWithoutContext('should automatically infer date time placeholders that are not explicitly defined', () {
        setupLocalizations(<String, String>{
          'en': '''
{
  "datetime": "{today, date, ::yMd}"
}'''
        });
        expect(getGeneratedFileContent(locale: 'en'), contains('String datetime(DateTime today) {'));
      });

      testWithoutContext('should throw on invalid DateFormat', () {
        try {
          setupLocalizations(<String, String>{
            'en': '''
{
  "datetime": "{today, date, ::yMMMMMd}"
}'''
          });
          assert(false);
        } on L10nException {
          expect(logger.errorText, contains('Date format "yMMMMMd" for placeholder today does not have a corresponding DateFormat constructor'));
        }
      });
    });

    // All error handling for messages should collect errors on a per-error
    // basis and log them out individually. Then, it will throw an L10nException.
    group('error handling tests', () {
      testWithoutContext('syntax/code-gen errors properly logs errors per message', () {
        // TODO(thkim1011): Fix error handling so that long indents don't get truncated.
        // See https://github.com/flutter/flutter/issues/120490.
        try {
          setupLocalizations(<String, String>{
            'en': '''
{
  "hello": "Hello { name",
  "plural": "This is an incorrectly formatted plural: { count, plural, zero{No frog} one{One frog} other{{count} frogs}",
  "explanationWithLexingError": "The 'string above is incorrect as it forgets to close the brace",
  "pluralWithInvalidCase": "{ count, plural, woohoo{huh?} other{lol} }"
}'''
          }, useEscaping: true);
        } on L10nException {
          expect(logger.errorText, contains('''
[app_en.arb:hello] ICU Syntax Error: Expected "}" but found no tokens.
    Hello { name
                 ^
[app_en.arb:plural] ICU Syntax Error: Expected "}" but found no tokens.
    This is an incorrectly formatted plural: { count, plural, zero{No frog} one{One frog} other{{count} frogs}
                                                                                          ^
[app_en.arb:explanationWithLexingError] ICU Lexing Error: Unmatched single quotes.
    The 'string above is incorrect as it forgets to close the brace
        ^
[app_en.arb:pluralWithInvalidCase] ICU Syntax Error: Plural expressions case must be one of "zero", "one", "two", "few", "many", or "other".
    { count, plural, woohoo{huh?} other{lol} }
                     ^'''));
        }
      });

      testWithoutContext('errors thrown in multiple languages are all shown', () {
        try {
          setupLocalizations(<String, String>{
            'en': '{ "hello": "Hello { name" }',
            'es': '{ "hello": "Hola { name" }',
          });
        } on L10nException {
          expect(logger.errorText, contains('''
[app_en.arb:hello] ICU Syntax Error: Expected "}" but found no tokens.
    Hello { name
                 ^
[app_es.arb:hello] ICU Syntax Error: Expected "}" but found no tokens.
    Hola { name
                ^'''));
        }
      });
    });


    testWithoutContext('intl package import should be kept in subclass files when plurals are included', () {
      const String pluralMessageArb = '''
{
  "helloWorlds": "{count,plural, =0{Hello} =1{Hello World} =2{Hello two worlds} few{Hello {count} worlds} many{Hello all {count} worlds} other{Hello other {count} worlds}}",
  "@helloWorlds": {
    "description": "A plural message",
    "placeholders": {
      "count": {}
    }
  }
}
''';
      const String pluralMessageEsArb = '''
{
  "helloWorlds": "{count,plural, =0{ES - Hello} =1{ES - Hello World} =2{ES - Hello two worlds} few{ES - Hello {count} worlds} many{ES - Hello all {count} worlds} other{ES - Hello other {count} worlds}}"
}
''';
      setupLocalizations(<String, String>{
        'en': pluralMessageArb,
        'es': pluralMessageEsArb,
      });
      expect(getGeneratedFileContent(locale: 'en'), contains(intlImportDartCode));
      expect(getGeneratedFileContent(locale: 'es'), contains(intlImportDartCode));
    });

    testWithoutContext('intl package import should be kept in subclass files when select is included', () {
      const String selectMessageArb = '''
{
  "genderSelect": "{gender, select, female {She} male {He} other {they} }",
  "@genderSelect": {
    "description": "A select message",
    "placeholders": {
      "gender": {}
    }
  }
}
''';
      const String selectMessageEsArb = '''
{
  "genderSelect": "{gender, select, female {ES - She} male {ES - He} other {ES - they} }"
}
''';
      setupLocalizations(<String, String>{
        'en': selectMessageArb,
        'es': selectMessageEsArb,
      });
      expect(getGeneratedFileContent(locale: 'en'), contains(intlImportDartCode));
      expect(getGeneratedFileContent(locale: 'es'), contains(intlImportDartCode));
    });

    testWithoutContext('check indentation on generated files', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      });
      // Tests a few of the lines in the generated code.
      // Localizations lookup code
      final String localizationsFile = getGeneratedFileContent();
      expect(localizationsFile.contains('  switch (locale.languageCode) {'), true);
      expect(localizationsFile.contains("    case 'en': return AppLocalizationsEn();"), true);
      expect(localizationsFile.contains("    case 'es': return AppLocalizationsEs();"), true);
      expect(localizationsFile.contains('  }'), true);

      // Supported locales list
      expect(localizationsFile.contains('  static const List<Locale> supportedLocales = <Locale>['), true);
      expect(localizationsFile.contains("    Locale('en'),"), true);
      expect(localizationsFile.contains("    Locale('es')"), true);
      expect(localizationsFile.contains('  ];'), true);
    });

    testWithoutContext('foundation package import should be omitted from file template when deferred loading = true', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      }, useDeferredLoading: true);
      expect(getGeneratedFileContent(), isNot(contains(foundationImportDartCode)));
    });

    testWithoutContext('foundation package import should be kept in file template when deferred loading = false', () {
      setupLocalizations(<String, String>{
        'en': singleMessageArbFileString,
        'es': singleEsMessageArbFileString,
      });
      expect(getGeneratedFileContent(), contains(foundationImportDartCode));
    });

    testWithoutContext('check for string interpolation rules', () {
      const String enArbCheckList = '''
{
  "one": "The number of {one} elapsed is: 44",
  "@one": {
    "description": "test one",
    "placeholders": {
      "one": {
        "type": "String"
      }
    }
  },
  "two": "哈{two}哈",
  "@two": {
    "description": "test two",
    "placeholders": {
      "two": {
        "type": "String"
      }
    }
  },
  "three": "m{three}m",
  "@three": {
    "description": "test three",
    "placeholders": {
      "three": {
        "type": "String"
      }
    }
  },
  "four": "I have to work _{four}_ sometimes.",
  "@four": {
    "description": "test four",
    "placeholders": {
      "four": {
        "type": "String"
      }
    }
  },
  "five": "{five} elapsed.",
  "@five": {
    "description": "test five",
    "placeholders": {
      "five": {
        "type": "String"
      }
    }
  },
  "six": "{six}m",
  "@six": {
    "description": "test six",
    "placeholders": {
      "six": {
        "type": "String"
      }
    }
  },
  "seven": "hours elapsed: {seven}",
  "@seven": {
    "description": "test seven",
    "placeholders": {
      "seven": {
        "type": "String"
      }
    }
  },
  "eight": " {eight}",
  "@eight": {
    "description": "test eight",
    "placeholders": {
      "eight": {
        "type": "String"
      }
    }
  },
  "nine": "m{nine}",
  "@nine": {
    "description": "test nine",
    "placeholders": {
      "nine": {
        "type": "String"
      }
    }
  }
}
''';

      // It's fine that the arb is identical -- Just checking
      // generated code for use of '${variable}' vs '$variable'
      const String esArbCheckList = '''
{
  "one": "The number of {one} elapsed is: 44",
  "two": "哈{two}哈",
  "three": "m{three}m",
  "four": "I have to work _{four}_ sometimes.",
  "five": "{five} elapsed.",
  "six": "{six}m",
  "seven": "hours elapsed: {seven}",
  "eight": " {eight}",
  "nine": "m{nine}"
}
''';
      setupLocalizations(<String, String>{
        'en': enArbCheckList,
        'es': esArbCheckList,
      });
      final String localizationsFile = getGeneratedFileContent(locale: 'es');
      expect(localizationsFile, contains(r'$one'));
      expect(localizationsFile, contains(r'$two'));
      expect(localizationsFile, contains(r'${three}'));
      expect(localizationsFile, contains(r'${four}'));
      expect(localizationsFile, contains(r'$five'));
      expect(localizationsFile, contains(r'${six}m'));
      expect(localizationsFile, contains(r'$seven'));
      expect(localizationsFile, contains(r'$eight'));
      expect(localizationsFile, contains(r'$nine'));
    });

    testWithoutContext('check for string interpolation rules - plurals', () {
      const String enArbCheckList = '''
{
  "first": "{count,plural, =0{test {count} test} =1{哈{count}哈} =2{m{count}m} few{_{count}_} many{{count} test} other{{count}m}}",
  "@first": {
    "description": "First set of plural messages to test.",
    "placeholders": {
      "count": {}
    }
  },
  "second": "{count,plural, =0{test {count}} other{ {count}}}",
  "@second": {
    "description": "Second set of plural messages to test.",
    "placeholders": {
      "count": {}
    }
  },
  "third": "{total,plural, =0{test {total}} other{ {total}}}",
  "@third": {
    "description": "Third set of plural messages to test, for number.",
    "placeholders": {
      "total": {
        "type": "int",
        "format": "compactLong"
      }
    }
  }
}
''';

      // It's fine that the arb is identical -- Just checking
      // generated code for use of '${variable}' vs '$variable'
      const String esArbCheckList = '''
{
  "first": "{count,plural, =0{test {count} test} =1{哈{count}哈} =2{m{count}m} few{_{count}_} many{{count} test} other{{count}m}}",
  "second": "{count,plural, =0{test {count}} other{ {count}}}"
}
''';
      setupLocalizations(<String, String>{
        'en': enArbCheckList,
        'es': esArbCheckList,
      });
      final String localizationsFile = getGeneratedFileContent(locale: 'es');
      expect(localizationsFile, contains(r'test $count test'));
      expect(localizationsFile, contains(r'哈$count哈'));
      expect(localizationsFile, contains(r'm${count}m'));
      expect(localizationsFile, contains(r'_${count}_'));
      expect(localizationsFile, contains(r'$count test'));
      expect(localizationsFile, contains(r'${count}m'));
      expect(localizationsFile, contains(r'test $count'));
      expect(localizationsFile, contains(r' $count'));
      expect(localizationsFile, contains(r'String totalString = totalNumberFormat'));
      expect(localizationsFile, contains(r'totalString'));
      expect(localizationsFile, contains(r'totalString'));
    });

    testWithoutContext(
      'should throw with descriptive error message when failing to parse the '
      'arb file',
      () {
        const String arbFileWithTrailingComma = '''
{
  "title": "Stocks",
  "@title": {
    "description": "Title for the Stocks application"
  },
}''';
        expect(
          () {
            setupLocalizations(<String, String>{ 'en': arbFileWithTrailingComma });
          },
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            allOf(
              contains('app_en.arb'),
              contains('FormatException'),
              contains('Unexpected character'),
            ),
          )),
        );
      },
    );

    testWithoutContext('should throw when resource is missing resource attribute (isResourceAttributeRequired = true)', () {
      const String arbFileWithMissingResourceAttribute = '''
{
  "title": "Stocks"
}''';
      expect(
        () {
          setupLocalizations(
            <String, String>{ 'en': arbFileWithMissingResourceAttribute },
            areResourceAttributeRequired: true,
          );
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('Resource attribute "@title" was not found'),
        )),
      );
    });

    group('checks for method/getter formatting', () {
      testWithoutContext('cannot contain non-alphanumeric symbols', () {
        const String nonAlphaNumericArbFile = '''
{
  "title!!": "Stocks",
  "@title!!": {
    "description": "Title for the Stocks application"
  }
}''';
        expect(
          () => setupLocalizations(<String, String>{ 'en': nonAlphaNumericArbFile }),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('Invalid ARB resource name'),
          )),
        );
      });

      testWithoutContext('must start with lowercase character', () {
        const String nonAlphaNumericArbFile = '''
{
  "Title": "Stocks",
  "@Title": {
    "description": "Title for the Stocks application"
  }
}''';
        expect(
          () => setupLocalizations(<String, String>{ 'en': nonAlphaNumericArbFile }),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('Invalid ARB resource name'),
          )),
        );
      });

      testWithoutContext('cannot start with a number', () {
        const String nonAlphaNumericArbFile = '''
{
  "123title": "Stocks",
  "@123title": {
    "description": "Title for the Stocks application"
  }
}''';
        expect(
          () => setupLocalizations(<String, String>{ 'en': nonAlphaNumericArbFile }),
          throwsA(isA<L10nException>().having(
            (L10nException e) => e.message,
            'message',
            contains('Invalid ARB resource name'),
          )),
        );
      });

      testWithoutContext('can start with and contain a dollar sign', () {
        const String dollarArbFile = r'''
{
  "$title$": "Stocks",
  "@$title$": {
    "description": "Title for the application"
  }
}''';
        setupLocalizations(<String, String>{ 'en': dollarArbFile });
      });
    });

    testWithoutContext('throws when the language code is not supported', () {
      const String arbFileWithInvalidCode = '''
{
  "@@locale": "invalid",
  "title": "invalid"
}''';

      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
        ..createSync(recursive: true);
      l10nDirectory.childFile('app_invalid.arb')
        .writeAsStringSync(arbFileWithInvalidCode);

      expect(
        () {
          LocalizationsGenerator(
            fileSystem: fs,
            inputPathString: defaultL10nPathString,
            outputPathString: defaultL10nPathString,
            templateArbFileName: 'app_invalid.arb',
            outputFileString: defaultOutputFileString,
            classNameString: defaultClassNameString,
            logger: logger,
          )
            ..loadResources()
            ..writeOutputFiles();
        },
        throwsA(isA<L10nException>().having(
          (L10nException e) => e.message,
          'message',
          contains('"invalid" is not a supported language code.'),
        )),
      );
    });

    testWithoutContext('handle number with multiple locale', () {
      setupLocalizations(<String, String>{
        'en': '''
{
"@@locale": "en",
"money": "Sum {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int",
      "format": "currency"
    }
  }
}
}''',
        'ja': '''
{
"@@locale": "ja",
"money": "合計 {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int",
      "format": "decimalPatternDigits",
      "optionalParameters": {
        "decimalDigits": 3
      }
    }
  }
}
}'''
      });

      expect(getGeneratedFileContent(locale: 'en'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'ja'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'en'), contains('intl.NumberFormat.currency('));
      expect(getGeneratedFileContent(locale: 'ja'), contains('intl.NumberFormat.decimalPatternDigits('));
      expect(getGeneratedFileContent(locale: 'ja'), contains('decimalDigits: 3'));
    });

    testWithoutContext('handle number with multiple locale specifying a format only in template', () {
      setupLocalizations(<String, String>{
        'en': '''
{
"@@locale": "en",
"money": "Sum {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int",
      "format": "decimalPatternDigits",
      "optionalParameters": {
        "decimalDigits": 3
      }
    }
  }
}
}''',
        'ja': '''
{
"@@locale": "ja",
"money": "合計 {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int"
    }
  }
}
}'''
      });

      expect(getGeneratedFileContent(locale: 'en'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'ja'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'en'), contains('intl.NumberFormat.decimalPatternDigits('));
      expect(getGeneratedFileContent(locale: 'en'), contains('decimalDigits: 3'));
      expect(getGeneratedFileContent(locale: 'en'), contains(r"return 'Sum $numberString'"));
      expect(getGeneratedFileContent(locale: 'ja'), isNot(contains('intl.NumberFormat')));
      expect(getGeneratedFileContent(locale: 'ja'), contains(r"return '合計 $number'"));
    });

    testWithoutContext('handle number with multiple locale specifying a format only in non-template', () {
      setupLocalizations(<String, String>{
        'en': '''
{
"@@locale": "en",
"money": "Sum {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int"
    }
  }
}
}''',
        'ja': '''
{
"@@locale": "ja",
"money": "合計 {number}",
"@money": {
  "placeholders": {
    "number": {
      "type": "int",
      "format": "decimalPatternDigits",
      "optionalParameters": {
        "decimalDigits": 3
      }
    }
  }
}
}'''
      });

      expect(getGeneratedFileContent(locale: 'en'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'ja'), contains('String money(int number)'));
      expect(getGeneratedFileContent(locale: 'en'), isNot(contains('intl.NumberFormat')));
      expect(getGeneratedFileContent(locale: 'en'), contains(r"return 'Sum $number'"));
      expect(getGeneratedFileContent(locale: 'ja'), contains('intl.NumberFormat.decimalPatternDigits('));
      expect(getGeneratedFileContent(locale: 'ja'), contains('decimalDigits: 3'));
      expect(getGeneratedFileContent(locale: 'ja'), contains(r"return '合計 $numberString'"));
    });
  });

  testWithoutContext('should generate a valid pubspec.yaml file when using synthetic package if it does not already exist', () {
    setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
    });
    final Directory outputDirectory = fs.directory(syntheticPackagePath);
    final File pubspecFile = outputDirectory.childFile('pubspec.yaml');
    expect(pubspecFile.existsSync(), isTrue);

    final YamlNode yamlNode = loadYamlNode(pubspecFile.readAsStringSync());
    expect(yamlNode, isA<YamlMap>());

    final YamlMap yamlMap = yamlNode as YamlMap;
    final String pubspecName = yamlMap['name'] as String;
    final String pubspecDescription = yamlMap['description'] as String;
    expect(pubspecName, 'synthetic_package');
    expect(pubspecDescription, "The Flutter application's synthetic package.");
  });

  testWithoutContext('should not overwrite existing pubspec.yaml file when using synthetic package', () {
    final File pubspecFile = fs.file(fs.path.join(syntheticPackagePath, 'pubspec.yaml'))
      ..createSync(recursive: true)
      ..writeAsStringSync('abcd');
    setupLocalizations(<String, String>{
      'en': singleMessageArbFileString,
    });
    // The original pubspec file should not be overwritten.
    expect(pubspecFile.readAsStringSync(), 'abcd');
  });

  testWithoutContext('can use type: int without specifying a format', () {
    const String arbFile = '''
{
  "orderNumber": "This is order #{number}.",
  "@orderNumber": {
    "description": "The title for an order with a given number.",
    "placeholders": {
      "number": {
        "type": "int"
      }
    }
  }
}''';
    setupLocalizations(<String, String>{
      'en': arbFile,
    });
    expect(getGeneratedFileContent(locale: 'en'), containsIgnoringWhitespace(r'''
String orderNumber(int number) {
  return 'This is order #$number.';
}
'''));
    expect(getGeneratedFileContent(locale: 'en'), contains(intlImportDartCode));
  });

  testWithoutContext('app localizations lookup is a public method', () {
    setupLocalizations(<String, String>{ 'en': singleMessageArbFileString });
    expect(getGeneratedFileContent(), containsIgnoringWhitespace(r'''
AppLocalizations lookupAppLocalizations(Locale locale) {
'''));
  });

  testWithoutContext('escaping with single quotes', () {
    const String arbFile = '''
{
  "singleQuote": "Flutter''s amazing!",
  "@singleQuote": {
    "description": "A message with a single quote."
  }
}''';
    setupLocalizations(<String, String>{ 'en': arbFile }, useEscaping: true);
    expect(getGeneratedFileContent(locale: 'en'), contains(r"Flutter\'s amazing"));
  });

  testWithoutContext('suppress warnings flag actually suppresses warnings', () {
    const String pluralMessageWithOverriddenParts = '''
{
  "helloWorlds": "{count,plural, =0{Hello}zero{hello} other{hi}}",
  "@helloWorlds": {
    "description": "Properly formatted but has redundant zero cases.",
    "placeholders": {
      "count": {}
    }
  }
}''';
    setupLocalizations(
      <String, String>{ 'en': pluralMessageWithOverriddenParts },
      suppressWarnings: true,
    );
    expect(logger.hadWarningOutput, isFalse);
  });

  testWithoutContext('can use decimalPatternDigits with decimalDigits optional parameter', () {
    const String arbFile = '''
{
  "treeHeight": "Tree height is {height}m.",
  "@treeHeight": {
    "placeholders": {
      "height": {
        "type": "double",
        "format": "decimalPatternDigits",
        "optionalParameters": {
          "decimalDigits": 3
        }
      }
    }
  }
}''';
    setupLocalizations(<String, String>{ 'en': arbFile });
    final String localizationsFile = getGeneratedFileContent(locale: 'en');
    expect(localizationsFile, containsIgnoringWhitespace(r'''
String treeHeight(double height) {
'''));
    expect(localizationsFile, containsIgnoringWhitespace(r'''
NumberFormat.decimalPatternDigits(
  locale: localeName,
  decimalDigits: 3
);
'''));
  });

  // Regression test for https://github.com/flutter/flutter/issues/125461.
  testWithoutContext('dollar signs are escaped properly when there is a select clause', () {
    const String dollarSignWithSelect = r'''
{
  "dollarSignWithSelect": "$nice_bug\nHello Bug! Manifestation #1 {selectPlaceholder, select, case{message} other{messageOther}}"
}''';
    setupLocalizations(<String, String>{ 'en': dollarSignWithSelect });
    expect(getGeneratedFileContent(locale: 'en'), contains(r'\$nice_bug\nHello Bug! Manifestation #1 $_temp0'));
  });

  testWithoutContext('can generate method with named parameter', () {
    const String arbFile = '''
{
  "helloName": "Hello {name}!",
  "@helloName": {
    "description": "A more personal greeting",
    "placeholders": {
      "name": {
        "type": "String",
        "description": "The name of the person to greet"
      }
    }
  },
  "helloNameAndAge": "Hello {name}! You are {age} years old.",
  "@helloNameAndAge": {
    "description": "A more personal greeting",
    "placeholders": {
      "name": {
        "type": "String",
        "description": "The name of the person to greet"
      },
      "age": {
        "type": "int",
        "description": "The age of the person to greet"
      }
    }
  }
}
    ''';
    setupLocalizations(<String, String>{ 'en': arbFile }, useNamedParameters: true);
    final String localizationsFile = getGeneratedFileContent(locale: 'en');
    expect(localizationsFile, containsIgnoringWhitespace(r'''
String helloName({required String name}) {
  '''));
    expect(localizationsFile, containsIgnoringWhitespace(r'''
String helloNameAndAge({required String name, required int age}) {
  '''));
  });

  testWithoutContext('throws when templateLocale and templateArbFileName are both null', () {
    fs.currentDirectory.childDirectory('lib').childDirectory('l10n')
      ..createSync(recursive: true)
      ..childFile('app.arb')
      .writeAsStringSync(singleMessageArbFileString);

    expect(
      () {
        LocalizationsGenerator(
          fileSystem: fs,
          inputPathString: defaultL10nPathString,
          outputPathString: defaultL10nPathString,
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          logger: logger,
          // ignore: avoid_redundant_argument_values
          templateArbFileName: null,
          // ignore: avoid_redundant_argument_values
          templateLocale: null,
        );
      },
      throwsA(isA<AssertionError>().having(
        (AssertionError e) => e.message,
        'message',
        contains('templateArbFileName or templateLocale must be provided.'),
      )),
    );
  });

  testWithoutContext(
    'correctly generates output files when localizations are inside a subdirectory',
    () {
      final Directory l10nDirectory = fs.currentDirectory.childDirectory('lib').childDirectory('l10n').childDirectory(defaultNamespace)
        ..createSync(recursive: true);

      l10nDirectory.childFile(defaultTemplateArbFileName)
        .writeAsStringSync(singleMessageArbFileString);
      l10nDirectory.childFile(esArbFileName)
        .writeAsStringSync(singleEsMessageArbFileString);

      LocalizationsGenerator(
        fileSystem: fs,
          inputPathString: defaultL10nPathString,
          outputPathString: fs.path.join('lib', 'l10n', 'output'),
          templateLocale: defaultTemplateLocale,
          outputFileString: defaultOutputFileString,
          classNameString: defaultClassNameString,
          useSyntheticPackage: false,
          logger: logger,
        )
        ..loadResources()
        ..writeOutputFiles();

      final Directory outputDirectory = fs.directory('lib').childDirectory('l10n').childDirectory('output');
      expect(outputDirectory.existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_en.dart').existsSync(), isTrue);
      expect(outputDirectory.childFile('output-localization-file_es.dart').existsSync(), isTrue);
    },
  );

  testWithoutContext(
    'correctly prefixes the generated keys with the namespace',
    () {
      setupLocalizations(
        <String, String>{ 'en': singleMessageArbFileString },
        namespace: defaultNamespace,
      );
      final String enLocalizationsFile = getGeneratedFileContent(locale: 'en');

      expect(enLocalizationsFile, containsIgnoringWhitespace(r'''
String get home_title => 'Title';
'''));
  });
}
