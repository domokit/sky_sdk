// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../convert.dart';
import '../flutter_manifest.dart';
import 'gen_l10n_templates.dart';
import 'gen_l10n_types.dart';
import 'localizations_utils.dart';
import 'message_parser.dart';

/// Run the localizations generation script with the configuration [options].
Future<LocalizationsGenerator> generateLocalizations({
  required Directory projectDir,
  Directory? dependenciesDir,
  required LocalizationOptions options,
  required Logger logger,
  required FileSystem fileSystem,
  required Artifacts artifacts,
  required ProcessManager processManager,
}) async {
  // If generating a synthetic package, generate a warning if
  // flutter: generate is not set.
  final FlutterManifest? flutterManifest = FlutterManifest.createFromPath(
    projectDir.childFile('pubspec.yaml').path,
    fileSystem: projectDir.fileSystem,
    logger: logger,
  );
  if (options.syntheticPackage &&
      (flutterManifest == null || !flutterManifest.generateLocalizations)) {
    throwToolExit(
      'Attempted to generate localizations code without having '
      'the flutter: generate flag turned on.'
      '\n'
      'Check pubspec.yaml and ensure that flutter: generate: true has '
      'been added and rebuild the project. Otherwise, the localizations '
      'source code will not be importable.',
    );
  }

  precacheLanguageAndRegionTags();

  // Use \r\n if project's pubspec file contains \r\n.
  final bool useCRLF = projectDir.childFile('pubspec.yaml').readAsStringSync().contains('\r\n');

  LocalizationsGenerator generator;
  try {
    generator =
        LocalizationsGenerator(
            fileSystem: fileSystem,
            inputsAndOutputsListPath: dependenciesDir?.path,
            projectPathString: projectDir.path,
            inputPathString: options.arbDir,
            templateArbFileName: options.templateArbFile,
            outputFileString: options.outputLocalizationFile,
            outputPathString: options.outputDir,
            classNameString: options.outputClass,
            preferredSupportedLocales: options.preferredSupportedLocales,
            headerString: options.header,
            headerFile: options.headerFile,
            useDeferredLoading: options.useDeferredLoading,
            useSyntheticPackage: options.syntheticPackage,
            areResourceAttributesRequired: options.requiredResourceAttributes,
            untranslatedMessagesFile: options.untranslatedMessagesFile,
            usesNullableGetter: options.nullableGetter,
            useEscaping: options.useEscaping,
            logger: logger,
            suppressWarnings: options.suppressWarnings,
            useRelaxedSyntax: options.relaxSyntax,
            useNamedParameters: options.useNamedParameters,
          )
          ..loadResources()
          ..writeOutputFiles(isFromYaml: true, useCRLF: useCRLF);
  } on L10nException catch (e) {
    throwToolExit(e.message);
  }

  if (options.format) {
    // Only format Dart files using `dart format`.
    final List<String> formatFileList = generator.outputFileList
        .where((String e) => e.endsWith('.dart'))
        .toList(growable: false);
    if (formatFileList.isEmpty) {
      return generator;
    }
    final String dartBinary = artifacts.getArtifactPath(Artifact.engineDartBinary);
    final List<String> command = <String>[dartBinary, 'format', ...formatFileList];
    final ProcessResult result = await processManager.run(command);
    if (result.exitCode != 0) {
      throw ProcessException(dartBinary, command, '''
`dart format` failed with exit code ${result.exitCode}

stdout:\n${result.stdout}\n
stderr:\n${result.stderr}''', result.exitCode);
    }
  }

  return generator;
}

/// The path for the synthetic package.
String _defaultSyntheticPackagePath(FileSystem fileSystem) =>
    fileSystem.path.join('.dart_tool', 'flutter_gen');

/// The default path used when the `_useSyntheticPackage` setting is set to true
/// in [LocalizationsGenerator].
///
/// See [LocalizationsGenerator.initialize] for where and how it is used by the
/// localizations tool.
String _syntheticL10nPackagePath(FileSystem fileSystem) =>
    fileSystem.path.join(_defaultSyntheticPackagePath(fileSystem), 'gen_l10n');

// Generate method parameters and also infer the correct types from the usage of the placeholders
// For example, if placeholders are used for plurals and no type was specified, then the type will
// automatically set to 'num'. Similarly, if such placeholders are used for selects, then the type
// will be set to 'String'. For such placeholders that are used for both, we should throw an error.
List<String> generateMethodParameters(
  Message message,
  LocaleInfo? locale,
  bool useNamedParameters,
) {
  // Check the compatibility of template placeholders and locale placeholders.
  final Map<String, Placeholder>? localePlaceholders = message.localePlaceholders[locale];

  return message.templatePlaceholders.entries.map((MapEntry<String, Placeholder> e) {
    final Placeholder placeholder = e.value;
    final Placeholder? localePlaceholder = localePlaceholders?[e.key];
    if (localePlaceholder != null && placeholder.type != localePlaceholder.type) {
      throw L10nException(
        'The placeholder, ${placeholder.name}, has its "type" resource attribute set to '
        'the "${localePlaceholder.type}" type in locale "$locale", but it is "${placeholder.type}" '
        'in the template placeholder. For compatibility with template placeholder, change '
        'the "type" attribute to "${placeholder.type}".',
      );
    }
    return '${useNamedParameters ? 'required ' : ''}${placeholder.type} ${placeholder.name}';
  }).toList();
}

// Similar to above, but is used for passing arguments into helper functions.
List<String> generateMethodArguments(Message message) {
  return message.templatePlaceholders.values
      .map((Placeholder placeholder) => placeholder.name)
      .toList();
}

String generateDateFormattingLogic(Message message, LocaleInfo locale) {
  if (message.templatePlaceholders.isEmpty) {
    return '@(none)';
  }

  final Iterable<String> formatStatements = message
      .getPlaceholders(locale)
      .where((Placeholder placeholder) => placeholder.requiresDateFormatting)
      .map((Placeholder placeholder) {
        final String? placeholderFormat = placeholder.format;
        if (placeholderFormat == null) {
          throw L10nException(
            'The placeholder, ${placeholder.name}, has its "type" resource attribute set to '
            'the "${placeholder.type}" type. To properly resolve for the right '
            '${placeholder.type} format, the "format" attribute needs to be set '
            'to determine which DateFormat to use. \n'
            "Check the intl library's DateFormat class constructors for allowed "
            'date formats.',
          );
        }
        final bool? isCustomDateFormat = placeholder.isCustomDateFormat;
        if (!placeholder.hasValidDateFormat &&
            (isCustomDateFormat == null || !isCustomDateFormat)) {
          if (placeholder.dateFormatParts.length > 1) {
            throw L10nException(
              'Date format "$placeholderFormat" for placeholder '
              '${placeholder.name} contains at least one invalid date format. '
              'Ensure all date formats joined by a "+" character have '
              'a corresponding DateFormat constructor.\n Check the intl '
              "library's DateFormat class constructors for allowed date formats.",
            );
          }

          throw L10nException(
            'Date format "$placeholderFormat" for placeholder '
            '${placeholder.name} does not have a corresponding DateFormat '
            "constructor\n. Check the intl library's DateFormat class "
            'constructors for allowed date formats, or set "isCustomDateFormat" attribute '
            'to "true".',
          );
        }
        if (placeholder.hasValidDateFormat) {
          // The first format is the main format, and the rest are added formats.
          final List<String> formatParts = placeholder.dateFormatParts;
          final String mainFormat = formatParts.first;
          final List<String> addedFormats = formatParts.skip(1).toList();

          final String addedFormatsString =
              addedFormats.map((String addFormat) {
                return dateFormatAddFormatTemplate.replaceAll('@(format)', addFormat);
              }).join();

          return dateFormatTemplate
              .replaceAll('@(placeholder)', placeholder.name)
              .replaceAll('@(format)', mainFormat)
              .replaceAll('@(addedFormats)', addedFormatsString);
        }
        return dateFormatCustomTemplate
            .replaceAll('@(placeholder)', placeholder.name)
            .replaceAll('@(format)', "'${generateString(placeholderFormat)}'");
      });

  return formatStatements.isEmpty ? '@(none)' : formatStatements.join();
}

String generateNumberFormattingLogic(Message message, LocaleInfo locale) {
  if (message.templatePlaceholders.isEmpty) {
    return '@(none)';
  }

  final Iterable<String> formatStatements = message
      .getPlaceholders(locale)
      .where((Placeholder placeholder) => placeholder.requiresNumFormatting)
      .map((Placeholder placeholder) {
        final String? placeholderFormat = placeholder.format;
        if (!placeholder.hasValidNumberFormat || placeholderFormat == null) {
          throw L10nException(
            'Number format $placeholderFormat for the ${placeholder.name} '
            'placeholder does not have a corresponding NumberFormat constructor.\n'
            "Check the intl library's NumberFormat class constructors for allowed "
            'number formats.',
          );
        }
        final Iterable<String> parameters = placeholder.optionalParameters.map<String>((
          OptionalParameter parameter,
        ) {
          if (parameter.value is num) {
            return '${parameter.name}: ${parameter.value}';
          } else {
            return "${parameter.name}: '${generateString(parameter.value.toString())}'";
          }
        });

        if (placeholder.hasNumberFormatWithParameters) {
          return numberFormatNamedTemplate
              .replaceAll('@(placeholder)', placeholder.name)
              .replaceAll('@(format)', placeholderFormat)
              .replaceAll('@(parameters)', parameters.join(',\n      '));
        } else {
          return numberFormatPositionalTemplate
              .replaceAll('@(placeholder)', placeholder.name)
              .replaceAll('@(format)', placeholderFormat);
        }
      });

  return formatStatements.isEmpty ? '@(none)' : formatStatements.join();
}

/// List of possible cases for plurals defined the ICU messageFormat syntax.
Map<String, String> pluralCases = <String, String>{
  '0': 'zero',
  '1': 'one',
  '2': 'two',
  'zero': 'zero',
  'one': 'one',
  'two': 'two',
  'few': 'few',
  'many': 'many',
  'other': 'other',
};

String generateBaseClassMethod(
  Message message,
  LocaleInfo? templateArbLocale,
  bool useNamedParameters,
) {
  final String comment =
      message.description?.split('\n').map((String line) => '  /// $line').join('\n') ??
      '  /// No description provided for @${message.resourceId}.';
  final String templateLocaleTranslationComment = '''
  /// In $templateArbLocale, this message translates to:
  /// **'${generateString(message.value)}'**''';

  if (message.templatePlaceholders.isNotEmpty) {
    return (useNamedParameters
            ? baseClassMethodWithNamedParameterTemplate
            : baseClassMethodTemplate)
        .replaceAll('@(comment)', comment)
        .replaceAll('@(templateLocaleTranslationComment)', templateLocaleTranslationComment)
        .replaceAll('@(name)', message.resourceId)
        .replaceAll(
          '@(parameters)',
          generateMethodParameters(message, null, useNamedParameters).join(', '),
        );
  }
  return baseClassGetterTemplate
      .replaceAll('@(comment)', comment)
      .replaceAll('@(templateLocaleTranslationComment)', templateLocaleTranslationComment)
      .replaceAll('@(name)', message.resourceId);
}

// Add spaces to pad the start of each line. Skips the first line
// assuming that the padding is already present.
String _addSpaces(String message, {int spaces = 0}) {
  bool isFirstLine = true;
  return message
      .split('\n')
      .map((String value) {
        if (isFirstLine) {
          isFirstLine = false;
          return value;
        }
        return value.padLeft(spaces);
      })
      .join('\n');
}

String _generateLookupByAllCodes(
  AppResourceBundleCollection allBundles,
  String Function(LocaleInfo) generateSwitchClauseTemplate,
) {
  final Iterable<LocaleInfo> localesWithAllCodes = allBundles.locales.where((LocaleInfo locale) {
    return locale.scriptCode != null && locale.countryCode != null;
  });

  if (localesWithAllCodes.isEmpty) {
    return '';
  }

  final Iterable<String> switchClauses = localesWithAllCodes.map<String>((LocaleInfo locale) {
    return generateSwitchClauseTemplate(locale).replaceAll('@(case)', locale.toString());
  });

  return allCodesLookupTemplate.replaceAll(
    '@(allCodesSwitchClauses)',
    switchClauses.join('\n        '),
  );
}

String _generateLookupByScriptCode(
  AppResourceBundleCollection allBundles,
  String Function(LocaleInfo) generateSwitchClauseTemplate,
) {
  final Iterable<String> switchClauses =
      allBundles.languages.map((String language) {
        final Iterable<LocaleInfo> locales = allBundles.localesForLanguage(language);
        final Iterable<LocaleInfo> localesWithScriptCodes = locales.where((LocaleInfo locale) {
          return locale.scriptCode != null && locale.countryCode == null;
        });

        if (localesWithScriptCodes.isEmpty) {
          return null;
        }

        return _addSpaces(
          nestedSwitchTemplate
              .replaceAll('@(languageCode)', language)
              .replaceAll('@(code)', 'scriptCode')
              .replaceAll(
                '@(switchClauses)',
                _addSpaces(
                  localesWithScriptCodes
                      .map((LocaleInfo locale) {
                        return generateSwitchClauseTemplate(
                          locale,
                        ).replaceAll('@(case)', locale.scriptCode!);
                      })
                      .join('\n'),
                  spaces: 8,
                ),
              ),
          spaces: 4,
        );
      }).whereType<String>();

  if (switchClauses.isEmpty) {
    return '';
  }

  return languageCodeSwitchTemplate
      .replaceAll('@(comment)', '// Lookup logic when language+script codes are specified.')
      .replaceAll('@(switchClauses)', switchClauses.join('\n      '));
}

String _generateLookupByCountryCode(
  AppResourceBundleCollection allBundles,
  String Function(LocaleInfo) generateSwitchClauseTemplate,
) {
  final Iterable<String> switchClauses =
      allBundles.languages.map((String language) {
        final Iterable<LocaleInfo> locales = allBundles.localesForLanguage(language);
        final Iterable<LocaleInfo> localesWithCountryCodes = locales.where((LocaleInfo locale) {
          return locale.countryCode != null && locale.scriptCode == null;
        });

        if (localesWithCountryCodes.isEmpty) {
          return null;
        }

        return _addSpaces(
          nestedSwitchTemplate
              .replaceAll('@(languageCode)', language)
              .replaceAll('@(code)', 'countryCode')
              .replaceAll(
                '@(switchClauses)',
                _addSpaces(
                  localesWithCountryCodes
                      .map((LocaleInfo locale) {
                        return generateSwitchClauseTemplate(
                          locale,
                        ).replaceAll('@(case)', locale.countryCode!);
                      })
                      .join('\n'),
                  spaces: 4,
                ),
              ),
          spaces: 4,
        );
      }).whereType<String>();

  if (switchClauses.isEmpty) {
    return '';
  }

  return languageCodeSwitchTemplate
      .replaceAll('@(comment)', '// Lookup logic when language+country codes are specified.')
      .replaceAll('@(switchClauses)', switchClauses.join('\n    '));
}

String _generateLookupByLanguageCode(
  AppResourceBundleCollection allBundles,
  String Function(LocaleInfo) generateSwitchClauseTemplate,
) {
  final Iterable<String> switchClauses =
      allBundles.languages.map((String language) {
        final Iterable<LocaleInfo> locales = allBundles.localesForLanguage(language);
        final Iterable<LocaleInfo> localesWithLanguageCode = locales.where((LocaleInfo locale) {
          return locale.countryCode == null && locale.scriptCode == null;
        });

        if (localesWithLanguageCode.isEmpty) {
          return null;
        }

        return localesWithLanguageCode
            .map((LocaleInfo locale) {
              return generateSwitchClauseTemplate(
                locale,
              ).replaceAll('@(case)', locale.languageCode);
            })
            .join('\n      ');
      }).whereType<String>();

  if (switchClauses.isEmpty) {
    return '';
  }

  return languageCodeSwitchTemplate
      .replaceAll('@(comment)', '// Lookup logic when only language code is specified.')
      .replaceAll('@(switchClauses)', switchClauses.join('\n    '));
}

String _generateLookupBody(
  AppResourceBundleCollection allBundles,
  String className,
  bool useDeferredLoading,
  String fileName,
) {
  String generateSwitchClauseTemplate(LocaleInfo locale) {
    return (useDeferredLoading ? switchClauseDeferredLoadingTemplate : switchClauseTemplate)
        .replaceAll('@(localeClass)', '$className${locale.camelCase()}')
        .replaceAll('@(appClass)', className)
        .replaceAll('@(library)', '${fileName}_${locale.languageCode}');
  }

  return lookupBodyTemplate
      .replaceAll(
        '@(lookupAllCodesSpecified)',
        _generateLookupByAllCodes(allBundles, generateSwitchClauseTemplate),
      )
      .replaceAll(
        '@(lookupScriptCodeSpecified)',
        _generateLookupByScriptCode(allBundles, generateSwitchClauseTemplate),
      )
      .replaceAll(
        '@(lookupCountryCodeSpecified)',
        _generateLookupByCountryCode(allBundles, generateSwitchClauseTemplate),
      )
      .replaceAll(
        '@(lookupLanguageCodeSpecified)',
        _generateLookupByLanguageCode(allBundles, generateSwitchClauseTemplate),
      );
}

String _generateDelegateClass({
  required AppResourceBundleCollection allBundles,
  required String className,
  required Set<String> supportedLanguageCodes,
  required bool useDeferredLoading,
  required String fileName,
}) {
  final String lookupBody = _generateLookupBody(
    allBundles,
    className,
    useDeferredLoading,
    fileName,
  );
  final String loadBody = (useDeferredLoading ? loadBodyDeferredLoadingTemplate : loadBodyTemplate)
      .replaceAll('@(class)', className)
      .replaceAll('@(lookupName)', 'lookup$className');
  final String lookupFunction = (useDeferredLoading
          ? lookupFunctionDeferredLoadingTemplate
          : lookupFunctionTemplate)
      .replaceAll('@(class)', className)
      .replaceAll('@(lookupName)', 'lookup$className')
      .replaceAll('@(lookupBody)', lookupBody);
  return delegateClassTemplate
      .replaceAll('@(class)', className)
      .replaceAll('@(loadBody)', loadBody)
      .replaceAll('@(supportedLanguageCodes)', supportedLanguageCodes.join(', '))
      .replaceAll('@(lookupFunction)', lookupFunction);
}

class LocalizationsGenerator {
  /// Initializes [inputDirectory], [outputDirectory], [templateArbFile],
  /// [outputFile] and [className].
  ///
  /// Throws an [L10nException] when a provided configuration is not allowed
  /// by [LocalizationsGenerator].
  ///
  /// Throws a [FileSystemException] when a file operation necessary for setting
  /// up the [LocalizationsGenerator] cannot be completed.
  factory LocalizationsGenerator({
    required FileSystem fileSystem,
    required String inputPathString,
    String? outputPathString,
    required String templateArbFileName,
    required String outputFileString,
    required String classNameString,
    List<String>? preferredSupportedLocales,
    String? headerString,
    String? headerFile,
    bool useDeferredLoading = false,
    String? inputsAndOutputsListPath,
    bool useSyntheticPackage = true,
    String? projectPathString,
    bool areResourceAttributesRequired = false,
    String? untranslatedMessagesFile,
    bool usesNullableGetter = true,
    bool useEscaping = false,
    required Logger logger,
    bool suppressWarnings = false,
    bool useRelaxedSyntax = false,
    bool useNamedParameters = false,
  }) {
    final Directory? projectDirectory = projectDirFromPath(fileSystem, projectPathString);
    final Directory inputDirectory = inputDirectoryFromPath(
      fileSystem,
      inputPathString,
      projectDirectory,
    );
    final Directory outputDirectory = outputDirectoryFromPath(
      fileSystem,
      outputPathString ?? inputPathString,
      useSyntheticPackage,
      projectDirectory,
    );
    return LocalizationsGenerator._(
      fileSystem,
      useSyntheticPackage: useSyntheticPackage,
      usesNullableGetter: usesNullableGetter,
      className: classNameFromString(classNameString),
      projectDirectory: projectDirectory,
      inputDirectory: inputDirectory,
      outputDirectory: outputDirectory,
      templateArbFile: templateArbFileFromFileName(templateArbFileName, inputDirectory),
      baseOutputFile: outputDirectory.childFile(outputFileString),
      preferredSupportedLocales: preferredSupportedLocalesFromLocales(preferredSupportedLocales),
      header: headerFromFile(headerString, headerFile, inputDirectory),
      useDeferredLoading: useDeferredLoading,
      untranslatedMessagesFile: _untranslatedMessagesFileFromPath(
        fileSystem,
        untranslatedMessagesFile,
      ),
      inputsAndOutputsListFile: _inputsAndOutputsListFileFromPath(
        fileSystem,
        inputsAndOutputsListPath,
      ),
      areResourceAttributesRequired: areResourceAttributesRequired,
      useEscaping: useEscaping,
      logger: logger,
      suppressWarnings: suppressWarnings,
      useRelaxedSyntax: useRelaxedSyntax,
      useNamedParameters: useNamedParameters,
    );
  }

  /// Creates an instance of the localizations generator class.
  ///
  /// It takes in a [FileSystem] representation that the class will act upon.
  LocalizationsGenerator._(
    this._fs, {
    required this.inputDirectory,
    required this.outputDirectory,
    required this.templateArbFile,
    required this.baseOutputFile,
    required this.className,
    this.preferredSupportedLocales = const <LocaleInfo>[],
    this.header = '',
    this.useDeferredLoading = false,
    required this.inputsAndOutputsListFile,
    this.useSyntheticPackage = true,
    this.projectDirectory,
    this.areResourceAttributesRequired = false,
    this.untranslatedMessagesFile,
    this.usesNullableGetter = true,
    required this.logger,
    this.useEscaping = false,
    this.suppressWarnings = false,
    this.useRelaxedSyntax = false,
    this.useNamedParameters = false,
  });

  final FileSystem _fs;
  List<Message> _allMessages = <Message>[];
  late final AppResourceBundleCollection _allBundles = AppResourceBundleCollection(inputDirectory);
  late final AppResourceBundle _templateBundle = AppResourceBundle(templateArbFile);
  late final Map<LocaleInfo, String> _inputFileNames = Map<LocaleInfo, String>.fromEntries(
    _allBundles.bundles.map(
      (AppResourceBundle bundle) =>
          MapEntry<LocaleInfo, String>(bundle.locale, bundle.file.basename),
    ),
  );
  late final LocaleInfo _templateArbLocale = _templateBundle.locale;

  @visibleForTesting
  final bool useSyntheticPackage;

  // Used to decide if the generated code is nullable or not
  // (whether AppLocalizations? or AppLocalizations is returned from
  // `static {name}Localizations{?} of (BuildContext context))`
  @visibleForTesting
  final bool usesNullableGetter;

  /// The directory that contains the project's arb files, as well as the
  /// header file, if specified.
  ///
  /// It is assumed that all input files (e.g. [templateArbFile], arb files
  /// for translated messages, header file templates) will reside here.
  final Directory inputDirectory;

  /// The Flutter project's root directory.
  final Directory? projectDirectory;

  /// The directory to generate the project's localizations files in.
  ///
  /// It is assumed that all output files (e.g. The localizations
  /// [outputFile], `messages_<locale>.dart` and `messages_all.dart`)
  /// will reside here.
  final Directory outputDirectory;

  /// The input arb file which defines all of the messages that will be
  /// exported by the generated class that's written to [outputFile].
  final File templateArbFile;

  /// The file to write the generated abstract localizations and
  /// localizations delegate classes to. Separate localizations
  /// files will also be generated for each language using this
  /// filename as a prefix and the locale as the suffix.
  final File baseOutputFile;

  /// The class name to be used for the localizations class in [outputFile].
  ///
  /// For example, if 'AppLocalizations' is passed in, a class named
  /// AppLocalizations will be used for localized message lookups.
  final String className;

  /// The list of preferred supported locales.
  ///
  /// By default, the list of supported locales in the localizations class
  /// will be sorted in alphabetical order. However, this option
  /// allows for a set of preferred locales to appear at the top of the
  /// list.
  ///
  /// The order of locales in this list will also be the order of locale
  /// priority. For example, if a device supports 'en' and 'es' and
  /// ['es', 'en'] is passed in, the 'es' locale will take priority over 'en'.
  final List<LocaleInfo> preferredSupportedLocales;

  // Whether we want to use escaping for ICU messages.
  bool useEscaping = false;

  /// Whether any errors were caught. This is set after encountering any errors
  /// from calling [_generateMethod].
  bool hadErrors = false;

  /// Whether to use relaxed syntax.
  bool useRelaxedSyntax = false;

  /// The list of all arb path strings in [inputDirectory].
  List<String> get arbPathStrings {
    return _allBundles.bundles.map((AppResourceBundle bundle) => bundle.file.path).toList();
  }

  List<String> get outputFileList {
    return _outputFileList;
  }

  /// The supported language codes as found in the arb files located in
  /// [inputDirectory].
  final Set<String> supportedLanguageCodes = <String>{};

  /// The supported locales as found in the arb files located in
  /// [inputDirectory].
  final Set<LocaleInfo> supportedLocales = <LocaleInfo>{};

  /// The header to be prepended to the generated Dart localization file.
  final String header;

  final Map<LocaleInfo, List<String>> _unimplementedMessages = <LocaleInfo, List<String>>{};

  /// Whether to generate the Dart localization file with locales imported as
  /// deferred, allowing for lazy loading of each locale in Flutter web.
  ///
  /// This can reduce a web app’s initial startup time by decreasing the size of
  /// the JavaScript bundle. When [_useDeferredLoading] is set to true, the
  /// messages for a particular locale are only downloaded and loaded by the
  /// Flutter app as they are needed. For projects with a lot of different
  /// locales and many localization strings, it can be an performance
  /// improvement to have deferred loading. For projects with a small number of
  /// locales, the difference is negligible, and might slow down the start up
  /// compared to bundling the localizations with the rest of the application.
  ///
  /// This flag does not affect other platforms such as mobile or desktop.
  final bool useDeferredLoading;

  /// Contains a map of each output language file to its corresponding content in
  /// string format.
  final Map<File, String> _languageFileMap = <File, String>{};

  /// A generated file that will contain the list of messages for each locale
  /// that do not have a translation yet.
  final File? untranslatedMessagesFile;

  /// The file that contains the list of inputs and outputs for generating
  /// localizations.
  @visibleForTesting
  final File? inputsAndOutputsListFile;
  final List<String> _inputFileList = <String>[];
  final List<String> _outputFileList = <String>[];

  /// Whether or not resource attributes are required for each corresponding
  /// resource id.
  ///
  /// Resource attributes provide metadata about the message.
  @visibleForTesting
  final bool areResourceAttributesRequired;

  /// Logger to be used during the execution of the script.
  Logger logger;

  /// Whether or not to suppress warnings or not.
  final bool suppressWarnings;

  /// Whether to generate the Dart localization methods with named parameters.
  ///
  /// If this sets to true, the generated Dart localization methods will be:
  /// ```dart
  /// String helloWorld({required String name});
  /// ```
  final bool useNamedParameters;

  static bool _isNotReadable(FileStat fileStat) {
    final String rawStatString = fileStat.modeString();
    // Removes potential prepended permission bits, such as '(suid)' and '(guid)'.
    final String statString = rawStatString.substring(rawStatString.length - 9);
    return !(statString[0] == 'r' || statString[3] == 'r' || statString[6] == 'r');
  }

  static bool _isNotWritable(FileStat fileStat) {
    final String rawStatString = fileStat.modeString();
    // Removes potential prepended permission bits, such as '(suid)' and '(guid)'.
    final String statString = rawStatString.substring(rawStatString.length - 9);
    return !(statString[1] == 'w' || statString[4] == 'w' || statString[7] == 'w');
  }

  @visibleForTesting
  static Directory? projectDirFromPath(FileSystem fileSystem, String? projectPathString) {
    if (projectPathString == null) {
      return null;
    }

    final Directory directory = fileSystem.directory(projectPathString);
    if (!directory.existsSync()) {
      throw L10nException(
        'Directory does not exist: $directory.\n'
        "Please select a directory that contains the project's localizations "
        'resource files.',
      );
    }
    return directory;
  }

  /// Sets the reference [Directory] for [inputDirectory].
  @visibleForTesting
  static Directory inputDirectoryFromPath(
    FileSystem fileSystem,
    String inputPathString,
    Directory? projectDirectory,
  ) {
    final Directory inputDirectory = fileSystem.directory(
      projectDirectory != null
          ? _getAbsoluteProjectPath(inputPathString, projectDirectory)
          : inputPathString,
    );

    if (!inputDirectory.existsSync()) {
      throw L10nException(
        "The 'arb-dir' directory, '$inputDirectory', does not exist.\n"
        'Make sure that the correct path was provided.',
      );
    }

    final FileStat fileStat = inputDirectory.statSync();
    if (_isNotReadable(fileStat) || _isNotWritable(fileStat)) {
      throw L10nException(
        "The 'arb-dir' directory, '$inputDirectory', doesn't allow reading and writing.\n"
        'Please ensure that the user has read and write permissions.',
      );
    }
    return inputDirectory;
  }

  /// Sets the reference [Directory] for [outputDirectory].
  @visibleForTesting
  static Directory outputDirectoryFromPath(
    FileSystem fileSystem,
    String outputPathString,
    bool useSyntheticPackage,
    Directory? projectDirectory,
  ) {
    Directory outputDirectory;
    if (useSyntheticPackage) {
      outputDirectory = fileSystem.directory(
        projectDirectory != null
            ? _getAbsoluteProjectPath(_syntheticL10nPackagePath(fileSystem), projectDirectory)
            : _syntheticL10nPackagePath(fileSystem),
      );
    } else {
      outputDirectory = fileSystem.directory(
        projectDirectory != null
            ? _getAbsoluteProjectPath(outputPathString, projectDirectory)
            : outputPathString,
      );
    }
    return outputDirectory;
  }

  /// Sets the reference [File] for [templateArbFile].
  @visibleForTesting
  static File templateArbFileFromFileName(String templateArbFileName, Directory inputDirectory) {
    final File templateArbFile = inputDirectory.childFile(templateArbFileName);
    final FileStat templateArbFileStat = templateArbFile.statSync();
    if (templateArbFileStat.type == FileSystemEntityType.notFound) {
      throw L10nException("The 'template-arb-file', $templateArbFile, does not exist.");
    }
    final String templateArbFileStatModeString = templateArbFileStat.modeString();
    if (templateArbFileStatModeString[0] == '-' && templateArbFileStatModeString[3] == '-') {
      throw L10nException(
        "The 'template-arb-file', $templateArbFile, is not readable.\n"
        'Please ensure that the user has read permissions.',
      );
    }
    return templateArbFile;
  }

  static bool _isValidClassName(String className) {
    // Public Dart class name cannot begin with an underscore
    if (className[0] == '_') {
      return false;
    }
    // Dart class name cannot contain non-alphanumeric symbols
    if (className.contains(RegExp(r'[^a-zA-Z_\d]'))) {
      return false;
    }
    // Dart class name must start with upper case character
    if (className[0].contains(RegExp(r'[a-z]'))) {
      return false;
    }
    // Dart class name cannot start with a number
    if (className[0].contains(RegExp(r'\d'))) {
      return false;
    }
    return true;
  }

  /// Sets the [className] for the localizations and localizations delegate
  /// classes.
  @visibleForTesting
  static String classNameFromString(String classNameString) {
    if (classNameString.isEmpty) {
      throw L10nException('classNameString argument cannot be empty');
    }
    if (!_isValidClassName(classNameString)) {
      throw L10nException(
        "The 'output-class', $classNameString, is not a valid public Dart class name.\n",
      );
    }
    return classNameString;
  }

  /// Sets [preferredSupportedLocales] so that this particular list of locales
  /// will take priority over the other locales.
  @visibleForTesting
  static List<LocaleInfo> preferredSupportedLocalesFromLocales(List<String>? inputLocales) {
    if (inputLocales == null || inputLocales.isEmpty) {
      return const <LocaleInfo>[];
    }
    return inputLocales.map((String localeString) {
      return LocaleInfo.fromString(localeString);
    }).toList();
  }

  static String headerFromFile(String? headerString, String? headerFile, Directory inputDirectory) {
    if (headerString != null && headerFile != null) {
      throw L10nException(
        'Cannot accept both header and header file arguments. \n'
        'Please make sure to define only one or the other. ',
      );
    }

    if (headerString != null) {
      return headerString;
    } else if (headerFile != null) {
      try {
        return inputDirectory.childFile(headerFile).readAsStringSync();
      } on FileSystemException catch (error) {
        throw L10nException(
          'Failed to read header file: "$headerFile". \n'
          'FileSystemException: ${error.message}',
        );
      }
    }
    return '';
  }

  static String _getAbsoluteProjectPath(String relativePath, Directory projectDirectory) =>
      projectDirectory.fileSystem.path.join(projectDirectory.path, relativePath);

  static File? _untranslatedMessagesFileFromPath(
    FileSystem fileSystem,
    String? untranslatedMessagesFileString,
  ) {
    if (untranslatedMessagesFileString == null || untranslatedMessagesFileString.isEmpty) {
      return null;
    }
    untranslatedMessagesFileString = untranslatedMessagesFileString.replaceAll(
      r'\',
      fileSystem.path.separator,
    );
    return fileSystem.file(untranslatedMessagesFileString);
  }

  static File? _inputsAndOutputsListFileFromPath(
    FileSystem fileSystem,
    String? inputsAndOutputsListPath,
  ) {
    if (inputsAndOutputsListPath == null) {
      return null;
    }

    return fileSystem.file(
      fileSystem.path.join(inputsAndOutputsListPath, 'gen_l10n_inputs_and_outputs.json'),
    );
  }

  static bool _isValidGetterAndMethodName(String name) {
    if (name.isEmpty) {
      return false;
    }
    // Public Dart method name must not start with an underscore
    if (name[0] == '_') {
      return false;
    }
    // Dart identifiers can only use letters, numbers, underscores, and `$`
    if (name.contains(RegExp(r'[^a-zA-Z_$\d]'))) {
      return false;
    }
    // Dart getter and method name should start with lower case character
    if (name[0].contains(RegExp(r'[A-Z]'))) {
      return false;
    }
    // Dart getter and method name cannot start with a number
    if (name[0].contains(RegExp(r'\d'))) {
      return false;
    }
    return true;
  }

  // Load _allMessages from templateArbFile and _allBundles from all of the ARB
  // files in inputDirectory. Also initialized: supportedLocales.
  void loadResources() {
    for (final String resourceId in _templateBundle.resourceIds) {
      if (!_isValidGetterAndMethodName(resourceId)) {
        throw L10nException(
          'Invalid ARB resource name "$resourceId" in $templateArbFile.\n'
          'Resources names must be valid Dart method names: they have to be '
          'camel case, cannot start with a number or underscore, and cannot '
          'contain non-alphanumeric characters.',
        );
      }
    }
    // The call to .toList() is absolutely necessary. Otherwise, it is an iterator and will call Message's constructor again.
    _allMessages =
        _templateBundle.resourceIds
            .map(
              (String id) => Message(
                _templateBundle,
                _allBundles,
                id,
                areResourceAttributesRequired,
                useEscaping: useEscaping,
                logger: logger,
                useRelaxedSyntax: useRelaxedSyntax,
              ),
            )
            .toList();
    hadErrors = _allMessages.any((Message message) => message.hadErrors);
    if (inputsAndOutputsListFile != null) {
      _inputFileList.addAll(
        _allBundles.bundles.map((AppResourceBundle bundle) {
          return bundle.file.absolute.path;
        }),
      );
    }

    final List<LocaleInfo> allLocales = List<LocaleInfo>.from(_allBundles.locales);
    for (final LocaleInfo preferredLocale in preferredSupportedLocales) {
      final int index = allLocales.indexOf(preferredLocale);
      if (index == -1) {
        throw L10nException(
          "The preferred supported locale, '$preferredLocale', cannot be "
          'added. Please make sure that there is a corresponding ARB file '
          'with translations for the locale, or remove the locale from the '
          'preferred supported locale list.',
        );
      }
      allLocales.removeAt(index);
      allLocales.insertAll(0, preferredSupportedLocales);
    }
    supportedLocales.addAll(allLocales);
  }

  void _addUnimplementedMessage(LocaleInfo locale, String message) {
    if (_unimplementedMessages.containsKey(locale)) {
      _unimplementedMessages[locale]!.add(message);
    } else {
      _unimplementedMessages.putIfAbsent(locale, () => <String>[message]);
    }
  }

  String _generateBaseClassFile(
    String className,
    String fileName,
    String header,
    final LocaleInfo locale,
  ) {
    final Iterable<String> methods = _allMessages.map((Message message) {
      LocaleInfo localeWithFallback = locale;
      if (message.messages[locale] == null) {
        _addUnimplementedMessage(locale, message.resourceId);
        localeWithFallback = _templateArbLocale;
      }
      if (message.parsedMessages[localeWithFallback] == null) {
        // The message exists, but parsedMessages[locale] is null due to a syntax error.
        // This means that we have already set hadErrors = true while constructing the Message.
        return '';
      }
      return _generateMethod(message, localeWithFallback);
    });

    return classFileTemplate
        .replaceAll('@(header)', header.isEmpty ? '' : '$header\n\n')
        .replaceAll('@(language)', describeLocale(locale.toString()))
        .replaceAll('@(baseClass)', className)
        .replaceAll('@(fileName)', fileName)
        .replaceAll('@(class)', '$className${locale.camelCase()}')
        .replaceAll('@(localeName)', locale.toString())
        .replaceAll('@(methods)', methods.join('\n\n'));
  }

  String _generateSubclass(String className, AppResourceBundle bundle) {
    final LocaleInfo locale = bundle.locale;
    final String baseClassName =
        '$className${LocaleInfo.fromString(locale.languageCode).camelCase()}';

    _allMessages.where((Message message) => message.messages[locale] == null).forEach((
      Message message,
    ) {
      _addUnimplementedMessage(locale, message.resourceId);
    });

    final Iterable<String> methods = _allMessages
        .where((Message message) => message.parsedMessages[locale] != null)
        .map((Message message) => _generateMethod(message, locale));

    return subclassTemplate
        .replaceAll('@(language)', describeLocale(locale.toString()))
        .replaceAll('@(baseLanguageClassName)', baseClassName)
        .replaceAll('@(class)', '$className${locale.camelCase()}')
        .replaceAll('@(localeName)', locale.toString())
        .replaceAll('@(methods)', methods.join('\n\n'));
  }

  // Generate the AppLocalizations class, its LocalizationsDelegate subclass,
  // and all AppLocalizations subclasses for every locale. This method by
  // itself does not generate the output files.
  String _generateCode() {
    bool isBaseClassLocale(LocaleInfo locale, String language) {
      return locale.languageCode == language &&
          locale.countryCode == null &&
          locale.scriptCode == null;
    }

    List<LocaleInfo> getLocalesForLanguage(String language) {
      return _allBundles.bundles
          // Return locales for the language specified, except for the base locale itself
          .where((AppResourceBundle bundle) {
            final LocaleInfo locale = bundle.locale;
            return !isBaseClassLocale(locale, language) && locale.languageCode == language;
          })
          .map((AppResourceBundle bundle) => bundle.locale)
          .toList();
    }

    final String directory = _fs.path.basename(outputDirectory.path);
    final String outputFileName = _fs.path.basename(baseOutputFile.path);
    if (!outputFileName.endsWith('.dart')) {
      throw L10nException(
        "The 'output-localization-file', $outputFileName, is invalid.\n"
        'The file name must have a .dart extension.',
      );
    }

    final Iterable<String> supportedLocalesCode = supportedLocales.map((LocaleInfo locale) {
      final String languageCode = locale.languageCode;
      final String? countryCode = locale.countryCode;
      final String? scriptCode = locale.scriptCode;

      if (countryCode == null && scriptCode == null) {
        return "Locale('$languageCode')";
      } else if (countryCode != null && scriptCode == null) {
        return "Locale('$languageCode', '$countryCode')";
      } else if (countryCode != null && scriptCode != null) {
        return "Locale.fromSubtags(languageCode: '$languageCode', countryCode: '$countryCode', scriptCode: '$scriptCode')";
      } else {
        return "Locale.fromSubtags(languageCode: '$languageCode', scriptCode: '$scriptCode')";
      }
    });

    final Set<String> supportedLanguageCodes = Set<String>.from(
      _allBundles.locales.map<String>((LocaleInfo locale) => "'${locale.languageCode}'"),
    );

    final List<LocaleInfo> allLocales = _allBundles.locales.toList()..sort();
    final int extensionIndex = outputFileName.indexOf('.');
    if (extensionIndex <= 0) {
      throw L10nException(
        "The 'output-localization-file', $outputFileName, is invalid.\n"
        'The base name cannot be empty.',
      );
    }
    final String fileName = outputFileName.substring(0, extensionIndex);
    final String fileExtension = outputFileName.substring(extensionIndex + 1);
    for (final LocaleInfo locale in allLocales) {
      if (isBaseClassLocale(locale, locale.languageCode)) {
        final File languageMessageFile = outputDirectory.childFile(
          '${fileName}_$locale.$fileExtension',
        );

        // Generate the template for the base class file. Further string
        // interpolation will be done to determine if there are
        // subclasses that extend the base class.
        final String languageBaseClassFile = _generateBaseClassFile(
          className,
          outputFileName,
          header,
          locale,
        );

        // Every locale for the language except the base class.
        final List<LocaleInfo> localesForLanguage = getLocalesForLanguage(locale.languageCode);

        // Generate every subclass that is needed for the particular language
        final Iterable<String> subclasses = localesForLanguage.map<String>((LocaleInfo locale) {
          return _generateSubclass(className, _allBundles.bundleFor(locale)!);
        });

        _languageFileMap.putIfAbsent(languageMessageFile, () {
          return languageBaseClassFile.replaceAll('@(subclasses)', subclasses.join());
        });
      }
    }

    final List<String> sortedClassImports =
        supportedLocales
            .where((LocaleInfo locale) => isBaseClassLocale(locale, locale.languageCode))
            .map((LocaleInfo locale) {
              final String library = '${fileName}_$locale';
              if (useDeferredLoading) {
                return "import '$library.$fileExtension' deferred as $library;";
              } else {
                return "import '$library.$fileExtension';";
              }
            })
            .toList()
          ..sort();

    final String delegateClass = _generateDelegateClass(
      allBundles: _allBundles,
      className: className,
      supportedLanguageCodes: supportedLanguageCodes,
      useDeferredLoading: useDeferredLoading,
      fileName: fileName,
    );

    return fileTemplate
        .replaceAll('@(header)', header.isEmpty ? '' : '$header\n')
        .replaceAll('@(class)', className)
        .replaceAll(
          '@(methods)',
          _allMessages
              .map(
                (Message message) =>
                    generateBaseClassMethod(message, _templateArbLocale, useNamedParameters),
              )
              .join('\n'),
        )
        .replaceAll('@(importFile)', '$directory/$outputFileName')
        .replaceAll('@(supportedLocales)', supportedLocalesCode.join(',\n    '))
        .replaceAll('@(supportedLanguageCodes)', supportedLanguageCodes.join(', '))
        .replaceAll('@(messageClassImports)', sortedClassImports.join('\n'))
        .replaceAll('@(delegateClass)', delegateClass)
        .replaceAll(
          '@(requiresFoundationImport)',
          useDeferredLoading ? '' : "import 'package:flutter/foundation.dart';",
        )
        .replaceAll('@(canBeNullable)', usesNullableGetter ? '?' : '')
        .replaceAll('@(needsNullCheck)', usesNullableGetter ? '' : '!')
        // Removes all trailing whitespace from the generated file.
        .split('\n')
        .map((String line) => line.trimRight())
        .join('\n')
        // Cleans out unnecessary newlines.
        .replaceAll('\n\n\n', '\n\n');
  }

  String _generateMethod(Message message, LocaleInfo locale) {
    try {
      final String translationForMessage = message.messages[locale]!;
      final Node node = message.parsedMessages[locale]!;
      // If the placeholders list is empty, then return a getter method.
      if (message.templatePlaceholders.isEmpty) {
        // Use the parsed translation to handle escaping with the same behavior.
        return getterTemplate
            .replaceAll('@(name)', message.resourceId)
            .replaceAll(
              '@(message)',
              "'${generateString(node.children.map((Node child) => child.value!).join())}'",
            );
      }

      final List<String> tempVariables = <String>[];
      // Get a unique temporary variable name.
      int variableCount = 0;
      String getTempVariableName() {
        return '_temp${variableCount++}';
      }

      // Do a DFS post order traversal through placeholderExpr, pluralExpr, and selectExpr nodes.
      // When traversing through a placeholderExpr node, return "$placeholderName".
      // When traversing through a pluralExpr node, return "$tempVarN" and add variable declaration in "tempVariables".
      // When traversing through a selectExpr node, return "$tempVarN" and add variable declaration in "tempVariables".
      // When traversing through an argumentExpr node, return "$tempVarN" and add variable declaration in "tempVariables".
      // When traversing through a message node, return concatenation of all of "generateVariables(child)" for each child.
      String generateVariables(Node node, {bool isRoot = false}) {
        switch (node.type) {
          case ST.message:
            final List<String> expressions =
                node.children.map<String>((Node node) {
                  if (node.type == ST.string) {
                    return generateString(node.value!);
                  }
                  return generateVariables(node);
                }).toList();
            return generateReturnExpr(expressions);

          case ST.placeholderExpr:
            assert(node.children[1].type == ST.identifier);
            final String identifier = node.children[1].value!;
            final Placeholder placeholder =
                message.localePlaceholders[locale]?[identifier] ??
                message.templatePlaceholders[identifier]!;
            if (placeholder.requiresFormatting) {
              return '\$${node.children[1].value}String';
            }
            return '\$${node.children[1].value}';

          case ST.pluralExpr:
            final Map<String, String> pluralLogicArgs = <String, String>{};
            // Recall that pluralExpr are of the form
            // pluralExpr := "{" ID "," "plural" "," pluralParts "}"
            assert(node.children[1].type == ST.identifier);
            assert(node.children[5].type == ST.pluralParts);

            final Node identifier = node.children[1];
            final Node pluralParts = node.children[5];

            for (final Node pluralPart in pluralParts.children.reversed) {
              String pluralCase;
              Node pluralMessage;
              if (pluralPart.children[0].value == '=') {
                assert(pluralPart.children[1].type == ST.number);
                assert(pluralPart.children[3].type == ST.message);
                pluralCase = pluralPart.children[1].value!;
                pluralMessage = pluralPart.children[3];
              } else {
                assert(
                  pluralPart.children[0].type == ST.identifier ||
                      pluralPart.children[0].type == ST.other,
                );
                assert(pluralPart.children[2].type == ST.message);
                pluralCase = pluralPart.children[0].value!;
                pluralMessage = pluralPart.children[2];
              }
              if (!pluralLogicArgs.containsKey(pluralCases[pluralCase])) {
                final String pluralPartExpression = generateVariables(pluralMessage);
                final String? transformedPluralCase = pluralCases[pluralCase];
                // A valid plural case is one of "=0", "=1", "=2", "zero", "one", "two", "few", "many", or "other".
                if (transformedPluralCase == null) {
                  throw L10nParserException(
                    '''
The plural cases must be one of "=0", "=1", "=2", "zero", "one", "two", "few", "many", or "other.
    $pluralCase is not a valid plural case.''',
                    _inputFileNames[locale]!,
                    message.resourceId,
                    translationForMessage,
                    pluralPart.positionInMessage,
                  );
                }
                pluralLogicArgs[transformedPluralCase] =
                    '      ${pluralCases[pluralCase]}: $pluralPartExpression,';
              } else if (!suppressWarnings) {
                logger.printWarning('''
[${_inputFileNames[locale]}:${message.resourceId}] ICU Syntax Warning: The plural part specified below is overridden by a later plural part.
    $translationForMessage
    ${Parser.indentForError(pluralPart.positionInMessage)}''');
              }
            }
            final String tempVarName = getTempVariableName();
            tempVariables.add(
              pluralVariableTemplate
                  .replaceAll('@(varName)', tempVarName)
                  .replaceAll('@(count)', identifier.value!)
                  .replaceAll('@(pluralLogicArgs)', pluralLogicArgs.values.join('\n')),
            );
            return '\$$tempVarName';

          case ST.selectExpr:
            // Recall that pluralExpr are of the form
            // pluralExpr := "{" ID "," "plural" "," pluralParts "}"
            assert(node.children[1].type == ST.identifier);
            assert(node.children[5].type == ST.selectParts);

            final Node identifier = node.children[1];
            final List<String> selectLogicArgs = <String>[];
            final Node selectParts = node.children[5];
            for (final Node selectPart in selectParts.children) {
              assert(
                selectPart.children[0].type == ST.identifier ||
                    selectPart.children[0].type == ST.other,
              );
              assert(selectPart.children[2].type == ST.message);
              final String selectCase = selectPart.children[0].value!;
              final Node selectMessage = selectPart.children[2];
              final String selectPartExpression = generateVariables(selectMessage);
              selectLogicArgs.add("        '$selectCase': $selectPartExpression,");
            }
            final String tempVarName = getTempVariableName();
            tempVariables.add(
              selectVariableTemplate
                  .replaceAll('@(varName)', tempVarName)
                  .replaceAll('@(choice)', identifier.value!)
                  .replaceAll('@(selectCases)', selectLogicArgs.join('\n')),
            );
            return '\$$tempVarName';
          case ST.argumentExpr:
            assert(node.children[1].type == ST.identifier);
            assert(node.children[3].type == ST.argType);
            assert(node.children[7].type == ST.identifier);
            final String identifierName = node.children[1].value!;
            final Node formatType = node.children[7];
            // Check that formatType is a valid intl.DateFormat.
            if (!validDateFormats.contains(formatType.value)) {
              throw L10nParserException(
                'Date format "${formatType.value!}" for placeholder '
                '$identifierName does not have a corresponding DateFormat '
                "constructor\n. Check the intl library's DateFormat class "
                'constructors for allowed date formats, or set "isCustomDateFormat" attribute '
                'to "true".',
                _inputFileNames[locale]!,
                message.resourceId,
                translationForMessage,
                formatType.positionInMessage,
              );
            }
            final String tempVarName = getTempVariableName();
            tempVariables.add(
              dateVariableTemplate
                  .replaceAll('@(varName)', tempVarName)
                  .replaceAll('@(formatType)', formatType.value!)
                  .replaceAll('@(argument)', identifierName),
            );
            return '\$$tempVarName';
          // ignore: no_default_cases
          default:
            throw Exception('Cannot call "generateHelperMethod" on node type ${node.type}');
        }
      }

      final String messageString = generateVariables(node, isRoot: true);
      final String tempVarLines = tempVariables.isEmpty ? '' : '${tempVariables.join('\n')}\n';
      return (useNamedParameters ? methodWithNamedParameterTemplate : methodTemplate)
          .replaceAll('@(name)', message.resourceId)
          .replaceAll(
            '@(parameters)',
            generateMethodParameters(message, locale, useNamedParameters).join(', '),
          )
          .replaceAll('@(dateFormatting)', generateDateFormattingLogic(message, locale))
          .replaceAll('@(numberFormatting)', generateNumberFormattingLogic(message, locale))
          .replaceAll('@(tempVars)', tempVarLines)
          .replaceAll('@(message)', messageString)
          .replaceAll('@(none)\n', '');
    } on L10nParserException catch (error) {
      logger.printError(error.toString());
      hadErrors = true;
      return '';
    }
  }

  List<String> writeOutputFiles({bool isFromYaml = false, bool useCRLF = false}) {
    // First, generate the string contents of all necessary files.
    final String generatedLocalizationsFile = _generateCode();

    // If there were any syntax errors, don't write to files.
    if (hadErrors) {
      throw L10nException('Found syntax errors.');
    }

    // A pubspec.yaml file is required when using a synthetic package. If it does not
    // exist, create a blank one.
    if (useSyntheticPackage) {
      final Directory syntheticPackageDirectory =
          projectDirectory != null
              ? projectDirectory!.childDirectory(_defaultSyntheticPackagePath(_fs))
              : _fs.directory(_defaultSyntheticPackagePath(_fs));
      syntheticPackageDirectory.createSync(recursive: true);
      final File flutterGenPubspec = syntheticPackageDirectory.childFile('pubspec.yaml');
      if (!flutterGenPubspec.existsSync()) {
        flutterGenPubspec.writeAsStringSync(
          useCRLF ? emptyPubspecTemplate.replaceAll('\n', '\r\n') : emptyPubspecTemplate,
        );
      }
    }

    // Since all validity checks have passed up to this point,
    // write the contents into the directory.
    outputDirectory.createSync(recursive: true);

    // Ensure that the created directory has read/write permissions.
    final FileStat fileStat = outputDirectory.statSync();
    if (_isNotReadable(fileStat) || _isNotWritable(fileStat)) {
      throw L10nException(
        "The 'output-dir' directory, $outputDirectory, doesn't allow reading and writing.\n"
        'Please ensure that the user has read and write permissions.',
      );
    }

    // Generate the required files for localizations.
    _languageFileMap.forEach((File file, String contents) {
      file.writeAsStringSync(useCRLF ? contents.replaceAll('\n', '\r\n') : contents);
      _outputFileList.add(file.absolute.path);
    });

    baseOutputFile.writeAsStringSync(
      useCRLF ? generatedLocalizationsFile.replaceAll('\n', '\r\n') : generatedLocalizationsFile,
    );
    final File? messagesFile = untranslatedMessagesFile;
    if (messagesFile != null) {
      _generateUntranslatedMessagesFile(logger, messagesFile);
    } else if (_unimplementedMessages.isNotEmpty) {
      _unimplementedMessages.forEach((LocaleInfo locale, List<String> messages) {
        logger.printStatus('"$locale": ${messages.length} untranslated message(s).');
      });
      if (isFromYaml) {
        logger.printStatus(
          'To see a detailed report, use the untranslated-messages-file \n'
          'option in the l10n.yaml file:\n'
          'untranslated-messages-file: desiredFileName.txt\n'
          '<other option>: <other selection> \n\n',
        );
      } else {
        logger.printStatus(
          'To see a detailed report, use the --untranslated-messages-file \n'
          'option in the flutter gen-l10n tool:\n'
          'flutter gen-l10n --untranslated-messages-file=desiredFileName.txt\n'
          '<other options> \n\n',
        );
      }

      logger.printStatus(
        'This will generate a JSON format file containing all messages that \n'
        'need to be translated.',
      );
    }
    final File? inputsAndOutputsListFileLocal = inputsAndOutputsListFile;
    _outputFileList.add(baseOutputFile.absolute.path);
    if (inputsAndOutputsListFileLocal != null) {
      // Generate a JSON file containing the inputs and outputs of the gen_l10n script.
      if (!inputsAndOutputsListFileLocal.existsSync()) {
        inputsAndOutputsListFileLocal.createSync(recursive: true);
      }
      final String filesListContent = json.encode(<String, Object>{
        'inputs': _inputFileList,
        'outputs': _outputFileList,
      });
      inputsAndOutputsListFileLocal.writeAsStringSync(
        useCRLF ? filesListContent.replaceAll('\n', '\r\n') : filesListContent,
      );
    }

    return _outputFileList;
  }

  void _generateUntranslatedMessagesFile(Logger logger, File untranslatedMessagesFile) {
    if (_unimplementedMessages.isEmpty) {
      untranslatedMessagesFile.writeAsStringSync('{}');
      _outputFileList.add(untranslatedMessagesFile.absolute.path);
      return;
    }

    String resultingFile = '{\n';
    int count = 0;
    final int numberOfLocales = _unimplementedMessages.length;
    _unimplementedMessages.forEach((LocaleInfo locale, List<String> messages) {
      resultingFile += '  "$locale": [\n';

      for (int i = 0; i < messages.length; i += 1) {
        resultingFile += '    "${messages[i]}"';
        if (i != messages.length - 1) {
          resultingFile += ',';
        }
        resultingFile += '\n';
      }

      resultingFile += '  ]';
      count += 1;
      if (count < numberOfLocales) {
        resultingFile += ',\n';
      }
      resultingFile += '\n';
    });

    resultingFile += '}\n';
    untranslatedMessagesFile.writeAsStringSync(resultingFile);
    _outputFileList.add(untranslatedMessagesFile.absolute.path);
  }
}
