// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:intl/locale.dart';
import 'package:meta/meta.dart';
import 'package:yaml/yaml.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../convert.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';
import 'gen_l10n_types.dart';
import 'language_subtag_registry.dart';

typedef HeaderGenerator = String Function(String regenerateInstructions);
typedef ConstructorGenerator = String Function(LocaleInfo locale);

int sortFilesByPath (File a, File b) {
  return a.path.compareTo(b.path);
}

/// Simple data class to hold parsed locale. Does not promise validity of any data.
@immutable
class LocaleInfo implements Comparable<LocaleInfo> {
  const LocaleInfo({
    required this.languageCode,
    required this.scriptCode,
    required this.countryCode,
    required this.length,
    required this.originalString,
  });

  /// Simple parser. Expects the locale string to be in the form of 'language_script_COUNTRY'
  /// where the language is 2 characters, script is 4 characters with the first uppercase,
  /// and country is 2-3 characters and all uppercase.
  ///
  /// 'language_COUNTRY' or 'language_script' are also valid. Missing fields will be null.
  ///
  /// When `deriveScriptCode` is true, if [scriptCode] was unspecified, it will
  /// be derived from the [languageCode] and [countryCode] if possible.
  factory LocaleInfo.fromString(String locale, { bool deriveScriptCode = false }) {
    final List<String> codes = locale.split('_'); // [language, script, country]
    assert(codes.isNotEmpty && codes.length < 4);
    final String languageCode = codes[0];
    String? scriptCode;
    String? countryCode;
    int length = codes.length;
    String originalString = locale;
    if (codes.length == 2) {
      scriptCode = codes[1].length >= 4 ? codes[1] : null;
      countryCode = codes[1].length < 4 ? codes[1] : null;
    } else if (codes.length == 3) {
      scriptCode = codes[1].length > codes[2].length ? codes[1] : codes[2];
      countryCode = codes[1].length < codes[2].length ? codes[1] : codes[2];
    }
    assert(codes[0].isNotEmpty);
    assert(countryCode == null || countryCode.isNotEmpty);
    assert(scriptCode == null || scriptCode.isNotEmpty);

    /// Adds scriptCodes to locales where we are able to assume it to provide
    /// finer granularity when resolving locales.
    ///
    /// The basis of the assumptions here are based off of known usage of scripts
    /// across various countries. For example, we know Taiwan uses traditional (Hant)
    /// script, so it is safe to apply (Hant) to Taiwanese languages.
    if (deriveScriptCode && scriptCode == null) {
      scriptCode = switch ((languageCode, countryCode)) {
        ('zh', 'CN' || 'SG' || null) => 'Hans',
        ('zh', 'TW' || 'HK' || 'MO') => 'Hant',
        ('sr', null) => 'Cyrl',
        _ => null,
      };
      // Increment length if we were able to assume a scriptCode.
      if (scriptCode != null) {
        length += 1;
      }
      // Update the base string to reflect assumed scriptCodes.
      originalString = languageCode;
      if (scriptCode != null) {
        originalString += '_$scriptCode';
      }
      if (countryCode != null) {
        originalString += '_$countryCode';
      }
    }

    return LocaleInfo(
      languageCode: languageCode,
      scriptCode: scriptCode,
      countryCode: countryCode,
      length: length,
      originalString: originalString,
    );
  }

  final String languageCode;
  final String? scriptCode;
  final String? countryCode;
  final int length;             // The number of fields. Ranges from 1-3.
  final String originalString;  // Original un-parsed locale string.

  String camelCase() {
    return originalString
      .split('_')
      .map<String>((String part) => part.substring(0, 1).toUpperCase() + part.substring(1).toLowerCase())
      .join();
  }

  @override
  bool operator ==(Object other) {
    return other is LocaleInfo
        && other.originalString == originalString;
  }

  @override
  int get hashCode => originalString.hashCode;

  @override
  String toString() {
    return originalString;
  }

  @override
  int compareTo(LocaleInfo other) {
    return originalString.compareTo(other.originalString);
  }
}

// See also //master/tools/gen_locale.dart in the engine repo.
Map<String, List<String>> _parseSection(String section) {
  final Map<String, List<String>> result = <String, List<String>>{};
  late List<String> lastHeading;
  for (final String line in section.split('\n')) {
    if (line == '') {
      continue;
    }
    if (line.startsWith('  ')) {
      lastHeading[lastHeading.length - 1] = '${lastHeading.last}${line.substring(1)}';
      continue;
    }
    final int colon = line.indexOf(':');
    if (colon <= 0) {
      throw Exception('not sure how to deal with "$line"');
    }
    final String name = line.substring(0, colon);
    final String value = line.substring(colon + 2);
    lastHeading = result.putIfAbsent(name, () => <String>[]);
    result[name]!.add(value);
  }
  return result;
}

final Map<String, String> _languages = <String, String>{};
final Map<String, String> _regions = <String, String>{};
final Map<String, String> _scripts = <String, String>{};
const String kProvincePrefix = ', Province of ';
const String kParentheticalPrefix = ' (';

/// Prepares the data for the [describeLocale] method below.
///
/// The data is obtained from the official IANA registry.
void precacheLanguageAndRegionTags() {
  final List<Map<String, List<String>>> sections =
      languageSubtagRegistry.split('%%').skip(1).map<Map<String, List<String>>>(_parseSection).toList();
  for (final Map<String, List<String>> section in sections) {
    assert(section.containsKey('Type'), section.toString());
    final String type = section['Type']!.single;
    if (type == 'language' || type == 'region' || type == 'script') {
      assert(section.containsKey('Subtag') && section.containsKey('Description'), section.toString());
      final String subtag = section['Subtag']!.single;
      String description = section['Description']!.join(' ');
      if (description.startsWith('United ')) {
        description = 'the $description';
      }
      if (description.contains(kParentheticalPrefix)) {
        description = description.substring(0, description.indexOf(kParentheticalPrefix));
      }
      if (description.contains(kProvincePrefix)) {
        description = description.substring(0, description.indexOf(kProvincePrefix));
      }
      if (description.endsWith(' Republic')) {
        description = 'the $description';
      }
      switch (type) {
        case 'language':
          _languages[subtag] = description;
        case 'region':
          _regions[subtag] = description;
        case 'script':
          _scripts[subtag] = description;
      }
    }
  }
}

String describeLocale(String tag) {
  final List<String> subtags = tag.split('_');
  assert(subtags.isNotEmpty);
  final String languageCode = subtags[0];
  if (!_languages.containsKey(languageCode)) {
    throw L10nException(
      '"$languageCode" is not a supported language code.\n'
      'See https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry '
      'for the supported list.',
    );
  }
  final String language = _languages[languageCode]!;
  String output = language;
  String? region;
  String? script;
  if (subtags.length == 2) {
    region = _regions[subtags[1]];
    script = _scripts[subtags[1]];
    assert(region != null || script != null);
  } else if (subtags.length >= 3) {
    region = _regions[subtags[2]];
    script = _scripts[subtags[1]];
    assert(region != null && script != null);
  }
  if (region != null) {
    output += ', as used in $region';
  }
  if (script != null) {
    output += ', using the $script script';
  }
  return output;
}

/// Return the input string as a Dart-parsable string.
///
/// ```none
/// foo => 'foo'
/// foo "bar" => 'foo "bar"'
/// foo 'bar' => "foo 'bar'"
/// foo 'bar' "baz" => '''foo 'bar' "baz"'''
/// ```
///
/// This function is used by tools that take in a JSON-formatted file to
/// generate Dart code. For this reason, characters with special meaning
/// in JSON files are escaped. For example, the backspace character (\b)
/// has to be properly escaped by this function so that the generated
/// Dart code correctly represents this character:
/// ```none
/// foo\bar => 'foo\\bar'
/// foo\nbar => 'foo\\nbar'
/// foo\\nbar => 'foo\\\\nbar'
/// foo\\bar => 'foo\\\\bar'
/// foo\ bar => 'foo\\ bar'
/// foo$bar = 'foo\$bar'
/// ```
String generateString(String value) {
  const String backslash = '__BACKSLASH__';
  assert(
    !value.contains(backslash),
    'Input string cannot contain the sequence: '
    '"__BACKSLASH__", as it is used as part of '
    'backslash character processing.'
  );

  value = value
    // Replace backslashes with a placeholder for now to properly parse
    // other special characters.
    .replaceAll(r'\', backslash)
    .replaceAll(r'$', r'\$')
    .replaceAll("'", r"\'")
    .replaceAll('"', r'\"')
    .replaceAll('\n', r'\n')
    .replaceAll('\f', r'\f')
    .replaceAll('\t', r'\t')
    .replaceAll('\r', r'\r')
    .replaceAll('\b', r'\b')
    // Reintroduce escaped backslashes into generated Dart string.
    .replaceAll(backslash, r'\\');

  return value;
}

/// Given a list of normal strings or interpolated variables, concatenate them
/// into a single dart string to be returned. An example of a normal string
/// would be "'Hello world!'" and an example of a interpolated variable would be
/// "'$placeholder'".
///
/// Each of the strings in [expressions] should be a raw string, which, if it
/// were to be added to a dart file, would be a properly formatted dart string
/// with escapes and/or interpolation. The purpose of this function is to
/// concatenate these dart strings into a single dart string which can be
/// returned in the generated localization files.
///
/// The following rules describe the kinds of string expressions that can be
/// handled:
/// 1. If [expressions] is empty, return the empty string "''".
/// 2. If [expressions] has only one [String] which is an interpolated variable,
///    it is converted to the variable itself e.g. ["'$expr'"] -> "expr".
/// 3. If one string in [expressions] is an interpolation and the next begins
///    with an alphanumeric character, then the former interpolation should be
///    wrapped in braces e.g. ["'$expr1'", "'another'"] -> "'${expr1}another'".
String generateReturnExpr(List<String> expressions, { bool isSingleStringVar = false }) {
  if (expressions.isEmpty) {
    return "''";
  } else if (isSingleStringVar) {
    // If our expression is "$varName" where varName is a String, this is equivalent to just varName.
    return expressions[0].substring(1);
  } else {
    final String string = expressions.reversed.fold<String>('', (String string, String expression) {
      if (expression[0] != r'$') {
        return expression + string;
      }
      final RegExp alphanumeric = RegExp(r'^([0-9a-zA-Z]|_)+$');
      if (alphanumeric.hasMatch(expression.substring(1)) && !(string.isNotEmpty && alphanumeric.hasMatch(string[0]))) {
        return '$expression$string';
      } else {
        return '\${${expression.substring(1)}}$string';
      }
    });
    return "'$string'";
  }
}

/// Typed configuration from the localizations config file.
class LocalizationOptions {
  LocalizationOptions({
    required this.arbDir,
    this.outputDir,
    String? templateArbFile,
    String? templateLocale,
    String? outputLocalizationFile,
    this.untranslatedMessagesFile,
    String? outputClass,
    this.preferredSupportedLocales,
    this.header,
    this.headerFile,
    bool? useDeferredLoading,
    this.genInputsAndOutputsList,
    bool? syntheticPackage,
    this.projectDir,
    bool? requiredResourceAttributes,
    bool? nullableGetter,
    bool? format,
    bool? useEscaping,
    bool? suppressWarnings,
    bool? relaxSyntax,
    bool? useNamedParameters,
  }) : templateArbFile = templateArbFile ?? 'app_en.arb',
       templateLocale = templateLocale ?? 'en',
       outputLocalizationFile = outputLocalizationFile ?? 'app_localizations.dart',
       outputClass = outputClass ?? 'AppLocalizations',
       useDeferredLoading = useDeferredLoading ?? false,
       syntheticPackage = syntheticPackage ?? true,
       requiredResourceAttributes = requiredResourceAttributes ?? false,
       nullableGetter = nullableGetter ?? true,
       format = format ?? false,
       useEscaping = useEscaping ?? false,
       suppressWarnings = suppressWarnings ?? false,
       relaxSyntax = relaxSyntax ?? false,
       useNamedParameters = useNamedParameters ?? false;

  /// The `--arb-dir` argument.
  ///
  /// The directory where all input localization files should reside.
  final String arbDir;

  /// The `--output-dir` argument.
  ///
  /// The directory where all output localization files should be generated.
  final String? outputDir;

  /// The `--template-arb-file` argument.
  ///
  /// This path is relative to [arbDirectory].
  @Deprecated(
    'Use `templateLocale` instead. '
    'Deprecated in favor of `templateLocale` since we can now have multiple localization files for the same locale. '
    'This feature was deprecated after v3.27.0-1.0.pre.',
  )
  final String templateArbFile;

  /// The `--template-locale` argument.
  ///
  /// The locale that should be used as the basis for generating the files
  final String templateLocale;

  /// The `--output-localization-file` argument.
  ///
  /// This path is relative to [arbDir].
  final String outputLocalizationFile;

  /// The `--untranslated-messages-file` argument.
  ///
  /// This path is relative to [arbDir].
  final String? untranslatedMessagesFile;

  /// The `--output-class` argument.
  final String outputClass;

  /// The `--preferred-supported-locales` argument.
  final List<String>? preferredSupportedLocales;

  /// The `--header` argument.
  ///
  /// The header to prepend to the generated Dart localizations.
  final String? header;

  /// The `--header-file` argument.
  ///
  /// A file containing the header to prepend to the generated
  /// Dart localizations.
  final String? headerFile;

  /// The `--use-deferred-loading` argument.
  ///
  /// Whether to generate the Dart localization file with locales imported
  /// as deferred.
  final bool useDeferredLoading;

  /// The `--gen-inputs-and-outputs-list` argument.
  ///
  /// This path is relative to [arbDir].
  final String? genInputsAndOutputsList;

  /// The `--synthetic-package` argument.
  ///
  /// Whether to generate the Dart localization files in a synthetic package
  /// or in a custom directory.
  final bool syntheticPackage;

  /// The `--project-dir` argument.
  ///
  /// This path is relative to [arbDir].
  final String? projectDir;

  /// The `required-resource-attributes` argument.
  ///
  /// Whether to require all resource ids to contain a corresponding
  /// resource attribute.
  final bool requiredResourceAttributes;

  /// The `nullable-getter` argument.
  ///
  /// Whether or not the localizations class getter is nullable.
  final bool nullableGetter;

  /// The `format` argument.
  ///
  /// Whether or not to format the generated files.
  final bool format;

  /// The `use-escaping` argument.
  ///
  /// Whether or not the ICU escaping syntax is used.
  final bool useEscaping;

  /// The `suppress-warnings` argument.
  ///
  /// Whether or not to suppress warnings.
  final bool suppressWarnings;

  /// The `relax-syntax` argument.
  ///
  /// Whether or not to relax the syntax. When specified, the syntax will be
  /// relaxed so that the special character "{" is treated as a string if it is
  /// not followed by a valid placeholder and "}" is treated as a string if it
  /// does not close any previous "{" that is treated as a special character.
  /// This was added in for backward compatibility and is not recommended
  /// as it may mask errors.
  final bool relaxSyntax;

  /// The `use-named-parameters` argument.
  ///
  /// Whether or not to use named parameters for the generated localization
  /// methods.
  ///
  /// Defaults to `false`.
  final bool useNamedParameters;
}

/// Parse the localizations configuration options from [file].
///
/// Throws [Exception] if any of the contents are invalid. Returns a
/// [LocalizationOptions] with all fields as `null` if the config file exists
/// but is empty.
LocalizationOptions parseLocalizationsOptionsFromYAML({
  required File file,
  required Logger logger,
  required String defaultArbDir,
  required bool defaultSyntheticPackage,
}) {
  final String contents = file.readAsStringSync();
  if (contents.trim().isEmpty) {
    return LocalizationOptions(arbDir: defaultArbDir);
  }
  final YamlNode yamlNode;
  try {
    yamlNode = loadYamlNode(file.readAsStringSync());
  } on YamlException catch (err) {
    throwToolExit(err.message);
  }
  if (yamlNode is! YamlMap) {
    logger.printError('Expected ${file.path} to contain a map, instead was $yamlNode');
    throw Exception();
  }
  return LocalizationOptions(
    arbDir: _tryReadUri(yamlNode, 'arb-dir', logger)?.path ?? defaultArbDir,
    outputDir: _tryReadUri(yamlNode, 'output-dir', logger)?.path,
    templateArbFile: _tryReadUri(yamlNode, 'template-arb-file', logger)?.path,
    templateLocale: _tryReadUri(yamlNode, 'template-locale', logger)?.path,
    outputLocalizationFile: _tryReadUri(yamlNode, 'output-localization-file', logger)?.path,
    untranslatedMessagesFile: _tryReadUri(yamlNode, 'untranslated-messages-file', logger)?.path,
    outputClass: _tryReadString(yamlNode, 'output-class', logger),
    header: _tryReadString(yamlNode, 'header', logger),
    headerFile: _tryReadUri(yamlNode, 'header-file', logger)?.path,
    useDeferredLoading: _tryReadBool(yamlNode, 'use-deferred-loading', logger),
    preferredSupportedLocales: _tryReadStringList(yamlNode, 'preferred-supported-locales', logger),
    syntheticPackage: _tryReadBool(yamlNode, 'synthetic-package', logger) ?? defaultSyntheticPackage,
    requiredResourceAttributes: _tryReadBool(yamlNode, 'required-resource-attributes', logger),
    nullableGetter: _tryReadBool(yamlNode, 'nullable-getter', logger),
    format: _tryReadBool(yamlNode, 'format', logger),
    useEscaping: _tryReadBool(yamlNode, 'use-escaping', logger),
    suppressWarnings: _tryReadBool(yamlNode, 'suppress-warnings', logger),
    relaxSyntax: _tryReadBool(yamlNode, 'relax-syntax', logger),
    useNamedParameters: _tryReadBool(yamlNode, 'use-named-parameters', logger),
  );
}

/// Parse the localizations configuration from [FlutterCommand].
LocalizationOptions parseLocalizationsOptionsFromCommand({
  required FlutterCommand command,
  required String defaultArbDir,
}) {
  // TODO(matanlurey): Remove as part of https://github.com/flutter/flutter/issues/102983.
  final bool syntheticPackage;
  if (command.argResults!.wasParsed('synthetic-package')) {
    // If provided explicitly, use the explicit value.
    syntheticPackage = command.boolArg('synthetic-package');
  } else {
    // Otherwise, inherit from whatever the default of --implicit-pubspec-resolution is.
    syntheticPackage = command.globalResults!.flag(FlutterGlobalOptions.kImplicitPubspecResolution);
  }
  return LocalizationOptions(
    arbDir: command.stringArg('arb-dir') ?? defaultArbDir,
    outputDir: command.stringArg('output-dir'),
    outputLocalizationFile: command.stringArg('output-localization-file'),
    templateArbFile: command.stringArg('template-arb-file'),
    templateLocale: command.stringArg('template-locale'),
    untranslatedMessagesFile: command.stringArg('untranslated-messages-file'),
    outputClass: command.stringArg('output-class'),
    header: command.stringArg('header'),
    headerFile: command.stringArg('header-file'),
    useDeferredLoading: command.boolArg('use-deferred-loading'),
    genInputsAndOutputsList: command.stringArg('gen-inputs-and-outputs-list'),
    syntheticPackage: syntheticPackage,
    projectDir: command.stringArg('project-dir'),
    requiredResourceAttributes: command.boolArg('required-resource-attributes'),
    nullableGetter: command.boolArg('nullable-getter'),
    format: command.boolArg('format'),
    useEscaping: command.boolArg('use-escaping'),
    suppressWarnings: command.boolArg('suppress-warnings'),
    useNamedParameters: command.boolArg('use-named-parameters'),
  );
}

// Try to read a `bool` value or null from `yamlMap`, otherwise throw.
bool? _tryReadBool(YamlMap yamlMap, String key, Logger logger) {
  final Object? value = yamlMap[key];
  if (value == null) {
    return null;
  }
  if (value is! bool) {
    logger.printError('Expected "$key" to have a bool value, instead was "$value"');
    throw Exception();
  }
  return value;
}

// Try to read a `String` value or null from `yamlMap`, otherwise throw.
String? _tryReadString(YamlMap yamlMap, String key, Logger logger) {
  final Object? value = yamlMap[key];
  if (value == null) {
    return null;
  }
  if (value is! String) {
    logger.printError('Expected "$key" to have a String value, instead was "$value"');
    throw Exception();
  }
  return value;
}

List<String>? _tryReadStringList(YamlMap yamlMap, String key, Logger logger) {
  final Object? value = yamlMap[key];
  if (value == null) {
    return null;
  }
  if (value is String) {
    return <String>[value];
  }
  if (value is Iterable) {
    return value.map((dynamic e) => e.toString()).toList();
  }
  logger.printError('"$value" must be String or List.');
  throw Exception();
}

// Try to read a valid `Uri` or null from `yamlMap`, otherwise throw.
Uri? _tryReadUri(YamlMap yamlMap, String key, Logger logger) {
  final String? value = _tryReadString(yamlMap, key, logger);
  if (value == null) {
    return null;
  }
  final Uri? uri = Uri.tryParse(value);
  if (uri == null) {
    logger.printError('"$value" must be a relative file URI');
  }
  return uri;
}

Map<String, Object?> parseJsonFile(File file) {
  try {
    final String content = file.readAsStringSync().trim();
    if (content.isEmpty) {
      return <String, Object?>{};
    }
    return json.decode(content) as Map<String, Object?>;
  } on FormatException catch (e) {
    throw L10nException(
      'The arb file ${file.path} has the following formatting issue: \n'
      '$e',
    );
  }
}

LocaleInfo localeInfoFromFile(File file, {Map<String, Object?>? cachedResources}) {
  final Map<String, Object?> resources = cachedResources ?? parseJsonFile(file);
  final LocaleInfo? resourcesLocaleInfo = localeInfoFromResources(resources);
  final LocaleInfo? fileNameLocaleInfo = localeInfoFromFileName(file);

  switch ((fileNameLocaleInfo, resourcesLocaleInfo)) {
    case (null, null):
      throw L10nException(
        "The following .arb file's locale could not be determined: \n"
        '${file.path} \n'
        "Make sure that the locale is specified in the file's '@@locale' "
        'property or as part of the filename (e.g. file_en.arb)'
      );
    case (LocaleInfo(), LocaleInfo()) when fileNameLocaleInfo != resourcesLocaleInfo:
      throw L10nException(
        'The locale specified in @@locale and the arb filename do not match. \n'
        'Please make sure that they match, since this prevents any confusion \n'
        'with which locale to use. Otherwise, specify the locale in either the \n'
        'filename of the @@locale key only.\n'
        'Current @@locale value: $resourcesLocaleInfo\n'
        'Current filename extension: $fileNameLocaleInfo'
      );
    default:
      return fileNameLocaleInfo ?? resourcesLocaleInfo!;
  }
}

LocaleInfo? localeInfoFromResources(Map<String, Object?> resources) {
  final String? localeString = resources['@@locale'] as String?;

  if (localeString == null) {
    return null;
  }

  return LocaleInfo.fromString(localeString);
}

// Look for the first instance of an ISO 639-1 language code, matching exactly.
LocaleInfo? localeInfoFromFileName(File file) {
  final String fileName = file.fileSystem.path.basenameWithoutExtension(file.path);

  for (int index = 0; index < fileName.length; index += 1) {
    // If an underscore was found, check if locale string follows.
    if (fileName[index] == '_') {
      // If Locale.tryParse fails, it returns null.
      final Locale? parserResult = Locale.tryParse(fileName.substring(index + 1));
      // If the parserResult is not an actual locale identifier, end the loop.
      if (parserResult != null && _iso639Languages.contains(parserResult.languageCode)) {
        // The parsed result uses dashes ('-'), but we want underscores ('_').
        final String parserLocaleString = parserResult.toString().replaceAll('-', '_');
        return LocaleInfo.fromString(parserLocaleString);
      }
    }
  }

  return null;
}

// A set containing all the ISO630-1 languages. This list was pulled from https://datahub.io/core/language-codes.
final Set<String> _iso639Languages = <String>{
  'aa',
  'ab',
  'ae',
  'af',
  'ak',
  'am',
  'an',
  'ar',
  'as',
  'av',
  'ay',
  'az',
  'ba',
  'be',
  'bg',
  'bh',
  'bi',
  'bm',
  'bn',
  'bo',
  'br',
  'bs',
  'ca',
  'ce',
  'ch',
  'co',
  'cr',
  'cs',
  'cu',
  'cv',
  'cy',
  'da',
  'de',
  'dv',
  'dz',
  'ee',
  'el',
  'en',
  'eo',
  'es',
  'et',
  'eu',
  'fa',
  'ff',
  'fi',
  'fil',
  'fj',
  'fo',
  'fr',
  'fy',
  'ga',
  'gd',
  'gl',
  'gn',
  'gsw',
  'gu',
  'gv',
  'ha',
  'he',
  'hi',
  'ho',
  'hr',
  'ht',
  'hu',
  'hy',
  'hz',
  'ia',
  'id',
  'ie',
  'ig',
  'ii',
  'ik',
  'io',
  'is',
  'it',
  'iu',
  'ja',
  'jv',
  'ka',
  'kg',
  'ki',
  'kj',
  'kk',
  'kl',
  'km',
  'kn',
  'ko',
  'kr',
  'ks',
  'ku',
  'kv',
  'kw',
  'ky',
  'la',
  'lb',
  'lg',
  'li',
  'ln',
  'lo',
  'lt',
  'lu',
  'lv',
  'mg',
  'mh',
  'mi',
  'mk',
  'ml',
  'mn',
  'mr',
  'ms',
  'mt',
  'my',
  'na',
  'nb',
  'nd',
  'ne',
  'ng',
  'nl',
  'nn',
  'no',
  'nr',
  'nv',
  'ny',
  'oc',
  'oj',
  'om',
  'or',
  'os',
  'pa',
  'pi',
  'pl',
  'ps',
  'pt',
  'qu',
  'rm',
  'rn',
  'ro',
  'ru',
  'rw',
  'sa',
  'sc',
  'sd',
  'se',
  'sg',
  'si',
  'sk',
  'sl',
  'sm',
  'sn',
  'so',
  'sq',
  'sr',
  'ss',
  'st',
  'su',
  'sv',
  'sw',
  'ta',
  'te',
  'tg',
  'th',
  'ti',
  'tk',
  'tl',
  'tn',
  'to',
  'tr',
  'ts',
  'tt',
  'tw',
  'ty',
  'ug',
  'uk',
  'ur',
  'uz',
  've',
  'vi',
  'vo',
  'wa',
  'wo',
  'xh',
  'yi',
  'yo',
  'za',
  'zh',
  'zu',
};
