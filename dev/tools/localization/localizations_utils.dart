// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart' as argslib;
import 'package:meta/meta.dart';

import 'language_subtag_registry.dart';

typedef HeaderGenerator = String Function(String regenerateInstructions);
typedef ConstructorGenerator = String Function(LocaleInfo locale);

int sortFilesByPath(FileSystemEntity a, FileSystemEntity b) {
  return a.path.compareTo(b.path);
}

/// Simple data class to hold parsed locale. Does not promise validity of any data.
@immutable
class LocaleInfo implements Comparable<LocaleInfo> {
  const LocaleInfo({
    required this.languageCode,
    this.scriptCode,
    this.countryCode,
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
  factory LocaleInfo.fromString(String locale, {bool deriveScriptCode = false}) {
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
  final int length; // The number of fields. Ranges from 1-3.
  final String originalString; // Original un-parsed locale string.

  String camelCase() {
    return originalString
        .split('_')
        .map<String>(
          (String part) => part.substring(0, 1).toUpperCase() + part.substring(1).toLowerCase(),
        )
        .join();
  }

  @override
  bool operator ==(Object other) {
    return other is LocaleInfo && other.originalString == originalString;
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

/// Parse the data for a locale from a file, and store it in the [attributes]
/// and [resources] keys.
void loadMatchingArbsIntoBundleMaps({
  required Directory directory,
  required RegExp filenamePattern,
  required Map<LocaleInfo, Map<String, String>> localeToResources,
  required Map<LocaleInfo, Map<String, dynamic>> localeToResourceAttributes,
}) {
  /// Set that holds the locales that were assumed from the existing locales.
  ///
  /// For example, when the data lacks data for zh_Hant, we will use the data of
  /// the first Hant Chinese locale as a default by repeating the data. If an
  /// explicit match is later found, we can reference this set to see if we should
  /// overwrite the existing assumed data.
  final Set<LocaleInfo> assumedLocales = <LocaleInfo>{};

  for (final FileSystemEntity entity in directory.listSync().toList()..sort(sortFilesByPath)) {
    final String entityPath = entity.path;
    if (FileSystemEntity.isFileSync(entityPath) && filenamePattern.hasMatch(entityPath)) {
      final String localeString = filenamePattern.firstMatch(entityPath)![1]!;
      final File arbFile = File(entityPath);

      // Helper method to fill the maps with the correct data from file.
      void populateResources(LocaleInfo locale, File file) {
        final Map<String, String> resources = localeToResources[locale]!;
        final Map<String, dynamic> attributes = localeToResourceAttributes[locale]!;
        final Map<String, dynamic> bundle =
            json.decode(file.readAsStringSync()) as Map<String, dynamic>;
        for (final String key in bundle.keys) {
          // The ARB file resource "attributes" for foo are called @foo.
          if (key.startsWith('@')) {
            attributes[key.substring(1)] = bundle[key];
          } else {
            resources[key] = bundle[key] as String;
          }
        }
      }

      // Only pre-assume scriptCode if there is a country or script code to assume off of.
      // When we assume scriptCode based on languageCode-only, we want this initial pass
      // to use the un-assumed version as a base class.
      LocaleInfo locale = LocaleInfo.fromString(
        localeString,
        deriveScriptCode: localeString.split('_').length > 1,
      );
      // Allow overwrite if the existing data is assumed.
      if (assumedLocales.contains(locale)) {
        localeToResources[locale] = <String, String>{};
        localeToResourceAttributes[locale] = <String, dynamic>{};
        assumedLocales.remove(locale);
      } else {
        localeToResources[locale] ??= <String, String>{};
        localeToResourceAttributes[locale] ??= <String, dynamic>{};
      }
      populateResources(locale, arbFile);
      // Add an assumed locale to default to when there is no info on scriptOnly locales.
      locale = LocaleInfo.fromString(localeString, deriveScriptCode: true);
      if (locale.scriptCode != null) {
        final LocaleInfo scriptLocale = LocaleInfo.fromString(
          '${locale.languageCode}_${locale.scriptCode}',
        );
        if (!localeToResources.containsKey(scriptLocale)) {
          assumedLocales.add(scriptLocale);
          localeToResources[scriptLocale] ??= <String, String>{};
          localeToResourceAttributes[scriptLocale] ??= <String, dynamic>{};
          populateResources(scriptLocale, arbFile);
        }
      }
    }
  }
}

void exitWithError(String errorMessage) {
  stderr.writeln('fatal: $errorMessage');
  exit(1);
}

void checkCwdIsRepoRoot(String commandName) {
  final bool isRepoRoot = Directory('.git').existsSync();

  if (!isRepoRoot) {
    exitWithError(
      '$commandName must be run from the root of the Flutter repository. The '
      'current working directory is: ${Directory.current.path}',
    );
  }
}

GeneratorOptions parseArgs(List<String> rawArgs) {
  final argslib.ArgParser argParser =
      argslib.ArgParser()
        ..addFlag('help', abbr: 'h', help: 'Print the usage message for this command')
        ..addFlag('overwrite', abbr: 'w', help: 'Overwrite existing localizations')
        ..addFlag(
          'remove-undefined',
          help: 'Remove any localizations that are not defined in the canonical locale.',
        )
        ..addFlag(
          'widgets',
          help:
              'Whether to print the generated classes for the Widgets package only. Ignored when --overwrite is passed.',
        )
        ..addFlag(
          'material',
          help:
              'Whether to print the generated classes for the Material package only. Ignored when --overwrite is passed.',
        )
        ..addFlag(
          'cupertino',
          help:
              'Whether to print the generated classes for the Cupertino package only. Ignored when --overwrite is passed.',
        );
  final argslib.ArgResults args = argParser.parse(rawArgs);
  if (args.wasParsed('help') && args['help'] == true) {
    stderr.writeln(argParser.usage);
    exit(0);
  }
  final bool writeToFile = args['overwrite'] as bool;
  final bool removeUndefined = args['remove-undefined'] as bool;
  final bool widgetsOnly = args['widgets'] as bool;
  final bool materialOnly = args['material'] as bool;
  final bool cupertinoOnly = args['cupertino'] as bool;

  return GeneratorOptions(
    writeToFile: writeToFile,
    materialOnly: materialOnly,
    cupertinoOnly: cupertinoOnly,
    widgetsOnly: widgetsOnly,
    removeUndefined: removeUndefined,
  );
}

class GeneratorOptions {
  GeneratorOptions({
    required this.writeToFile,
    required this.removeUndefined,
    required this.materialOnly,
    required this.cupertinoOnly,
    required this.widgetsOnly,
  });

  final bool writeToFile;
  final bool removeUndefined;
  final bool materialOnly;
  final bool cupertinoOnly;
  final bool widgetsOnly;
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
      throw 'not sure how to deal with "$line"';
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
      languageSubtagRegistry
          .split('%%')
          .skip(1)
          .map<Map<String, List<String>>>(_parseSection)
          .toList();
  for (final Map<String, List<String>> section in sections) {
    assert(section.containsKey('Type'), section.toString());
    final String type = section['Type']!.single;
    if (type == 'language' || type == 'region' || type == 'script') {
      assert(
        section.containsKey('Subtag') && section.containsKey('Description'),
        section.toString(),
      );
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
  assert(_languages.containsKey(subtags[0]));
  final String language = _languages[subtags[0]]!;
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

/// Writes the header of each class which corresponds to a locale.
String generateClassDeclaration(LocaleInfo locale, String classNamePrefix, String superClass) {
  final String camelCaseName = locale.camelCase();
  return '''

/// The translations for ${describeLocale(locale.originalString)} (`${locale.originalString}`).
class $classNamePrefix$camelCaseName extends $superClass {''';
}

/// Return the input string as a Dart-parseable string.
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
  if (<String>['\n', '\f', '\t', '\r', '\b'].every((String pattern) => !value.contains(pattern))) {
    final bool hasDollar = value.contains(r'$');
    final bool hasBackslash = value.contains(r'\');
    final bool hasQuote = value.contains("'");
    final bool hasDoubleQuote = value.contains('"');
    if (!hasQuote) {
      return hasBackslash || hasDollar ? "r'$value'" : "'$value'";
    }
    if (!hasDoubleQuote) {
      return hasBackslash || hasDollar ? 'r"$value"' : '"$value"';
    }
  }

  const String backslash = '__BACKSLASH__';
  assert(
    !value.contains(backslash),
    'Input string cannot contain the sequence: '
    '"__BACKSLASH__", as it is used as part of '
    'backslash character processing.',
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

  return "'$value'";
}

/// Only used to generate localization strings for the Kannada locale ('kn') because
/// some of the localized strings contain characters that can crash Emacs on Linux.
/// See packages/flutter_localizations/lib/src/l10n/README for more information.
String generateEncodedString(String? locale, String value) {
  if (locale != 'kn' || value.runes.every((int code) => code <= 0xFF)) {
    return generateString(value);
  }

  final String unicodeEscapes =
      value.runes.map((int code) => '\\u{${code.toRadixString(16)}}').join();
  return "'$unicodeEscapes'";
}
