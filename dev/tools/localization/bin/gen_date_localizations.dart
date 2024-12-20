// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../localizations_utils.dart';

const String _kCommandName = 'gen_date_localizations.dart';

// Used to let _jsonToMap know what locale it's date symbols converting for.
// Date symbols for the Kannada locale ('kn') are handled specially because
// some of the strings contain characters that can crash Emacs on Linux.
// See packages/flutter_localizations/lib/src/l10n/README for more information.
String? currentLocale;

/// This program extracts localized date symbols and patterns from the intl
/// package for the subset of locales supported by the flutter_localizations
/// package.
///
/// The extracted data is written into:
///   packages/flutter_localizations/lib/src/l10n/generated_date_localizations.dart
///
/// ## Usage
///
/// Run this program from the root of the git repository.
///
/// The following outputs the generated Dart code to the console as a dry run:
///
/// ```bash
/// dart dev/tools/localization/bin/gen_date_localizations.dart
/// ```
///
/// If the data looks good, use the `--overwrite` option to overwrite the
/// lib/src/l10n/date_localizations.dart file:
///
/// ```bash
/// dart dev/tools/localization/bin/gen_date_localizations.dart --overwrite
/// ```
Future<void> main(List<String> rawArgs) async {
  checkCwdIsRepoRoot(_kCommandName);

  final bool writeToFile = parseArgs(rawArgs).writeToFile;

  final File packageConfigFile = File(
    path.join('packages', 'flutter_localizations', '.dart_tool', 'package_config.json'),
  );
  final bool packageConfigExists = packageConfigFile.existsSync();

  if (!packageConfigExists) {
    exitWithError(
      'File not found: ${packageConfigFile.path}. $_kCommandName must be run '
      'after a successful "flutter update-packages".',
    );
  }

  final List<Object?> packages =
      (json.decode(packageConfigFile.readAsStringSync()) as Map<String, Object?>)['packages']!
          as List<Object?>;

  String? pathToIntl;
  for (final Object? package in packages) {
    final Map<String, Object?> packageAsMap = package! as Map<String, Object?>;
    if (packageAsMap['name'] == 'intl') {
      pathToIntl = Uri.parse(packageAsMap['rootUri']! as String).toFilePath();
      break;
    }
  }

  if (pathToIntl == null) {
    exitWithError(
      'Could not find "intl" package. $_kCommandName must be run '
      'after a successful "flutter update-packages".',
    );
  }

  final Directory dateSymbolsDirectory = Directory(
    path.join(pathToIntl!, 'lib', 'src', 'data', 'dates', 'symbols'),
  );
  final Map<String, File> symbolFiles = _listIntlData(dateSymbolsDirectory);
  final Directory datePatternsDirectory = Directory(
    path.join(pathToIntl, 'lib', 'src', 'data', 'dates', 'patterns'),
  );
  final Map<String, File> patternFiles = _listIntlData(datePatternsDirectory);
  final StringBuffer buffer = StringBuffer();
  final Set<String> supportedLocales = _supportedLocales();

  buffer.writeln('''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file has been automatically generated. Please do not edit it manually.
// To regenerate run (omit --overwrite to print to console instead of the file):
// dart --enable-asserts dev/tools/localization/bin/gen_date_localizations.dart --overwrite

import 'package:intl/date_symbols.dart' as intl;

''');
  buffer.writeln('''
/// The subset of date symbols supported by the intl package which are also
/// supported by flutter_localizations.''');
  buffer.writeln('final Map<String, intl.DateSymbols> dateSymbols = <String, intl.DateSymbols> {');
  symbolFiles.forEach((String locale, File data) {
    currentLocale = locale;
    if (supportedLocales.contains(locale)) {
      final Map<String, Object?> objData =
          json.decode(data.readAsStringSync()) as Map<String, Object?>;
      buffer.writeln("'$locale': intl.DateSymbols(");
      objData.forEach((String key, Object? value) {
        if (value == null) {
          return;
        }
        buffer.writeln(_jsonToConstructorEntry(key, value));
      });
      buffer.writeln('),');
    }
  });
  currentLocale = null;
  buffer.writeln('};');

  // Code that uses datePatterns expects it to contain values of type
  // Map<String, String> not Map<String, dynamic>.
  buffer.writeln('''
/// The subset of date patterns supported by the intl package which are also
/// supported by flutter_localizations.''');
  buffer.writeln(
    'const Map<String, Map<String, String>> datePatterns = <String, Map<String, String>> {',
  );
  patternFiles.forEach((String locale, File data) {
    if (supportedLocales.contains(locale)) {
      final Map<String, dynamic> patterns =
          json.decode(data.readAsStringSync()) as Map<String, dynamic>;
      buffer.writeln("'$locale': <String, String>{");
      patterns.forEach((String key, dynamic value) {
        assert(value is String);
        buffer.writeln(_jsonToMapEntry(key, value));
      });
      buffer.writeln('},');
    }
  });
  buffer.writeln('};');

  if (writeToFile) {
    final File dateLocalizationsFile = File(
      path.join(
        'packages',
        'flutter_localizations',
        'lib',
        'src',
        'l10n',
        'generated_date_localizations.dart',
      ),
    );
    dateLocalizationsFile.writeAsStringSync(buffer.toString());
    final String extension = Platform.isWindows ? '.exe' : '';
    final ProcessResult result = Process.runSync(
      path.join('bin', 'cache', 'dart-sdk', 'bin', 'dart$extension'),
      <String>['format', dateLocalizationsFile.path],
    );
    if (result.exitCode != 0) {
      print(result.exitCode);
      print(result.stdout);
      print(result.stderr);
    }
  } else {
    print(buffer);
  }
}

String _jsonToConstructorEntry(String key, dynamic value) {
  return '$key: ${_jsonToObject(value)},';
}

String _jsonToMapEntry(String key, dynamic value) {
  return "'$key': ${_jsonToMap(value)},";
}

String _jsonToObject(dynamic json) {
  switch (json) {
    case null || num() || bool():
      return '$json';
    case String():
      return generateEncodedString(currentLocale, json);
    case Iterable<Object?>():
      final Type type = json.first.runtimeType;
      final StringBuffer buffer = StringBuffer('const <$type>[');
      for (final dynamic value in json) {
        buffer.writeln('${_jsonToMap(value)},');
      }
      buffer.write(']');
      return buffer.toString();
    case Map<String, dynamic>():
      final StringBuffer buffer = StringBuffer('<String, Object>{');
      json.forEach((String key, dynamic value) {
        buffer.writeln(_jsonToMapEntry(key, value));
      });
      buffer.write('}');
      return buffer.toString();
  }

  throw 'Unsupported JSON type ${json.runtimeType} of value $json.';
}

String _jsonToMap(dynamic json) {
  switch (json) {
    case null || num() || bool():
      return '$json';
    case String():
      return generateEncodedString(currentLocale, json);
    case Iterable<dynamic>():
      final StringBuffer buffer = StringBuffer('<String>[');
      for (final dynamic value in json) {
        buffer.writeln('${_jsonToMap(value)},');
      }
      buffer.write(']');
      return buffer.toString();
    case Map<String, dynamic>():
      final StringBuffer buffer = StringBuffer('<String, Object>{');
      json.forEach((String key, dynamic value) {
        buffer.writeln(_jsonToMapEntry(key, value));
      });
      buffer.write('}');
      return buffer.toString();
  }

  throw 'Unsupported JSON type ${json.runtimeType} of value $json.';
}

Set<String> _supportedLocales() {
  // Assumes that en_US is a supported locale by default. Without this, usage
  // of the intl package APIs before Flutter populates its set of supported i18n
  // date patterns and symbols may cause problems.
  //
  // For more context, see https://github.com/flutter/flutter/issues/67644.
  final Set<String> supportedLocales = <String>{'en_US'};
  final RegExp filenameRE = RegExp(r'(?:material|cupertino)_(\w+)\.arb$');
  final Directory supportedLocalesDirectory = Directory(
    path.join('packages', 'flutter_localizations', 'lib', 'src', 'l10n'),
  );
  for (final FileSystemEntity entity in supportedLocalesDirectory.listSync()) {
    final String filePath = entity.path;
    if (FileSystemEntity.isFileSync(filePath) && filenameRE.hasMatch(filePath)) {
      supportedLocales.add(filenameRE.firstMatch(filePath)![1]!);
    }
  }

  return supportedLocales;
}

Map<String, File> _listIntlData(Directory directory) {
  final Map<String, File> localeFiles = <String, File>{};
  final Iterable<File> files = directory.listSync().whereType<File>().where(
    (File file) => file.path.endsWith('.json'),
  );
  for (final File file in files) {
    final String locale = path.basenameWithoutExtension(file.path);
    localeFiles[locale] = file;
  }

  final List<String> locales = localeFiles.keys.toList(growable: false);
  locales.sort();
  return Map<String, File>.fromIterable(locales, value: (dynamic locale) => localeFiles[locale]!);
}
