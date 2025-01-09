// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This program updates the language locale arb files with any missing resource
// entries that are included in the English arb files. This is useful when
// adding new resources for localization. You can just add the appropriate
// entries to the English arb file and then run this script. It will then check
// all of the other language locale arb files and update them with the English
// source for any missing resources. These will be picked up by the localization
// team and then translated.
//
// ## Usage
//
// Run this program from the root of the git repository.
//
// ```
// dart dev/tools/localization/bin/gen_missing_localizations.dart
// ```

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../localizations_utils.dart';
import '../localizations_validator.dart';

Future<void> main(List<String> rawArgs) async {
  bool removeUndefined = false;
  if (rawArgs.contains('--remove-undefined')) {
    removeUndefined = true;
  }
  checkCwdIsRepoRoot('gen_missing_localizations');

  final String localizationPath = path.join(
    'packages',
    'flutter_localizations',
    'lib',
    'src',
    'l10n',
  );
  updateMissingResources(localizationPath, 'material', removeUndefined: removeUndefined);
  updateMissingResources(localizationPath, 'cupertino', removeUndefined: removeUndefined);
}

Map<String, dynamic> loadBundle(File file) {
  if (!FileSystemEntity.isFileSync(file.path)) {
    exitWithError('Unable to find input file: ${file.path}');
  }
  return json.decode(file.readAsStringSync()) as Map<String, dynamic>;
}

void writeBundle(File file, Map<String, dynamic> bundle) {
  final StringBuffer contents = StringBuffer();
  contents.writeln('{');
  for (final String key in bundle.keys) {
    contents.writeln('  "$key": ${json.encode(bundle[key])}${key == bundle.keys.last ? '' : ','}');
  }
  contents.writeln('}');
  file.writeAsStringSync(contents.toString());
}

Set<String> resourceKeys(Map<String, dynamic> bundle) {
  return Set<String>.from(
    // Skip any attribute keys
    bundle.keys.where((String key) => !key.startsWith('@')),
  );
}

bool intentionallyOmitted(String key, Map<String, dynamic> bundle) {
  final String attributeKey = '@$key';
  final dynamic attribute = bundle[attributeKey];
  return attribute is Map && attribute.containsKey('notUsed');
}

/// Whether `key` corresponds to one of the plural variations of a key with
/// the same prefix and suffix "Other".
bool isPluralVariation(String key, Map<String, dynamic> bundle) {
  final Match? pluralMatch = kPluralRegexp.firstMatch(key);
  if (pluralMatch == null) {
    return false;
  }
  final String prefix = pluralMatch[1]!;
  return bundle.containsKey('${prefix}Other');
}

void updateMissingResources(
  String localizationPath,
  String groupPrefix, {
  bool removeUndefined = false,
}) {
  final Directory localizationDir = Directory(localizationPath);
  final RegExp filenamePattern = RegExp('${groupPrefix}_(\\w+)\\.arb');

  final Map<String, dynamic> englishBundle = loadBundle(
    File(path.join(localizationPath, '${groupPrefix}_en.arb')),
  );
  final Set<String> requiredKeys = resourceKeys(englishBundle);

  for (final FileSystemEntity entity
      in localizationDir.listSync().toList()..sort(sortFilesByPath)) {
    final String entityPath = entity.path;
    if (FileSystemEntity.isFileSync(entityPath) && filenamePattern.hasMatch(entityPath)) {
      final String localeString = filenamePattern.firstMatch(entityPath)![1]!;
      final LocaleInfo locale = LocaleInfo.fromString(localeString);

      // Only look at top-level language locales
      if (locale.length == 1) {
        final File arbFile = File(entityPath);
        final Map<String, dynamic> localeBundle = loadBundle(arbFile);
        final Set<String> localeResources = resourceKeys(localeBundle);
        // Whether or not the resources were modified and need to be updated.
        bool shouldWrite = false;

        // Remove any localizations that are not defined in the canonical
        // locale. This allows unused localizations to be removed if
        // --remove-undefined is passed.
        if (removeUndefined) {
          bool isIncluded(String key) {
            return !isPluralVariation(key, localeBundle) &&
                !intentionallyOmitted(key, localeBundle);
          }

          // Find any resources in this locale that don't appear in the
          // canonical locale, and skipping any which should not be included
          // (plurals and intentionally omitted).
          final Set<String> extraResources =
              localeResources.difference(requiredKeys).where(isIncluded).toSet();

          // Remove them.
          localeBundle.removeWhere((String key, dynamic value) {
            final bool found = extraResources.contains(key);
            if (found) {
              shouldWrite = true;
            }
            return found;
          });
          if (shouldWrite) {
            print('Updating $entityPath by removing extra entries for $extraResources');
          }
        }

        // Add in any resources that are in the canonical locale and not present
        // in this locale.
        final Set<String> missingResources =
            requiredKeys
                .difference(localeResources)
                .where(
                  (String key) =>
                      !isPluralVariation(key, localeBundle) &&
                      !intentionallyOmitted(key, localeBundle),
                )
                .toSet();
        if (missingResources.isNotEmpty) {
          localeBundle.addEntries(
            missingResources.map(
              (String k) => MapEntry<String, String>(k, englishBundle[k].toString()),
            ),
          );
          shouldWrite = true;
          print('Updating $entityPath with missing entries for $missingResources');
        }
        if (shouldWrite) {
          writeBundle(arbFile, localeBundle);
        }
      }
    }
  }
}
