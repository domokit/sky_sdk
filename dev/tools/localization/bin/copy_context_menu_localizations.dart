import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../localizations_utils.dart';
import '../localizations_validator.dart';

Future<void> main(List<String> rawArgs) async {
  final String localizationPath = path.join(
    'packages',
    'flutter_localizations',
    'lib',
    'src',
    'l10n',
  );
  updateMissingResources(localizationPath, 'widgets');
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
/*
bool isPluralVariation(String key, Map<String, dynamic> bundle) {
  final Match? pluralMatch = kPluralRegexp.firstMatch(key);
  if (pluralMatch == null) {
    return false;
  }
  final String prefix = pluralMatch[1]!;
  return bundle.containsKey('${prefix}Other');
}
*/

const List<String> keys = <String>[
  'copyButtonLabel',
  'cutButtonLabel',
  'lookUpButtonLabel',
  'searchWebButtonLabel',
  'shareButtonLabel',
  'pasteButtonLabel',
  'selectAllButtonLabel',
];

void updateMissingResources(
  String localizationPath,
  String groupPrefix, {
  bool removeUndefined = false,
}) {
  final Directory localizationDir = Directory(localizationPath);

  final Map<String, dynamic> englishMaterialBundle = loadBundle(
    File(path.join(localizationPath, 'material_en.arb')),
  );
  final Map<String, dynamic> englishBundle = loadBundle(
    File(path.join(localizationPath, 'widgets_en.arb')),
  );

  final List<FileSystemEntity> files = localizationDir.listSync().toList()..sort(sortFilesByPath);

  final RegExp widgetsPattern = RegExp('widgets_(\\w+)\\.arb');
  final RegExp materialPattern = RegExp('material_(\\w+)\\.arb');

  final Iterable<FileSystemEntity> widgetsFiles = files.where(
    (FileSystemEntity entity) => widgetsPattern.hasMatch(entity.path),
  );
  final Iterable<FileSystemEntity> materialFiles = files.where(
    (FileSystemEntity entity) => materialPattern.hasMatch(entity.path),
  );
  assert(widgetsFiles.length == materialFiles.length);

  final Map<FileSystemEntity, FileSystemEntity> fileMap = <FileSystemEntity, FileSystemEntity>{};
  for (final FileSystemEntity widgetsEntity in widgetsFiles) {
    final String widgetsFilename = widgetsEntity.path.split(Platform.pathSeparator).last;
    final String widgetsLanguage = widgetsFilename.substring(8);
    final FileSystemEntity materialEntity = materialFiles.firstWhere((
      FileSystemEntity possibleMaterialEntity,
    ) {
      final String materialFilename =
          possibleMaterialEntity.path.split(Platform.pathSeparator).last;
      return materialFilename.substring(9) == widgetsLanguage;
    }, orElse: () => throw Exception('No matching material file found for $widgetsLanguage.'));
    fileMap[widgetsEntity] = materialEntity;
  }

  return;
  /*
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
  */
}
