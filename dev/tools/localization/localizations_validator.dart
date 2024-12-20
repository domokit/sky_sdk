// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show json;
import 'dart:io';

import 'localizations_utils.dart';

// The first suffix in kPluralSuffixes must be "Other". "Other" is special
// because it's the only one that is required.
const List<String> kPluralSuffixes = <String>['Other', 'Zero', 'One', 'Two', 'Few', 'Many'];
final RegExp kPluralRegexp = RegExp(r'(\w*)(' + kPluralSuffixes.skip(1).join(r'|') + r')$');

class ValidationError implements Exception {
  ValidationError(this.message);
  final String message;
  @override
  String toString() => message;
}

/// Sanity checking of the @foo metadata in the English translations, *_en.arb.
///
/// - For each foo, resource, there must be a corresponding @foo.
/// - For each @foo resource, there must be a corresponding foo, except
///   for plurals, for which there must be a fooOther.
/// - Each @foo resource must have a Map value with a String valued
///   description entry.
///
/// Throws an exception upon failure.
void validateEnglishLocalizations(File file) {
  final StringBuffer errorMessages = StringBuffer();

  if (!file.existsSync()) {
    errorMessages.writeln('English localizations do not exist: $file');
    throw ValidationError(errorMessages.toString());
  }

  final Map<String, dynamic> bundle = json.decode(file.readAsStringSync()) as Map<String, dynamic>;

  for (final String resourceId in bundle.keys) {
    if (resourceId.startsWith('@')) {
      continue;
    }

    if (bundle['@$resourceId'] != null) {
      continue;
    }

    bool checkPluralResource(String suffix) {
      final int suffixIndex = resourceId.indexOf(suffix);
      return suffixIndex != -1 && bundle['@${resourceId.substring(0, suffixIndex)}'] != null;
    }

    if (kPluralSuffixes.any(checkPluralResource)) {
      continue;
    }

    errorMessages.writeln('A value was not specified for @$resourceId');
  }

  for (final String atResourceId in bundle.keys) {
    if (!atResourceId.startsWith('@')) {
      continue;
    }

    final dynamic atResourceValue = bundle[atResourceId];
    final Map<String, dynamic>? atResource =
        atResourceValue is Map<String, dynamic> ? atResourceValue : null;
    if (atResource == null) {
      errorMessages.writeln('A map value was not specified for $atResourceId');
      continue;
    }

    final bool optional = atResource.containsKey('optional');
    final String? description = atResource['description'] as String?;
    if (description == null && !optional) {
      errorMessages.writeln('No description specified for $atResourceId');
    }

    final String? plural = atResource['plural'] as String?;
    final String resourceId = atResourceId.substring(1);
    if (plural != null) {
      final String resourceIdOther = '${resourceId}Other';
      if (!bundle.containsKey(resourceIdOther)) {
        errorMessages.writeln('Default plural resource $resourceIdOther undefined');
      }
    } else {
      if (!optional && !bundle.containsKey(resourceId)) {
        errorMessages.writeln('No matching $resourceId defined for $atResourceId');
      }
    }
  }

  if (errorMessages.isNotEmpty) {
    throw ValidationError(errorMessages.toString());
  }
}

/// This removes undefined localizations (localizations that aren't present in
/// the canonical locale anymore) by:
///
/// 1. Looking up the canonical (English, in this case) localizations.
/// 2. For each locale, getting the resources.
/// 3. Determining the set of keys that aren't plural variations (we're only
///    interested in the base terms being translated and not their variants)
/// 4. Determining the set of invalid keys; that is those that are (non-plural)
///    keys in the resources for this locale, but which _aren't_ keys in the
///    canonical list.
/// 5. Removes the invalid mappings from this resource's locale.
void removeUndefinedLocalizations(Map<LocaleInfo, Map<String, String>> localeToResources) {
  final Map<String, String> canonicalLocalizations =
      localeToResources[LocaleInfo.fromString('en')]!;
  final Set<String> canonicalKeys = Set<String>.from(canonicalLocalizations.keys);

  localeToResources.forEach((LocaleInfo locale, Map<String, String> resources) {
    bool isPluralVariation(String key) {
      final Match? pluralMatch = kPluralRegexp.firstMatch(key);
      if (pluralMatch == null) {
        return false;
      }
      final String? prefix = pluralMatch[1];
      return resources.containsKey('${prefix}Other');
    }

    final Set<String> keys = Set<String>.from(
      resources.keys.where((String key) => !isPluralVariation(key)),
    );

    final Set<String> invalidKeys = keys.difference(canonicalKeys);
    resources.removeWhere((String key, String value) => invalidKeys.contains(key));
  });
}

/// Enforces the following invariants in our localizations:
///
/// - Resource keys are valid, i.e. they appear in the canonical list.
/// - Resource keys are complete for language-level locales, e.g. "es", "he".
///
/// Uses "en" localizations as the canonical source of locale keys that other
/// locales are compared against.
///
/// If validation fails, throws an exception.
void validateLocalizations(
  Map<LocaleInfo, Map<String, String>> localeToResources,
  Map<LocaleInfo, Map<String, dynamic>> localeToAttributes, {
  bool removeUndefined = false,
}) {
  final Map<String, String> canonicalLocalizations =
      localeToResources[LocaleInfo.fromString('en')]!;
  final Set<String> canonicalKeys = Set<String>.from(canonicalLocalizations.keys);
  final StringBuffer errorMessages = StringBuffer();
  bool explainMissingKeys = false;
  for (final LocaleInfo locale in localeToResources.keys) {
    final Map<String, String> resources = localeToResources[locale]!;

    // Whether `key` corresponds to one of the plural variations of a key with
    // the same prefix and suffix "Other".
    //
    // Many languages require only a subset of these variations, so we do not
    // require them so long as the "Other" variation exists.
    bool isPluralVariation(String key) {
      final Match? pluralMatch = kPluralRegexp.firstMatch(key);
      if (pluralMatch == null) {
        return false;
      }
      final String? prefix = pluralMatch[1];
      return resources.containsKey('${prefix}Other');
    }

    final Set<String> keys = Set<String>.from(
      resources.keys.where((String key) => !isPluralVariation(key)),
    );

    // Make sure keys are valid (i.e. they also exist in the canonical
    // localizations)
    final Set<String> invalidKeys = keys.difference(canonicalKeys);
    if (invalidKeys.isNotEmpty && !removeUndefined) {
      errorMessages.writeln(
        'Locale "$locale" contains invalid resource keys: ${invalidKeys.join(', ')}',
      );
    }

    // For language-level locales only, check that they have a complete list of
    // keys, or opted out of using certain ones.
    if (locale.length == 1) {
      final Map<String, dynamic>? attributes = localeToAttributes[locale];
      final List<String?> missingKeys = <String?>[];
      for (final String missingKey in canonicalKeys.difference(keys)) {
        final dynamic attribute = attributes?[missingKey];
        final bool intentionallyOmitted = attribute is Map && attribute.containsKey('notUsed');
        if (!intentionallyOmitted && !isPluralVariation(missingKey)) {
          missingKeys.add(missingKey);
        }
      }
      if (missingKeys.isNotEmpty) {
        explainMissingKeys = true;
        errorMessages.writeln(
          'Locale "$locale" is missing the following resource keys: ${missingKeys.join(', ')}',
        );
      }
    }
  }

  if (errorMessages.isNotEmpty) {
    if (explainMissingKeys) {
      errorMessages
        ..writeln()
        ..writeln(
          'If a resource key is intentionally omitted, add an attribute corresponding '
          'to the key name with a "notUsed" property explaining why. Example:',
        )
        ..writeln()
        ..writeln('"@anteMeridiemAbbreviation": {')
        ..writeln('  "notUsed": "Sindhi time format does not use a.m. indicator"')
        ..writeln('}');
    }
    throw ValidationError(errorMessages.toString());
  }
}
