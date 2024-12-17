// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import 'localizations_utils.dart';
import 'message_parser.dart';

// The set of date formats that can be automatically localized.
//
// The localizations generation tool makes use of the intl library's
// DateFormat class to properly format dates based on the locale, the
// desired format, as well as the passed in [DateTime]. For example, using
// DateFormat.yMMMMd("en_US").format(DateTime.utc(1996, 7, 10)) results
// in the string "July 10, 1996".
//
// Since the tool generates code that uses DateFormat's constructor and its
// add_* methods, it is necessary to verify that the constructor/method exists,
// or the tool will generate code that may cause a compile-time error.
//
// See also:
//
// * <https://pub.dev/packages/intl>
// * <https://pub.dev/documentation/intl/latest/intl/DateFormat-class.html>
// * <https://api.dartlang.org/stable/2.7.0/dart-core/DateTime-class.html>
const Set<String> validDateFormats = <String>{
  'd',
  'E',
  'EEEE',
  'LLL',
  'LLLL',
  'M',
  'Md',
  'MEd',
  'MMM',
  'MMMd',
  'MMMEd',
  'MMMM',
  'MMMMd',
  'MMMMEEEEd',
  'QQQ',
  'QQQQ',
  'y',
  'yM',
  'yMd',
  'yMEd',
  'yMMM',
  'yMMMd',
  'yMMMEd',
  'yMMMM',
  'yMMMMd',
  'yMMMMEEEEd',
  'yQQQ',
  'yQQQQ',
  'H',
  'Hm',
  'Hms',
  'j',
  'jm',
  'jms',
  'jmv',
  'jmz',
  'jv',
  'jz',
  'm',
  'ms',
  's',
};

const String _dateFormatPartsDelimiter = '+';

// The set of number formats that can be automatically localized.
//
// The localizations generation tool makes use of the intl library's
// NumberFormat class to properly format numbers based on the locale and
// the desired format. For example, using
// NumberFormat.compactLong("en_US").format(1200000) results
// in the string "1.2 million".
//
// Since the tool generates code that uses NumberFormat's constructor, it is
// necessary to verify that the constructor exists, or the
// tool will generate code that may cause a compile-time error.
//
// See also:
//
// * <https://pub.dev/packages/intl>
// * <https://pub.dev/documentation/intl/latest/intl/NumberFormat-class.html>
const Set<String> _validNumberFormats = <String>{
  'compact',
  'compactCurrency',
  'compactSimpleCurrency',
  'compactLong',
  'currency',
  'decimalPattern',
  'decimalPatternDigits',
  'decimalPercentPattern',
  'percentPattern',
  'scientificPattern',
  'simpleCurrency',
};

// The names of the NumberFormat factory constructors which have named
// parameters rather than positional parameters.
//
// This helps the tool correctly generate number formatting code correctly.
//
// Example of code that uses named parameters:
// final NumberFormat format = NumberFormat.compact(
//   locale: localeName,
// );
//
// Example of code that uses positional parameters:
// final NumberFormat format = NumberFormat.scientificPattern(localeName);
const Set<String> _numberFormatsWithNamedParameters = <String>{
  'compact',
  'compactCurrency',
  'compactSimpleCurrency',
  'compactLong',
  'currency',
  'decimalPatternDigits',
  'decimalPercentPattern',
  'simpleCurrency',
};

class L10nException implements Exception {
  L10nException(this.message);

  final String message;

  @override
  String toString() => message;
}

class L10nParserException extends L10nException {
  L10nParserException(
    this.error,
    this.fileName,
    this.messageId,
    this.messageString,
    this.charNumber
  ): super('''
[$fileName:$messageId] $error
    $messageString
    ${List<String>.filled(charNumber, ' ').join()}^''');

  final String error;
  final String fileName;
  final String messageId;
  final String messageString;
  // Position of character within the "messageString" where the error is.
  final int charNumber;
}

class L10nMissingPlaceholderException extends L10nParserException {
  L10nMissingPlaceholderException(
    super.error,
    super.fileName,
    super.messageId,
    super.messageString,
    super.charNumber,
    this.placeholderName,
  );

  final String placeholderName;
}

// One optional named parameter to be used by a NumberFormat.
//
// Some of the NumberFormat factory constructors have optional named parameters.
// For example NumberFormat.compactCurrency has a decimalDigits parameter that
// specifies the number of decimal places to use when formatting.
//
// Optional parameters for NumberFormat placeholders are specified as a
// JSON map value for optionalParameters in a resource's "@" ARB file entry:
//
// "@myResourceId": {
//   "placeholders": {
//     "myNumberPlaceholder": {
//       "type": "double",
//       "format": "compactCurrency",
//       "optionalParameters": {
//         "decimalDigits": 2
//       }
//     }
//   }
// }
class OptionalParameter {
  const OptionalParameter(this.name, this.value);

  final String name;
  final Object value;
}

// One message parameter: one placeholder from an @foo entry in the template ARB file.
//
// Placeholders are specified as a JSON map with one entry for each placeholder.
// One placeholder must be specified for each message "{parameter}".
// Each placeholder entry is also a JSON map. If the map is empty, the placeholder
// is assumed to be an Object value whose toString() value will be displayed.
// For example:
//
// "greeting": "{hello} {world}",
// "@greeting": {
//   "description": "A message with a two parameters",
//   "placeholders": {
//     "hello": {},
//     "world": {}
//   }
// }
//
// Each placeholder can optionally specify a valid Dart type. If the type
// is NumberFormat or DateFormat then a format which matches one of the
// type's factory constructors can also be specified. In this example the
// date placeholder is to be formatted with DateFormat.yMMMMd:
//
// "helloWorldOn": "Hello World on {date}",
// "@helloWorldOn": {
//   "description": "A message with a date parameter",
//   "placeholders": {
//     "date": {
//       "type": "DateTime",
//       "format": "yMMMMd"
//     }
//   }
// }
//
class Placeholder {
  Placeholder(this.resourceId, this.name, Map<String, Object?> attributes)
    : example = _stringAttribute(resourceId, name, attributes, 'example'),
      type = _stringAttribute(resourceId, name, attributes, 'type'),
      format = _stringAttribute(resourceId, name, attributes, 'format'),
      optionalParameters = _optionalParameters(resourceId, name, attributes),
      isCustomDateFormat = _boolAttribute(resourceId, name, attributes, 'isCustomDateFormat');

  final String resourceId;
  final String name;
  final String? example;
  final String? format;
  final List<OptionalParameter> optionalParameters;
  final bool? isCustomDateFormat;
  // The following will be initialized after all messages are parsed in the Message constructor.
  String? type;
  bool isPlural = false;
  bool isSelect = false;
  bool isDateTime = false;
  bool requiresDateFormatting = false;

  bool get requiresFormatting => requiresDateFormatting || requiresNumFormatting;
  bool get requiresNumFormatting => <String>['int', 'num', 'double'].contains(type) && format != null;
  bool get hasValidNumberFormat => _validNumberFormats.contains(format);
  bool get hasNumberFormatWithParameters => _numberFormatsWithNamedParameters.contains(format);
  // 'format' can contain a number of date time formats separated by `dateFormatPartsDelimiter`.
  List<String> get dateFormatParts => format?.split(_dateFormatPartsDelimiter) ?? <String>[];
  bool get hasValidDateFormat => dateFormatParts.every(validDateFormats.contains);

  static String? _stringAttribute(
    String resourceId,
    String name,
    Map<String, Object?> attributes,
    String attributeName,
  ) {
    final Object? value = attributes[attributeName];
    if (value == null) {
      return null;
    }
    if (value is! String || value.isEmpty) {
      throw L10nException(
        'The "$attributeName" value of the "$name" placeholder in message $resourceId '
        'must be a non-empty string.',
      );
    }
    return value;
  }

  static bool? _boolAttribute(
      String resourceId,
      String name,
      Map<String, Object?> attributes,
      String attributeName,
      ) {
    final Object? value = attributes[attributeName];
    if (value == null) {
      return null;
    }
    if (value is bool) {
      return value;
    }
    if (value != 'true' && value != 'false') {
      throw L10nException(
        'The "$attributeName" value of the "$name" placeholder in message $resourceId '
            'must be a boolean value.',
      );
    }
    return value == 'true';
  }

  static List<OptionalParameter> _optionalParameters(
    String resourceId,
    String name,
    Map<String, Object?> attributes
  ) {
    final Object? value = attributes['optionalParameters'];
    if (value == null) {
      return <OptionalParameter>[];
    }
    if (value is! Map<String, Object?>) {
      throw L10nException(
        'The "optionalParameters" value of the "$name" placeholder in message '
        '$resourceId is not a properly formatted Map. Ensure that it is a map '
        'with keys that are strings.'
      );
    }
    final Map<String, Object?> optionalParameterMap = value;
    return optionalParameterMap.keys.map<OptionalParameter>((String parameterName) {
      return OptionalParameter(parameterName, optionalParameterMap[parameterName]!);
    }).toList();
  }
}

// All translations for a given message specified by a resource id.
//
// The template ARB file must contain an entry called @myResourceId for each
// message named myResourceId. The @ entry describes message parameters
// called "placeholders" and can include an optional description.
// Here's a simple example message with no parameters:
//
// "helloWorld": "Hello World",
// "@helloWorld": {
//   "description": "The conventional newborn programmer greeting"
// }
//
// The value of this Message is "Hello World". The Message's value is the
// localized string to be shown for the template ARB file's locale.
// The docs for the Placeholder explain how placeholder entries are defined.
class Message {
  Message(
    AppResourceBundle templateBundle,
    AppResourceBundleCollection allBundles,
    this.resourceId,
    bool isResourceAttributeRequired,
    {
      this.useRelaxedSyntax = false,
      this.useEscaping = false,
      this.logger,
    }
  ) : assert(resourceId.isNotEmpty),
      value = _value(templateBundle.resources, resourceId),
      formattedResourceId = _formattedResourceId(resourceId, templateBundle.namespace),
      description = _description(templateBundle.resources, resourceId, isResourceAttributeRequired),
      templatePlaceholders = _placeholders(templateBundle.resources, resourceId, isResourceAttributeRequired),
      localePlaceholders = <LocaleInfo, Map<String, Placeholder>>{},
      messages = <LocaleInfo, String?>{},
      parsedMessages = <LocaleInfo, Node?>{} {
    // Filenames for error handling.
    final Map<LocaleInfo, String> filenames = <LocaleInfo, String>{};
    // Collect all translations from allBundles and parse them.
    for (final AppResourceBundle bundle in allBundles.bundles) {
      filenames[bundle.locale] = bundle.file.basename;
      final String? translation = bundle.translationFor(resourceId);
      messages[bundle.locale] = translation;

      localePlaceholders[bundle.locale] = templateBundle.locale == bundle.locale
        ? templatePlaceholders
        : _placeholders(bundle.resources, resourceId, false);

      List<String>? validPlaceholders;
      if (useRelaxedSyntax) {
        validPlaceholders = templatePlaceholders.entries.map((MapEntry<String, Placeholder> e) => e.key).toList();
      }
      try {
        parsedMessages[bundle.locale] = translation == null ? null : Parser(
          resourceId,
          bundle.file.basename,
          translation,
          useEscaping: useEscaping,
          placeholders: validPlaceholders,
          logger: logger,
        ).parse();
      } on L10nParserException catch (error) {
        logger?.printError(error.toString());
        // Treat it as an untranslated message in case we can't parse.
        parsedMessages[bundle.locale] = null;
        hadErrors = true;
      }
    }
    // Infer the placeholders
    _inferPlaceholders();
  }

  final String resourceId;
  final String formattedResourceId;
  final String value;
  final String? description;
  late final Map<LocaleInfo, String?> messages;
  final Map<LocaleInfo, Node?> parsedMessages;
  final Map<LocaleInfo, Map<String, Placeholder>> localePlaceholders;
  final Map<String, Placeholder> templatePlaceholders;
  final bool useEscaping;
  final bool useRelaxedSyntax;
  final Logger? logger;
  bool hadErrors = false;

  Iterable<Placeholder> getPlaceholders(LocaleInfo locale) {
    final Map<String, Placeholder>? placeholders = localePlaceholders[locale];
    if (placeholders == null) {
      return templatePlaceholders.values;
    }
    return templatePlaceholders.values
      .map((Placeholder templatePlaceholder) => placeholders[templatePlaceholder.name] ?? templatePlaceholder);
  }

  static String _formattedResourceId(String resourceId, String namespace) =>
    namespace.isEmpty ? resourceId : '${namespace}_$resourceId';

  static String _value(Map<String, Object?> bundle, String resourceId) {
    final Object? value = bundle[resourceId];
    if (value == null) {
      throw L10nException('A value for resource "$resourceId" was not found.');
    }
    if (value is! String) {
      throw L10nException('The value of "$resourceId" is not a string.');
    }
    return value;
  }

  static Map<String, Object?>? _attributes(
    Map<String, Object?> bundle,
    String resourceId,
    bool isResourceAttributeRequired,
  ) {
    final Object? attributes = bundle['@$resourceId'];
    if (isResourceAttributeRequired) {
      if (attributes == null) {
        throw L10nException(
          'Resource attribute "@$resourceId" was not found. Please '
          'ensure that each resource has a corresponding @resource.'
        );
      }
    }

    if (attributes != null && attributes is! Map<String, Object?>) {
      throw L10nException(
        'The resource attribute "@$resourceId" is not a properly formatted Map. '
        'Ensure that it is a map with keys that are strings.'
      );
    }

    return attributes as Map<String, Object?>?;
  }

  static String? _description(
    Map<String, Object?> bundle,
    String resourceId,
    bool isResourceAttributeRequired,
  ) {
    final Map<String, Object?>? resourceAttributes = _attributes(bundle, resourceId, isResourceAttributeRequired);
    if (resourceAttributes == null) {
      return null;
    }

    final Object? value = resourceAttributes['description'];
    if (value == null) {
      return null;
    }
    if (value is! String) {
      throw L10nException(
        'The description for "@$resourceId" is not a properly formatted String.'
      );
    }
    return value;
  }

  static Map<String, Placeholder> _placeholders(
    Map<String, Object?> bundle,
    String resourceId,
    bool isResourceAttributeRequired,
  ) {
    final Map<String, Object?>? resourceAttributes = _attributes(bundle, resourceId, isResourceAttributeRequired);
    if (resourceAttributes == null) {
      return <String, Placeholder>{};
    }
    final Object? allPlaceholdersMap = resourceAttributes['placeholders'];
    if (allPlaceholdersMap == null) {
      return <String, Placeholder>{};
    }
    if (allPlaceholdersMap is! Map<String, Object?>) {
      throw L10nException(
        'The "placeholders" attribute for message $resourceId, is not '
        'properly formatted. Ensure that it is a map with string valued keys.'
      );
    }
    return Map<String, Placeholder>.fromEntries(
      allPlaceholdersMap.keys.map((String placeholderName) {
        final Object? value = allPlaceholdersMap[placeholderName];
        if (value is! Map<String, Object?>) {
          throw L10nException(
            'The value of the "$placeholderName" placeholder attribute for message '
            '"$resourceId", is not properly formatted. Ensure that it is a map '
            'with string valued keys.'
          );
        }
        return MapEntry<String, Placeholder>(placeholderName, Placeholder(resourceId, placeholderName, value));
      }),
    );
  }

  // Using parsed translations, attempt to infer types of placeholders used by plurals and selects.
  // For undeclared placeholders, create a new placeholder.
  void _inferPlaceholders() {
    // We keep the undeclared placeholders separate so that we can sort them alphabetically afterwards.
    final Map<String, Placeholder> undeclaredPlaceholders = <String, Placeholder>{};
    // Helper for getting placeholder by name.
    for (final LocaleInfo locale in parsedMessages.keys) {
      Placeholder? getPlaceholder(String name) =>
          localePlaceholders[locale]?[name] ??
          templatePlaceholders[name] ??
          undeclaredPlaceholders[name];
      if (parsedMessages[locale] == null) {
        continue;
      }
      final List<Node> traversalStack = <Node>[parsedMessages[locale]!];
      while (traversalStack.isNotEmpty) {
        final Node node = traversalStack.removeLast();
        if (<ST>[
          ST.placeholderExpr,
          ST.pluralExpr,
          ST.selectExpr,
          ST.argumentExpr
        ].contains(node.type)) {
          final String identifier = node.children[1].value!;
          Placeholder? placeholder = getPlaceholder(identifier);
          if (placeholder == null) {
            placeholder = Placeholder(resourceId, identifier, <String, Object?>{});
            undeclaredPlaceholders[identifier] = placeholder;
          }
          if (node.type == ST.pluralExpr) {
            placeholder.isPlural = true;
          } else if (node.type == ST.selectExpr) {
            placeholder.isSelect = true;
          } else if (node.type == ST.argumentExpr) {
            placeholder.isDateTime = true;
          } else {
            // Here the node type must be ST.placeholderExpr.
            // A DateTime placeholder must require date formatting.
            if (placeholder.type == 'DateTime') {
              placeholder.requiresDateFormatting = true;
            }
          }
        }
        traversalStack.addAll(node.children);
      }
    }
    templatePlaceholders.addEntries(
      undeclaredPlaceholders.entries
        .toList()
        ..sort((MapEntry<String, Placeholder> p1, MapEntry<String, Placeholder> p2) => p1.key.compareTo(p2.key))
    );

    bool atMostOneOf(bool x, bool y, bool z) {
      return x && !y && !z
        || !x && y && !z
        || !x && !y && z
        || !x && !y && !z;
    }

    for (final Placeholder placeholder in templatePlaceholders.values) {
      if (!atMostOneOf(placeholder.isPlural, placeholder.isDateTime, placeholder.isSelect)) {
        throw L10nException('Placeholder is used as plural/select/datetime in certain languages.');
      } else if (placeholder.isPlural) {
        if (placeholder.type == null) {
          placeholder.type = 'num';
        }
        else if (!<String>['num', 'int'].contains(placeholder.type)) {
          throw L10nException("Placeholders used in plurals must be of type 'num' or 'int'");
        }
      } else if (placeholder.isSelect) {
        if (placeholder.type == null) {
          placeholder.type = 'String';
        } else if (placeholder.type != 'String') {
          throw L10nException("Placeholders used in selects must be of type 'String'");
        }
      } else if (placeholder.isDateTime) {
        if (placeholder.type == null) {
          placeholder.type = 'DateTime';
        } else if (placeholder.type != 'DateTime') {
          throw L10nException("Placeholders used in datetime expressions much be of type 'DateTime'");
        }
      }
      placeholder.type ??= 'Object';
    }
  }
}

/// Represents the contents of one ARB file.
class AppResourceBundle {
  /// Assuming that the caller has verified that the file exists and is readable.
  factory AppResourceBundle(File file, String namespace) {
    final Map<String, Object?> resources = parseJsonFile(file);
    final LocaleInfo localeInfo = localeInfoFromFile(file, cachedResources: resources);
    final Iterable<String> ids = resources.keys.where((String key) => !key.startsWith('@'));
    return AppResourceBundle._(file, localeInfo, resources, ids, namespace);
  }

  const AppResourceBundle._(this.file, this.locale, this.resources, this.resourceIds, this.namespace);

  final File file;
  final LocaleInfo locale;
  final String namespace;
  /// JSON representation of the contents of the ARB file.
  final Map<String, Object?> resources;
  final Iterable<String> resourceIds;

  String? translationFor(String resourceId) {
    final Object? result = resources[resourceId];
    if (result is! String?) {
      throwToolExit('Localized message for key "$resourceId" in "${file.path}" '
        'is not a string.');
    }
    return result;
  }

  @override
  String toString() {
    return 'AppResourceBundle($locale, ${file.path})';
  }
}

// Represents all directories that contain ARB files.
class AppResourceGroupCollection {
  factory AppResourceGroupCollection(Directory inputDirectory) {
    final List<FileSystemEntity> entities = inputDirectory.listSync();
    final List<Directory> directories = <Directory>[inputDirectory];
    directories.addAll(entities.whereType<Directory>());

    final Map<String, AppResourceBundleCollection> namespaceToBundleCollection =
        <String, AppResourceBundleCollection>{};

    for (final Directory directory in directories) {
      final bool isRootDirectory = directory == inputDirectory;
      final String namespace = isRootDirectory ? '' : directory.basename;
      final AppResourceBundleCollection bundleCollection =
        AppResourceBundleCollection(
          directory,
          namespace
        );

      namespaceToBundleCollection[namespace] = bundleCollection;
    }

    return AppResourceGroupCollection._(namespaceToBundleCollection);
  }

  AppResourceGroupCollection._(this._namespaceToBundleCollection);

  final Map<String, AppResourceBundleCollection> _namespaceToBundleCollection;

  Iterable<AppResourceBundle> get allBundles => _namespaceToBundleCollection.values.expand((AppResourceBundleCollection element) => element.bundles);

  AppResourceBundleCollection bundleForNamespace(String namespace) => _namespaceToBundleCollection[namespace]!;

  Iterable<AppResourceBundle> bundlesForLanguage(LocaleInfo locale) => allBundles.where((AppResourceBundle bundle) => bundle.locale == locale);

  Set<LocaleInfo> get supportedLocales => Set<LocaleInfo>.from(allBundles.map((AppResourceBundle bundle) => bundle.locale));
}

// Represents all of the ARB files in [directory] as [AppResourceBundle]s.
class AppResourceBundleCollection {
  factory AppResourceBundleCollection(Directory directory, String namespace) {
    // Assuming that the caller has verified that the directory is readable.

    final RegExp filenameRE = RegExp(r'(\w+)\.arb$');
    final Map<LocaleInfo, AppResourceBundle> localeToBundle = <LocaleInfo, AppResourceBundle>{};
    final Map<String, List<LocaleInfo>> languageToLocales = <String, List<LocaleInfo>>{};
    // We require the list of files to be sorted so that
    // "languageToLocales[bundle.locale.languageCode]" is not null
    // by the time we handle locales with country codes.
    final List<File> files = directory
      .listSync()
.whereType<File>()
      .where((File e) => filenameRE.hasMatch(e.path))
      .toList()
      ..sort(sortFilesByPath);
    for (final File file in files) {
      final AppResourceBundle bundle = AppResourceBundle(file, namespace);
      if (localeToBundle[bundle.locale] != null) {
        throw L10nException(
          "Multiple arb files with the same '${bundle.locale}' locale detected. \n"
          'Ensure that there is exactly one arb file for each locale.'
        );
      }
      localeToBundle[bundle.locale] = bundle;
      languageToLocales[bundle.locale.languageCode] ??= <LocaleInfo>[];
      languageToLocales[bundle.locale.languageCode]!.add(bundle.locale);
    }

    languageToLocales.forEach((String language, List<LocaleInfo> listOfCorrespondingLocales) {
      final List<String> localeStrings = listOfCorrespondingLocales.map((LocaleInfo locale) {
        return locale.toString();
      }).toList();
      if (!localeStrings.contains(language)) {
        throw L10nException(
          'Arb file for a fallback, $language, does not exist, even though \n'
          'the following locale(s) exist: $listOfCorrespondingLocales. \n'
          'When locales specify a script code or country code, a \n'
          'base locale (without the script code or country code) should \n'
          'exist as the fallback. Please create a {fileName}_$language.arb \n'
          'file.'
        );
      }
    });

    return AppResourceBundleCollection._(namespace, directory, localeToBundle, languageToLocales);
  }

  const AppResourceBundleCollection._(this.namespace, this._directory, this._localeToBundle, this._languageToLocales);

  final String namespace;
  final Directory _directory;
  final Map<LocaleInfo, AppResourceBundle> _localeToBundle;
  final Map<String, List<LocaleInfo>> _languageToLocales;

  Iterable<LocaleInfo> get locales => _localeToBundle.keys;
  Iterable<AppResourceBundle> get bundles => _localeToBundle.values;
  AppResourceBundle? bundleFor(LocaleInfo locale) => _localeToBundle[locale];

  Iterable<String> get languages => _languageToLocales.keys;
  Iterable<LocaleInfo> localesForLanguage(String language) => _languageToLocales[language] ?? <LocaleInfo>[];

  @override
  String toString() {
    return 'AppResourceBundleCollection(${_directory.path}, ${locales.length} locales)';
  }
}
