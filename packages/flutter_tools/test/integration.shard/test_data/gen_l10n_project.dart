// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import '../test_utils.dart';
import 'project.dart';

class GenL10nProject extends Project {
  GenL10nProject({required this.useNamedParameters});

  @override
  Future<void> setUpIn(
    Directory dir, {
    bool useDeferredLoading = false,
    bool useSyntheticPackage = false,
  }) {
    this.dir = dir;
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_en.arb'), appEn);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_en_CA.arb'), appEnCa);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_en_GB.arb'), appEnGb);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_es.arb'), appEs);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_es_419.arb'), appEs419);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_zh.arb'), appZh);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_zh_Hant.arb'), appZhHant);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_zh_Hans.arb'), appZhHans);
    writeFile(fileSystem.path.join(dir.path, 'lib', 'l10n', 'app_zh_Hant_TW.arb'), appZhHantTw);
    writeFile(
      fileSystem.path.join(dir.path, 'l10n.yaml'),
      l10nYaml(
        useDeferredLoading: useDeferredLoading,
        useSyntheticPackage: useSyntheticPackage,
        useNamedParameters: useNamedParameters,
      ),
    );
    return super.setUpIn(dir);
  }

  @override
  final String pubspec = '''
name: test_l10n_project
environment:
  sdk: ^3.7.0-0

dependencies:
  flutter:
    sdk: flutter
  flutter_localizations:
    sdk: flutter
  intl: any # Pick up the pinned version from flutter_localizations
''';

  String? _main;

  @override
  String get main => _main ??= (useNamedParameters ? _getMainWithNamedParameters() : _getMain());

  final bool useNamedParameters;

  final String appEn = r'''
{
  "@@locale": "en",

  "helloWorld": "Hello World",
  "@helloWorld": {
    "description": "The conventional newborn programmer greeting"
  },

  "helloNewlineWorld": "Hello \n World",
  "@helloNewlineWorld": {
    "description": "The JSON decoder should convert backslash-n to a newline character in the generated Dart string."
  },

  "testDollarSign": "Hello $ World",
  "@testDollarSign": {
    "description": "The generated Dart String should handle the dollar sign correctly."
  },

  "hello": "Hello {world}",
  "@hello": {
    "description": "A message with a single parameter",
    "placeholders": {
      "world": {}
    }
  },

  "greeting": "{hello} {world}",
  "@greeting": {
    "description": "A message with a two parameters",
    "placeholders": {
      "hello": {},
      "world": {}
    }
  },

  "helloWorldOn": "Hello World on {date}",
  "@helloWorldOn": {
    "description": "A message with a date parameter",
    "placeholders": {
      "date": {
        "type": "DateTime",
        "format": "yMMMMEEEEd"
      }
    }
  },

  "helloWorldDuring": "Hello World from {startDate} to {endDate}",
  "@helloWorldDuring": {
    "description": "A message with two date parameters",
    "placeholders": {
      "startDate": {
        "type": "DateTime",
        "format": "y"
      },
      "endDate": {
        "type": "DateTime",
        "format": "y"
      }
    }
  },

  "helloOn": "Hello {world} on {date} at {time}",
  "@helloOn": {
    "description": "A message with date and string parameters",
    "placeholders": {
      "world": {
      },
      "date": {
        "type": "DateTime",
        "format": "yMd"
      },
      "time": {
        "type": "DateTime",
        "format": "Hm"
      }
    }
  },

  "helloFor": "Hello for {value}",
  "@helloFor": {
    "description": "A message with a double parameter",
    "placeholders": {
      "value": {
        "type": "double",
        "format": "compact"
      }
    }
  },

  "helloCost": "Hello for {price} {value}",
  "@helloCost": {
    "description": "A message with string and int (currency) parameters",
    "placeholders": {
      "price": {
      },
      "value": {
        "type": "int",
        "format": "currency"
      }
    }
  },

  "helloCostWithOptionalParam": "Hello for {price} {value} (with optional param)",
  "@helloCostWithOptionalParam": {
    "description": "A message with string and int (currency) parameters",
    "placeholders": {
      "price": {},
      "value": {
        "type": "double",
        "format": "currency",
        "optionalParameters": {
          "name": "BTC"
        }
      }
    }
  },

  "helloCostWithSpecialCharacter1": "Hello for {price} {value} (with special character)",
  "@helloCostWithSpecialCharacter1": {
    "description": "A message with string and int (currency) parameters",
    "placeholders": {
      "price": {},
      "value": {
        "type": "double",
        "format": "currency",
        "optionalParameters": {
          "name": "BTC'"
        }
      }
    }
  },

  "helloCostWithSpecialCharacter2": "Hello for {price} {value} (with special character)",
  "@helloCostWithSpecialCharacter2": {
    "description": "A message with string and int (currency) parameters",
    "placeholders": {
      "price": {},
      "value": {
        "type": "double",
        "format": "currency",
        "optionalParameters": {
          "name": "BTC\""
        }
      }
    }
  },

  "helloCostWithSpecialCharacter3": "Hello for {price} {value} (with special character)",
  "@helloCostWithSpecialCharacter3": {
    "description": "A message with string and int (currency) parameters",
    "placeholders": {
      "price": {},
      "value": {
        "type": "double",
        "format": "currency",
        "optionalParameters": {
          "name": "BTC\"'"
        }
      }
    }
  },

  "helloDecimalPattern": "Hello for Decimal Pattern {value}",
  "@helloDecimalPattern": {
    "description": "A message which displays a number in decimal pattern",
    "placeholders": {
      "value": {
        "type": "double",
        "format": "decimalPattern"
      }
    }
  },

  "helloPercentPattern": "Hello for Percent Pattern {value}",
  "@helloPercentPattern": {
    "description": "A message which displays a number in percent pattern",
    "placeholders": {
      "value": {
        "type": "double",
        "format": "percentPattern"
      }
    }
  },

  "helloScientificPattern": "Hello for Scientific Pattern {value}",
  "@helloScientificPattern": {
    "description": "A message which displays scientific notation of a number",
    "placeholders": {
      "value": {
        "type": "double",
        "format": "scientificPattern"
      }
    }
  },

  "helloWorlds": "{count,plural, =0{Hello} =1{Hello World} =2{Hello two worlds} few{Hello {count} worlds} many{Hello all {count} worlds} other{Hello other {count} worlds}}",
  "@helloWorlds": {
    "description": "A plural message",
    "placeholders": {
      "count": {}
    }
  },

  "helloAdjectiveWorlds": "{count,plural, =0{Hello} =1{Hello {adjective} World} =2{Hello two {adjective} worlds} other{Hello other {count} {adjective} worlds}}",
  "@helloAdjectiveWorlds": {
    "description": "A plural message with an additional parameter",
    "placeholders": {
      "count": {},
      "adjective": {}
    }
  },

  "helloWorldsOn": "{count,plural, =0{Hello on {date}} =1{Hello World, on {date}} =2{Hello two worlds, on {date}} other{Hello other {count} worlds, on {date}}}",
  "@helloWorldsOn": {
    "description": "A plural message with an additional date parameter",
    "placeholders": {
      "count": {},
      "date": {
        "type": "DateTime",
        "format": "yMMMMEEEEd"
      }
    }
  },

  "helloWorldPopulation": "{count,plural, =1{Hello World of {population} citizens} =2{Hello two worlds with {population} total citizens} many{Hello all {count} worlds, with a total of {population} citizens} other{Hello other {count} worlds, with a total of {population} citizens}}",
  "@helloWorldPopulation": {
    "description": "A plural message with an additional integer parameter",
    "placeholders": {
      "count": {},
      "population": {
        "type": "int",
        "format": "compactLong"
      }
    }
  },

  "helloWorldInterpolation": "[{hello}] #{world}#",
  "@helloWorldInterpolation": {
    "description": "A message with parameters that need string interpolation braces",
    "placeholders": {
      "hello": {},
      "world": {}
    }
  },

  "helloWorldsInterpolation": "{count,plural, other {[{hello}] -{world}- #{count}#}}",
  "@helloWorldsInterpolation": {
    "description": "A plural message with parameters that need string interpolation braces",
    "placeholders": {
      "count": {},
      "hello": {},
      "world": {}
    }
  },

  "dollarSign": "$!",
  "@dollarSign": {
    "description": "A message with a dollar sign."
  },

  "dollarSignPlural": "{count,plural, =1{One $} other{Many $}}",
  "@dollarSignPlural": {
    "description": "A plural message with a dollar sign.",
    "placeholders": {
      "count": {}
    }
  },

  "singleQuote": "Flutter's amazing!",
  "@singleQuote": {
    "description": "A message with a single quote."
  },

  "singleQuotePlural": "{count,plural, =1{Flutter's amazing, times 1!} other{Flutter's amazing, times {count}!}}",
  "@singleQuotePlural": {
    "description": "A plural message with a single quote.",
    "placeholders": {
      "count": {}
    }
  },

  "doubleQuote": "Flutter is \"amazing\"!",
  "@doubleQuote": {
    "description": "A message with double quotes."
  },

  "doubleQuotePlural": "{count,plural, =1{Flutter is \"amazing\", times 1!} other{Flutter is \"amazing\", times {count}!}}",
  "@doubleQuotePlural": {
    "description": "A plural message with double quotes.",
    "placeholders": {
      "count": {}
    }
  },

  "vehicleSelect": "{vehicleType, select, sedan{Sedan} cabriolet{Solid roof cabriolet} truck{16 wheel truck} other{Other}}",
  "@vehicleSelect": {
    "description": "A select message.",
    "placeholders": {
      "vehicleType": {}
    }
  },

  "singleQuoteSelect": "{vehicleType, select, sedan{Sedan's elegance} cabriolet{Cabriolet's acceleration} truck{truck's heavy duty} other{Other's mirrors!}}",
  "@singleQuoteSelect": {
    "description": "A select message with a single quote.",
    "placeholders": {
      "vehicleType": {}
    }
  },

  "doubleQuoteSelect": "{vehicleType, select, sedan{Sedan has \"elegance\"} cabriolet{Cabriolet has \"acceleration\"} truck{truck is \"heavy duty\"} other{Other have \"mirrors\"!}}",
  "@doubleQuoteSelect": {
    "description": "A select message with double quote.",
    "placeholders": {
      "vehicleType": {}
    }
  },

  "pluralInString": "Oh, she found {count, plural, =1 {1 item} other {all {count} items} }!",
  "@pluralInString": {
    "description": "A plural message with prefix and suffix strings.",
    "placeholders": {
      "count": {}
    }
  },

  "selectInString": "Indeed, {gender, select, male {he likes} female {she likes} other {they like} } Flutter!",
  "@selectInString": {
    "description": "A select message with prefix and suffix strings.",
    "placeholders": {
      "gender": {}
    }
  },

  "selectWithPlaceholder": "Indeed, {gender, select, male {he likes {preference}} female {she likes {preference}} other {they like {preference}}}!",
  "@selectWithPlaceholder": {
    "description": "A select message with prefix, suffix strings, and a placeholder.",
    "placeholders": {
      "gender": {},
      "preference": {}
    }
  },

  "selectInPlural": "{count, plural, =1{{gender, select, male{he} female{she} other{they}}} other{they}}",
  "@selectInPlural": {
    "description": "Pronoun dependent on the count and gender.",
    "placeholders": {
      "gender": {
        "type": "String"
      },
      "count": {
        "type": "num"
      }
    }
  },
  "datetime1": "{today, date, ::yMd}",
  "datetime2": "{current, time, ::jms}",
  "datetimeAddedFormats": "{firstDate} and {secondDate}",
  "@datetimeAddedFormats": {
    "description": "A message with two dates, with added formats",
    "placeholders": {
      "firstDate": {
        "type": "DateTime",
        "format": "yMd+jms"
      },
      "secondDate": {
        "type": "DateTime",
        "format": "yMMMMEEEEd+Hms+QQQQ"
      }
    }
  }
}
''';

  final String appEnCa = r'''
{
  "@@locale": "en_CA",
  "helloWorld": "CA Hello World"
}
''';

  final String appEnGb = r'''
{
  "@@locale": "en_GB",
  "helloWorld": "GB Hello World"
}
''';

  // All these messages are the template language's message with 'ES - '
  // appended. This makes validating test behavior easier. The interpolated
  // messages are different where applicable.
  final String appEs = r'''
{
  "@@locale": "es",
  "helloWorld": "ES - Hello world",
  "helloWorlds": "{count,plural, =0{ES - Hello} =1{ES - Hello World} =2{ES - Hello two worlds} few{ES - Hello {count} worlds} many{ES - Hello all {count} worlds} other{ES - Hello other {count} worlds}}",
  "helloNewlineWorld": "ES - Hello \n World",
  "testDollarSign": "ES - Hola $ Mundo",
  "hello": "ES - Hello {world}",
  "greeting": "ES - {hello} {world}",
  "helloWorldOn": "ES - Hello World on {date}",
  "helloWorldDuring": "ES - Hello World from {startDate} to {endDate}",
  "helloOn": "ES - Hello {world} on {date} at {time}",
  "helloFor": "ES - Hello for {value}",
  "helloAdjectiveWorlds": "{count,plural, =0{ES - Hello} =1{ES - Hello {adjective} World} =2{ES - Hello two {adjective} worlds} other{ES - Hello other {count} {adjective} worlds}}",
  "helloWorldsOn": "{count,plural, =0{ES - Hello on {date}} =1{ES - Hello World, on {date}} =2{ES - Hello two worlds, on {date}} other{ES - Hello other {count} worlds, on {date}}}",
  "helloWorldPopulation": "{count,plural, =1{ES - Hello World of {population} citizens} =2{ES - Hello two worlds with {population} total citizens} many{ES - Hello all {count} worlds, with a total of {population} citizens} other{ES - Hello other {count} worlds, with a total of {population} citizens}}",
  "helloWorldInterpolation": "ES - [{hello}] #{world}#",
  "helloWorldsInterpolation": "{count,plural, other {ES - [{hello}] -{world}- #{count}#}}",
  "dollarSign": "ES - $!",
  "dollarSignPlural": "{count,plural, =1{ES - One $} other{ES - Many $}}",
  "singleQuote": "ES - Flutter's amazing!",
  "singleQuotePlural": "{count,plural, =1{ES - Flutter's amazing, times 1!} other{ES - Flutter's amazing, times {count}!}}",
  "doubleQuote": "ES - Flutter is \"amazing\"!",
  "doubleQuotePlural": "{count,plural, =1{ES - Flutter is \"amazing\", times 1!} other{ES - Flutter is \"amazing\", times {count}!}}",
  "vehicleSelect": "{vehicleType, select, sedan{ES - Sedan} cabriolet{ES - Solid roof cabriolet} truck{ES - 16 wheel truck} other{ES - Other}}",
  "singleQuoteSelect": "{vehicleType, select, sedan{ES - Sedan's elegance} cabriolet{ES - Cabriolet' acceleration} truck{ES - truck's heavy duty} other{ES - Other's mirrors!}}",
  "doubleQuoteSelect": "{vehicleType, select, sedan{ES - Sedan has \"elegance\"} cabriolet{ES - Cabriolet has \"acceleration\"} truck{ES - truck is \"heavy duty\"} other{ES - Other have \"mirrors\"!}}",
  "pluralInString": "ES - Oh, she found {count, plural, =1 {ES - 1 item} other {ES - all {count} items} }ES - !",
  "selectInString": "ES - Indeed, {gender, select, male {ES - he likes} female {ES - she likes} other {ES - they like} } ES - Flutter!"
}
''';

  final String appEs419 = r'''
{
  "@@locale": "es_419",
  "helloWorld": "ES 419 - Hello World",
  "helloWorlds": "{count,plural, =0{ES 419 - Hello} =1{ES 419 - Hello World} =2{ES 419 - Hello two worlds} few{ES 419 - Hello {count} worlds} many{ES 419 - Hello all {count} worlds} other{ES - Hello other {count} worlds}}"
}
''';

  final String appZh = r'''
{
  "@@locale": "zh",
  "helloWorld": "你好世界",
  "helloWorlds": "{count,plural, =0{你好} =1{你好世界} other{你好{count}个其他世界}}",
  "helloCost": "zh - Hello for {price} {value}"
}
''';

  final String appZhHans = r'''
{
  "@@locale": "zh_Hans",
  "helloWorld": "简体你好世界"
}
  ''';

  final String appZhHant = r'''
{
  "@@locale": "zh_Hant",
  "helloWorld": "繁體你好世界"
}
  ''';

  final String appZhHantTw = r'''
{
  "@@locale": "zh_Hant_TW",
  "helloWorld": "台灣繁體你好世界"
}
''';

  String _getMain() => r'''
import 'package:flutter/material.dart';

import 'l10n/app_localizations.dart';

class LocaleBuilder extends StatelessWidget {
  const LocaleBuilder({
    Key? key,
    this.locale,
    this.test,
    required this.callback,
  }) : super(key: key);

  final Locale? locale;
  final String? test;
  final void Function (BuildContext context) callback;

  @override build(BuildContext context) {
    return Localizations.override(
      locale: locale,
      context: context,
      child: ResultBuilder(
        test: test,
        callback: callback,
      ),
    );
  }
}

class ResultBuilder extends StatelessWidget {
  const ResultBuilder({
    Key? key,
    this.test,
    required this.callback,
  }) : super(key: key);

  final String? test;
  final void Function (BuildContext context) callback;

  @override build(BuildContext context) {
    return Builder(
      builder: (BuildContext context) {
        try {
          callback(context);
        } on Exception catch (e) {
          print('#l10n A(n) $e has occurred trying to generate "$test" results.');
          print('#l10n END');
        }
        return Container();
      },
    );
  }
}

class Home extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final List<String> results = [];
    return Row(
      children: <Widget>[
        LocaleBuilder(
          test: 'supportedLocales',
          callback: (BuildContext context) {
            results.add('--- supportedLocales tests ---');
            int n = 0;
            for (Locale locale in AppLocalizations.supportedLocales) {
              String languageCode = locale.languageCode;
              String? countryCode = locale.countryCode;
              String? scriptCode = locale.scriptCode;
              results.add('supportedLocales[$n]: languageCode: $languageCode, countryCode: $countryCode, scriptCode: $scriptCode');
              n += 1;
            }
          },
        ),
        LocaleBuilder(
          locale: Locale('en', 'CA'),
          test: 'countryCode - en_CA',
          callback: (BuildContext context) {
            results.add('--- countryCode (en_CA) tests ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.hello("CA fallback World"));
          },
        ),
        LocaleBuilder(
          locale: Locale('en', 'GB'),
          test: 'countryCode - en_GB',
          callback: (BuildContext context) {
            results.add('--- countryCode (en_GB) tests ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.hello("GB fallback World"));
          },
        ),
        LocaleBuilder(
          locale: Locale('zh'),
          test: 'zh',
          callback: (BuildContext context) {
            results.add('--- zh ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.helloWorlds(0));
            results.add(AppLocalizations.of(context)!.helloWorlds(1));
            results.add(AppLocalizations.of(context)!.helloWorlds(2));
            // Should use the fallback language, in this case,
            // "Hello 世界" should be displayed.
            results.add(AppLocalizations.of(context)!.hello("世界"));
            // helloCost is tested in 'zh' because 'es' currency format contains a
            // non-breaking space character (U+00A0), which if removed,
            // makes it hard to decipher why the test is failing.
            results.add(AppLocalizations.of(context)!.helloCost("价钱", 123));
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans'),
          test: 'zh',
          callback: (BuildContext context) {
            results.add('--- scriptCode: zh_Hans ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
          test: 'scriptCode - zh_Hant',
          callback: (BuildContext context) {
            results.add('--- scriptCode - zh_Hant ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', countryCode: 'TW', scriptCode: 'Hant'),
          test: 'scriptCode - zh_TW_Hant',
          callback: (BuildContext context) {
            results.add('--- scriptCode - zh_Hant_TW ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale('en'),
          test: 'General formatting',
          callback: (BuildContext context) {
            results.add('--- General formatting tests ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll(<String>[
              '${localizations.helloWorld}',
              '${localizations.helloNewlineWorld}',
              '${localizations.testDollarSign}',
              '${localizations.hello("World")}',
              '${localizations.greeting("Hello", "World")}',
              '${localizations.helloWorldOn(DateTime(1960))}',
              '${localizations.helloOn("world argument", DateTime(1960), DateTime(1960))}',
              '${localizations.helloWorldDuring(DateTime(1960), DateTime(2020))}',
              '${localizations.helloFor(123)}',
              '${localizations.helloCost("price", 123)}',
              '${localizations.helloCostWithOptionalParam("price", .5)}',
              '${localizations.helloCostWithSpecialCharacter1("price", .5)}',
              '${localizations.helloCostWithSpecialCharacter2("price", .5)}',
              '${localizations.helloCostWithSpecialCharacter3("price", .5)}',
              '${localizations.helloDecimalPattern(1200000)}',
              '${localizations.helloPercentPattern(1200000)}',
              '${localizations.helloScientificPattern(1200000)}',
              '${localizations.helloWorlds(0)}',
              '${localizations.helloWorlds(1)}',
              '${localizations.helloWorlds(2)}',
              '${localizations.helloAdjectiveWorlds(0, "new")}',
              '${localizations.helloAdjectiveWorlds(1, "new")}',
              '${localizations.helloAdjectiveWorlds(2, "new")}',
              '${localizations.helloWorldsOn(0, DateTime(1960))}',
              '${localizations.helloWorldsOn(1, DateTime(1960))}',
              '${localizations.helloWorldsOn(2, DateTime(1960))}',
              '${localizations.helloWorldPopulation(0, 100)}',
              '${localizations.helloWorldPopulation(1, 101)}',
              '${localizations.helloWorldPopulation(2, 102)}',
              '${localizations.helloWorldsInterpolation(123, "Hello", "World")}',
              '${localizations.dollarSign}',
              '${localizations.dollarSignPlural(1)}',
              '${localizations.singleQuote}',
              '${localizations.singleQuotePlural(2)}',
              '${localizations.doubleQuote}',
              '${localizations.doubleQuotePlural(2)}',
              "${localizations.vehicleSelect('truck')}",
              "${localizations.singleQuoteSelect('sedan')}",
              "${localizations.doubleQuoteSelect('cabriolet')}",
              "${localizations.pluralInString(1)}",
              "${localizations.selectInString('he')}",
              "${localizations.selectWithPlaceholder('male', 'ice cream')}",
              "${localizations.selectWithPlaceholder('female', 'chocolate')}",
              "${localizations.selectInPlural('male', 1)}",
              "${localizations.selectInPlural('male', 2)}",
              "${localizations.selectInPlural('female', 1)}",
              '${localizations.datetime1(DateTime(2023, 6, 26))}',
              '${localizations.datetime2(DateTime(2023, 6, 26, 5, 23))}',
              '${localizations.datetimeAddedFormats(DateTime(2024, 10, 6, 23, 29, 48), DateTime(2000, 7, 4, 12, 54, 32))}',
            ]);
          },
        ),
        LocaleBuilder(
          locale: Locale('es'),
          test: '--- es ---',
          callback: (BuildContext context) {
            results.add('--- es ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll(<String>[
              '${localizations.helloWorld}',
              '${localizations.helloNewlineWorld}',
              '${localizations.testDollarSign}',
              '${localizations.hello("Mundo")}',
              '${localizations.greeting("Hola", "Mundo")}',
              '${localizations.helloWorldOn(DateTime(1960))}',
              '${localizations.helloOn("world argument", DateTime(1960), DateTime(1960))}',
              '${localizations.helloWorldDuring(DateTime(1960), DateTime(2020))}',
              '${localizations.helloFor(123)}',
              // helloCost is tested in 'zh' because 'es' currency format contains a
              // non-breaking space character (U+00A0), which if removed,
              // makes it hard to decipher why the test is failing.
              '${localizations.helloWorlds(0)}',
              '${localizations.helloWorlds(1)}',
              '${localizations.helloWorlds(2)}',
              '${localizations.helloAdjectiveWorlds(0, "nuevo")}',
              '${localizations.helloAdjectiveWorlds(1, "nuevo")}',
              '${localizations.helloAdjectiveWorlds(2, "nuevo")}',
              '${localizations.helloWorldsOn(0, DateTime(1960))}',
              '${localizations.helloWorldsOn(1, DateTime(1960))}',
              '${localizations.helloWorldsOn(2, DateTime(1960))}',
              '${localizations.helloWorldPopulation(0, 100)}',
              '${localizations.helloWorldPopulation(1, 101)}',
              '${localizations.helloWorldPopulation(2, 102)}',
              '${localizations.helloWorldsInterpolation(123, "Hola", "Mundo")}',
              '${localizations.dollarSign}',
              '${localizations.dollarSignPlural(1)}',
              '${localizations.singleQuote}',
              '${localizations.singleQuotePlural(2)}',
              '${localizations.doubleQuote}',
              '${localizations.doubleQuotePlural(2)}',
              "${localizations.vehicleSelect('truck')}",
              "${localizations.singleQuoteSelect('sedan')}",
              "${localizations.doubleQuoteSelect('cabriolet')}",
              "${localizations.pluralInString(1)}",
              "${localizations.selectInString('he')}",
            ]);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'es', countryCode: '419'),
          test: 'countryCode - es_419',
          callback: (BuildContext context) {
            results.add('--- es_419 ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll([
              '${localizations.helloWorld}',
              '${localizations.helloWorlds(0)}',
              '${localizations.helloWorlds(1)}',
              '${localizations.helloWorlds(2)}',
            ]);
          },
        ),
        LocaleBuilder(
          callback: (BuildContext context) {
            try {
              int n = 0;
              for (final String result in results) {
                // Newline character replacement is necessary because
                // the stream breaks up stdout by new lines.
                print('#l10n $n (${result.replaceAll('\n', '_NEWLINE_')})');
                n += 1;
              }
            }
            finally {
              print('#l10n END');
            }
          },
        ),
      ],
    );
  }
}

void main() {
  runApp(
    MaterialApp(
      localizationsDelegates: AppLocalizations.localizationsDelegates,
      supportedLocales: AppLocalizations.supportedLocales,
      home: Home(),
    ),
  );
}
''';

  String _getMainWithNamedParameters() => r'''
import 'package:flutter/material.dart';

import 'l10n/app_localizations.dart';

class LocaleBuilder extends StatelessWidget {
  const LocaleBuilder({
    Key? key,
    this.locale,
    this.test,
    required this.callback,
  }) : super(key: key);

  final Locale? locale;
  final String? test;
  final void Function (BuildContext context) callback;

  @override build(BuildContext context) {
    return Localizations.override(
      locale: locale,
      context: context,
      child: ResultBuilder(
        test: test,
        callback: callback,
      ),
    );
  }
}

class ResultBuilder extends StatelessWidget {
  const ResultBuilder({
    Key? key,
    this.test,
    required this.callback,
  }) : super(key: key);

  final String? test;
  final void Function (BuildContext context) callback;

  @override build(BuildContext context) {
    return Builder(
      builder: (BuildContext context) {
        try {
          callback(context);
        } on Exception catch (e) {
          print('#l10n A(n) $e has occurred trying to generate "$test" results.');
          print('#l10n END');
        }
        return Container();
      },
    );
  }
}

class Home extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final List<String> results = [];
    return Row(
      children: <Widget>[
        LocaleBuilder(
          test: 'supportedLocales',
          callback: (BuildContext context) {
            results.add('--- supportedLocales tests ---');
            int n = 0;
            for (Locale locale in AppLocalizations.supportedLocales) {
              String languageCode = locale.languageCode;
              String? countryCode = locale.countryCode;
              String? scriptCode = locale.scriptCode;
              results.add('supportedLocales[$n]: languageCode: $languageCode, countryCode: $countryCode, scriptCode: $scriptCode');
              n += 1;
            }
          },
        ),
        LocaleBuilder(
          locale: Locale('en', 'CA'),
          test: 'countryCode - en_CA',
          callback: (BuildContext context) {
            results.add('--- countryCode (en_CA) tests ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.hello(world: "CA fallback World"));
          },
        ),
        LocaleBuilder(
          locale: Locale('en', 'GB'),
          test: 'countryCode - en_GB',
          callback: (BuildContext context) {
            results.add('--- countryCode (en_GB) tests ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.hello(world: "GB fallback World"));
          },
        ),
        LocaleBuilder(
          locale: Locale('zh'),
          test: 'zh',
          callback: (BuildContext context) {
            results.add('--- zh ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
            results.add(AppLocalizations.of(context)!.helloWorlds(count: 0));
            results.add(AppLocalizations.of(context)!.helloWorlds(count: 1));
            results.add(AppLocalizations.of(context)!.helloWorlds(count: 2));
            // Should use the fallback language, in this case,
            // "Hello 世界" should be displayed.
            results.add(AppLocalizations.of(context)!.hello(world: "世界"));
            // helloCost is tested in 'zh' because 'es' currency format contains a
            // non-breaking space character (U+00A0), which if removed,
            // makes it hard to decipher why the test is failing.
            results.add(AppLocalizations.of(context)!.helloCost(price: "价钱", value: 123));
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans'),
          test: 'zh',
          callback: (BuildContext context) {
            results.add('--- scriptCode: zh_Hans ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
          test: 'scriptCode - zh_Hant',
          callback: (BuildContext context) {
            results.add('--- scriptCode - zh_Hant ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'zh', countryCode: 'TW', scriptCode: 'Hant'),
          test: 'scriptCode - zh_TW_Hant',
          callback: (BuildContext context) {
            results.add('--- scriptCode - zh_Hant_TW ---');
            results.add(AppLocalizations.of(context)!.helloWorld);
          },
        ),
        LocaleBuilder(
          locale: Locale('en'),
          test: 'General formatting',
          callback: (BuildContext context) {
            results.add('--- General formatting tests ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll(<String>[
              '${localizations.helloWorld}',
              '${localizations.helloNewlineWorld}',
              '${localizations.testDollarSign}',
              '${localizations.hello(world: "World")}',
              '${localizations.greeting(hello: "Hello", world: "World")}',
              '${localizations.helloWorldOn(date: DateTime(1960))}',
              '${localizations.helloOn(world: "world argument", date: DateTime(1960), time: DateTime(1960))}',
              '${localizations.helloWorldDuring(startDate: DateTime(1960), endDate: DateTime(2020))}',
              '${localizations.helloFor(value: 123)}',
              '${localizations.helloCost(price: "price", value: 123)}',
              '${localizations.helloCostWithOptionalParam(price: "price", value: .5)}',
              '${localizations.helloCostWithSpecialCharacter1(price: "price", value: .5)}',
              '${localizations.helloCostWithSpecialCharacter2(price: "price", value: .5)}',
              '${localizations.helloCostWithSpecialCharacter3(price: "price", value: .5)}',
              '${localizations.helloDecimalPattern(value: 1200000)}',
              '${localizations.helloPercentPattern(value: 1200000)}',
              '${localizations.helloScientificPattern(value: 1200000)}',
              '${localizations.helloWorlds(count: 0)}',
              '${localizations.helloWorlds(count: 1)}',
              '${localizations.helloWorlds(count: 2)}',
              '${localizations.helloAdjectiveWorlds(count: 0, adjective: "new")}',
              '${localizations.helloAdjectiveWorlds(count: 1, adjective: "new")}',
              '${localizations.helloAdjectiveWorlds(count: 2, adjective: "new")}',
              '${localizations.helloWorldsOn(count: 0, date: DateTime(1960))}',
              '${localizations.helloWorldsOn(count: 1, date: DateTime(1960))}',
              '${localizations.helloWorldsOn(count: 2, date: DateTime(1960))}',
              '${localizations.helloWorldPopulation(count: 0, population: 100)}',
              '${localizations.helloWorldPopulation(count: 1, population: 101)}',
              '${localizations.helloWorldPopulation(count: 2, population: 102)}',
              '${localizations.helloWorldsInterpolation(count: 123, hello: "Hello", world: "World")}',
              '${localizations.dollarSign}',
              '${localizations.dollarSignPlural(count: 1)}',
              '${localizations.singleQuote}',
              '${localizations.singleQuotePlural(count: 2)}',
              '${localizations.doubleQuote}',
              '${localizations.doubleQuotePlural(count: 2)}',
              "${localizations.vehicleSelect(vehicleType: 'truck')}",
              "${localizations.singleQuoteSelect(vehicleType: 'sedan')}",
              "${localizations.doubleQuoteSelect(vehicleType: 'cabriolet')}",
              "${localizations.pluralInString(count: 1)}",
              "${localizations.selectInString(gender: 'he')}",
              "${localizations.selectWithPlaceholder(gender: 'male', preference: 'ice cream')}",
              "${localizations.selectWithPlaceholder(gender: 'female', preference: 'chocolate')}",
              "${localizations.selectInPlural(gender: 'male', count: 1)}",
              "${localizations.selectInPlural(gender: 'male', count: 2)}",
              "${localizations.selectInPlural(gender: 'female', count: 1)}",
              '${localizations.datetime1(today: DateTime(2023, 6, 26))}',
              '${localizations.datetime2(current: DateTime(2023, 6, 26, 5, 23))}',
              '${localizations.datetimeAddedFormats(firstDate: DateTime(2024, 10, 6, 23, 29, 48), secondDate: DateTime(2000, 7, 4, 12, 54, 32))}',
            ]);
          },
        ),
        LocaleBuilder(
          locale: Locale('es'),
          test: '--- es ---',
          callback: (BuildContext context) {
            results.add('--- es ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll(<String>[
              '${localizations.helloWorld}',
              '${localizations.helloNewlineWorld}',
              '${localizations.testDollarSign}',
              '${localizations.hello(world: "Mundo")}',
              '${localizations.greeting(hello: "Hola", world: "Mundo")}',
              '${localizations.helloWorldOn(date: DateTime(1960))}',
              '${localizations.helloOn(world: "world argument", date: DateTime(1960), time: DateTime(1960))}',
              '${localizations.helloWorldDuring(startDate: DateTime(1960), endDate: DateTime(2020))}',
              '${localizations.helloFor(value: 123)}',
              // helloCost is tested in 'zh' because 'es' currency format contains a
              // non-breaking space character (U+00A0), which if removed,
              // makes it hard to decipher why the test is failing.
              '${localizations.helloWorlds(count: 0)}',
              '${localizations.helloWorlds(count: 1)}',
              '${localizations.helloWorlds(count: 2)}',
              '${localizations.helloAdjectiveWorlds(count: 0, adjective: "nuevo")}',
              '${localizations.helloAdjectiveWorlds(count: 1, adjective: "nuevo")}',
              '${localizations.helloAdjectiveWorlds(count: 2, adjective: "nuevo")}',
              '${localizations.helloWorldsOn(count: 0, date: DateTime(1960))}',
              '${localizations.helloWorldsOn(count: 1, date: DateTime(1960))}',
              '${localizations.helloWorldsOn(count: 2, date: DateTime(1960))}',
              '${localizations.helloWorldPopulation(count: 0, population: 100)}',
              '${localizations.helloWorldPopulation(count: 1, population: 101)}',
              '${localizations.helloWorldPopulation(count: 2, population: 102)}',
              '${localizations.helloWorldsInterpolation(count: 123, hello: "Hola", world: "Mundo")}',
              '${localizations.dollarSign}',
              '${localizations.dollarSignPlural(count: 1)}',
              '${localizations.singleQuote}',
              '${localizations.singleQuotePlural(count: 2)}',
              '${localizations.doubleQuote}',
              '${localizations.doubleQuotePlural(count: 2)}',
              "${localizations.vehicleSelect(vehicleType: 'truck')}",
              "${localizations.singleQuoteSelect(vehicleType: 'sedan')}",
              "${localizations.doubleQuoteSelect(vehicleType: 'cabriolet')}",
              "${localizations.pluralInString(count: 1)}",
              "${localizations.selectInString(gender: 'he')}",
            ]);
          },
        ),
        LocaleBuilder(
          locale: Locale.fromSubtags(languageCode: 'es', countryCode: '419'),
          test: 'countryCode - es_419',
          callback: (BuildContext context) {
            results.add('--- es_419 ---');
            final AppLocalizations localizations = AppLocalizations.of(context)!;
            results.addAll([
              '${localizations.helloWorld}',
              '${localizations.helloWorlds(count: 0)}',
              '${localizations.helloWorlds(count: 1)}',
              '${localizations.helloWorlds(count: 2)}',
            ]);
          },
        ),
        LocaleBuilder(
          callback: (BuildContext context) {
            try {
              int n = 0;
              for (final String result in results) {
                // Newline character replacement is necessary because
                // the stream breaks up stdout by new lines.
                print('#l10n $n (${result.replaceAll('\n', '_NEWLINE_')})');
                n += 1;
              }
            }
            finally {
              print('#l10n END');
            }
          },
        ),
      ],
    );
  }
}

void main() {
  runApp(
    MaterialApp(
      localizationsDelegates: AppLocalizations.localizationsDelegates,
      supportedLocales: AppLocalizations.supportedLocales,
      home: Home(),
    ),
  );
}''';

  String l10nYaml({
    required bool useDeferredLoading,
    required bool useSyntheticPackage,
    required bool useNamedParameters,
  }) {
    String l10nYamlString = '';

    if (useDeferredLoading) {
      l10nYamlString += 'use-deferred-loading: true\n';
    }

    if (!useSyntheticPackage) {
      l10nYamlString += 'synthetic-package: false\n';
    }

    if (useNamedParameters) {
      l10nYamlString += 'use-named-parameters: true\n';
    }

    return l10nYamlString;
  }
}
