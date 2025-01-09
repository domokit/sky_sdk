// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:flutter_tools/executable.dart' as executable;
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/analyze.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:flutter_tools/src/runner/flutter_command_runner.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/testbed.dart';
import 'runner/utils.dart';

void main() {
  setUpAll(() {
    Cache.disableLocking();
  });

  tearDownAll(() {
    Cache.enableLocking();
  });

  test(
    'Help for command line arguments is consistently styled and complete',
    () => Testbed().run(() {
      final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);
      executable.generateCommands(verboseHelp: true, verbose: true).forEach(runner.addCommand);
      verifyCommandRunner(runner);
      for (final Command<void> command in runner.commands.values) {
        if (command.name == 'analyze') {
          final AnalyzeCommand analyze = command as AnalyzeCommand;
          expect(analyze.allProjectValidators().length, 2);
        }
      }
    }),
  );

  testUsingContext('Global arg results are available in FlutterCommands', () async {
    final DummyFlutterCommand command = DummyFlutterCommand(
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );

    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);

    runner.addCommand(command);
    await runner.run(<String>['dummy', '--${FlutterGlobalOptions.kContinuousIntegrationFlag}']);

    expect(command.globalResults, isNotNull);
    expect(command.boolArg(FlutterGlobalOptions.kContinuousIntegrationFlag, global: true), true);
  });

  testUsingContext('Global arg results are available in FlutterCommands sub commands', () async {
    final DummyFlutterCommand command = DummyFlutterCommand(
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );

    final DummyFlutterCommand subcommand = DummyFlutterCommand(
      name: 'sub',
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );

    command.addSubcommand(subcommand);

    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);

    runner.addCommand(command);
    runner.addCommand(subcommand);
    await runner.run(<String>[
      'dummy',
      'sub',
      '--${FlutterGlobalOptions.kContinuousIntegrationFlag}',
    ]);

    expect(subcommand.globalResults, isNotNull);
    expect(subcommand.boolArg(FlutterGlobalOptions.kContinuousIntegrationFlag, global: true), true);
  });

  testUsingContext('bool? safe argResults', () async {
    final DummyFlutterCommand command = DummyFlutterCommand(
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );
    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);
    command.argParser.addFlag('key');
    command.argParser.addFlag('key-false');
    // argResults will be null at this point, if attempt to read them is made,
    // exception `Null check operator used on a null value` would be thrown.
    expect(() => command.boolArg('key'), throwsA(const TypeMatcher<TypeError>()));

    runner.addCommand(command);
    await runner.run(<String>['dummy', '--key']);

    expect(command.boolArg('key'), true);
    expect(() => command.boolArg('non-existent'), throwsArgumentError);

    expect(command.boolArg('key'), true);
    expect(() => command.boolArg('non-existent'), throwsA(const TypeMatcher<ArgumentError>()));

    expect(command.boolArg('key-false'), false);
    expect(command.boolArg('key-false'), false);
  });

  testUsingContext('String? safe argResults', () async {
    final DummyFlutterCommand command = DummyFlutterCommand(
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );
    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);
    command.argParser.addOption('key');
    // argResults will be null at this point, if attempt to read them is made,
    // exception `Null check operator used on a null value` would be thrown
    expect(() => command.stringArg('key'), throwsA(const TypeMatcher<TypeError>()));

    runner.addCommand(command);
    await runner.run(<String>['dummy', '--key=value']);

    expect(command.stringArg('key'), 'value');
    expect(() => command.stringArg('non-existent'), throwsArgumentError);

    expect(command.stringArg('key'), 'value');
    expect(() => command.stringArg('non-existent'), throwsA(const TypeMatcher<ArgumentError>()));
  });

  testUsingContext('List<String> safe argResults', () async {
    final DummyFlutterCommand command = DummyFlutterCommand(
      commandFunction: () async {
        return const FlutterCommandResult(ExitStatus.success);
      },
    );
    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: true);
    command.argParser.addMultiOption('key', allowed: <String>['a', 'b', 'c']);
    // argResults will be null at this point, if attempt to read them is made,
    // exception `Null check operator used on a null value` would be thrown.
    expect(() => command.stringsArg('key'), throwsA(const TypeMatcher<TypeError>()));

    runner.addCommand(command);
    await runner.run(<String>['dummy', '--key', 'a']);

    // throws error when trying to parse non-existent key.
    expect(() => command.stringsArg('non-existent'), throwsA(const TypeMatcher<ArgumentError>()));

    expect(command.stringsArg('key'), <String>['a']);

    await runner.run(<String>['dummy', '--key', 'a', '--key', 'b']);
    expect(command.stringsArg('key'), <String>['a', 'b']);

    await runner.run(<String>['dummy']);
    expect(command.stringsArg('key'), <String>[]);
  });
}

void verifyCommandRunner(CommandRunner<Object?> runner) {
  expect(runner.argParser, isNotNull, reason: '${runner.runtimeType} has no argParser');
  expect(runner.argParser.allowsAnything, isFalse, reason: '${runner.runtimeType} allows anything');
  expect(
    runner.argParser.allowTrailingOptions,
    isFalse,
    reason: '${runner.runtimeType} allows trailing options',
  );
  verifyOptions(null, runner.argParser.options.values);
  runner.commands.values.forEach(verifyCommand);
}

void verifyCommand(Command<Object?> runner) {
  expect(runner.argParser, isNotNull, reason: 'command ${runner.name} has no argParser');
  verifyOptions(runner.name, runner.argParser.options.values);

  final String firstDescriptionLine = runner.description.split('\n').first;
  expect(
    firstDescriptionLine,
    matches(_allowedTrailingPatterns),
    reason:
        "command ${runner.name}'s description does not end with the expected single period that a full sentence should end with",
  );

  if (!runner.hidden && runner.parent == null) {
    expect(
      runner.category,
      anyOf(
        FlutterCommandCategory.sdk,
        FlutterCommandCategory.project,
        FlutterCommandCategory.tools,
      ),
      reason: "top-level command ${runner.name} doesn't have a valid category",
    );
  }

  runner.subcommands.values.forEach(verifyCommand);
}

// Patterns for arguments names.
final RegExp _allowedArgumentNamePattern = RegExp(r'^([-a-z0-9]+)$');
final RegExp _allowedArgumentNamePatternForPrecache = RegExp(r'^([-a-z0-9_]+)$');
final RegExp _bannedArgumentNamePattern = RegExp(r'-uri$');

// Patterns for help messages.
final RegExp _bannedLeadingPatterns = RegExp(r'^[-a-z]', multiLine: true);
final RegExp _allowedTrailingPatterns = RegExp(r'([^ ]([^.^!^:][.!:])\)?|: https?://[^ ]+[^.]|^)$');
final RegExp _bannedQuotePatterns = RegExp(r" '|' |'\.|\('|'\)|`");
final RegExp _bannedArgumentReferencePatterns = RegExp(r'[^"=]--[^ ]');
final RegExp _questionablePatterns = RegExp(r'[a-z]\.[A-Z]');
final RegExp _bannedUri = RegExp(r'\b[Uu][Rr][Ii]\b');
final RegExp _nonSecureFlutterDartUrl = RegExp(
  r'http://([a-z0-9-]+\.)*(flutter|dart)\.dev',
  caseSensitive: false,
);
const String _needHelp =
    "Every option must have help explaining what it does, even if it's "
    'for testing purposes, because this is the bare minimum of '
    'documentation we can add just for ourselves. If it is not intended '
    'for developers, then use "hide: !verboseHelp" to only show the '
    'help when people run with "--help --verbose".';

const String _header = ' Comment: ';

void verifyOptions(String? command, Iterable<Option> options) {
  String target;
  if (command == null) {
    target = 'the global argument "';
  } else {
    target = '"flutter $command ';
  }
  assert(target.contains('"'));
  for (final Option option in options) {
    // If you think you need to add an exception here, please ask Hixie (but he'll say no).
    if (command == 'precache') {
      expect(
        option.name,
        matches(_allowedArgumentNamePatternForPrecache),
        reason:
            '$_header$target--${option.name}" is not a valid name for a command line argument. (Is it all lowercase?)',
      );
    } else {
      expect(
        option.name,
        matches(_allowedArgumentNamePattern),
        reason:
            '$_header$target--${option.name}" is not a valid name for a command line argument. (Is it all lowercase? Does it use hyphens rather than underscores?)',
      );
    }
    expect(
      option.name,
      isNot(matches(_bannedArgumentNamePattern)),
      reason:
          '$_header$target--${option.name}" is not a valid name for a command line argument. (We use "--foo-url", not "--foo-uri", for example.)',
    );

    // Deprecated options and flags should be hidden but still have help text.
    const List<String> deprecatedOptions = <String>[
      FlutterOptions.kNullSafety,
      FlutterOptions.kNullAssertions,
    ];
    final bool isOptionDeprecated = deprecatedOptions.contains(option.name);
    if (!isOptionDeprecated) {
      expect(
        option.hide,
        isFalse,
        reason:
            '${_header}Option "--${option.name}" for "flutter $command" should not be hidden. $_needHelp',
      );
    } else {
      expect(
        option.hide,
        isTrue,
        reason:
            '${_header}Deprecated option "--${option.name}" for "flutter $command" should be hidden. $_needHelp',
      );
    }

    expect(
      option.help,
      isNotNull,
      reason: '${_header}Help for $target--${option.name}" has null help. $_needHelp',
    );
    expect(
      option.help,
      isNotEmpty,
      reason: '${_header}Help for $target--${option.name}" has empty help. $_needHelp',
    );
    expect(
      option.help,
      isNot(matches(_bannedLeadingPatterns)),
      reason:
          '${_header}A line in the help for $target--${option.name}" starts with a lowercase letter. For stylistic consistency, all help messages must start with a capital letter.',
    );
    expect(
      option.help,
      isNot(startsWith('(Deprecated')),
      reason:
          '${_header}Help for $target--${option.name}" should start with lowercase "(deprecated)" for consistency with other deprecated commands.',
    );
    expect(
      option.help,
      isNot(startsWith('(Required')),
      reason:
          '${_header}Help for $target--${option.name}" should start with lowercase "(required)" for consistency with other deprecated commands.',
    );
    expect(
      option.help,
      isNot(contains('?')),
      reason:
          '${_header}Help for $target--${option.name}" has a question mark. Generally we prefer the passive voice for help messages.',
    );
    expect(
      option.help,
      isNot(contains('Note:')),
      reason:
          '${_header}Help for $target--${option.name}" uses "Note:". See our style guide entry about "empty prose".',
    );
    expect(
      option.help,
      isNot(contains('Note that')),
      reason:
          '${_header}Help for $target--${option.name}" uses "Note that". See our style guide entry about "empty prose".',
    );
    expect(
      option.help,
      isNot(matches(_bannedQuotePatterns)),
      reason:
          '${_header}Help for $target--${option.name}" uses single quotes or backticks instead of double quotes in the help message. For consistency we use double quotes throughout.',
    );
    expect(
      option.help,
      isNot(matches(_questionablePatterns)),
      reason:
          '${_header}Help for $target--${option.name}" may have a typo. (If it does not you may have to update args_test.dart, sorry. Search for "_questionablePatterns")',
    );
    if (option.defaultsTo != null) {
      expect(
        option.help,
        isNot(contains('Default')),
        reason:
            '${_header}Help for $target--${option.name}" mentions the default value but that is redundant with the defaultsTo option which is also specified (and preferred).',
      );

      final Map<String, String>? allowedHelp = option.allowedHelp;
      if (allowedHelp != null) {
        for (final String allowedValue in allowedHelp.keys) {
          expect(
            allowedHelp[allowedValue],
            isNot(anyOf(contains('default'), contains('Default'))),
            reason:
                '${_header}Help for $target--${option.name} $allowedValue" mentions the default value but that is redundant with the defaultsTo option which is also specified (and preferred).',
          );
        }
      }
    }
    expect(
      option.help,
      isNot(matches(_bannedArgumentReferencePatterns)),
      reason:
          '${_header}Help for $target--${option.name}" contains the string "--" in an unexpected way. If it\'s trying to mention another argument, it should be quoted, as in "--foo".',
    );
    for (final String line in option.help!.split('\n')) {
      if (!line.startsWith('    ')) {
        expect(
          line,
          isNot(contains('  ')),
          reason:
              '${_header}Help for $target--${option.name}" has excessive whitespace (check e.g. for double spaces after periods or round line breaks in the source).',
        );
        expect(
          line,
          matches(_allowedTrailingPatterns),
          reason:
              '${_header}A line in the help for $target--${option.name}" does not end with the expected period that a full sentence should end with. (If the help ends with a URL, place it after a colon, don\'t leave a trailing period; if it\'s sample code, prefix the line with four spaces.)',
        );
      }
    }
    expect(
      option.help,
      isNot(endsWith(':')),
      reason:
          '${_header}Help for $target--${option.name}" ends with a colon, which seems unlikely to be correct.',
    );
    expect(
      option.help,
      isNot(contains(_bannedUri)),
      reason: '${_header}Help for $target--${option.name}" uses the term "URI" rather than "URL".',
    );
    expect(
      option.help,
      isNot(contains(_nonSecureFlutterDartUrl)),
      reason:
          '${_header}Help for $target--${option.name}" links to a non-secure ("http") version of a Flutter or Dart site.',
    );
    // TODO(ianh): add some checking for embedded URLs to make sure we're consistent on how we format those.
    // TODO(ianh): arguably we should ban help text that starts with "Whether to..." since by definition a flag is to enable a feature, so the "whether to" is redundant.
  }
}
