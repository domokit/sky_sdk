// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:collection/collection.dart';
import 'package:meta/meta.dart';

@immutable
class CustomerTest {
  factory CustomerTest(File testFile) {
    final String errorPrefix = 'Could not parse: ${testFile.path}\n';
    final List<String> contacts = <String>[];
    final List<String> fetch = <String>[];
    final List<String> setup = <String>[];
    final List<Directory> update = <Directory>[];
    final List<String> test = <String>[];
    int? iterations;
    bool hasTests = false;
    for (final String line in testFile.readAsLinesSync().map((String line) => line.trim())) {
      if (line.isEmpty || line.startsWith('#')) {
        // Blank line or comment.
        continue;
      }

      final bool isUnknownDirective =
          _TestDirective.values.firstWhereOrNull((_TestDirective d) => line.startsWith(d.name)) ==
          null;
      if (isUnknownDirective) {
        throw FormatException('${errorPrefix}Unexpected directive:\n$line');
      }

      _maybeAddTestConfig(line, directive: _TestDirective.contact, directiveValues: contacts);
      _maybeAddTestConfig(line, directive: _TestDirective.fetch, directiveValues: fetch);
      _maybeAddTestConfig(
        line,
        directive: _TestDirective.setup,
        directiveValues: setup,
        platformAgnostic: false,
      );

      final String updatePrefix = _directive(_TestDirective.update);
      if (line.startsWith(updatePrefix)) {
        update.add(Directory(line.substring(updatePrefix.length)));
      }

      final String iterationsPrefix = _directive(_TestDirective.iterations);
      if (line.startsWith(iterationsPrefix)) {
        if (iterations != null) {
          throw FormatException(
            'Cannot specify "${_TestDirective.iterations.name}" directive multiple times.',
          );
        }
        iterations = int.parse(line.substring(iterationsPrefix.length));
        if (iterations < 1) {
          throw FormatException(
            'The "${_TestDirective.iterations.name}" directive must have a positive integer value.',
          );
        }
      }

      if (line.startsWith(_directive(_TestDirective.test)) ||
          line.startsWith('${_TestDirective.test.name}.')) {
        hasTests = true;
      }
      _maybeAddTestConfig(
        line,
        directive: _TestDirective.test,
        directiveValues: test,
        platformAgnostic: false,
      );
    }

    if (contacts.isEmpty) {
      throw FormatException(
        '${errorPrefix}No "${_TestDirective.contact.name}" directives specified. At least one contact e-mail address must be specified.',
      );
    }
    for (final String email in contacts) {
      if (!email.contains(_email) || email.endsWith('@example.com')) {
        throw FormatException(
          '${errorPrefix}The following e-mail address appears to be an invalid e-mail address: $email',
        );
      }
    }
    if (fetch.isEmpty) {
      throw FormatException(
        '${errorPrefix}No "${_TestDirective.fetch.name}" directives specified. Two lines are expected: "git clone https://github.com/USERNAME/REPOSITORY.git tests" and "git -C tests checkout HASH".',
      );
    }
    if (fetch.length < 2) {
      throw FormatException(
        '${errorPrefix}Only one "${_TestDirective.fetch.name}" directive specified. Two lines are expected: "git clone https://github.com/USERNAME/REPOSITORY.git tests" and "git -C tests checkout HASH".',
      );
    }
    if (!fetch[0].contains(_fetch1)) {
      throw FormatException(
        '${errorPrefix}First "${_TestDirective.fetch.name}" directive does not match expected pattern (expected "git clone https://github.com/USERNAME/REPOSITORY.git tests").',
      );
    }
    if (!fetch[1].contains(_fetch2)) {
      throw FormatException(
        '${errorPrefix}Second "${_TestDirective.fetch.name}" directive does not match expected pattern (expected "git -C tests checkout HASH").',
      );
    }
    if (update.isEmpty) {
      throw FormatException(
        '${errorPrefix}No "${_TestDirective.update.name}" directives specified. At least one directory must be specified. (It can be "." to just upgrade the root of the repository.)',
      );
    }
    if (!hasTests) {
      throw FormatException(
        '${errorPrefix}No "${_TestDirective.test.name}" directives specified. At least one command must be specified to run tests.',
      );
    }
    return CustomerTest._(
      List<String>.unmodifiable(contacts),
      List<String>.unmodifiable(fetch),
      List<String>.unmodifiable(setup),
      List<Directory>.unmodifiable(update),
      List<String>.unmodifiable(test),
      iterations,
    );
  }

  const CustomerTest._(
    this.contacts,
    this.fetch,
    this.setup,
    this.update,
    this.tests,
    this.iterations,
  );

  // (e-mail regexp from HTML standard)
  static final RegExp _email = RegExp(
    r"^[a-zA-Z0-9.!#$%&'*+\/=?^_`{|}~-]+@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?(?:\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?)*$",
  );
  static final RegExp _fetch1 = RegExp(
    r'^git(?: -c core.longPaths=true)? clone https://github.com/[-a-zA-Z0-9]+/[-_a-zA-Z0-9]+.git tests$',
  );
  static final RegExp _fetch2 = RegExp(
    r'^git(?: -c core.longPaths=true)? -C tests checkout [0-9a-f]+$',
  );

  final List<String> contacts;
  final List<String> fetch;
  final List<String> setup;
  final List<Directory> update;
  final List<String> tests;
  final int? iterations;

  static void _maybeAddTestConfig(
    String line, {
    required _TestDirective directive,
    required List<String> directiveValues,
    bool platformAgnostic = true,
  }) {
    final List<_PlatformType> platforms =
        platformAgnostic ? <_PlatformType>[_PlatformType.all] : _PlatformType.values;
    for (final _PlatformType platform in platforms) {
      final String directiveName = _directive(directive, platform: platform);
      if (line.startsWith(directiveName) && platform.conditionMet) {
        directiveValues.add(line.substring(directiveName.length));
      }
    }
  }

  static String _directive(_TestDirective directive, {_PlatformType platform = _PlatformType.all}) {
    return switch (platform) {
      _PlatformType.all => '${directive.name}=',
      _ => '${directive.name}.${platform.name}=',
    };
  }
}

enum _PlatformType {
  all,
  windows,
  macos,
  linux,
  posix;

  bool get conditionMet => switch (this) {
    _PlatformType.all => true,
    _PlatformType.windows => Platform.isWindows,
    _PlatformType.macos => Platform.isMacOS,
    _PlatformType.linux => Platform.isLinux,
    _PlatformType.posix => Platform.isLinux || Platform.isMacOS,
  };
}

enum _TestDirective { contact, fetch, setup, update, test, iterations }
