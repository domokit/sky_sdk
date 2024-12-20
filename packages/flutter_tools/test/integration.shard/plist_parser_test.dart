// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';

import '../src/common.dart';
import '../src/fakes.dart';
import 'test_utils.dart';

const String base64PlistXml =
    'PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KPCFET0NUWVBFIHBsaXN0I'
    'FBVQkxJQyAiLS8vQXBwbGUvL0RURCBQTElTVCAxLjAvL0VOIiAiaHR0cDovL3d3dy5hcHBsZS'
    '5jb20vRFREcy9Qcm9wZXJ0eUxpc3QtMS4wLmR0ZCI+CjxwbGlzdCB2ZXJzaW9uPSIxLjAiPgo'
    '8ZGljdD4KICA8a2V5PkNGQnVuZGxlRXhlY3V0YWJsZTwva2V5PgogIDxzdHJpbmc+QXBwPC9z'
    'dHJpbmc+CiAgPGtleT5DRkJ1bmRsZUlkZW50aWZpZXI8L2tleT4KICA8c3RyaW5nPmlvLmZsd'
    'XR0ZXIuZmx1dHRlci5hcHA8L3N0cmluZz4KPC9kaWN0Pgo8L3BsaXN0Pgo=';

const String base64PlistBinary =
    'YnBsaXN0MDDSAQIDBF8QEkNGQnVuZGxlRXhlY3V0YWJsZV8QEkNGQnVuZGxlSWRlbnRpZmllc'
    'lNBcHBfEBZpby5mbHV0dGVyLmZsdXR0ZXIuYXBwCA0iNzsAAAAAAAABAQAAAAAAAAAFAAAAAA'
    'AAAAAAAAAAAAAAVA==';

const String base64PlistJson =
    'eyJDRkJ1bmRsZUV4ZWN1dGFibGUiOiJBcHAiLCJDRkJ1bmRsZUlkZW50aWZpZXIiOiJpby5mb'
    'HV0dGVyLmZsdXR0ZXIuYXBwIn0=';

const String base64PlistXmlWithComplexDatatypes =
    'PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KPCFET0NUWVBFIHBsaXN0I'
    'FBVQkxJQyAiLS8vQXBwbGUvL0RURCBQTElTVCAxLjAvL0VOIiAiaHR0cDovL3d3dy5hcHBsZS'
    '5jb20vRFREcy9Qcm9wZXJ0eUxpc3QtMS4wLmR0ZCI+CjxwbGlzdCB2ZXJzaW9uPSIxLjAiPgo'
    '8ZGljdD4KICA8a2V5PkNGQnVuZGxlRXhlY3V0YWJsZTwva2V5PgogIDxzdHJpbmc+QXBwPC9z'
    'dHJpbmc+CiAgPGtleT5DRkJ1bmRsZUlkZW50aWZpZXI8L2tleT4KICA8c3RyaW5nPmlvLmZsd'
    'XR0ZXIuZmx1dHRlci5hcHA8L3N0cmluZz4KICA8a2V5PmludFZhbHVlPC9rZXk+CiAgPGludG'
    'VnZXI+MjwvaW50ZWdlcj4KICA8a2V5PmRvdWJsZVZhbHVlPC9rZXk+CiAgPHJlYWw+MS41PC9'
    'yZWFsPgogIDxrZXk+YmluYXJ5VmFsdWU8L2tleT4KICA8ZGF0YT5ZV0pqWkE9PTwvZGF0YT4K'
    'ICA8a2V5PmFycmF5VmFsdWU8L2tleT4KICA8YXJyYXk+CiAgICA8dHJ1ZSAvPgogICAgPGZhb'
    'HNlIC8+CiAgICA8aW50ZWdlcj4zPC9pbnRlZ2VyPgogIDwvYXJyYXk+CiAgPGtleT5kYXRlVm'
    'FsdWU8L2tleT4KICA8ZGF0ZT4yMDIxLTEyLTAxVDEyOjM0OjU2WjwvZGF0ZT4KPC9kaWN0Pgo'
    '8L3BsaXN0Pg==';

void main() {
  // The tests herein explicitly don't use `MemoryFileSystem` or a mocked
  // `ProcessManager` because doing so wouldn't actually test what we want to
  // test, which is that the underlying tool we're using to parse Plist files
  // works with the way we're calling it.
  late File file;
  late PlistParser parser;
  late BufferLogger logger;

  setUp(() {
    logger = BufferLogger(
      outputPreferences: OutputPreferences.test(),
      terminal: AnsiTerminal(platform: const LocalPlatform(), stdio: FakeStdio()),
    );
    parser = PlistParser(fileSystem: fileSystem, processManager: processManager, logger: logger);
    file = fileSystem.file('foo.plist')..createSync();
  });

  tearDown(() {
    file.deleteSync();
  });

  testWithoutContext('PlistParser.getValueFromFile<String> works with an XML file', () {
    file.writeAsBytesSync(base64.decode(base64PlistXml));

    expect(
      parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(
      parser.getValueFromFile<String>(file.absolute.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.getValueFromFile<String> works with a binary file', () {
    file.writeAsBytesSync(base64.decode(base64PlistBinary));

    expect(
      parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(
      parser.getValueFromFile<String>(file.absolute.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.getValueFromFile<String> works with a JSON file', () {
    file.writeAsBytesSync(base64.decode(base64PlistJson));

    expect(
      parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(
      parser.getValueFromFile<String>(file.absolute.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext(
    'PlistParser.getValueFromFile<String> returns null for a non-existent plist file',
    () {
      expect(parser.getValueFromFile<String>('missing.plist', 'CFBundleIdentifier'), null);
      expect(logger.statusText, isEmpty);
      expect(logger.errorText, isEmpty);
    },
    skip: !platform.isMacOS, // [intended] requires macos tool chain.
  );

  testWithoutContext(
    'PlistParser.getValueFromFile<String> returns null for a non-existent key within a plist',
    () {
      file.writeAsBytesSync(base64.decode(base64PlistXml));

      expect(parser.getValueFromFile<String>(file.path, 'BadKey'), null);
      expect(parser.getValueFromFile<String>(file.absolute.path, 'BadKey'), null);
      expect(logger.statusText, isEmpty);
      expect(logger.errorText, isEmpty);
    },
    skip: !platform.isMacOS, // [intended] requires macos tool chain.
  );

  testWithoutContext(
    'PlistParser.getValueFromFile<String> returns null for a malformed plist file',
    () {
      file.writeAsBytesSync(const <int>[1, 2, 3, 4, 5, 6]);

      expect(parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'), null);
      expect(
        logger.statusText,
        contains(
          'Property List error: Unexpected character \x01 at line 1 / '
          'JSON error: JSON text did not start with array or object and option to allow fragments not '
          'set. around line 1, column 0.\n',
        ),
      );
      expect(
        logger.errorText,
        'ProcessException: The command failed with exit code 1\n'
        '  Command: /usr/bin/plutil -convert xml1 -o - ${file.absolute.path}\n',
      );
    },
    skip: !platform.isMacOS, // [intended] requires macos tool chain.
  );

  testWithoutContext(
    'PlistParser.getValueFromFile<String> throws when /usr/bin/plutil is not found',
    () async {
      file.writeAsBytesSync(base64.decode(base64PlistXml));

      expect(
        () => parser.getValueFromFile<String>(file.path, 'unused'),
        throwsA(isA<FileNotFoundException>()),
      );
      expect(logger.statusText, isEmpty);
      expect(logger.errorText, isEmpty);
    },
    skip: platform.isMacOS, // [intended] requires absence of macos tool chain.
  );

  testWithoutContext('PlistParser.replaceKey can replace a key', () async {
    file.writeAsBytesSync(base64.decode(base64PlistXml));

    expect(
      parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(
      parser.replaceKey(file.path, key: 'CFBundleIdentifier', value: 'dev.flutter.fake'),
      isTrue,
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
    expect(
      parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'),
      equals('dev.flutter.fake'),
    );
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.replaceKey can create a new key', () async {
    file.writeAsBytesSync(base64.decode(base64PlistXml));

    expect(parser.getValueFromFile<String>(file.path, 'CFNewKey'), isNull);
    expect(parser.replaceKey(file.path, key: 'CFNewKey', value: 'dev.flutter.fake'), isTrue);
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
    expect(parser.getValueFromFile<String>(file.path, 'CFNewKey'), equals('dev.flutter.fake'));
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.replaceKey can delete a key', () async {
    file.writeAsBytesSync(base64.decode(base64PlistXml));

    expect(parser.replaceKey(file.path, key: 'CFBundleIdentifier'), isTrue);
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
    expect(parser.getValueFromFile<String>(file.path, 'CFBundleIdentifier'), isNull);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.replaceKey throws when /usr/bin/plutil is not found', () async {
    file.writeAsBytesSync(base64.decode(base64PlistXml));

    expect(
      () => parser.replaceKey(file.path, key: 'CFBundleIdentifier', value: 'dev.flutter.fake'),
      throwsA(isA<FileNotFoundException>()),
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: platform.isMacOS); // [intended] requires absence of macos tool chain.

  testWithoutContext('PlistParser.replaceKey returns false for a malformed plist file', () {
    file.writeAsBytesSync(const <int>[1, 2, 3, 4, 5, 6]);

    expect(
      parser.replaceKey(file.path, key: 'CFBundleIdentifier', value: 'dev.flutter.fake'),
      isFalse,
    );
    expect(
      logger.statusText,
      contains(
        'foo.plist: Property List error: Unexpected character \x01 '
        'at line 1 / JSON error: JSON text did not start with array or object and option to allow '
        'fragments not set. around line 1, column 0.\n',
      ),
    );
    expect(
      logger.errorText,
      equals(
        'ProcessException: The command failed with exit code 1\n'
        '  Command: /usr/bin/plutil -replace CFBundleIdentifier -string dev.flutter.fake foo.plist\n',
      ),
    );
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.replaceKey works with a JSON file', () {
    file.writeAsBytesSync(base64.decode(base64PlistJson));

    expect(
      parser.getValueFromFile<String>(file.absolute.path, 'CFBundleIdentifier'),
      'io.flutter.flutter.app',
    );
    expect(
      parser.replaceKey(file.path, key: 'CFBundleIdentifier', value: 'dev.flutter.fake'),
      isTrue,
    );
    expect(
      parser.getValueFromFile<String>(file.absolute.path, 'CFBundleIdentifier'),
      'dev.flutter.fake',
    );
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.parseFile can handle different datatypes', () async {
    file.writeAsBytesSync(base64.decode(base64PlistXmlWithComplexDatatypes));
    final Map<String, Object> values = parser.parseFile(file.path);

    expect(values['CFBundleIdentifier'], 'io.flutter.flutter.app');
    expect(values['CFBundleIdentifier'], 'io.flutter.flutter.app');
    expect(values['intValue'], 2);
    expect(values['doubleValue'], 1.5);
    expect(values['binaryValue'], base64.decode('YWJjZA=='));
    expect(values['arrayValue'], <dynamic>[true, false, 3]);
    expect(values['dateValue'], DateTime.utc(2021, 12, 1, 12, 34, 56));
    expect(logger.statusText, isEmpty);
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext('PlistParser.plistJsonContent can parse pbxproj file', () async {
    final String xcodeProjectFile = fileSystem.path.join(
      getFlutterRoot(),
      'dev',
      'integration_tests',
      'flutter_gallery',
      'ios',
      'Runner.xcodeproj',
      'project.pbxproj',
    );

    final BufferLogger logger = BufferLogger(
      terminal: Terminal.test(),
      outputPreferences: OutputPreferences(),
    );

    final PlistParser parser = PlistParser(
      fileSystem: fileSystem,
      processManager: processManager,
      logger: logger,
    );

    final String? projectFileAsJson = parser.plistJsonContent(xcodeProjectFile);
    expect(projectFileAsJson, isNotNull);
    expect(projectFileAsJson, contains('"PRODUCT_NAME":"Flutter Gallery"'));
    expect(logger.errorText, isEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.

  testWithoutContext(
    'PlistParser.plistJsonContent can parse pbxproj file with unicode and emojis',
    () async {
      String xcodeProjectFile = fileSystem.path.join(
        getFlutterRoot(),
        'dev',
        'integration_tests',
        'flutter_gallery',
        'ios',
        'Runner.xcodeproj',
        'project.pbxproj',
      );

      final BufferLogger logger = BufferLogger(
        terminal: Terminal.test(),
        outputPreferences: OutputPreferences(),
      );

      final PlistParser parser = PlistParser(
        fileSystem: fileSystem,
        processManager: processManager,
        logger: logger,
      );

      xcodeProjectFile = xcodeProjectFile.replaceAll('AppDelegate.m', 'AppDélegate.m');
      xcodeProjectFile = xcodeProjectFile.replaceAll('AppDelegate.h', 'App👍Delegate.h');

      final String? projectFileAsJson = parser.plistJsonContent(xcodeProjectFile);
      expect(projectFileAsJson, isNotNull);
      expect(projectFileAsJson, contains('"PRODUCT_NAME":"Flutter Gallery"'));
      expect(logger.errorText, isEmpty);
    },
    skip: !platform.isMacOS, // [intended] requires macos tool chain.
  );

  testWithoutContext('PlistParser.plistJsonContent returns null when errors', () async {
    final BufferLogger logger = BufferLogger(
      terminal: Terminal.test(),
      outputPreferences: OutputPreferences(),
    );

    final PlistParser parser = PlistParser(
      fileSystem: fileSystem,
      processManager: processManager,
      logger: logger,
    );

    final String? projectFileAsJson = parser.plistJsonContent('bad/path');
    expect(projectFileAsJson, isNull);
    expect(logger.errorText, isNotEmpty);
  }, skip: !platform.isMacOS); // [intended] requires macos tool chain.
}
