// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show json;

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/fingerprint.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/utils.dart';

import '../../src/common.dart';

void main() {
  group('Fingerprinter', () {
    late MemoryFileSystem fileSystem;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
    });

    testWithoutContext('creates fingerprint with specified properties and files', () {
      fileSystem.file('a.dart').createSync();

      final Fingerprinter fingerprinter = Fingerprinter(
        fingerprintPath: 'out.fingerprint',
        paths: <String>['a.dart'],
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      final Fingerprint fingerprint = fingerprinter.buildFingerprint();
      expect(fingerprint, Fingerprint.fromBuildInputs(const <String>['a.dart'], fileSystem));
    });

    testWithoutContext('creates fingerprint with file checksums', () {
      fileSystem.file('a.dart').createSync();

      final Fingerprinter fingerprinter = Fingerprinter(
        fingerprintPath: 'out.fingerprint',
        paths: <String>['a.dart'],
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      final Fingerprint fingerprint = fingerprinter.buildFingerprint();
      expect(fingerprint, Fingerprint.fromBuildInputs(const <String>['a.dart'], fileSystem));
    });

    testWithoutContext('fingerprint does not match if not present', () {
      fileSystem.file('a.dart').createSync();
      fileSystem.file('b.dart').createSync();

      final Fingerprinter fingerprinter = Fingerprinter(
        fingerprintPath: 'out.fingerprint',
        paths: <String>['a.dart', 'b.dart'],
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      expect(fingerprinter.doesFingerprintMatch(), isFalse);
    });

    testWithoutContext('fingerprint does match if identical', () {
      fileSystem.file('a.dart').createSync();
      fileSystem.file('b.dart').createSync();

      const String fingerprintPath = 'path/to/out.fingerprint';
      final Fingerprinter fingerprinter = Fingerprinter(
        fingerprintPath: fingerprintPath,
        paths: <String>['a.dart', 'b.dart'],
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      fingerprinter.writeFingerprint();
      expect(fingerprinter.doesFingerprintMatch(), isTrue);
      expect(fileSystem.file(fingerprintPath), exists);
    });

    testWithoutContext('fails to write fingerprint if inputs are missing', () {
      const String fingerprintPath = 'path/to/out.fingerprint';
      final Fingerprinter fingerprinter = Fingerprinter(
        fingerprintPath: fingerprintPath,
        paths: <String>['a.dart'],
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      fingerprinter.writeFingerprint();
      expect(fileSystem.file(fingerprintPath), isNot(exists));
    });

    group('Fingerprint', () {
      group('fromBuildInputs', () {
        late MemoryFileSystem fileSystem;

        setUp(() {
          fileSystem = MemoryFileSystem.test();
        });

        testWithoutContext('throws if any input file does not exist', () {
          fileSystem.file('a.dart').createSync();
          expect(
            () => Fingerprint.fromBuildInputs(const <String>['a.dart', 'b.dart'], fileSystem),
            throwsException,
          );
        });

        testWithoutContext('populates checksums for valid files', () {
          fileSystem.file('a.dart').writeAsStringSync('This is a');
          fileSystem.file('b.dart').writeAsStringSync('This is b');
          final Fingerprint fingerprint = Fingerprint.fromBuildInputs(const <String>[
            'a.dart',
            'b.dart',
          ], fileSystem);

          final Map<String, dynamic>? jsonObject = castStringKeyedMap(
            json.decode(fingerprint.toJson()),
          );
          final Map<String, dynamic> files = jsonObject!['files'] as Map<String, dynamic>;
          expect(files, hasLength(2));
          expect(files['a.dart'], '8a21a15fad560b799f6731d436c1b698');
          expect(files['b.dart'], '6f144e08b58cd0925328610fad7ac07c');
        });
      });

      group('fromJson', () {
        testWithoutContext('throws if JSON is invalid', () {
          expect(() => Fingerprint.fromJson('<xml></xml>'), throwsA(anything));
        });

        testWithoutContext('creates fingerprint from valid JSON', () {
          final String jsonString = json.encode(<String, dynamic>{
            'files': <String, dynamic>{
              'a.dart': '8a21a15fad560b799f6731d436c1b698',
              'b.dart': '6f144e08b58cd0925328610fad7ac07c',
            },
          });
          final Fingerprint fingerprint = Fingerprint.fromJson(jsonString);
          final Map<String, dynamic>? content = castStringKeyedMap(
            json.decode(fingerprint.toJson()),
          );
          final Map<String, dynamic> files = content!['files'] as Map<String, dynamic>;
          expect(content, hasLength(1));
          expect(files, hasLength(2));
          expect(files['a.dart'], '8a21a15fad560b799f6731d436c1b698');
          expect(files['b.dart'], '6f144e08b58cd0925328610fad7ac07c');
        });
        testWithoutContext('treats missing properties and files entries as if empty', () {
          final String jsonString = json.encode(<String, dynamic>{});
          expect(
            Fingerprint.fromJson(jsonString),
            Fingerprint.fromBuildInputs(const <String>[], fileSystem),
          );
        });
      });

      group('operator ==', () {
        testWithoutContext('reports not equal if file checksums do not match', () {
          final Map<String, dynamic> a = <String, dynamic>{
            'files': <String, dynamic>{
              'a.dart': '8a21a15fad560b799f6731d436c1b698',
              'b.dart': '6f144e08b58cd0925328610fad7ac07c',
            },
          };
          final Map<String, dynamic> b = Map<String, dynamic>.of(a);
          b['files'] = <String, dynamic>{
            'a.dart': '8a21a15fad560b799f6731d436c1b698',
            'b.dart': '6f144e08b58cd0925328610fad7ac07d',
          };
          expect(
            Fingerprint.fromJson(json.encode(a)) == Fingerprint.fromJson(json.encode(b)),
            isFalse,
          );
        });

        testWithoutContext('reports not equal if file paths do not match', () {
          final Map<String, dynamic> a = <String, dynamic>{
            'files': <String, dynamic>{
              'a.dart': '8a21a15fad560b799f6731d436c1b698',
              'b.dart': '6f144e08b58cd0925328610fad7ac07c',
            },
          };
          final Map<String, dynamic> b = Map<String, dynamic>.of(a);
          b['files'] = <String, dynamic>{
            'a.dart': '8a21a15fad560b799f6731d436c1b698',
            'c.dart': '6f144e08b58cd0925328610fad7ac07d',
          };
          expect(
            Fingerprint.fromJson(json.encode(a)) == Fingerprint.fromJson(json.encode(b)),
            isFalse,
          );
        });

        testWithoutContext('reports equal if properties and file checksums match', () {
          final Map<String, dynamic> a = <String, dynamic>{
            'files': <String, dynamic>{
              'a.dart': '8a21a15fad560b799f6731d436c1b698',
              'b.dart': '6f144e08b58cd0925328610fad7ac07c',
            },
          };
          expect(
            Fingerprint.fromJson(json.encode(a)) == Fingerprint.fromJson(json.encode(a)),
            isTrue,
          );
        });
      });
      group('hashCode', () {
        testWithoutContext('is consistent with equals, even if map entries are reordered', () {
          final Fingerprint a = Fingerprint.fromJson('{"properties":{"a":"A","b":"B"},"files":{}}');
          final Fingerprint b = Fingerprint.fromJson('{"properties":{"b":"B","a":"A"},"files":{}}');
          expect(a, b);
          expect(a.hashCode, b.hashCode);
        });
      });
    });
  });
}
