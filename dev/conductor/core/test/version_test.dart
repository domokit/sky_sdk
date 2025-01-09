// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:conductor_core/src/proto/conductor_state.pbenum.dart';
import 'package:conductor_core/src/version.dart';

import 'common.dart';

void main() {
  group('Version.fromString()', () {
    test('parses commits past a tagged stable', () {
      const String versionString = '2.8.0-1-g2ef5ad67fe';
      final Version version;
      try {
        version = Version.fromString(versionString);
      } on Exception catch (exception) {
        fail('Failed parsing "$versionString" with:\n$exception');
      }
      expect(version.x, 2);
      expect(version.y, 8);
      expect(version.z, 0);
      expect(version.m, isNull);
      expect(version.n, isNull);
      expect(version.commits, 1);
      expect(version.type, VersionType.gitDescribe);
    });
  });
  group(
    'Version.increment()',
    () {
      test('throws exception on nonsensical `level`', () {
        final List<String> levels = <String>['f', '0', 'xyz'];
        for (final String level in levels) {
          final Version version = Version.fromString('1.0.0-0.0.pre');
          expect(
            () => Version.increment(version, level).toString(),
            throwsExceptionWith('Unknown increment level $level.'),
          );
        }
      });

      test('does not support incrementing x', () {
        const String level = 'x';

        final Version version = Version.fromString('1.0.0-0.0.pre');
        expect(
          () => Version.increment(version, level).toString(),
          throwsExceptionWith('Incrementing $level is not supported by this tool'),
        );
      });

      test('successfully increments y', () {
        const String level = 'y';

        Version version = Version.fromString('1.0.0-0.0.pre');
        expect(Version.increment(version, level).toString(), '1.1.0-0.0.pre');

        version = Version.fromString('10.20.0-40.50.pre');
        expect(Version.increment(version, level).toString(), '10.21.0-0.0.pre');

        version = Version.fromString('1.18.0-3.0.pre');
        expect(Version.increment(version, level).toString(), '1.19.0-0.0.pre');
      });

      test('successfully increments z', () {
        const String level = 'z';

        Version version = Version.fromString('1.0.0');
        expect(Version.increment(version, level).toString(), '1.0.1');

        version = Version.fromString('10.20.0');
        expect(Version.increment(version, level).toString(), '10.20.1');

        version = Version.fromString('1.18.3');
        expect(Version.increment(version, level).toString(), '1.18.4');
      });

      test('does not support incrementing m', () {
        const String level = 'm';

        final Version version = Version.fromString('1.0.0-0.0.pre');
        expect(
          () => Version.increment(version, level).toString(),
          throwsAssertionWith("Do not increment 'm' via Version.increment"),
        );
      });

      test('successfully increments n', () {
        const String level = 'n';

        Version version = Version.fromString('1.0.0-0.0.pre');
        expect(Version.increment(version, level).toString(), '1.0.0-0.1.pre');

        version = Version.fromString('10.20.0-40.50.pre');
        expect(Version.increment(version, level).toString(), '10.20.0-40.51.pre');

        version = Version.fromString('1.18.0-3.0.pre');
        expect(Version.increment(version, level).toString(), '1.18.0-3.1.pre');
      });
    },
    onPlatform: <String, dynamic>{
      'windows': const Skip('Flutter Conductor only supported on macos/linux'),
    },
  );

  group('.ensureValid()', () {
    test('throws when x does not match', () {
      const String versionString = '1.2.3-4.5.pre';
      const String candidateBranch = 'flutter-3.2-candidate.4';
      final Version version = Version.fromString(versionString);
      expect(
        () => version.ensureValid(candidateBranch, ReleaseType.BETA_HOTFIX),
        throwsExceptionWith(
          'Parsed version $versionString has a different x value than '
          'candidate branch $candidateBranch',
        ),
      );
    });

    test('throws when y does not match', () {
      const String versionString = '1.2.3';
      const String candidateBranch = 'flutter-1.15-candidate.4';
      final Version version = Version.fromString(versionString);
      expect(
        () => version.ensureValid(candidateBranch, ReleaseType.BETA_INITIAL),
        throwsExceptionWith(
          'Parsed version $versionString has a different y value than '
          'candidate branch $candidateBranch',
        ),
      );
    });

    test('throws when m does not match', () {
      const String versionString = '1.2.3-4.5.pre';
      const String candidateBranch = 'flutter-1.2-candidate.0';
      final Version version = Version.fromString(versionString);
      expect(
        () => version.ensureValid(candidateBranch, ReleaseType.BETA_HOTFIX),
        throwsExceptionWith(
          'Parsed version $versionString has a different m value than '
          'candidate branch $candidateBranch',
        ),
      );
    });

    test('does not validate m if version type is stable', () {
      const String versionString = '1.2.0';
      const String candidateBranch = 'flutter-1.2-candidate.98';
      final Version version = Version.fromString(versionString);
      expect(
        () => version.ensureValid(candidateBranch, ReleaseType.STABLE_HOTFIX),
        isNot(throwsException),
      );
    });

    test('throws on malformed candidate branch', () {
      const String versionString = '1.2.0';
      const String candidateBranch = 'stable';
      final Version version = Version.fromString(versionString);
      expect(
        () => version.ensureValid(candidateBranch, ReleaseType.STABLE_HOTFIX),
        throwsExceptionWith('Candidate branch $candidateBranch does not match the pattern'),
      );
    });
  });
}
