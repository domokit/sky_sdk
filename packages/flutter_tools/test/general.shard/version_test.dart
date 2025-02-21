// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart' show FakeFlutterVersion;

final SystemClock _testClock = SystemClock.fixed(DateTime.utc(2015));
final DateTime _stampUpToDate = _testClock.ago(
  VersionFreshnessValidator.checkAgeConsideredUpToDate ~/ 2,
);
final DateTime _stampOutOfDate = _testClock.ago(
  VersionFreshnessValidator.checkAgeConsideredUpToDate * 2,
);

void main() {
  late FakeCache cache;
  late FakeProcessManager processManager;

  setUp(() {
    processManager = FakeProcessManager.empty();
    cache = FakeCache();
  });

  testUsingContext('Channel enum and string transform to each other', () {
    for (final Channel channel in Channel.values) {
      expect(getNameForChannel(channel), kOfficialChannels.toList()[channel.index]);
    }
    expect(
      kOfficialChannels.toList().map((String str) => getChannelForName(str)).toList(),
      Channel.values,
    );
  });

  for (final String channel in kOfficialChannels) {
    DateTime getChannelUpToDateVersion() {
      return _testClock.ago(VersionFreshnessValidator.versionAgeConsideredUpToDate(channel) ~/ 2);
    }

    DateTime getChannelOutOfDateVersion() {
      return _testClock.ago(VersionFreshnessValidator.versionAgeConsideredUpToDate(channel) * 2);
    }

    group('$FlutterVersion for $channel', () {
      late FileSystem fs;
      const String flutterRoot = '/path/to/flutter';

      setUpAll(() {
        fs = MemoryFileSystem.test();
        Cache.disableLocking();
        VersionFreshnessValidator.timeToPauseToLetUserReadTheMessage = Duration.zero;
      });

      testUsingContext(
        'prints nothing when Flutter installation looks fresh',
        () async {
          const String flutterUpstreamUrl = 'https://github.com/flutter/flutter.git';
          processManager.addCommands(<FakeCommand>[
            const FakeCommand(
              command: <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                '-n',
                '1',
                '--pretty=format:%H',
              ],
              stdout: '1234abcd',
            ),
            const FakeCommand(command: <String>['git', 'tag', '--points-at', '1234abcd']),
            const FakeCommand(
              command: <String>[
                'git',
                'describe',
                '--match',
                '*.*.*',
                '--long',
                '--tags',
                '1234abcd',
              ],
              stdout: '0.1.2-3-1234abcd',
            ),
            FakeCommand(
              command: const <String>['git', 'symbolic-ref', '--short', 'HEAD'],
              stdout: channel,
            ),
            FakeCommand(
              command: const <String>[
                'git',
                'rev-parse',
                '--abbrev-ref',
                '--symbolic',
                '@{upstream}',
              ],
              stdout: 'origin/$channel',
            ),
            const FakeCommand(
              command: <String>['git', 'ls-remote', '--get-url', 'origin'],
              stdout: flutterUpstreamUrl,
            ),
            FakeCommand(
              command: const <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                'HEAD',
                '-n',
                '1',
                '--pretty=format:%ad',
                '--date=iso',
              ],
              stdout: getChannelUpToDateVersion().toString(),
            ),
            const FakeCommand(command: <String>['git', 'fetch', '--tags']),
            FakeCommand(
              command: const <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                '@{upstream}',
                '-n',
                '1',
                '--pretty=format:%ad',
                '--date=iso',
              ],
              stdout: getChannelOutOfDateVersion().toString(),
            ),
            const FakeCommand(
              command: <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                '-n',
                '1',
                '--pretty=format:%ar',
              ],
              stdout: '1 second ago',
            ),
            FakeCommand(
              command: const <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                'HEAD',
                '-n',
                '1',
                '--pretty=format:%ad',
                '--date=iso',
              ],
              stdout: getChannelUpToDateVersion().toString(),
            ),
            const FakeCommand(
              command: <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                '-n',
                '1',
                '--pretty=format:%ar',
                'abcdefg',
              ],
              stdout: '2 seconds ago',
            ),
            FakeCommand(
              command: const <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                'abcdefg',
                '-n',
                '1',
                '--pretty=format:%ad',
                '--date=iso',
              ],
              stdout: getChannelUpToDateVersion().toString(),
            ),
          ]);

          final FlutterVersion flutterVersion = FlutterVersion(
            clock: _testClock,
            fs: fs,
            flutterRoot: flutterRoot,
          );
          await flutterVersion.checkFlutterVersionFreshness();
          expect(flutterVersion.channel, channel);
          expect(flutterVersion.repositoryUrl, flutterUpstreamUrl);
          expect(flutterVersion.frameworkRevision, '1234abcd');
          expect(flutterVersion.frameworkRevisionShort, '1234abcd');
          expect(flutterVersion.frameworkVersion, '0.0.0-unknown');
          expect(
            flutterVersion.toString(),
            'Flutter • channel $channel • $flutterUpstreamUrl\n'
            'Framework • revision 1234abcd (1 second ago) • ${getChannelUpToDateVersion()}\n'
            'Engine • revision abcdefg (2 seconds ago) • ${getChannelUpToDateVersion()}\n'
            'Tools • Dart 2.12.0 • DevTools 2.8.0',
          );
          expect(flutterVersion.frameworkAge, '1 second ago');
          expect(flutterVersion.getVersionString(), '$channel/1234abcd');
          expect(flutterVersion.getBranchName(), channel);
          expect(flutterVersion.getVersionString(redactUnknownBranches: true), '$channel/1234abcd');
          expect(flutterVersion.getBranchName(redactUnknownBranches: true), channel);

          expect(testLogger.statusText, isEmpty);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
      );

      testUsingContext(
        'does not crash when git log outputs malformed output',
        () async {
          const String flutterUpstreamUrl = 'https://github.com/flutter/flutter.git';

          final String malformedGitLogOutput =
              '${getChannelUpToDateVersion()}[0x7FF9E2A75000] ANOMALY: meaningless REX prefix used';
          processManager.addCommands(<FakeCommand>[
            const FakeCommand(
              command: <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                '-n',
                '1',
                '--pretty=format:%H',
              ],
              stdout: '1234abcd',
            ),
            const FakeCommand(command: <String>['git', 'tag', '--points-at', '1234abcd']),
            const FakeCommand(
              command: <String>[
                'git',
                'describe',
                '--match',
                '*.*.*',
                '--long',
                '--tags',
                '1234abcd',
              ],
              stdout: '0.1.2-3-1234abcd',
            ),
            FakeCommand(
              command: const <String>['git', 'symbolic-ref', '--short', 'HEAD'],
              stdout: channel,
            ),
            FakeCommand(
              command: const <String>[
                'git',
                'rev-parse',
                '--abbrev-ref',
                '--symbolic',
                '@{upstream}',
              ],
              stdout: 'origin/$channel',
            ),
            const FakeCommand(
              command: <String>['git', 'ls-remote', '--get-url', 'origin'],
              stdout: flutterUpstreamUrl,
            ),
            FakeCommand(
              command: const <String>[
                'git',
                '-c',
                'log.showSignature=false',
                'log',
                'HEAD',
                '-n',
                '1',
                '--pretty=format:%ad',
                '--date=iso',
              ],
              stdout: malformedGitLogOutput,
            ),
          ]);

          final FlutterVersion flutterVersion = FlutterVersion(
            clock: _testClock,
            fs: fs,
            flutterRoot: flutterRoot,
          );
          await flutterVersion.checkFlutterVersionFreshness();

          expect(testLogger.statusText, isEmpty);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
      );

      testWithoutContext(
        'prints nothing when Flutter installation looks out-of-date but is actually up-to-date',
        () async {
          final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
          final BufferLogger logger = BufferLogger.test();
          final VersionCheckStamp stamp = VersionCheckStamp(
            lastTimeVersionWasChecked: _stampOutOfDate,
            lastKnownRemoteVersion: getChannelOutOfDateVersion(),
          );
          cache.versionStamp = json.encode(stamp);

          await VersionFreshnessValidator(
            version: flutterVersion,
            cache: cache,
            clock: _testClock,
            logger: logger,
            localFrameworkCommitDate: getChannelOutOfDateVersion(),
            latestFlutterCommitDate: getChannelOutOfDateVersion(),
          ).run();

          expect(logger.statusText, isEmpty);
        },
      );

      testWithoutContext('does not ping server when version stamp is up-to-date', () async {
        final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
        final BufferLogger logger = BufferLogger.test();
        final VersionCheckStamp stamp = VersionCheckStamp(
          lastTimeVersionWasChecked: _stampUpToDate,
          lastKnownRemoteVersion: getChannelUpToDateVersion(),
        );
        cache.versionStamp = json.encode(stamp);

        await VersionFreshnessValidator(
          version: flutterVersion,
          cache: cache,
          clock: _testClock,
          logger: logger,
          localFrameworkCommitDate: getChannelOutOfDateVersion(),
          latestFlutterCommitDate: getChannelUpToDateVersion(),
        ).run();

        expect(logger.statusText, contains('A new version of Flutter is available!'));
        expect(cache.setVersionStamp, true);
      });

      testWithoutContext('does not print warning if printed recently', () async {
        final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
        final BufferLogger logger = BufferLogger.test();
        final VersionCheckStamp stamp = VersionCheckStamp(
          lastTimeVersionWasChecked: _stampUpToDate,
          lastKnownRemoteVersion: getChannelUpToDateVersion(),
          lastTimeWarningWasPrinted: _testClock.now(),
        );
        cache.versionStamp = json.encode(stamp);

        await VersionFreshnessValidator(
          version: flutterVersion,
          cache: cache,
          clock: _testClock,
          logger: logger,
          localFrameworkCommitDate: getChannelOutOfDateVersion(),
          latestFlutterCommitDate: getChannelUpToDateVersion(),
        ).run();

        expect(logger.statusText, isEmpty);
      });

      testWithoutContext('pings server when version stamp is missing', () async {
        final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
        final BufferLogger logger = BufferLogger.test();
        cache.versionStamp = '{}';

        await VersionFreshnessValidator(
          version: flutterVersion,
          cache: cache,
          clock: _testClock,
          logger: logger,
          localFrameworkCommitDate: getChannelOutOfDateVersion(),
          latestFlutterCommitDate: getChannelUpToDateVersion(),
        ).run();

        expect(logger.statusText, contains('A new version of Flutter is available!'));
        expect(cache.setVersionStamp, true);
      });

      testWithoutContext('pings server when version stamp is out-of-date', () async {
        final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
        final BufferLogger logger = BufferLogger.test();
        final VersionCheckStamp stamp = VersionCheckStamp(
          lastTimeVersionWasChecked: _stampOutOfDate,
          lastKnownRemoteVersion: _testClock.ago(const Duration(days: 2)),
        );
        cache.versionStamp = json.encode(stamp);

        await VersionFreshnessValidator(
          version: flutterVersion,
          cache: cache,
          clock: _testClock,
          logger: logger,
          localFrameworkCommitDate: getChannelOutOfDateVersion(),
          latestFlutterCommitDate: getChannelUpToDateVersion(),
        ).run();

        expect(logger.statusText, contains('A new version of Flutter is available!'));
      });

      testWithoutContext(
        'does not print warning when unable to connect to server if not out of date',
        () async {
          final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
          final BufferLogger logger = BufferLogger.test();
          cache.versionStamp = '{}';

          await VersionFreshnessValidator(
            version: flutterVersion,
            cache: cache,
            clock: _testClock,
            logger: logger,
            localFrameworkCommitDate: getChannelUpToDateVersion(),
            // latestFlutterCommitDate defaults to null because we failed to get remote version
          ).run();

          expect(logger.statusText, isEmpty);
        },
      );

      testWithoutContext(
        'prints warning when unable to connect to server if really out of date',
        () async {
          final FakeFlutterVersion flutterVersion = FakeFlutterVersion(branch: channel);
          final BufferLogger logger = BufferLogger.test();
          final VersionCheckStamp stamp = VersionCheckStamp(
            lastTimeVersionWasChecked: _stampOutOfDate,
            lastKnownRemoteVersion: _testClock.ago(const Duration(days: 2)),
          );
          cache.versionStamp = json.encode(stamp);

          await VersionFreshnessValidator(
            version: flutterVersion,
            cache: cache,
            clock: _testClock,
            logger: logger,
            localFrameworkCommitDate: getChannelOutOfDateVersion(),
            // latestFlutterCommitDate defaults to null because we failed to get remote version
          ).run();

          final Duration frameworkAge = _testClock.now().difference(getChannelOutOfDateVersion());
          expect(
            logger.statusText,
            contains('WARNING: your installation of Flutter is ${frameworkAge.inDays} days old.'),
          );
        },
      );

      group('$VersionCheckStamp for $channel', () {
        void expectDefault(VersionCheckStamp stamp) {
          expect(stamp.lastKnownRemoteVersion, isNull);
          expect(stamp.lastTimeVersionWasChecked, isNull);
          expect(stamp.lastTimeWarningWasPrinted, isNull);
        }

        testWithoutContext('loads blank when stamp file missing', () async {
          cache.versionStamp = null;

          expectDefault(await VersionCheckStamp.load(cache, BufferLogger.test()));
        });

        testWithoutContext('loads blank when stamp file is malformed JSON', () async {
          cache.versionStamp = '<';

          expectDefault(await VersionCheckStamp.load(cache, BufferLogger.test()));
        });

        testWithoutContext('loads blank when stamp file is well-formed but invalid JSON', () async {
          cache.versionStamp = '[]';

          expectDefault(await VersionCheckStamp.load(cache, BufferLogger.test()));
        });

        testWithoutContext('loads valid JSON', () async {
          final String value = '''
        {
          "lastKnownRemoteVersion": "${_testClock.ago(const Duration(days: 1))}",
          "lastTimeVersionWasChecked": "${_testClock.ago(const Duration(days: 2))}",
          "lastTimeWarningWasPrinted": "${_testClock.now()}"
        }
        ''';
          cache.versionStamp = value;

          final VersionCheckStamp stamp = await VersionCheckStamp.load(cache, BufferLogger.test());

          expect(stamp.lastKnownRemoteVersion, _testClock.ago(const Duration(days: 1)));
          expect(stamp.lastTimeVersionWasChecked, _testClock.ago(const Duration(days: 2)));
          expect(stamp.lastTimeWarningWasPrinted, _testClock.now());
        });
      });
    });
  }

  group('VersionUpstreamValidator', () {
    const String flutterStandardUrlDotGit = 'https://github.com/flutter/flutter.git';
    const String flutterNonStandardUrlDotGit = 'https://githubmirror.com/flutter/flutter.git';
    const String flutterStandardSshUrlDotGit = 'git@github.com:flutter/flutter.git';
    const String flutterFullSshUrlDotGit = 'ssh://git@github.com/flutter/flutter.git';

    VersionCheckError? runUpstreamValidator({String? versionUpstreamUrl, String? flutterGitUrl}) {
      final Platform testPlatform = FakePlatform(
        environment: <String, String>{if (flutterGitUrl != null) 'FLUTTER_GIT_URL': flutterGitUrl},
      );
      return VersionUpstreamValidator(
        version: FakeFlutterVersion(repositoryUrl: versionUpstreamUrl),
        platform: testPlatform,
      ).run();
    }

    testWithoutContext('returns error if repository url is null', () {
      final VersionCheckError error =
          runUpstreamValidator(
            // repositoryUrl is null by default
          )!;
      expect(error, isNotNull);
      expect(
        error.message,
        contains(
          'The tool could not determine the remote upstream which is being tracked by the SDK.',
        ),
      );
    });

    testWithoutContext(
      'does not return error at standard remote url with FLUTTER_GIT_URL unset',
      () {
        expect(runUpstreamValidator(versionUpstreamUrl: flutterStandardUrlDotGit), isNull);
      },
    );

    testWithoutContext('returns error at non-standard remote url with FLUTTER_GIT_URL unset', () {
      final VersionCheckError error =
          runUpstreamValidator(versionUpstreamUrl: flutterNonStandardUrlDotGit)!;
      expect(error, isNotNull);
      expect(
        error.message,
        contains(
          'The Flutter SDK is tracking a non-standard remote "$flutterNonStandardUrlDotGit".\n'
          'Set the environment variable "FLUTTER_GIT_URL" to "$flutterNonStandardUrlDotGit". '
          'If this is intentional, it is recommended to use "git" directly to manage the SDK.',
        ),
      );
    });

    testWithoutContext(
      'does not return error at non-standard remote url with FLUTTER_GIT_URL set',
      () {
        expect(
          runUpstreamValidator(
            versionUpstreamUrl: flutterNonStandardUrlDotGit,
            flutterGitUrl: flutterNonStandardUrlDotGit,
          ),
          isNull,
        );
      },
    );

    testWithoutContext('respects FLUTTER_GIT_URL even if upstream remote url is standard', () {
      final VersionCheckError error =
          runUpstreamValidator(
            versionUpstreamUrl: flutterStandardUrlDotGit,
            flutterGitUrl: flutterNonStandardUrlDotGit,
          )!;
      expect(error, isNotNull);
      expect(
        error.message,
        contains(
          'The Flutter SDK is tracking "$flutterStandardUrlDotGit" but "FLUTTER_GIT_URL" is set to "$flutterNonStandardUrlDotGit".\n'
          'Either remove "FLUTTER_GIT_URL" from the environment or set it to "$flutterStandardUrlDotGit". '
          'If this is intentional, it is recommended to use "git" directly to manage the SDK.',
        ),
      );
    });

    testWithoutContext('does not return error at standard ssh url with FLUTTER_GIT_URL unset', () {
      expect(runUpstreamValidator(versionUpstreamUrl: flutterStandardSshUrlDotGit), isNull);
    });

    testWithoutContext('does not return error at full ssh url with FLUTTER_GIT_URL unset', () {
      expect(runUpstreamValidator(versionUpstreamUrl: flutterFullSshUrlDotGit), isNull);
    });

    testWithoutContext('stripDotGit removes ".git" suffix if any', () {
      expect(
        VersionUpstreamValidator.stripDotGit('https://github.com/flutter/flutter.git'),
        'https://github.com/flutter/flutter',
      );
      expect(
        VersionUpstreamValidator.stripDotGit('https://github.com/flutter/flutter'),
        'https://github.com/flutter/flutter',
      );
      expect(
        VersionUpstreamValidator.stripDotGit('git@github.com:flutter/flutter.git'),
        'git@github.com:flutter/flutter',
      );
      expect(
        VersionUpstreamValidator.stripDotGit('git@github.com:flutter/flutter'),
        'git@github.com:flutter/flutter',
      );
      expect(
        VersionUpstreamValidator.stripDotGit('https://githubmirror.com/flutter/flutter.git.git'),
        'https://githubmirror.com/flutter/flutter.git',
      );
      expect(
        VersionUpstreamValidator.stripDotGit('https://githubmirror.com/flutter/flutter.gitgit'),
        'https://githubmirror.com/flutter/flutter.gitgit',
      );
    });
  });

  testUsingContext(
    'version handles unknown branch',
    () async {
      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            '-n',
            '1',
            '--pretty=format:%H',
          ],
          stdout: '1234abcd',
        ),
        const FakeCommand(command: <String>['git', 'tag', '--points-at', '1234abcd']),
        const FakeCommand(
          command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', '1234abcd'],
          stdout: '0.1.2-3-1234abcd',
        ),
        const FakeCommand(
          command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
          stdout: 'feature-branch',
        ),
      ]);

      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: '/path/to/flutter',
      );
      expect(flutterVersion.channel, '[user-branch]');
      expect(flutterVersion.getVersionString(), 'feature-branch/1234abcd');
      expect(flutterVersion.getBranchName(), 'feature-branch');
      expect(
        flutterVersion.getVersionString(redactUnknownBranches: true),
        '[user-branch]/1234abcd',
      );
      expect(flutterVersion.getBranchName(redactUnknownBranches: true), '[user-branch]');

      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext(
    'ensureVersionFile() writes version information to disk',
    () async {
      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            '-n',
            '1',
            '--pretty=format:%H',
          ],
          stdout: '1234abcd',
        ),
        const FakeCommand(command: <String>['git', 'tag', '--points-at', '1234abcd']),
        const FakeCommand(
          command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', '1234abcd'],
          stdout: '0.1.2-3-1234abcd',
        ),
        const FakeCommand(
          command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
          stdout: 'feature-branch',
        ),
        const FakeCommand(
          command: <String>['git', 'rev-parse', '--abbrev-ref', '--symbolic', '@{upstream}'],
        ),
        FakeCommand(
          command: const <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            'HEAD',
            '-n',
            '1',
            '--pretty=format:%ad',
            '--date=iso',
          ],
          stdout:
              _testClock
                  .ago(VersionFreshnessValidator.versionAgeConsideredUpToDate('stable') ~/ 2)
                  .toString(),
        ),
        FakeCommand(
          command: const <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            'abcdefg',
            '-n',
            '1',
            '--pretty=format:%ad',
            '--date=iso',
          ],
          stdout:
              _testClock
                  .ago(VersionFreshnessValidator.versionAgeConsideredUpToDate('stable') ~/ 2)
                  .toString(),
        ),
      ]);

      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory flutterRoot = fs.directory('/path/to/flutter');
      flutterRoot.childDirectory('bin').childDirectory('cache').createSync(recursive: true);
      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: flutterRoot.path,
      );

      final File versionFile = fs.file('/path/to/flutter/bin/cache/flutter.version.json');
      expect(versionFile.existsSync(), isFalse);

      flutterVersion.ensureVersionFile();
      expect(versionFile.existsSync(), isTrue);
      expect(versionFile.readAsStringSync(), '''
{
  "frameworkVersion": "0.0.0-unknown",
  "channel": "[user-branch]",
  "repositoryUrl": "unknown source",
  "frameworkRevision": "1234abcd",
  "frameworkCommitDate": "2014-10-02 00:00:00.000Z",
  "engineRevision": "abcdefg",
  "engineCommitDate": "2014-10-02 00:00:00.000Z",
  "dartSdkVersion": "2.12.0",
  "devToolsVersion": "2.8.0",
  "flutterVersion": "0.0.0-unknown"
}''');
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext(
    'version does not call git if a .version.json file exists',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory flutterRoot = fs.directory('/path/to/flutter');
      final Directory cacheDir = flutterRoot.childDirectory('bin').childDirectory('cache')
        ..createSync(recursive: true);
      const String devToolsVersion = '0000000';
      const Map<String, Object> versionJson = <String, Object>{
        'channel': 'stable',
        'frameworkVersion': '1.2.3',
        'repositoryUrl': 'https://github.com/flutter/flutter.git',
        'frameworkRevision': '1234abcd',
        'frameworkCommitDate': '2023-04-28 12:34:56 -0400',
        'engineRevision': 'deadbeef',
        'dartSdkVersion': 'deadbeef2',
        'devToolsVersion': devToolsVersion,
        'flutterVersion': 'foo',
      };
      cacheDir.childFile('flutter.version.json').writeAsStringSync(jsonEncode(versionJson));
      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: flutterRoot.path,
      );
      expect(flutterVersion.channel, 'stable');
      expect(flutterVersion.getVersionString(), 'stable/1.2.3');
      expect(flutterVersion.getBranchName(), 'stable');
      expect(flutterVersion.dartSdkVersion, 'deadbeef2');
      expect(flutterVersion.devToolsVersion, devToolsVersion);
      expect(flutterVersion.engineRevision, 'deadbeef');

      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext(
    '_FlutterVersionFromFile.ensureVersionFile ensures legacy version file exists',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory flutterRoot = fs.directory('/path/to/flutter');
      final Directory cacheDir = flutterRoot.childDirectory('bin').childDirectory('cache')
        ..createSync(recursive: true);
      const String devToolsVersion = '0000000';
      final File legacyVersionFile = flutterRoot.childFile('version');
      const Map<String, Object> versionJson = <String, Object>{
        'channel': 'stable',
        'frameworkVersion': '1.2.3',
        'repositoryUrl': 'https://github.com/flutter/flutter.git',
        'frameworkRevision': '1234abcd',
        'frameworkCommitDate': '2023-04-28 12:34:56 -0400',
        'engineRevision': 'deadbeef',
        'dartSdkVersion': 'deadbeef2',
        'devToolsVersion': devToolsVersion,
        'flutterVersion': 'foo',
      };
      cacheDir.childFile('flutter.version.json').writeAsStringSync(jsonEncode(versionJson));
      expect(legacyVersionFile.existsSync(), isFalse);
      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: flutterRoot.path,
      );
      flutterVersion.ensureVersionFile();
      expect(legacyVersionFile.existsSync(), isTrue);
      expect(legacyVersionFile.readAsStringSync(), '1.2.3');
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext(
    '_FlutterVersionFromFile ignores engineCommitDate if historically omitted',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory flutterRoot = fs.directory('/path/to/flutter');
      final Directory cacheDir = flutterRoot.childDirectory('bin').childDirectory('cache')
        ..createSync(recursive: true);

      const Map<String, Object> versionJson = <String, Object>{
        'channel': 'stable',
        'frameworkVersion': '1.2.3',
        'repositoryUrl': 'https://github.com/flutter/flutter.git',
        'frameworkRevision': '1234abcd',
        'frameworkCommitDate': '2023-04-28 12:34:56 -0400',
        'engineRevision': 'deadbeef',
        'dartSdkVersion': 'deadbeef2',
        'devToolsVersion': '0000000',
        'flutterVersion': 'foo',
      };
      cacheDir.childFile('flutter.version.json').writeAsStringSync(jsonEncode(versionJson));

      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            '-n',
            '1',
            '--pretty=format:%ar',
          ],
          stdout: '1 second ago',
        ),
        const FakeCommand(
          command: <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            '-n',
            '1',
            '--pretty=format:%ar',
            'deadbeef',
          ],
          stdout: '1 second ago',
        ),
      ]);

      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: flutterRoot.path,
      );
      expect(flutterVersion.engineCommitDate, isNull);
      expect(flutterVersion.toJson(), isNot(contains('engineCommitDate')));
      expect(flutterVersion.toString(), contains('Engine • revision deadbeef (1 second ago)\n'));
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext(
    'FlutterVersion() falls back to git if .version.json is malformed',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory flutterRoot = fs.directory(fs.path.join('path', 'to', 'flutter'));
      final Directory cacheDir = flutterRoot.childDirectory('bin').childDirectory('cache')
        ..createSync(recursive: true);
      final File legacyVersionFile = flutterRoot.childFile('version');
      final File versionFile = cacheDir.childFile('flutter.version.json')..writeAsStringSync('{');

      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            '-n',
            '1',
            '--pretty=format:%H',
          ],
          stdout: '1234abcd',
        ),
        const FakeCommand(command: <String>['git', 'tag', '--points-at', '1234abcd']),
        const FakeCommand(
          command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', '1234abcd'],
          stdout: '0.1.2-3-1234abcd',
        ),
        const FakeCommand(
          command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
          stdout: 'feature-branch',
        ),
        const FakeCommand(
          command: <String>['git', 'rev-parse', '--abbrev-ref', '--symbolic', '@{upstream}'],
          stdout: 'feature-branch',
        ),
        FakeCommand(
          command: const <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            'HEAD',
            '-n',
            '1',
            '--pretty=format:%ad',
            '--date=iso',
          ],
          stdout:
              _testClock
                  .ago(VersionFreshnessValidator.versionAgeConsideredUpToDate('stable') ~/ 2)
                  .toString(),
        ),
        FakeCommand(
          command: const <String>[
            'git',
            '-c',
            'log.showSignature=false',
            'log',
            'abcdefg',
            '-n',
            '1',
            '--pretty=format:%ad',
            '--date=iso',
          ],
          stdout:
              _testClock
                  .ago(VersionFreshnessValidator.versionAgeConsideredUpToDate('stable') ~/ 2)
                  .toString(),
        ),
      ]);

      // version file exists in a malformed state
      expect(versionFile.existsSync(), isTrue);
      final FlutterVersion flutterVersion = FlutterVersion(
        clock: _testClock,
        fs: fs,
        flutterRoot: flutterRoot.path,
      );

      // version file was deleted because it couldn't be parsed
      expect(versionFile.existsSync(), isFalse);
      expect(legacyVersionFile.existsSync(), isFalse);
      // version file was written to disk
      flutterVersion.ensureVersionFile();
      expect(processManager, hasNoRemainingExpectations);
      expect(versionFile.existsSync(), isTrue);
      expect(legacyVersionFile.existsSync(), isTrue);
    },
    overrides: <Type, Generator>{ProcessManager: () => processManager, Cache: () => cache},
  );

  testUsingContext('GitTagVersion', () {
    const String hash = 'abcdef';
    GitTagVersion gitTagVersion;

    // Master channel
    gitTagVersion = GitTagVersion.parse('1.2.0-4.5.pre-13-g$hash');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.0-5.0.pre.13');
    expect(gitTagVersion.gitTag, '1.2.0-4.5.pre');
    expect(gitTagVersion.devVersion, 4);
    expect(gitTagVersion.devPatch, 5);

    // Stable channel
    gitTagVersion = GitTagVersion.parse('1.2.3');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.3');
    expect(gitTagVersion.x, 1);
    expect(gitTagVersion.y, 2);
    expect(gitTagVersion.z, 3);
    expect(gitTagVersion.devVersion, null);
    expect(gitTagVersion.devPatch, null);

    // Beta channel
    gitTagVersion = GitTagVersion.parse('1.2.3-4.5.pre');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.3-4.5.pre');
    expect(gitTagVersion.gitTag, '1.2.3-4.5.pre');
    expect(gitTagVersion.devVersion, 4);
    expect(gitTagVersion.devPatch, 5);

    gitTagVersion = GitTagVersion.parse('1.2.3-13-g$hash');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.4-0.0.pre.13');
    expect(gitTagVersion.gitTag, '1.2.3');
    expect(gitTagVersion.devVersion, null);
    expect(gitTagVersion.devPatch, null);

    // new tag release format, beta channel
    gitTagVersion = GitTagVersion.parse('1.2.3-4.5.pre-0-g$hash');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.3-4.5.pre');
    expect(gitTagVersion.gitTag, '1.2.3-4.5.pre');
    expect(gitTagVersion.devVersion, 4);
    expect(gitTagVersion.devPatch, 5);

    // new tag release format, stable channel
    gitTagVersion = GitTagVersion.parse('1.2.3-13-g$hash');
    expect(gitTagVersion.frameworkVersionFor(hash), '1.2.4-0.0.pre.13');
    expect(gitTagVersion.gitTag, '1.2.3');
    expect(gitTagVersion.devVersion, null);
    expect(gitTagVersion.devPatch, null);

    expect(
      GitTagVersion.parse('98.76.54-32-g$hash').frameworkVersionFor(hash),
      '98.76.55-0.0.pre.32',
    );
    expect(GitTagVersion.parse('10.20.30-0-g$hash').frameworkVersionFor(hash), '10.20.30');
    expect(testLogger.traceText, '');
    expect(
      GitTagVersion.parse('v1.2.3+hotfix.1-4-g$hash').frameworkVersionFor(hash),
      '0.0.0-unknown',
    );
    expect(GitTagVersion.parse('x1.2.3-4-g$hash').frameworkVersionFor(hash), '0.0.0-unknown');
    expect(
      GitTagVersion.parse('1.0.0-unknown-0-g$hash').frameworkVersionFor(hash),
      '0.0.0-unknown',
    );
    expect(GitTagVersion.parse('beta-1-g$hash').frameworkVersionFor(hash), '0.0.0-unknown');
    expect(GitTagVersion.parse('1.2.3-4-gx$hash').frameworkVersionFor(hash), '0.0.0-unknown');
    expect(testLogger.statusText, '');
    expect(testLogger.errorText, '');
    expect(
      testLogger.traceText,
      'Could not interpret results of "git describe": v1.2.3+hotfix.1-4-gabcdef\n'
      'Could not interpret results of "git describe": x1.2.3-4-gabcdef\n'
      'Could not interpret results of "git describe": 1.0.0-unknown-0-gabcdef\n'
      'Could not interpret results of "git describe": beta-1-gabcdef\n'
      'Could not interpret results of "git describe": 1.2.3-4-gxabcdef\n',
    );
  });

  testUsingContext('determine reports correct stable version if HEAD is at a tag', () {
    const String stableTag = '1.2.3';
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(command: <String>['git', 'tag', '--points-at', 'HEAD'], stdout: stableTag),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();
    final GitTagVersion gitTagVersion = GitTagVersion.determine(
      processUtils,
      platform,
      workingDirectory: '.',
    );
    expect(gitTagVersion.frameworkVersionFor('abcd1234'), stableTag);
  });

  testUsingContext('determine favors stable tag over beta tag if both identify HEAD', () {
    const String stableTag = '1.2.3';
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['git', 'tag', '--points-at', 'HEAD'],
        // This tests the unlikely edge case where a beta release made it to stable without any cherry picks
        stdout: '1.2.3-6.0.pre\n$stableTag',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();

    final GitTagVersion gitTagVersion = GitTagVersion.determine(
      processUtils,
      platform,
      workingDirectory: '.',
    );
    expect(gitTagVersion.frameworkVersionFor('abcd1234'), stableTag);
  });

  testUsingContext('determine reports correct git describe version if HEAD is not at a tag', () {
    const String devTag = '1.2.0-2.0.pre';
    const String headRevision = 'abcd1234';
    const String commitsAhead = '12';
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['git', 'tag', '--points-at', 'HEAD'],
        // no output, since there's no tag
      ),
      const FakeCommand(
        command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', 'HEAD'],
        stdout: '$devTag-$commitsAhead-g$headRevision',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();

    final GitTagVersion gitTagVersion = GitTagVersion.determine(
      processUtils,
      platform,
      workingDirectory: '.',
    );
    // reported version should increment the m
    expect(gitTagVersion.frameworkVersionFor(headRevision), '1.2.0-3.0.pre.12');
  });

  testUsingContext('determine does not call fetch --tags', () {
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(command: <String>['git', 'tag', '--points-at', 'HEAD']),
      const FakeCommand(
        command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', 'HEAD'],
        stdout: 'v0.1.2-3-1234abcd',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();

    GitTagVersion.determine(processUtils, platform, workingDirectory: '.');
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testUsingContext('determine does not fetch tags on beta', () {
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
        stdout: 'beta',
      ),
      const FakeCommand(command: <String>['git', 'tag', '--points-at', 'HEAD']),
      const FakeCommand(
        command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', 'HEAD'],
        stdout: 'v0.1.2-3-1234abcd',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();

    GitTagVersion.determine(processUtils, platform, workingDirectory: '.', fetchTags: true);
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testUsingContext('determine calls fetch --tags on master', () {
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
        stdout: 'master',
      ),
      const FakeCommand(
        command: <String>['git', 'fetch', 'https://github.com/flutter/flutter.git', '--tags', '-f'],
      ),
      const FakeCommand(command: <String>['git', 'tag', '--points-at', 'HEAD']),
      const FakeCommand(
        command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', 'HEAD'],
        stdout: 'v0.1.2-3-1234abcd',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform();

    GitTagVersion.determine(processUtils, platform, workingDirectory: '.', fetchTags: true);
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });

  testUsingContext('determine uses overridden git url', () {
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['git', 'symbolic-ref', '--short', 'HEAD'],
        stdout: 'master',
      ),
      const FakeCommand(
        command: <String>['git', 'fetch', 'https://githubmirror.com/flutter.git', '--tags', '-f'],
      ),
      const FakeCommand(command: <String>['git', 'tag', '--points-at', 'HEAD']),
      const FakeCommand(
        command: <String>['git', 'describe', '--match', '*.*.*', '--long', '--tags', 'HEAD'],
        stdout: 'v0.1.2-3-1234abcd',
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    final FakePlatform platform = FakePlatform(
      environment: <String, String>{'FLUTTER_GIT_URL': 'https://githubmirror.com/flutter.git'},
    );

    GitTagVersion.determine(processUtils, platform, workingDirectory: '.', fetchTags: true);
    expect(fakeProcessManager, hasNoRemainingExpectations);
  });
}

class FakeCache extends Fake implements Cache {
  String? versionStamp;
  bool setVersionStamp = false;

  @override
  String get engineRevision => 'abcdefg';

  @override
  String get devToolsVersion => '2.8.0';

  @override
  String get dartSdkVersion => '2.12.0';

  @override
  void checkLockAcquired() {}

  @override
  String? getStampFor(String artifactName) {
    if (artifactName == VersionCheckStamp.flutterVersionCheckStampFile) {
      return versionStamp;
    }
    return null;
  }

  @override
  void setStampFor(String artifactName, String version) {
    if (artifactName == VersionCheckStamp.flutterVersionCheckStampFile) {
      setVersionStamp = true;
    }
  }
}
