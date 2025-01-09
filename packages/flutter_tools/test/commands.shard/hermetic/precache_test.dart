// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/precache.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late FakeCache cache;

  setUp(() {
    cache = FakeCache();
    cache.isUpToDateValue = false;
  });

  testUsingContext('precache should acquire lock', () async {
    final Platform platform = FakePlatform(environment: <String, String>{});
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      platform: platform,
      featureFlags: TestFeatureFlags(),
    );
    await createTestCommandRunner(command).run(const <String>['precache']);

    expect(cache.locked, true);
  });

  testUsingContext('precache should not re-entrantly acquire lock', () async {
    final Platform platform = FakePlatform(
      operatingSystem: 'windows',
      environment: <String, String>{'FLUTTER_ROOT': 'flutter', 'FLUTTER_ALREADY_LOCKED': 'true'},
    );
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(),
      platform: platform,
    );
    await createTestCommandRunner(command).run(const <String>['precache']);

    expect(cache.locked, false);
  });

  testUsingContext(
    'precache downloads web artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(isWebEnabled: true),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--web', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{
          DevelopmentArtifact.universal,
          DevelopmentArtifact.web,
        }),
      );
    },
  );

  testUsingContext(
    'precache does not download web artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--web', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{DevelopmentArtifact.universal}),
      );
    },
  );

  testUsingContext(
    'precache downloads macOS artifacts on dev branch when macOS is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(isMacOSEnabled: true),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--macos', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{
          DevelopmentArtifact.universal,
          DevelopmentArtifact.macOS,
        }),
      );
    },
  );

  testUsingContext(
    'precache does not download macOS artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--macos', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{DevelopmentArtifact.universal}),
      );
    },
  );

  testUsingContext(
    'precache downloads Windows artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(isWindowsEnabled: true),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--windows', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{
          DevelopmentArtifact.universal,
          DevelopmentArtifact.windows,
        }),
      );
    },
  );

  testUsingContext(
    'precache does not download Windows artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--windows', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{DevelopmentArtifact.universal}),
      );
    },
  );

  testUsingContext(
    'precache downloads Linux artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(isLinuxEnabled: true),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--linux', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{
          DevelopmentArtifact.universal,
          DevelopmentArtifact.linux,
        }),
      );
    },
  );

  testUsingContext(
    'precache does not download Linux artifacts on dev branch when feature is enabled.',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(),
        platform: FakePlatform(environment: <String, String>{}),
      );
      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--linux', '--no-android', '--no-ios']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{DevelopmentArtifact.universal}),
      );
    },
  );

  testUsingContext('precache exits if requesting mismatched artifacts.', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--no-android', '--android_gen_snapshot']),
      throwsToolExit(message: '--android_gen_snapshot requires --android'),
    );
  });

  testUsingContext('precache adds artifact flags to requested artifacts', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(
        isWebEnabled: true,
        isLinuxEnabled: true,
        isMacOSEnabled: true,
        isWindowsEnabled: true,
        isFuchsiaEnabled: true,
      ),
      platform: FakePlatform(environment: <String, String>{}),
    );
    await createTestCommandRunner(command).run(const <String>[
      'precache',
      '--ios',
      '--android',
      '--web',
      '--macos',
      '--linux',
      '--windows',
      '--fuchsia',
      '--flutter_runner',
    ]);
    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
        DevelopmentArtifact.iOS,
        DevelopmentArtifact.androidGenSnapshot,
        DevelopmentArtifact.androidMaven,
        DevelopmentArtifact.androidInternalBuild,
        DevelopmentArtifact.web,
        DevelopmentArtifact.macOS,
        DevelopmentArtifact.linux,
        DevelopmentArtifact.windows,
        DevelopmentArtifact.fuchsia,
        DevelopmentArtifact.flutterRunner,
      }),
    );
  });

  testUsingContext('precache expands android artifacts when the android flag is used', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(),
      platform: FakePlatform(environment: <String, String>{}),
    );
    await createTestCommandRunner(command).run(const <String>['precache', '--no-ios', '--android']);
    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
        DevelopmentArtifact.androidGenSnapshot,
        DevelopmentArtifact.androidMaven,
        DevelopmentArtifact.androidInternalBuild,
      }),
    );
  });

  testUsingContext('precache adds artifact flags to requested android artifacts', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(),
      platform: FakePlatform(environment: <String, String>{}),
    );
    await createTestCommandRunner(command).run(const <String>[
      'precache',
      '--no-ios',
      '--android',
      '--android_gen_snapshot',
      '--android_maven',
      '--android_internal_build',
    ]);
    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
        DevelopmentArtifact.androidGenSnapshot,
        DevelopmentArtifact.androidMaven,
        DevelopmentArtifact.androidInternalBuild,
      }),
    );
  });

  testUsingContext('precache downloads iOS and Android artifacts by default', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    await createTestCommandRunner(command).run(const <String>['precache']);

    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
        DevelopmentArtifact.iOS,
        DevelopmentArtifact.androidGenSnapshot,
        DevelopmentArtifact.androidMaven,
        DevelopmentArtifact.androidInternalBuild,
      }),
    );
  });

  testUsingContext('precache --all-platforms gets all artifacts', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(
        isWebEnabled: true,
        isLinuxEnabled: true,
        isMacOSEnabled: true,
        isWindowsEnabled: true,
        isFuchsiaEnabled: true,
      ),
      platform: FakePlatform(environment: <String, String>{}),
    );

    await createTestCommandRunner(command).run(const <String>['precache', '--all-platforms']);

    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
        DevelopmentArtifact.iOS,
        DevelopmentArtifact.androidGenSnapshot,
        DevelopmentArtifact.androidMaven,
        DevelopmentArtifact.androidInternalBuild,
        DevelopmentArtifact.web,
        DevelopmentArtifact.macOS,
        DevelopmentArtifact.linux,
        DevelopmentArtifact.windows,
        DevelopmentArtifact.fuchsia,
        DevelopmentArtifact.flutterRunner,
      }),
    );
  });

  testUsingContext(
    'precache with default artifacts does not override platform filtering',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(),
        platform: FakePlatform(environment: <String, String>{}),
      );

      await createTestCommandRunner(command).run(const <String>['precache']);

      expect(cache.platformOverrideArtifacts, <String>{});
    },
  );

  testUsingContext(
    'precache with explicit artifact options overrides platform filtering',
    () async {
      final PrecacheCommand command = PrecacheCommand(
        cache: cache,
        logger: BufferLogger.test(),
        featureFlags: TestFeatureFlags(isMacOSEnabled: true),
        platform: FakePlatform(
          operatingSystem: 'windows',
          environment: <String, String>{
            'FLUTTER_ROOT': 'flutter',
            'FLUTTER_ALREADY_LOCKED': 'true',
          },
        ),
      );

      await createTestCommandRunner(
        command,
      ).run(const <String>['precache', '--no-ios', '--no-android', '--macos']);

      expect(
        cache.artifacts,
        unorderedEquals(<DevelopmentArtifact>{
          DevelopmentArtifact.universal,
          DevelopmentArtifact.macOS,
        }),
      );
      expect(cache.platformOverrideArtifacts, <String>{'macos'});
    },
  );

  testUsingContext('precache deletes artifact stampfiles when --force is provided', () async {
    cache.isUpToDateValue = true;
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(isMacOSEnabled: true),
      platform: FakePlatform(environment: <String, String>{}),
    );
    await createTestCommandRunner(command).run(const <String>['precache', '--force']);

    expect(cache.clearedStampFiles, true);
  });

  testUsingContext('precache downloads all enabled platforms if no flags are provided.', () async {
    final PrecacheCommand command = PrecacheCommand(
      cache: cache,
      logger: BufferLogger.test(),
      featureFlags: TestFeatureFlags(
        isWebEnabled: true,
        isLinuxEnabled: true,
        isWindowsEnabled: true,
        isMacOSEnabled: true,
        isIOSEnabled: false,
        isAndroidEnabled: false,
      ),
      platform: FakePlatform(environment: <String, String>{}),
    );
    await createTestCommandRunner(command).run(const <String>['precache']);

    expect(
      cache.artifacts,
      unorderedEquals(<DevelopmentArtifact>{
        DevelopmentArtifact.web,
        DevelopmentArtifact.macOS,
        DevelopmentArtifact.windows,
        DevelopmentArtifact.linux,
        DevelopmentArtifact.universal,
        // iOS and android specifically excluded
      }),
    );
  });
}

class FakeCache extends Fake implements Cache {
  bool isUpToDateValue = false;
  bool clearedStampFiles = false;
  bool locked = false;
  Set<DevelopmentArtifact>? artifacts;

  @override
  Future<void> lock() async {
    locked = true;
  }

  @override
  void releaseLock() {
    locked = false;
  }

  @override
  Future<bool> isUpToDate() async => isUpToDateValue;

  @override
  Future<void> updateAll(Set<DevelopmentArtifact> requiredArtifacts, {bool offline = false}) async {
    artifacts = requiredArtifacts;
  }

  @override
  void clearStampFiles() {
    clearedStampFiles = true;
  }

  @override
  Set<String>? platformOverrideArtifacts;

  @override
  bool includeAllPlatforms = false;
}
