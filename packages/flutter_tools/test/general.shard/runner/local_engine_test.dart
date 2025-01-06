// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/runner/local_engine.dart';

import '../../src/common.dart';

const String kEngineRoot = '/flutter/flutter/engine';
const String kArbitraryEngineRoot = '/arbitrary/engine';
const String kPackageConfig = '.dart_tool/package_config.json';

void main() {
  testWithoutContext('works if --local-engine is specified and --local-engine-src-path '
      'is determined by sky_engine', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem
        .directory('$kArbitraryEngineRoot/src/out/ios_debug/gen/dart-pkg/sky_engine/lib/')
        .createSync(recursive: true);
    fileSystem.directory('$kArbitraryEngineRoot/src/out/host_debug').createSync(recursive: true);
    fileSystem.file(kPackageConfig)
      ..createSync(recursive: true)
      ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "sky_engine",
      "rootUri": "file://$kArbitraryEngineRoot/src/out/ios_debug/gen/dart-pkg/sky_engine",
      "packageUri": "lib/"
    }
  ]
}
''');
    fileSystem.file('bin/cache/pkg/sky_engine/lib').createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: '',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: 'ios_debug',
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug',
        targetEngine: '/arbitrary/engine/src/out/ios_debug',
      ),
    );
    expect(logger.traceText, contains('Local engine source at /arbitrary/engine/src'));

    // Verify that this also works if the sky_engine path is a symlink to the engine root.
    fileSystem.link('/symlink').createSync(kArbitraryEngineRoot);
    fileSystem.file(kPackageConfig)
      ..createSync(recursive: true)
      ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "sky_engine",
      "rootUri": "file:///symlink/src/out/ios_debug/gen/dart-pkg/sky_engine/",
      "packageUri": "lib/"
    }
  ]
}
''');

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: 'ios_debug',
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/symlink/src/out/host_debug',
        targetEngine: '/symlink/src/out/ios_debug',
      ),
    );
    expect(logger.traceText, contains('Local engine source at /symlink/src'));
  });

  testWithoutContext('works if --local-engine is specified and --local-engine-src-path '
      'is specified', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    // Intentionally do not create a package_config to verify that it is not required.
    fileSystem.directory('$kArbitraryEngineRoot/src/out/ios_debug').createSync(recursive: true);
    fileSystem.directory('$kArbitraryEngineRoot/src/out/host_debug').createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: '',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        engineSourcePath: '$kArbitraryEngineRoot/src',
        localEngine: 'ios_debug',
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug',
        targetEngine: '/arbitrary/engine/src/out/ios_debug',
      ),
    );
    expect(logger.traceText, contains('Local engine source at /arbitrary/engine/src'));
  });

  testWithoutContext(
    'works if --local-engine is specified and --local-engine-host is specified',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Directory localEngine = fileSystem.directory(
        '$kArbitraryEngineRoot/src/out/android_debug_unopt_arm64/',
      )..createSync(recursive: true);
      fileSystem
          .directory('$kArbitraryEngineRoot/src/out/host_debug_unopt_arm64/')
          .createSync(recursive: true);

      final BufferLogger logger = BufferLogger.test();
      final LocalEngineLocator localEngineLocator = LocalEngineLocator(
        fileSystem: fileSystem,
        flutterRoot: 'flutter/flutter',
        logger: logger,
        userMessages: UserMessages(),
        platform: FakePlatform(environment: <String, String>{}),
      );

      expect(
        await localEngineLocator.findEnginePath(
          localEngine: localEngine.path,
          localHostEngine: 'host_debug_unopt_arm64',
        ),
        matchesEngineBuildPaths(
          hostEngine: '/arbitrary/engine/src/out/host_debug_unopt_arm64',
          targetEngine: '/arbitrary/engine/src/out/android_debug_unopt_arm64',
        ),
      );
      expect(logger.traceText, contains('Local engine source at /arbitrary/engine/src'));
    },
  );

  testWithoutContext('fails if --local-engine-host is omitted', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/android_debug_unopt_arm64/',
    )..createSync(recursive: true);
    fileSystem
        .directory('$kArbitraryEngineRoot/src/out/host_debug_unopt/')
        .createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    await expectLater(
      localEngineLocator.findEnginePath(localEngine: localEngine.path),
      throwsToolExit(
        message:
            'You are using a locally built engine (--local-engine) but have not specified --local-engine-host',
      ),
    );
  });

  testWithoutContext('works if --local-engine is specified and --local-engine-src-path '
      'is determined by --local-engine', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory('$kArbitraryEngineRoot/src/out/ios_debug/')
      ..createSync(recursive: true);
    fileSystem.directory('$kArbitraryEngineRoot/src/out/host_debug/').createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug',
        targetEngine: '/arbitrary/engine/src/out/ios_debug',
      ),
    );
    expect(
      logger.traceText,
      contains('Parsed engine source from local engine as /arbitrary/engine/src'),
    );
    expect(logger.traceText, contains('Local engine source at /arbitrary/engine/src'));
  });

  testWithoutContext('works if local engine is host engine', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory('$kArbitraryEngineRoot/src/out/host_debug/')
      ..createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: localEngine.path,
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug',
        targetEngine: '/arbitrary/engine/src/out/host_debug',
      ),
    );
    expect(logger.traceText, contains('Local engine source at /arbitrary/engine/src'));
  });

  testWithoutContext('works if local engine is host engine with suffixes', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/host_debug_unopt_arm64/',
    )..createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: localEngine.path,
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug_unopt_arm64',
        targetEngine: '/arbitrary/engine/src/out/host_debug_unopt_arm64',
      ),
    );
  });

  testWithoutContext('works if local engine is simulator', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/ios_debug_sim/',
    )..createSync(recursive: true);
    fileSystem.directory('$kArbitraryEngineRoot/src/out/host_debug/').createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug',
        targetEngine: '/arbitrary/engine/src/out/ios_debug_sim',
      ),
    );
  });

  testWithoutContext('works if local engine is simulator unoptimized', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/ios_debug_sim_unopt/',
    )..createSync(recursive: true);
    fileSystem
        .directory('$kArbitraryEngineRoot/src/out/host_debug_unopt/')
        .createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: 'host_debug_unopt',
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/host_debug_unopt',
        targetEngine: '/arbitrary/engine/src/out/ios_debug_sim_unopt',
      ),
    );
  });

  testWithoutContext('fails if host_debug does not exist', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localEngine = fileSystem.directory('$kArbitraryEngineRoot/src/out/ios_debug/')
      ..createSync(recursive: true);

    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    await expectToolExitLater(
      localEngineLocator.findEnginePath(
        localEngine: localEngine.path,
        localHostEngine: 'host_debug',
      ),
      contains('No Flutter engine build found at /arbitrary/engine/src/out/host_debug'),
    );
  });

  testWithoutContext('works if --local-engine is specified and --local-engine-src-path '
      'is determined by flutter root', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file(kPackageConfig).createSync(recursive: true);
    fileSystem.directory('$kEngineRoot/src/out/ios_debug').createSync(recursive: true);
    fileSystem.directory('$kEngineRoot/src/out/host_debug').createSync(recursive: true);
    fileSystem.file('bin/cache/pkg/sky_engine/lib').createSync(recursive: true);

    final BufferLogger logger = BufferLogger.test();
    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: logger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localEngineLocator.findEnginePath(
        localEngine: 'ios_debug',
        localHostEngine: 'host_debug',
      ),
      matchesEngineBuildPaths(
        hostEngine: 'flutter/flutter/engine/src/out/host_debug',
        targetEngine: 'flutter/flutter/engine/src/out/ios_debug',
      ),
    );
    expect(logger.traceText, contains('Local engine source at flutter/flutter/engine/src'));
  });

  testWithoutContext('fails if --local-engine is specified and --local-engine-src-path '
      'cannot be determined', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();

    final LocalEngineLocator localEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    await expectToolExitLater(
      localEngineLocator.findEnginePath(
        localEngine: '/path/to/nothing',
        localHostEngine: '/path/to/nothing',
      ),
      contains('Unable to detect local Flutter engine src directory'),
    );
  });

  testWithoutContext('works for local web engine', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Directory localWasmEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/wasm_whatever/',
    )..createSync(recursive: true);
    final Directory localWebEngine = fileSystem.directory(
      '$kArbitraryEngineRoot/src/out/web_whatever/',
    )..createSync(recursive: true);

    final BufferLogger wasmLogger = BufferLogger.test();
    final LocalEngineLocator localWasmEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: wasmLogger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localWasmEngineLocator.findEnginePath(
        localEngine: localWasmEngine.path,
        localHostEngine: localWasmEngine.path,
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/wasm_whatever',
        targetEngine: '/arbitrary/engine/src/out/wasm_whatever',
      ),
    );
    expect(wasmLogger.traceText, contains('Local engine source at /arbitrary/engine/src'));

    final BufferLogger webLogger = BufferLogger.test();
    final LocalEngineLocator localWebEngineLocator = LocalEngineLocator(
      fileSystem: fileSystem,
      flutterRoot: 'flutter/flutter',
      logger: webLogger,
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    expect(
      await localWebEngineLocator.findEnginePath(
        localEngine: localWebEngine.path,
        localHostEngine: localWebEngine.path,
      ),
      matchesEngineBuildPaths(
        hostEngine: '/arbitrary/engine/src/out/web_whatever',
        targetEngine: '/arbitrary/engine/src/out/web_whatever',
      ),
    );
    expect(webLogger.traceText, contains('Local engine source at /arbitrary/engine/src'));
  });

  test('returns null without throwing if nothing is specified', () async {
    final LocalEngineLocator localWebEngineLocator = LocalEngineLocator(
      fileSystem: MemoryFileSystem.test(),
      flutterRoot: 'flutter/flutter',
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{}),
    );

    final EngineBuildPaths? paths = await localWebEngineLocator.findEnginePath();
    expect(paths, isNull);
  });

  test(
    'throws if nothing is specified but the FLUTTER_ENGINE environment variable is set',
    () async {
      final LocalEngineLocator localWebEngineLocator = LocalEngineLocator(
        fileSystem: MemoryFileSystem.test(),
        flutterRoot: 'flutter/flutter',
        logger: BufferLogger.test(),
        userMessages: UserMessages(),
        platform: FakePlatform(environment: <String, String>{'FLUTTER_ENGINE': 'blah'}),
      );

      await expectToolExitLater(
        localWebEngineLocator.findEnginePath(),
        contains('Unable to detect a Flutter engine build directory in blah'),
      );
    },
  );
}

Matcher matchesEngineBuildPaths({String? hostEngine, String? targetEngine}) {
  return const TypeMatcher<EngineBuildPaths>()
      .having((EngineBuildPaths paths) => paths.hostEngine, 'hostEngine', hostEngine)
      .having((EngineBuildPaths paths) => paths.targetEngine, 'targetEngine', targetEngine);
}
