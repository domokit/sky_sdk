// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_tool/src/build_plan.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:test/test.dart';

import 'src/test_build_configs.dart';
import 'src/utils.dart';

void main() {
  /// Configures and parses [BuildPlan] from [args].
  ///
  /// By default a non-verbose, non-RBE, Linux x64 configured [BuildPlan]
  /// is created and used to configure the available options and instructions
  /// for the returned [ArgParser].
  ///
  /// - To use a different [TestEnvironment], provide [environment].
  /// - To use a different set of builds, provide [builds].
  BuildPlan configureAndParse(
    List<String> args, {
    TestEnvironment? environment,
    void Function(TestBuilderConfig)? builds,
  }) {
    final (builders, parser, env) = _createTestFixture(environment: environment, builds: builds);

    return BuildPlan.fromArgResults(parser.parse(args), env, builds: builders);
  }

  test('rbe defaults to true if detected', () {
    final plan = configureAndParse([], environment: TestEnvironment.withTestEngine(withRbe: true));

    expect(plan.useRbe, isTrue);
    expect(plan.toGnArgs(), isNot(contains('--no-rbe')));
  });

  test('rbe defaults to false if not detected', () {
    final plan = configureAndParse(
      [],
      environment: TestEnvironment.withTestEngine(
        // This is the default, but make it explicit for the test.
        // ignore: avoid_redundant_argument_values
        withRbe: false,
      ),
    );

    expect(plan.useRbe, isFalse);
    expect(plan.toGnArgs(), contains('--no-rbe'));
  });

  test('lto is true if explicitly enabled', () {
    final plan = configureAndParse(['--lto']);

    expect(plan.useLto, isTrue);
    expect(plan.toGnArgs(), contains('--lto'));
  });

  test('lto is false if explicitly disabled', () {
    final plan = configureAndParse(['--no-lto']);

    expect(plan.useLto, isFalse);
    expect(plan.toGnArgs(), contains('--no-lto'));
  });

  test('lto is true if the config omits --no-lto', () {
    final plan = configureAndParse([]);

    expect(
      plan.useLto,
      isTrue,
      reason: 'Not specified and the build config did not include --no-lto',
    );
    expect(plan.toGnArgs(), contains('--lto'));
  });

  test('lto is false if the config uses --no-lto', () {
    final plan = configureAndParse(
      [],
      builds: (testConfig) {
        testConfig.addBuild(
          name: 'linux/host_debug',
          dimension: TestDroneDimension.linux,
          enableLto: false,
        );
      },
    );

    expect(plan.useLto, isFalse, reason: 'Not specified and the build config included --no-lto');
    expect(plan.toGnArgs(), contains('--no-lto'));
  });

  test('concurrency defaults to null if not specified', () {
    final plan = configureAndParse([]);

    expect(plan.concurrency, isNull);
  });

  test('concurrency parses the number provided', () {
    final plan = configureAndParse(['--concurrency=1024']);

    expect(plan.concurrency, 1024);
  });

  test('strategy defaults to auto', () {
    final plan = configureAndParse([]);

    expect(plan.strategy, BuildStrategy.auto);
    expect(
      plan.toRbeConfig(),
      same(const RbeConfig()),
      reason: 'Auto should use the default RbeConfig instance.',
    );
  });

  test('strategy can be set to --local', () {
    final plan = configureAndParse(['--build-strategy=local']);

    expect(plan.strategy, BuildStrategy.local);
    expect(
      plan.toRbeConfig(),
      same(const RbeConfig(execStrategy: RbeExecStrategy.local, remoteDisabled: true)),
      reason: 'Local should use RbeExecStrategy.local with RBE disabled',
    );
  });

  test('strategy can be set to --remote', () {
    final plan = configureAndParse([
      '--build-strategy=remote',
    ], environment: TestEnvironment.withTestEngine(withRbe: true));

    expect(plan.strategy, BuildStrategy.remote);
    expect(
      plan.toRbeConfig(),
      same(const RbeConfig(execStrategy: RbeExecStrategy.remote)),
      reason: 'Local should use RbeExecStrategy.remote',
    );
  });

  test('can provide a single extra "--gn-args"', () {
    final result = configureAndParse(['--gn-args', '--foo']);
    expect(result.extraGnArgs, ['--foo']);
  });

  test('can provide multiple extra "--gn-arg"s', () {
    final result = configureAndParse(['--gn-args', '--foo', '--gn-args', '--bar']);
    expect(result.extraGnArgs, ['--foo', '--bar']);
  });

  test('build defaults to host_debug', () {
    final plan = configureAndParse([]);

    expect(plan.build.name, 'linux/host_debug');
  });

  test('build defaults to the provided default', () {
    final testEnv = TestEnvironment.withTestEngine(withRbe: true);
    addTearDown(testEnv.cleanup);

    final testConfig = TestBuilderConfig();
    testConfig.addBuild(name: 'linux/host_debug_unopt_arm64', dimension: TestDroneDimension.linux);

    final parser = ArgParser();
    final builds = BuildPlan.configureArgParser(
      parser,
      testEnv.environment,
      configs: {
        'linux_test_config': testConfig.buildConfig(path: 'ci/builders/linux_test_config.json'),
      },
      help: false,
    );

    final plan = BuildPlan.fromArgResults(
      parser.parse([]),
      testEnv.environment,
      builds: builds,
      defaultBuild: () => 'host_debug_unopt_arm64',
    );

    expect(plan.build.name, 'linux/host_debug_unopt_arm64');
  });

  // These tests should strictly check for build failures as a result of
  // invalid combination of flags or flags with invalid values; i.e. a valid
  // BuildPlan is not returned.
  group('prevents invalid flags', () {
    test('can provide only long-form "--gn-arg"s', () {
      expect(
        () => configureAndParse(['--gn-args', '-F']),
        throwsA(BuildPlan.argumentsMustBeFlagsError),
      );
    });

    test('strategy of --remote with RBE disabled fails', () {
      expect(
        () => configureAndParse(
          ['--build-strategy=remote'],
          environment: TestEnvironment.withTestEngine(
            // This is the default, but make it explicit for the test.
            // ignore: avoid_redundant_argument_values
            withRbe: false,
          ),
        ),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('Cannot use remote builds without RBE enabled'),
          ),
        ),
      );
    });

    test('rbe forced to true if not detected is an error', () {
      expect(
        () => configureAndParse(
          ['--rbe'],
          environment: TestEnvironment.withTestEngine(
            // This is the default, but make it explicit for the test.
            // ignore: avoid_redundant_argument_values
            withRbe: false,
          ),
        ),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('RBE requested but configuration not found'),
          ),
        ),
      );
    });

    test('concurrency fails on a non-integer', () {
      expect(
        () => configureAndParse(['--concurrency=ABCD']),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('Invalid value for --concurrency: ABCD'),
          ),
        ),
      );
    });

    test('concurrency fails on a negative integer', () {
      expect(
        () => configureAndParse(['--concurrency=-1024']),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('Invalid value for --concurrency: -1024'),
          ),
        ),
      );
    });

    test('build fails if host_debug not specified and no config set', () {
      expect(
        () => configureAndParse(
          [],
          builds: (testConfig) {
            testConfig.addBuild(
              name: 'linux/host_debug_unopt_arm64',
              dimension: TestDroneDimension.linux,
            );
          },
        ),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('Unknown build configuration: host_debug'),
          ),
        ),
      );
    });

    group('build fails if an extra "--gn-args" contains a reserved flag', () {
      for (final reserved in BuildPlan.reservedGnArgs) {
        test(reserved, () {
          expect(
            () => configureAndParse(['--gn-args', '--$reserved']),
            throwsA(
              isA<FatalError>().having(
                (e) => e.toString(),
                'toString()',
                contains(BuildPlan.reservedGnArgsError.toString()),
              ),
            ),
          );
        });
      }
    });

    test('builds fails if a non-flag (space) is provided to "--gn-args"', () {
      expect(
        () => configureAndParse(['--gn-args', '--foo name']),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains(BuildPlan.argumentsMustBeFlagsError.toString()),
          ),
        ),
      );
    });

    test('builds fails if a non-flag (with =) is provided to "--gn-args"', () {
      expect(
        () => configureAndParse(['--gn-args', '--foo=name']),
        throwsA(
          isA<FatalError>().having(
            (e) => e.toString(),
            'toString()',
            contains('Arguments provided to --gn-args must be flags'),
          ),
        ),
      );
    });

    test('build fails if a config not available is requested', () {
      expect(
        () => configureAndParse(['--config=host_debug_unopt_arm64']),
        throwsA(
          isA<ArgParserException>().having(
            (e) => e.toString(),
            'toString()',
            contains('"host_debug_unopt_arm64" is not an allowed value for option'),
          ),
        ),
      );
    });
  });

  // These tests should strictly check the usage (--help) output.
  group('shows instructions based on verbosity', () {
    /// Creates a configured [ArgParser] based on then environemnt and builds.
    ///
    /// By default a non-verbose, non-RBE, Linux x64 configured [BuildPlan]
    /// is created and used to configure the available options and instructions
    /// for the returned [ArgParser].
    ///
    /// - To use a different [TestEnvironment], provide [environment].
    /// - To use a different set of builds, provide [builds].
    ArgParser createArgParser({
      TestEnvironment? environment,
      void Function(TestBuilderConfig)? builds,
    }) {
      final (_, parser, _) = _createTestFixture(environment: environment, builds: builds);
      return parser;
    }

    test('show builds in help message as long as not a [ci/...] build', () {
      final parser = createArgParser(
        builds: (testConfig) {
          testConfig.addBuild(name: 'ci/host_debug', dimension: TestDroneDimension.linux);
          testConfig.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
        },
      );

      expect(parser.usage, contains('host_debug'));
      expect(parser.usage, isNot(contains('ci/host_debug')));
    });

    test('shows [ci/...] builds if verbose is true', () {
      final parser = createArgParser(
        environment: TestEnvironment.withTestEngine(verbose: true),
        builds: (testConfig) {
          testConfig.addBuild(name: 'ci/host_debug', dimension: TestDroneDimension.linux);
          testConfig.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
        },
      );

      expect(parser.usage, contains('host_debug'));
      expect(parser.usage, contains('ci/host_debug'));
    });

    test('hides LTO instructions normally', () {
      final parser = createArgParser();

      expect(parser.usage, isNot(contains('Whether LTO should be enabled for a build')));
    });

    test('shows LTO instructions if verbose', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(verbose: true));

      expect(parser.usage, contains('Whether LTO should be enabled for a build'));
    });

    test('shows RBE instructions if not configured', () {
      final parser = createArgParser(
        environment: TestEnvironment.withTestEngine(
          // This is the default, but make it explicit for the test.
          // ignore: avoid_redundant_argument_values
          withRbe: false,
        ),
      );

      expect(
        parser.usage,
        stringContainsInOrder([
          'Enable pre-configured remote build execution',
          'https://flutter.dev/to/engine-rbe',
        ]),
      );
    });

    test('shows RBE instructions if verbose', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(verbose: true));

      expect(
        parser.usage,
        stringContainsInOrder([
          'Enable pre-configured remote build execution',
          'https://flutter.dev/to/engine-rbe',
        ]),
      );
    });

    test('hides RBE instructions if enabled', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(withRbe: true));

      expect(parser.usage, contains('Enable pre-configured remote build execution'));
      expect(parser.usage, isNot(contains('https://flutter.dev/to/engine-rbe')));
    });

    test('hides --build-strategy if RBE not enabled', () {
      final parser = createArgParser(
        environment: TestEnvironment.withTestEngine(
          // This is the default, but make it explicit for the test.
          // ignore: avoid_redundant_argument_values
          withRbe: false,
        ),
      );

      expect(parser.usage, isNot(contains('How to prefer remote or local builds')));
    });

    test('shows --build-strategy if RBE enabled', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(withRbe: true));

      expect(parser.usage, contains('How to prefer remote or local builds'));
    });

    test('shows --build-strategy if verbose', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(verbose: true));

      expect(parser.usage, contains('How to prefer remote or local builds'));
    });

    test('hides --gn-args if not verbose', () {
      final parser = createArgParser();

      expect(parser.usage, isNot(contains('Additional arguments to provide to "gn"')));
    });

    test('shows --gn-args if verbose', () {
      final parser = createArgParser(environment: TestEnvironment.withTestEngine(verbose: true));

      expect(parser.usage, contains('Additional arguments to provide to "gn"'));
    });

    /// Returns an [ArgParser] pre-primed with both MacOS and Linux builds.
    ArgParser createMultiPlatformArgParser({required bool verbose}) {
      final testEnv = TestEnvironment.withTestEngine(verbose: verbose);
      addTearDown(testEnv.cleanup);

      final linuxConfig = TestBuilderConfig();
      linuxConfig.addBuild(
        name: 'linux/host_debug',
        dimension: TestDroneDimension.linux,
        description: 'A development build of the Linux host.',
      );
      linuxConfig.addBuild(
        name: 'ci/linux_host_debug',
        dimension: TestDroneDimension.linux,
        description: 'A CI-suitable development build of the Linux host.',
      );

      final macOSConfig = TestBuilderConfig();
      macOSConfig.addBuild(
        name: 'macos/host_debug',
        dimension: TestDroneDimension.mac,
        description: 'A build we do not expect to see due to filtering.',
      );

      final parser = ArgParser();
      final _ = BuildPlan.configureArgParser(
        parser,
        testEnv.environment,
        configs: {
          'linux_test_config': linuxConfig.buildConfig(path: 'ci/builders/linux_test_config.json'),
          'macos_test_config': macOSConfig.buildConfig(path: 'ci/builders/macos_test_config.json'),
        },
        help: true,
      );

      return parser;
    }

    test('shows only non-CI builds that can run locally', () {
      final parser = createMultiPlatformArgParser(verbose: false);

      expect(parser.usage, contains('[host_debug]'));
    });

    test('shows builds that can run locally, with details', () {
      final parser = createMultiPlatformArgParser(verbose: true);

      expect(
        parser.usage,
        stringContainsInOrder([
          '[ci/linux_host_debug]',
          'A CI-suitable development build of the Linux host.',
          '[host_debug]',
          'A development build of the Linux host.',
        ]),
      );
    });
  });
}

(List<Build>, ArgParser, Environment) _createTestFixture({
  TestEnvironment? environment,
  void Function(TestBuilderConfig)? builds,
}) {
  final testEnv = environment ?? TestEnvironment.withTestEngine();
  addTearDown(testEnv.cleanup);

  final testConfig = TestBuilderConfig();
  if (builds != null) {
    builds(testConfig);
  } else {
    testConfig.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
  }

  final parser = ArgParser();
  final builders = BuildPlan.configureArgParser(
    parser,
    testEnv.environment,
    configs: {
      'linux_test_config': testConfig.buildConfig(path: 'ci/builders/linux_test_config.json'),
    },
    help: true,
  );

  return (builders, parser, testEnv.environment);
}
