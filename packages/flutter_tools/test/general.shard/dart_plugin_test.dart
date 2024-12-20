// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/dart/package_map.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/flutter_plugins.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/plugins.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:package_config/package_config.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:test/fake.dart';
import 'package:yaml/yaml.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_pub_deps.dart';
import '../src/fakes.dart';

void main() {
  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  group('Dart plugin registrant', () {
    late FileSystem fs;
    late FakeFlutterProject flutterProject;
    late FakeFlutterManifest flutterManifest;

    setUp(() async {
      fs = MemoryFileSystem.test();
      final Directory directory = fs.currentDirectory.childDirectory('app');
      flutterManifest = FakeFlutterManifest();
      flutterProject =
          FakeFlutterProject()
            ..manifest = flutterManifest
            ..directory = directory
            ..flutterPluginsFile = directory.childFile('.flutter-plugins')
            ..flutterPluginsDependenciesFile = directory.childFile('.flutter-plugins-dependencies')
            ..dartPluginRegistrant = directory.childFile('dart_plugin_registrant.dart');
      flutterProject.directory
          .childDirectory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
    });

    group('resolvePlatformImplementation', () {
      testWithoutContext('selects uncontested implementation from direct dependency', () async {
        final Set<String> directDependencies = <String>{'url_launcher_linux', 'url_launcher_macos'};
        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher_linux',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'url_launcher_macos',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginMacOS'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);

        expect(resolutions.length, equals(2));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher_linux',
            'dartClass': 'UrlLauncherPluginLinux',
            'platform': 'linux',
            'dartFileName': 'url_launcher_linux.dart',
          }),
        );
        expect(
          resolutions[1].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher_macos',
            'dartClass': 'UrlLauncherPluginMacOS',
            'platform': 'macos',
            'dartFileName': 'url_launcher_macos.dart',
          }),
        );
      });

      testWithoutContext(
        'selects uncontested implementation from direct dependency with additional native implementation',
        () async {
          final Set<String> directDependencies = <String>{
            'url_launcher_linux',
            'url_launcher_macos',
          };
          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              // Following plugin is native only and is not resolved as a dart plugin:
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{
                      'package': 'com.example.url_launcher',
                      'pluginClass': 'UrlLauncherPluginLinux',
                    },
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_macos',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginMacOS'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            selectDartPluginsOnly: true,
          );

          expect(resolutions.length, equals(1));
          expect(
            resolutions[0].toMap(),
            equals(<String, String>{
              'pluginName': 'url_launcher_macos',
              'dartClass': 'UrlLauncherPluginMacOS',
              'platform': 'macos',
              'dartFileName': 'url_launcher_macos.dart',
            }),
          );
        },
      );

      testWithoutContext('selects uncontested implementation from transitive dependency', () async {
        final Set<String> directDependencies = <String>{'url_launcher_macos'};
        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher_macos',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginMacOS'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'transitive_dependency_plugin',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginWindows'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);

        expect(resolutions.length, equals(2));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher_macos',
            'dartClass': 'UrlLauncherPluginMacOS',
            'platform': 'macos',
            'dartFileName': 'url_launcher_macos.dart',
          }),
        );
        expect(
          resolutions[1].toMap(),
          equals(<String, String>{
            'pluginName': 'transitive_dependency_plugin',
            'dartClass': 'UrlLauncherPluginWindows',
            'platform': 'windows',
            'dartFileName': 'transitive_dependency_plugin.dart',
          }),
        );
      });

      testWithoutContext('selects inline implementation on mobile', () async {
        final Set<String> directDependencies = <String>{};

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'android': <String, dynamic>{'dartPluginClass': 'UrlLauncherAndroid'},
                'ios': <String, dynamic>{'dartPluginClass': 'UrlLauncherIos'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(2));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher',
            'dartClass': 'UrlLauncherAndroid',
            'platform': 'android',
            'dartFileName': 'url_launcher.dart',
          }),
        );
        expect(
          resolutions[1].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher',
            'dartClass': 'UrlLauncherIos',
            'platform': 'ios',
            'dartFileName': 'url_launcher.dart',
          }),
        );
      });

      // See https://github.com/flutter/flutter/issues/87862 for details.
      testWithoutContext('does not select inline implementation on desktop for '
          'missing min Flutter SDK constraint', () async {
        final Set<String> directDependencies = <String>{};

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherLinux'},
                'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherMacOS'},
                'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherWindows'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(0));
      });

      // See https://github.com/flutter/flutter/issues/87862 for details.
      testWithoutContext('does not select inline implementation on desktop for '
          'min Flutter SDK constraint < 2.11', () async {
        final Set<String> directDependencies = <String>{};

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherLinux'},
                'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherMacOS'},
                'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherWindows'},
              },
            }),
            VersionConstraint.parse('>=2.10.0'),
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(0));
      });

      testWithoutContext('selects inline implementation on desktop for '
          'min Flutter SDK requirement of at least 2.11', () async {
        final Set<String> directDependencies = <String>{};

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherLinux'},
                'macos': <String, dynamic>{'dartPluginClass': 'UrlLauncherMacOS'},
                'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherWindows'},
              },
            }),
            VersionConstraint.parse('>=2.11.0'),
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(3));
        expect(
          resolutions.map((PluginInterfaceResolution resolution) => resolution.toMap()),
          containsAll(<Map<String, String>>[
            <String, String>{
              'pluginName': 'url_launcher',
              'dartClass': 'UrlLauncherLinux',
              'platform': 'linux',
              'dartFileName': 'url_launcher.dart',
            },
            <String, String>{
              'pluginName': 'url_launcher',
              'dartClass': 'UrlLauncherMacOS',
              'platform': 'macos',
              'dartFileName': 'url_launcher.dart',
            },
            <String, String>{
              'pluginName': 'url_launcher',
              'dartClass': 'UrlLauncherWindows',
              'platform': 'windows',
              'dartFileName': 'url_launcher.dart',
            },
          ]),
        );
      });

      testWithoutContext('selects default implementation', () async {
        final Set<String> directDependencies = <String>{};

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          // Include three possible implementations, one before and one after
          // to ensure that the selection is working as intended, not just by
          // coincidence of order.
          Plugin.fromYaml(
            'another_url_launcher_linux',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UnofficialUrlLauncherPluginLinux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'url_launcher_linux',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'yet_another_url_launcher_linux',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UnofficialUrlLauncherPluginLinux2'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(1));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher_linux',
            'dartClass': 'UrlLauncherPluginLinux',
            'platform': 'linux',
            'dartFileName': 'url_launcher_linux.dart',
          }),
        );
      });

      testWithoutContext(
        'selects default implementation if interface is direct dependency',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};

          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            selectDartPluginsOnly: true,
          );
          expect(resolutions.length, equals(1));
          expect(
            resolutions[0].toMap(),
            equals(<String, String>{
              'pluginName': 'url_launcher_linux',
              'dartClass': 'UrlLauncherPluginLinux',
              'platform': 'linux',
              'dartFileName': 'url_launcher_linux.dart',
            }),
          );
        },
      );

      testWithoutContext('user-selected implementation overrides default implementation', () async {
        final Set<String> directDependencies = <String>{
          'user_selected_url_launcher_implementation',
          'url_launcher',
        };

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'url_launcher_linux',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'user_selected_url_launcher_implementation',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(1));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'user_selected_url_launcher_implementation',
            'dartClass': 'UrlLauncherPluginLinux',
            'platform': 'linux',
            'dartFileName': 'user_selected_url_launcher_implementation.dart',
          }),
        );
      });

      testWithoutContext('user-selected implementation overrides inline implementation', () async {
        final Set<String> directDependencies = <String>{
          'user_selected_url_launcher_implementation',
          'url_launcher',
        };

        final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(<Plugin>[
          Plugin.fromYaml(
            'url_launcher',
            '',
            YamlMap.wrap(<String, dynamic>{
              'platforms': <String, dynamic>{
                'android': <String, dynamic>{'dartPluginClass': 'UrlLauncherAndroid'},
                'ios': <String, dynamic>{'dartPluginClass': 'UrlLauncherIos'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
          Plugin.fromYaml(
            'user_selected_url_launcher_implementation',
            '',
            YamlMap.wrap(<String, dynamic>{
              'implements': 'url_launcher',
              'platforms': <String, dynamic>{
                'android': <String, dynamic>{'dartPluginClass': 'UrlLauncherAndroid'},
              },
            }),
            null,
            <String>[],
            fileSystem: fs,
            isDevDependency: false,
            appDependencies: directDependencies,
          ),
        ], selectDartPluginsOnly: true);
        expect(resolutions.length, equals(2));
        expect(
          resolutions[0].toMap(),
          equals(<String, String>{
            'pluginName': 'user_selected_url_launcher_implementation',
            'dartClass': 'UrlLauncherAndroid',
            'platform': 'android',
            'dartFileName': 'user_selected_url_launcher_implementation.dart',
          }),
        );
        expect(
          resolutions[1].toMap(),
          equals(<String, String>{
            'pluginName': 'url_launcher',
            'dartClass': 'UrlLauncherIos',
            'platform': 'ios',
            'dartFileName': 'url_launcher.dart',
          }),
        );
      });

      testUsingContext(
        'provides error when a plugin has a default implementation and implements another plugin',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          expect(() {
            resolvePlatformImplementation(<Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux_1'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux_1',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux_2'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux_2',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ], selectDartPluginsOnly: true);
          }, throwsToolExit(message: 'Please resolve the plugin pubspec errors'));

          expect(
            testLogger.errorText,
            'Plugin url_launcher_linux_1:linux provides an implementation for url_launcher '
            'and also references a default implementation for url_launcher_linux_2, which is currently not supported. '
            'Ask the maintainers of url_launcher_linux_1 to either remove the implementation via `implements: url_launcher` '
            'or avoid referencing a default implementation via `platforms: linux: default_package: url_launcher_linux_2`.'
            '\n\n',
          );
        },
      );

      testUsingContext(
        'provides error when a plugin has a default implementation and an inline implementation',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          expect(() {
            resolvePlatformImplementation(<Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{
                      'default_package': 'url_launcher_linux',
                      'dartPluginClass': 'UrlLauncherPluginLinux',
                    },
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ], selectDartPluginsOnly: true);
          }, throwsToolExit(message: 'Please resolve the plugin pubspec errors'));

          expect(
            testLogger.errorText,
            'Plugin url_launcher:linux which provides an inline implementation '
            'cannot also reference a default implementation for url_launcher_linux. '
            'Ask the maintainers of url_launcher to either remove the implementation via `platforms: linux: dartPluginClass` '
            'or avoid referencing a default implementation via `platforms: linux: default_package: url_launcher_linux`.'
            '\n\n',
          );
        },
      );

      testUsingContext(
        'provides warning when a plugin references a default plugin without implementation',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{},
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            selectDartPluginsOnly: true,
          );

          expect(resolutions.length, equals(0));
          expect(
            testLogger.warningText,
            'Package url_launcher:linux references url_launcher_linux:linux as the default plugin, '
            'but it does not provide an inline implementation.\n'
            'Ask the maintainers of url_launcher to either avoid referencing a default implementation via `platforms: linux: default_package: url_launcher_linux` '
            'or add an inline implementation to url_launcher_linux via `platforms: linux:` `pluginClass` or `dartPluginClass`.\n'
            '\n',
          );
        },
      );

      testUsingContext(
        'avoid warning when a plugin references a default plugin with a native implementation only',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'pluginClass': 'UrlLauncherLinux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            selectDartPluginsOnly: true,
          );

          expect(resolutions.length, equals(0));
          expect(testLogger.warningText, '');
        },
      );

      testUsingContext(
        'selects default Dart implementation without warning, while choosing plugin selection for nativeOrDart',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherLinux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            // Using nativeOrDart plugin selection.
            selectDartPluginsOnly: false,
          );
          expect(resolutions.length, equals(1));
          // Test avoiding trigger a warning for default plugins, while Dart and native plugins selection is enabled.
          expect(testLogger.warningText, '');
          expect(
            resolutions[0].toMap(),
            equals(<String, String>{
              'pluginName': 'url_launcher_linux',
              'dartClass': 'UrlLauncherLinux',
              'platform': 'linux',
              'dartFileName': 'url_launcher_linux.dart',
            }),
          );
        },
      );

      testUsingContext(
        'provides warning when a plugin references a default plugin which does not exist',
        () async {
          final Set<String> directDependencies = <String>{'url_launcher'};
          final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
            <Plugin>[
              Plugin.fromYaml(
                'url_launcher',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'default_package': 'url_launcher_linux'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ],
            selectDartPluginsOnly: true,
          );

          expect(resolutions.length, equals(0));
          expect(
            testLogger.warningText,
            'Package url_launcher:linux references url_launcher_linux:linux as the default plugin, '
            'but the package does not exist, or is not a plugin package.\n'
            'Ask the maintainers of url_launcher to either avoid referencing a default implementation via `platforms: linux: default_package: url_launcher_linux` '
            'or create a plugin named url_launcher_linux.\n'
            '\n',
          );
        },
      );

      testUsingContext('provides error when user selected multiple implementations', () async {
        final Set<String> directDependencies = <String>{
          'url_launcher_linux_1',
          'url_launcher_linux_2',
        };
        expect(() {
          resolvePlatformImplementation(<Plugin>[
            Plugin.fromYaml(
              'url_launcher_linux_1',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
            Plugin.fromYaml(
              'url_launcher_linux_2',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
          ], selectDartPluginsOnly: true);
        }, throwsToolExit(message: 'Please resolve the plugin implementation selection errors'));

        expect(
          testLogger.errorText,
          'Plugin url_launcher:linux has conflicting direct dependency implementations:\n'
          '  url_launcher_linux_1\n'
          '  url_launcher_linux_2\n'
          'To fix this issue, remove all but one of these dependencies from pubspec.yaml.\n'
          '\n',
        );
      });

      testUsingContext('provides all errors when user selected multiple implementations', () async {
        final Set<String> directDependencies = <String>{
          'url_launcher_linux_1',
          'url_launcher_linux_2',
          'url_launcher_windows_1',
          'url_launcher_windows_2',
        };
        expect(() {
          resolvePlatformImplementation(<Plugin>[
            Plugin.fromYaml(
              'url_launcher_linux_1',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
            Plugin.fromYaml(
              'url_launcher_linux_2',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
            Plugin.fromYaml(
              'url_launcher_windows_1',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginWindows1'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
            Plugin.fromYaml(
              'url_launcher_windows_2',
              '',
              YamlMap.wrap(<String, dynamic>{
                'implements': 'url_launcher',
                'platforms': <String, dynamic>{
                  'windows': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginWindows2'},
                },
              }),
              null,
              <String>[],
              fileSystem: fs,
              isDevDependency: false,
              appDependencies: directDependencies,
            ),
          ], selectDartPluginsOnly: true);
        }, throwsToolExit(message: 'Please resolve the plugin implementation selection errors'));

        expect(
          testLogger.errorText,
          'Plugin url_launcher:linux has conflicting direct dependency implementations:\n'
          '  url_launcher_linux_1\n'
          '  url_launcher_linux_2\n'
          'To fix this issue, remove all but one of these dependencies from pubspec.yaml.\n'
          '\n'
          'Plugin url_launcher:windows has conflicting direct dependency implementations:\n'
          '  url_launcher_windows_1\n'
          '  url_launcher_windows_2\n'
          'To fix this issue, remove all but one of these dependencies from pubspec.yaml.\n'
          '\n',
        );
      });

      testUsingContext(
        'provides error when user needs to select among multiple implementations',
        () async {
          final Set<String> directDependencies = <String>{};
          expect(() {
            resolvePlatformImplementation(<Plugin>[
              Plugin.fromYaml(
                'url_launcher_linux_1',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux1'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
              Plugin.fromYaml(
                'url_launcher_linux_2',
                '',
                YamlMap.wrap(<String, dynamic>{
                  'implements': 'url_launcher',
                  'platforms': <String, dynamic>{
                    'linux': <String, dynamic>{'dartPluginClass': 'UrlLauncherPluginLinux2'},
                  },
                }),
                null,
                <String>[],
                fileSystem: fs,
                isDevDependency: false,
                appDependencies: directDependencies,
              ),
            ], selectDartPluginsOnly: true);
          }, throwsToolExit(message: 'Please resolve the plugin implementation selection errors'));

          expect(
            testLogger.errorText,
            'Plugin url_launcher:linux has multiple possible implementations:\n'
            '  url_launcher_linux_1\n'
            '  url_launcher_linux_2\n'
            'To fix this issue, add one of these dependencies to pubspec.yaml.\n'
            '\n',
          );
        },
      );
    });

    group('generateMainDartWithPluginRegistrant', () {
      testUsingContext(
        'Generates new entrypoint',
        () async {
          flutterProject.isModule = true;

          createFakeDartPlugins(flutterProject, flutterManifest, fs, <String, String>{
            'url_launcher_android': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        android:
          dartPluginClass: AndroidPlugin
''',
            'url_launcher_ios': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        ios:
          dartPluginClass: IosPlugin
''',
            'url_launcher_macos': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        macos:
          dartPluginClass: MacOSPlugin
''',
            'url_launcher_linux': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        linux:
          dartPluginClass: LinuxPlugin
''',
            'url_launcher_windows': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        windows:
          dartPluginClass: WindowsPlugin
''',
            'awesome_macos': '''
  flutter:
    plugin:
      implements: awesome
      platforms:
        macos:
          dartPluginClass: AwesomeMacOS
''',
          });

          final Directory libDir = flutterProject.directory.childDirectory('lib');
          libDir.createSync(recursive: true);

          final File mainFile = libDir.childFile('main.dart');
          mainFile.writeAsStringSync('''
// @dart = 2.8
void main() {
}
''');
          final PackageConfig packageConfig = await loadPackageConfigWithLogging(
            flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json'),
            logger: globals.logger,
            throwOnError: false,
          );
          await generateMainDartWithPluginRegistrant(
            flutterProject,
            packageConfig,
            'package:app/main.dart',
            mainFile,
          );
          expect(
            flutterProject.dartPluginRegistrant.readAsStringSync(),
            '//\n'
            '// Generated file. Do not edit.\n'
            '// This file is generated from template in file `flutter_tools/lib/src/flutter_plugins.dart`.\n'
            '//\n'
            '\n'
            '// @dart = 2.8\n'
            '\n'
            "import 'dart:io'; // flutter_ignore: dart_io_import.\n"
            "import 'package:url_launcher_android/url_launcher_android.dart';\n"
            "import 'package:url_launcher_ios/url_launcher_ios.dart';\n"
            "import 'package:url_launcher_linux/url_launcher_linux.dart';\n"
            "import 'package:awesome_macos/awesome_macos.dart';\n"
            "import 'package:url_launcher_macos/url_launcher_macos.dart';\n"
            "import 'package:url_launcher_windows/url_launcher_windows.dart';\n"
            '\n'
            "@pragma('vm:entry-point')\n"
            'class _PluginRegistrant {\n'
            '\n'
            "  @pragma('vm:entry-point')\n"
            '  static void register() {\n'
            '    if (Platform.isAndroid) {\n'
            '      try {\n'
            '        AndroidPlugin.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`url_launcher_android` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isIOS) {\n'
            '      try {\n'
            '        IosPlugin.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`url_launcher_ios` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isLinux) {\n'
            '      try {\n'
            '        LinuxPlugin.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`url_launcher_linux` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isMacOS) {\n'
            '      try {\n'
            '        AwesomeMacOS.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`awesome_macos` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '      try {\n'
            '        MacOSPlugin.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`url_launcher_macos` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isWindows) {\n'
            '      try {\n'
            '        WindowsPlugin.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`url_launcher_windows` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    }\n'
            '  }\n'
            '}\n',
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Plugin without platform support throws tool exit',
        () async {
          flutterProject.isModule = false;

          createFakeDartPlugins(flutterProject, flutterManifest, fs, <String, String>{
            'url_launcher_macos': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        macos:
          invalid:
''',
          });

          final Directory libDir = flutterProject.directory.childDirectory('lib');
          libDir.createSync(recursive: true);

          final File mainFile = libDir.childFile('main.dart')..writeAsStringSync('');
          final PackageConfig packageConfig = await loadPackageConfigWithLogging(
            flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json'),
            logger: globals.logger,
            throwOnError: false,
          );
          await expectLater(
            generateMainDartWithPluginRegistrant(
              flutterProject,
              packageConfig,
              'package:app/main.dart',
              mainFile,
            ),
            throwsToolExit(
              message:
                  'Invalid plugin specification url_launcher_macos.\n'
                  'Invalid "macos" plugin specification.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Plugin with platform support without dart plugin class throws tool exit',
        () async {
          flutterProject.isModule = false;

          createFakeDartPlugins(flutterProject, flutterManifest, fs, <String, String>{
            'url_launcher_macos': '''
  flutter:
    plugin:
      implements: url_launcher
''',
          });

          final Directory libDir = flutterProject.directory.childDirectory('lib');
          libDir.createSync(recursive: true);

          final File mainFile = libDir.childFile('main.dart')..writeAsStringSync('');
          final PackageConfig packageConfig = await loadPackageConfigWithLogging(
            flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json'),
            logger: globals.logger,
            throwOnError: false,
          );
          await expectLater(
            generateMainDartWithPluginRegistrant(
              flutterProject,
              packageConfig,
              'package:app/main.dart',
              mainFile,
            ),
            throwsToolExit(
              message:
                  'Invalid plugin specification url_launcher_macos.\n'
                  'Cannot find the `flutter.plugin.platforms` key in the `pubspec.yaml` file. '
                  'An instruction to format the `pubspec.yaml` can be found here: '
                  'https://flutter.dev/to/pubspec-plugin-platforms',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Does not create new entrypoint if there are no platform resolutions',
        () async {
          flutterProject.isModule = false;

          final Directory libDir = flutterProject.directory.childDirectory('lib');
          libDir.createSync(recursive: true);

          final File mainFile = libDir.childFile('main.dart')..writeAsStringSync('');
          final PackageConfig packageConfig = await loadPackageConfigWithLogging(
            flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json'),
            logger: globals.logger,
            throwOnError: false,
          );
          await generateMainDartWithPluginRegistrant(
            flutterProject,
            packageConfig,
            'package:app/main.dart',
            mainFile,
          );
          expect(flutterProject.dartPluginRegistrant.existsSync(), isFalse);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Deletes new entrypoint if there are no platform resolutions',
        () async {
          flutterProject.isModule = false;

          createFakeDartPlugins(flutterProject, flutterManifest, fs, <String, String>{
            'url_launcher_macos': '''
  flutter:
    plugin:
      implements: url_launcher
      platforms:
        macos:
          dartPluginClass: MacOSPlugin
''',
          });

          final Directory libDir = flutterProject.directory.childDirectory('lib');
          libDir.createSync(recursive: true);

          final File mainFile = libDir.childFile('main.dart')..writeAsStringSync('');
          final PackageConfig packageConfig = await loadPackageConfigWithLogging(
            flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json'),
            logger: globals.logger,
            throwOnError: false,
          );
          await generateMainDartWithPluginRegistrant(
            flutterProject,
            packageConfig,
            'package:app/main.dart',
            mainFile,
          );
          expect(flutterProject.dartPluginRegistrant.existsSync(), isTrue);

          // No plugins.
          createFakeDartPlugins(flutterProject, flutterManifest, fs, <String, String>{});

          await generateMainDartWithPluginRegistrant(
            flutterProject,
            packageConfig,
            'package:app/main.dart',
            mainFile,
          );
          expect(flutterProject.dartPluginRegistrant.existsSync(), isFalse);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );
    });
  });
}

void addToPackageConfig(FlutterProject project, String name, Directory packageDir) {
  final File packageConfigFile = project.directory
      .childDirectory('.dart_tool')
      .childFile('package_config.json');

  final Map<String, Object?> packageConfig =
      jsonDecode(packageConfigFile.readAsStringSync()) as Map<String, Object?>;

  (packageConfig['packages']! as List<Object?>).add(<String, Object?>{
    'name': name,
    'rootUri': packageDir.uri.toString(),
    'packageUri': 'lib/',
  });

  packageConfigFile.writeAsStringSync(jsonEncode(packageConfig));
}

void createFakeDartPlugins(
  FakeFlutterProject flutterProject,
  FakeFlutterManifest flutterManifest,
  FileSystem fs,
  Map<String, String> plugins,
) {
  final Directory fakePubCache = fs.systemTempDirectory.childDirectory('cache');

  flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..deleteSync(recursive: true)
    ..createSync(recursive: true)
    ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');

  for (final MapEntry<String, String> entry in plugins.entries) {
    final String name = fs.path.basename(entry.key);
    final Directory pluginDirectory = fakePubCache.childDirectory(name);
    addToPackageConfig(flutterProject, name, pluginDirectory);
    pluginDirectory.childFile('pubspec.yaml')
      ..createSync(recursive: true)
      ..writeAsStringSync(entry.value);
  }
  flutterManifest.dependencies = plugins.keys.toSet();
}

class FakeFlutterManifest extends Fake implements FlutterManifest {
  @override
  Set<String> dependencies = <String>{};
}

class FakeFlutterProject extends Fake implements FlutterProject {
  @override
  bool isModule = false;

  @override
  late FlutterManifest manifest;

  @override
  late Directory directory;

  @override
  late File flutterPluginsFile;

  @override
  late File flutterPluginsDependenciesFile;

  @override
  late File dartPluginRegistrant;

  @override
  late IosProject ios;

  @override
  late AndroidProject android;

  @override
  late WebProject web;

  @override
  late MacOSProject macos;

  @override
  late LinuxProject linux;

  @override
  late WindowsProject windows;
}
