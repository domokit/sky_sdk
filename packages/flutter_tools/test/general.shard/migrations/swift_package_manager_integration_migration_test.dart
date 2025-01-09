// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/migrations/swift_package_manager_integration_migration.dart';

import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

const List<SupportedPlatform> supportedPlatforms = <SupportedPlatform>[
  SupportedPlatform.ios,
  SupportedPlatform.macos,
];

void main() {
  final TestFeatureFlags swiftPackageManagerFullyEnabledFlags = TestFeatureFlags(
    isSwiftPackageManagerEnabled: true,
  );

  group('Flutter Package Migration', () {
    testWithoutContext('skips if swift package manager is off', () async {
      final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
      final BufferLogger testLogger = BufferLogger.test();

      final SwiftPackageManagerIntegrationMigration projectMigration =
          SwiftPackageManagerIntegrationMigration(
            FakeXcodeProject(
              platform: SupportedPlatform.ios.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            ),
            SupportedPlatform.ios,
            BuildInfo.debug,
            xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
            logger: testLogger,
            fileSystem: memoryFileSystem,
            plistParser: FakePlistParser(),
            features: TestFeatureFlags(),
          );
      await projectMigration.migrate();
      expect(
        testLogger.traceText,
        contains('Skipping the migration that adds Swift Package Manager integration...'),
      );
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext("skips if there's no generated swift package", () async {
      final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
      final BufferLogger testLogger = BufferLogger.test();

      final SwiftPackageManagerIntegrationMigration projectMigration =
          SwiftPackageManagerIntegrationMigration(
            FakeXcodeProject(
              platform: SupportedPlatform.ios.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            ),
            SupportedPlatform.ios,
            BuildInfo.debug,
            xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
            logger: testLogger,
            fileSystem: memoryFileSystem,
            plistParser: FakePlistParser(),
            features: swiftPackageManagerFullyEnabledFlags,
          );
      await projectMigration.migrate();
      expect(
        testLogger.traceText,
        contains('Skipping the migration that adds Swift Package Manager integration...'),
      );
      expect(testLogger.traceText, contains('The tool did not generate a Swift package.'));
      expect(testLogger.statusText, isEmpty);
    });

    testWithoutContext('fails if Xcode project not found', () async {
      final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
      final BufferLogger testLogger = BufferLogger.test();
      final FakeXcodeProject project = FakeXcodeProject(
        platform: SupportedPlatform.ios.name,
        fileSystem: memoryFileSystem,
        logger: testLogger,
      );

      project.flutterPluginSwiftPackageManifest.createSync(recursive: true);

      final SwiftPackageManagerIntegrationMigration projectMigration =
          SwiftPackageManagerIntegrationMigration(
            project,
            SupportedPlatform.ios,
            BuildInfo.debug,
            xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
            logger: testLogger,
            fileSystem: memoryFileSystem,
            plistParser: FakePlistParser(),
            features: swiftPackageManagerFullyEnabledFlags,
          );
      await expectLater(
        () => projectMigration.migrate(),
        throwsToolExit(message: 'Xcode project not found.'),
      );
      expect(testLogger.traceText, isEmpty);
      expect(testLogger.statusText, isEmpty);
    });

    group('get scheme file', () {
      testWithoutContext('fails if Xcode project info not found', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios);
        project._projectInfo = null;

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(
          () => projectMigration.migrate(),
          throwsToolExit(message: 'Unable to get Xcode project info.'),
        );
        expect(testLogger.traceText, isEmpty);
        expect(testLogger.statusText, isEmpty);
      });

      testWithoutContext('fails if Xcode workspace not found', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios);
        project.xcodeWorkspace = null;

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(
          () => projectMigration.migrate(),
          throwsToolExit(message: 'Xcode workspace not found.'),
        );
        expect(testLogger.traceText, isEmpty);
        expect(testLogger.statusText, isEmpty);
      });

      testWithoutContext('fails if scheme not found', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios);
        project._projectInfo = XcodeProjectInfo(<String>[], <String>[], <String>[], testLogger);

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(
          () => projectMigration.migrate(),
          throwsToolExit(
            message: 'You must specify a --flavor option to select one of the available schemes.',
          ),
        );
        expect(testLogger.traceText, isEmpty);
        expect(testLogger.statusText, isEmpty);
      });

      testWithoutContext('fails if scheme file not found', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios, createSchemeFile: false);

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(
          () => projectMigration.migrate(),
          throwsToolExit(message: 'Unable to get scheme file for Runner.'),
        );
        expect(testLogger.traceText, isEmpty);
        expect(testLogger.statusText, isEmpty);
      });
    });

    testWithoutContext('does not migrate if already migrated', () async {
      final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
      final BufferLogger testLogger = BufferLogger.test();
      final FakeXcodeProject project = FakeXcodeProject(
        platform: SupportedPlatform.ios.name,
        fileSystem: memoryFileSystem,
        logger: testLogger,
      );
      _createProjectFiles(project, SupportedPlatform.ios);
      project.xcodeProjectSchemeFile().writeAsStringSync(
        _validBuildActions(SupportedPlatform.ios, hasFrameworkScript: true),
      );
      project.xcodeProjectInfoFile.writeAsStringSync(
        _projectSettings(_allSectionsMigrated(SupportedPlatform.ios)),
      );

      final SwiftPackageManagerIntegrationMigration projectMigration =
          SwiftPackageManagerIntegrationMigration(
            project,
            SupportedPlatform.ios,
            BuildInfo.debug,
            xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
            logger: testLogger,
            fileSystem: memoryFileSystem,
            plistParser: FakePlistParser(),
            features: swiftPackageManagerFullyEnabledFlags,
          );
      await projectMigration.migrate();
      expect(testLogger.traceText, isEmpty);
      expect(testLogger.statusText, isEmpty);
      expect(testLogger.warningText, isEmpty);
      expect(testLogger.errorText, isEmpty);
    });

    group('migrate scheme', () {
      testWithoutContext('skipped if already updated', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios);
        project.xcodeProjectSchemeFile().writeAsStringSync(
          _validBuildActions(SupportedPlatform.ios, hasFrameworkScript: true),
        );

        project.xcodeProjectInfoFile.writeAsStringSync('');

        final List<String> settingsAsJsonBeforeMigration = <String>[
          ..._allSectionsMigratedAsJson(SupportedPlatform.ios),
        ];
        settingsAsJsonBeforeMigration.removeAt(_buildFileSectionIndex);

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(json: _plutilOutput(settingsAsJsonBeforeMigration)),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(() => projectMigration.migrate(), throwsToolExit());
        expect(testLogger.traceText, contains('Runner.xcscheme already migrated. Skipping...'));
      });

      for (final SupportedPlatform platform in supportedPlatforms) {
        group('for ${platform.name}', () {
          testWithoutContext(
            'fails if scheme is missing BlueprintIdentifier for Runner native target',
            () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);
              project.xcodeProjectSchemeFile().writeAsStringSync('');

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Failed to parse Runner.xcscheme: Could not find BuildableReference',
                ),
              );
            },
          );

          testWithoutContext(
            'fails if BuildableName does not follow BlueprintIdentifier in scheme',
            () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);
              project.xcodeProjectSchemeFile().writeAsStringSync('''
         <BuildableReference
            BuildableIdentifier = "primary"
            BlueprintIdentifier = "${_runnerNativeTargetIdentifier(platform)}"
            BlueprintName = "Runner"
            ReferencedContainer = "container:Runner.xcodeproj">
         </BuildableReference>
''');

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );

              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Failed to parse Runner.xcscheme: Could not find BuildableName',
                ),
              );
            },
          );

          testWithoutContext(
            'fails if BlueprintName does not follow BuildableName in scheme',
            () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);
              project.xcodeProjectSchemeFile().writeAsStringSync('''
         <BuildableReference
            BuildableIdentifier = "primary"
            BlueprintIdentifier = "${_runnerNativeTargetIdentifier(platform)}"
            BuildableName = "Runner.app"
            ReferencedContainer = "container:Runner.xcodeproj">
         </BuildableReference>
''');

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );

              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Failed to parse Runner.xcscheme: Could not find BlueprintName',
                ),
              );
            },
          );

          testWithoutContext(
            'fails if ReferencedContainer does not follow BlueprintName in scheme',
            () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);
              project.xcodeProjectSchemeFile().writeAsStringSync('''
         <BuildableReference
            BuildableIdentifier = "primary"
            BlueprintIdentifier = "${_runnerNativeTargetIdentifier(platform)}"
            BuildableName = "Runner.app"
            BlueprintName = "Runner">
         </BuildableReference>
''');

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );

              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Failed to parse Runner.xcscheme: Could not find ReferencedContainer',
                ),
              );
            },
          );

          testWithoutContext('fails if cannot find BuildActionEntries in scheme', () async {
            final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
            final BufferLogger testLogger = BufferLogger.test();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            );
            _createProjectFiles(project, platform);
            project.xcodeProjectSchemeFile().writeAsStringSync(_validBuildableReference(platform));

            final SwiftPackageManagerIntegrationMigration projectMigration =
                SwiftPackageManagerIntegrationMigration(
                  project,
                  platform,
                  BuildInfo.debug,
                  xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                  logger: testLogger,
                  fileSystem: memoryFileSystem,
                  plistParser: FakePlistParser(),
                  features: swiftPackageManagerFullyEnabledFlags,
                );

            await expectLater(
              () => projectMigration.migrate(),
              throwsToolExit(
                message: 'Failed to parse Runner.xcscheme: Could not find BuildActionEntries',
              ),
            );
          });

          testWithoutContext('fails if updated scheme is not valid xml', () async {
            final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
            final BufferLogger testLogger = BufferLogger.test();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            );
            _createProjectFiles(project, platform);
            project.xcodeProjectSchemeFile().writeAsStringSync(
              '${_validBuildActions(platform)} <an opening without a close>',
            );
            final SwiftPackageManagerIntegrationMigration projectMigration =
                SwiftPackageManagerIntegrationMigration(
                  project,
                  platform,
                  BuildInfo.debug,
                  xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                  logger: testLogger,
                  fileSystem: memoryFileSystem,
                  plistParser: FakePlistParser(),
                  features: swiftPackageManagerFullyEnabledFlags,
                );

            await expectLater(
              () => projectMigration.migrate(),
              throwsToolExit(message: 'Failed to parse Runner.xcscheme: Invalid xml:'),
            );
          });

          testWithoutContext('successfully updates scheme with preexisting PreActions', () async {
            final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
            final BufferLogger testLogger = BufferLogger.test();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            );
            _createProjectFiles(project, platform);
            project.xcodeProjectSchemeFile().writeAsStringSync(
              _validBuildActions(platform, hasPreActions: true),
            );
            final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
              _plutilOutput(_allSectionsMigratedAsJson(platform)),
              _plutilOutput(_allSectionsMigratedAsJson(platform)),
            ]);
            final SwiftPackageManagerIntegrationMigration projectMigration =
                SwiftPackageManagerIntegrationMigration(
                  project,
                  platform,
                  BuildInfo.debug,
                  xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                  logger: testLogger,
                  fileSystem: memoryFileSystem,
                  plistParser: plistParser,
                  features: swiftPackageManagerFullyEnabledFlags,
                );

            await projectMigration.migrate();
            expect(
              project.xcodeProjectSchemeFile().readAsStringSync(),
              _validBuildActions(platform, hasFrameworkScript: true),
            );
          });

          testWithoutContext(
            'successfully updates scheme with no preexisting PreActions',
            () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);
              project.xcodeProjectSchemeFile().writeAsStringSync(_validBuildActions(platform));

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);
              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );

              await projectMigration.migrate();
              expect(
                project.xcodeProjectSchemeFile().readAsStringSync(),
                _validBuildActions(platform, hasFrameworkScript: true),
              );
            },
          );
        });
      }
    });

    group('migrate pbxproj', () {
      testWithoutContext('skipped if already updated', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        final FakeXcodeProject project = FakeXcodeProject(
          platform: SupportedPlatform.ios.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, SupportedPlatform.ios);

        project.xcodeProjectInfoFile.writeAsStringSync(
          _projectSettings(_allSectionsMigrated(SupportedPlatform.ios)),
        );

        final List<String> settingsAsJsonBeforeMigration = <String>[
          ..._allSectionsMigratedAsJson(SupportedPlatform.ios),
        ];
        settingsAsJsonBeforeMigration.removeAt(_buildFileSectionIndex);

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              SupportedPlatform.ios,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: FakePlistParser(json: _plutilOutput(settingsAsJsonBeforeMigration)),
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await projectMigration.migrate();
        expect(testLogger.traceText, contains('project.pbxproj already migrated. Skipping...'));
      });

      group('fails if parsing project.pbxproj', () {
        testWithoutContext('fails plutil command', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          await expectLater(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'Failed to parse project settings.'),
          );
        });

        testWithoutContext('returns unexpected JSON', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(json: '[]'),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          await expectLater(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'project.pbxproj returned unexpected JSON response'),
          );
        });

        testWithoutContext('returns non-JSON', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(json: 'this is not json'),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          await expectLater(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'project.pbxproj returned non-JSON response'),
          );
        });
      });

      group('fails if duplicate id', () {
        testWithoutContext('for PBXBuildFile', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);
          project.xcodeProjectInfoFile.writeAsStringSync('78A318202AECB46A00862997');

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          expect(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'Duplicate id found for PBXBuildFile'),
          );
        });

        testWithoutContext('for XCSwiftPackageProductDependency', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);
          project.xcodeProjectInfoFile.writeAsStringSync('78A3181F2AECB46A00862997');

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          expect(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'Duplicate id found for XCSwiftPackageProductDependency'),
          );
        });

        testWithoutContext('for XCLocalSwiftPackageReference', () async {
          final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
          final BufferLogger testLogger = BufferLogger.test();
          final FakeXcodeProject project = FakeXcodeProject(
            platform: SupportedPlatform.ios.name,
            fileSystem: memoryFileSystem,
            logger: testLogger,
          );
          _createProjectFiles(project, SupportedPlatform.ios);
          project.xcodeProjectInfoFile.writeAsStringSync('781AD8BC2B33823900A9FFBB');

          final SwiftPackageManagerIntegrationMigration projectMigration =
              SwiftPackageManagerIntegrationMigration(
                project,
                SupportedPlatform.ios,
                BuildInfo.debug,
                xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                logger: testLogger,
                fileSystem: memoryFileSystem,
                plistParser: FakePlistParser(),
                features: swiftPackageManagerFullyEnabledFlags,
              );
          expect(
            () => projectMigration.migrate(),
            throwsToolExit(message: 'Duplicate id found for XCLocalSwiftPackageReference'),
          );
        });
      });

      for (final SupportedPlatform platform in supportedPlatforms) {
        group('for ${platform.name}', () {
          group('migrate PBXBuildFile', () {
            testWithoutContext('fails if missing Begin PBXBuildFile section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(json: _plutilOutput(<String>[])),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              expect(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find beginning of PBXBuildFile section'),
              );
            });

            testWithoutContext('fails if missing End PBXBuildFile section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_buildFileSectionIndex] = '''
/* Begin PBXBuildFile section */
''';
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration[_buildFileSectionIndex] =
                  unmigratedBuildFileSectionAsJson;

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(json: _plutilOutput(<String>[])),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              expect(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find end of PBXBuildFile section'),
              );
            });

            testWithoutContext('fails if End before Begin for PBXBuildFile section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_buildFileSectionIndex] = '''
/* End PBXBuildFile section */
/* Begin PBXBuildFile section */
''';
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration[_buildFileSectionIndex] =
                  unmigratedBuildFileSectionAsJson;

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(json: _plutilOutput(<String>[])),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              expect(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Found the end of PBXBuildFile section before the beginning.',
                ),
              );
            });

            testWithoutContext('successfully added', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_buildFileSectionIndex] = unmigratedBuildFileSection;
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration[_buildFileSectionIndex] =
                  unmigratedBuildFileSectionAsJson;

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains('PBXBuildFile already migrated. Skipping...'),
                isFalse,
              );
              settingsBeforeMigration[_buildFileSectionIndex] = migratedBuildFileSection;
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(settingsBeforeMigration),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });
          });

          group('migrate PBXFrameworksBuildPhase', () {
            testWithoutContext('fails if missing PBXFrameworksBuildPhase section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration.removeAt(_frameworksBuildPhaseSectionIndex);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_frameworksBuildPhaseSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Unable to find beginning of PBXFrameworksBuildPhase section',
                ),
              );
            });

            testWithoutContext(
              'fails if missing Runner target subsection following PBXFrameworksBuildPhase begin header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_frameworksBuildPhaseSectionIndex] = '''
/* Begin PBXFrameworksBuildPhase section */
/* End PBXFrameworksBuildPhase section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsMigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration.removeAt(_frameworksBuildPhaseSectionIndex);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(
                    message: 'Unable to find PBXFrameworksBuildPhase for Runner target',
                  ),
                );
              },
            );

            testWithoutContext(
              'fails if missing Runner target subsection before PBXFrameworksBuildPhase end header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_frameworksBuildPhaseSectionIndex] = '''
/* Begin PBXFrameworksBuildPhase section */
/* End PBXFrameworksBuildPhase section */
/* Begin NonExistant section */
    ${_runnerFrameworksBuildPhaseIdentifier(platform)} /* Frameworks */ = {
    };
/* End NonExistant section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsMigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration.removeAt(_frameworksBuildPhaseSectionIndex);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(
                    message: 'Unable to find PBXFrameworksBuildPhase for Runner target',
                  ),
                );
              },
            );

            testWithoutContext('fails if missing Runner target in parsed settings', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_frameworksBuildPhaseSectionIndex] =
                  unmigratedFrameworksBuildPhaseSection(platform);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_frameworksBuildPhaseSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(
                  message: 'Unable to find parsed PBXFrameworksBuildPhase for Runner target',
                ),
              );
            });

            testWithoutContext('successfully added when files field is missing', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_frameworksBuildPhaseSectionIndex] =
                  unmigratedFrameworksBuildPhaseSection(platform, missingFiles: true);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration[_frameworksBuildPhaseSectionIndex] =
                  unmigratedFrameworksBuildPhaseSectionAsJson(platform, missingFiles: true);
              final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
              expectedSettings[_frameworksBuildPhaseSectionIndex] =
                  migratedFrameworksBuildPhaseSection(platform, missingFiles: true);

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'PBXFrameworksBuildPhase already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(expectedSettings),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when files field is empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'PBXFrameworksBuildPhase already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(_allSectionsMigrated(platform)),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when files field is not empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_frameworksBuildPhaseSectionIndex] =
                  unmigratedFrameworksBuildPhaseSection(platform, withCocoapods: true);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration[_frameworksBuildPhaseSectionIndex] =
                  unmigratedFrameworksBuildPhaseSectionAsJson(platform, withCocoapods: true);
              final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
              expectedSettings[_frameworksBuildPhaseSectionIndex] =
                  migratedFrameworksBuildPhaseSection(platform, withCocoapods: true);

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'PBXFrameworksBuildPhase already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(expectedSettings),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });
          });

          group('migrate PBXNativeTarget', () {
            testWithoutContext('fails if missing PBXNativeTarget section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration.removeAt(_nativeTargetSectionIndex);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_nativeTargetSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find beginning of PBXNativeTarget section'),
              );
            });

            testWithoutContext('fails if missing Runner target in parsed settings', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_nativeTargetSectionIndex] = unmigratedNativeTargetSection(
                platform,
              );
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_nativeTargetSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find parsed PBXNativeTarget for Runner target'),
              );
            });

            testWithoutContext(
              'fails if missing Runner target subsection following PBXNativeTarget begin header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_nativeTargetSectionIndex] = '''
/* Begin PBXNativeTarget section */
/* End PBXNativeTarget section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(message: 'Unable to find PBXNativeTarget for Runner target'),
                );
              },
            );

            testWithoutContext(
              'fails if missing Runner target subsection before PBXNativeTarget end header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_nativeTargetSectionIndex] = '''
/* Begin PBXNativeTarget section */
/* End PBXNativeTarget section */
/* Begin NonExistant section */
    ${_runnerNativeTargetIdentifier(platform)} /* Runner */ = {
    };
/* End NonExistant section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(message: 'Unable to find PBXNativeTarget for Runner target'),
                );
              },
            );

            testWithoutContext(
              'successfully added when packageProductDependencies field is missing',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_nativeTargetSectionIndex] = unmigratedNativeTargetSection(
                  platform,
                  missingPackageProductDependencies: true,
                );
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration[_nativeTargetSectionIndex] =
                    unmigratedNativeTargetSectionAsJson(
                      platform,
                      missingPackageProductDependencies: true,
                    );
                final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
                expectedSettings[_nativeTargetSectionIndex] = migratedNativeTargetSection(
                  platform,
                  missingPackageProductDependencies: true,
                );

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXNativeTarget already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(expectedSettings),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );

            testWithoutContext(
              'successfully added when packageProductDependencies field is empty',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_nativeTargetSectionIndex] = unmigratedNativeTargetSection(
                  platform,
                );
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXNativeTarget already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(_allSectionsMigrated(platform)),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );

            testWithoutContext(
              'successfully added when packageProductDependencies field is not empty',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_nativeTargetSectionIndex] = unmigratedNativeTargetSection(
                  platform,
                  withOtherDependency: true,
                );
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
                expectedSettings[_nativeTargetSectionIndex] = migratedNativeTargetSection(
                  platform,
                  withOtherDependency: true,
                );

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXNativeTarget already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(expectedSettings),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );
          });

          group('migrate PBXProject', () {
            testWithoutContext('fails if missing PBXProject section', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration.removeAt(_projectSectionIndex);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_projectSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find beginning of PBXProject section'),
              );
            });

            testWithoutContext(
              'fails if missing Runner project subsection following PBXProject begin header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_projectSectionIndex] = '''
/* Begin PBXProject section */
/* End PBXProject section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );

                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration.removeAt(_projectSectionIndex);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(message: 'Unable to find PBXProject for Runner'),
                );
              },
            );

            testWithoutContext(
              'fails if missing Runner project subsection before PBXProject end header',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_projectSectionIndex] = '''
/* Begin PBXProject section */
/* End PBXProject section */
/* Begin NonExistant section */
    ${_projectIdentifier(platform)} /* Project object */ = {
    };
/* End NonExistant section */
''';
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );

                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration.removeAt(_projectSectionIndex);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: FakePlistParser(
                        json: _plutilOutput(settingsAsJsonBeforeMigration),
                      ),
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await expectLater(
                  () => projectMigration.migrate(),
                  throwsToolExit(message: 'Unable to find PBXProject for Runner'),
                );
              },
            );

            testWithoutContext('fails if missing Runner project in parsed settings', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_projectSectionIndex] = unmigratedProjectSection(platform);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_projectSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find parsed PBXProject for Runner'),
              );
            });

            testWithoutContext(
              'successfully added when packageReferences field is missing',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_projectSectionIndex] = unmigratedProjectSection(
                  platform,
                  missingPackageReferences: true,
                );
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );

                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                settingsAsJsonBeforeMigration[_projectSectionIndex] =
                    unmigratedProjectSectionAsJson(platform, missingPackageReferences: true);

                final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
                expectedSettings[_projectSectionIndex] = migratedProjectSection(
                  platform,
                  missingPackageReferences: true,
                );

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXProject already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(expectedSettings),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );

            testWithoutContext(
              'successfully added when packageReferences field is empty',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_projectSectionIndex] = unmigratedProjectSection(platform);
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXProject already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(_allSectionsMigrated(platform)),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );

            testWithoutContext(
              'successfully added when packageReferences field is not empty',
              () async {
                final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final FakeXcodeProject project = FakeXcodeProject(
                  platform: platform.name,
                  fileSystem: memoryFileSystem,
                  logger: testLogger,
                );
                _createProjectFiles(project, platform);

                final List<String> settingsBeforeMigration = <String>[
                  ..._allSectionsUnmigrated(platform),
                ];
                settingsBeforeMigration[_projectSectionIndex] = unmigratedProjectSection(
                  platform,
                  withOtherReference: true,
                );
                project.xcodeProjectInfoFile.writeAsStringSync(
                  _projectSettings(settingsBeforeMigration),
                );
                final List<String> settingsAsJsonBeforeMigration = <String>[
                  ..._allSectionsUnmigratedAsJson(platform),
                ];
                final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
                expectedSettings[_projectSectionIndex] = migratedProjectSection(
                  platform,
                  withOtherReference: true,
                );

                final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                  _plutilOutput(settingsAsJsonBeforeMigration),
                  _plutilOutput(_allSectionsMigratedAsJson(platform)),
                ]);

                final SwiftPackageManagerIntegrationMigration projectMigration =
                    SwiftPackageManagerIntegrationMigration(
                      project,
                      platform,
                      BuildInfo.debug,
                      xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                      logger: testLogger,
                      fileSystem: memoryFileSystem,
                      plistParser: plistParser,
                      features: swiftPackageManagerFullyEnabledFlags,
                    );
                await projectMigration.migrate();
                expect(testLogger.errorText, isEmpty);
                expect(
                  testLogger.traceText.contains('PBXProject already migrated. Skipping...'),
                  isFalse,
                );
                expect(
                  project.xcodeProjectInfoFile.readAsStringSync(),
                  _projectSettings(expectedSettings),
                );
                expect(plistParser.hasRemainingExpectations, isFalse);
              },
            );
          });

          group('migrate XCLocalSwiftPackageReference', () {
            testWithoutContext('fails if unable to find section to append it after', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_localSwiftPackageReferenceSectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find any sections'),
              );
            });

            testWithoutContext('successfully added when section is missing', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration.removeAt(_localSwiftPackageReferenceSectionIndex);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
              expectedSettings.removeAt(_localSwiftPackageReferenceSectionIndex);
              expectedSettings.add(migratedLocalSwiftPackageReferenceSection());

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCLocalSwiftPackageReference already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(expectedSettings),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when section is empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_localSwiftPackageReferenceSectionIndex] =
                  unmigratedLocalSwiftPackageReferenceSection();
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCLocalSwiftPackageReference already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(_allSectionsMigrated(platform)),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when section is not empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_localSwiftPackageReferenceSectionIndex] =
                  unmigratedLocalSwiftPackageReferenceSection(withOtherReference: true);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
              expectedSettings[_localSwiftPackageReferenceSectionIndex] =
                  migratedLocalSwiftPackageReferenceSection(withOtherReference: true);

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCLocalSwiftPackageReference already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(expectedSettings),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });
          });

          group('migrate XCSwiftPackageProductDependency', () {
            testWithoutContext('fails if unable to find section to append it after', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsMigratedAsJson(platform),
              ];
              settingsAsJsonBeforeMigration.removeAt(_swiftPackageProductDependencySectionIndex);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: FakePlistParser(
                      json: _plutilOutput(settingsAsJsonBeforeMigration),
                    ),
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await expectLater(
                () => projectMigration.migrate(),
                throwsToolExit(message: 'Unable to find any sections'),
              );
            });

            testWithoutContext('successfully added when section is missing', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration.removeAt(_swiftPackageProductDependencySectionIndex);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCSwiftPackageProductDependency already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(_allSectionsMigrated(platform)),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when section is empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_swiftPackageProductDependencySectionIndex] =
                  unmigratedSwiftPackageProductDependencySection();
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCSwiftPackageProductDependency already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(_allSectionsMigrated(platform)),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });

            testWithoutContext('successfully added when section is not empty', () async {
              final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final FakeXcodeProject project = FakeXcodeProject(
                platform: platform.name,
                fileSystem: memoryFileSystem,
                logger: testLogger,
              );
              _createProjectFiles(project, platform);

              final List<String> settingsBeforeMigration = <String>[
                ..._allSectionsUnmigrated(platform),
              ];
              settingsBeforeMigration[_swiftPackageProductDependencySectionIndex] =
                  unmigratedSwiftPackageProductDependencySection(withOtherDependency: true);
              project.xcodeProjectInfoFile.writeAsStringSync(
                _projectSettings(settingsBeforeMigration),
              );
              final List<String> settingsAsJsonBeforeMigration = <String>[
                ..._allSectionsUnmigratedAsJson(platform),
              ];
              final List<String> expectedSettings = <String>[..._allSectionsMigrated(platform)];
              expectedSettings[_swiftPackageProductDependencySectionIndex] =
                  migratedSwiftPackageProductDependencySection(withOtherDependency: true);

              final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
                _plutilOutput(settingsAsJsonBeforeMigration),
                _plutilOutput(_allSectionsMigratedAsJson(platform)),
              ]);

              final SwiftPackageManagerIntegrationMigration projectMigration =
                  SwiftPackageManagerIntegrationMigration(
                    project,
                    platform,
                    BuildInfo.debug,
                    xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                    logger: testLogger,
                    fileSystem: memoryFileSystem,
                    plistParser: plistParser,
                    features: swiftPackageManagerFullyEnabledFlags,
                  );
              await projectMigration.migrate();
              expect(testLogger.errorText, isEmpty);
              expect(
                testLogger.traceText.contains(
                  'XCSwiftPackageProductDependency already migrated. Skipping...',
                ),
                isFalse,
              );
              expect(
                project.xcodeProjectInfoFile.readAsStringSync(),
                _projectSettings(expectedSettings),
              );
              expect(plistParser.hasRemainingExpectations, isFalse);
            });
          });

          testWithoutContext('throw if settings not updated correctly', () async {
            final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
            final BufferLogger testLogger = BufferLogger.test();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: memoryFileSystem,
              logger: testLogger,
            );
            _createProjectFiles(project, platform);
            project.xcodeProjectInfoFile.writeAsStringSync(
              _projectSettings(_allSectionsUnmigrated(platform)),
            );

            final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
              _plutilOutput(_allSectionsUnmigratedAsJson(platform)),
              _plutilOutput(_allSectionsUnmigratedAsJson(platform)),
            ]);

            final SwiftPackageManagerIntegrationMigration projectMigration =
                SwiftPackageManagerIntegrationMigration(
                  project,
                  platform,
                  BuildInfo.debug,
                  xcodeProjectInterpreter: FakeXcodeProjectInterpreter(),
                  logger: testLogger,
                  fileSystem: memoryFileSystem,
                  plistParser: plistParser,
                  features: swiftPackageManagerFullyEnabledFlags,
                );
            await expectLater(
              () => projectMigration.migrate(),
              throwsToolExit(message: 'Settings were not updated correctly.'),
            );
            expect(
              testLogger.errorText,
              contains('PBXBuildFile was not migrated or was migrated incorrectly.'),
            );
            expect(
              testLogger.errorText,
              contains('PBXFrameworksBuildPhase was not migrated or was migrated incorrectly.'),
            );
            expect(
              testLogger.errorText,
              contains('PBXNativeTarget was not migrated or was migrated incorrectly.'),
            );
            expect(
              testLogger.errorText,
              contains('PBXProject was not migrated or was migrated incorrectly.'),
            );
            expect(
              testLogger.errorText,
              contains(
                'XCLocalSwiftPackageReference was not migrated or was migrated incorrectly.',
              ),
            );
            expect(
              testLogger.errorText,
              contains(
                'XCSwiftPackageProductDependency was not migrated or was migrated incorrectly.',
              ),
            );
          });
        });
      }
    });

    group('validate project settings', () {
      testWithoutContext('throw if settings fail to compile', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        const SupportedPlatform platform = SupportedPlatform.ios;
        final FakeXcodeProject project = FakeXcodeProject(
          platform: platform.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, platform);
        project.xcodeProjectInfoFile.writeAsStringSync(
          _projectSettings(_allSectionsUnmigrated(platform)),
        );

        final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
          _plutilOutput(_allSectionsUnmigratedAsJson(platform)),
          _plutilOutput(_allSectionsMigratedAsJson(platform)),
        ]);

        final SwiftPackageManagerIntegrationMigration projectMigration =
            SwiftPackageManagerIntegrationMigration(
              project,
              platform,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(throwErrorOnGetInfo: true),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: plistParser,
              features: swiftPackageManagerFullyEnabledFlags,
            );
        await expectLater(
          () => projectMigration.migrate(),
          throwsToolExit(message: 'Unable to get Xcode project information'),
        );
      });

      testWithoutContext('restore project settings from backup on failure', () async {
        final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
        final BufferLogger testLogger = BufferLogger.test();
        const SupportedPlatform platform = SupportedPlatform.ios;
        final FakeXcodeProject project = FakeXcodeProject(
          platform: platform.name,
          fileSystem: memoryFileSystem,
          logger: testLogger,
        );
        _createProjectFiles(project, platform);

        final String originalProjectInfo = _projectSettings(_allSectionsUnmigrated(platform));
        project.xcodeProjectInfoFile.writeAsStringSync(originalProjectInfo);
        final String originalSchemeContents = _validBuildActions(platform);

        final FakePlistParser plistParser = FakePlistParser.multiple(<String>[
          _plutilOutput(_allSectionsUnmigratedAsJson(platform)),
          _plutilOutput(_allSectionsMigratedAsJson(platform)),
        ]);

        final FakeSwiftPackageManagerIntegrationMigration projectMigration =
            FakeSwiftPackageManagerIntegrationMigration(
              project,
              platform,
              BuildInfo.debug,
              xcodeProjectInterpreter: FakeXcodeProjectInterpreter(throwErrorOnGetInfo: true),
              logger: testLogger,
              fileSystem: memoryFileSystem,
              plistParser: plistParser,
              features: swiftPackageManagerFullyEnabledFlags,
              validateBackup: true,
            );
        await expectLater(() async => projectMigration.migrate(), throwsToolExit());
        expect(testLogger.traceText, contains('Restoring project settings from backup file...'));
        expect(project.xcodeProjectInfoFile.readAsStringSync(), originalProjectInfo);
        expect(project.xcodeProjectSchemeFile().readAsStringSync(), originalSchemeContents);
      });
    });
  });
}

void _createProjectFiles(
  FakeXcodeProject project,
  SupportedPlatform platform, {
  bool createSchemeFile = true,
  String? scheme,
}) {
  project.parent.directory.createSync(recursive: true);
  project.hostAppRoot.createSync(recursive: true);
  project.xcodeProjectInfoFile.createSync(recursive: true);
  project.flutterPluginSwiftPackageManifest.createSync(recursive: true);
  if (createSchemeFile) {
    project.xcodeProjectSchemeFile(scheme: scheme).createSync(recursive: true);
    project.xcodeProjectSchemeFile().writeAsStringSync(_validBuildActions(platform));
  }
}

String _validBuildActions(
  SupportedPlatform platform, {
  bool hasPreActions = false,
  bool hasFrameworkScript = false,
}) {
  final String scriptText;
  if (platform == SupportedPlatform.ios) {
    scriptText =
        r'scriptText = "/bin/sh &quot;$FLUTTER_ROOT/packages/flutter_tools/bin/xcode_backend.sh&quot; prepare&#10;">';
  } else {
    scriptText =
        r'scriptText = "&quot;$FLUTTER_ROOT&quot;/packages/flutter_tools/bin/macos_assemble.sh prepare&#10;">';
  }
  String preActions = '';
  if (hasFrameworkScript) {
    preActions = '''
\n      <PreActions>
         <ExecutionAction
            ActionType = "Xcode.IDEStandardExecutionActionsCore.ExecutionActionType.ShellScriptAction">
            <ActionContent
               title = "Run Prepare Flutter Framework Script"
               $scriptText
               <EnvironmentBuildable>
                  <BuildableReference
                     BuildableIdentifier = "primary"
                     BlueprintIdentifier = "${_runnerNativeTargetIdentifier(platform)}"
                     BuildableName = "Runner.app"
                     BlueprintName = "Runner"
                     ReferencedContainer = "container:Runner.xcodeproj">
                  </BuildableReference>
               </EnvironmentBuildable>
            </ActionContent>
         </ExecutionAction>
      </PreActions>''';
  } else if (hasPreActions) {
    preActions = '''
\n      <PreActions>
      </PreActions>''';
  }
  return '''
   <BuildAction
      parallelizeBuildables = "YES"
      buildImplicitDependencies = "YES">$preActions
      <BuildActionEntries>
         <BuildActionEntry
            buildForTesting = "YES"
            buildForRunning = "YES"
            buildForProfiling = "YES"
            buildForArchiving = "YES"
            buildForAnalyzing = "YES">
${_validBuildableReference(platform)}
         </BuildActionEntry>
      </BuildActionEntries>
   </BuildAction>
''';
}

String _validBuildableReference(SupportedPlatform platform) {
  return '''
            <BuildableReference
               BuildableIdentifier = "primary"
               BlueprintIdentifier = "${_runnerNativeTargetIdentifier(platform)}"
               BuildableName = "Runner.app"
               BlueprintName = "Runner"
               ReferencedContainer = "container:Runner.xcodeproj">
            </BuildableReference>''';
}

const int _buildFileSectionIndex = 0;
const int _frameworksBuildPhaseSectionIndex = 1;
const int _nativeTargetSectionIndex = 2;
const int _projectSectionIndex = 3;
const int _localSwiftPackageReferenceSectionIndex = 4;
const int _swiftPackageProductDependencySectionIndex = 5;

List<String> _allSectionsMigrated(SupportedPlatform platform) {
  return <String>[
    migratedBuildFileSection,
    migratedFrameworksBuildPhaseSection(platform),
    migratedNativeTargetSection(platform),
    migratedProjectSection(platform),
    migratedLocalSwiftPackageReferenceSection(),
    migratedSwiftPackageProductDependencySection(),
  ];
}

List<String> _allSectionsMigratedAsJson(SupportedPlatform platform) {
  return <String>[
    migratedBuildFileSectionAsJson,
    migratedFrameworksBuildPhaseSectionAsJson(platform),
    migratedNativeTargetSectionAsJson(platform),
    migratedProjectSectionAsJson(platform),
    migratedLocalSwiftPackageReferenceSectionAsJson,
    migratedSwiftPackageProductDependencySectionAsJson,
  ];
}

List<String> _allSectionsUnmigrated(SupportedPlatform platform) {
  return <String>[
    unmigratedBuildFileSection,
    unmigratedFrameworksBuildPhaseSection(platform),
    unmigratedNativeTargetSection(platform),
    unmigratedProjectSection(platform),
    unmigratedLocalSwiftPackageReferenceSection(),
    unmigratedSwiftPackageProductDependencySection(),
  ];
}

List<String> _allSectionsUnmigratedAsJson(SupportedPlatform platform) {
  return <String>[
    unmigratedBuildFileSectionAsJson,
    unmigratedFrameworksBuildPhaseSectionAsJson(platform),
    unmigratedNativeTargetSectionAsJson(platform),
    unmigratedProjectSectionAsJson(platform),
  ];
}

String _plutilOutput(List<String> objects) {
  return '''
{
  "archiveVersion" : "1",
  "classes" : {

  },
  "objects" : {
${objects.join(',\n')}
  }
}
''';
}

String _projectSettings(List<String> objects) {
  return '''
${objects.join('\n')}
''';
}

String _runnerFrameworksBuildPhaseIdentifier(SupportedPlatform platform) {
  return platform == SupportedPlatform.ios
      ? '97C146EB1CF9000F007C117D'
      : '33CC10EA2044A3C60003C045';
}

String _runnerNativeTargetIdentifier(SupportedPlatform platform) {
  return platform == SupportedPlatform.ios
      ? '97C146ED1CF9000F007C117D'
      : '33CC10EC2044A3C60003C045';
}

String _projectIdentifier(SupportedPlatform platform) {
  return platform == SupportedPlatform.ios
      ? '97C146E61CF9000F007C117D'
      : '33CC10E52044A3C60003C045';
}

// PBXBuildFile
const String unmigratedBuildFileSection = '''
/* Begin PBXBuildFile section */
		74858FAF1ED2DC5600515810 /* AppDelegate.swift in Sources */ = {isa = PBXBuildFile; fileRef = 74858FAE1ED2DC5600515810 /* AppDelegate.swift */; };
		97C146FC1CF9000F007C117D /* Main.storyboard in Resources */ = {isa = PBXBuildFile; fileRef = 97C146FA1CF9000F007C117D /* Main.storyboard */; };
/* End PBXBuildFile section */
''';
const String migratedBuildFileSection = '''
/* Begin PBXBuildFile section */
		74858FAF1ED2DC5600515810 /* AppDelegate.swift in Sources */ = {isa = PBXBuildFile; fileRef = 74858FAE1ED2DC5600515810 /* AppDelegate.swift */; };
		97C146FC1CF9000F007C117D /* Main.storyboard in Resources */ = {isa = PBXBuildFile; fileRef = 97C146FA1CF9000F007C117D /* Main.storyboard */; };
		78A318202AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage in Frameworks */ = {isa = PBXBuildFile; productRef = 78A3181F2AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage */; };
/* End PBXBuildFile section */
''';
const String unmigratedBuildFileSectionAsJson = '''
    "97C146FC1CF9000F007C117D" : {
      "fileRef" : "97C146FA1CF9000F007C117D",
      "isa" : "PBXBuildFile"
    },
    "74858FAF1ED2DC5600515810" : {
      "fileRef" : "74858FAE1ED2DC5600515810",
      "isa" : "PBXBuildFile"
    }''';
const String migratedBuildFileSectionAsJson = '''
    "78A318202AECB46A00862997" : {
      "isa" : "PBXBuildFile",
      "productRef" : "78A3181F2AECB46A00862997"
    },
    "97C146FC1CF9000F007C117D" : {
      "fileRef" : "97C146FA1CF9000F007C117D",
      "isa" : "PBXBuildFile"
    },
    "74858FAF1ED2DC5600515810" : {
      "fileRef" : "74858FAE1ED2DC5600515810",
      "isa" : "PBXBuildFile"
    }''';

// PBXFrameworksBuildPhase
String unmigratedFrameworksBuildPhaseSection(
  SupportedPlatform platform, {
  bool withCocoapods = false,
  bool missingFiles = false,
}) {
  return <String>[
    '/* Begin PBXFrameworksBuildPhase section */',
    '		${_runnerFrameworksBuildPhaseIdentifier(platform)} /* Frameworks */ = {',
    '			isa = PBXFrameworksBuildPhase;',
    '			buildActionMask = 2147483647;',
    if (!missingFiles) ...<String>[
      '			files = (',
      if (withCocoapods) '				FD5BB45FB410D26C457F3823 /* Pods_Runner.framework in Frameworks */,',
      '			);',
    ],
    '			runOnlyForDeploymentPostprocessing = 0;',
    '		};',
    '/* End PBXFrameworksBuildPhase section */',
  ].join('\n');
}

String migratedFrameworksBuildPhaseSection(
  SupportedPlatform platform, {
  bool withCocoapods = false,
  bool missingFiles = false,
}) {
  final List<String> filesField = <String>[
    '			files = (',
    '				78A318202AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage in Frameworks */,',
    if (withCocoapods) '				FD5BB45FB410D26C457F3823 /* Pods_Runner.framework in Frameworks */,',
    '			);',
  ];
  return <String>[
    '/* Begin PBXFrameworksBuildPhase section */',
    '		${_runnerFrameworksBuildPhaseIdentifier(platform)} /* Frameworks */ = {',
    if (missingFiles) ...filesField,
    '			isa = PBXFrameworksBuildPhase;',
    '			buildActionMask = 2147483647;',
    if (!missingFiles) ...filesField,
    '			runOnlyForDeploymentPostprocessing = 0;',
    '		};',
    '/* End PBXFrameworksBuildPhase section */',
  ].join('\n');
}

String unmigratedFrameworksBuildPhaseSectionAsJson(
  SupportedPlatform platform, {
  bool withCocoapods = false,
  bool missingFiles = false,
}) {
  return <String>[
    '    "${_runnerFrameworksBuildPhaseIdentifier(platform)}" : {',
    '      "buildActionMask" : "2147483647",',
    if (!missingFiles) ...<String>[
      '      "files" : [',
      if (withCocoapods) '        "FD5BB45FB410D26C457F3823"',
      '      ],',
    ],
    '      "isa" : "PBXFrameworksBuildPhase",',
    '      "runOnlyForDeploymentPostprocessing" : "0"',
    '    }',
  ].join('\n');
}

String migratedFrameworksBuildPhaseSectionAsJson(SupportedPlatform platform) {
  return '''
    "${_runnerFrameworksBuildPhaseIdentifier(platform)}" : {
      "buildActionMask" : "2147483647",
      "files" : [
        "78A318202AECB46A00862997"
      ],
      "isa" : "PBXFrameworksBuildPhase",
      "runOnlyForDeploymentPostprocessing" : "0"
    }''';
}

// PBXNativeTarget
String unmigratedNativeTargetSection(
  SupportedPlatform platform, {
  bool missingPackageProductDependencies = false,
  bool withOtherDependency = false,
}) {
  return <String>[
    '/* Begin PBXNativeTarget section */',
    '		${_runnerNativeTargetIdentifier(platform)} /* Runner */ = {',
    '			isa = PBXNativeTarget;',
    '			buildConfigurationList = 97C147051CF9000F007C117D /* Build configuration list for PBXNativeTarget "Runner" */;',
    '			buildPhases = (',
    '				9740EEB61CF901F6004384FC /* Run Script */,',
    '				97C146EA1CF9000F007C117D /* Sources */,',
    '				${_runnerFrameworksBuildPhaseIdentifier(platform)} /* Frameworks */,',
    '				97C146EC1CF9000F007C117D /* Resources */,',
    '				9705A1C41CF9048500538489 /* Embed Frameworks */,',
    '				3B06AD1E1E4923F5004D2608 /* Thin Binary */,',
    '			);',
    '			buildRules = (',
    '			);',
    '			dependencies = (',
    '			);',
    '			name = Runner;',
    if (!missingPackageProductDependencies) ...<String>[
      '			packageProductDependencies = (',
      if (withOtherDependency) '				010101010101010101010101 /* SomeOtherPackage */,',
      '			);',
    ],
    '			productName = Runner;',
    '			productReference = 97C146EE1CF9000F007C117D /* Runner.app */;',
    '			productType = "com.apple.product-type.application";',
    '		};',
    '/* End PBXNativeTarget section */',
  ].join('\n');
}

String migratedNativeTargetSection(
  SupportedPlatform platform, {
  bool missingPackageProductDependencies = false,
  bool withOtherDependency = false,
}) {
  final List<String> packageDependencies = <String>[
    '			packageProductDependencies = (',
    '				78A3181F2AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage */,',
    if (withOtherDependency) '				010101010101010101010101 /* SomeOtherPackage */,',
    '			);',
  ];
  return <String>[
    '/* Begin PBXNativeTarget section */',
    '		${_runnerNativeTargetIdentifier(platform)} /* Runner */ = {',
    if (missingPackageProductDependencies) ...packageDependencies,
    '			isa = PBXNativeTarget;',
    '			buildConfigurationList = 97C147051CF9000F007C117D /* Build configuration list for PBXNativeTarget "Runner" */;',
    '			buildPhases = (',
    '				9740EEB61CF901F6004384FC /* Run Script */,',
    '				97C146EA1CF9000F007C117D /* Sources */,',
    '				${_runnerFrameworksBuildPhaseIdentifier(platform)} /* Frameworks */,',
    '				97C146EC1CF9000F007C117D /* Resources */,',
    '				9705A1C41CF9048500538489 /* Embed Frameworks */,',
    '				3B06AD1E1E4923F5004D2608 /* Thin Binary */,',
    '			);',
    '			buildRules = (',
    '			);',
    '			dependencies = (',
    '			);',
    '			name = Runner;',
    if (!missingPackageProductDependencies) ...packageDependencies,
    '			productName = Runner;',
    '			productReference = 97C146EE1CF9000F007C117D /* Runner.app */;',
    '			productType = "com.apple.product-type.application";',
    '		};',
    '/* End PBXNativeTarget section */',
  ].join('\n');
}

String unmigratedNativeTargetSectionAsJson(
  SupportedPlatform platform, {
  bool missingPackageProductDependencies = false,
}) {
  return <String>[
    '    "${_runnerNativeTargetIdentifier(platform)}" : {',
    '      "buildConfigurationList" : "97C147051CF9000F007C117D",',
    '      "buildPhases" : [',
    '        "9740EEB61CF901F6004384FC",',
    '        "97C146EA1CF9000F007C117D",',
    '        "${_runnerFrameworksBuildPhaseIdentifier(platform)}",',
    '        "97C146EC1CF9000F007C117D",',
    '        "9705A1C41CF9048500538489",',
    '        "3B06AD1E1E4923F5004D2608"',
    '      ],',
    '      "buildRules" : [',
    '      ],',
    '      "dependencies" : [',
    '      ],',
    '      "isa" : "PBXNativeTarget",',
    '      "name" : "Runner",',
    if (!missingPackageProductDependencies) ...<String>[
      '      "packageProductDependencies" : [',
      '      ],',
    ],
    '      "productName" : "Runner",',
    '      "productReference" : "97C146EE1CF9000F007C117D",',
    '      "productType" : "com.apple.product-type.application"',
    '    }',
  ].join('\n');
}

String migratedNativeTargetSectionAsJson(SupportedPlatform platform) {
  return '''
    "${_runnerNativeTargetIdentifier(platform)}" : {
      "buildConfigurationList" : "97C147051CF9000F007C117D",
      "buildPhases" : [
        "9740EEB61CF901F6004384FC",
        "97C146EA1CF9000F007C117D",
        "${_runnerFrameworksBuildPhaseIdentifier(platform)}",
        "97C146EC1CF9000F007C117D",
        "9705A1C41CF9048500538489",
        "3B06AD1E1E4923F5004D2608"
      ],
      "buildRules" : [

      ],
      "dependencies" : [

      ],
      "isa" : "PBXNativeTarget",
      "name" : "Runner",
      "packageProductDependencies" : [
        "78A3181F2AECB46A00862997"
      ],
      "productName" : "Runner",
      "productReference" : "97C146EE1CF9000F007C117D",
      "productType" : "com.apple.product-type.application"
    }''';
}

// PBXProject
String unmigratedProjectSection(
  SupportedPlatform platform, {
  bool missingPackageReferences = false,
  bool withOtherReference = false,
}) {
  return <String>[
    '/* Begin PBXProject section */',
    '		${_projectIdentifier(platform)} /* Project object */ = {',
    '			isa = PBXProject;',
    '			attributes = {',
    '				BuildIndependentTargetsInParallel = YES;',
    '				LastUpgradeCheck = 1510;',
    '				ORGANIZATIONNAME = "";',
    '				TargetAttributes = {',
    '					331C8080294A63A400263BE5 = {',
    '						CreatedOnToolsVersion = 14.0;',
    '						TestTargetID = ${_runnerNativeTargetIdentifier(platform)};',
    '					};',
    '					${_runnerNativeTargetIdentifier(platform)} = {',
    '						CreatedOnToolsVersion = 7.3.1;',
    '						LastSwiftMigration = 1100;',
    '					};',
    '				};',
    '			};',
    '			buildConfigurationList = 97C146E91CF9000F007C117D /* Build configuration list for PBXProject "Runner" */;',
    '			compatibilityVersion = "Xcode 9.3";',
    '			developmentRegion = en;',
    '			hasScannedForEncodings = 0;',
    '			knownRegions = (',
    '				en,',
    '				Base,',
    '			);',
    '			mainGroup = 97C146E51CF9000F007C117D;',
    if (!missingPackageReferences) ...<String>[
      '			packageReferences = (',
      if (withOtherReference)
        '				010101010101010101010101 /* XCLocalSwiftPackageReference "SomeOtherPackage" */,',
      '			);',
    ],
    '			productRefGroup = 97C146EF1CF9000F007C117D /* Products */;',
    '			projectDirPath = "";',
    '			projectRoot = "";',
    '			targets = (',
    '				${_runnerNativeTargetIdentifier(platform)} /* Runner */,',
    '				331C8080294A63A400263BE5 /* RunnerTests */,',
    '			);',
    '		};',
    '/* End PBXProject section */',
  ].join('\n');
}

String migratedProjectSection(
  SupportedPlatform platform, {
  bool missingPackageReferences = false,
  bool withOtherReference = false,
}) {
  final List<String> packageDependencies = <String>[
    '			packageReferences = (',
    '				781AD8BC2B33823900A9FFBB /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */,',
    if (withOtherReference)
      '				010101010101010101010101 /* XCLocalSwiftPackageReference "SomeOtherPackage" */,',
    '			);',
  ];
  return <String>[
    '/* Begin PBXProject section */',
    '		${_projectIdentifier(platform)} /* Project object */ = {',
    if (missingPackageReferences) ...packageDependencies,
    '			isa = PBXProject;',
    '			attributes = {',
    '				BuildIndependentTargetsInParallel = YES;',
    '				LastUpgradeCheck = 1510;',
    '				ORGANIZATIONNAME = "";',
    '				TargetAttributes = {',
    '					331C8080294A63A400263BE5 = {',
    '						CreatedOnToolsVersion = 14.0;',
    '						TestTargetID = ${_runnerNativeTargetIdentifier(platform)};',
    '					};',
    '					${_runnerNativeTargetIdentifier(platform)} = {',
    '						CreatedOnToolsVersion = 7.3.1;',
    '						LastSwiftMigration = 1100;',
    '					};',
    '				};',
    '			};',
    '			buildConfigurationList = 97C146E91CF9000F007C117D /* Build configuration list for PBXProject "Runner" */;',
    '			compatibilityVersion = "Xcode 9.3";',
    '			developmentRegion = en;',
    '			hasScannedForEncodings = 0;',
    '			knownRegions = (',
    '				en,',
    '				Base,',
    '			);',
    '			mainGroup = 97C146E51CF9000F007C117D;',
    if (!missingPackageReferences) ...packageDependencies,
    '			productRefGroup = 97C146EF1CF9000F007C117D /* Products */;',
    '			projectDirPath = "";',
    '			projectRoot = "";',
    '			targets = (',
    '				${_runnerNativeTargetIdentifier(platform)} /* Runner */,',
    '				331C8080294A63A400263BE5 /* RunnerTests */,',
    '			);',
    '		};',
    '/* End PBXProject section */',
  ].join('\n');
}

String unmigratedProjectSectionAsJson(
  SupportedPlatform platform, {
  bool missingPackageReferences = false,
}) {
  return <String>[
    '    "${_projectIdentifier(platform)}" : {',
    '      "attributes" : {',
    '        "BuildIndependentTargetsInParallel" : "YES",',
    '        "LastUpgradeCheck" : "1510",',
    '        "ORGANIZATIONNAME" : "",',
    '        "TargetAttributes" : {',
    '          "${_runnerNativeTargetIdentifier(platform)}" : {',
    '            "CreatedOnToolsVersion" : "7.3.1",',
    '            "LastSwiftMigration" : "1100"',
    '          },',
    '          "331C8080294A63A400263BE5" : {',
    '            "CreatedOnToolsVersion" : "14.0",',
    '            "TestTargetID" : "${_runnerNativeTargetIdentifier(platform)}"',
    '          }',
    '        }',
    '      },',
    '      "buildConfigurationList" : "97C146E91CF9000F007C117D",',
    '      "compatibilityVersion" : "Xcode 9.3",',
    '      "developmentRegion" : "en",',
    '      "hasScannedForEncodings" : "0",',
    '      "isa" : "PBXProject",',
    '      "knownRegions" : [',
    '        "en",',
    '        "Base"',
    '      ],',
    '      "mainGroup" : "97C146E51CF9000F007C117D",',
    if (!missingPackageReferences) ...<String>['      "packageReferences" : [', '      ],'],
    '      "productRefGroup" : "97C146EF1CF9000F007C117D",',
    '      "projectDirPath" : "",',
    '      "projectRoot" : "",',
    '      "targets" : [',
    '        "${_runnerNativeTargetIdentifier(platform)}",',
    '        "331C8080294A63A400263BE5"',
    '      ]',
    '    }',
  ].join('\n');
}

String migratedProjectSectionAsJson(SupportedPlatform platform) {
  return '''
    "${_projectIdentifier(platform)}" : {
      "attributes" : {
        "BuildIndependentTargetsInParallel" : "YES",
        "LastUpgradeCheck" : "1510",
        "ORGANIZATIONNAME" : "",
        "TargetAttributes" : {
          "${_runnerNativeTargetIdentifier(platform)}" : {
            "CreatedOnToolsVersion" : "7.3.1",
            "LastSwiftMigration" : "1100"
          },
          "331C8080294A63A400263BE5" : {
            "CreatedOnToolsVersion" : "14.0",
            "TestTargetID" : "${_runnerNativeTargetIdentifier(platform)}"
          }
        }
      },
      "buildConfigurationList" : "97C146E91CF9000F007C117D",
      "compatibilityVersion" : "Xcode 9.3",
      "developmentRegion" : "en",
      "hasScannedForEncodings" : "0",
      "isa" : "PBXProject",
      "knownRegions" : [
        "en",
        "Base"
      ],
      "mainGroup" : "97C146E51CF9000F007C117D",
      "packageReferences" : [
        "781AD8BC2B33823900A9FFBB"
      ],
      "productRefGroup" : "97C146EF1CF9000F007C117D",
      "projectDirPath" : "",
      "projectRoot" : "",
      "targets" : [
        "${_runnerNativeTargetIdentifier(platform)}",
        "331C8080294A63A400263BE5"
      ]
    }''';
}

// XCLocalSwiftPackageReference
String unmigratedLocalSwiftPackageReferenceSection({bool withOtherReference = false}) {
  return <String>[
    '/* Begin XCLocalSwiftPackageReference section */',
    if (withOtherReference) ...<String>[
      '		010101010101010101010101 /* XCLocalSwiftPackageReference "SomeOtherPackage" */ = {',
      '			isa = XCLocalSwiftPackageReference;',
      '			relativePath = SomeOtherPackage;',
      '		};',
    ],
    '/* End XCLocalSwiftPackageReference section */',
  ].join('\n');
}

String migratedLocalSwiftPackageReferenceSection({bool withOtherReference = false}) {
  return <String>[
    '/* Begin XCLocalSwiftPackageReference section */',
    if (withOtherReference) ...<String>[
      '		010101010101010101010101 /* XCLocalSwiftPackageReference "SomeOtherPackage" */ = {',
      '			isa = XCLocalSwiftPackageReference;',
      '			relativePath = SomeOtherPackage;',
      '		};',
    ],
    '		781AD8BC2B33823900A9FFBB /* XCLocalSwiftPackageReference "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage" */ = {',
    '			isa = XCLocalSwiftPackageReference;',
    '			relativePath = Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage;',
    '		};',
    '/* End XCLocalSwiftPackageReference section */',
  ].join('\n');
}

const String migratedLocalSwiftPackageReferenceSectionAsJson = '''
    "781AD8BC2B33823900A9FFBB" : {
      "isa" : "XCLocalSwiftPackageReference",
      "relativePath" : "Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage"
    }''';

// XCSwiftPackageProductDependency
String unmigratedSwiftPackageProductDependencySection({bool withOtherDependency = false}) {
  return <String>[
    '/* Begin XCSwiftPackageProductDependency section */',
    if (withOtherDependency) ...<String>[
      '		010101010101010101010101 /* SomeOtherPackage */ = {',
      '			isa = XCSwiftPackageProductDependency;',
      '			productName = SomeOtherPackage;',
      '		};',
    ],
    '/* End XCSwiftPackageProductDependency section */',
  ].join('\n');
}

String migratedSwiftPackageProductDependencySection({bool withOtherDependency = false}) {
  return <String>[
    '/* Begin XCSwiftPackageProductDependency section */',
    if (withOtherDependency) ...<String>[
      '		010101010101010101010101 /* SomeOtherPackage */ = {',
      '			isa = XCSwiftPackageProductDependency;',
      '			productName = SomeOtherPackage;',
      '		};',
    ],
    '		78A3181F2AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage */ = {',
    '			isa = XCSwiftPackageProductDependency;',
    '			productName = FlutterGeneratedPluginSwiftPackage;',
    '		};',
    '/* End XCSwiftPackageProductDependency section */',
  ].join('\n');
}

const String migratedSwiftPackageProductDependencySectionAsJson = '''
    "78A3181F2AECB46A00862997" : {
      "isa" : "XCSwiftPackageProductDependency",
      "productName" : "FlutterGeneratedPluginSwiftPackage"
    }''';

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  FakeXcodeProjectInterpreter({this.throwErrorOnGetInfo = false});

  @override
  bool isInstalled = false;

  @override
  List<String> xcrunCommand() => <String>['xcrun'];

  final bool throwErrorOnGetInfo;

  @override
  Future<XcodeProjectInfo?> getInfo(String projectPath, {String? projectFilename}) async {
    if (throwErrorOnGetInfo) {
      throwToolExit('Unable to get Xcode project information');
    }
    return null;
  }
}

class FakePlistParser extends Fake implements PlistParser {
  FakePlistParser({String? json}) : _outputPerCall = (json != null) ? <String>[json] : null;

  FakePlistParser.multiple(this._outputPerCall);

  final List<String>? _outputPerCall;

  @override
  String? plistJsonContent(String filePath) {
    if (_outputPerCall != null && _outputPerCall.isNotEmpty) {
      return _outputPerCall.removeAt(0);
    }
    return null;
  }

  bool get hasRemainingExpectations {
    return _outputPerCall != null && _outputPerCall.isNotEmpty;
  }
}

class FakeXcodeProject extends Fake implements IosProject {
  FakeXcodeProject({
    required MemoryFileSystem fileSystem,
    required String platform,
    required this.logger,
  }) : hostAppRoot = fileSystem.directory('app_name').childDirectory(platform),
       parent = FakeFlutterProject(fileSystem: fileSystem);

  final Logger logger;
  late XcodeProjectInfo? _projectInfo = XcodeProjectInfo(
    <String>['Runner'],
    <String>['Debug', 'Release', 'Profile'],
    <String>['Runner'],
    logger,
  );

  @override
  Directory hostAppRoot;

  @override
  FakeFlutterProject parent;

  @override
  Directory get xcodeProject => hostAppRoot.childDirectory('$hostAppProjectName.xcodeproj');

  @override
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  @override
  late Directory? xcodeWorkspace = hostAppRoot.childDirectory('$hostAppProjectName.xcworkspace');

  @override
  String hostAppProjectName = 'Runner';

  @override
  Directory get flutterPluginSwiftPackageDirectory => hostAppRoot
      .childDirectory('Flutter')
      .childDirectory('ephemeral')
      .childDirectory('Packages')
      .childDirectory('FlutterGeneratedPluginSwiftPackage');

  @override
  File get flutterPluginSwiftPackageManifest =>
      flutterPluginSwiftPackageDirectory.childFile('Package.swift');

  @override
  bool get flutterPluginSwiftPackageInProjectSettings {
    return xcodeProjectInfoFile.existsSync() &&
        xcodeProjectInfoFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage');
  }

  @override
  Future<XcodeProjectInfo?> projectInfo() async {
    return _projectInfo;
  }

  @override
  File xcodeProjectSchemeFile({String? scheme}) {
    final String schemeName = scheme ?? 'Runner';
    return xcodeProject
        .childDirectory('xcshareddata')
        .childDirectory('xcschemes')
        .childFile('$schemeName.xcscheme');
  }
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({required MemoryFileSystem fileSystem})
    : directory = fileSystem.directory('app_name');

  @override
  Directory directory;
}

class FakeSwiftPackageManagerIntegrationMigration extends SwiftPackageManagerIntegrationMigration {
  FakeSwiftPackageManagerIntegrationMigration(
    super.project,
    super.platform,
    super.buildInfo, {
    required super.xcodeProjectInterpreter,
    required super.logger,
    required super.fileSystem,
    required super.plistParser,
    required super.features,
    this.validateBackup = false,
  }) : _xcodeProject = project;

  final XcodeBasedProject _xcodeProject;

  final bool validateBackup;
  @override
  void restoreFromBackup(SchemeInfo? schemeInfo) {
    if (validateBackup) {
      expect(backupProjectSettings.existsSync(), isTrue);
      final String originalSettings = backupProjectSettings.readAsStringSync();
      expect(_xcodeProject.xcodeProjectInfoFile.readAsStringSync() == originalSettings, isFalse);

      expect(schemeInfo?.backupSchemeFile, isNotNull);
      final File backupScheme = schemeInfo!.backupSchemeFile!;
      expect(backupScheme.existsSync(), isTrue);
      final String originalScheme = backupScheme.readAsStringSync();
      expect(_xcodeProject.xcodeProjectSchemeFile().readAsStringSync() == originalScheme, isFalse);

      super.restoreFromBackup(schemeInfo);
      expect(_xcodeProject.xcodeProjectInfoFile.readAsStringSync(), originalSettings);
      expect(_xcodeProject.xcodeProjectSchemeFile().readAsStringSync(), originalScheme);
    } else {
      super.restoreFromBackup(schemeInfo);
    }
  }
}
