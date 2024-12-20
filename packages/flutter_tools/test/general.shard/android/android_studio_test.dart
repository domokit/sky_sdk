// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';

void main() {
  group('installation detection on MacOS', () {
    const String homeMac = '/Users/me';

    const Map<String, Object> macStudioInfoPlist3_3 = <String, Object>{
      'CFBundleGetInfoString':
          'Android Studio 3.3, build AI-182.5107.16.33.5199772. Copyright JetBrains s.r.o., (c) 2000-2018',
      'CFBundleShortVersionString': '3.3',
      'CFBundleVersion': 'AI-182.5107.16.33.5199772',
      'JVMOptions': <String, Object>{
        'Properties': <String, Object>{
          'idea.paths.selector': 'AndroidStudio3.3',
          'idea.platform.prefix': 'AndroidStudio',
        },
      },
    };

    const Map<String, Object> macStudioInfoPlist4_1 = <String, Object>{
      'CFBundleGetInfoString':
          'Android Studio 4.1, build AI-201.8743.12.41.6858069. Copyright JetBrains s.r.o., (c) 2000-2020',
      'CFBundleShortVersionString': '4.1',
      'CFBundleVersion': 'AI-201.8743.12.41.6858069',
      'JVMOptions': <String, Object>{
        'Properties': <String, Object>{
          'idea.vendor.name': 'Google',
          'idea.paths.selector': 'AndroidStudio4.1',
          'idea.platform.prefix': 'AndroidStudio',
        },
      },
    };

    const Map<String, Object> macStudioInfoPlist2020_3 = <String, Object>{
      'CFBundleGetInfoString':
          'Android Studio 2020.3, build AI-203.7717.56.2031.7583922. Copyright JetBrains s.r.o., (c) 2000-2021',
      'CFBundleShortVersionString': '2020.3',
      'CFBundleVersion': 'AI-203.7717.56.2031.7583922',
      'JVMOptions': <String, Object>{
        'Properties': <String, Object>{
          'idea.vendor.name': 'Google',
          'idea.paths.selector': 'AndroidStudio2020.3',
          'idea.platform.prefix': 'AndroidStudio',
        },
      },
    };

    const Map<String, Object> macStudioInfoPlist2022_1 = <String, Object>{
      'CFBundleGetInfoString':
          'Android Studio 2022.1, build AI-221.6008.13.2211.9477386. Copyright JetBrains s.r.o., (c) 2000-2023',
      'CFBundleShortVersionString': '2022.1',
      'CFBundleVersion': 'AI-221.6008.13.2211.9477386',
      'JVMOptions': <String, Object>{
        'Properties': <String, Object>{
          'idea.vendor.name': 'Google',
          'idea.paths.selector': 'AndroidStudio2022.1',
          'idea.platform.prefix': 'AndroidStudio',
        },
      },
    };

    const Map<String, Object> macStudioInfoPlistEap_2022_3_1_11 = <String, Object>{
      'CFBundleGetInfoString':
          'Android Studio EAP AI-223.8836.35.2231.9848316, build AI-223.8836.35.2231.9848316. Copyright JetBrains s.r.o., (c) 2000-2023',
      'CFBundleShortVersionString': 'EAP AI-223.8836.35.2231.9848316',
      'CFBundleVersion': 'AI-223.8836.35.2231.9848316',
      'JVMOptions': <String, Object>{
        'Properties': <String, Object>{
          'idea.vendor.name': 'Google',
          'idea.paths.selector': 'AndroidStudioPreview2022.3',
          'idea.platform.prefix': 'AndroidStudio',
        },
      },
    };

    late Config config;
    late FileSystem fileSystem;
    late FileSystemUtils fsUtils;
    late Platform platform;
    late FakePlistUtils plistUtils;
    late FakeProcessManager processManager;

    setUp(() {
      config = Config.test();
      fileSystem = MemoryFileSystem.test();
      plistUtils = FakePlistUtils();
      platform = FakePlatform(
        operatingSystem: 'macos',
        environment: <String, String>{'HOME': homeMac},
      );
      fsUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform);
      processManager = FakeProcessManager.empty();
    });

    testUsingContext(
      'discovers Android Studio >=4.1 location',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist4_1;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jre',
                'jdk',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, equals(Version(4, 1, null)));
        expect(studio, isNotNull);
        expect(
          studio.pluginsPath,
          equals(
            fileSystem.path.join(
              homeMac,
              'Library',
              'Application Support',
              'Google',
              'AndroidStudio4.1',
            ),
          ),
        );
        expect(studio.validationMessages, <String>['Java version 123']);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'discovers Android Studio >=2020.3 location',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist2020_3;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jre',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, equals(Version(2020, 3, null)));
        expect(studio, isNotNull);
        expect(
          studio.pluginsPath,
          equals(
            fileSystem.path.join(
              homeMac,
              'Library',
              'Application Support',
              'Google',
              'AndroidStudio2020.3',
            ),
          ),
        );
        expect(studio.validationMessages, <String>['Java version 123']);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'discovers Android Studio <4.1 location',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist3_3;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jre',
                'jdk',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, equals(Version(3, 3, null)));
        expect(studio, isNotNull);
        expect(
          studio.pluginsPath,
          equals(
            fileSystem.path.join(homeMac, 'Library', 'Application Support', 'AndroidStudio3.3'),
          ),
        );
        expect(studio.validationMessages, <String>['Java version 123']);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'discovers Android Studio EAP location',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio with suffix.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlistEap_2022_3_1_11;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jbr',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, equals(Version(2022, 3, 1)));
        expect(studio, isNotNull);
        expect(
          studio.pluginsPath,
          equals(
            fileSystem.path.join(
              homeMac,
              'Library',
              'Application Support',
              'Google',
              'AndroidStudioPreview2022.3',
            ),
          ),
        );
        expect(studio.validationMessages, <String>['Java version 123']);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'does not discover Android Studio with JetBrainsToolboxApp wrapper',
      () {
        final String applicationPlistFolder = fileSystem.path.join(
          '/',
          'Applications',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(applicationPlistFolder).createSync(recursive: true);

        final String applicationsPlistFilePath = fileSystem.path.join(
          applicationPlistFolder,
          'Info.plist',
        );
        const Map<String, Object> jetbrainsInfoPlist = <String, Object>{
          'JetBrainsToolboxApp': 'ignored',
        };
        plistUtils.fileContents[applicationsPlistFilePath] = jetbrainsInfoPlist;

        final String homeDirectoryPlistFolder = fileSystem.path.join(
          fsUtils.homeDirPath!,
          'Applications',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(homeDirectoryPlistFolder).createSync(recursive: true);

        final String homeDirectoryPlistFilePath = fileSystem.path.join(
          homeDirectoryPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[homeDirectoryPlistFilePath] = macStudioInfoPlist2020_3;

        expect(AndroidStudio.allInstalled().length, 1);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => FakeProcessManager.any(),
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'discovers installation from Spotlight query',
      () {
        // One in expected location.
        final String studioInApplication = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
        );
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          studioInApplication,
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);
        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist4_1;

        // Two in random location only Spotlight knows about.
        final String randomLocation1 = fileSystem.path.join(
          '/',
          'random',
          'Android Studio Preview.app',
        );
        final String randomLocation1PlistFolder = fileSystem.path.join(randomLocation1, 'Contents');
        fileSystem.directory(randomLocation1PlistFolder).createSync(recursive: true);
        final String randomLocation1PlistPath = fileSystem.path.join(
          randomLocation1PlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[randomLocation1PlistPath] = macStudioInfoPlist4_1;

        final String randomLocation2 = fileSystem.path.join(
          '/',
          'random',
          'Android Studio with Blaze.app',
        );
        final String randomLocation2PlistFolder = fileSystem.path.join(randomLocation2, 'Contents');
        fileSystem.directory(randomLocation2PlistFolder).createSync(recursive: true);
        final String randomLocation2PlistPath = fileSystem.path.join(
          randomLocation2PlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[randomLocation2PlistPath] = macStudioInfoPlist4_1;
        final String javaBin = fileSystem.path.join(
          'jre',
          'jdk',
          'Contents',
          'Home',
          'bin',
          'java',
        );

        // Spotlight finds the one known and two random installations.
        processManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: const <String>[
              'mdfind',
              'kMDItemCFBundleIdentifier="com.google.android.studio*"',
            ],
            stdout: '$randomLocation1\n$randomLocation2\n$studioInApplication',
          ),
          FakeCommand(
            command: <String>[
              fileSystem.path.join(randomLocation1, 'Contents', javaBin),
              '-version',
            ],
          ),
          FakeCommand(
            command: <String>[
              fileSystem.path.join(randomLocation2, 'Contents', javaBin),
              '-version',
            ],
          ),
          FakeCommand(
            command: <String>[
              fileSystem.path.join(studioInApplicationPlistFolder, javaBin),
              '-version',
            ],
          ),
        ]);

        // Results are de-duplicated, only 3 installed.
        expect(AndroidStudio.allInstalled().length, 3);
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'finds latest valid install',
      () {
        final String applicationPlistFolder = fileSystem.path.join(
          '/',
          'Applications',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(applicationPlistFolder).createSync(recursive: true);

        final String applicationsPlistFilePath = fileSystem.path.join(
          applicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[applicationsPlistFilePath] = macStudioInfoPlist3_3;

        final String homeDirectoryPlistFolder = fileSystem.path.join(
          fsUtils.homeDirPath!,
          'Applications',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(homeDirectoryPlistFolder).createSync(recursive: true);

        final String homeDirectoryPlistFilePath = fileSystem.path.join(
          homeDirectoryPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[homeDirectoryPlistFilePath] = macStudioInfoPlist4_1;

        expect(AndroidStudio.allInstalled().length, 2);
        expect(AndroidStudio.latestValid()!.version, Version(4, 1, 0));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => FakeProcessManager.any(),
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'extracts custom paths for directly downloaded Android Studio',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist3_3;
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;
        expect(studio, isNotNull);
        expect(
          studio.pluginsPath,
          equals(
            fileSystem.path.join(homeMac, 'Library', 'Application Support', 'AndroidStudio3.3'),
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => FakeProcessManager.any(),
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'finds Android Studio 2020.3 bundled Java version',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist2020_3;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jre',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(
          studio.javaPath,
          equals(fileSystem.path.join(studioInApplicationPlistFolder, 'jre', 'Contents', 'Home')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'finds Android Studio 2022.1 bundled Java version',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist2022_1;
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              fileSystem.path.join(
                studioInApplicationPlistFolder,
                'jbr',
                'Contents',
                'Home',
                'bin',
                'java',
              ),
              '-version',
            ],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(
          studio.javaPath,
          equals(fileSystem.path.join(studioInApplicationPlistFolder, 'jbr', 'Contents', 'Home')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'finds bundled Java version when Android Studio version is unknown by assuming the latest version',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        final Map<String, Object> plistWithoutVersion = Map<String, Object>.from(
          macStudioInfoPlist2022_1,
        );
        plistWithoutVersion['CFBundleShortVersionString'] = '';
        plistUtils.fileContents[plistFilePath] = plistWithoutVersion;

        final String jdkPath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'jbr',
          'Contents',
          'Home',
        );

        processManager.addCommand(
          FakeCommand(
            command: <String>[fileSystem.path.join(jdkPath, 'bin', 'java'), '-version'],
            stderr: '123',
          ),
        );
        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, null);
        expect(studio.javaPath, jdkPath);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'when given an Android Studio newer than any known version, finds Java version by assuming latest known Android Studio version',
      () {
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          '/',
          'Application',
          'Android Studio.app',
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);

        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        final Map<String, Object> plistWithoutVersion = Map<String, Object>.from(
          macStudioInfoPlist2022_1,
        );
        plistWithoutVersion['CFBundleShortVersionString'] = '99999.99.99';
        plistUtils.fileContents[plistFilePath] = plistWithoutVersion;

        final String jdkPathFor2022 = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'jbr',
          'Contents',
          'Home',
        );

        final AndroidStudio studio =
            AndroidStudio.fromMacOSBundle(
              fileSystem.directory(studioInApplicationPlistFolder).parent.path,
            )!;

        expect(studio.version, equals(Version(99999, 99, 99)));
        expect(studio.javaPath, jdkPathFor2022);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => FakeProcessManager.any(),
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );

    testUsingContext(
      'discovers explicitly configured Android Studio',
      () {
        final String extractedDownloadZip = fileSystem.path.join(
          '/',
          'Users',
          'Dash',
          'Desktop',
          'android-studio',
        );
        config.setValue('android-studio-dir', extractedDownloadZip);
        final String studioInApplicationPlistFolder = fileSystem.path.join(
          extractedDownloadZip,
          'Contents',
        );
        fileSystem.directory(studioInApplicationPlistFolder).createSync(recursive: true);
        final String plistFilePath = fileSystem.path.join(
          studioInApplicationPlistFolder,
          'Info.plist',
        );
        plistUtils.fileContents[plistFilePath] = macStudioInfoPlist2022_1;

        final String studioInApplicationJavaBinary = fileSystem.path.join(
          extractedDownloadZip,
          'Contents',
          'jbr',
          'Contents',
          'Home',
          'bin',
          'java',
        );

        processManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: const <String>[
              'mdfind',
              'kMDItemCFBundleIdentifier="com.google.android.studio*"',
            ],
            stdout: extractedDownloadZip,
          ),
          FakeCommand(command: <String>[studioInApplicationJavaBinary, '-version']),
        ]);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.configuredPath, extractedDownloadZip);
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        Config: () => config,
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        ProcessManager: () => processManager,
        Platform: () => platform,
        PlistParser: () => plistUtils,
      },
    );
  });

  group('installation detection on Windows', () {
    late Config config;
    late Platform platform;
    late FileSystem fileSystem;

    setUp(() {
      config = Config.test();
      platform = FakePlatform(
        operatingSystem: 'windows',
        environment: <String, String>{'LOCALAPPDATA': r'C:\Users\Dash\AppData\Local'},
      );
      fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    });

    testUsingContext(
      'discovers Android Studio 4.1 location',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio4.1\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(4, 1, 0));
        expect(studio.studioAppName, 'Android Studio');
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'discovers Android Studio 4.2 location',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio4.2\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(4, 2, 0));
        expect(studio.studioAppName, 'Android Studio');
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'discovers Android Studio 2020.3 location',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio2020.3\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(2020, 3, 0));
        expect(studio.studioAppName, 'Android Studio');
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'does not discover Android Studio 4.1 location if LOCALAPPDATA is null',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio4.1\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        expect(AndroidStudio.allInstalled(), isEmpty);
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              operatingSystem: 'windows',
              environment: <String, String>{}, // Does not include LOCALAPPDATA
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'does not discover Android Studio 4.2 location if LOCALAPPDATA is null',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio4.2\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        expect(AndroidStudio.allInstalled(), isEmpty);
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              operatingSystem: 'windows',
              environment: <String, String>{}, // Does not include LOCALAPPDATA
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'does not discover Android Studio 2020.3 location if LOCALAPPDATA is null',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio2020.3\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        expect(AndroidStudio.allInstalled(), isEmpty);
      },
      overrides: <Type, Generator>{
        Platform:
            () => FakePlatform(
              operatingSystem: 'windows',
              environment: <String, String>{}, // Does not include LOCALAPPDATA
            ),
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds Android Studio 2020.3 bundled Java version',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio2020.3\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.javaPath, equals(r'C:\Program Files\AndroidStudio\jre'));
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds Android Studio 2022.1 bundled Java version',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio2022.1\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.javaPath, equals(r'C:\Program Files\AndroidStudio\jbr'));
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds bundled Java version when Android Studio version is unknown by assuming the latest version',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        fileSystem.file(r'C:\Program Files\AndroidStudio\jbr\bin\java').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, null);
        expect(studio.javaPath, equals(r'C:\Program Files\AndroidStudio\jbr'));
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'when given an Android Studio newer than any known version, finds Java version by assuming latest known Android Studio version',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio99999.99.99\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio');
        fileSystem.directory(r'C:\Program Files\AndroidStudio').createSync(recursive: true);

        fileSystem.file(r'C:\Program Files\AndroidStudio\jbr\bin\java').createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        const String expectedJdkLocationFor2022 = r'C:\Program Files\AndroidStudio\jbr';
        expect(studio.version, equals(Version(99999, 99, 99)));
        expect(studio.javaPath, equals(expectedJdkLocationFor2022));
      },
      overrides: <Type, Generator>{
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'discovers explicitly configured Android Studio',
      () {
        const String androidStudioDir = r'C:\Users\Dash\Desktop\android-studio';
        config.setValue('android-studio-dir', androidStudioDir);
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio2022.1\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(androidStudioDir);
        fileSystem.directory(androidStudioDir).createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, equals(Version(2022, 1, null)));
        expect(studio.configuredPath, androidStudioDir);
        expect(studio.javaPath, fileSystem.path.join(androidStudioDir, 'jbr'));
      },
      overrides: <Type, Generator>{
        Config: () => config,
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('installation detection on Linux', () {
    const String homeLinux = '/home/me';

    late Config config;
    late FileSystem fileSystem;
    late FileSystemUtils fsUtils;
    late Platform platform;

    setUp(() {
      config = Config.test();
      platform = FakePlatform(environment: <String, String>{'HOME': homeLinux});
      fileSystem = MemoryFileSystem.test();
      fsUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform);
    });

    testUsingContext(
      'discovers Android Studio <4.1',
      () {
        const String studioHomeFilePath = '$homeLinux/.AndroidStudio4.0/system/.home';
        const String studioInstallPath = '$homeLinux/AndroidStudio';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync();

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(4, 0, 0));
        expect(studio.studioAppName, 'AndroidStudio');
        expect(studio.pluginsPath, '/home/me/.AndroidStudio4.0/config/plugins');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'discovers Android Studio >=4.1',
      () {
        const String studioHomeFilePath = '$homeLinux/.cache/Google/AndroidStudio4.1/.home';
        const String studioInstallPath = '$homeLinux/AndroidStudio';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync();

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(4, 1, 0));
        expect(studio.studioAppName, 'AndroidStudio');
        expect(studio.pluginsPath, '/home/me/.local/share/Google/AndroidStudio4.1');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'discovers when installed with Toolbox',
      () {
        const String studioHomeFilePath = '$homeLinux/.cache/Google/AndroidStudio4.1/.home';
        const String studioInstallPath =
            '$homeLinux/.local/share/JetBrains/Toolbox/apps/AndroidStudio/ch-0/201.7042882';
        const String pluginsInstallPath = '$studioInstallPath.plugins';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync(recursive: true);
        fileSystem.directory(pluginsInstallPath).createSync();

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, Version(4, 1, 0));
        expect(studio.studioAppName, 'AndroidStudio');
        expect(studio.pluginsPath, pluginsInstallPath);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds Android Studio 2020.3 bundled Java version',
      () {
        const String studioHomeFilePath = '$homeLinux/.cache/Google/AndroidStudio2020.3/.home';
        const String studioInstallPath = '$homeLinux/AndroidStudio';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync();

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.javaPath, equals('$studioInstallPath/jre'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds Android Studio 2022.1 bundled Java version',
      () {
        const String studioHomeFilePath = '$homeLinux/.cache/Google/AndroidStudio2022.1/.home';
        const String studioInstallPath = '$homeLinux/AndroidStudio';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync();

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.javaPath, equals('$studioInstallPath/jbr'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'finds bundled Java version when Android Studio version is unknown by assuming the latest version',
      () {
        const String configuredStudioInstallPath = '$homeLinux/AndroidStudio';
        config.setValue('android-studio-dir', configuredStudioInstallPath);

        fileSystem.directory(configuredStudioInstallPath).createSync(recursive: true);

        fileSystem.directory(configuredStudioInstallPath).createSync();

        fileSystem
            .file(fileSystem.path.join(configuredStudioInstallPath, 'jbr', 'bin', 'java'))
            .createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, null);
        expect(studio.javaPath, equals('$configuredStudioInstallPath/jbr'));
      },
      overrides: <Type, Generator>{
        Config: () => config,
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'when given an Android Studio newer than any known version, finds Java version by assuming latest known Android Studio version',
      () {
        const String studioHomeFilePath = '$homeLinux/.cache/Google/AndroidStudio99999.99.99/.home';
        const String studioInstallPath = '$homeLinux/AndroidStudio';

        fileSystem.file(studioHomeFilePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(studioInstallPath);

        fileSystem.directory(studioInstallPath).createSync();

        final String expectedJdkLocationFor2022 = fileSystem.path.join(
          studioInstallPath,
          'jbr',
          'bin',
          'java',
        );
        fileSystem.file(expectedJdkLocationFor2022).createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, equals(Version(99999, 99, 99)));
        expect(studio.javaPath, equals('$studioInstallPath/jbr'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        FileSystemUtils: () => fsUtils,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'pluginsPath extracts custom paths from home dir',
      () {
        const String installPath = '/opt/android-studio-with-cheese-5.0';
        const String studioHome = '$homeLinux/.AndroidStudioWithCheese5.0';
        const String homeFile = '$studioHome/system/.home';
        fileSystem.directory(installPath).createSync(recursive: true);
        fileSystem.file(homeFile).createSync(recursive: true);
        fileSystem.file(homeFile).writeAsStringSync(installPath);

        final AndroidStudio studio = AndroidStudio.fromHomeDot(fileSystem.directory(studioHome))!;
        expect(studio, isNotNull);
        expect(studio.pluginsPath, equals('/home/me/.AndroidStudioWithCheese5.0/config/plugins'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        // Custom home paths are not supported on macOS nor Windows yet,
        // so we force the platform to fake Linux here.
        Platform: () => platform,
        FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: platform),
      },
    );

    testUsingContext(
      'discovers explicitly configured Android Studio',
      () {
        const String androidStudioDir = '/Users/Dash/Desktop/android-studio';
        config.setValue('android-studio-dir', androidStudioDir);
        const String studioHome = '$homeLinux/.cache/Google/AndroidStudio2022.3/.home';
        fileSystem.file(studioHome)
          ..createSync(recursive: true)
          ..writeAsStringSync(androidStudioDir);
        fileSystem.directory(androidStudioDir).createSync(recursive: true);

        final AndroidStudio studio = AndroidStudio.allInstalled().single;

        expect(studio.version, equals(Version(2022, 3, null)));
        expect(studio.configuredPath, androidStudioDir);
        expect(studio.javaPath, fileSystem.path.join(androidStudioDir, 'jbr'));
      },
      overrides: <Type, Generator>{
        Config: () => config,
        Platform: () => platform,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('latestValid', () {
    late Config config;
    late Platform platform;
    late FileSystem fileSystem;

    setUp(() {
      config = Config.test();
      platform = FakePlatform(
        operatingSystem: 'windows',
        environment: <String, String>{'LOCALAPPDATA': r'C:\Users\Dash\AppData\Local'},
      );
      fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    });

    testUsingContext(
      'chooses the install with the latest version',
      () {
        const List<String> versions = <String>['4.0', '2022.0', '3.1'];

        for (final String version in versions) {
          fileSystem.file('C:\\Users\\Dash\\AppData\\Local\\Google\\AndroidStudio$version\\.home')
            ..createSync(recursive: true)
            ..writeAsStringSync('C:\\Program Files\\AndroidStudio$version');
          fileSystem
              .directory('C:\\Program Files\\AndroidStudio$version')
              .createSync(recursive: true);
        }

        expect(AndroidStudio.allInstalled().length, 3);
        expect(AndroidStudio.latestValid()!.version, Version(2022, 0, 0));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'prefers installs with known versions over installs with unknown versions',
      () {
        const List<String> versions = <String>['3.0', 'unknown'];

        for (final String version in versions) {
          fileSystem.file('C:\\Users\\Dash\\AppData\\Local\\Google\\AndroidStudio$version\\.home')
            ..createSync(recursive: true)
            ..writeAsStringSync('C:\\Program Files\\AndroidStudio$version');
          fileSystem
              .directory('C:\\Program Files\\AndroidStudio$version')
              .createSync(recursive: true);
        }

        expect(AndroidStudio.allInstalled().length, 2);
        expect(AndroidStudio.latestValid()!.version, Version(3, 0, 0));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'chooses install with lexicographically greatest directory if no installs have known versions',
      () {
        const List<String> versions = <String>['Apple', 'Zucchini', 'Banana'];

        for (final String version in versions) {
          fileSystem.file('C:\\Users\\Dash\\AppData\\Local\\Google\\AndroidStudio$version\\.home')
            ..createSync(recursive: true)
            ..writeAsStringSync('C:\\Program Files\\AndroidStudio$version');
          fileSystem
              .directory('C:\\Program Files\\AndroidStudio$version')
              .createSync(recursive: true);
        }

        expect(AndroidStudio.allInstalled().length, 3);
        expect(AndroidStudio.latestValid()!.directory, r'C:\Program Files\AndroidStudioZucchini');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'chooses install with lexicographically greatest directory if all installs have the same version',
      () {
        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudioPreview4.0\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudioPreview4.0');
        fileSystem
            .directory(r'C:\Program Files\AndroidStudioPreview4.0')
            .createSync(recursive: true);

        fileSystem.file(r'C:\Users\Dash\AppData\Local\Google\AndroidStudio4.0\.home')
          ..createSync(recursive: true)
          ..writeAsStringSync(r'C:\Program Files\AndroidStudio4.0');
        fileSystem.directory(r'C:\Program Files\AndroidStudio4.0').createSync(recursive: true);

        expect(AndroidStudio.allInstalled().length, 2);
        expect(AndroidStudio.latestValid()!.directory, contains('Preview'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'always chooses the install configured by --android-studio-dir, even if the install is invalid',
      () {
        const String configuredAndroidStudioDir = r'C:\Users\Dash\Desktop\android-studio';
        config.setValue('android-studio-dir', configuredAndroidStudioDir);

        // The directory exists, but nothing is inside.
        fileSystem.directory(configuredAndroidStudioDir).createSync(recursive: true);
        (globals.processManager as FakeProcessManager).excludedExecutables.add(
          fileSystem.path.join(configuredAndroidStudioDir, 'jbr', 'bin', 'java'),
        );

        const List<String> validVersions = <String>['4.0', '2.0', '3.1'];

        for (final String version in validVersions) {
          fileSystem.file('C:\\Users\\Dash\\AppData\\Local\\Google\\AndroidStudio$version\\.home')
            ..createSync(recursive: true)
            ..writeAsStringSync('C:\\Program Files\\AndroidStudio$version');
          fileSystem
              .directory('C:\\Program Files\\AndroidStudio$version')
              .createSync(recursive: true);
        }

        const List<String> validJavaPaths = <String>[
          r'C:\Program Files\AndroidStudio4.0\jre\bin\java',
          r'C:\Program Files\AndroidStudio2.0\jre\bin\java',
          r'C:\Program Files\AndroidStudio3.1\jre\bin\java',
        ];

        for (final String javaPath in validJavaPaths) {
          (globals.processManager as FakeProcessManager).addCommand(
            FakeCommand(command: <String>[fileSystem.path.join(javaPath), '-version']),
          );
        }

        expect(AndroidStudio.allInstalled().length, 4);

        for (final String javaPath in validJavaPaths) {
          (globals.processManager as FakeProcessManager).addCommand(
            FakeCommand(command: <String>[fileSystem.path.join(javaPath), '-version']),
          );
        }

        final AndroidStudio chosenInstall = AndroidStudio.latestValid()!;
        expect(chosenInstall.directory, configuredAndroidStudioDir);
        expect(chosenInstall.isValid, false);
      },
      overrides: <Type, Generator>{
        Config: () => config,
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.empty(),
      },
    );

    testUsingContext(
      'throws a ToolExit if --android-studio-dir is configured but the directory does not exist',
      () async {
        const String configuredAndroidStudioDir = r'C:\Users\Dash\Desktop\android-studio';
        config.setValue('android-studio-dir', configuredAndroidStudioDir);

        expect(fileSystem.directory(configuredAndroidStudioDir).existsSync(), false);
        expect(
          () => AndroidStudio.latestValid(),
          throwsA(
            (dynamic e) =>
                e is ToolExit &&
                e.message!.startsWith(
                  'Could not find the Android Studio installation at the manually configured path',
                ),
          ),
        );
      },
      overrides: <Type, Generator>{
        Config: () => config,
        FileSystem: () => fileSystem,
        Platform: () => platform,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'handles file system exception when checking for explicitly configured Android Studio install',
      () {
        const String androidStudioDir = '/Users/Dash/Desktop/android-studio';
        config.setValue('android-studio-dir', androidStudioDir);

        expect(
          () => AndroidStudio.latestValid(),
          throwsToolExit(
            message: RegExp(r'[.\s\S]*Could not find[.\s\S]*FileSystemException[.\s\S]*'),
          ),
        );
      },
      overrides: <Type, Generator>{
        Config: () => config,
        Platform: () => platform,
        FileSystem: () => _FakeFileSystem(),
        FileSystemUtils: () => _FakeFsUtils(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });
}

class FakePlistUtils extends Fake implements PlistParser {
  final Map<String, Map<String, Object>> fileContents = <String, Map<String, Object>>{};

  @override
  Map<String, Object> parseFile(String plistFilePath) {
    return fileContents[plistFilePath]!;
  }
}

class _FakeFileSystem extends Fake implements FileSystem {
  @override
  Directory directory(dynamic path) {
    return _NonExistentDirectory();
  }

  @override
  Context get path {
    return MemoryFileSystem.test().path;
  }
}

class _NonExistentDirectory extends Fake implements Directory {
  @override
  bool existsSync() {
    throw const FileSystemException(
      'OS Error: Filename, directory name, or volume label syntax is incorrect.',
    );
  }

  @override
  String get path => '';

  @override
  Directory get parent => _NonExistentDirectory();
}

class _FakeFsUtils extends Fake implements FileSystemUtils {
  @override
  String get homeDirPath => '/home/';
}
