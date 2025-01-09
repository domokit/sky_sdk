// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:platform/platform.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:test/test.dart';

import '../../../packages/flutter_tools/test/src/fake_process_manager.dart';
import '../create_api_docs.dart' as apidocs;
import '../dartdoc_checker.dart';

void main() {
  group('FlutterInformation', () {
    late FakeProcessManager fakeProcessManager;
    late FakePlatform fakePlatform;
    late MemoryFileSystem memoryFileSystem;
    late apidocs.FlutterInformation flutterInformation;

    void setUpWithEnvironment(Map<String, String> environment) {
      fakePlatform = FakePlatform(environment: environment);
      flutterInformation = apidocs.FlutterInformation(
        filesystem: memoryFileSystem,
        processManager: fakeProcessManager,
        platform: fakePlatform,
      );
      apidocs.FlutterInformation.instance = flutterInformation;
    }

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      memoryFileSystem = MemoryFileSystem();
      setUpWithEnvironment(<String, String>{});
    });

    test('getBranchName does not call git if env LUCI_BRANCH provided', () {
      setUpWithEnvironment(<String, String>{'LUCI_BRANCH': branchName});
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );
      expect(apidocs.FlutterInformation.instance.getBranchName(), branchName);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    test('getBranchName calls git if env LUCI_BRANCH not provided', () {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );

      expect(apidocs.FlutterInformation.instance.getBranchName(), branchName);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    test('getBranchName calls git if env LUCI_BRANCH is empty', () {
      setUpWithEnvironment(<String, String>{'LUCI_BRANCH': ''});
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );

      expect(apidocs.FlutterInformation.instance.getBranchName(), branchName);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    test("runPubProcess doesn't use the pub binary", () {
      final Platform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': '/flutter'},
      );
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', '--one', '--two']),
      ]);
      apidocs.FlutterInformation.instance = apidocs.FlutterInformation(
        platform: platform,
        processManager: processManager,
        filesystem: memoryFileSystem,
      );

      apidocs.runPubProcess(
        arguments: <String>['--one', '--two'],
        processManager: processManager,
        filesystem: memoryFileSystem,
      );

      expect(processManager, hasNoRemainingExpectations);
    });

    test('calls out to flutter if FLUTTER_VERSION is not set', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0')));
    });
    test("doesn't call out to flutter if FLUTTER_VERSION is set", () async {
      setUpWithEnvironment(<String, String>{'FLUTTER_VERSION': testVersionInfo});
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0')));
    });
    test('getFlutterRoot calls out to flutter if FLUTTER_ROOT is not set', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      );
      fakeProcessManager.addCommand(
        const FakeCommand(command: <Pattern>['git', 'rev-parse', 'HEAD']),
      );
      final Directory root = flutterInformation.getFlutterRoot();
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(root.path, equals('/home/user/flutter'));
    });
    test("getFlutterRoot doesn't call out to flutter if FLUTTER_ROOT is set", () async {
      setUpWithEnvironment(<String, String>{'FLUTTER_ROOT': '/home/user/flutter'});
      final Directory root = flutterInformation.getFlutterRoot();
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(root.path, equals('/home/user/flutter'));
    });
    test('parses version properly', () async {
      fakePlatform.environment['FLUTTER_VERSION'] = testVersionInfo;
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD']),
      ]);
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(info['frameworkVersion'], isNotNull);
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0')));
      expect(info['dartSdkVersion'], isNotNull);
      expect(info['dartSdkVersion'], equals(Version.parse('2.14.0-360.0.dev')));
    });
    test('the engine realm is read from the engine.realm file', () async {
      final Directory flutterHome = memoryFileSystem
          .directory('/home')
          .childDirectory('user')
          .childDirectory('flutter')
          .childDirectory('bin')
          .childDirectory('internal');
      flutterHome.childFile('engine.realm')
        ..createSync(recursive: true)
        ..writeAsStringSync('realm');
      setUpWithEnvironment(<String, String>{'FLUTTER_ROOT': '/home/user/flutter'});
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <Pattern>['/home/user/flutter/bin/flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD']),
      ]);
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(info['engineRealm'], equals('realm'));
    });
  });

  group('Configurator', () {
    late MemoryFileSystem fs;
    late FakeProcessManager fakeProcessManager;
    late Directory publishRoot;
    late Directory packageRoot;
    late Directory docsRoot;
    late File searchTemplate;
    late apidocs.Configurator configurator;
    late FakePlatform fakePlatform;
    late apidocs.FlutterInformation flutterInformation;

    void setUpWithEnvironment(Map<String, String> environment) {
      fakePlatform = FakePlatform(environment: environment);
      flutterInformation = apidocs.FlutterInformation(
        filesystem: fs,
        processManager: fakeProcessManager,
        platform: fakePlatform,
      );
      apidocs.FlutterInformation.instance = flutterInformation;
    }

    setUp(() {
      fs = MemoryFileSystem.test();
      publishRoot = fs.directory('/path/to/publish');
      packageRoot = fs.directory('/path/to/package');
      docsRoot = fs.directory('/path/to/docs');
      searchTemplate = docsRoot.childDirectory('lib').childFile('opensearch.xml');
      fs.directory('/home/user/flutter/packages').createSync(recursive: true);
      fakeProcessManager = FakeProcessManager.empty();
      setUpWithEnvironment(<String, String>{});
      publishRoot.createSync(recursive: true);
      packageRoot.createSync(recursive: true);
      docsRoot.createSync(recursive: true);
      final List<String> files = <String>[
        'README.md',
        'analysis_options.yaml',
        'dartdoc_options.yaml',
        searchTemplate.path,
        publishRoot.childFile('opensearch.xml').path,
      ];
      for (final String file in files) {
        docsRoot.childFile(file).createSync(recursive: true);
      }
      searchTemplate.writeAsStringSync('{SITE_URL}');
      configurator = apidocs.Configurator(
        docsRoot: docsRoot,
        packageRoot: packageRoot,
        publishRoot: publishRoot,
        filesystem: fs,
        processManager: fakeProcessManager,
        platform: fakePlatform,
      );
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>['flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD']),
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', 'global', 'list']),
        FakeCommand(
          command: <Pattern>[
            '/flutter/bin/flutter',
            'pub',
            'global',
            'run',
            '--enable-asserts',
            'dartdoc',
            '--output',
            '/path/to/publish/flutter',
            '--allow-tools',
            '--json',
            '--validate-links',
            '--link-to-source-excludes',
            '/flutter/bin/cache',
            '--link-to-source-root',
            '/flutter',
            '--link-to-source-uri-template',
            'https://github.com/flutter/flutter/blob/main/%f%#L%l%',
            '--inject-html',
            '--use-base-href',
            '--header',
            '/path/to/docs/styles.html',
            '--header',
            '/path/to/docs/analytics-header.html',
            '--header',
            '/path/to/docs/survey.html',
            '--header',
            '/path/to/docs/snippets.html',
            '--header',
            '/path/to/docs/opensearch.html',
            '--footer',
            '/path/to/docs/analytics-footer.html',
            '--footer-text',
            '/path/to/package/footer.html',
            '--allow-warnings-in-packages',
            // match package names
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude-packages',
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude',
            // match dart package URIs
            RegExp(r'^([\w\/:.]+,)+([\w\/:.]+)$'),
            '--favicon',
            '/path/to/docs/favicon.ico',
            '--package-order',
            'flutter,Dart,${apidocs.kPlatformIntegrationPackageName},flutter_test,flutter_driver',
            '--auto-include-dependencies',
          ],
        ),
      ]);
    });

    test('.generateConfiguration generates pubspec.yaml', () async {
      configurator.generateConfiguration();
      expect(packageRoot.childFile('pubspec.yaml').existsSync(), isTrue);
      expect(packageRoot.childFile('pubspec.yaml').readAsStringSync(), contains('flutter_gpu:'));
      expect(
        packageRoot.childFile('pubspec.yaml').readAsStringSync(),
        contains('dependency_overrides:'),
      );
      expect(
        packageRoot.childFile('pubspec.yaml').readAsStringSync(),
        contains('platform_integration:'),
      );
    });

    test('.generateConfiguration generates fake lib', () async {
      configurator.generateConfiguration();
      expect(packageRoot.childDirectory('lib').existsSync(), isTrue);
      expect(packageRoot.childDirectory('lib').childFile('temp_doc.dart').existsSync(), isTrue);
      expect(
        packageRoot.childDirectory('lib').childFile('temp_doc.dart').readAsStringSync(),
        contains('library temp_doc;'),
      );
      expect(
        packageRoot.childDirectory('lib').childFile('temp_doc.dart').readAsStringSync(),
        contains("import 'package:flutter_gpu/gpu.dart';"),
      );
    });

    test('.generateConfiguration generates page footer', () async {
      configurator.generateConfiguration();
      expect(packageRoot.childFile('footer.html').existsSync(), isTrue);
      expect(
        packageRoot.childFile('footer.html').readAsStringSync(),
        contains('<script src="footer.js">'),
      );
      expect(publishRoot.childDirectory('flutter').childFile('footer.js').existsSync(), isTrue);
      expect(
        publishRoot.childDirectory('flutter').childFile('footer.js').readAsStringSync(),
        contains(RegExp(r'Flutter 2.5.0 •.*• stable')),
      );
    });

    test('.generateConfiguration generates search metadata', () async {
      configurator.generateConfiguration();
      expect(publishRoot.childFile('opensearch.xml').existsSync(), isTrue);
      expect(
        publishRoot.childFile('opensearch.xml').readAsStringSync(),
        contains('https://api.flutter.dev/'),
      );
    });
  });

  group('DartDocGenerator', () {
    late apidocs.DartdocGenerator generator;
    late MemoryFileSystem fs;
    late FakeProcessManager processManager;
    late Directory publishRoot;

    setUp(() {
      fs = MemoryFileSystem.test();
      publishRoot = fs.directory('/path/to/publish');
      processManager = FakeProcessManager.empty();
      generator = apidocs.DartdocGenerator(
        packageRoot: fs.directory('/path/to/package'),
        publishRoot: publishRoot,
        docsRoot: fs.directory('/path/to/docs'),
        filesystem: fs,
        processManager: processManager,
      );
      final Directory repoRoot = fs.directory('/flutter');
      repoRoot.childDirectory('packages').createSync(recursive: true);
      apidocs.FlutterInformation.instance = apidocs.FlutterInformation(
        filesystem: fs,
        processManager: processManager,
        platform: FakePlatform(environment: <String, String>{'FLUTTER_ROOT': repoRoot.path}),
      );
    });

    test('.generateDartDoc() invokes dartdoc with the correct command line arguments', () async {
      processManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', 'get']),
        const FakeCommand(
          command: <String>['/flutter/bin/flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD']),
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', 'global', 'list']),
        FakeCommand(
          command: <Pattern>[
            '/flutter/bin/flutter',
            'pub',
            'global',
            'run',
            '--enable-asserts',
            'dartdoc',
            '--output',
            '/path/to/publish/flutter',
            '--allow-tools',
            '--json',
            '--validate-links',
            '--link-to-source-excludes',
            '/flutter/bin/cache',
            '--link-to-source-root',
            '/flutter',
            '--link-to-source-uri-template',
            'https://github.com/flutter/flutter/blob/main/%f%#L%l%',
            '--inject-html',
            '--use-base-href',
            '--header',
            '/path/to/docs/styles.html',
            '--header',
            '/path/to/docs/analytics-header.html',
            '--header',
            '/path/to/docs/survey.html',
            '--header',
            '/path/to/docs/snippets.html',
            '--header',
            '/path/to/docs/opensearch.html',
            '--footer',
            '/path/to/docs/analytics-footer.html',
            '--footer-text',
            '/path/to/package/footer.html',
            '--allow-warnings-in-packages',
            // match package names
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude-packages',
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude',
            // match dart package URIs
            RegExp(r'^([\w\/:.]+,)+([\w\/:.]+)$'),
            '--favicon',
            '/path/to/docs/favicon.ico',
            '--package-order',
            'flutter,Dart,${apidocs.kPlatformIntegrationPackageName},flutter_test,flutter_driver',
            '--auto-include-dependencies',
          ],
        ),
      ]);

      // This will throw while sanity checking generated files, which is tested independently
      await expectLater(
        () => generator.generateDartdoc(),
        throwsA(
          isA<Exception>().having(
            (Exception e) => e.toString(),
            'message',
            contains(
              RegExp(
                r'Missing .* which probably means the documentation failed to build correctly.',
              ),
            ),
          ),
        ),
      );

      expect(processManager, hasNoRemainingExpectations);
    });

    test('sanity checks spot check generated files', () async {
      processManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', 'get']),
        const FakeCommand(
          command: <String>['/flutter/bin/flutter', '--version', '--machine'],
          stdout: testVersionInfo,
        ),
        const FakeCommand(
          command: <Pattern>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD']),
        const FakeCommand(command: <String>['/flutter/bin/flutter', 'pub', 'global', 'list']),
        FakeCommand(
          command: <Pattern>[
            '/flutter/bin/flutter',
            'pub',
            'global',
            'run',
            '--enable-asserts',
            'dartdoc',
            '--output',
            '/path/to/publish/flutter',
            '--allow-tools',
            '--json',
            '--validate-links',
            '--link-to-source-excludes',
            '/flutter/bin/cache',
            '--link-to-source-root',
            '/flutter',
            '--link-to-source-uri-template',
            'https://github.com/flutter/flutter/blob/main/%f%#L%l%',
            '--inject-html',
            '--use-base-href',
            '--header',
            '/path/to/docs/styles.html',
            '--header',
            '/path/to/docs/analytics-header.html',
            '--header',
            '/path/to/docs/survey.html',
            '--header',
            '/path/to/docs/snippets.html',
            '--header',
            '/path/to/docs/opensearch.html',
            '--footer',
            '/path/to/docs/analytics-footer.html',
            '--footer-text',
            '/path/to/package/footer.html',
            '--allow-warnings-in-packages',
            // match package names
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude-packages',
            RegExp(r'^(\w+,)+(\w+)$'),
            '--exclude',
            // match dart package URIs
            RegExp(r'^([\w\/:.]+,)+([\w\/:.]+)$'),
            '--favicon',
            '/path/to/docs/favicon.ico',
            '--package-order',
            'flutter,Dart,${apidocs.kPlatformIntegrationPackageName},flutter_test,flutter_driver',
            '--auto-include-dependencies',
          ],
          onRun: (_) {
            for (final File canary in generator.canaries) {
              canary.createSync(recursive: true);
            }
            for (final String path in dartdocDirectiveCanaryFiles) {
              publishRoot.childDirectory('flutter').childFile(path).createSync(recursive: true);
            }
            for (final String path in dartdocDirectiveCanaryLibraries) {
              publishRoot
                  .childDirectory('flutter')
                  .childDirectory(path)
                  .createSync(recursive: true);
            }
            publishRoot.childDirectory('flutter').childFile('index.html').createSync();

            final Directory widgetsDir = publishRoot
              .childDirectory('flutter')
              .childDirectory('widgets')..createSync(recursive: true);
            widgetsDir.childFile('showGeneralDialog.html').writeAsStringSync('''
<pre id="longSnippet1">
  <code class="language-dart">
    import &#39;package:flutter&#47;material.dart&#39;;
  </code>
</pre>
''');
            expect(publishRoot.childDirectory('flutter').existsSync(), isTrue);
            (widgetsDir.childDirectory('ModalRoute')
              ..createSync(recursive: true)).childFile('barrierColor.html').writeAsStringSync('''
<pre id="sample-code">
  <code class="language-dart">
    class FooClass {
      Color get barrierColor => FooColor();
    }
  </code>
</pre>
''');
            const String queryParams =
                'split=1&run=true&sample_id=widgets.Listener.123&channel=main';
            widgetsDir.childFile('Listener-class.html').writeAsStringSync('''
<iframe class="snippet-dartpad" src="https://dartpad.dev/embed-flutter.html?$queryParams">
</iframe>
''');
          },
        ),
      ]);

      await generator.generateDartdoc();
    });
  });
}

const String branchName = 'stable';
const String testVersionInfo = '''
{
  "frameworkVersion": "2.5.0",
  "channel": "$branchName",
  "repositoryUrl": "git@github.com:flutter/flutter.git",
  "frameworkRevision": "0000000000000000000000000000000000000000",
  "frameworkCommitDate": "2021-07-28 13:03:40 -0700",
  "engineRevision": "0000000000000000000000000000000000000001",
  "dartSdkVersion": "2.14.0 (build 2.14.0-360.0.dev)",
  "flutterRoot": "/home/user/flutter"
}
''';
