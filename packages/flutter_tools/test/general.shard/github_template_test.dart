// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/github_template.dart';

import '../src/common.dart';
import '../src/context.dart';

void main() {
  late BufferLogger logger;
  late FileSystem fs;

  setUp(() {
    logger = BufferLogger.test();
    fs = MemoryFileSystem.test();
  });

  group('GitHub template creator', () {
    testWithoutContext('similar issues URL', () {
      expect(
        GitHubTemplateCreator.toolCrashSimilarIssuesURL('this is a 100% error'),
        'https://github.com/flutter/flutter/issues?q=is%3Aissue+this+is+a+100%25+error',
      );
    });

    group('sanitized error message', () {
      testWithoutContext('ProcessException', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            const ProcessException('cd', <String>['path/to/something']),
          ),
          'ProcessException:  Command: cd, OS error code: 0',
        );
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            const ProcessException('cd', <String>['path/to/something'], 'message'),
          ),
          'ProcessException: message Command: cd, OS error code: 0',
        );
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            const ProcessException('cd', <String>['path/to/something'], 'message', -19),
          ),
          'ProcessException: message Command: cd, OS error code: -19',
        );
      });

      testWithoutContext('FileSystemException', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            const FileSystemException('delete failed', 'path/to/something'),
          ),
          'FileSystemException: delete failed, null',
        );
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            const FileSystemException(
              'delete failed',
              'path/to/something',
              OSError('message', -19),
            ),
          ),
          'FileSystemException: delete failed, OS Error: message, errno = -19',
        );
      });

      testWithoutContext('SocketException', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            SocketException(
              'message',
              osError: const OSError('message', -19),
              address: InternetAddress.anyIPv6,
              port: 2000,
            ),
          ),
          'SocketException: message, OS Error: message, errno = -19',
        );
      });

      testWithoutContext('DevFSException', () {
        final StackTrace stackTrace = StackTrace.fromString('''
#0      _File.open.<anonymous closure> (dart:io/file_impl.dart:366:9)
#1      _rootRunUnary (dart:async/zone.dart:1141:38)''');
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            DevFSException('message', ArgumentError('argument error message'), stackTrace),
          ),
          'DevFSException: message',
        );
      });

      testWithoutContext('ArgumentError', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(ArgumentError('argument error message')),
          'ArgumentError: Invalid argument(s): argument error message',
        );
      });

      testWithoutContext('Error', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(FakeError()),
          'FakeError: (#0      _File.open.<anonymous closure> (dart:io/file_impl.dart:366:9))',
        );
      });

      testWithoutContext('String', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            'May have non-tool-internal info, very long string, 0b8abb4724aa590dd0f429683339b' // ignore: missing_whitespace_between_adjacent_strings
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb'
            '24aa590dd0f429683339b1e045a1594d0b8abb4724aa590dd0f429683339b1e045a1594d0b8abb',
          ),
          'String: <1,016 characters>',
        );
      });

      testWithoutContext('Exception', () {
        expect(
          GitHubTemplateCreator.sanitizedCrashException(
            Exception('May have non-tool-internal info'),
          ),
          '_Exception',
        );
      });
    });

    group('new issue template URL', () {
      late StackTrace stackTrace;
      late Error error;
      const String command = 'flutter test';
      const String doctorText = ' [✓] Flutter (Channel report';

      setUp(() async {
        stackTrace = StackTrace.fromString('trace');
        error = ArgumentError('argument error message');
      });

      testUsingContext(
        'shows GitHub issue URL',
        () async {
          final GitHubTemplateCreator creator = GitHubTemplateCreator(
            fileSystem: fs,
            logger: logger,
            flutterProjectFactory: FlutterProjectFactory(fileSystem: fs, logger: logger),
          );
          expect(
            await creator.toolCrashIssueTemplateGitHubURL(command, error, stackTrace, doctorText),
            'https://github.com/flutter/flutter/issues/new?title=%5Btool_crash%5D+ArgumentError%3A+'
            'Invalid+argument%28s%29%3A+argument+error+message&body=%23%23+Command%0A%60%60%60sh%0A'
            'flutter+test%0A%60%60%60%0A%0A%23%23+Steps+to+Reproduce%0A1.+...%0A2.+...%0A3.+...%0'
            'A%0A%23%23+Logs%0AArgumentError%3A+Invalid+argument%28s%29%3A+argument+error+message'
            '%0A%60%60%60console%0Atrace%0A%60%60%60%0A%60%60%60console%0A+%5B%E2%9C%93%5D+Flutter+%28Channel+r'
            'eport%0A%60%60%60%0A%0A%23%23+Flutter+Application+Metadata%0ANo+pubspec+in+working+d'
            'irectory.%0A&labels=tool%2Csevere%3A+crash',
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'app metadata',
        () async {
          final GitHubTemplateCreator creator = GitHubTemplateCreator(
            fileSystem: fs,
            logger: logger,
            flutterProjectFactory: FlutterProjectFactory(fileSystem: fs, logger: logger),
          );
          final Directory projectDirectory = fs.currentDirectory;

          projectDirectory.childFile('pubspec.yaml').writeAsStringSync('''
name: failing_app
version: 2.0.1+100
flutter:
  uses-material-design: true
  module:
    androidX: true
    androidPackage: com.example.failing.android
    iosBundleIdentifier: com.example.failing.ios
''');

          final File pluginsFile = projectDirectory.childFile('.flutter-plugins');
          pluginsFile.writeAsStringSync('''
camera=/fake/pub.dartlang.org/camera-0.5.7+2/
device_info=/fake/pub.dartlang.org/pub.dartlang.org/device_info-0.4.1+4/
        ''');

          final File metadataFile = projectDirectory.childFile('.metadata');
          metadataFile.writeAsStringSync('''
version:
  revision: 0b8abb4724aa590dd0f429683339b1e045a1594d
  channel: stable

project_type: app
        ''');

          final String actualURL = await creator.toolCrashIssueTemplateGitHubURL(
            command,
            error,
            stackTrace,
            doctorText,
          );
          final String? actualBody = Uri.parse(actualURL).queryParameters['body'];
          const String expectedBody = '''
## Command
```sh
flutter test
```

## Steps to Reproduce
1. ...
2. ...
3. ...

## Logs
ArgumentError: Invalid argument(s): argument error message
```console
trace
```
```console
 [✓] Flutter (Channel report
```

## Flutter Application Metadata
**Type**: app
**Version**: 2.0.1+100
**Material**: true
**Android X**: true
**Module**: true
**Plugin**: false
**Android package**: com.example.failing.android
**iOS bundle identifier**: com.example.failing.ios
**Creation channel**: stable
**Creation framework version**: 0b8abb4724aa590dd0f429683339b1e045a1594d
### Plugins
camera-0.5.7+2
device_info-0.4.1+4

''';

          expect(actualBody, expectedBody);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );
    });
  });
}

class FakeError extends Error {
  @override
  StackTrace get stackTrace => StackTrace.fromString('''
#0      _File.open.<anonymous closure> (dart:io/file_impl.dart:366:9)
#1      _rootRunUnary (dart:async/zone.dart:1141:38)''');

  @override
  String toString() => 'PII to ignore';
}
