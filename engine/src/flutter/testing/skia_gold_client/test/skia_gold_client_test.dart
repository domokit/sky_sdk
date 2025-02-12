// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;
import 'dart:typed_data';

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:process_fakes/process_fakes.dart';
import 'package:skia_gold_client/skia_gold_client.dart';
import 'package:skia_gold_client/src/release_version.dart';
import 'package:test/test.dart';

void main() {
  /// A mock commit hash that is used to simulate a successful git call.
  const String mockCommitHash = '1234567890abcdef';

  /// Simulating what a presubmit environment would look like.
  const Map<String, String> presubmitEnv = <String, String>{
    'GIT_BRANCH': 'master',
    'GOLDCTL': 'python tools/goldctl.py',
    'GOLD_TRYJOB': 'flutter/flutter/1234567890',
    'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/1234567890/+/logdog',
    'LUCI_CONTEXT': '{}',
  };

  /// Simulating what a postsubmit environment would look like.
  const Map<String, String> postsubmitEnv = <String, String>{
    'GIT_BRANCH': 'master',
    'GOLDCTL': 'python tools/goldctl.py',
    'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/1234567890/+/logdog',
    'LUCI_CONTEXT': '{}',
  };

  /// Simulating what a local environment would look like.
  const Map<String, String> localEnv = <String, String>{};

  /// Creates a [SkiaGoldClient] with the given [dimensions] and [verbose] flag.
  ///
  /// Optionally, the [onRun] function can be provided to handle the execution
  /// of the command-line tool. If not provided, it throws an
  /// [UnsupportedError] by default.
  ///
  /// Side-effects of the client can be observed through the test fixture.
  SkiaGoldClient createClient(
    _TestFixture fixture, {
    required Map<String, String> environment,
    ReleaseVersion? engineVersion,
    Map<String, String>? dimensions,
    String? prefix,
    bool verbose = false,
    io.ProcessResult Function(FakeCommandLogEntry entry) onRun = _runUnhandled,
  }) {
    return SkiaGoldClient.forTesting(
      fixture.workDirectory,
      dimensions: dimensions,
      engineRoot: Engine.fromSrcPath(fixture.engineSrcDir.path),
      httpClient: fixture.httpClient,
      processManager: FakeProcessManager(onRun: onRun),
      verbose: verbose,
      stderr: fixture.outputSink,
      environment: environment,
      prefix: prefix,
    );
  }

  /// Creates a `temp/auth_opt.json` file in the working directory.
  ///
  /// This simulates what the goldctl tool does when it runs.
  void createAuthOptDotJson(String workDirectory) {
    final io.File authOptDotJson = io.File(p.join(workDirectory, 'temp', 'auth_opt.json'));
    authOptDotJson.createSync(recursive: true);
    authOptDotJson.writeAsStringSync('{"GSUtil": false}');
  }

  test('fails if GOLDCTL is not set', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(fixture, environment: localEnv);
      try {
        await client.auth();
        fail('auth should fail if GOLDCTL is not set');
      } on StateError catch (error) {
        expect('$error', contains('GOLDCTL is not set'));
      }
    } finally {
      fixture.dispose();
    }
  });

  test('prints a warning and skips when the git branch is not master or main', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: {...presubmitEnv, 'GIT_BRANCH': 'merge-queue-foo'},
        onRun: (FakeCommandLogEntry entry) {
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'auth',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--luci',
          ]);
          createAuthOptDotJson(fixture.workDirectory.path);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      // In case we change our mind, auth is still expected to work.
      await client.auth();

      expect(
        fixture.outputSink.toString(),
        stringContainsInOrder([
          'Current git branch',
          'merge-queue-foo',
          'is not "main" or "master"',
        ]),
      );
    } finally {
      fixture.dispose();
    }
  });

  test('always a success when the git branch is not master or main', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: {...presubmitEnv, 'GIT_BRANCH': 'merge-queue-foo'},
        onRun: (FakeCommandLogEntry entry) {
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'auth',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--luci',
          ]);
          createAuthOptDotJson(fixture.workDirectory.path);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      // In case we change our mind, auth is still expected to work.
      await client.auth();

      // Always completes OK.
      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );
    } finally {
      fixture.dispose();
    }
  });

  test('auth executes successfully', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'auth',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--luci',
          ]);
          createAuthOptDotJson(fixture.workDirectory.path);
          return io.ProcessResult(0, 0, '', '');
        },
      );
      await client.auth();
    } finally {
      fixture.dispose();
    }
  });

  test('auth is only invoked once per instance', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      int callsToGoldctl = 0;
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          callsToGoldctl++;
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'auth',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--luci',
          ]);
          createAuthOptDotJson(fixture.workDirectory.path);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      await client.auth();
      await client.auth();
      expect(callsToGoldctl, 1);
    } finally {
      fixture.dispose();
    }
  });

  test('auth executes successfully with verbose logging', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        verbose: true,
        onRun: (FakeCommandLogEntry entry) {
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'auth',
            '--verbose',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--luci',
          ]);
          return io.ProcessResult(0, 0, 'stdout', 'stderr');
        },
      );

      await client.auth();
      expect(fixture.outputSink.toString(), contains('stdout:\nstdout'));
      expect(fixture.outputSink.toString(), contains('stderr:\nstderr'));
    } finally {
      fixture.dispose();
    }
  });

  test('auth fails', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          return io.ProcessResult(1, 0, 'stdout-text', 'stderr-text');
        },
      );

      try {
        await client.auth();
      } on SkiaGoldProcessError catch (error) {
        expect(error.command, contains('auth'));
        expect(error.stdout, 'stdout-text');
        expect(error.stderr, 'stderr-text');
        expect(error.message, contains('Skia Gold authorization failed'));
      }
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [pre-submit] executes successfully', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );
    } finally {
      fixture.dispose();
    }
  });

  test('addImg uses prefix, if specified', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        prefix: 'engine.',
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'engine.test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [pre-submit] executes successfully with a release version', () async {
    // Adds a suffix of "_Release_3_21" to the test name.
    final _TestFixture fixture = _TestFixture(
      // Creates a file called "engine/src/fluter/.engine-release.version" with the contents "3.21".
      engineVersion: ReleaseVersion(major: 3, minor: 21),
    );
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            // This is the significant change.
            'test-name_Release_3_21',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [pre-submit] executes successfully with verbose logging', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        verbose: true,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--verbose',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, 'stdout', 'stderr');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );

      expect(fixture.outputSink.toString(), contains('stdout:\nstdout'));
      expect(fixture.outputSink.toString(), contains('stderr:\nstderr'));
    } finally {
      fixture.dispose();
    }
  });

  // A success case (exit code 0) with a message of "Untriaged" is OK.
  test('addImg [pre-submit] succeeds but has an untriaged image', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          // Intentionally returning a non-zero exit code.
          return io.ProcessResult(0, 1, 'Untriaged', '');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );

      // Expect a stderr log message.
      final String log = fixture.outputSink.toString();
      expect(log, contains('Untriaged image detected'));
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [pre-submit] fails due to an unexpected error', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          return io.ProcessResult(1, 0, 'stdout-text', 'stderr-text');
        },
      );

      try {
        await client.addImg(
          'test-name.foo',
          io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
          screenshotSize: 1000,
        );
      } on SkiaGoldProcessError catch (error) {
        expect(error.message, contains('Skia Gold image test failed.'));
        expect(error.stdout, 'stdout-text');
        expect(error.stderr, 'stderr-text');
        expect(error.command, contains('imgtest add'));
      }
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [post-submit] executes successfully', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: postsubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--passfail',
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, '', '');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [post-submit] executes successfully with verbose logging', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: postsubmitEnv,
        verbose: true,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'add',
            '--verbose',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
            '--png-file',
            p.join(fixture.workDirectory.path, 'temp', 'golden.png'),
            '--passfail',
            '--add-test-optional-key',
            'image_matching_algorithm:fuzzy',
            '--add-test-optional-key',
            'fuzzy_max_different_pixels:10',
            '--add-test-optional-key',
            'fuzzy_pixel_delta_threshold:0',
          ]);
          return io.ProcessResult(0, 0, 'stdout', 'stderr');
        },
      );

      await client.addImg(
        'test-name.foo',
        io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
        screenshotSize: 1000,
      );

      expect(fixture.outputSink.toString(), contains('stdout:\nstdout'));
      expect(fixture.outputSink.toString(), contains('stderr:\nstderr'));
    } finally {
      fixture.dispose();
    }
  });

  test('addImg [post-submit] fails due to an unapproved image', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: postsubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          if (entry.command case ['git', ...]) {
            return io.ProcessResult(0, 0, mockCommitHash, '');
          }
          if (entry.command case ['python tools/goldctl.py', 'imgtest', 'init', ...]) {
            return io.ProcessResult(0, 0, '', '');
          }
          return io.ProcessResult(1, 0, 'stdout-text', 'stderr-text');
        },
      );

      try {
        await client.addImg(
          'test-name.foo',
          io.File(p.join(fixture.workDirectory.path, 'temp', 'golden.png')),
          screenshotSize: 1000,
        );
      } on SkiaGoldProcessError catch (error) {
        expect(error.message, contains('Skia Gold image test failed.'));
        expect(error.stdout, 'stdout-text');
        expect(error.stderr, 'stderr-text');
        expect(error.command, contains('imgtest add'));
      }
    } finally {
      fixture.dispose();
    }
  });

  test('getExpectationsForTest returns the latest positive digest', () async {
    final _TestFixture fixture = _TestFixture();
    try {
      final SkiaGoldClient client = createClient(
        fixture,
        environment: presubmitEnv,
        onRun: (FakeCommandLogEntry entry) {
          expect(entry.command, <String>[
            'python tools/goldctl.py',
            'imgtest',
            'get',
            '--work-dir',
            p.join(fixture.workDirectory.path, 'temp'),
            '--test-name',
            'test-name',
          ]);
          return io.ProcessResult(0, 0, '{"digest":"digest"}', '');
        },
      );

      final String hash = client.getTraceID('test-name');
      fixture.httpClient.setJsonResponse(
        Uri.parse('https://flutter-gold.skia.org/json/v2/latestpositivedigest/$hash'),
        <String, Object?>{'digest': 'digest'},
      );

      final String? digest = await client.getExpectationForTest('test-name');
      expect(digest, 'digest');
    } finally {
      fixture.dispose();
    }
  });
}

final class _TestFixture {
  _TestFixture({ReleaseVersion? engineVersion}) {
    workDirectory = rootDirectory.createTempSync('working');

    // Create the engine/src directory.
    engineSrcDir = io.Directory(p.join(rootDirectory.path, 'engine', 'src'));
    engineSrcDir.createSync(recursive: true);

    // Create a .engine-release.version file in the engine root.
    final io.Directory flutterDir = io.Directory(p.join(engineSrcDir.path, 'flutter'));
    flutterDir.createSync(recursive: true);

    final String version = engineVersion?.toString() ?? 'none';
    io.File(p.join(flutterDir.path, '.engine-release.version')).writeAsStringSync(version);
  }

  final io.Directory rootDirectory = io.Directory.systemTemp.createTempSync(
    'skia_gold_client_test',
  );
  late final io.Directory workDirectory;
  late final io.Directory engineSrcDir;

  final _FakeHttpClient httpClient = _FakeHttpClient();
  final StringSink outputSink = StringBuffer();

  void dispose() {
    rootDirectory.deleteSync(recursive: true);
  }
}

io.ProcessResult _runUnhandled(FakeCommandLogEntry entry) {
  throw UnimplementedError('Unhandled run: ${entry.command.join(' ')}');
}

/// An  in-memory fake of [io.HttpClient] that allows [getUrl] to be mocked.
///
/// This class is used to simulate a response from the server.
///
/// Any other methods called on this class will throw a [NoSuchMethodError].
final class _FakeHttpClient implements io.HttpClient {
  final Map<Uri, Object?> _expectedResponses = <Uri, Object?>{};

  /// Sets an expected response for the given [request] to [jsonEncodableValue].
  ///
  /// This method is used to simulate a response from the server.
  void setJsonResponse(Uri request, Object? jsonEncodableValue) {
    _expectedResponses[request] = jsonEncodableValue;
  }

  @override
  Future<io.HttpClientRequest> getUrl(Uri url) async {
    final Object? response = _expectedResponses[url];
    if (response == null) {
      throw StateError('No request expected for $url');
    }
    return _FakeHttpClientRequest.withJsonResponse(response);
  }

  @override
  Object? noSuchMethod(Invocation invocation) {
    return super.noSuchMethod(invocation);
  }
}

final class _FakeHttpClientRequest implements io.HttpClientRequest {
  factory _FakeHttpClientRequest.withJsonResponse(Object? jsonResponse) {
    final Uint8List bytes = utf8.encoder.convert(jsonEncode(jsonResponse));
    return _FakeHttpClientRequest._(_FakeHttpClientResponse(bytes));
  }

  _FakeHttpClientRequest._(this._response);

  final io.HttpClientResponse _response;

  @override
  Future<io.HttpClientResponse> close() async {
    return _response;
  }

  @override
  Object? noSuchMethod(Invocation invocation) {
    return super.noSuchMethod(invocation);
  }
}

final class _FakeHttpClientResponse extends Stream<List<int>> implements io.HttpClientResponse {
  _FakeHttpClientResponse(this._bytes);

  final Uint8List _bytes;

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int> event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return Stream<List<int>>.fromIterable(<List<int>>[
      _bytes,
    ]).listen(onData, onError: onError, onDone: onDone, cancelOnError: cancelOnError);
  }

  @override
  int get statusCode => 200;

  @override
  Object? noSuchMethod(Invocation invocation) {
    return super.noSuchMethod(invocation);
  }
}
