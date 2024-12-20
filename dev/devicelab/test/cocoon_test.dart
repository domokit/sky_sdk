// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_devicelab/framework/cocoon.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:http/http.dart';
import 'package:http/testing.dart';

import 'common.dart';

void main() {
  late ProcessResult processResult;
  ProcessResult runSyncStub(
    String executable,
    List<String> args, {
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stderrEncoding,
    Encoding? stdoutEncoding,
    String? workingDirectory,
  }) => processResult;

  // Expected test values.
  const String commitSha = 'a4952838bf288a81d8ea11edfd4b4cd649fa94cc';
  const String serviceAccountTokenPath = 'test_account_file';
  const String serviceAccountToken = 'test_token';

  group('Cocoon', () {
    late Client mockClient;
    late Cocoon cocoon;
    late FileSystem fs;

    setUp(() {
      fs = MemoryFileSystem();
      mockClient = MockClient((Request request) async => Response('{}', 200));

      final File serviceAccountFile = fs.file(serviceAccountTokenPath)..createSync();
      serviceAccountFile.writeAsStringSync(serviceAccountToken);
    });

    test('returns expected commit sha', () {
      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        serviceAccountTokenPath: serviceAccountTokenPath,
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
      );

      expect(cocoon.commitSha, commitSha);
    });

    test('throws exception on git cli errors', () {
      processResult = ProcessResult(1, 1, '', '');
      cocoon = Cocoon(
        serviceAccountTokenPath: serviceAccountTokenPath,
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
      );

      expect(() => cocoon.commitSha, throwsA(isA<CocoonException>()));
    });

    test('writes expected update task json', () async {
      processResult = ProcessResult(1, 0, commitSha, '');
      final TaskResult result = TaskResult.fromJson(<String, dynamic>{
        'success': true,
        'data': <String, dynamic>{'i': 0, 'j': 0, 'not_a_metric': 'something'},
        'benchmarkScoreKeys': <String>['i', 'j'],
      });

      cocoon = Cocoon(fs: fs, processRunSync: runSyncStub);

      const String resultsPath = 'results.json';
      await cocoon.writeTaskResultToFile(
        builderName: 'builderAbc',
        gitBranch: 'master',
        result: result,
        resultsPath: resultsPath,
      );

      final String resultJson = fs.file(resultsPath).readAsStringSync();
      const String expectedJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      expect(resultJson, expectedJson);
    });

    test('uploads metrics sends expected post body', () async {
      processResult = ProcessResult(1, 0, commitSha, '');
      const String uploadMetricsRequestWithSpaces =
          '{"CommitBranch":"master","CommitSha":"a4952838bf288a81d8ea11edfd4b4cd649fa94cc","BuilderName":"builder a b c","NewStatus":"Succeeded","ResultData":{},"BenchmarkScoreKeys":[],"TestFlaky":false}';
      final MockClient client = MockClient((Request request) async {
        if (request.body == uploadMetricsRequestWithSpaces) {
          return Response('{}', 200);
        }

        return Response(
          'Expected: $uploadMetricsRequestWithSpaces\nReceived: ${request.body}',
          500,
        );
      });
      cocoon = Cocoon(
        fs: fs,
        httpClient: client,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builder a b c",' //ignore: missing_whitespace_between_adjacent_strings
          '"NewStatus":"Succeeded",'
          '"ResultData":{},'
          '"BenchmarkScoreKeys":[]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('uploads expected update task payload from results file', () async {
      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('Verify retries for task result upload', () async {
      int requestCount = 0;
      mockClient = MockClient((Request request) async {
        requestCount++;
        if (requestCount == 1) {
          return Response('{}', 500);
        } else {
          return Response('{}', 200);
        }
      });

      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 3,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('Verify timeout and retry for task result upload', () async {
      int requestCount = 0;
      const int timeoutValue = 2;
      mockClient = MockClient((Request request) async {
        requestCount++;
        if (requestCount == 1) {
          await Future<void>.delayed(const Duration(seconds: timeoutValue + 2));
          throw Exception('Should not reach this, because timeout should trigger');
        } else {
          return Response('{}', 200);
        }
      });

      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 2,
        requestTimeoutLimit: timeoutValue,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('Verify timeout does not trigger for result upload', () async {
      int requestCount = 0;
      const int timeoutValue = 2;
      mockClient = MockClient((Request request) async {
        requestCount++;
        if (requestCount == 1) {
          await Future<void>.delayed(const Duration(seconds: timeoutValue - 1));
          return Response('{}', 200);
        } else {
          throw Exception('This iteration should not be reached, since timeout should not happen.');
        }
      });

      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 2,
        requestTimeoutLimit: timeoutValue,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('Verify failure without retries for task result upload', () async {
      int requestCount = 0;
      mockClient = MockClient((Request request) async {
        requestCount++;
        if (requestCount == 1) {
          return Response('{}', 500);
        } else {
          return Response('{}', 200);
        }
      });

      processResult = ProcessResult(1, 0, commitSha, '');
      cocoon = Cocoon(
        fs: fs,
        httpClient: mockClient,
        processRunSync: runSyncStub,
        serviceAccountTokenPath: serviceAccountTokenPath,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      expect(
        () => cocoon.sendTaskStatus(resultsPath: resultsPath),
        throwsA(isA<ClientException>()),
      );
    });

    test('throws client exception on non-200 responses', () async {
      mockClient = MockClient((Request request) async => Response('', 500));

      cocoon = Cocoon(
        serviceAccountTokenPath: serviceAccountTokenPath,
        fs: fs,
        httpClient: mockClient,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);
      expect(
        () => cocoon.sendTaskStatus(resultsPath: resultsPath),
        throwsA(isA<ClientException>()),
      );
    });

    test('does not upload results on non-supported branches', () async {
      // Any network failure would cause the upload to fail
      mockClient = MockClient((Request request) async => Response('', 500));

      cocoon = Cocoon(
        serviceAccountTokenPath: serviceAccountTokenPath,
        fs: fs,
        httpClient: mockClient,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"stable",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);

      // This will fail if it decided to upload results
      await cocoon.sendTaskStatus(resultsPath: resultsPath);
    });

    test('does not update for staging test', () async {
      // Any network failure would cause the upload to fail
      mockClient = MockClient((Request request) async => Response('', 500));

      cocoon = Cocoon(
        serviceAccountTokenPath: serviceAccountTokenPath,
        fs: fs,
        httpClient: mockClient,
        requestRetryLimit: 0,
      );

      const String resultsPath = 'results.json';
      const String updateTaskJson =
          '{'
          '"CommitBranch":"master",'
          '"CommitSha":"$commitSha",'
          '"BuilderName":"builderAbc",'
          '"NewStatus":"Succeeded",'
          '"ResultData":{"i":0.0,"j":0.0,"not_a_metric":"something"},'
          '"BenchmarkScoreKeys":["i","j"]}';
      fs.file(resultsPath).writeAsStringSync(updateTaskJson);

      // This will fail if it decided to upload results
      await cocoon.sendTaskStatus(resultsPath: resultsPath, builderBucket: 'staging');
    });
  });

  group('AuthenticatedCocoonClient', () {
    late FileSystem fs;

    setUp(() {
      fs = MemoryFileSystem();
      final File serviceAccountFile = fs.file(serviceAccountTokenPath)..createSync();
      serviceAccountFile.writeAsStringSync(serviceAccountToken);
    });

    test('reads token from service account file', () {
      final AuthenticatedCocoonClient client = AuthenticatedCocoonClient(
        serviceAccountTokenPath,
        filesystem: fs,
      );
      expect(client.serviceAccountToken, serviceAccountToken);
    });

    test('reads token from service account file with whitespace', () {
      final File serviceAccountFile = fs.file(serviceAccountTokenPath)..createSync();
      serviceAccountFile.writeAsStringSync('$serviceAccountToken \n');
      final AuthenticatedCocoonClient client = AuthenticatedCocoonClient(
        serviceAccountTokenPath,
        filesystem: fs,
      );
      expect(client.serviceAccountToken, serviceAccountToken);
    });

    test('throws error when service account file not found', () {
      final AuthenticatedCocoonClient client = AuthenticatedCocoonClient(
        'idontexist',
        filesystem: fs,
      );
      expect(() => client.serviceAccountToken, throwsA(isA<FileSystemException>()));
    });
  });
}
