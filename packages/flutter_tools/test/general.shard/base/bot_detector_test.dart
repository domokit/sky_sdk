// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/bot_detector.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/persistent_tool_state.dart';

import '../../src/common.dart';
import '../../src/fake_http_client.dart';
import '../../src/fakes.dart';

final Uri azureUrl = Uri.parse('http://169.254.169.254/metadata/instance');

void main() {
  group('BotDetector', () {
    late FakePlatform fakePlatform;
    late FakeStdio fakeStdio;
    late PersistentToolState persistentToolState;

    setUp(() {
      fakePlatform = FakePlatform()..environment = <String, String>{};
      fakeStdio = FakeStdio();
      persistentToolState = PersistentToolState.test(
        directory: MemoryFileSystem.test().currentDirectory,
        logger: BufferLogger.test(),
      );
    });

    group('isRunningOnBot', () {
      testWithoutContext('returns false unconditionally if BOT=false is set', () async {
        fakePlatform.environment['BOT'] = 'false';
        fakePlatform.environment['TRAVIS'] = 'true';

        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isFalse);
        expect(persistentToolState.isRunningOnBot, isFalse);
      });

      testWithoutContext('does not cache BOT environment variable', () async {
        fakePlatform.environment['BOT'] = 'true';

        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isTrue);
        expect(persistentToolState.isRunningOnBot, isTrue);

        fakePlatform.environment['BOT'] = 'false';

        expect(await botDetector.isRunningOnBot, isFalse);
        expect(persistentToolState.isRunningOnBot, isFalse);
      });

      testWithoutContext('returns false unconditionally if FLUTTER_HOST is set', () async {
        fakePlatform.environment['FLUTTER_HOST'] = 'foo';
        fakePlatform.environment['TRAVIS'] = 'true';

        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isFalse);
        expect(persistentToolState.isRunningOnBot, isFalse);
      });

      testWithoutContext('returns false with and without a terminal attached', () async {
        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory:
              () => FakeHttpClient.list(<FakeRequest>[
                FakeRequest(
                  azureUrl,
                  responseError: const SocketException('HTTP connection timed out'),
                ),
              ]),
          persistentToolState: persistentToolState,
        );

        fakeStdio.stdout.hasTerminal = true;
        expect(await botDetector.isRunningOnBot, isFalse);
        fakeStdio.stdout.hasTerminal = false;
        expect(await botDetector.isRunningOnBot, isFalse);
        expect(persistentToolState.isRunningOnBot, isFalse);
      });

      testWithoutContext('can test analytics outputs on bots when outputting to a file', () async {
        fakePlatform.environment['TRAVIS'] = 'true';
        fakePlatform.environment['FLUTTER_ANALYTICS_LOG_FILE'] = '/some/file';

        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isFalse);
        expect(persistentToolState.isRunningOnBot, isFalse);
      });

      testWithoutContext('returns true when azure metadata is reachable', () async {
        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isTrue);
        expect(persistentToolState.isRunningOnBot, isTrue);
      });

      testWithoutContext('caches azure bot detection results across instances', () async {
        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isTrue);
        expect(
          await BotDetector(
            platform: fakePlatform,
            httpClientFactory: () => FakeHttpClient.list(<FakeRequest>[]),
            persistentToolState: persistentToolState,
          ).isRunningOnBot,
          isTrue,
        );
      });

      testWithoutContext('returns true when running on borg', () async {
        fakePlatform.environment['BORG_ALLOC_DIR'] = 'true';

        final BotDetector botDetector = BotDetector(
          platform: fakePlatform,
          httpClientFactory: () => FakeHttpClient.any(),
          persistentToolState: persistentToolState,
        );

        expect(await botDetector.isRunningOnBot, isTrue);
        expect(persistentToolState.isRunningOnBot, isTrue);
      });
    });
  });

  group('AzureDetector', () {
    testWithoutContext('isRunningOnAzure returns false when connection times out', () async {
      final AzureDetector azureDetector = AzureDetector(
        httpClientFactory:
            () => FakeHttpClient.list(<FakeRequest>[
              FakeRequest(
                azureUrl,
                responseError: const SocketException('HTTP connection timed out'),
              ),
            ]),
      );

      expect(await azureDetector.isRunningOnAzure, isFalse);
    });

    testWithoutContext('isRunningOnAzure returns false when OsError is thrown', () async {
      final AzureDetector azureDetector = AzureDetector(
        httpClientFactory:
            () => FakeHttpClient.list(<FakeRequest>[
              FakeRequest(azureUrl, responseError: const OSError('Connection Refused', 111)),
            ]),
      );

      expect(await azureDetector.isRunningOnAzure, isFalse);
    });

    testWithoutContext('isRunningOnAzure returns true when azure metadata is reachable', () async {
      final AzureDetector azureDetector = AzureDetector(
        httpClientFactory: () => FakeHttpClient.list(<FakeRequest>[FakeRequest(azureUrl)]),
      );

      expect(await azureDetector.isRunningOnAzure, isTrue);
    });
  });
}
