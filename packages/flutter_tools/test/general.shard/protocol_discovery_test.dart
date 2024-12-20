// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/protocol_discovery.dart';

import '../src/common.dart';
import '../src/fake_devices.dart';

void main() {
  group('service_protocol discovery', () {
    late FakeDeviceLogReader logReader;
    late ProtocolDiscovery discoverer;

    setUp(() {
      logReader = FakeDeviceLogReader();
      discoverer = ProtocolDiscovery.vmService(
        logReader,
        ipv6: false,
        throttleDuration: const Duration(milliseconds: 5),
        logger: BufferLogger.test(),
      );
    });

    testWithoutContext('returns non-null uri future', () async {
      expect(discoverer.uri, isNotNull);
    });

    group('no port forwarding', () {
      tearDown(() {
        discoverer.cancel();
        logReader.dispose();
      });

      testWithoutContext('discovers uri if logs already produced output', () async {
        logReader.addLine('HELLO WORLD');
        logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');
        final Uri uri = (await discoverer.uri)!;
        expect(uri.port, 9999);
        expect('$uri', 'http://127.0.0.1:9999');
      });

      testWithoutContext('does not discover uri with no host', () async {
        final Future<Uri?> pendingUri = discoverer.uri;
        logReader.addLine('The Dart VM service is listening on http12asdasdsd9999');
        await Future<void>.delayed(const Duration(milliseconds: 10));
        logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');

        await Future<void>.delayed(Duration.zero);

        final Uri uri = (await pendingUri)!;
        expect(uri, isNotNull);
        expect(uri.port, 9999);
        expect('$uri', 'http://127.0.0.1:9999');
      });

      testWithoutContext(
        'discovers uri if logs already produced output and no listener is attached',
        () async {
          logReader.addLine('HELLO WORLD');
          logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');

          await Future<void>.delayed(Duration.zero);

          final Uri uri = (await discoverer.uri)!;
          expect(uri, isNotNull);
          expect(uri.port, 9999);
          expect('$uri', 'http://127.0.0.1:9999');
        },
      );

      testWithoutContext(
        'uri throws if logs produce bad line and no listener is attached',
        () async {
          logReader.addLine('The Dart VM service is listening on http://127.0.0.1:apple');

          await Future<void>.delayed(Duration.zero);

          expect(discoverer.uri, throwsA(isFormatException));
        },
      );

      testWithoutContext('discovers uri if logs not yet produced output', () async {
        final Future<Uri?> uriFuture = discoverer.uri;
        logReader.addLine('The Dart VM service is listening on http://127.0.0.1:3333');
        final Uri uri = (await uriFuture)!;
        expect(uri.port, 3333);
        expect('$uri', 'http://127.0.0.1:3333');
      });

      testWithoutContext('discovers uri with Ascii Esc code', () async {
        logReader.addLine('The Dart VM service is listening on http://127.0.0.1:3333\x1b[');
        final Uri uri = (await discoverer.uri)!;
        expect(uri.port, 3333);
        expect('$uri', 'http://127.0.0.1:3333');
      });

      testWithoutContext('uri throws if logs produce bad line', () async {
        logReader.addLine('The Dart VM service is listening on http://127.0.0.1:apple');
        expect(discoverer.uri, throwsA(isFormatException));
      });

      testWithoutContext('uri is null when the log reader closes early', () async {
        final Future<Uri?> uriFuture = discoverer.uri;
        await logReader.dispose();

        expect(await uriFuture, isNull);
      });

      testWithoutContext('uri waits for correct log line', () async {
        final Future<Uri?> uriFuture = discoverer.uri;
        logReader.addLine('VM Service not listening...');
        final Uri timeoutUri = Uri.parse('http://timeout');
        final Uri? actualUri = await uriFuture.timeout(
          const Duration(milliseconds: 100),
          onTimeout: () => timeoutUri,
        );
        expect(actualUri, timeoutUri);
      });

      testWithoutContext('discovers uri if log line contains Android prefix', () async {
        logReader.addLine('I/flutter : The Dart VM service is listening on http://127.0.0.1:52584');
        final Uri uri = (await discoverer.uri)!;
        expect(uri.port, 52584);
        expect('$uri', 'http://127.0.0.1:52584');
      });

      testWithoutContext('discovers uri if log line contains auth key', () async {
        final Future<Uri?> uriFuture = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await uriFuture)!;
        expect(uri.port, 54804);
        expect('$uri', 'http://127.0.0.1:54804/PTwjm8Ii8qg=/');
      });

      testWithoutContext('discovers uri if log line contains non-localhost', () async {
        final Future<Uri?> uriFuture = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await uriFuture)!;
        expect(uri.port, 54804);
        expect('$uri', 'http://127.0.0.1:54804/PTwjm8Ii8qg=/');
      });

      testWithoutContext(
        'skips uri if port does not match the requested vmservice - requested last',
        () async {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            devicePort: 12346,
            throttleDuration: const Duration(milliseconds: 200),
            logger: BufferLogger.test(),
          );
          final Future<Uri?> uriFuture = discoverer.uri;
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          final Uri uri = (await uriFuture)!;
          expect(uri.port, 12346);
          expect('$uri', 'http://127.0.0.1:12346/PTwjm8Ii8qg=/');
        },
      );

      testWithoutContext(
        'skips uri if port does not match the requested vmservice - requested first',
        () async {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            devicePort: 12346,
            throttleDuration: const Duration(milliseconds: 200),
            logger: BufferLogger.test(),
          );
          final Future<Uri?> uriFuture = discoverer.uri;
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
          );
          final Uri uri = (await uriFuture)!;
          expect(uri.port, 12346);
          expect('$uri', 'http://127.0.0.1:12346/PTwjm8Ii8qg=/');
        },
      );

      testWithoutContext('first uri in the stream is the last one from the log', () async {
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
        );
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
        );
        final Uri uri = await discoverer.uris.first;
        expect(uri.port, 12345);
        expect('$uri', 'http://127.0.0.1:12345/PTwjm8Ii8qg=/');
      });

      testWithoutContext(
        'first uri in the stream is the last one from the log that matches the port',
        () async {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            devicePort: 12345,
            throttleDuration: const Duration(milliseconds: 200),
            logger: BufferLogger.test(),
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12344/PTwjm8Ii8qg=/',
          );
          final Uri uri = await discoverer.uris.first;
          expect(uri.port, 12345);
          expect('$uri', 'http://127.0.0.1:12345/PTwjm8Ii8qg=/');
        },
      );

      testWithoutContext(
        'protocol discovery does not crash if the log reader is closed while delaying',
        () async {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            devicePort: 12346,
            throttleDuration: const Duration(milliseconds: 10),
            logger: BufferLogger.test(),
          );
          final Future<List<Uri>> results = discoverer.uris.toList();
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          await logReader.dispose();

          // Give time for throttle to finish.
          await Future<void>.delayed(const Duration(milliseconds: 11));
          expect(await results, isEmpty);
        },
      );

      testWithoutContext('uris in the stream are throttled', () async {
        const Duration kThrottleDuration = Duration(milliseconds: 10);

        FakeAsync().run((FakeAsync time) {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            throttleDuration: kThrottleDuration,
            logger: BufferLogger.test(),
          );

          final List<Uri> discoveredUris = <Uri>[];
          discoverer.uris.listen((Uri uri) {
            discoveredUris.add(uri);
          });

          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
          );

          time.elapse(kThrottleDuration);

          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12344/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12343/PTwjm8Ii8qg=/',
          );

          time.elapse(kThrottleDuration);

          expect(discoveredUris.length, 2);
          expect(discoveredUris[0].port, 12345);
          expect('${discoveredUris[0]}', 'http://127.0.0.1:12345/PTwjm8Ii8qg=/');
          expect(discoveredUris[1].port, 12343);
          expect('${discoveredUris[1]}', 'http://127.0.0.1:12343/PTwjm8Ii8qg=/');
        });
      });

      testWithoutContext('uris in the stream are throttled when they match the port', () async {
        const Duration kThrottleTimeInMilliseconds = Duration(milliseconds: 10);

        FakeAsync().run((FakeAsync time) {
          discoverer = ProtocolDiscovery.vmService(
            logReader,
            ipv6: false,
            devicePort: 12345,
            throttleDuration: kThrottleTimeInMilliseconds,
            logger: BufferLogger.test(),
          );

          final List<Uri> discoveredUris = <Uri>[];
          discoverer.uris.listen((Uri uri) {
            discoveredUris.add(uri);
          });

          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12346/PTwjm8Ii8qg=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qg=/',
          );

          time.elapse(kThrottleTimeInMilliseconds);

          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12345/PTwjm8Ii8qc=/',
          );
          logReader.addLine(
            'I/flutter : The Dart VM service is listening on http://127.0.0.1:12344/PTwjm8Ii8qf=/',
          );

          time.elapse(kThrottleTimeInMilliseconds);

          expect(discoveredUris.length, 2);
          expect(discoveredUris[0].port, 12345);
          expect('${discoveredUris[0]}', 'http://127.0.0.1:12345/PTwjm8Ii8qg=/');
          expect(discoveredUris[1].port, 12345);
          expect('${discoveredUris[1]}', 'http://127.0.0.1:12345/PTwjm8Ii8qc=/');
        });
      });
    });

    group('port forwarding', () {
      testWithoutContext('default port', () async {
        final FakeDeviceLogReader logReader = FakeDeviceLogReader();
        final ProtocolDiscovery discoverer = ProtocolDiscovery.vmService(
          logReader,
          portForwarder: MockPortForwarder(99),
          ipv6: false,
          logger: BufferLogger.test(),
        );

        // Get next port future.
        final Future<Uri?> nextUri = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await nextUri)!;
        expect(uri.port, 99);
        expect('$uri', 'http://127.0.0.1:99/PTwjm8Ii8qg=/');

        await discoverer.cancel();
        await logReader.dispose();
      });

      testWithoutContext('specified port', () async {
        final FakeDeviceLogReader logReader = FakeDeviceLogReader();
        final ProtocolDiscovery discoverer = ProtocolDiscovery.vmService(
          logReader,
          portForwarder: MockPortForwarder(99),
          hostPort: 1243,
          ipv6: false,
          logger: BufferLogger.test(),
        );

        // Get next port future.
        final Future<Uri?> nextUri = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await nextUri)!;
        expect(uri.port, 1243);
        expect('$uri', 'http://127.0.0.1:1243/PTwjm8Ii8qg=/');

        await discoverer.cancel();
        await logReader.dispose();
      });

      testWithoutContext('specified port zero', () async {
        final FakeDeviceLogReader logReader = FakeDeviceLogReader();
        final ProtocolDiscovery discoverer = ProtocolDiscovery.vmService(
          logReader,
          portForwarder: MockPortForwarder(99),
          hostPort: 0,
          ipv6: false,
          logger: BufferLogger.test(),
        );

        // Get next port future.
        final Future<Uri?> nextUri = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await nextUri)!;
        expect(uri.port, 99);
        expect('$uri', 'http://127.0.0.1:99/PTwjm8Ii8qg=/');

        await discoverer.cancel();
        await logReader.dispose();
      });

      testWithoutContext('ipv6', () async {
        final FakeDeviceLogReader logReader = FakeDeviceLogReader();
        final ProtocolDiscovery discoverer = ProtocolDiscovery.vmService(
          logReader,
          portForwarder: MockPortForwarder(99),
          hostPort: 54777,
          ipv6: true,
          logger: BufferLogger.test(),
        );

        // Get next port future.
        final Future<Uri?> nextUri = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://127.0.0.1:54804/PTwjm8Ii8qg=/',
        );
        final Uri uri = (await nextUri)!;
        expect(uri.port, 54777);
        expect('$uri', 'http://[::1]:54777/PTwjm8Ii8qg=/');

        await discoverer.cancel();
        await logReader.dispose();
      });

      testWithoutContext('ipv6 with Ascii Escape code', () async {
        final FakeDeviceLogReader logReader = FakeDeviceLogReader();
        final ProtocolDiscovery discoverer = ProtocolDiscovery.vmService(
          logReader,
          portForwarder: MockPortForwarder(99),
          hostPort: 54777,
          ipv6: true,
          logger: BufferLogger.test(),
        );

        // Get next port future.
        final Future<Uri?> nextUri = discoverer.uri;
        logReader.addLine(
          'I/flutter : The Dart VM service is listening on http://[::1]:54777/PTwjm8Ii8qg=/\x1b[',
        );
        final Uri uri = (await nextUri)!;
        expect(uri.port, 54777);
        expect('$uri', 'http://[::1]:54777/PTwjm8Ii8qg=/');

        await discoverer.cancel();
        await logReader.dispose();
      });
    });
  });
}

class MockPortForwarder extends DevicePortForwarder {
  MockPortForwarder([this.availablePort]);

  final int? availablePort;

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async {
    hostPort ??= 0;
    if (hostPort == 0) {
      return availablePort!;
    }
    return hostPort;
  }

  @override
  List<ForwardedPort> get forwardedPorts => throw UnimplementedError();

  @override
  Future<void> unforward(ForwardedPort forwardedPort) {
    throw UnimplementedError();
  }

  @override
  Future<void> dispose() async {}
}
