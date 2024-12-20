// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/mdns_discovery.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:multicast_dns/multicast_dns.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../src/common.dart';
import '../src/fakes.dart';

void main() {
  group('mDNS Discovery', () {
    final int future = DateTime.now().add(const Duration(days: 1)).millisecondsSinceEpoch;

    setUp(() {
      setNetworkInterfaceLister(
        ({bool? includeLoopback, bool? includeLinkLocal, InternetAddressType? type}) async =>
            <NetworkInterface>[],
      );
    });

    tearDown(() {
      resetNetworkInterfaceLister();
    });

    group('for attach', () {
      late MDnsClient emptyClient;

      setUp(() {
        emptyClient = FakeMDnsClient(<PtrResourceRecord>[], <String, List<SrvResourceRecord>>{});
      });

      testWithoutContext('Find result in preliminary client', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: emptyClient,
          preliminaryMDnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        final MDnsVmServiceDiscoveryResult? result = await portDiscovery.queryForAttach();
        expect(result, isNotNull);
      });

      testWithoutContext(
        'Do not find result in preliminary client, but find in main client',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
            <String, List<SrvResourceRecord>>{
              'bar': <SrvResourceRecord>[
                SrvResourceRecord(
                  'bar',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'appId',
                ),
              ],
            },
          );

          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            preliminaryMDnsClient: emptyClient,
            logger: BufferLogger.test(),
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );

          final MDnsVmServiceDiscoveryResult? result = await portDiscovery.queryForAttach();
          expect(result, isNotNull);
        },
      );

      testWithoutContext('Find multiple in preliminary client', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'bar'),
            PtrResourceRecord('baz', future, domainName: 'fiz'),
          ],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
            'fiz': <SrvResourceRecord>[
              SrvResourceRecord('fiz', future, port: 321, weight: 1, priority: 1, target: 'local'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: emptyClient,
          preliminaryMDnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        expect(portDiscovery.queryForAttach, throwsToolExit());
      });

      testWithoutContext('Find duplicates in preliminary client', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'bar'),
            PtrResourceRecord('foo', future, domainName: 'bar'),
          ],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: emptyClient,
          preliminaryMDnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        final MDnsVmServiceDiscoveryResult? result = await portDiscovery.queryForAttach();
        expect(result, isNotNull);
      });

      testWithoutContext('Find similar named in preliminary client', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'bar'),
            PtrResourceRecord('foo', future, domainName: 'bar (2)'),
          ],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
            'bar (2)': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: emptyClient,
          preliminaryMDnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        expect(portDiscovery.queryForAttach, throwsToolExit());
      });

      testWithoutContext('No ports available', () async {
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: emptyClient,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        final int? port = (await portDiscovery.queryForAttach())?.port;
        expect(port, isNull);
      });

      testWithoutContext(
        'Prints helpful message when there is no ipv4 link local address.',
        () async {
          final BufferLogger logger = BufferLogger.test();
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final FakeAnalytics fakeAnalytics = getInitializedFakeAnalyticsInstance(
            fs: fs,
            fakeFlutterVersion: FakeFlutterVersion(),
          );
          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: emptyClient,
            preliminaryMDnsClient: emptyClient,
            logger: logger,
            flutterUsage: TestUsage(),
            analytics: fakeAnalytics,
          );
          final Uri? uri = await portDiscovery.getVMServiceUriForAttach('', FakeIOSDevice());
          expect(uri, isNull);
          expect(logger.errorText, contains('Personal Hotspot'));
          expect(
            fakeAnalytics.sentEvents,
            contains(Event.appleUsageEvent(workflow: 'ios-mdns', parameter: 'no-ipv4-link-local')),
          );
        },
      );

      testWithoutContext('One port available, no appId', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final int? port = (await portDiscovery.queryForAttach())?.port;
        expect(port, 123);
      });

      testWithoutContext('One port available, no appId, with authCode', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
          txtResponse: <String, List<TxtResourceRecord>>{
            'bar': <TxtResourceRecord>[TxtResourceRecord('bar', future, text: 'authCode=xyz\n')],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final MDnsVmServiceDiscoveryResult? result = await portDiscovery.queryForAttach();
        expect(result?.port, 123);
        expect(result?.authCode, 'xyz/');
      });

      testWithoutContext('Multiple ports available, with appId', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'bar'),
            PtrResourceRecord('baz', future, domainName: 'fiz'),
          ],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
            'fiz': <SrvResourceRecord>[
              SrvResourceRecord('fiz', future, port: 321, weight: 1, priority: 1, target: 'local'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final int? port = (await portDiscovery.queryForAttach(applicationId: 'fiz'))?.port;
        expect(port, 321);
      });

      testWithoutContext('Multiple ports available per process, with appId', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'bar'),
            PtrResourceRecord('baz', future, domainName: 'fiz'),
          ],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 1234, weight: 1, priority: 1, target: 'appId'),
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
            'fiz': <SrvResourceRecord>[
              SrvResourceRecord('fiz', future, port: 4321, weight: 1, priority: 1, target: 'local'),
              SrvResourceRecord('fiz', future, port: 321, weight: 1, priority: 1, target: 'local'),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final int? port = (await portDiscovery.queryForAttach(applicationId: 'bar'))?.port;
        expect(port, 1234);
      });

      testWithoutContext('Throws Exception when client throws OSError on start', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[],
          <String, List<SrvResourceRecord>>{},
          osErrorOnStart: true,
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        expect(() async => portDiscovery.queryForAttach(), throwsException);
      });

      testWithoutContext('Correctly builds VM Service URI with hostVmservicePort == 0', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForAttach(
          'bar',
          device,
          hostVmservicePort: 0,
        );
        expect(uri.toString(), 'http://127.0.0.1:123/');
      });

      testWithoutContext('Get wireless device IP (iPv4)', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 1234, weight: 1, priority: 1, target: 'appId'),
            ],
          },
          ipResponse: <String, List<IPAddressResourceRecord>>{
            'appId': <IPAddressResourceRecord>[
              IPAddressResourceRecord(
                'Device IP',
                0,
                address: InternetAddress.tryParse('111.111.111.111')!,
              ),
            ],
          },
          txtResponse: <String, List<TxtResourceRecord>>{
            'bar': <TxtResourceRecord>[TxtResourceRecord('bar', future, text: 'authCode=xyz\n')],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForAttach(
          'bar',
          device,
          useDeviceIPAsHost: true,
        );
        expect(uri.toString(), 'http://111.111.111.111:1234/xyz/');
      });

      testWithoutContext('Get wireless device IP (iPv6)', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 1234, weight: 1, priority: 1, target: 'appId'),
            ],
          },
          ipResponse: <String, List<IPAddressResourceRecord>>{
            'appId': <IPAddressResourceRecord>[
              IPAddressResourceRecord(
                'Device IP',
                0,
                address: InternetAddress.tryParse('1111:1111:1111:1111:1111:1111:1111:1111')!,
              ),
            ],
          },
          txtResponse: <String, List<TxtResourceRecord>>{
            'bar': <TxtResourceRecord>[TxtResourceRecord('bar', future, text: 'authCode=xyz\n')],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForAttach(
          'bar',
          device,
          useDeviceIPAsHost: true,
        );
        expect(uri.toString(), 'http://[1111:1111:1111:1111:1111:1111:1111:1111]:1234/xyz/');
      });

      testWithoutContext(
        'Throw error if unable to find VM service with app id and device port',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[
              PtrResourceRecord('foo', future, domainName: 'srv-foo'),
              PtrResourceRecord('bar', future, domainName: 'srv-bar'),
              PtrResourceRecord('baz', future, domainName: 'srv-boo'),
            ],
            <String, List<SrvResourceRecord>>{
              'srv-foo': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-foo',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-foo',
                ),
              ],
              'srv-bar': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-bar',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-bar',
                ),
              ],
              'srv-baz': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-baz',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-baz',
                ),
              ],
            },
          );
          final FakeIOSDevice device = FakeIOSDevice();
          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            preliminaryMDnsClient: emptyClient,
            logger: BufferLogger.test(),
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );
          expect(
            portDiscovery.getVMServiceUriForAttach('srv-bar', device, deviceVmservicePort: 321),
            throwsToolExit(
              message: 'Did not find a Dart VM Service advertised for srv-bar on port 321.',
            ),
          );
        },
      );

      testWithoutContext('Throw error if unable to find VM Service with app id', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'srv-foo')],
          <String, List<SrvResourceRecord>>{
            'srv-foo': <SrvResourceRecord>[
              SrvResourceRecord(
                'srv-foo',
                future,
                port: 123,
                weight: 1,
                priority: 1,
                target: 'target-foo',
              ),
            ],
          },
        );
        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          preliminaryMDnsClient: emptyClient,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        expect(
          portDiscovery.getVMServiceUriForAttach('srv-asdf', device),
          throwsToolExit(message: 'Did not find a Dart VM Service advertised for srv-asdf.'),
        );
      });
    });

    group('for launch', () {
      testWithoutContext('Ensure either port or device name are provided', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[],
          <String, List<SrvResourceRecord>>{},
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        expect(
          () async => portDiscovery.queryForLaunch(applicationId: 'app-id'),
          throwsAssertionError,
        );
      });

      testWithoutContext('No ports available', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[],
          <String, List<SrvResourceRecord>>{},
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        final MDnsVmServiceDiscoveryResult? result = await portDiscovery.queryForLaunch(
          applicationId: 'app-id',
          deviceVmservicePort: 123,
        );

        expect(result, null);
      });

      testWithoutContext(
        'Prints helpful message when there is no ipv4 link local address.',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[],
            <String, List<SrvResourceRecord>>{},
          );
          final BufferLogger logger = BufferLogger.test();
          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            logger: logger,
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );

          final Uri? uri = await portDiscovery.getVMServiceUriForLaunch(
            '',
            FakeIOSDevice(),
            deviceVmservicePort: 0,
          );
          expect(uri, isNull);
          expect(logger.errorText, contains('Personal Hotspot'));
        },
      );

      testWithoutContext('Throws Exception when client throws OSError on start', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[],
          <String, List<SrvResourceRecord>>{},
          osErrorOnStart: true,
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        expect(
          () async =>
              portDiscovery.queryForLaunch(applicationId: 'app-id', deviceVmservicePort: 123),
          throwsException,
        );
      });

      test(
        'On macOS, throw tool exit with a helpful message when client throws a SocketException on lookup',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[],
            <String, List<SrvResourceRecord>>{},
            socketExceptionOnStart: true,
          );

          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            logger: BufferLogger.test(),
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );

          expect(
            portDiscovery.firstMatchingVmService(client),
            throwsToolExit(
              message:
                  'You might be having a permissions issue with your IDE. '
                  'Please try going to '
                  'System Settings -> Privacy & Security -> Local Network -> '
                  '[Find your IDE] -> Toggle ON, then restart your phone.',
            ),
          );
        },
        // [intended] This tool exit message only works for macOS
        skip: !globals.platform.isMacOS,
      );

      testWithoutContext('Correctly builds VM Service URI with hostVmservicePort == 0', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 123, weight: 1, priority: 1, target: 'appId'),
            ],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForLaunch(
          'bar',
          device,
          hostVmservicePort: 0,
          deviceVmservicePort: 123,
        );
        expect(uri.toString(), 'http://127.0.0.1:123/');
      });

      testWithoutContext('Get wireless device IP (iPv4)', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 1234, weight: 1, priority: 1, target: 'appId'),
            ],
          },
          ipResponse: <String, List<IPAddressResourceRecord>>{
            'appId': <IPAddressResourceRecord>[
              IPAddressResourceRecord(
                'Device IP',
                0,
                address: InternetAddress.tryParse('111.111.111.111')!,
              ),
            ],
          },
          txtResponse: <String, List<TxtResourceRecord>>{
            'bar': <TxtResourceRecord>[TxtResourceRecord('bar', future, text: 'authCode=xyz\n')],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForLaunch(
          'bar',
          device,
          useDeviceIPAsHost: true,
          deviceVmservicePort: 1234,
        );
        expect(uri.toString(), 'http://111.111.111.111:1234/xyz/');
      });

      testWithoutContext('Get wireless device IP (iPv6)', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
          <String, List<SrvResourceRecord>>{
            'bar': <SrvResourceRecord>[
              SrvResourceRecord('bar', future, port: 1234, weight: 1, priority: 1, target: 'appId'),
            ],
          },
          ipResponse: <String, List<IPAddressResourceRecord>>{
            'appId': <IPAddressResourceRecord>[
              IPAddressResourceRecord(
                'Device IP',
                0,
                address: InternetAddress.tryParse('1111:1111:1111:1111:1111:1111:1111:1111')!,
              ),
            ],
          },
          txtResponse: <String, List<TxtResourceRecord>>{
            'bar': <TxtResourceRecord>[TxtResourceRecord('bar', future, text: 'authCode=xyz\n')],
          },
        );

        final FakeIOSDevice device = FakeIOSDevice();
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final Uri? uri = await portDiscovery.getVMServiceUriForLaunch(
          'bar',
          device,
          useDeviceIPAsHost: true,
          deviceVmservicePort: 1234,
        );
        expect(uri.toString(), 'http://[1111:1111:1111:1111:1111:1111:1111:1111]:1234/xyz/');
      });

      testWithoutContext(
        'Throw error if unable to find VM Service with app id and device port',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[
              PtrResourceRecord('foo', future, domainName: 'srv-foo'),
              PtrResourceRecord('bar', future, domainName: 'srv-bar'),
              PtrResourceRecord('baz', future, domainName: 'srv-boo'),
            ],
            <String, List<SrvResourceRecord>>{
              'srv-foo': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-foo',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-foo',
                ),
              ],
              'srv-bar': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-bar',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-bar',
                ),
              ],
              'srv-baz': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-baz',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-baz',
                ),
              ],
            },
          );
          final FakeIOSDevice device = FakeIOSDevice();
          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            logger: BufferLogger.test(),
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );
          expect(
            portDiscovery.getVMServiceUriForLaunch('srv-bar', device, deviceVmservicePort: 321),
            throwsToolExit(
              message: 'Did not find a Dart VM Service advertised for srv-bar on port 321.',
            ),
          );
        },
      );

      testWithoutContext('Matches on application id and device name', () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'srv-foo'),
            PtrResourceRecord('bar', future, domainName: 'srv-bar'),
            PtrResourceRecord('baz', future, domainName: 'srv-boo'),
          ],
          <String, List<SrvResourceRecord>>{
            'srv-bar': <SrvResourceRecord>[
              SrvResourceRecord(
                'srv-foo',
                future,
                port: 123,
                weight: 1,
                priority: 1,
                target: 'My-Phone.local',
              ),
            ],
          },
        );
        final FakeIOSDevice device = FakeIOSDevice(name: 'My Phone');
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        final Uri? uri = await portDiscovery.getVMServiceUriForLaunch('srv-bar', device);
        expect(uri.toString(), 'http://127.0.0.1:123/');
      });

      testWithoutContext(
        'Throw error if unable to find VM Service with app id and device name',
        () async {
          final MDnsClient client = FakeMDnsClient(
            <PtrResourceRecord>[
              PtrResourceRecord('foo', future, domainName: 'srv-foo'),
              PtrResourceRecord('bar', future, domainName: 'srv-bar'),
              PtrResourceRecord('baz', future, domainName: 'srv-boo'),
            ],
            <String, List<SrvResourceRecord>>{
              'srv-foo': <SrvResourceRecord>[
                SrvResourceRecord(
                  'srv-foo',
                  future,
                  port: 123,
                  weight: 1,
                  priority: 1,
                  target: 'target-foo',
                ),
              ],
            },
          );
          final FakeIOSDevice device = FakeIOSDevice(name: 'My Phone');
          final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
            mdnsClient: client,
            logger: BufferLogger.test(),
            flutterUsage: TestUsage(),
            analytics: const NoOpAnalytics(),
          );
          expect(
            portDiscovery.getVMServiceUriForLaunch('srv-bar', device),
            throwsToolExit(message: 'Did not find a Dart VM Service advertised for srv-bar'),
          );
        },
      );
    });

    group('deviceNameMatchesTargetName', () {
      testWithoutContext('compares case insensitive and without spaces, hyphens, .local', () {
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: FakeMDnsClient(<PtrResourceRecord>[], <String, List<SrvResourceRecord>>{}),
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );

        expect(portDiscovery.deviceNameMatchesTargetName('My phone', 'My-Phone.local'), isTrue);
      });

      testWithoutContext('includes numbers in comparison', () {
        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: FakeMDnsClient(<PtrResourceRecord>[], <String, List<SrvResourceRecord>>{}),
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        expect(portDiscovery.deviceNameMatchesTargetName('My phone', 'My-Phone-2.local'), isFalse);
      });
    });

    testWithoutContext(
      'Find firstMatchingVmService with many available and no application id',
      () async {
        final MDnsClient client = FakeMDnsClient(
          <PtrResourceRecord>[
            PtrResourceRecord('foo', future, domainName: 'srv-foo'),
            PtrResourceRecord('bar', future, domainName: 'srv-bar'),
            PtrResourceRecord('baz', future, domainName: 'srv-boo'),
          ],
          <String, List<SrvResourceRecord>>{
            'srv-foo': <SrvResourceRecord>[
              SrvResourceRecord(
                'srv-foo',
                future,
                port: 123,
                weight: 1,
                priority: 1,
                target: 'target-foo',
              ),
            ],
            'srv-bar': <SrvResourceRecord>[
              SrvResourceRecord(
                'srv-bar',
                future,
                port: 123,
                weight: 1,
                priority: 1,
                target: 'target-bar',
              ),
            ],
            'srv-baz': <SrvResourceRecord>[
              SrvResourceRecord(
                'srv-baz',
                future,
                port: 123,
                weight: 1,
                priority: 1,
                target: 'target-baz',
              ),
            ],
          },
        );

        final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
          mdnsClient: client,
          logger: BufferLogger.test(),
          flutterUsage: TestUsage(),
          analytics: const NoOpAnalytics(),
        );
        final MDnsVmServiceDiscoveryResult? result = await portDiscovery.firstMatchingVmService(
          client,
        );
        expect(result?.domainName, 'srv-foo');
      },
    );

    testWithoutContext('Find firstMatchingVmService app id', () async {
      final MDnsClient client = FakeMDnsClient(
        <PtrResourceRecord>[
          PtrResourceRecord('foo', future, domainName: 'srv-foo'),
          PtrResourceRecord('bar', future, domainName: 'srv-bar'),
          PtrResourceRecord('baz', future, domainName: 'srv-boo'),
        ],
        <String, List<SrvResourceRecord>>{
          'srv-foo': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-foo',
              future,
              port: 111,
              weight: 1,
              priority: 1,
              target: 'target-foo',
            ),
          ],
          'srv-bar': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-bar',
              future,
              port: 222,
              weight: 1,
              priority: 1,
              target: 'target-bar',
            ),
            SrvResourceRecord(
              'srv-bar',
              future,
              port: 333,
              weight: 1,
              priority: 1,
              target: 'target-bar-2',
            ),
          ],
          'srv-baz': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-baz',
              future,
              port: 444,
              weight: 1,
              priority: 1,
              target: 'target-baz',
            ),
          ],
        },
      );

      final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
        mdnsClient: client,
        logger: BufferLogger.test(),
        flutterUsage: TestUsage(),
        analytics: const NoOpAnalytics(),
      );
      final MDnsVmServiceDiscoveryResult? result = await portDiscovery.firstMatchingVmService(
        client,
        applicationId: 'srv-bar',
      );
      expect(result?.domainName, 'srv-bar');
      expect(result?.port, 222);
    });
    testWithoutContext('find with no txt record', () async {
      final MDnsClient client = FakeMDnsClient(
        <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'srv-foo')],
        <String, List<SrvResourceRecord>>{
          'srv-foo': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-foo',
              future,
              port: 111,
              weight: 1,
              priority: 1,
              target: 'target-foo',
            ),
          ],
        },
        ipResponse: <String, List<IPAddressResourceRecord>>{
          'target-foo': <IPAddressResourceRecord>[
            IPAddressResourceRecord(
              'target-foo',
              0,
              address: InternetAddress.tryParse('111.111.111.111')!,
            ),
          ],
        },
      );

      final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
        mdnsClient: client,
        logger: BufferLogger.test(),
        flutterUsage: TestUsage(),
        analytics: const NoOpAnalytics(),
      );
      final MDnsVmServiceDiscoveryResult? result = await portDiscovery.firstMatchingVmService(
        client,
        applicationId: 'srv-foo',
        useDeviceIPAsHost: true,
      );
      expect(result?.domainName, 'srv-foo');
      expect(result?.port, 111);
      expect(result?.authCode, '');
      expect(result?.ipAddress?.address, '111.111.111.111');
    });
    testWithoutContext('find with empty txt record', () async {
      final MDnsClient client = FakeMDnsClient(
        <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'srv-foo')],
        <String, List<SrvResourceRecord>>{
          'srv-foo': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-foo',
              future,
              port: 111,
              weight: 1,
              priority: 1,
              target: 'target-foo',
            ),
          ],
        },
        txtResponse: <String, List<TxtResourceRecord>>{
          'srv-foo': <TxtResourceRecord>[TxtResourceRecord('srv-foo', future, text: '')],
        },
        ipResponse: <String, List<IPAddressResourceRecord>>{
          'target-foo': <IPAddressResourceRecord>[
            IPAddressResourceRecord(
              'target-foo',
              0,
              address: InternetAddress.tryParse('111.111.111.111')!,
            ),
          ],
        },
      );

      final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
        mdnsClient: client,
        logger: BufferLogger.test(),
        flutterUsage: TestUsage(),
        analytics: const NoOpAnalytics(),
      );
      final MDnsVmServiceDiscoveryResult? result = await portDiscovery.firstMatchingVmService(
        client,
        applicationId: 'srv-foo',
        useDeviceIPAsHost: true,
      );
      expect(result?.domainName, 'srv-foo');
      expect(result?.port, 111);
      expect(result?.authCode, '');
      expect(result?.ipAddress?.address, '111.111.111.111');
    });
    testWithoutContext('find with valid txt record', () async {
      final MDnsClient client = FakeMDnsClient(
        <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'srv-foo')],
        <String, List<SrvResourceRecord>>{
          'srv-foo': <SrvResourceRecord>[
            SrvResourceRecord(
              'srv-foo',
              future,
              port: 111,
              weight: 1,
              priority: 1,
              target: 'target-foo',
            ),
          ],
        },
        txtResponse: <String, List<TxtResourceRecord>>{
          'srv-foo': <TxtResourceRecord>[
            TxtResourceRecord('srv-foo', future, text: 'authCode=xyz\n'),
          ],
        },
        ipResponse: <String, List<IPAddressResourceRecord>>{
          'target-foo': <IPAddressResourceRecord>[
            IPAddressResourceRecord(
              'target-foo',
              0,
              address: InternetAddress.tryParse('111.111.111.111')!,
            ),
          ],
        },
      );

      final MDnsVmServiceDiscovery portDiscovery = MDnsVmServiceDiscovery(
        mdnsClient: client,
        logger: BufferLogger.test(),
        flutterUsage: TestUsage(),
        analytics: const NoOpAnalytics(),
      );
      final MDnsVmServiceDiscoveryResult? result = await portDiscovery.firstMatchingVmService(
        client,
        applicationId: 'srv-foo',
        useDeviceIPAsHost: true,
      );
      expect(result?.domainName, 'srv-foo');
      expect(result?.port, 111);
      expect(result?.authCode, 'xyz/');
      expect(result?.ipAddress?.address, '111.111.111.111');
    });
  });
}

class FakeMDnsClient extends Fake implements MDnsClient {
  FakeMDnsClient(
    this.ptrRecords,
    this.srvResponse, {
    this.txtResponse = const <String, List<TxtResourceRecord>>{},
    this.ipResponse = const <String, List<IPAddressResourceRecord>>{},
    this.osErrorOnStart = false,
    this.socketExceptionOnStart = false,
  });

  final List<PtrResourceRecord> ptrRecords;
  final Map<String, List<SrvResourceRecord>> srvResponse;
  final Map<String, List<TxtResourceRecord>> txtResponse;
  final Map<String, List<IPAddressResourceRecord>> ipResponse;
  final bool osErrorOnStart;
  final bool socketExceptionOnStart;

  @override
  Future<void> start({
    InternetAddress? listenAddress,
    NetworkInterfacesFactory? interfacesFactory,
    int mDnsPort = 5353,
    InternetAddress? mDnsAddress,
  }) async {
    if (osErrorOnStart) {
      throw const OSError('Operation not supported on socket', 102);
    }
  }

  @override
  Stream<T> lookup<T extends ResourceRecord>(
    ResourceRecordQuery query, {
    Duration timeout = const Duration(seconds: 5),
  }) {
    if (socketExceptionOnStart) {
      throw const SocketException('Socket Exception');
    }
    if (T == PtrResourceRecord &&
        query.fullyQualifiedName == MDnsVmServiceDiscovery.dartVmServiceName) {
      return Stream<PtrResourceRecord>.fromIterable(ptrRecords) as Stream<T>;
    }
    if (T == SrvResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<SrvResourceRecord>.fromIterable(srvResponse[key] ?? <SrvResourceRecord>[])
          as Stream<T>;
    }
    if (T == TxtResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<TxtResourceRecord>.fromIterable(txtResponse[key] ?? <TxtResourceRecord>[])
          as Stream<T>;
    }
    if (T == IPAddressResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<IPAddressResourceRecord>.fromIterable(
            ipResponse[key] ?? <IPAddressResourceRecord>[],
          )
          as Stream<T>;
    }
    throw UnsupportedError('Unsupported query type $T');
  }

  @override
  void stop() {}
}

class FakeIOSDevice extends Fake implements IOSDevice {
  FakeIOSDevice({this.name = 'iPhone'});

  @override
  final String name;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.ios;

  @override
  bool isSupported() => true;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => true;

  @override
  DevicePortForwarder get portForwarder => const NoOpDevicePortForwarder();
}
