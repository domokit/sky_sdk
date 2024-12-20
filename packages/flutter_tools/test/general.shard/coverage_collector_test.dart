// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show jsonEncode;
import 'dart:io' show Directory, File;

import 'package:coverage/coverage.dart' show HitMap;
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart' show FileSystem;
import 'package:flutter_tools/src/test/coverage_collector.dart';
import 'package:flutter_tools/src/test/test_device.dart' show TestDevice;
import 'package:flutter_tools/src/test/test_time_recorder.dart';
import 'package:stream_channel/stream_channel.dart' show StreamChannel;
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_vm_services.dart';
import '../src/logging_logger.dart';

void main() {
  testWithoutContext('Coverage collector Can handle coverage SentinelException', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        const FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage'],
            'forceCompile': true,
            'reportLines': true,
            'libraryFilters': <Object>['package:foo/'],
            'librariesAlreadyCompiled': <Object>[],
          },
          jsonResponse: <String, Object>{'type': 'Sentinel'},
        ),
      ],
    );

    final Map<String, Object?> result = await collect(
      Uri(),
      <String>{'foo'},
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{'type': 'CodeCoverage', 'coverage': <Object>[]});
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector processes coverage and script data', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage'],
            'forceCompile': true,
            'reportLines': true,
            'libraryFilters': <Object>['package:foo/'],
            'librariesAlreadyCompiled': <Object>[],
          },
          jsonResponse:
              SourceReport(
                ranges: <SourceReportRange>[
                  SourceReportRange(
                    scriptIndex: 0,
                    startPos: 0,
                    endPos: 0,
                    compiled: true,
                    coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                  ),
                ],
                scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
              ).toJson(),
        ),
      ],
    );

    final Map<String, Object?> result = await collect(
      Uri(),
      <String>{'foo'},
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
        },
      ],
    });
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector with null libraryNames accepts all libraries', () async {
    final FakeVmServiceHost fakeVmServiceHost = createFakeVmServiceHostWithFooAndBar();

    final Map<String, Object?> result = await collect(
      Uri(),
      null,
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
        },
        <String, Object>{
          'source': 'package:bar/bar.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Abar%2Fbar.dart',
            'uri': 'package:bar/bar.dart',
            '_kind': 'library',
          },
          'hits': <Object>[47, 1, 21, 1, 32, 0, 86, 0],
        },
      ],
    });
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector with libraryFilters', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage'],
            'forceCompile': true,
            'reportLines': true,
            'libraryFilters': <Object>['package:foo/'],
            'librariesAlreadyCompiled': <Object>[],
          },
          jsonResponse:
              SourceReport(
                ranges: <SourceReportRange>[
                  SourceReportRange(
                    scriptIndex: 0,
                    startPos: 0,
                    endPos: 0,
                    compiled: true,
                    coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                  ),
                ],
                scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
              ).toJson(),
        ),
      ],
    );

    final Map<String, Object?> result = await collect(
      Uri(),
      <String>{'foo'},
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
        },
      ],
    });
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector with libraryFilters and null libraryNames', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage'],
            'forceCompile': true,
            'reportLines': true,
            'librariesAlreadyCompiled': <Object>[],
          },
          jsonResponse:
              SourceReport(
                ranges: <SourceReportRange>[
                  SourceReportRange(
                    scriptIndex: 0,
                    startPos: 0,
                    endPos: 0,
                    compiled: true,
                    coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                  ),
                ],
                scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
              ).toJson(),
        ),
      ],
    );

    final Map<String, Object?> result = await collect(
      Uri(),
      null,
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
        },
      ],
    });
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector with branch coverage', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage', 'BranchCoverage'],
            'forceCompile': true,
            'reportLines': true,
            'libraryFilters': <Object>['package:foo/'],
            'librariesAlreadyCompiled': <Object>[],
          },
          jsonResponse:
              SourceReport(
                ranges: <SourceReportRange>[
                  SourceReportRange(
                    scriptIndex: 0,
                    startPos: 0,
                    endPos: 0,
                    compiled: true,
                    coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                    branchCoverage: SourceReportCoverage(hits: <int>[4, 6], misses: <int>[5]),
                  ),
                ],
                scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
              ).toJson(),
        ),
      ],
    );

    final Map<String, Object?> result = await collect(
      Uri(),
      <String>{'foo'},
      serviceOverride: fakeVmServiceHost.vmService,
      branchCoverage: true,
      coverableLineCache: <String, Set<int>>{},
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
          'branchHits': <Object>[4, 1, 6, 1, 5, 0],
        },
      ],
    });
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('Coverage collector caches read files', () async {
    Directory? tempDir;
    try {
      tempDir = Directory.systemTemp.createTempSync('flutter_coverage_collector_test.');
      final File packagesFile = writeFooBarPackagesJson(tempDir);
      final Directory fooDir = Directory('${tempDir.path}/foo/');
      fooDir.createSync();
      final File fooFile = File('${fooDir.path}/foo.dart');
      fooFile.writeAsStringSync('hit\nnohit but ignored // coverage:ignore-line\nhit\n');

      final String packagesPath = packagesFile.path;
      final CoverageCollector collector = CoverageCollector(
        libraryNames: <String>{'foo', 'bar'},
        verbose: false,
        packagesPath: packagesPath,
        resolver: await CoverageCollector.getResolver(packagesPath),
      );
      await collector.collectCoverage(
        TestTestDevice(),
        serviceOverride:
            createFakeVmServiceHostWithFooAndBar(
              libraryFilters: <String>['package:foo/', 'package:bar/'],
            ).vmService,
      );

      Future<void> getHitMapAndVerify() async {
        final Map<String, HitMap> gottenHitmap = <String, HitMap>{};
        await collector.finalizeCoverage(
          formatter: (Map<String, HitMap> hitmap) {
            gottenHitmap.addAll(hitmap);
            return '';
          },
        );
        expect(gottenHitmap.keys.toList()..sort(), <String>[
          'package:bar/bar.dart',
          'package:foo/foo.dart',
        ]);
        expect(gottenHitmap['package:foo/foo.dart']!.lineHits, <int, int>{
          1: 1,
          /* 2: 0, is ignored in file */ 3: 1,
        });
        expect(gottenHitmap['package:bar/bar.dart']!.lineHits, <int, int>{
          21: 1,
          32: 0,
          47: 1,
          86: 0,
        });
      }

      Future<void> verifyHitmapEmpty() async {
        final Map<String, HitMap> gottenHitmap = <String, HitMap>{};
        await collector.finalizeCoverage(
          formatter: (Map<String, HitMap> hitmap) {
            gottenHitmap.addAll(hitmap);
            return '';
          },
        );
        expect(gottenHitmap.isEmpty, isTrue);
      }

      // Get hit map the first time.
      await getHitMapAndVerify();

      // Getting the hitmap clears it so we now doesn't get any data.
      await verifyHitmapEmpty();

      // Collecting again gets us the same data even though the foo file has been deleted.
      // This means that the fact that line 2 was ignored has been cached.
      fooFile.deleteSync();
      await collector.collectCoverage(
        TestTestDevice(),
        serviceOverride:
            createFakeVmServiceHostWithFooAndBar(
              libraryFilters: <String>['package:foo/', 'package:bar/'],
              librariesAlreadyCompiled: <String>['package:foo/foo.dart', 'package:bar/bar.dart'],
            ).vmService,
      );
      await getHitMapAndVerify();
    } finally {
      tempDir?.deleteSync(recursive: true);
    }
  });

  testWithoutContext('Coverage collector respects ignore whole file', () async {
    Directory? tempDir;
    try {
      tempDir = Directory.systemTemp.createTempSync('flutter_coverage_collector_test.');
      final File packagesFile = writeFooBarPackagesJson(tempDir);
      final Directory fooDir = Directory('${tempDir.path}/foo/');
      fooDir.createSync();
      final File fooFile = File('${fooDir.path}/foo.dart');
      fooFile.writeAsStringSync('hit\nnohit but ignored // coverage:ignore-file\nhit\n');

      final String packagesPath = packagesFile.path;
      final CoverageCollector collector = CoverageCollector(
        libraryNames: <String>{'foo', 'bar'},
        verbose: false,
        packagesPath: packagesPath,
        resolver: await CoverageCollector.getResolver(packagesPath),
      );
      await collector.collectCoverage(
        TestTestDevice(),
        serviceOverride:
            createFakeVmServiceHostWithFooAndBar(
              libraryFilters: <String>['package:foo/', 'package:bar/'],
            ).vmService,
      );

      final Map<String, HitMap> gottenHitmap = <String, HitMap>{};
      await collector.finalizeCoverage(
        formatter: (Map<String, HitMap> hitmap) {
          gottenHitmap.addAll(hitmap);
          return '';
        },
      );
      expect(gottenHitmap.keys.toList()..sort(), <String>['package:bar/bar.dart']);
      expect(gottenHitmap['package:bar/bar.dart']!.lineHits, <int, int>{
        21: 1,
        32: 0,
        47: 1,
        86: 0,
      });
    } finally {
      tempDir?.deleteSync(recursive: true);
    }
  });

  testUsingContext(
    'Coverage collector respects libraryNames in finalized report',
    () async {
      Directory? tempDir;
      try {
        tempDir = Directory.systemTemp.createTempSync('flutter_coverage_collector_test.');
        final File packagesFile = writeFooBarPackagesJson(tempDir);
        File('${tempDir.path}/foo/foo.dart').createSync(recursive: true);
        File('${tempDir.path}/bar/bar.dart').createSync(recursive: true);

        final String packagesPath = packagesFile.path;
        CoverageCollector collector = CoverageCollector(
          libraryNames: <String>{'foo', 'bar'},
          verbose: false,
          packagesPath: packagesPath,
          resolver: await CoverageCollector.getResolver(packagesPath),
        );
        await collector.collectCoverage(
          TestTestDevice(),
          serviceOverride:
              createFakeVmServiceHostWithFooAndBar(
                libraryFilters: <String>['package:foo/', 'package:bar/'],
              ).vmService,
        );

        String? report = await collector.finalizeCoverage();
        expect(report, contains('foo.dart'));
        expect(report, contains('bar.dart'));

        collector = CoverageCollector(
          libraryNames: <String>{'foo'},
          verbose: false,
          packagesPath: packagesPath,
          resolver: await CoverageCollector.getResolver(packagesPath),
        );
        await collector.collectCoverage(
          TestTestDevice(),
          serviceOverride:
              createFakeVmServiceHostWithFooAndBar(
                libraryFilters: <String>['package:foo/'],
              ).vmService,
        );

        report = await collector.finalizeCoverage();
        expect(report, contains('foo.dart'));
        expect(report, isNot(contains('bar.dart')));
      } finally {
        tempDir?.deleteSync(recursive: true);
      }
    },
    overrides: <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext(
    'Coverage collector records test timings when provided TestTimeRecorder',
    () async {
      Directory? tempDir;
      try {
        tempDir = Directory.systemTemp.createTempSync('flutter_coverage_collector_test.');
        final File packagesFile = writeFooBarPackagesJson(tempDir);
        final Directory fooDir = Directory('${tempDir.path}/foo/');
        fooDir.createSync();
        final File fooFile = File('${fooDir.path}/foo.dart');
        fooFile.writeAsStringSync('hit\nnohit but ignored // coverage:ignore-line\nhit\n');

        final String packagesPath = packagesFile.path;
        final LoggingLogger logger = LoggingLogger();
        final TestTimeRecorder testTimeRecorder = TestTimeRecorder(logger);
        final CoverageCollector collector = CoverageCollector(
          libraryNames: <String>{'foo', 'bar'},
          verbose: false,
          packagesPath: packagesPath,
          resolver: await CoverageCollector.getResolver(packagesPath),
          testTimeRecorder: testTimeRecorder,
        );
        await collector.collectCoverage(
          TestTestDevice(),
          serviceOverride:
              createFakeVmServiceHostWithFooAndBar(
                libraryFilters: <String>['package:foo/', 'package:bar/'],
              ).vmService,
        );

        // Expect one message for each phase.
        final List<String> logPhaseMessages =
            testTimeRecorder
                .getPrintAsListForTesting()
                .where((String m) => m.startsWith('Runtime for phase '))
                .toList();
        expect(logPhaseMessages, hasLength(TestTimePhases.values.length));

        // Several phases actually does something, but here we just expect at
        // least one phase to take a non-zero amount of time.
        final List<String> logPhaseMessagesNonZero =
            logPhaseMessages.where((String m) => !m.contains(Duration.zero.toString())).toList();
        expect(logPhaseMessagesNonZero, isNotEmpty);
      } finally {
        tempDir?.deleteSync(recursive: true);
      }
    },
  );

  testWithoutContext('Coverage collector fills coverableLineCache', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getVM',
          jsonResponse:
              (VM.parse(<String, Object>{})!
                    ..isolates = <IsolateRef>[
                      IsolateRef.parse(<String, Object>{'id': '1'})!,
                    ])
                  .toJson(),
        ),
        FakeVmServiceRequest(
          method: 'getSourceReport',
          args: <String, Object>{
            'isolateId': '1',
            'reports': <Object>['Coverage'],
            'forceCompile': true,
            'reportLines': true,
            'libraryFilters': <String>['package:foo/'],
            'librariesAlreadyCompiled': <String>[],
          },
          jsonResponse:
              SourceReport(
                ranges: <SourceReportRange>[
                  SourceReportRange(
                    scriptIndex: 0,
                    startPos: 0,
                    endPos: 0,
                    compiled: true,
                    coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                  ),
                ],
                scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
              ).toJson(),
        ),
      ],
    );

    final Map<String, Set<int>> coverableLineCache = <String, Set<int>>{};
    final Map<String, Object?> result = await collect(
      Uri(),
      <String>{'foo'},
      serviceOverride: fakeVmServiceHost.vmService,
      coverableLineCache: coverableLineCache,
    );

    expect(result, <String, Object>{
      'type': 'CodeCoverage',
      'coverage': <Object>[
        <String, Object>{
          'source': 'package:foo/foo.dart',
          'script': <String, Object>{
            'type': '@Script',
            'fixedId': true,
            'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
            'uri': 'package:foo/foo.dart',
            '_kind': 'library',
          },
          'hits': <Object>[1, 1, 3, 1, 2, 0],
        },
      ],
    });

    // coverableLineCache should contain every line mentioned in the report.
    expect(coverableLineCache, <String, Set<int>>{
      'package:foo/foo.dart': <int>{1, 2, 3},
    });

    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext(
    'Coverage collector avoids recompiling libraries in coverableLineCache',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: 'getVM',
            jsonResponse:
                (VM.parse(<String, Object>{})!
                      ..isolates = <IsolateRef>[
                        IsolateRef.parse(<String, Object>{'id': '1'})!,
                      ])
                    .toJson(),
          ),

          // This collection sets librariesAlreadyCompiled. The response doesn't
          // include any misses.
          FakeVmServiceRequest(
            method: 'getSourceReport',
            args: <String, Object>{
              'isolateId': '1',
              'reports': <Object>['Coverage'],
              'forceCompile': true,
              'reportLines': true,
              'libraryFilters': <String>['package:foo/'],
              'librariesAlreadyCompiled': <String>['package:foo/foo.dart'],
            },
            jsonResponse:
                SourceReport(
                  ranges: <SourceReportRange>[
                    SourceReportRange(
                      scriptIndex: 0,
                      startPos: 0,
                      endPos: 0,
                      compiled: true,
                      coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[]),
                    ),
                  ],
                  scripts: <ScriptRef>[ScriptRef(uri: 'package:foo/foo.dart', id: '1')],
                ).toJson(),
          ),
        ],
      );

      final Map<String, Set<int>> coverableLineCache = <String, Set<int>>{
        'package:foo/foo.dart': <int>{1, 2, 3},
      };
      final Map<String, Object?> result2 = await collect(
        Uri(),
        <String>{'foo'},
        serviceOverride: fakeVmServiceHost.vmService,
        coverableLineCache: coverableLineCache,
      );

      // Expect that line 2 is marked as missed, even though it wasn't mentioned
      // in the getSourceReport response.
      expect(result2, <String, Object>{
        'type': 'CodeCoverage',
        'coverage': <Object>[
          <String, Object>{
            'source': 'package:foo/foo.dart',
            'script': <String, Object>{
              'type': '@Script',
              'fixedId': true,
              'id': 'libraries/1/scripts/package%3Afoo%2Ffoo.dart',
              'uri': 'package:foo/foo.dart',
              '_kind': 'library',
            },
            'hits': <Object>[1, 1, 2, 0, 3, 1],
          },
        ],
      });
      expect(coverableLineCache, <String, Set<int>>{
        'package:foo/foo.dart': <int>{1, 2, 3},
      });

      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
  );
}

File writeFooBarPackagesJson(Directory tempDir) {
  final File file = File('${tempDir.path}/packages.json');
  file.writeAsStringSync(
    jsonEncode(<String, dynamic>{
      'configVersion': 2,
      'packages': <Map<String, String>>[
        <String, String>{'name': 'foo', 'rootUri': 'foo'},
        <String, String>{'name': 'bar', 'rootUri': 'bar'},
      ],
    }),
  );
  return file;
}

FakeVmServiceHost createFakeVmServiceHostWithFooAndBar({
  List<String>? libraryFilters,
  List<String> librariesAlreadyCompiled = const <String>[],
}) {
  return FakeVmServiceHost(
    requests: <VmServiceExpectation>[
      FakeVmServiceRequest(
        method: 'getVM',
        jsonResponse:
            (VM.parse(<String, Object>{})!
                  ..isolates = <IsolateRef>[
                    IsolateRef.parse(<String, Object>{'id': '1'})!,
                  ])
                .toJson(),
      ),
      FakeVmServiceRequest(
        method: 'getSourceReport',
        args: <String, Object>{
          'isolateId': '1',
          'reports': <Object>['Coverage'],
          'forceCompile': true,
          'reportLines': true,
          'librariesAlreadyCompiled': librariesAlreadyCompiled,
          if (libraryFilters != null) 'libraryFilters': libraryFilters,
        },
        jsonResponse:
            SourceReport(
              ranges: <SourceReportRange>[
                SourceReportRange(
                  scriptIndex: 0,
                  startPos: 0,
                  endPos: 0,
                  compiled: true,
                  coverage: SourceReportCoverage(hits: <int>[1, 3], misses: <int>[2]),
                ),
                SourceReportRange(
                  scriptIndex: 1,
                  startPos: 0,
                  endPos: 0,
                  compiled: true,
                  coverage: SourceReportCoverage(hits: <int>[47, 21], misses: <int>[32, 86]),
                ),
              ],
              scripts: <ScriptRef>[
                ScriptRef(uri: 'package:foo/foo.dart', id: '1'),
                ScriptRef(uri: 'package:bar/bar.dart', id: '2'),
              ],
            ).toJson(),
      ),
    ],
  );
}

class TestTestDevice extends TestDevice {
  @override
  Future<void> get finished => Future<void>.delayed(const Duration(seconds: 1));

  @override
  Future<void> kill() => Future<void>.value();

  @override
  Future<Uri?> get vmServiceUri => Future<Uri?>.value(Uri());

  @override
  Future<StreamChannel<String>> start(String entrypointPath) {
    throw UnimplementedError();
  }
}
