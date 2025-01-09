// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' hide Directory, File;

import 'package:dwds/dwds.dart';
import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/isolated/devfs_web.dart';
import 'package:flutter_tools/src/web/compile.dart';
import 'package:flutter_tools/src/web_template.dart';
import 'package:logging/logging.dart' as logging;
import 'package:package_config/package_config.dart';
import 'package:shelf/shelf.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../../src/common.dart';
import '../../src/testbed.dart';

const List<int> kTransparentImage = <int>[
  0x89,
  0x50,
  0x4E,
  0x47,
  0x0D,
  0x0A,
  0x1A,
  0x0A,
  0x00,
  0x00,
  0x00,
  0x0D,
  0x49,
  0x48,
  0x44,
  0x52,
  0x00,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00,
  0x00,
  0x01,
  0x08,
  0x06,
  0x00,
  0x00,
  0x00,
  0x1F,
  0x15,
  0xC4,
  0x89,
  0x00,
  0x00,
  0x00,
  0x0A,
  0x49,
  0x44,
  0x41,
  0x54,
  0x78,
  0x9C,
  0x63,
  0x00,
  0x01,
  0x00,
  0x00,
  0x05,
  0x00,
  0x01,
  0x0D,
  0x0A,
  0x2D,
  0xB4,
  0x00,
  0x00,
  0x00,
  0x00,
  0x49,
  0x45,
  0x4E,
  0x44,
  0xAE,
];

void main() {
  late Testbed testbed;
  late WebAssetServer webAssetServer;
  late ReleaseAssetServer releaseAssetServer;
  late Platform linux;
  late PackageConfig packages;
  late Platform windows;
  late FakeHttpServer httpServer;
  late BufferLogger logger;
  const bool usesDdcModuleSystem = false;

  setUpAll(() async {
    packages = PackageConfig(<Package>[
      Package('flutter_tools', Uri.file('/flutter_tools/lib/').normalizePath()),
    ]);
  });

  setUp(() {
    httpServer = FakeHttpServer();
    linux = FakePlatform(environment: <String, String>{});
    windows = FakePlatform(operatingSystem: 'windows', environment: <String, String>{});
    logger = BufferLogger.test();
    testbed = Testbed(
      setup: () {
        webAssetServer = WebAssetServer(
          httpServer,
          packages,
          InternetAddress.loopbackIPv4,
          <String, String>{},
          <String, String>{},
          NullSafetyMode.unsound,
          usesDdcModuleSystem,
          webRenderer: WebRendererMode.canvaskit,
          useLocalCanvasKit: false,
        );
        releaseAssetServer = ReleaseAssetServer(
          globals.fs.file('main.dart').uri,
          fileSystem: globals.fs,
          flutterRoot: null,
          platform: FakePlatform(),
          webBuildDirectory: null,
          needsCoopCoep: false,
        );
      },
      overrides: <Type, Generator>{Logger: () => logger},
    );
  });

  test(
    '.log() reports warnings',
    () => testbed.run(() {
      const String unresolvedUriMessage = 'Unresolved uri:';
      const String otherMessage = 'Something bad happened';

      final List<logging.LogRecord> events = <logging.LogRecord>[
        logging.LogRecord(logging.Level.WARNING, unresolvedUriMessage, 'DartUri'),
        logging.LogRecord(logging.Level.WARNING, otherMessage, 'DartUri'),
      ];

      events.forEach(log);
      expect(logger.warningText, contains(unresolvedUriMessage));
      expect(logger.warningText, contains(otherMessage));
    }),
  );

  test(
    'Handles against malformed manifest',
    () => testbed.run(() async {
      final File source = globals.fs.file('source')..writeAsStringSync('main() {}');
      final File sourcemap = globals.fs.file('sourcemap')..writeAsStringSync('{}');
      final File metadata = globals.fs.file('metadata')..writeAsStringSync('{}');

      // Missing ending offset.
      final File manifestMissingOffset = globals.fs.file('manifestA')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.js': <String, Object>{
            'code': <int>[0],
            'sourcemap': <int>[0],
            'metadata': <int>[0],
          },
        }),
      );
      final File manifestOutOfBounds = globals.fs.file('manifest')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.js': <String, Object>{
            'code': <int>[0, 100],
            'sourcemap': <int>[0],
            'metadata': <int>[0],
          },
        }),
      );

      expect(webAssetServer.write(source, manifestMissingOffset, sourcemap, metadata), isEmpty);
      expect(webAssetServer.write(source, manifestOutOfBounds, sourcemap, metadata), isEmpty);
    }),
  );

  test(
    'serves JavaScript files from in memory cache',
    () => testbed.run(() async {
      final File source = globals.fs.file('source')..writeAsStringSync('main() {}');
      final File sourcemap = globals.fs.file('sourcemap')..writeAsStringSync('{}');
      final File metadata = globals.fs.file('metadata')..writeAsStringSync('{}');
      final File manifest = globals.fs.file('manifest')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.js': <String, Object>{
            'code': <int>[0, source.lengthSync()],
            'sourcemap': <int>[0, 2],
            'metadata': <int>[0, 2],
          },
        }),
      );
      webAssetServer.write(source, manifest, sourcemap, metadata);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.js')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'application/javascript'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }, overrides: <Type, Generator>{Platform: () => linux}),
  );

  test(
    'serves metadata files from in memory cache',
    () => testbed.run(() async {
      const String metadataContents = '{"name":"foo"}';
      final File source = globals.fs.file('source')..writeAsStringSync('main() {}');
      final File sourcemap = globals.fs.file('sourcemap')..writeAsStringSync('{}');
      final File metadata = globals.fs.file('metadata')..writeAsStringSync(metadataContents);
      final File manifest = globals.fs.file('manifest')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.js': <String, Object>{
            'code': <int>[0, source.lengthSync()],
            'sourcemap': <int>[0, sourcemap.lengthSync()],
            'metadata': <int>[0, metadata.lengthSync()],
          },
        }),
      );
      webAssetServer.write(source, manifest, sourcemap, metadata);

      final String? merged = await webAssetServer.metadataContents(
        'main_module.ddc_merged_metadata',
      );
      expect(merged, equals(metadataContents));

      final String? single = await webAssetServer.metadataContents('foo.js.metadata');
      expect(single, equals(metadataContents));
    }, overrides: <Type, Generator>{Platform: () => linux}),
  );

  test(
    'Removes leading slashes for valid requests to avoid requesting outside'
    ' of served directory',
    () => testbed.run(() async {
      globals.fs.file('foo.png').createSync();
      globals.fs.currentDirectory = globals.fs.directory('project_directory')..createSync();

      final File source =
          globals.fs.file(globals.fs.path.join('web', 'foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);
      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar////foo.png')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'takes base path into account when serving',
    () => testbed.run(() async {
      webAssetServer.basePath = 'base/path';

      globals.fs.file('foo.png').createSync();
      globals.fs.currentDirectory = globals.fs.directory('project_directory')..createSync();

      final File source =
          globals.fs.file(globals.fs.path.join('web', 'foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);
      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/base/path/foo.png')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'serves index.html at the base path',
    () => testbed.run(() async {
      webAssetServer.basePath = 'base/path';

      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/base/path/')),
      );

      expect(response.statusCode, HttpStatus.ok);
      expect(await response.readAsString(), htmlContent);
    }),
  );

  test(
    'serves index.html at / if href attribute is $kBaseHrefPlaceholder',
    () => testbed.run(() async {
      const String htmlContent =
          '<html><head><base href ="$kBaseHrefPlaceholder"></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/')),
      );

      expect(response.statusCode, HttpStatus.ok);
      expect(await response.readAsString(), htmlContent.replaceAll(kBaseHrefPlaceholder, '/'));
    }),
  );

  test(
    'does not serve outside the base path',
    () => testbed.run(() async {
      webAssetServer.basePath = 'base/path';

      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/')),
      );

      expect(response.statusCode, HttpStatus.notFound);
    }),
  );

  test(
    'parses base path from index.html',
    () => testbed.run(() async {
      const String htmlContent =
          '<html><head><base href="/foo/bar/"></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      final WebAssetServer webAssetServer = WebAssetServer(
        httpServer,
        packages,
        InternetAddress.loopbackIPv4,
        <String, String>{},
        <String, String>{},
        NullSafetyMode.unsound,
        usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        useLocalCanvasKit: false,
      );

      expect(webAssetServer.basePath, 'foo/bar');
    }),
  );

  test(
    'handles lack of base path in index.html',
    () => testbed.run(() async {
      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      final WebAssetServer webAssetServer = WebAssetServer(
        httpServer,
        packages,
        InternetAddress.loopbackIPv4,
        <String, String>{},
        <String, String>{},
        NullSafetyMode.unsound,
        usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        useLocalCanvasKit: false,
      );

      // Defaults to "/" when there's no base element.
      expect(webAssetServer.basePath, '');
    }),
  );

  test(
    'throws if base path is relative',
    () => testbed.run(() async {
      const String htmlContent =
          '<html><head><base href="foo/bar/"></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      expect(
        () => WebAssetServer(
          httpServer,
          packages,
          InternetAddress.loopbackIPv4,
          <String, String>{},
          <String, String>{},
          NullSafetyMode.unsound,
          usesDdcModuleSystem,
          webRenderer: WebRendererMode.canvaskit,
          useLocalCanvasKit: false,
        ),
        throwsToolExit(),
      );
    }),
  );

  test(
    'throws if base path does not end with slash',
    () => testbed.run(() async {
      const String htmlContent =
          '<html><head><base href="/foo/bar"></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      expect(
        () => WebAssetServer(
          httpServer,
          packages,
          InternetAddress.loopbackIPv4,
          <String, String>{},
          <String, String>{},
          NullSafetyMode.unsound,
          usesDdcModuleSystem,
          webRenderer: WebRendererMode.canvaskit,
          useLocalCanvasKit: false,
        ),
        throwsToolExit(),
      );
    }),
  );

  test(
    'serves JavaScript files from in memory cache not from manifest',
    () => testbed.run(() async {
      webAssetServer.writeFile('foo.js', 'main() {}');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.js')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, '9'),
          containsPair(HttpHeaders.contentTypeHeader, 'application/javascript'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, utf8.encode('main() {}'));
    }),
  );

  test(
    'serves flutter_bootstrap.js without useLocalCanvasKit',
    () => testbed.run(() async {
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/flutter_bootstrap.js')),
      );

      expect(response.statusCode, 200);
      final String body = await response.readAsString();
      expect(body, isNot(contains('useLocalCanvasKit')));
    }),
  );

  test(
    'serves flutter_bootstrap.js with useLocalCanvasKit',
    () => testbed.run(() async {
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      webAssetServer = WebAssetServer(
        httpServer,
        packages,
        InternetAddress.loopbackIPv4,
        <String, String>{},
        <String, String>{},
        NullSafetyMode.unsound,
        usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        useLocalCanvasKit: true,
      );

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/flutter_bootstrap.js')),
      );

      expect(response.statusCode, 200);
      final String body = await response.readAsString();
      expect(body, contains('"useLocalCanvasKit":true'));
    }),
  );

  test(
    'Returns notModified when the ifNoneMatch header matches the etag',
    () => testbed.run(() async {
      webAssetServer.writeFile('foo.js', 'main() {}');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.js')),
      );
      final String etag = response.headers[HttpHeaders.etagHeader]!;

      final Response cachedResponse = await webAssetServer.handleRequest(
        Request(
          'GET',
          Uri.parse('http://foobar/foo.js'),
          headers: <String, String>{HttpHeaders.ifNoneMatchHeader: etag},
        ),
      );

      expect(cachedResponse.statusCode, HttpStatus.notModified);
      expect(await cachedResponse.read().toList(), isEmpty);
    }),
  );

  test(
    'serves index.html when path is unknown',
    () => testbed.run(() async {
      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/bar/baz')),
      );

      expect(response.statusCode, HttpStatus.ok);
      expect(await response.readAsString(), htmlContent);
    }),
  );

  test(
    'does not serve outside the base path',
    () => testbed.run(() async {
      webAssetServer.basePath = 'base/path';

      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/')),
      );

      expect(response.statusCode, HttpStatus.notFound);
    }),
  );

  test(
    'does not serve index.html when path is inside assets or packages',
    () => testbed.run(() async {
      const String htmlContent = '<html><head></head><body id="test"></body></html>';
      final Directory webDir = globals.fs.currentDirectory.childDirectory('web')..createSync();
      webDir.childFile('index.html').writeAsStringSync(htmlContent);

      Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/assets/foo/bar.png')),
      );
      expect(response.statusCode, HttpStatus.notFound);

      response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/packages/foo/bar.dart.js')),
      );
      expect(response.statusCode, HttpStatus.notFound);

      webAssetServer.basePath = 'base/path';

      response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/base/path/assets/foo/bar.png')),
      );
      expect(response.statusCode, HttpStatus.notFound);

      response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/base/path/packages/foo/bar.dart.js')),
      );
      expect(response.statusCode, HttpStatus.notFound);
    }),
  );

  test(
    'serves default index.html',
    () => testbed.run(() async {
      globals.fs.file(
          globals.fs.path.join(
            globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
            'flutter.js',
          ),
        )
        ..createSync(recursive: true)
        ..writeAsStringSync('flutter.js content');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/')),
      );

      expect(response.statusCode, HttpStatus.ok);
      expect((await response.read().toList()).first, containsAllInOrder(utf8.encode('<html>')));
    }),
  );

  test(
    'handles web server paths without .lib extension',
    () => testbed.run(() async {
      final File source = globals.fs.file('source')..writeAsStringSync('main() {}');
      final File sourcemap = globals.fs.file('sourcemap')..writeAsStringSync('{}');
      final File metadata = globals.fs.file('metadata')..writeAsStringSync('{}');
      final File manifest = globals.fs.file('manifest')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.dart.lib.js': <String, Object>{
            'code': <int>[0, source.lengthSync()],
            'sourcemap': <int>[0, 2],
            'metadata': <int>[0, 2],
          },
        }),
      );
      webAssetServer.write(source, manifest, sourcemap, metadata);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.dart.js')),
      );

      expect(response.statusCode, HttpStatus.ok);
    }),
  );

  test(
    'serves JavaScript files from in memory cache on Windows',
    () => testbed.run(() async {
      final File source = globals.fs.file('source')..writeAsStringSync('main() {}');
      final File sourcemap = globals.fs.file('sourcemap')..writeAsStringSync('{}');
      final File metadata = globals.fs.file('metadata')..writeAsStringSync('{}');
      final File manifest = globals.fs.file('manifest')..writeAsStringSync(
        json.encode(<String, Object>{
          '/foo.js': <String, Object>{
            'code': <int>[0, source.lengthSync()],
            'sourcemap': <int>[0, 2],
            'metadata': <int>[0, 2],
          },
        }),
      );
      webAssetServer.write(source, manifest, sourcemap, metadata);
      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://localhost/foo.js')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'application/javascript'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }, overrides: <Type, Generator>{Platform: () => windows}),
  );

  test(
    'serves asset files from in filesystem with url-encoded paths',
    () => testbed.run(() async {
      final File source =
          globals.fs.file(
              globals.fs.path.join('build', 'flutter_assets', Uri.encodeFull('abcd象形字.png')),
            )
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);
      final Response response = await webAssetServer.handleRequest(
        Request(
          'GET',
          Uri.parse('http://foobar/assets/abcd%25E8%25B1%25A1%25E5%25BD%25A2%25E5%25AD%2597.png'),
        ),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );
  test(
    'serves files from web directory',
    () => testbed.run(() async {
      final File source =
          globals.fs.file(globals.fs.path.join('web', 'foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);
      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.png')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'serves asset files from in filesystem with known mime type on Windows',
    () => testbed.run(() async {
      final File source =
          globals.fs.file(globals.fs.path.join('build', 'flutter_assets', 'foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);
      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/assets/foo.png')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
          containsPair(HttpHeaders.etagHeader, isNotNull),
          containsPair(HttpHeaders.cacheControlHeader, 'max-age=0, must-revalidate'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }, overrides: <Type, Generator>{Platform: () => windows}),
  );

  test(
    'serves Dart files from in filesystem on Linux/macOS',
    () => testbed.run(() async {
      final File source =
          globals.fs.file('foo.dart').absolute
            ..createSync(recursive: true)
            ..writeAsStringSync('void main() {}');

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.dart')),
      );

      expect(
        response.headers,
        containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }, overrides: <Type, Generator>{Platform: () => linux}),
  );

  test(
    'serves asset files from in filesystem with known mime type',
    () => testbed.run(() async {
      final File source =
          globals.fs.file(globals.fs.path.join('build', 'flutter_assets', 'foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(kTransparentImage);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/assets/foo.png')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, source.lengthSync().toString()),
          containsPair(HttpHeaders.contentTypeHeader, 'image/png'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'serves asset files from in filesystem with known mime type and empty content',
    () => testbed.run(() async {
      final File source = globals.fs.file(globals.fs.path.join('web', 'foo.js'))
        ..createSync(recursive: true);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/foo.js')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, '0'),
          containsPair(HttpHeaders.contentTypeHeader, 'text/javascript'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'serves asset files from in filesystem with unknown mime type',
    () => testbed.run(() async {
      final File source =
          globals.fs.file(globals.fs.path.join('build', 'flutter_assets', 'foo'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(List<int>.filled(100, 0));

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/assets/foo')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, '100'),
          containsPair(HttpHeaders.contentTypeHeader, 'application/octet-stream'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'serves valid etag header for asset files with non-ascii characters',
    () => testbed.run(() async {
      globals.fs.file(globals.fs.path.join('build', 'flutter_assets', 'fooπ'))
        ..createSync(recursive: true)
        ..writeAsBytesSync(<int>[1, 2, 3]);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http://foobar/assets/fooπ')),
      );
      final String etag = response.headers[HttpHeaders.etagHeader]!;

      expect(etag.runes, everyElement(predicate((int char) => char < 255)));
    }),
  );

  test(
    'serves /packages/<package>/<path> files as if they were '
    'package:<package>/<path> uris',
    () => testbed.run(() async {
      final Uri? expectedUri = packages.resolve(Uri.parse('package:flutter_tools/foo.dart'));
      final File source =
          globals.fs.file(globals.fs.path.fromUri(expectedUri))
            ..createSync(recursive: true)
            ..writeAsBytesSync(<int>[1, 2, 3]);

      final Response response = await webAssetServer.handleRequest(
        Request('GET', Uri.parse('http:///packages/flutter_tools/foo.dart')),
      );

      expect(
        response.headers,
        allOf(<Matcher>[
          containsPair(HttpHeaders.contentLengthHeader, '3'),
          containsPair(HttpHeaders.contentTypeHeader, 'text/x-dart'),
        ]),
      );
      expect((await response.read().toList()).first, source.readAsBytesSync());
    }),
  );

  test(
    'calling dispose closes the http server',
    () => testbed.run(() async {
      await webAssetServer.dispose();

      expect(httpServer.closed, true);
    }),
  );

  test(
    'Can start web server with specified AMD module system assets',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');
      outputFile.parent.childFile('a.metadata').writeAsStringSync('{}');

      final ResidentCompiler residentCompiler =
          FakeResidentCompiler()..output = const CompilerOutput('a', 0, <Uri>[]);

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: const BuildInfo(
          BuildMode.debug,
          '',
          treeShakeIcons: false,
          nullSafetyMode: NullSafetyMode.unsound,
          packageConfigPath: '.dart_tool/package_config.json',
        ),
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.unsound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.html,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.flutterJs.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      final Uri uri = await webDevFS.create();
      webDevFS.webAssetServer.entrypointCacheDirectory = globals.fs.currentDirectory;
      final String webPrecompiledSdk =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdSdk).path;
      final String webPrecompiledSdkSourcemaps =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdSdkSourcemaps).path;
      final String webPrecompiledCanvaskitSdk =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSdk).path;
      final String webPrecompiledCanvaskitSdkSourcemaps =
          globals.artifacts!
              .getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSdkSourcemaps)
              .path;
      globals.fs.currentDirectory.childDirectory('lib').childFile('web_entrypoint.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('GENERATED');
      globals.fs.file(webPrecompiledSdk)
        ..createSync(recursive: true)
        ..writeAsStringSync('HELLO');
      globals.fs.file(webPrecompiledSdkSourcemaps)
        ..createSync(recursive: true)
        ..writeAsStringSync('THERE');
      globals.fs.file(webPrecompiledCanvaskitSdk)
        ..createSync(recursive: true)
        ..writeAsStringSync('OL');
      globals.fs.file(webPrecompiledCanvaskitSdkSourcemaps)
        ..createSync(recursive: true)
        ..writeAsStringSync('CHUM');

      await webDevFS.update(
        mainUri: globals.fs.file(globals.fs.path.join('lib', 'main.dart')).uri,
        generator: residentCompiler,
        trackWidgetCreation: true,
        bundleFirstUpload: true,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        pathToReload: '',
        dillOutputPath: 'out.dill',
        shaderCompiler: const FakeShaderCompiler(),
      );

      expect(webDevFS.webAssetServer.getFile('require.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('stack_trace_mapper.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('main.dart'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('manifest.json'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('flutter.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('flutter_service_worker.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('version.json'), isNotNull);
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js'), 'HELLO');
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js.map'), 'THERE');

      // Update to the SDK.
      globals.fs.file(webPrecompiledSdk).writeAsStringSync('BELLOW');

      // New SDK should be visible..
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js'), 'BELLOW');

      // Generated entrypoint.
      expect(
        await webDevFS.webAssetServer.dartSourceContents('web_entrypoint.dart'),
        contains('GENERATED'),
      );

      // served on localhost
      expect(uri.host, 'localhost');

      await webDevFS.destroy();
    }, overrides: <Type, Generator>{Artifacts: () => Artifacts.test()}),
  );

  test(
    'Can start web server with specified assets in sound null safety mode',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');
      outputFile.parent.childFile('a.metadata').writeAsStringSync('{}');

      final ResidentCompiler residentCompiler =
          FakeResidentCompiler()..output = const CompilerOutput('a', 0, <Uri>[]);

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: const BuildInfo(
          BuildMode.debug,
          '',
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        ),
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.sound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.html,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.flutterJs.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      final Uri uri = await webDevFS.create();
      webDevFS.webAssetServer.entrypointCacheDirectory = globals.fs.currentDirectory;
      globals.fs.currentDirectory.childDirectory('lib').childFile('web_entrypoint.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('GENERATED');
      final String webPrecompiledSdk =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdSoundSdk).path;
      final String webPrecompiledSdkSourcemaps =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdSoundSdkSourcemaps).path;
      final String webPrecompiledCanvaskitSdk =
          globals.artifacts!.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSoundSdk).path;
      final String webPrecompiledCanvaskitSdkSourcemaps =
          globals.artifacts!
              .getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSoundSdkSourcemaps)
              .path;
      final String flutterJs = globals.fs.path.join(
        globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
        'flutter.js',
      );
      globals.fs.file(webPrecompiledSdk)
        ..createSync(recursive: true)
        ..writeAsStringSync('HELLO');
      globals.fs.file(webPrecompiledSdkSourcemaps)
        ..createSync(recursive: true)
        ..writeAsStringSync('THERE');
      globals.fs.file(webPrecompiledCanvaskitSdk)
        ..createSync(recursive: true)
        ..writeAsStringSync('OL');
      globals.fs.file(webPrecompiledCanvaskitSdkSourcemaps)
        ..createSync(recursive: true)
        ..writeAsStringSync('CHUM');
      globals.fs.file(flutterJs)
        ..createSync(recursive: true)
        ..writeAsStringSync('(flutter.js content)');

      await webDevFS.update(
        mainUri: globals.fs.file(globals.fs.path.join('lib', 'main.dart')).uri,
        generator: residentCompiler,
        trackWidgetCreation: true,
        bundleFirstUpload: true,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        pathToReload: '',
        dillOutputPath: '',
        shaderCompiler: const FakeShaderCompiler(),
      );

      expect(webDevFS.webAssetServer.getFile('require.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('stack_trace_mapper.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('main.dart'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('manifest.json'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('flutter.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('flutter_service_worker.js'), isNotNull);
      expect(webDevFS.webAssetServer.getFile('version.json'), isNotNull);
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js'), 'HELLO');
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js.map'), 'THERE');

      // Update to the SDK.
      globals.fs.file(webPrecompiledSdk).writeAsStringSync('BELLOW');

      // New SDK should be visible..
      expect(await webDevFS.webAssetServer.dartSourceContents('dart_sdk.js'), 'BELLOW');

      // Generated entrypoint.
      expect(
        await webDevFS.webAssetServer.dartSourceContents('web_entrypoint.dart'),
        contains('GENERATED'),
      );

      // served on localhost
      expect(uri.host, 'localhost');

      await webDevFS.destroy();
    }, overrides: <Type, Generator>{Artifacts: () => Artifacts.test()}),
  );

  test(
    '.connect() will never call vmServiceFactory twice',
    () => testbed.run(() async {
      await FakeAsync().run<Future<void>>((FakeAsync time) {
        final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
          ..createSync(recursive: true);
        outputFile.parent.childFile('a.sources').writeAsStringSync('');
        outputFile.parent.childFile('a.json').writeAsStringSync('{}');
        outputFile.parent.childFile('a.map').writeAsStringSync('{}');
        outputFile.parent.childFile('a.metadata').writeAsStringSync('{}');

        final WebDevFS webDevFS = WebDevFS(
          // if this is any other value, we will do a real ip lookup
          hostname: 'any',
          port: 0,
          tlsCertPath: null,
          tlsCertKeyPath: null,
          packagesFilePath: '.dart_tool/package_config.json',
          urlTunneller: null,
          useSseForDebugProxy: true,
          useSseForDebugBackend: true,
          useSseForInjectedClient: true,
          nullAssertions: true,
          nativeNullAssertions: true,
          buildInfo: const BuildInfo(
            BuildMode.debug,
            '',
            treeShakeIcons: false,
            packageConfigPath: '.dart_tool/package_config.json',
          ),
          enableDwds: true,
          enableDds: false,
          entrypoint: Uri.base,
          testMode: true,
          expressionCompiler: null,
          extraHeaders: const <String, String>{},
          chromiumLauncher: null,
          nullSafetyMode: NullSafetyMode.sound,
          ddcModuleSystem: usesDdcModuleSystem,
          webRenderer: WebRendererMode.canvaskit,
          isWasm: false,
          useLocalCanvasKit: false,
          rootDirectory: globals.fs.currentDirectory,
        );
        webDevFS.requireJS.createSync(recursive: true);
        webDevFS.stackTraceMapper.createSync(recursive: true);
        final FakeAppConnection firstConnection = FakeAppConnection();
        final FakeAppConnection secondConnection = FakeAppConnection();

        final Future<void> done = webDevFS.create().then<void>((Uri _) {
          // In non-test mode, webDevFS.create() would have initialized DWDS
          webDevFS.webAssetServer.dwds = FakeDwds(<AppConnection>[
            firstConnection,
            secondConnection,
          ]);

          int vmServiceFactoryInvocationCount = 0;
          Future<vm_service.VmService> vmServiceFactory(
            Uri uri, {
            CompressionOptions? compression,
            required Logger logger,
          }) {
            if (vmServiceFactoryInvocationCount > 0) {
              fail('Called vmServiceFactory twice!');
            }
            vmServiceFactoryInvocationCount += 1;
            return Future<vm_service.VmService>.delayed(
              const Duration(seconds: 2),
              () => FakeVmService(),
            );
          }

          return webDevFS.connect(false, vmServiceFactory: vmServiceFactory).then<void>((
            ConnectionResult? firstConnectionResult,
          ) {
            return webDevFS.destroy();
          });
        });
        time.elapse(const Duration(seconds: 1));
        time.elapse(const Duration(seconds: 2));
        return done;
      });
    }, overrides: <Type, Generator>{Artifacts: () => Artifacts.test()}),
  );

  test(
    'Can start web server with hostname any',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'any',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        buildInfo: BuildInfo.debug,
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullAssertions: true,
        nativeNullAssertions: true,
        nullSafetyMode: NullSafetyMode.sound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      final Uri uri = await webDevFS.create();

      expect(uri.host, 'localhost');
      await webDevFS.destroy();
    }),
  );

  test(
    'Can start web server with canvaskit enabled',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: const BuildInfo(
          BuildMode.debug,
          '',
          treeShakeIcons: false,
          dartDefines: <String>['FLUTTER_WEB_USE_SKIA=true'],
          packageConfigPath: '.dart_tool/package_config.json',
        ),
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.sound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      await webDevFS.create();

      expect(webDevFS.webAssetServer.webRenderer, WebRendererMode.canvaskit);

      await webDevFS.destroy();
    }),
  );

  test(
    'Can start web server with auto detect enabled',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: const BuildInfo(
          BuildMode.debug,
          '',
          treeShakeIcons: false,
          dartDefines: <String>['FLUTTER_WEB_AUTO_DETECT=true'],
          packageConfigPath: '.dart_tool/package_config.json',
        ),
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.sound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.auto,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      await webDevFS.create();

      expect(webDevFS.webAssetServer.webRenderer, WebRendererMode.auto);

      await webDevFS.destroy();
    }),
  );

  test(
    'Can start web server with tls connection',
    () => testbed.run(() async {
      final String dataPath = globals.fs.path.join(
        getFlutterRoot(),
        'packages',
        'flutter_tools',
        'test',
        'data',
        'asset_test',
      );

      final String dummyCertPath = globals.fs.path.join(dataPath, 'tls_cert', 'dummy-cert.pem');
      final String dummyCertKeyPath = globals.fs.path.join(dataPath, 'tls_cert', 'dummy-key.pem');

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: dummyCertPath,
        tlsCertKeyPath: dummyCertKeyPath,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: BuildInfo.debug,
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.unsound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      final Uri uri = await webDevFS.create();

      // Ensure the connection established is secure
      expect(uri.scheme, 'https');

      await webDevFS.destroy();
    }, overrides: <Type, Generator>{Artifacts: () => Artifacts.test()}),
  );

  test('allows frame embedding', () async {
    final WebAssetServer webAssetServer = await WebAssetServer.start(
      null,
      'localhost',
      0,
      null,
      null,
      null,
      true,
      true,
      true,
      const BuildInfo(
        BuildMode.debug,
        '',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      ),
      false,
      false,
      Uri.base,
      null,
      const <String, String>{},
      NullSafetyMode.unsound,
      webRenderer: WebRendererMode.canvaskit,
      isWasm: false,
      useLocalCanvasKit: false,
      testMode: true,
    );

    expect(webAssetServer.defaultResponseHeaders['x-frame-options'], null);
    await webAssetServer.dispose();
  });

  test('passes on extra headers', () async {
    const String extraHeaderKey = 'hurray';
    const String extraHeaderValue = 'flutter';
    final WebAssetServer webAssetServer = await WebAssetServer.start(
      null,
      'localhost',
      0,
      null,
      null,
      null,
      true,
      true,
      true,
      const BuildInfo(
        BuildMode.debug,
        '',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      ),
      false,
      false,
      Uri.base,
      null,
      const <String, String>{extraHeaderKey: extraHeaderValue},
      NullSafetyMode.unsound,
      webRenderer: WebRendererMode.canvaskit,
      isWasm: false,
      useLocalCanvasKit: false,
      testMode: true,
    );

    expect(webAssetServer.defaultResponseHeaders[extraHeaderKey], <String>[extraHeaderValue]);

    await webAssetServer.dispose();
  });

  test(
    'WebAssetServer responds to POST requests with 404 not found',
    () => testbed.run(() async {
      final Response response = await webAssetServer.handleRequest(
        Request('POST', Uri.parse('http://foobar/something')),
      );
      expect(response.statusCode, 404);
    }),
  );

  test(
    'ReleaseAssetServer responds to POST requests with 404 not found',
    () => testbed.run(() async {
      final Response response = await releaseAssetServer.handle(
        Request('POST', Uri.parse('http://foobar/something')),
      );
      expect(response.statusCode, 404);
    }),
  );

  test(
    'WebAssetServer strips leading base href off of asset requests',
    () => testbed.run(() async {
      const String htmlContent =
          '<html><head><base href="/foo/"></head><body id="test"></body></html>';
      globals.fs.currentDirectory.childDirectory('web').childFile('index.html')
        ..createSync(recursive: true)
        ..writeAsStringSync(htmlContent);
      final WebAssetServer webAssetServer = WebAssetServer(
        FakeHttpServer(),
        PackageConfig.empty,
        InternetAddress.anyIPv4,
        <String, String>{},
        <String, String>{},
        NullSafetyMode.sound,
        usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        useLocalCanvasKit: false,
      );

      expect(await webAssetServer.metadataContents('foo/main_module.ddc_merged_metadata'), null);
      // Not base href.
      expect(
        () async => webAssetServer.metadataContents('bar/main_module.ddc_merged_metadata'),
        throwsException,
      );
    }),
  );

  test(
    'DevFS URI includes any specified base path.',
    () => testbed.run(() async {
      final File outputFile = globals.fs.file(globals.fs.path.join('lib', 'main.dart'))
        ..createSync(recursive: true);
      const String htmlContent =
          '<html><head><base href="/foo/"></head><body id="test"></body></html>';
      globals.fs.currentDirectory.childDirectory('web').childFile('index.html')
        ..createSync(recursive: true)
        ..writeAsStringSync(htmlContent);
      outputFile.parent.childFile('a.sources').writeAsStringSync('');
      outputFile.parent.childFile('a.json').writeAsStringSync('{}');
      outputFile.parent.childFile('a.map').writeAsStringSync('{}');
      outputFile.parent.childFile('a.metadata').writeAsStringSync('{}');

      final WebDevFS webDevFS = WebDevFS(
        hostname: 'localhost',
        port: 0,
        tlsCertPath: null,
        tlsCertKeyPath: null,
        packagesFilePath: '.dart_tool/package_config.json',
        urlTunneller: null,
        useSseForDebugProxy: true,
        useSseForDebugBackend: true,
        useSseForInjectedClient: true,
        nullAssertions: true,
        nativeNullAssertions: true,
        buildInfo: BuildInfo.debug,
        enableDwds: false,
        enableDds: false,
        entrypoint: Uri.base,
        testMode: true,
        expressionCompiler: null,
        extraHeaders: const <String, String>{},
        chromiumLauncher: null,
        nullSafetyMode: NullSafetyMode.unsound,
        ddcModuleSystem: usesDdcModuleSystem,
        webRenderer: WebRendererMode.canvaskit,
        isWasm: false,
        useLocalCanvasKit: false,
        rootDirectory: globals.fs.currentDirectory,
      );
      webDevFS.requireJS.createSync(recursive: true);
      webDevFS.stackTraceMapper.createSync(recursive: true);

      final Uri uri = await webDevFS.create();

      // served on localhost
      expect(uri.host, 'localhost');
      // Matches base URI specified in html.
      expect(uri.path, '/foo');

      await webDevFS.destroy();
    }, overrides: <Type, Generator>{Artifacts: () => Artifacts.test()}),
  );
}

class FakeHttpServer extends Fake implements HttpServer {
  bool closed = false;

  @override
  Future<void> close({bool force = false}) async {
    closed = true;
  }
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  CompilerOutput? output;

  @override
  void addFileSystemRoot(String root) {}

  @override
  Future<CompilerOutput?> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    String? outputPath,
    PackageConfig? packageConfig,
    String? projectRootPath,
    FileSystem? fs,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    Uri? nativeAssetsYaml,
  }) async {
    return output;
  }
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) {
    throw UnimplementedError();
  }
}

class FakeDwds extends Fake implements Dwds {
  FakeDwds(Iterable<AppConnection> connectedAppsIterable)
    : connectedApps = Stream<AppConnection>.fromIterable(connectedAppsIterable);

  @override
  final Stream<AppConnection> connectedApps;

  @override
  Future<DebugConnection> debugConnection(AppConnection appConnection) =>
      Future<DebugConnection>.value(FakeDebugConnection());
}

class FakeAppConnection extends Fake implements AppConnection {
  @override
  void runMain() {}
}

class FakeDebugConnection extends Fake implements DebugConnection {
  FakeDebugConnection({this.uri = 'http://foo'});

  @override
  final String uri;
}

class FakeVmService extends Fake implements vm_service.VmService {}
