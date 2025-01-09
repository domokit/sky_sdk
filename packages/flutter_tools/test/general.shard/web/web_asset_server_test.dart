// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/isolated/devfs_web.dart';
import 'package:shelf/shelf.dart';

import '../../src/common.dart';

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

final Platform platform = FakePlatform(environment: <String, String>{'HOME': '/'});

void main() {
  late FileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fileSystem.file('lib/main.dart').createSync(recursive: true);
    fileSystem.file('web/index.html')
      ..createSync(recursive: true)
      ..writeAsStringSync('hello');
    fileSystem.file('build/flutter_assets/foo.png')
      ..createSync(recursive: true)
      ..writeAsBytesSync(kTransparentImage);
    fileSystem.file('build/flutter_assets/bar')
      ..createSync(recursive: true)
      ..writeAsBytesSync(<int>[1, 2, 3]);
  });

  testWithoutContext(
    'release asset server serves correct mime type and content length for png',
    () async {
      final ReleaseAssetServer assetServer = ReleaseAssetServer(
        Uri.base,
        fileSystem: fileSystem,
        platform: platform,
        flutterRoot: '/flutter',
        webBuildDirectory: 'build/web',
        needsCoopCoep: false,
      );
      fileSystem.file('build/web/assets/foo.png')
        ..createSync(recursive: true)
        ..writeAsBytesSync(kTransparentImage);
      final Response response = await assetServer.handle(
        Request('GET', Uri.parse('http://localhost:8080/assets/foo.png')),
      );

      expect(response.headers, <String, String>{
        'Content-Type': 'image/png',
        'Cross-Origin-Resource-Policy': 'cross-origin',
        'Access-Control-Allow-Origin': '*',
        'content-length': '64',
      });
    },
  );

  testWithoutContext(
    'release asset server serves correct mime type and content length for JavaScript',
    () async {
      final ReleaseAssetServer assetServer = ReleaseAssetServer(
        Uri.base,
        fileSystem: fileSystem,
        platform: platform,
        flutterRoot: '/flutter',
        webBuildDirectory: 'build/web',
        needsCoopCoep: false,
      );
      fileSystem.file('build/web/assets/foo.js')
        ..createSync(recursive: true)
        ..writeAsStringSync('function main() {}');
      final Response response = await assetServer.handle(
        Request('GET', Uri.parse('http://localhost:8080/assets/foo.js')),
      );

      expect(response.headers, <String, String>{
        'Content-Type': 'text/javascript',
        'Cross-Origin-Resource-Policy': 'cross-origin',
        'Access-Control-Allow-Origin': '*',
        'content-length': '18',
      });
    },
  );

  testWithoutContext(
    'release asset server serves correct mime type and content length for html',
    () async {
      final ReleaseAssetServer assetServer = ReleaseAssetServer(
        Uri.base,
        fileSystem: fileSystem,
        platform: platform,
        flutterRoot: '/flutter',
        webBuildDirectory: 'build/web',
        needsCoopCoep: false,
      );
      fileSystem.file('build/web/assets/foo.html')
        ..createSync(recursive: true)
        ..writeAsStringSync('<!doctype html><html></html>');
      final Response response = await assetServer.handle(
        Request('GET', Uri.parse('http://localhost:8080/assets/foo.html')),
      );

      expect(response.headers, <String, String>{
        'Content-Type': 'text/html',
        'Cross-Origin-Resource-Policy': 'cross-origin',
        'Access-Control-Allow-Origin': '*',
        'content-length': '28',
      });
    },
  );

  testWithoutContext('release asset server serves content from flutter root', () async {
    final ReleaseAssetServer assetServer = ReleaseAssetServer(
      Uri.base,
      fileSystem: fileSystem,
      platform: platform,
      flutterRoot: '/flutter',
      webBuildDirectory: 'build/web',
      needsCoopCoep: false,
    );
    fileSystem.file('flutter/bar.dart')
      ..createSync(recursive: true)
      ..writeAsStringSync('void main() { }');
    final Response response = await assetServer.handle(
      Request('GET', Uri.parse('http://localhost:8080/flutter/bar.dart')),
    );

    expect(response.statusCode, HttpStatus.ok);
  });

  testWithoutContext('release asset server serves content from project directory', () async {
    final ReleaseAssetServer assetServer = ReleaseAssetServer(
      Uri.base,
      fileSystem: fileSystem,
      platform: platform,
      flutterRoot: '/flutter',
      webBuildDirectory: 'build/web',
      needsCoopCoep: false,
    );
    fileSystem.file('bar.dart')
      ..createSync(recursive: true)
      ..writeAsStringSync('void main() { }');
    final Response response = await assetServer.handle(
      Request('GET', Uri.parse('http://localhost:8080/bar.dart')),
    );

    expect(response.statusCode, HttpStatus.ok);
  });

  testWithoutContext(
    'release asset server serves html content with COOP/COEP headers when specified',
    () async {
      final ReleaseAssetServer assetServer = ReleaseAssetServer(
        Uri.base,
        fileSystem: fileSystem,
        platform: platform,
        flutterRoot: '/flutter',
        webBuildDirectory: 'build/web',
        needsCoopCoep: true,
      );
      fileSystem.file('build/web/index.html')
        ..createSync(recursive: true)
        ..writeAsStringSync('<html></html>');
      final Response response = await assetServer.handle(
        Request('GET', Uri.parse('http://localhost:8080/index.html')),
      );

      expect(response.statusCode, HttpStatus.ok);
      final Map<String, String> headers = response.headers;
      expect(headers['Cross-Origin-Opener-Policy'], 'same-origin');
      expect(headers['Cross-Origin-Embedder-Policy'], 'credentialless');
    },
  );

  testWithoutContext(
    'release asset server serves html content without COOP/COEP headers when specified',
    () async {
      final ReleaseAssetServer assetServer = ReleaseAssetServer(
        Uri.base,
        fileSystem: fileSystem,
        platform: platform,
        flutterRoot: '/flutter',
        webBuildDirectory: 'build/web',
        needsCoopCoep: false,
      );
      fileSystem.file('build/web/index.html')
        ..createSync(recursive: true)
        ..writeAsStringSync('<html></html>');
      final Response response = await assetServer.handle(
        Request('GET', Uri.parse('http://localhost:8080/index.html')),
      );

      expect(response.statusCode, HttpStatus.ok);
      final Map<String, String> headers = response.headers;
      expect(headers.containsKey('Cross-Origin-Opener-Policy'), false);
      expect(headers.containsKey('Cross-Origin-Embedder-Policy'), false);
    },
  );
}
