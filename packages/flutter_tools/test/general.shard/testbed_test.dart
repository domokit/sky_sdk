// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/context.dart';
import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../src/common.dart';
import '../src/testbed.dart';

void main() {
  group('Testbed', () {
    test('Can provide default interfaces', () async {
      final Testbed testbed = Testbed();

      late FileSystem localFileSystem;
      await testbed.run(() {
        localFileSystem = globals.fs;
      });

      expect(localFileSystem, isA<ErrorHandlingFileSystem>());
      expect((localFileSystem as ErrorHandlingFileSystem).fileSystem, isA<MemoryFileSystem>());
    });

    test('Can provide setup interfaces', () async {
      final Testbed testbed = Testbed(overrides: <Type, Generator>{A: () => A()});

      A? instance;
      await testbed.run(() {
        instance = context.get<A>();
      });

      expect(instance, isA<A>());
    });

    test('Can provide local overrides', () async {
      final Testbed testbed = Testbed(overrides: <Type, Generator>{A: () => A()});

      A? instance;
      await testbed.run(() {
        instance = context.get<A>();
      }, overrides: <Type, Generator>{A: () => B()});

      expect(instance, isA<B>());
    });

    test('provides a mocked http client', () async {
      final Testbed testbed = Testbed();
      await testbed.run(() async {
        final HttpClient client = HttpClient();
        final HttpClientRequest request = await client.getUrl(Uri.parse('http://foo.dev'));
        final HttpClientResponse response = await request.close();

        expect(response.statusCode, HttpStatus.ok);
        expect(response.contentLength, 0);
      });
    });

    test('Throws StateError if Timer is left pending', () async {
      final Testbed testbed = Testbed();

      expect(
        testbed.run(() async {
          Timer.periodic(const Duration(seconds: 1), (Timer timer) {});
        }),
        throwsStateError,
      );
    });

    test("Doesn't throw a StateError if Timer is left cleaned up", () async {
      final Testbed testbed = Testbed();

      await testbed.run(() async {
        final Timer timer = Timer.periodic(const Duration(seconds: 1), (Timer timer) {});
        timer.cancel();
      });
    });

    test('Throws if ProcessUtils is injected', () {
      final Testbed testbed = Testbed(overrides: <Type, Generator>{ProcessUtils: () => null});

      expect(() => testbed.run(() {}), throwsStateError);
    });
  });
}

class A {}

class B extends A {}
