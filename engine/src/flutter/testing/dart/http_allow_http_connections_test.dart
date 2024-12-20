// Copyright 2021 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:test/test.dart';

import 'http_disallow_http_connections_test.dart';

void main() {
  test('Normal HTTP request succeeds', () async {
    final String host = await getLocalHostIP();
    await bindServerAndTest(host, (HttpClient httpClient, Uri uri) async {
      await httpClient.getUrl(uri);
    });
  });

  test('We can ban HTTP explicitly.', () async {
    final String host = await getLocalHostIP();
    await bindServerAndTest(host, (HttpClient httpClient, Uri uri) async {
      asyncExpectThrows<UnsupportedError>(
        () async => runZoned(
          () => httpClient.getUrl(uri),
          zoneValues: <dynamic, dynamic>{#flutter.io.allow_http: false},
        ),
      );
    });
  });
}
