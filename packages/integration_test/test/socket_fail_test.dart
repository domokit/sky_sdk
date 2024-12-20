// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

class SocketExceptionHttpClient extends Fake implements HttpClient {
  @override
  Future<HttpClientRequest> openUrl(String method, Uri url) {
    throw const SocketException('always throw');
  }
}

Future<void> main() async {
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  test('Prints an appropriate message on socket exception', () async {
    bool gotStateError = false;
    try {
      await binding.enableTimeline(httpClient: SocketExceptionHttpClient());
    } on StateError catch (e) {
      gotStateError = true;
      expect(e.toString(), contains('This may happen if DDS is enabled'));
    } on SocketException catch (_) {
      fail('Did not expect a socket exception.');
    }
    expect(gotStateError, true);
  });
}
