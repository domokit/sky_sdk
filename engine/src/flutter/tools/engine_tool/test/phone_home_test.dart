// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_tool/src/phone_home.dart';
import 'package:test/test.dart';

void main() {
  test('can et phone home', () async {
    final bool emptyResult = phoneHome(List<String>.empty());
    expect(emptyResult, isFalse);

    final bool buildResult = phoneHome(<String>['build']);
    expect(buildResult, isFalse);

    final bool phoneHomeResult = phoneHome(<String>['Phone', 'Home']);
    expect(phoneHomeResult, isTrue);
  });
}
