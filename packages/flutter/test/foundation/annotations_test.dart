// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('test Category constructor', () {
    const List<String> sections = <String>['First section', 'Second section', 'Third section'];
    const Category category = Category(sections);
    expect(category.sections, sections);
  });
  test('test DocumentationIcon constructor', () {
    const DocumentationIcon docIcon = DocumentationIcon('Test String');
    expect(docIcon.url, contains('Test String'));
  });

  test('test Summary constructor', () {
    const Summary summary = Summary('Test String');
    expect(summary.text, contains('Test String'));
  });
}
