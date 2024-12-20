// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/semantics.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SemanticsEvent.toString', () {
    expect(TestSemanticsEvent().toString(), 'TestSemanticsEvent()');
    expect(TestSemanticsEvent(number: 10).toString(), 'TestSemanticsEvent(number: 10)');
    expect(TestSemanticsEvent(text: 'hello').toString(), 'TestSemanticsEvent(text: hello)');
    expect(
      TestSemanticsEvent(text: 'hello', number: 10).toString(),
      'TestSemanticsEvent(number: 10, text: hello)',
    );
  });
  test('SemanticsEvent.toMap', () {
    expect(TestSemanticsEvent(text: 'hi', number: 11).toMap(), <String, dynamic>{
      'type': 'TestEvent',
      'data': <String, dynamic>{'text': 'hi', 'number': 11},
    });
    expect(TestSemanticsEvent(text: 'hi', number: 11).toMap(nodeId: 123), <String, dynamic>{
      'type': 'TestEvent',
      'nodeId': 123,
      'data': <String, dynamic>{'text': 'hi', 'number': 11},
    });
  });
  test('FocusSemanticEvent.toMap', () {
    expect(const FocusSemanticEvent().toMap(), <String, dynamic>{
      'type': 'focus',
      'data': <String, dynamic>{},
    });
  });
}

class TestSemanticsEvent extends SemanticsEvent {
  TestSemanticsEvent({this.text, this.number}) : super('TestEvent');

  final String? text;
  final int? number;

  @override
  Map<String, dynamic> getDataMap() {
    final Map<String, dynamic> result = <String, dynamic>{};
    if (text != null) {
      result['text'] = text;
    }
    if (number != null) {
      result['number'] = number;
    }
    return result;
  }
}
