// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_constrained_cross_axis.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverConstrainedCrossAxis example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverConstrainedCrossAxisExampleApp());

    final RenderSliverList renderSliverList = tester.renderObject(find.byType(SliverList));
    expect(renderSliverList.constraints.crossAxisExtent, equals(200));
  });
}
