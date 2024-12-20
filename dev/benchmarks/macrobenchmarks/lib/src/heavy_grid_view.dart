// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class HeavyGridViewPage extends StatelessWidget {
  const HeavyGridViewPage({super.key});

  @override
  Widget build(BuildContext context) {
    return GridView.builder(
      itemCount: 1000,
      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
      itemBuilder: (BuildContext context, int index) => HeavyWidget(index),
    ).build(context);
  }
}

class HeavyWidget extends StatelessWidget {
  HeavyWidget(this.index) : super(key: ValueKey<int>(index));

  final int index;
  final List<int> _weight = List<int>.filled(1000000, 0);

  @override
  Widget build(BuildContext context) {
    return SizedBox(width: 200, height: 200, child: Text('$index: ${_weight.length}'));
  }
}
