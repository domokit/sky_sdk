// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/86198
  SliverAppBar sliverAppBar = SliverAppBar();
  sliverAppBar = SliverAppBar(brightness: Brightness.light);
  sliverAppBar = SliverAppBar(brightness: Brightness.dark);
  sliverAppBar = SliverAppBar(error: '');
  sliverAppBar.brightness;

  TextTheme myTextTheme = TextTheme();
  SliverAppBar sliverAppBar = SliverAppBar();
  sliverAppBar = SliverAppBar(textTheme: myTextTheme);

  SliverAppBar sliverAppBar = SliverAppBar();
  sliverAppBar = SliverAppBar(backwardsCompatibility: true);
  sliverAppBar = SliverAppBar(backwardsCompatibility: false);
  sliverAppBar
      .backwardsCompatibility; // Removing field reference not supported.
}
