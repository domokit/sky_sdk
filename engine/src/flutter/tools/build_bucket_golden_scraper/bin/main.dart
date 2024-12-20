// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:build_bucket_golden_scraper/build_bucket_golden_scraper.dart';

void main(List<String> arguments) async {
  final int result;
  try {
    result = await BuildBucketGoldenScraper.fromCommandLine(arguments).run();
  } on FormatException catch (e) {
    io.stderr.writeln(e.message);
    io.exit(1);
  }
  if (result != 0) {
    io.exit(result);
  }
}
