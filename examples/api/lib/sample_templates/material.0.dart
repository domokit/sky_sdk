// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a template file for illustrating best practices when creating a
// Flutter API example that uses the Material library. To use it, copy the file
// to your destination filename, search/replace 'Placeholder' with the name of
// the class this is an example for, delete this block, and implement your
// example.
//
// The name and location of this file should be:
//
//   examples/api/lib/<library>/<filename_without_dart>/<lower_snake_symbol>.<index>.dart
//
// So, if your example is the third example of the Foo.bar function in the
// "baz.dart" file in the material library, then the filename for your example
// should be:
//
//   examples/api/lib/material/baz/foo_bar.2.dart
//
// and its associated test should be in:
//
//   examples/api/test/material/baz/foo_bar.2_test.dart
//
// The following doc comment should remain, and be a doc comment so that the
// symbol will be linked in the IDE. Don't use the whole description of the
// example, since that should already be in the API docs where this example is
// referenced, and we don't want the two descriptions to diverge. If this sample
// is referenced more than once, link back to the instance the example file is
// named for.

import 'package:flutter/material.dart';

/// Flutter code sample for [Placeholder].

void main() {
  runApp(const SampleApp());
}

class SampleApp extends StatelessWidget {
  const SampleApp({super.key});
  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: PlaceholderExample()));
  }
}

/// Include comments about each class, and make them dartdoc comments, so that
/// links (e.g. [Placeholder]) are active in IDEs.
///
/// Name the classes appropriately for the example (don't leave it as
/// "PlaceholderExample"!).
class PlaceholderExample extends StatelessWidget {
  const PlaceholderExample({super.key});

  @override
  Widget build(BuildContext context) {
    // Since this is an example, add plenty of comments, explaining things that
    // both a newcomer and an experienced user might want to know.
    //
    // TRY THIS: Prefix things with "TRY THIS" in places in the example that
    // might be interesting to modify when exploring what the code does.
    return const Placeholder();
  }
}
