// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

const String registry =
    'https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry';

/// A script to generate a Dart cache of https://www.iana.org. This should be
/// run occasionally. It was created since iana.org was found to be flaky.
///
/// To execute: dart gen_subtag_registry.dart > language_subtag_registry.dart
Future<void> main() async {
  final HttpClient client = HttpClient();
  final HttpClientRequest request = await client.getUrl(Uri.parse(registry));
  final HttpClientResponse response = await request.close();
  final String body =
      (await response.cast<List<int>>().transform<String>(utf8.decoder).toList()).join();
  final File subtagRegistry = File('../language_subtag_registry.dart');
  final File subtagRegistryFlutterTools = File(
    '../../../../packages/flutter_tools/lib/src/localizations/language_subtag_registry.dart',
  );

  final String content = '''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Cache of $registry.
const String languageSubtagRegistry = \'\'\'$body\'\'\';''';

  subtagRegistry.writeAsStringSync(content);
  subtagRegistryFlutterTools.writeAsStringSync(content);

  client.close(force: true);
}
