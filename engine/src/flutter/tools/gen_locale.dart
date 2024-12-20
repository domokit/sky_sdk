// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is used to generate the switch statements in the Locale class.
// See: ../lib/ui/window.dart

// When running this script, use the output of this script to update the
// comments that say when the script was last run (that comment appears twice in
// window.dart), and then replace all the map entries with the output from
// this script (the first set for _canonicalizeLanguageCode and the second set
// for _canonicalizeRegionCode).

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert';
import 'dart:io';

const String registry =
    'https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry';

Map<String, List<String>> parseSection(String section) {
  final Map<String, List<String>> result = <String, List<String>>{};
  late List<String> lastHeading;
  for (final String line in section.split('\n')) {
    if (line == '') {
      continue;
    }
    if (line.startsWith('  ')) {
      lastHeading[lastHeading.length - 1] = '${lastHeading.last}${line.substring(1)}';
      continue;
    }
    final int colon = line.indexOf(':');
    if (colon <= 0) {
      throw StateError('not sure how to deal with "$line"');
    }
    final String name = line.substring(0, colon);
    final String value = line.substring(colon + 2);
    lastHeading = result.putIfAbsent(name, () => <String>[])..add(value);
  }
  return result;
}

Future<void> main() async {
  final HttpClient client = HttpClient();
  final String body =
      (await (await (await client.getUrl(Uri.parse(registry))).close())
              .transform(utf8.decoder)
              .toList())
          .join();
  final List<Map<String, List<String>>> sections =
      body.split('%%').map<Map<String, List<String>>>(parseSection).toList();
  final Map<String, List<String>> outputs = <String, List<String>>{
    'language': <String>[],
    'region': <String>[],
  };
  String? fileDate;
  for (final Map<String, List<String>> section in sections) {
    if (fileDate == null) {
      // first block should contain a File-Date metadata line.
      fileDate = section['File-Date']!.single;
      continue;
    }
    assert(section.containsKey('Type'), section.toString());
    final String type = section['Type']!.single;
    if ((type == 'language' || type == 'region') && (section.containsKey('Preferred-Value'))) {
      assert(section.containsKey('Subtag'), section.toString());
      final String subtag = section['Subtag']!.single;
      final List<String> descriptions = section['Description']!;
      assert(descriptions.isNotEmpty);
      assert(section.containsKey('Deprecated'));
      final String comment =
          section.containsKey('Comment')
              ? section['Comment']!.single
              : 'deprecated ${section['Deprecated']!.single}';
      final String preferredValue = section['Preferred-Value']!.single;
      outputs[type]!.add("'$subtag': '$preferredValue', // ${descriptions.join(", ")}; $comment");
    }
  }
  print('// Mappings generated for language subtag registry as of $fileDate.');
  print('// For languageCode:');
  print(outputs['language']!.join('\n'));
  print('// For regionCode:');
  print(outputs['region']!.join('\n'));
}
