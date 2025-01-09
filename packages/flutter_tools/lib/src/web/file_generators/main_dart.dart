// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';

/// Generates the main.dart file.
String generateMainDartFile(
  String appEntrypoint, {
  required String pluginRegistrantEntrypoint,
  LanguageVersion? languageVersion,
}) {
  return <String>[
    if (languageVersion != null) '// @dart=${languageVersion.major}.${languageVersion.minor}',
    '// Flutter web bootstrap script for $appEntrypoint.',
    '//',
    '// Generated file. Do not edit.',
    '//',
    '',
    '// ignore_for_file: type=lint',
    '',
    "import 'dart:ui_web' as ui_web;",
    "import 'dart:async';",
    '',
    "import '$appEntrypoint' as entrypoint;",
    "import '$pluginRegistrantEntrypoint' as pluginRegistrant;",
    '',
    'typedef _UnaryFunction = dynamic Function(List<String> args);',
    'typedef _NullaryFunction = dynamic Function();',
    '',
    'Future<void> main() async {',
    '  await ui_web.bootstrapEngine(',
    '    runApp: () {',
    '      if (entrypoint.main is _UnaryFunction) {',
    '        return (entrypoint.main as _UnaryFunction)(<String>[]);',
    '      }',
    '      return (entrypoint.main as _NullaryFunction)();',
    '    },',
    '    registerPlugins: () {',
    '      pluginRegistrant.registerPlugins();',
    '    },',
    '  );',
    '}',
    '',
  ].join('\n');
}
