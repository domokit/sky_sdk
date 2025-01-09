// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/test.dart';

import '../sdk_rewriter.dart';

void main() {
  test('handles exports correctly in the engine library file', () {
    const String source = '''
// Comment 1

library engine;

// Comment 2

export 'engine/file1.dart';
export'engine/file2.dart';
export      'engine/file3.dart';
''';

    const String expected = '''
// Comment 1

@JS()
library dart._engine;

import 'dart:async';
import 'dart:collection';
import 'dart:convert' hide Codec;
import 'dart:developer' as developer;
import 'dart:js_util' as js_util;
import 'dart:_js_annotations';
import 'dart:js_interop' hide JS;
import 'dart:js_interop_unsafe';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:extra';


// Comment 2

part 'engine/file1.dart';
part 'engine/file2.dart';
part 'engine/file3.dart';
''';

    final String result = processSource(
      source,
      (String source) =>
          validateApiFile('/path/to/lib/web_ui/lib/src/engine.dart', source, 'engine'),
      generateApiFilePatterns('engine', false, <String>["import 'dart:extra';"]),
    );
    expect(result, expected);
  });

  test('underscore is not added to library name for public library in API file', () {
    const String source = '''
library engine;
''';

    const String expected = '''
@JS()
library dart.engine;

import 'dart:async';
import 'dart:collection';
import 'dart:convert' hide Codec;
import 'dart:developer' as developer;
import 'dart:js_util' as js_util;
import 'dart:_js_annotations';
import 'dart:js_interop' hide JS;
import 'dart:js_interop_unsafe';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:extra';

''';

    final String result = processSource(
      source,
      (String source) =>
          validateApiFile('/path/to/lib/web_ui/lib/src/engine.dart', source, 'engine'),
      generateApiFilePatterns('engine', true, <String>["import 'dart:extra';"]),
    );
    expect(result, expected);
  });

  test('complains about non-compliant engine.dart file', () {
    const String source = '''
library engine;

import 'dart:something';
export 'engine/file1.dart';
export 'engine/file3.dart';
''';

    Object? caught;
    try {
      processSource(
        source,
        (String source) =>
            validateApiFile('/path/to/lib/web_ui/lib/src/engine.dart', source, 'engine'),
        generateApiFilePatterns('engine', false, <String>[]),
      );
    } catch (error) {
      caught = error;
    }
    expect(caught, isA<Exception>());
    expect(
      '$caught',
      'Exception: on line 3: unexpected code in /path/to/lib/web_ui/lib/src/engine.dart. '
          'This file may only contain comments and exports. Found:\n'
          "import 'dart:something';",
    );
  });

  test('removes imports/exports from engine files', () {
    const String source = '''
import 'package:some_package/some_package.dart';
import 'package:some_package/some_package/foo.dart';
import 'package:some_package/some_package' as some_package;

import 'file1.dart';
import'file2.dart';
import      'file3.dart';

export 'file4.dart';
export'file5.dart';
export      'file6.dart';

void printSomething() {
  print('something');
}
''';

    const String expected = '''
part of dart._engine;



void printSomething() {
  print('something');
}
''';

    final String result = processSource(
      source,
      (String source) => preprocessPartFile(source, 'engine'),
      generatePartsPatterns('engine', false),
    );
    expect(result, expected);
  });

  test('gets correct extra imports', () {
    // Root libraries.
    expect(getExtraImportsForLibrary('engine'), <String>[
      "import 'dart:_skwasm_impl' if (dart.library.html) 'dart:_skwasm_stub';",
      "import 'dart:ui_web' as ui_web;",
      "import 'dart:_web_unicode';",
      "import 'dart:_web_test_fonts';",
      "import 'dart:_web_locale_keymap' as locale_keymap;",
    ]);
    expect(getExtraImportsForLibrary('skwasm_stub'), <String>[
      "import 'dart:ui_web' as ui_web;",
      "import 'dart:_engine';",
      "import 'dart:_web_unicode';",
      "import 'dart:_web_test_fonts';",
      "import 'dart:_web_locale_keymap' as locale_keymap;",
    ]);
    expect(getExtraImportsForLibrary('skwasm_impl'), <String>[
      "import 'dart:ui_web' as ui_web;",
      "import 'dart:_engine';",
      "import 'dart:_web_unicode';",
      "import 'dart:_web_test_fonts';",
      "import 'dart:_web_locale_keymap' as locale_keymap;",
      "import 'dart:_wasm';",
    ]);

    // Other libraries (should not have extra imports).
    expect(getExtraImportsForLibrary('web_unicode'), isEmpty);
    expect(getExtraImportsForLibrary('web_test_fonts'), isEmpty);
    expect(getExtraImportsForLibrary('web_locale_keymap'), isEmpty);
  });

  test('allows imports to line-break', () {
    const String source = '''
import 'package:some_package/some_package.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart'
    if (dart.library.html) 'package:ui/src/engine/skwasm/skwasm_stub.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart'
    if (dart.library.js_interop) 'package:ui/src/engine/skwasm/skwasm_stub.dart';
import 'package:some_package/some_package' as some_package;

void printSomething() {
  print('something');
}
''';

    const String expected = '''
part of dart._engine;

void printSomething() {
  print('something');
}
''';

    final String result = processSource(
      source,
      (String source) => preprocessPartFile(source, 'engine'),
      generatePartsPatterns('engine', false),
    );
    expect(result, expected);
  });
}
