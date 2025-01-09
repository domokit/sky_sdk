// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/web/bootstrap.dart';
import 'package:package_config/package_config.dart';

import '../../src/common.dart';

void main() {
  test('generateBootstrapScript embeds urls correctly', () {
    final String result = generateBootstrapScript(
      requireUrl: 'require.js',
      mapperUrl: 'mapper.js',
      generateLoadingIndicator: true,
    );
    // require js source is interpolated correctly.
    expect(result, contains('"requireJs": "require.js"'));
    expect(result, contains('requireEl.src = getTTScriptUrl("requireJs");'));
    // stack trace mapper source is interpolated correctly.
    expect(result, contains('"mapper": "mapper.js"'));
    expect(result, contains('mapperEl.src = getTTScriptUrl("mapper");'));
    // data-main is set to correct bootstrap module.
    expect(result, contains('requireEl.setAttribute("data-main", "main_module.bootstrap");'));
  });

  test('generateBootstrapScript includes loading indicator', () {
    final String result = generateBootstrapScript(
      requireUrl: 'require.js',
      mapperUrl: 'mapper.js',
      generateLoadingIndicator: true,
    );
    expect(result, contains('"flutter-loader"'));
    expect(result, contains('"indeterminate"'));
  });

  test('generateBootstrapScript does not include loading indicator', () {
    final String result = generateBootstrapScript(
      requireUrl: 'require.js',
      mapperUrl: 'mapper.js',
      generateLoadingIndicator: false,
    );
    expect(result, isNot(contains('"flutter-loader"')));
    expect(result, isNot(contains('"indeterminate"')));
  });

  // https://github.com/flutter/flutter/issues/107742
  test('generateBootstrapScript loading indicator does not trigger scrollbars', () {
    final String result = generateBootstrapScript(
      requireUrl: 'require.js',
      mapperUrl: 'mapper.js',
      generateLoadingIndicator: true,
    );

    // See: https://regexr.com/6q0ft
    final RegExp regex = RegExp(r'(?:\.flutter-loader\s*\{)[^}]+(?:overflow\:\s*hidden;)[^}]+}');

    expect(result, matches(regex), reason: '.flutter-loader must have overflow: hidden');
  });

  // https://github.com/flutter/flutter/issues/82524
  test('generateMainModule removes timeout from requireJS', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: false,
      nativeNullAssertions: false,
    );

    // See: https://regexr.com/6q0kp
    final RegExp regex = RegExp(
      r'(?:require\.config\(\{)(?:.|\s(?!\}\);))*'
      r'(?:waitSeconds\:\s*0[,]?)'
      r'(?:(?!\}\);).|\s)*\}\);',
    );

    expect(
      result,
      matches(regex),
      reason: 'require.config must have a waitSeconds: 0 config entry',
    );
  });

  test('generateMainModule hides requireJS injected by DDC', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: false,
      nativeNullAssertions: false,
    );
    expect(
      result,
      contains('''define._amd = define.amd;'''),
      reason: 'define.amd must be preserved as _amd so users may restore it if needed.',
    );
    expect(
      result,
      contains('''delete define.amd;'''),
      reason: "define.amd must be removed so packages don't attempt to use Dart's instance.",
    );
  });

  test('generateMainModule embeds urls correctly', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: false,
      nativeNullAssertions: false,
    );
    // bootstrap main module has correct defined module.
    expect(
      result,
      contains(
        'define("main_module.bootstrap", ["foo/bar/main.js", "dart_sdk"], '
        'function(app, dart_sdk) {',
      ),
    );
  });

  test('generateMainModule can set bootstrap name', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: false,
      nativeNullAssertions: false,
      bootstrapModule: 'foo_module.bootstrap',
    );
    // bootstrap main module has correct defined module.
    expect(
      result,
      contains(
        'define("foo_module.bootstrap", ["foo/bar/main.js", "dart_sdk"], '
        'function(app, dart_sdk) {',
      ),
    );
  });

  test('generateMainModule includes null safety switches', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: true,
      nativeNullAssertions: true,
    );

    expect(result, contains('''dart_sdk.dart.nonNullAsserts(true);'''));
    expect(result, contains('''dart_sdk.dart.nativeNonNullAsserts(true);'''));
  });

  test('generateMainModule can disable null safety switches', () {
    final String result = generateMainModule(
      entrypoint: 'foo/bar/main.js',
      nullAssertions: false,
      nativeNullAssertions: false,
    );

    expect(result, contains('''dart_sdk.dart.nonNullAsserts(false);'''));
    expect(result, contains('''dart_sdk.dart.nativeNonNullAsserts(false);'''));
  });

  test('generateTestBootstrapFileContents embeds urls correctly', () {
    final String result = generateTestBootstrapFileContents(
      'foo.dart.js',
      'require.js',
      'mapper.js',
    );

    expect(result, contains('el.setAttribute("data-main", \'foo.dart.js\');'));
  });

  test('generateTestEntrypoint generates proper imports and mappings for tests', () {
    final String result = generateTestEntrypoint(
      testInfos: <WebTestInfo>[
        (entryPoint: 'foo.dart', goldensUri: Uri.parse('foo.dart'), configFile: null),
        (entryPoint: 'bar.dart', goldensUri: Uri.parse('bar.dart'), configFile: 'bar_config.dart'),
      ],
      languageVersion: LanguageVersion(2, 8),
    );

    expect(result, contains("import 'org-dartlang-app:///foo.dart'"));
    expect(result, contains("import 'org-dartlang-app:///bar.dart'"));
    expect(result, contains("import 'org-dartlang-app:///bar_config.dart'"));
  });

  group('Using the DDC module system', () {
    test('generateDDCBootstrapScript embeds urls correctly', () {
      final String result = generateDDCBootstrapScript(
        entrypoint: 'foo/bar/main.js',
        ddcModuleLoaderUrl: 'ddc_module_loader.js',
        mapperUrl: 'mapper.js',
        generateLoadingIndicator: true,
      );
      // ddc module loader js source is interpolated correctly.
      expect(result, contains('"moduleLoader": "ddc_module_loader.js"'));
      expect(result, contains('"src": "ddc_module_loader.js"'));
      // stack trace mapper source is interpolated correctly.
      expect(result, contains('"mapper": "mapper.js"'));
      expect(result, contains('"src": "mapper.js"'));
      // data-main is set to correct bootstrap module.
      expect(result, contains('"src": "main_module.bootstrap.js"'));
      expect(result, contains('"id": "data-main"'));
    });

    test('generateDDCBootstrapScript initializes configuration objects', () {
      final String result = generateDDCBootstrapScript(
        entrypoint: 'foo/bar/main.js',
        ddcModuleLoaderUrl: 'ddc_module_loader.js',
        mapperUrl: 'mapper.js',
        generateLoadingIndicator: true,
      );
      // LoadConfiguration and DDCLoader objects must be constructed.
      expect(result, contains(r'new window.$dartLoader.LoadConfiguration('));
      expect(result, contains(r'new window.$dartLoader.DDCLoader('));
      // Specific fields must be set on the LoadConfiguration.
      expect(result, contains('.bootstrapScript ='));
      expect(result, contains('.loadScriptFn ='));
      // DDCLoader.nextAttempt must be invoked to begin loading.
      expect(result, contains('nextAttempt()'));
      // Proper window objects are initialized.
      expect(result, contains(r'window.$dartLoader.loadConfig ='));
      expect(result, contains(r'window.$dartLoader.loader ='));
    });

    test('generateDDCBootstrapScript includes loading indicator', () {
      final String result = generateDDCBootstrapScript(
        entrypoint: 'foo/bar/main.js',
        ddcModuleLoaderUrl: 'ddc_module_loader.js',
        mapperUrl: 'mapper.js',
        generateLoadingIndicator: true,
      );
      expect(result, contains('"flutter-loader"'));
      expect(result, contains('"indeterminate"'));
    });

    test('generateDDCBootstrapScript does not include loading indicator', () {
      final String result = generateDDCBootstrapScript(
        entrypoint: 'foo/bar/main.js',
        ddcModuleLoaderUrl: 'ddc_module_loader.js',
        mapperUrl: 'mapper.js',
        generateLoadingIndicator: false,
      );
      expect(result, isNot(contains('"flutter-loader"')));
      expect(result, isNot(contains('"indeterminate"')));
    });

    // https://github.com/flutter/flutter/issues/107742
    test('generateDDCBootstrapScript loading indicator does not trigger scrollbars', () {
      final String result = generateDDCBootstrapScript(
        entrypoint: 'foo/bar/main.js',
        ddcModuleLoaderUrl: 'ddc_module_loader.js',
        mapperUrl: 'mapper.js',
        generateLoadingIndicator: true,
      );

      // See: https://regexr.com/6q0ft
      final RegExp regex = RegExp(r'(?:\.flutter-loader\s*\{)[^}]+(?:overflow\:\s*hidden;)[^}]+}');

      expect(result, matches(regex), reason: '.flutter-loader must have overflow: hidden');
    });

    test('generateDDCMainModule embeds the entrypoint correctly', () {
      final String result = generateDDCMainModule(
        entrypoint: 'main.js',
        nullAssertions: false,
        nativeNullAssertions: false,
      );
      // bootstrap main module has correct defined module.
      expect(result, contains('let appName = "main.js"'));
      expect(result, contains('let moduleName = "main.js"'));
      expect(result, contains('dart_library.start(appName, uuid, moduleName, "main");'));
    });

    test('generateDDCMainModule embeds its exported main correctly', () {
      final String result = generateDDCMainModule(
        entrypoint: 'foo/bar/main.js',
        nullAssertions: false,
        nativeNullAssertions: false,
        exportedMain: 'foo__bar__main',
      );
      // bootstrap main module has correct defined module.
      expect(result, contains('let appName = "foo/bar/main.js"'));
      expect(result, contains('let moduleName = "foo/bar/main.js"'));
      expect(result, contains('dart_library.start(appName, uuid, moduleName, "foo__bar__main");'));
    });

    test('generateDDCMainModule includes null safety switches', () {
      final String result = generateDDCMainModule(
        entrypoint: 'main.js',
        nullAssertions: true,
        nativeNullAssertions: true,
      );

      expect(result, contains('''dart.nonNullAsserts(true);'''));
      expect(result, contains('''dart.nativeNonNullAsserts(true);'''));
    });

    test('generateDDCMainModule can disable null safety switches', () {
      final String result = generateDDCMainModule(
        entrypoint: 'main.js',
        nullAssertions: false,
        nativeNullAssertions: false,
      );

      expect(result, contains('''dart.nonNullAsserts(false);'''));
      expect(result, contains('''dart.nativeNonNullAsserts(false);'''));
    });

    test('generateTestBootstrapFileContents embeds urls correctly', () {
      final String result = generateTestBootstrapFileContents(
        'foo.dart.js',
        'require.js',
        'mapper.js',
      );

      expect(result, contains('el.setAttribute("data-main", \'foo.dart.js\');'));
    });
  });
}
