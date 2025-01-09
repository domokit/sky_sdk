// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:html/dom.dart';
import 'package:html/parser.dart';

import 'base/common.dart';
import 'base/file_system.dart';

/// Placeholder for base href
const String kBaseHrefPlaceholder = r'$FLUTTER_BASE_HREF';

class WebTemplateWarning {
  WebTemplateWarning(this.warningText, this.lineNumber);
  final String warningText;
  final int lineNumber;
}

/// Utility class for parsing and performing operations on the contents of the
/// index.html file.
///
/// For example, to parse the base href from the index.html file:
///
/// ```dart
/// String parseBaseHref(File indexHtmlFile) {
///   final IndexHtml indexHtml = IndexHtml(indexHtmlFile.readAsStringSync());
///   return indexHtml.getBaseHref();
/// }
/// ```
class WebTemplate {
  WebTemplate(this._content);

  String get content => _content;
  String _content;

  Document _getDocument() => parse(_content);

  /// Parses the base href from the index.html file.
  String getBaseHref() {
    final Element? baseElement = _getDocument().querySelector('base');
    final String? baseHref =
        baseElement?.attributes == null ? null : baseElement!.attributes['href'];

    if (baseHref == null || baseHref == kBaseHrefPlaceholder) {
      return '';
    }

    if (!baseHref.startsWith('/')) {
      throwToolExit(
        'Error: The base href in "web/index.html" must be absolute (i.e. start '
        'with a "/"), but found: `${baseElement!.outerHtml}`.\n'
        '$_kBasePathExample',
      );
    }

    if (!baseHref.endsWith('/')) {
      throwToolExit(
        'Error: The base href in "web/index.html" must end with a "/", but found: `${baseElement!.outerHtml}`.\n'
        '$_kBasePathExample',
      );
    }

    return stripLeadingSlash(stripTrailingSlash(baseHref));
  }

  List<WebTemplateWarning> getWarnings() {
    return <WebTemplateWarning>[
      ..._getWarningsForPattern(
        RegExp('(const|var) serviceWorkerVersion = null'),
        'Local variable for "serviceWorkerVersion" is deprecated. Use "{{flutter_service_worker_version}}" template token instead. See https://docs.flutter.dev/platform-integration/web/initialization for more details.',
      ),
      ..._getWarningsForPattern(
        "navigator.serviceWorker.register('flutter_service_worker.js')",
        'Manual service worker registration deprecated. Use flutter.js service worker bootstrapping instead. See https://docs.flutter.dev/platform-integration/web/initialization for more details.',
      ),
      ..._getWarningsForPattern(
        '_flutter.loader.loadEntrypoint(',
        '"FlutterLoader.loadEntrypoint" is deprecated. Use "FlutterLoader.load" instead. See https://docs.flutter.dev/platform-integration/web/initialization for more details.',
      ),
    ];
  }

  List<WebTemplateWarning> _getWarningsForPattern(Pattern pattern, String warningText) {
    return <WebTemplateWarning>[
      for (final Match match in pattern.allMatches(_content))
        _getWarningForMatch(match, warningText),
    ];
  }

  WebTemplateWarning _getWarningForMatch(Match match, String warningText) {
    final int lineCount =
        RegExp(r'(\r\n|\r|\n)').allMatches(_content.substring(0, match.start)).length;
    return WebTemplateWarning(warningText, lineCount + 1);
  }

  /// Applies substitutions to the content of the index.html file.
  void applySubstitutions({
    required String baseHref,
    required String? serviceWorkerVersion,
    required File flutterJsFile,
    String? buildConfig,
    String? flutterBootstrapJs,
  }) {
    if (_content.contains(kBaseHrefPlaceholder)) {
      _content = _content.replaceAll(kBaseHrefPlaceholder, baseHref);
    }

    if (serviceWorkerVersion != null) {
      _content = _content
          .replaceFirst(
            // Support older `var` syntax as well as new `const` syntax
            RegExp('(const|var) serviceWorkerVersion = null'),
            'const serviceWorkerVersion = "$serviceWorkerVersion"',
          )
          // This is for legacy index.html that still uses the old service
          // worker loading mechanism.
          .replaceFirst(
            "navigator.serviceWorker.register('flutter_service_worker.js')",
            "navigator.serviceWorker.register('flutter_service_worker.js?v=$serviceWorkerVersion')",
          );
    }
    _content = _content.replaceAll(
      '{{flutter_service_worker_version}}',
      serviceWorkerVersion != null ? '"$serviceWorkerVersion"' : 'null',
    );
    if (buildConfig != null) {
      _content = _content.replaceAll('{{flutter_build_config}}', buildConfig);
    }

    if (_content.contains('{{flutter_js}}')) {
      _content = _content.replaceAll('{{flutter_js}}', flutterJsFile.readAsStringSync());
    }

    if (flutterBootstrapJs != null) {
      _content = _content.replaceAll('{{flutter_bootstrap_js}}', flutterBootstrapJs);
    }
  }
}

/// Strips the leading slash from a path.
String stripLeadingSlash(String path) {
  while (path.startsWith('/')) {
    path = path.substring(1);
  }
  return path;
}

/// Strips the trailing slash from a path.
String stripTrailingSlash(String path) {
  while (path.endsWith('/')) {
    path = path.substring(0, path.length - 1);
  }
  return path;
}

const String _kBasePathExample = '''
For example, to serve from the root use:

    <base href="/">

To serve from a subpath "foo" (i.e. http://localhost:8080/foo/ instead of http://localhost:8080/) use:

    <base href="/foo/">

For more information, see: https://developer.mozilla.org/en-US/docs/Web/HTML/Element/base
''';
