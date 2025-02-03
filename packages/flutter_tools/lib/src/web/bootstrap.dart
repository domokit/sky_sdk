// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';

/// Used to load prerequisite scripts such as ddc_module_loader.js
const String _simpleLoaderScript = r'''
window.$dartCreateScript = (function() {
  // Find the nonce value. (Note, this is only computed once.)
  var scripts = Array.from(document.getElementsByTagName("script"));
  var nonce;
  scripts.some(
      script => (nonce = script.nonce || script.getAttribute("nonce")));
  // If present, return a closure that automatically appends the nonce.
  if (nonce) {
    return function() {
      var script = document.createElement("script");
      script.nonce = nonce;
      return script;
    };
  } else {
    return function() {
      return document.createElement("script");
    };
  }
})();

// Loads a module [relativeUrl] relative to [root].
//
// If not specified, [root] defaults to the directory serving the main app.
var forceLoadModule = function (relativeUrl, root) {
  var actualRoot = root ?? _currentDirectory;
  return new Promise(function(resolve, reject) {
    var script = self.$dartCreateScript();
    let policy = {
      createScriptURL: function(src) {return src;}
    };
    if (self.trustedTypes && self.trustedTypes.createPolicy) {
      policy = self.trustedTypes.createPolicy('dartDdcModuleUrl', policy);
    }
    script.onload = resolve;
    script.onerror = reject;
    script.src = policy.createScriptURL(actualRoot + relativeUrl);
    document.head.appendChild(script);
  });
};
''';

// TODO(srujzs): Delete this once it's no longer used internally.
String generateDDCBootstrapScript({
  required String entrypoint,
  required String ddcModuleLoaderUrl,
  required String mapperUrl,
  required bool generateLoadingIndicator,
  String appRootDirectory = '/',
}) {
  return '''
${generateLoadingIndicator ? _generateLoadingIndicator() : ""}
// TODO(markzipan): This is safe if Flutter app roots are always equal to the
// host root '/'. Validate if this is true.
var _currentDirectory = "$appRootDirectory";

$_simpleLoaderScript

// A map containing the URLs for the bootstrap scripts in debug.
let _scriptUrls = {
  "mapper": "$mapperUrl",
  "moduleLoader": "$ddcModuleLoaderUrl"
};

(function() {
  let appName = "$entrypoint";

  // A uuid that identifies a subapp.
  // Stubbed out since subapps aren't supported in Flutter.
  let uuid = "00000000-0000-0000-0000-000000000000";

  window.postMessage(
      {type: "DDC_STATE_CHANGE", state: "initial_load", targetUuid: uuid}, "*");

  // Load pre-requisite DDC scripts.
  // We intentionally use invalid names to avoid namespace clashes.
  let prerequisiteScripts = [
    {
      "src": "$ddcModuleLoaderUrl",
      "id": "ddc_module_loader \x00"
    },
    {
      "src": "$mapperUrl",
      "id": "dart_stack_trace_mapper \x00"
    }
  ];

  // Load ddc_module_loader.js to access DDC's module loader API.
  let prerequisiteLoads = [];
  for (let i = 0; i < prerequisiteScripts.length; i++) {
    prerequisiteLoads.push(forceLoadModule(prerequisiteScripts[i].src));
  }
  Promise.all(prerequisiteLoads).then((_) => afterPrerequisiteLogic());

  // Save the current script so we can access it in a closure.
  var _currentScript = document.currentScript;

  var afterPrerequisiteLogic = function() {
    window.\$dartLoader.rootDirectories.push(_currentDirectory);
    let scripts = [
      {
        "src": "dart_sdk.js",
        "id": "dart_sdk"
      },
      {
        "src": "main_module.bootstrap.js",
        "id": "data-main"
      }
    ];
    let loadConfig = new window.\$dartLoader.LoadConfiguration();
    loadConfig.bootstrapScript = scripts[scripts.length - 1];

    loadConfig.loadScriptFn = function(loader) {
      loader.addScriptsToQueue(scripts, null);
      loader.loadEnqueuedModules();
    }
    loadConfig.ddcEventForLoadStart = /* LOAD_ALL_MODULES_START */ 1;
    loadConfig.ddcEventForLoadedOk = /* LOAD_ALL_MODULES_END_OK */ 2;
    loadConfig.ddcEventForLoadedError = /* LOAD_ALL_MODULES_END_ERROR */ 3;

    let loader = new window.\$dartLoader.DDCLoader(loadConfig);

    // Record prerequisite scripts' fully resolved URLs.
    prerequisiteScripts.forEach(script => loader.registerScript(script));

    // Note: these variables should only be used in non-multi-app scenarios since
    // they can be arbitrarily overridden based on multi-app load order.
    window.\$dartLoader.loadConfig = loadConfig;
    window.\$dartLoader.loader = loader;
    loader.nextAttempt();
  }
})();
''';
}

String generateDDCLibraryBundleBootstrapScript({
  required String entrypoint,
  required String ddcModuleLoaderUrl,
  required String mapperUrl,
  required bool generateLoadingIndicator,
  required bool isWindows,
}) {
  return '''
${generateLoadingIndicator ? _generateLoadingIndicator() : ""}
// Save the current directory so we can access it in a closure.
var _currentDirectory = (function () {
  var _url = document.currentScript.src;
  var lastSlash = _url.lastIndexOf('/');
  if (lastSlash == -1) return _url;
  var currentDirectory = _url.substring(0, lastSlash + 1);
  return currentDirectory;
})();

$_simpleLoaderScript

(function() {
  let appName = "org-dartlang-app:/$entrypoint";

  // Load pre-requisite DDC scripts. We intentionally use invalid names to avoid
  // namespace clashes.
  let prerequisiteScripts = [
    {
      "src": "$ddcModuleLoaderUrl",
      "id": "ddc_module_loader \x00"
    },
    {
      "src": "$mapperUrl",
      "id": "dart_stack_trace_mapper \x00"
    }
  ];

  // Load ddc_module_loader.js to access DDC's module loader API.
  let prerequisiteLoads = [];
  for (let i = 0; i < prerequisiteScripts.length; i++) {
    prerequisiteLoads.push(forceLoadModule(prerequisiteScripts[i].src));
  }
  Promise.all(prerequisiteLoads).then((_) => afterPrerequisiteLogic());

  // Save the current script so we can access it in a closure.
  var _currentScript = document.currentScript;

  // Create a policy if needed to load the files during a hot restart.
  let policy = {
    createScriptURL: function(src) {return src;}
  };
  if (self.trustedTypes && self.trustedTypes.createPolicy) {
    policy = self.trustedTypes.createPolicy('dartDdcModuleUrl', policy);
  }

  var afterPrerequisiteLogic = function() {
    window.\$dartLoader.rootDirectories.push(_currentDirectory);
    let scripts = [
      {
        "src": "dart_sdk.js",
        "id": "dart_sdk"
      },
      {
        "src": "main_module.bootstrap.js",
        "id": "data-main"
      }
    ];

    let loadConfig = new window.\$dartLoader.LoadConfiguration();
    // TODO(srujzs): Verify this is sufficient for Windows.
    loadConfig.isWindows = $isWindows;
    loadConfig.bootstrapScript = scripts[scripts.length - 1];

    loadConfig.loadScriptFn = function(loader) {
      loader.addScriptsToQueue(scripts, null);
      loader.loadEnqueuedModules();
    }
    loadConfig.ddcEventForLoadStart = /* LOAD_ALL_MODULES_START */ 1;
    loadConfig.ddcEventForLoadedOk = /* LOAD_ALL_MODULES_END_OK */ 2;
    loadConfig.ddcEventForLoadedError = /* LOAD_ALL_MODULES_END_ERROR */ 3;

    let loader = new window.\$dartLoader.DDCLoader(loadConfig);

    // Record prerequisite scripts' fully resolved URLs.
    prerequisiteScripts.forEach(script => loader.registerScript(script));

    // Note: these variables should only be used in non-multi-app scenarios
    // since they can be arbitrarily overridden based on multi-app load order.
    window.\$dartLoader.loadConfig = loadConfig;
    window.\$dartLoader.loader = loader;

    // Begin loading libraries
    loader.nextAttempt();

    // Set up stack trace mapper.
    if (window.\$dartStackTraceUtility &&
        !window.\$dartStackTraceUtility.ready) {
      window.\$dartStackTraceUtility.ready = true;
      window.\$dartStackTraceUtility.setSourceMapProvider(function(url) {
        var baseUrl = window.location.protocol + '//' + window.location.host;
        url = url.replace(baseUrl + '/', '');
        if (url == 'dart_sdk.js') {
          return dartDevEmbedder.debugger.getSourceMap('dart_sdk');
        }
        url = url.replace(".lib.js", "");
        return dartDevEmbedder.debugger.getSourceMap(url);
      });
    }

    let currentUri = _currentScript.src;
    // We should have written a file containing all the scripts that need to be
    // reloaded into the page. This is then read when a hot restart is triggered
    // in DDC via the `\$dartReloadModifiedModules` callback.
    let restartScripts = _currentDirectory + 'restart_scripts.json';
    // Flutter tools should write a file containing the scripts and libraries
    // that need to be hot reloaded. This is read in DWDS when a hot reload is
    // triggered.
    // TODO(srujzs): Ideally, this should be passed to the
    // `FrontendServerDdcLibraryBundleStrategyProvider` instead. See
    // https://github.com/dart-lang/webdev/issues/2584 for more details.
    let reloadScripts = _currentDirectory + 'reload_scripts.json';
    window.\$reloadScriptsPath = reloadScripts;

    if (!window.\$dartReloadModifiedModules) {
      window.\$dartReloadModifiedModules = (function(appName, callback) {
        var xhttp = new XMLHttpRequest();
        xhttp.withCredentials = true;
        xhttp.onreadystatechange = function() {
          // https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest/readyState
          if (this.readyState == 4 && this.status == 200 || this.status == 304) {
            var scripts = JSON.parse(this.responseText);
            var numToLoad = 0;
            var numLoaded = 0;
            for (var i = 0; i < scripts.length; i++) {
              var script = scripts[i];
              if (script.id == null) continue;
              var src = _currentDirectory + script.src.toString();
              var oldSrc = window.\$dartLoader.moduleIdToUrl.get(script.id);
              // Only compare the search parameters which contain the cache
              // busting portion of the uri. The path might be different if the
              // script is loaded from a different application on the page.
              if (window.\$dartLoader.moduleIdToUrl.has(script.id) &&
                  new URL(oldSrc).search == new URL(src).search) continue;

              // We might actually load from a different uri, delete the old one
              // just to be sure.
              window.\$dartLoader.urlToModuleId.delete(oldSrc);

              window.\$dartLoader.moduleIdToUrl.set(script.id, src);
              window.\$dartLoader.urlToModuleId.set(src, script.id);

              numToLoad++;

              var el = document.getElementById(script.id);
              if (el) el.remove();
              el = window.\$dartCreateScript();
              el.src = policy.createScriptURL(src);
              el.async = false;
              el.defer = true;
              el.id = script.id;
              el.onload = function() {
                numLoaded++;
                if (numToLoad == numLoaded) callback();
              };
              document.head.appendChild(el);
            }
            // Call `callback` right away if we found no updated scripts.
            if (numToLoad == 0) callback();
          }
        };
        xhttp.open("GET", restartScripts, true);
        xhttp.send();
      });
    }
  };
})();
''';
}

/// The JavaScript bootstrap script to support in-browser hot restart.
///
/// The [requireUrl] loads our cached RequireJS script file. The [mapperUrl]
/// loads the special Dart stack trace mapper. The [entrypoint] is the
/// actual main.dart file.
///
/// This file is served when the browser requests "main.dart.js" in debug mode,
/// and is responsible for bootstrapping the RequireJS modules and attaching
/// the hot reload hooks.
///
/// If `generateLoadingIndicator` is true, embeds a loading indicator onto the
/// web page that's visible while the Flutter app is loading.
String generateBootstrapScript({
  required String requireUrl,
  required String mapperUrl,
  required bool generateLoadingIndicator,
}) {
  return '''
"use strict";

${generateLoadingIndicator ? _generateLoadingIndicator() : ''}

// A map containing the URLs for the bootstrap scripts in debug.
let _scriptUrls = {
  "mapper": "$mapperUrl",
  "requireJs": "$requireUrl"
};

// Create a TrustedTypes policy so we can attach Scripts...
let _ttPolicy;
if (window.trustedTypes) {
  _ttPolicy = trustedTypes.createPolicy("flutter-tools-bootstrap", {
    createScriptURL: (url) => {
      let scriptUrl = _scriptUrls[url];
      if (!scriptUrl) {
        console.error("Unknown Flutter Web bootstrap resource!", url);
      }
      return scriptUrl;
    }
  });
}

// Creates a TrustedScriptURL for a given `scriptName`.
// See `_scriptUrls` and `_ttPolicy` above.
function getTTScriptUrl(scriptName) {
  let defaultUrl = _scriptUrls[scriptName];
  return _ttPolicy ? _ttPolicy.createScriptURL(scriptName) : defaultUrl;
}

// Attach source mapping.
var mapperEl = document.createElement("script");
mapperEl.defer = true;
mapperEl.async = false;
mapperEl.src = getTTScriptUrl("mapper");
document.head.appendChild(mapperEl);

// Attach require JS.
var requireEl = document.createElement("script");
requireEl.defer = true;
requireEl.async = false;
requireEl.src = getTTScriptUrl("requireJs");
// This attribute tells require JS what to load as main (defined below).
requireEl.setAttribute("data-main", "main_module.bootstrap");
document.head.appendChild(requireEl);
''';
}

/// Creates a visual animated loading indicator and puts it on the page to
/// provide feedback to the developer that the app is being loaded. Otherwise,
/// the developer would be staring at a blank page wondering if the app will
/// come up or not.
///
/// This indicator should only be used when DWDS is enabled, e.g. with the
/// `-d chrome` option. Debug builds without DWDS, e.g. `flutter run -d web-server`
/// or `flutter build web --debug` should not use this indicator.
String _generateLoadingIndicator() {
  return '''
var styles = `
  .flutter-loader {
    width: 100%;
    height: 8px;
    background-color: #13B9FD;
    position: absolute;
    top: 0px;
    left: 0px;
    overflow: hidden;
  }

  .indeterminate {
      position: relative;
      width: 100%;
      height: 100%;
  }

  .indeterminate:before {
      content: '';
      position: absolute;
      height: 100%;
      background-color: #0175C2;
      animation: indeterminate_first 2.0s infinite ease-out;
  }

  .indeterminate:after {
      content: '';
      position: absolute;
      height: 100%;
      background-color: #02569B;
      animation: indeterminate_second 2.0s infinite ease-in;
  }

  @keyframes indeterminate_first {
      0% {
          left: -100%;
          width: 100%;
      }
      100% {
          left: 100%;
          width: 10%;
      }
  }

  @keyframes indeterminate_second {
      0% {
          left: -150%;
          width: 100%;
      }
      100% {
          left: 100%;
          width: 10%;
      }
  }
`;

var styleSheet = document.createElement("style")
styleSheet.type = "text/css";
styleSheet.innerText = styles;
document.head.appendChild(styleSheet);

var loader = document.createElement('div');
loader.className = "flutter-loader";
document.body.append(loader);

var indeterminate = document.createElement('div');
indeterminate.className = "indeterminate";
loader.appendChild(indeterminate);

document.addEventListener('dart-app-ready', function (e) {
   loader.parentNode.removeChild(loader);
   styleSheet.parentNode.removeChild(styleSheet);
});
''';
}

// TODO(srujzs): Delete this once it's no longer used internally.
String generateDDCMainModule({
  required String entrypoint,
  required bool nullAssertions,
  required bool nativeNullAssertions,
  String? exportedMain,
}) {
  final String entrypointMainName = exportedMain ?? entrypoint.split('.')[0];
  // The typo below in "EXTENTION" is load-bearing, package:build depends on it.
  return '''
/* ENTRYPOINT_EXTENTION_MARKER */

(function() {
  // Flutter Web uses a generated main entrypoint, which shares app and module names.
  let appName = "$entrypoint";
  let moduleName = "$entrypoint";

  // Use a dummy UUID since multi-apps are not supported on Flutter Web.
  let uuid = "00000000-0000-0000-0000-000000000000";

  let child = {};
  child.main = function() {
    let dart = self.dart_library.import('dart_sdk', appName).dart;
    dart.nonNullAsserts($nullAssertions);
    dart.nativeNonNullAsserts($nativeNullAssertions);
    self.dart_library.start(appName, uuid, moduleName, "$entrypointMainName");
  }

  /* MAIN_EXTENSION_MARKER */
  child.main();
})();
''';
}

String generateDDCLibraryBundleMainModule({
  required String entrypoint,
  required bool nullAssertions,
  required bool nativeNullAssertions,
}) {
  // The typo below in "EXTENTION" is load-bearing, package:build depends on it.
  return '''
/* ENTRYPOINT_EXTENTION_MARKER */

(function() {
  let appName = "org-dartlang-app:/$entrypoint";

  dartDevEmbedder.debugger.registerDevtoolsFormatter();

  let child = {};
  child.main = function() {
    let sdkOptions = {
      nonNullAsserts: $nullAssertions,
      nativeNonNullAsserts: $nativeNullAssertions,
    };
    dartDevEmbedder.runMain(appName, sdkOptions);
  }

  /* MAIN_EXTENSION_MARKER */
  child.main();
})();
''';
}

/// Generate a synthetic main module which captures the application's main
/// method.
///
/// If a [bootstrapModule] name is not provided, defaults to 'main_module.bootstrap'.
///
/// RE: Object.keys usage in app.main:
/// This attaches the main entrypoint and hot reload functionality to the window.
/// The app module will have a single property which contains the actual application
/// code. The property name is based off of the entrypoint that is generated, for example
/// the file `foo/bar/baz.dart` will generate a property named approximately
/// `foo__bar__baz`. Rather than attempt to guess, we assume the first property of
/// this object is the module.
String generateMainModule({
  required String entrypoint,
  required bool nullAssertions,
  required bool nativeNullAssertions,
  String bootstrapModule = 'main_module.bootstrap',
  String loaderRootDirectory = '',
}) {
  // The typo below in "EXTENTION" is load-bearing, package:build depends on it.
  return '''
/* ENTRYPOINT_EXTENTION_MARKER */
// Disable require module timeout
require.config({
  waitSeconds: 0
});
// Create the main module loaded below.
define("$bootstrapModule", ["$entrypoint", "dart_sdk"], function(app, dart_sdk) {
  dart_sdk.dart.setStartAsyncSynchronously(true);
  dart_sdk._debugger.registerDevtoolsFormatter();
  dart_sdk.dart.nonNullAsserts($nullAssertions);
  dart_sdk.dart.nativeNonNullAsserts($nativeNullAssertions);

  // See the generateMainModule doc comment.
  var child = {};
  child.main = app[Object.keys(app)[0]].main;

  /* MAIN_EXTENSION_MARKER */
  child.main();

  window.\$dartLoader = {};
  window.\$dartLoader.rootDirectories = ["$loaderRootDirectory"];
  if (window.\$requireLoader) {
    window.\$requireLoader.getModuleLibraries = dart_sdk.dart.getModuleLibraries;
  }
  if (window.\$dartStackTraceUtility && !window.\$dartStackTraceUtility.ready) {
    window.\$dartStackTraceUtility.ready = true;
    let dart = dart_sdk.dart;
    window.\$dartStackTraceUtility.setSourceMapProvider(function(url) {
      var baseUrl = window.location.protocol + '//' + window.location.host;
      url = url.replace(baseUrl + '/', '');
      if (url == 'dart_sdk.js') {
        return dart.getSourceMap('dart_sdk');
      }
      url = url.replace(".lib.js", "");
      return dart.getSourceMap(url);
    });
  }
  // Prevent DDC's requireJS to interfere with modern bundling.
  if (typeof define === 'function' && define.amd) {
    // Preserve a copy just in case...
    define._amd = define.amd;
    delete define.amd;
  }
});
''';
}

typedef WebTestInfo = ({String entryPoint, Uri goldensUri, String? configFile});

/// Generates the bootstrap logic required for running a group of unit test
/// files in the browser.
///
/// This creates one "switchboard" main function that imports all the main
/// functions of the unit test files that need to be run. The javascript code
/// that starts the test sets a `window.testSelector` that specifies which main
/// function to invoke. This allows us to compile all the unit test files as a
/// single web application and invoke that with a different selector for each
/// test.
String generateTestEntrypoint({
  required List<WebTestInfo> testInfos,
  required LanguageVersion languageVersion,
}) {
  final List<String> importMainStatements = <String>[];
  final List<String> importTestConfigStatements = <String>[];
  final List<String> webTestPairs = <String>[];

  for (int index = 0; index < testInfos.length; index++) {
    final WebTestInfo testInfo = testInfos[index];
    final String entryPointPath = testInfo.entryPoint;
    importMainStatements.add(
      "import 'org-dartlang-app:///${Uri.file(entryPointPath)}' as test_$index show main;",
    );

    final String? testConfigPath = testInfo.configFile;
    String? testConfigFunction = 'null';
    if (testConfigPath != null) {
      importTestConfigStatements.add(
        "import 'org-dartlang-app:///${Uri.file(testConfigPath)}' as test_config_$index show testExecutable;",
      );
      testConfigFunction = 'test_config_$index.testExecutable';
    }
    webTestPairs.add('''
  '$entryPointPath': (
    entryPoint: test_$index.main,
    entryPointRunner: $testConfigFunction,
    goldensUri: Uri.parse('${testInfo.goldensUri}'),
  ),
''');
  }
  return '''
// @dart = ${languageVersion.major}.${languageVersion.minor}

${importMainStatements.join('\n')}

${importTestConfigStatements.join('\n')}

import 'package:flutter_test/flutter_test.dart';

Map<String, WebTest> webTestMap = <String, WebTest>{
  ${webTestPairs.join('\n')}
};

Future<void> main() {
  final WebTest? webTest = webTestMap[testSelector];
  if (webTest == null) {
    throw Exception('Web test for \${testSelector} not found');
  }
  return runWebTest(webTest);
}
  ''';
}

/// Generate the unit test bootstrap file.
String generateTestBootstrapFileContents(String mainUri, String requireUrl, String mapperUrl) {
  return '''
(function() {
  if (typeof document != 'undefined') {
    var el = document.createElement("script");
    el.defer = true;
    el.async = false;
    el.src = '$mapperUrl';
    document.head.appendChild(el);

    el = document.createElement("script");
    el.defer = true;
    el.async = false;
    el.src = '$requireUrl';
    el.setAttribute("data-main", '$mainUri');
    document.head.appendChild(el);
  } else {
    importScripts('$mapperUrl', '$requireUrl');
    require.config({
      baseUrl: baseUrl,
    });
    window = self;
    require(['$mainUri']);
  }
})();
''';
}

String generateDefaultFlutterBootstrapScript() {
  return '''
{{flutter_js}}
{{flutter_build_config}}

_flutter.loader.load({
  serviceWorkerSettings: {
    serviceWorkerVersion: {{flutter_service_worker_version}}
  }
});
''';
}
