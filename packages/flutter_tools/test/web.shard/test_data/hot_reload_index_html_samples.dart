// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains a bunch of different index.html "styles" that can be written
// by Flutter Web users.
// This should be somewhat kept in sync with the different index.html files present
// in `flutter/dev/integration_tests/web/web`.
// @see https://github.com/flutter/flutter/tree/main/dev/integration_tests/web/web

/// index_with_flutterjs_entrypoint_loaded.html
String indexHtmlFlutterJsCallback = _generateFlutterJsIndexHtml('''
    window.addEventListener('load', function(ev) {
      // Download main.dart.js
      _flutter.loader.loadEntrypoint({
        onEntrypointLoaded: onEntrypointLoaded,
        serviceWorker: {
          serviceWorkerVersion: serviceWorkerVersion,
        }
      });
      // Once the entrypoint is ready, do things!
      async function onEntrypointLoaded(engineInitializer) {
        const appRunner = await engineInitializer.initializeEngine();
        appRunner.runApp();
      }
    });
''');

/// index_with_flutterjs_short.html
String indexHtmlFlutterJsPromisesShort = _generateFlutterJsIndexHtml('''
    window.addEventListener('load', function(ev) {
      // Download main.dart.js
      _flutter.loader.loadEntrypoint({
        serviceWorker: {
          serviceWorkerVersion: serviceWorkerVersion,
        }
      }).then(function(engineInitializer) {
        return engineInitializer.autoStart();
      });
    });
''');

/// index_with_flutterjs.html
String indexHtmlFlutterJsPromisesFull = _generateFlutterJsIndexHtml('''
    window.addEventListener('load', function(ev) {
      // Download main.dart.js
      _flutter.loader.loadEntrypoint({
        serviceWorker: {
          serviceWorkerVersion: serviceWorkerVersion,
        }
      }).then(function(engineInitializer) {
        return engineInitializer.initializeEngine();
      }).then(function(appRunner) {
        return appRunner.runApp();
      });
    });
''');

/// index_with_flutterjs.html
String indexHtmlFlutterJsLoad = _generateFlutterJsIndexHtml('''
    window.addEventListener('load', function(ev) {
      _flutter.buildConfig = {
        builds: [
          {
            "compileTarget": "dartdevc",
            "renderer": "canvaskit",
            "mainJsPath": "main.dart.js",
          }
        ]
      };
      // Download main.dart.js
      _flutter.loader.load({
        serviceWorkerSettings: {
          serviceWorkerVersion: serviceWorkerVersion,
        },
      });
    });
''');

/// index_without_flutterjs.html
String indexHtmlNoFlutterJs = '''
<!DOCTYPE HTML>
<!-- Copyright 2014 The Flutter Authors. All rights reserved.
Use of this source code is governed by a BSD-style license that can be
found in the LICENSE file. -->
<html>
<head>
  <meta charset="UTF-8">
  <meta content="IE=Edge" http-equiv="X-UA-Compatible">

  <title>Web Test</title>
  <!-- iOS meta tags & icons -->
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black">
  <meta name="apple-mobile-web-app-title" content="Web Test">
  <link rel="manifest" href="manifest.json">
</head>
<body>
  <!-- This script installs service_worker.js to provide PWA functionality to
  application. For more information, see:
  https://developers.google.com/web/fundamentals/primers/service-workers -->
  <script>
    const serviceWorkerVersion = null;
    var scriptLoaded = false;
    function loadMainDartJs() {
      if (scriptLoaded) {
        return;
      }
      scriptLoaded = true;
      var scriptTag = document.createElement('script');
      scriptTag.src = 'main.dart.js';
      scriptTag.type = 'application/javascript';
      document.body.append(scriptTag);
    }

    if ('serviceWorker' in navigator) {
      // Service workers are supported. Use them.
      window.addEventListener('load', function () {
        // Wait for registration to finish before dropping the <script> tag.
        // Otherwise, the browser will load the script multiple times,
        // potentially different versions.
        var serviceWorkerUrl = 'flutter_service_worker.js?v=' + serviceWorkerVersion;
        navigator.serviceWorker.register(serviceWorkerUrl)
          .then((reg) => {
            function waitForActivation(serviceWorker) {
              serviceWorker.addEventListener('statechange', () => {
                if (serviceWorker.state == 'activated') {
                  console.log('Installed new service worker.');
                  loadMainDartJs();
                }
              });
            }
            if (!reg.active && (reg.installing || reg.waiting)) {
              // No active web worker and we have installed or are installing
              // one for the first time. Simply wait for it to activate.
              waitForActivation(reg.installing ?? reg.waiting);
            } else if (!reg.active.scriptURL.endsWith(serviceWorkerVersion)) {
              // When the app updates the serviceWorkerVersion changes, so we
              // need to ask the service worker to update.
              console.log('New service worker available.');
              reg.update();
              waitForActivation(reg.installing);
            } else {
              // Existing service worker is still good.
              console.log('Loading app from service worker.');
              loadMainDartJs();
            }
          });

        // If service worker doesn't succeed in a reasonable amount of time,
        // fallback to plaint <script> tag.
        setTimeout(() => {
          if (!scriptLoaded) {
            console.warn(
              'Failed to load app from service worker. Falling back to plain <script> tag.',
            );
            loadMainDartJs();
          }
        }, 4000);
      });
    } else {
      // Service workers not supported. Just drop the <script> tag.
      loadMainDartJs();
    }
  </script>
</body>
</html>
''';

// Generates the scaffolding of an index.html file, with a configurable `initScript`.
String _generateFlutterJsIndexHtml(String initScript) => '''
<!DOCTYPE HTML>
<!-- Copyright 2014 The Flutter Authors. All rights reserved.
Use of this source code is governed by a BSD-style license that can be
found in the LICENSE file. -->
<html>
<head>
  <meta charset="UTF-8">
  <meta content="IE=Edge" http-equiv="X-UA-Compatible">

  <title>Integration test. App load with flutter.js and onEntrypointLoaded API</title>
  <!-- iOS meta tags & icons -->
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black">
  <meta name="apple-mobile-web-app-title" content="Web Test">
  <link rel="manifest" href="manifest.json">
  <script>
    // The value below is injected by flutter build, do not touch.
    const serviceWorkerVersion = null;
  </script>
  <!-- This script adds the flutter initialization JS code -->
  <script src="flutter.js" defer></script>
</head>
<body>
  <script>
$initScript
  </script>
</body>
</html>
''';

/// index.html using flutter bootstrap script
const String indexHtmlWithFlutterBootstrapScriptTag = '''
<!DOCTYPE HTML>
<!-- Copyright 2014 The Flutter Authors. All rights reserved.
Use of this source code is governed by a BSD-style license that can be
found in the LICENSE file. -->
<html>
<head>
  <meta charset="UTF-8">
  <meta content="IE=Edge" http-equiv="X-UA-Compatible">

  <title>Web Test</title>
  <!-- iOS meta tags & icons -->
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black">
  <meta name="apple-mobile-web-app-title" content="Web Test">
  <link rel="manifest" href="manifest.json">
</head>
<body>
  <script src="flutter_bootstrap.js" async></script>
</body>
</html>
''';

/// index.html using flutter bootstrap script
const String indexHtmlWithInlinedFlutterBootstrapScript = '''
<!DOCTYPE HTML>
<!-- Copyright 2014 The Flutter Authors. All rights reserved.
Use of this source code is governed by a BSD-style license that can be
found in the LICENSE file. -->
<html>
<head>
  <meta charset="UTF-8">
  <meta content="IE=Edge" http-equiv="X-UA-Compatible">

  <title>Web Test</title>
  <!-- iOS meta tags & icons -->
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black">
  <meta name="apple-mobile-web-app-title" content="Web Test">
  <link rel="manifest" href="manifest.json">
</head>
<body>
  <script>
    {{flutter_bootstrap_js}}
  </script>
</body>
</html>
''';
