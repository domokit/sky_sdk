// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'dom.dart';

/// A flag to check if the current browser is running on a laptop/desktop device.
bool get isDesktop => ui_web.browser.isDesktop;

/// A flag to check if the current browser is running on a mobile device.
///
/// Flutter web considers "mobile" everything that's not [isDesktop].
bool get isMobile => ui_web.browser.isMobile;

/// Whether the current browser is [ui_web.BrowserEngine.blink] (Chrom(e|ium)).
bool get isChromium => ui_web.browser.isChromium;

/// Whether the current browser is [ui_web.BrowserEngine.webkit] (Safari).
bool get isSafari => ui_web.browser.isSafari;

/// Whether the current browser is [ui_web.BrowserEngine.firefox].
bool get isFirefox => ui_web.browser.isFirefox;

/// Whether the current browser is Edge.
bool get isEdge => ui_web.browser.isEdge;

/// Whether we are running from a wasm module compiled with dart2wasm.
///
/// Note: Currently the ffi library is available from dart2wasm but not dart2js
/// or dartdevc.
bool get isWasm => ui_web.browser.isWasm;

// Whether the detected `operatingSystem` is `OperatingSystem.iOs`.
bool get _isIOS => ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs;

/// Whether the browser is running on macOS or iOS.
///
/// - See [operatingSystem].
/// - See [OperatingSystem].
bool get isMacOrIOS => _isIOS || ui_web.browser.operatingSystem == ui_web.OperatingSystem.macOs;

/// Detect iOS 15.
bool get isIOS15 => debugIsIOS15 ?? _isIOS && ui_web.browser.userAgent.contains('OS 15_');

/// Use in tests to simulate the detection of iOS 15.
bool? debugIsIOS15;

/// Detect if running on Chrome version 110 or older.
///
/// These versions of Chrome have a bug which causes rendering to be flipped
/// upside down when using `createImageBitmap`: see
/// https://chromium.googlesource.com/chromium/src/+/a7f9b00e422a1755918f8ca5500380f98b6fddf2
// TODO(harryterkelsen): Remove this check once we stop supporting Chrome 110
// and earlier, https://github.com/flutter/flutter/issues/139186.
bool get isChrome110OrOlder {
  if (debugIsChrome110OrOlder != null) {
    return debugIsChrome110OrOlder!;
  }
  if (_cachedIsChrome110OrOlder != null) {
    return _cachedIsChrome110OrOlder!;
  }
  final RegExp chromeRegexp = RegExp(r'Chrom(e|ium)\/([0-9]+)\.');
  final RegExpMatch? match = chromeRegexp.firstMatch(ui_web.browser.userAgent);
  if (match != null) {
    final int chromeVersion = int.parse(match.group(2)!);
    return _cachedIsChrome110OrOlder = chromeVersion <= 110;
  }
  return _cachedIsChrome110OrOlder = false;
}

// Cache the result of checking if the app is running on Chrome 110 on Windows
// since we check this on every frame.
bool? _cachedIsChrome110OrOlder;

/// Used in tests to simulate the detection of Chrome 110 or older on Windows.
bool? debugIsChrome110OrOlder;

/// Returns true if the browser is iOS Safari, false otherwise.
bool get isIosSafari => debugEmulateIosSafari || _isActualIosSafari;

bool get _isActualIosSafari =>
    ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
    ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs;

/// If set to true pretends that the current browser is iOS Safari.
///
/// Useful for tests. Do not use in production code.
@visibleForTesting
bool debugEmulateIosSafari = false;

/// html webgl version qualifier constants.
abstract class WebGLVersion {
  /// WebGL 1.0 is based on OpenGL ES 2.0 / GLSL 1.00
  static const int webgl1 = 1;

  /// WebGL 2.0 is based on OpenGL ES 3.0 / GLSL 3.00
  static const int webgl2 = 2;
}

/// The highest WebGL version supported by the current browser, or -1 if WebGL
/// is not supported.
int get webGLVersion => _cachedWebGLVersion ?? (_cachedWebGLVersion = _detectWebGLVersion());

int? _cachedWebGLVersion;

/// Detects the highest WebGL version supported by the current browser, or
/// -1 if WebGL is not supported.
///
/// Chrome reports that `WebGL2RenderingContext` is available even when WebGL 2 is
/// disabled due hardware-specific issues. This happens, for example, on Chrome on
/// Moto E5. Therefore checking for the presence of `WebGL2RenderingContext` or
/// using the current [browserEngine] is insufficient.
///
/// Our CanvasKit backend is affected due to: https://github.com/emscripten-core/emscripten/issues/11819
int _detectWebGLVersion() {
  final DomCanvasElement canvas = createDomCanvasElement(width: 1, height: 1);
  if (canvas.getContext('webgl2') != null) {
    if (_workAroundBug91333) {
      return WebGLVersion.webgl1;
    }
    return WebGLVersion.webgl2;
  }
  if (canvas.getContext('webgl') != null) {
    return WebGLVersion.webgl1;
  }
  return -1;
}

// iOS 15 launched WebGL 2.0, but there's something broken about it, which
// leads to apps failing to load. For now, we're forcing WebGL 1 on iOS.
//
// TODO(yjbanov): https://github.com/flutter/flutter/issues/91333
bool get _workAroundBug91333 => _isIOS;

/// Whether the current browser supports the Chromium variant of CanvasKit.
bool get browserSupportsCanvaskitChromium =>
    domIntl.v8BreakIterator != null && domIntl.Segmenter != null;
