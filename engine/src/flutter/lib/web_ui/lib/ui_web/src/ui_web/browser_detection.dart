// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'package:ui/src/engine/dom.dart' show DomNavigatorExtension, DomWindowExtension, domWindow;

/// The HTML engine used by the current browser.
enum BrowserEngine {
  /// The engine that powers Chrome, Samsung Internet Browser, UC Browser,
  /// Microsoft Edge, Opera, and others.
  ///
  /// Blink is assumed in case when a more precise browser engine wasn't
  /// detected.
  blink,

  /// The engine that powers Safari.
  webkit,

  /// The engine that powers Firefox.
  firefox,
}

/// Operating system where the current browser runs.
///
/// Taken from the navigator platform.
/// <https://developer.mozilla.org/en-US/docs/Web/API/NavigatorID/platform>
enum OperatingSystem {
  /// iOS: <http://www.apple.com/ios/>
  iOs,

  /// Android: <https://www.android.com/>
  android,

  /// Linux: <https://www.linux.org/>
  linux,

  /// Windows: <https://www.microsoft.com/windows/>
  windows,

  /// MacOs: <https://www.apple.com/macos/>
  macOs,

  /// We were unable to detect the current operating system.
  unknown,
}

// List of Operating Systems we know to be working on laptops/desktops.
//
// These devices tend to behave differently on many core issues such as events,
// screen readers, input devices.
const Set<OperatingSystem> _desktopOperatingSystems = <OperatingSystem>{
  OperatingSystem.macOs,
  OperatingSystem.linux,
  OperatingSystem.windows,
};

/// The core Browser Detection functionality from the Flutter web engine.
class BrowserDetection {
  BrowserDetection._();

  /// The singleton instance of the [BrowserDetection] class.
  static final BrowserDetection instance = BrowserDetection._();

  /// Returns the User Agent of the current browser.
  String get userAgent => debugUserAgentOverride ?? _userAgent;

  /// Override value for [userAgent].
  ///
  /// Setting this to `null` uses the default [domWindow.navigator.userAgent].
  @visibleForTesting
  String? debugUserAgentOverride;

  // Lazily initialized current user agent.
  late final String _userAgent = _detectUserAgent();

  String _detectUserAgent() {
    return domWindow.navigator.userAgent;
  }

  /// Returns the [BrowserEngine] used by the current browser.
  ///
  /// This is used to implement browser-specific behavior.
  BrowserEngine get browserEngine {
    return debugBrowserEngineOverride ?? _browserEngine;
  }

  /// Override the value of [browserEngine].
  ///
  /// Setting this to `null` lets [browserEngine] detect the browser that the
  /// app is running on.
  @visibleForTesting
  BrowserEngine? debugBrowserEngineOverride;

  // Lazily initialized current browser engine.
  late final BrowserEngine _browserEngine = _detectBrowserEngine();

  BrowserEngine _detectBrowserEngine() {
    final String vendor = domWindow.navigator.vendor;
    final String agent = userAgent.toLowerCase();
    return detectBrowserEngineByVendorAgent(vendor, agent);
  }

  /// Detects browser engine for a given vendor and agent string.
  @visibleForTesting
  BrowserEngine detectBrowserEngineByVendorAgent(String vendor, String agent) {
    if (vendor == 'Google Inc.') {
      return BrowserEngine.blink;
    } else if (vendor == 'Apple Computer, Inc.') {
      return BrowserEngine.webkit;
    } else if (agent.contains('Edg/')) {
      // Chromium based Microsoft Edge has `Edg` in the user-agent.
      // https://docs.microsoft.com/en-us/microsoft-edge/web-platform/user-agent-string
      return BrowserEngine.blink;
    } else if (vendor == '' && agent.contains('firefox')) {
      // An empty string means firefox:
      // https://developer.mozilla.org/en-US/docs/Web/API/Navigator/vendor
      return BrowserEngine.firefox;
    }

    // Assume Blink otherwise, but issue a warning.
    print(
      'WARNING: failed to detect current browser engine. Assuming this is a Chromium-compatible browser.',
    );
    return BrowserEngine.blink;
  }

  /// Returns the [OperatingSystem] the current browsers works on.
  ///
  /// This is used to implement operating system specific behavior such as
  /// soft keyboards.
  OperatingSystem get operatingSystem {
    return debugOperatingSystemOverride ?? _operatingSystem;
  }

  /// Override the value of [operatingSystem].
  ///
  /// Setting this to `null` lets [operatingSystem] detect the real OS that the
  /// app is running on.
  ///
  /// This is intended to be used for testing and debugging only.
  OperatingSystem? debugOperatingSystemOverride;

  /// Lazily initialized current operating system.
  late final OperatingSystem _operatingSystem = detectOperatingSystem();

  /// Detects operating system using platform and UA used for unit testing.
  @visibleForTesting
  OperatingSystem detectOperatingSystem({String? overridePlatform, int? overrideMaxTouchPoints}) {
    final String platform = overridePlatform ?? domWindow.navigator.platform!;

    if (platform.startsWith('Mac')) {
      // iDevices requesting a "desktop site" spoof their UA so it looks like a Mac.
      // This checks if we're in a touch device, or on a real mac.
      final int maxTouchPoints =
          overrideMaxTouchPoints ?? domWindow.navigator.maxTouchPoints?.toInt() ?? 0;
      if (maxTouchPoints > 2) {
        return OperatingSystem.iOs;
      }
      return OperatingSystem.macOs;
    } else if (platform.toLowerCase().contains('iphone') ||
        platform.toLowerCase().contains('ipad') ||
        platform.toLowerCase().contains('ipod')) {
      return OperatingSystem.iOs;
    } else if (userAgent.contains('Android')) {
      // The Android OS reports itself as "Linux armv8l" in
      // [domWindow.navigator.platform]. So we have to check the user-agent to
      // determine if the OS is Android or not.
      return OperatingSystem.android;
    } else if (platform.startsWith('Linux')) {
      return OperatingSystem.linux;
    } else if (platform.startsWith('Win')) {
      return OperatingSystem.windows;
    } else {
      return OperatingSystem.unknown;
    }
  }

  /// A flag to check if the current [operatingSystem] is a laptop/desktop
  /// operating system.
  bool get isDesktop => _desktopOperatingSystems.contains(operatingSystem);

  /// A flag to check if the current browser is running on a mobile device.
  ///
  /// Flutter web considers "mobile" everything that not [isDesktop].
  bool get isMobile => !isDesktop;

  /// Whether the current [browserEngine] is [BrowserEngine.blink] (Chrom(e|ium)).
  bool get isChromium => browserEngine == BrowserEngine.blink;

  /// Whether the current [browserEngine] is [BrowserEngine.webkit] (Safari).
  bool get isSafari => browserEngine == BrowserEngine.webkit;

  /// Whether the current [browserEngine] is [BrowserEngine.firefox].
  bool get isFirefox => browserEngine == BrowserEngine.firefox;

  /// Whether the current browser is Edge.
  bool get isEdge => userAgent.contains('Edg/');

  /// Whether we are running from a wasm module compiled with dart2wasm.
  bool get isWasm => !const bool.fromEnvironment('dart.library.html');
}

/// A short-hand accessor to the [BrowserDetection.instance] singleton.
BrowserDetection browser = BrowserDetection.instance;
