// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'felt_config.dart';

class SuiteFilterResult {
  SuiteFilterResult.accepted();
  SuiteFilterResult.rejected(String reason) : rejectReason = reason;

  String? rejectReason;

  bool get isAccepted => rejectReason == null;
}

abstract class SuiteFilter {
  SuiteFilterResult filterSuite(TestSuite suite);
}

abstract class AllowListSuiteFilter<T> implements SuiteFilter {
  AllowListSuiteFilter({required this.allowList});

  final Set<T> allowList;

  T getAttributeForSuite(TestSuite suite);

  String rejectReason(TestSuite suite) {
    return '${getAttributeForSuite(suite)} does not match filter.';
  }

  @override
  SuiteFilterResult filterSuite(TestSuite suite) {
    if (allowList.contains(getAttributeForSuite(suite))) {
      return SuiteFilterResult.accepted();
    } else {
      return SuiteFilterResult.rejected(rejectReason(suite));
    }
  }
}

class BrowserSuiteFilter extends AllowListSuiteFilter<BrowserName> {
  BrowserSuiteFilter({required super.allowList});

  @override
  BrowserName getAttributeForSuite(TestSuite suite) => suite.runConfig.browser;
}

class SuiteNameFilter extends AllowListSuiteFilter<String> {
  SuiteNameFilter({required super.allowList});

  @override
  String getAttributeForSuite(TestSuite suite) => suite.name;
}

class BundleNameFilter extends AllowListSuiteFilter<String> {
  BundleNameFilter({required super.allowList});

  @override
  String getAttributeForSuite(TestSuite suite) => suite.testBundle.name;
}

class FileFilter extends BundleNameFilter {
  FileFilter({required super.allowList});

  @override
  String rejectReason(TestSuite suite) {
    return "Doesn't contain any of the indicated files.";
  }
}

class CompilerFilter extends SuiteFilter {
  CompilerFilter({required this.allowList});

  final Set<Compiler> allowList;

  @override
  SuiteFilterResult filterSuite(TestSuite suite) =>
      suite.testBundle.compileConfigs.any(
            (CompileConfiguration config) => allowList.contains(config.compiler),
          )
          ? SuiteFilterResult.accepted()
          : SuiteFilterResult.rejected('Selected compilers not used in suite.');
}

class RendererFilter extends SuiteFilter {
  RendererFilter({required this.allowList});

  final Set<Renderer> allowList;

  @override
  SuiteFilterResult filterSuite(TestSuite suite) =>
      suite.testBundle.compileConfigs.any(
            (CompileConfiguration config) => allowList.contains(config.renderer),
          )
          ? SuiteFilterResult.accepted()
          : SuiteFilterResult.rejected('Selected renderers not used in suite.');
}

class CanvasKitVariantFilter extends AllowListSuiteFilter<CanvasKitVariant> {
  CanvasKitVariantFilter({required super.allowList});

  @override
  // TODO(jackson): Is this the right default?
  CanvasKitVariant getAttributeForSuite(TestSuite suite) =>
      suite.runConfig.variant ?? CanvasKitVariant.full;
}

Set<BrowserName> get _supportedPlatformBrowsers {
  if (io.Platform.isLinux) {
    return <BrowserName>{BrowserName.chrome, BrowserName.firefox};
  } else if (io.Platform.isMacOS) {
    return <BrowserName>{BrowserName.chrome, BrowserName.firefox, BrowserName.safari};
  } else if (io.Platform.isWindows) {
    return <BrowserName>{BrowserName.chrome, BrowserName.edge};
  } else {
    throw AssertionError('Unsupported OS: ${io.Platform.operatingSystem}');
  }
}

class PlatformBrowserFilter extends BrowserSuiteFilter {
  PlatformBrowserFilter() : super(allowList: _supportedPlatformBrowsers);

  @override
  String rejectReason(TestSuite suite) =>
      'Current platform (${io.Platform.operatingSystem}) does not support browser ${suite.runConfig.browser}';
}
