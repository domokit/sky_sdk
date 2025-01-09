// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:native_assets_cli/code_assets_builder.dart';

import '../../../globals.dart' as globals;
import '../../../windows/visual_studio.dart';

Future<CCompilerConfig?> cCompilerConfigWindows() async {
  final VisualStudio visualStudio = VisualStudio(
    fileSystem: globals.fs,
    platform: globals.platform,
    logger: globals.logger,
    processManager: globals.processManager,
    osUtils: globals.os,
  );

  final Uri? compiler = _toOptionalFileUri(visualStudio.clPath);
  final Uri? archiver = _toOptionalFileUri(visualStudio.libPath);
  final Uri? linker = _toOptionalFileUri(visualStudio.linkPath);
  final Uri? envScript = _toOptionalFileUri(visualStudio.vcvarsPath);

  if (compiler == null || archiver == null || linker == null || envScript == null) {
    // Visual Studio might not be installed, don't exit tool.
    return null;
  }

  return CCompilerConfig(
    compiler: compiler,
    archiver: archiver,
    linker: linker,
    envScript: envScript,
    envScriptArgs: <String>[],
  );
}

Uri? _toOptionalFileUri(String? string) {
  if (string == null) {
    return null;
  }
  return Uri.file(string);
}
