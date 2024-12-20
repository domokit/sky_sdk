// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:native_assets_cli/code_assets_builder.dart';

import '../../../base/common.dart';
import '../../../base/file_system.dart';
import '../../../base/io.dart';
import '../../../globals.dart' as globals;

/// Flutter expects `clang++` to be on the path on Linux hosts.
///
/// Search for the accompanying `clang`, `ar`, and `ld`.
Future<CCompilerConfig> cCompilerConfigLinux() async {
  const String kClangPlusPlusBinary = 'clang++';
  const String kClangBinary = 'clang';
  const String kArBinary = 'llvm-ar';
  const String kLdBinary = 'ld.lld';

  final ProcessResult whichResult = await globals.processManager.run(<String>[
    'which',
    kClangPlusPlusBinary,
  ]);
  if (whichResult.exitCode != 0) {
    throwToolExit('Failed to find $kClangPlusPlusBinary on PATH.');
  }
  File clangPpFile = globals.fs.file((whichResult.stdout as String).trim());
  clangPpFile = globals.fs.file(await clangPpFile.resolveSymbolicLinks());

  final Directory clangDir = clangPpFile.parent;
  final Map<String, Uri> binaryPaths = <String, Uri>{};
  for (final String binary in <String>[kClangBinary, kArBinary, kLdBinary]) {
    final File binaryFile = clangDir.childFile(binary);
    if (!await binaryFile.exists()) {
      throwToolExit("Failed to find $binary relative to $clangPpFile: $binaryFile doesn't exist.");
    }
    binaryPaths[binary] = binaryFile.uri;
  }
  return CCompilerConfig(
    archiver: binaryPaths[kArBinary],
    compiler: binaryPaths[kClangBinary],
    linker: binaryPaths[kLdBinary],
  );
}
