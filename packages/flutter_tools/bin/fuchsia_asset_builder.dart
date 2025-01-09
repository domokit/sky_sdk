// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:flutter_tools/src/asset.dart' hide defaultManifestPath;
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/context.dart';
import 'package:flutter_tools/src/base/file_system.dart' as libfs;
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/depfile.dart';
import 'package:flutter_tools/src/bundle.dart';
import 'package:flutter_tools/src/bundle_builder.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/context_runner.dart';
import 'package:flutter_tools/src/dart/package_map.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:unified_analytics/unified_analytics.dart';

const String _kOptionPackages = 'packages';
const String _kOptionAsset = 'asset-dir';
const String _kOptionManifest = 'manifest';
const String _kOptionAssetManifestOut = 'asset-manifest-out';
const String _kOptionComponentName = 'component-name';
const String _kOptionDepfile = 'depfile';
const List<String> _kRequiredOptions = <String>[
  _kOptionPackages,
  _kOptionAsset,
  _kOptionAssetManifestOut,
  _kOptionComponentName,
];

Future<void> main(List<String> args) {
  return runInContext<void>(
    () => run(args),
    overrides: <Type, Generator>{Analytics: () => const NoOpAnalytics()},
  );
}

Future<void> writeAssetFile(libfs.File outputFile, AssetBundleEntry asset) async {
  outputFile.createSync(recursive: true);
  final List<int> data = await asset.contentsAsBytes();
  outputFile.writeAsBytesSync(data);
}

Future<void> run(List<String> args) async {
  final ArgParser parser =
      ArgParser()
        ..addOption(_kOptionPackages, help: 'The .dart_tool/package_config file')
        ..addOption(_kOptionAsset, help: 'The directory where to put temporary files')
        ..addOption(_kOptionManifest, help: 'The manifest file')
        ..addOption(_kOptionAssetManifestOut)
        ..addOption(_kOptionComponentName)
        ..addOption(_kOptionDepfile);
  final ArgResults argResults = parser.parse(args);
  if (_kRequiredOptions.any((String option) => !argResults.options.contains(option))) {
    globals.printError('Missing option! All options must be specified.');
    exit(1);
  }
  Cache.flutterRoot = globals.platform.environment['FLUTTER_ROOT'];

  final String assetDir = argResults[_kOptionAsset] as String;
  final AssetBundle? assets = await buildAssets(
    manifestPath: argResults[_kOptionManifest] as String? ?? defaultManifestPath,
    assetDirPath: assetDir,
    packageConfigPath:
        argResults[_kOptionPackages] as String? ??
        findPackageConfigFileOrDefault(globals.fs.currentDirectory).path,
    targetPlatform: TargetPlatform.fuchsia_arm64, // This is not arch specific.
  );

  if (assets == null) {
    throwToolExit('Unable to find assets.', exitCode: 1);
  }

  final List<Future<void>> calls = <Future<void>>[];
  assets.entries.forEach((String fileName, AssetBundleEntry entry) {
    final libfs.File outputFile = globals.fs.file(globals.fs.path.join(assetDir, fileName));
    calls.add(writeAssetFile(outputFile, entry));
  });
  await Future.wait<void>(calls);

  final String outputMan = argResults[_kOptionAssetManifestOut] as String;
  await writeFuchsiaManifest(
    assets,
    argResults[_kOptionAsset] as String,
    outputMan,
    argResults[_kOptionComponentName] as String,
  );

  final String? depfilePath = argResults[_kOptionDepfile] as String?;
  if (depfilePath != null) {
    await writeDepfile(assets, outputMan, depfilePath);
  }
}

Future<void> writeDepfile(AssetBundle assets, String outputManifest, String depfilePath) async {
  final Depfile depfileContent = Depfile(assets.inputFiles, <libfs.File>[
    globals.fs.file(outputManifest),
  ]);
  final DepfileService depfileService = DepfileService(
    fileSystem: globals.fs,
    logger: globals.logger,
  );

  final libfs.File depfile = globals.fs.file(depfilePath);
  await depfile.create(recursive: true);
  depfileService.writeToFile(depfileContent, depfile);
}

Future<void> writeFuchsiaManifest(
  AssetBundle assets,
  String outputBase,
  String fileDest,
  String componentName,
) async {
  final libfs.File destFile = globals.fs.file(fileDest);
  await destFile.create(recursive: true);
  final libfs.IOSink outFile = destFile.openWrite();

  for (final String path in assets.entries.keys) {
    outFile.write('data/$componentName/$path=$outputBase/$path\n');
  }
  await outFile.flush();
  await outFile.close();
}
