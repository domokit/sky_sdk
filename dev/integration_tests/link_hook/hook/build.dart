// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:logging/logging.dart';
import 'package:native_assets_cli/code_assets.dart';
import 'package:native_assets_cli/code_assets_builder.dart';
import 'package:native_toolchain_c/native_toolchain_c.dart';

void main(List<String> args) async {
  await build(args, (BuildConfig config, BuildOutputBuilder output) async {
    final String assetName;
    if (config.linkingEnabled) {
      // The link hook will be run. So emit an asset with a name that is
      // not used, so that the link hook can rename it.
      // This will ensure the test fails if the link-hooks are not run
      // while being reported that linking is enabled.
      assetName = 'some_asset_name_that_is_not_used';
    } else {
      // The link hook will not be run, so immediately emit an asset for
      // bundling.
      assetName = '${config.packageName}_bindings_generated.dart';
    }
    final String packageName = config.packageName;
    final CBuilder cbuilder = CBuilder.library(
      name: packageName,
      assetName: assetName,
      sources: <String>['src/$packageName.c'],
      dartBuildFiles: <String>['hook/build.dart'],
    );
    final BuildOutputBuilder outputCatcher = BuildOutputBuilder();
    await cbuilder.run(
      config: config,
      output: outputCatcher,
      logger:
          Logger('')
            ..level = Level.ALL
            ..onRecord.listen((LogRecord record) => print(record.message)),
    );
    final BuildOutput catchedOutput = BuildOutput(outputCatcher.json);
    output.addDependencies(catchedOutput.dependencies);
    // Send the asset to hook/link.dart or immediately for bundling.
    output.codeAssets.add(
      catchedOutput.codeAssets.single,
      linkInPackage: config.linkingEnabled ? 'link_hook' : null,
    );
  });
}
