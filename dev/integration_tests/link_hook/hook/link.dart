// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:native_assets_cli/code_assets.dart';

void main(List<String> args) async {
  await link(args, (LinkConfig config, LinkOutputBuilder output) async {
    final CodeAsset asset = config.codeAssets.single;
    final String packageName = config.packageName;
    output.codeAssets.add(
      CodeAsset(
        package: packageName,
        // Change the asset id to something that is used.
        name: '${packageName}_bindings_generated.dart',
        linkMode: asset.linkMode,
        os: asset.os,
        architecture: asset.architecture,
        file: asset.file,
      ),
    );
    output.addDependency(config.packageRoot.resolve('hook/link.dart'));
  });
}
