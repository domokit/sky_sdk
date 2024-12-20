// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../runner/flutter_command.dart';

class MakeHostAppEditableCommand extends FlutterCommand {
  MakeHostAppEditableCommand() {
    requiresPubspecYaml();

    argParser.addFlag(
      'ios',
      help: "Whether to make this project's iOS app editable.",
      negatable: false,
    );
    argParser.addFlag(
      'android',
      help: "Whether ot make this project's Android app editable.",
      negatable: false,
    );
  }

  @override
  final String name = 'make-host-app-editable';

  @override
  bool get deprecated => true;

  @override
  final String description =
      'Moves host apps from generated directories to non-generated directories so that they can be edited by developers.';

  @override
  Future<FlutterCommandResult> runCommand() async {
    // Deprecated. No-op.
    return FlutterCommandResult.success();
  }
}
