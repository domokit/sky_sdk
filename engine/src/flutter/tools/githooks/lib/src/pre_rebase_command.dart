// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';

/// The command that implements the pre-rebase githook
class PreRebaseCommand extends Command<bool> {
  @override
  final String name = 'pre-rebase';

  @override
  final String description = 'Checks to run before a "git rebase"';

  @override
  Future<bool> run() async {
    // Returning false here will block the rebase.
    return true;
  }
}
