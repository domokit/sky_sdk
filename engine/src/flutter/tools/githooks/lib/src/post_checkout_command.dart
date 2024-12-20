// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';

import 'messages.dart';

/// The command that implements the post-checkout githook
class PostCheckoutCommand extends Command<bool> {
  @override
  final String name = 'post-checkout';

  @override
  final String description = 'Checks that run after the worktree is updated';

  @override
  Future<bool> run() async {
    printGclientSyncReminder(name);
    return true;
  }
}
