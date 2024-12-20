// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';

import 'messages.dart';

/// The command that implements the post-merge githook
class PostMergeCommand extends Command<bool> {
  @override
  final String name = 'post-merge';

  @override
  final String description = 'Checks to run after a "git merge"';

  @override
  Future<bool> run() async {
    printGclientSyncReminder(name);
    return true;
  }
}
