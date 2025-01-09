// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../cmake_project.dart';
import 'utils.dart';

const String _before = r'''
  flutter_controller_->engine()->SetNextFrameCallback([&]() {
    this->Show();
  });

  return true;
''';
const String _after = r'''
  flutter_controller_->engine()->SetNextFrameCallback([&]() {
    this->Show();
  });

  // Flutter can complete the first frame before the "show window" callback is
  // registered. The following call ensures a frame is pending to ensure the
  // window is shown. It is a no-op if the first frame hasn't completed yet.
  flutter_controller_->ForceRedraw();

  return true;
''';

/// Migrates Windows apps to ensure the window is shown.
///
/// This prevents a race condition between Flutter rendering the first frame
/// and the app registering the callback to show the window on the first frame.
/// See https://github.com/flutter/flutter/issues/119415.
class ShowWindowMigration extends ProjectMigrator {
  ShowWindowMigration(WindowsProject project, super.logger)
    : _file = project.runnerFlutterWindowFile;

  final File _file;

  @override
  Future<void> migrate() async {
    // Skip this migration if the affected file does not exist. This indicates
    // the app has done non-trivial changes to its runner and this migration
    // might not work as expected if applied.
    if (!_file.existsSync()) {
      logger.printTrace('''
windows/runner/flutter_window.cpp file not found, skipping show window migration.

This indicates non-trivial changes have been made to the Windows runner in the
"windows" folder. If needed, you can reset the Windows runner by deleting the
"windows" folder and then using the "flutter create --platforms=windows ." command.
''');
      return;
    }

    // Migrate the windows/runner/flutter_window.cpp file.
    final String originalContents = _file.readAsStringSync();
    final String newContents = replaceFirst(originalContents, _before, _after);
    if (originalContents != newContents) {
      logger.printStatus(
        'windows/runner/flutter_window.cpp does not ensure the show window '
        'callback is called, updating.',
      );
      _file.writeAsStringSync(newContents);
    }
  }
}
