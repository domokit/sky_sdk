// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('build succeeds with api 33 features', () async {
    ProcessResult result = await processManager.run(<String>[
      flutterBin,
      'create',
      tempDir.path,
      '--project-name=testapp',
    ], workingDirectory: tempDir.path);
    expect(result, const ProcessResultMatcher());

    final File api33File = tempDir
        .childDirectory('android')
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('java')
        .childFile('Android33Api.java');

    api33File.createSync(recursive: true);
    // AccessibilityManager.isAudioDescriptionRequested() is an API 33 feature
    api33File.writeAsStringSync('''
import android.app.Activity;
import android.view.accessibility.AccessibilityManager;
import androidx.annotation.Keep;
import io.flutter.Log;

@Keep
public final class Android33Api extends Activity {
  private AccessibilityManager accessibilityManager;

  public Android33Api() {
    accessibilityManager = getSystemService(AccessibilityManager.class);
  }

  public void doSomething() {
    if (accessibilityManager.isAudioDescriptionRequested()) {
      Log.e("flutter", "User has requested to enable audio descriptions");
    }
  }
}

''');

    result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
    ], workingDirectory: tempDir.path);
    expect(result, const ProcessResultMatcher(stdoutPattern: 'app-release.apk'));
  });
}
