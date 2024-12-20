# `git_repo_tools`

This is a repo-internal library for `flutter/engine`, that contains shared code
for writing tools that want to interact with the `git` repository. For example,
finding all changed files in the current branch:

```dart
import 'dart:io' as io show File, Platform;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:git_repo_tools/git_repo_tools.dart';
import 'package:path/path.dart' as path;

void main() async {
  // Finds the root of the engine repository from the current script.
  final Engine engine = Engine.findWithin(path.dirname(path.fromUri(io.Platform.script)));
  final GitRepo gitRepo = GitRepo(engine.flutterDir);

  for (final io.File file in gitRepo.changedFiles) {
    print('Changed file: ${file.path}');
  }
}
```
