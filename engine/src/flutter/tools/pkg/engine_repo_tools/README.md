# engine_repo_tools

This is a repo-internal library for `flutter/engine`, that contains shared code
for writing tools that operate on the engine repository. For example, finding
the latest compiled engine artifacts in the `out/` directory:

```dart
import 'package:engine_repo_tools/engine_repo_tools.dart';

void main() {
  final engine = Engine.findWithin();
  final latest = engine.latestOutput();
  if (latest != null) {
    print('Latest compile_commands.json: ${latest.compileCommandsJson?.path}');
  }
}
```
