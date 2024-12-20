# `process_fakes`

Fake implementations of `Process` and `ProcessManager` for testing.

This is not a great package, and is the bare minimum needed for fairly basic
tooling that uses `ProcessManager`. If we ever need a more complete solution
we should look at upstreaming [`flutter_tools/.../fake_proecss_manager.dart`](https://github.com/flutter/flutter/blob/a9183f696c8e12617d05a26b0b5e80035e515f2a/packages/flutter_tools/test/src/fake_process_manager.dart#L223)
