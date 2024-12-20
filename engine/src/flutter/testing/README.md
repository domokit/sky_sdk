# Engine Testing

This directory contains the infrastructure for running tests on the engine,
which are most often run by Flutter's continuous integration (CI) systems.

The tests themselves are located in other directories, closer to the source for
each platform, language, and variant. For instance, macOS engine unit tests
written in objective C are located in the same directory as the source files,
but with a `Test` suffix added (e.g. "FlutterEngineTest.mm" holds the tests for
"FlutterEngine.mm", and they are located in the same directory).

## Testing the Engine locally

If you are working on the engine, you will want to be able to run tests locally.

In order to learn the details of how do that, please consult the [Flutter Wiki
page](https://github.com/flutter/flutter/wiki/Testing-the-engine) on the
subject.
