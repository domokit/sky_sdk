# clang_tidy

A wrapper library and program that runs `clang_tidy` on the Flutter engine repo.

```shell
# Assuming you are in the `flutter` root of the engine repo.
dart ./tools/clang_tidy/bin/main.dart
```

By default, the linter runs over _modified_[^1] files in the _latest_[^2] build
of the engine.

A subset of checks can also be fixed automatically by passing `--fix`:

```shell
dart ./tools/clang_tidy/bin/main.dart --fix
```

To configure what lints are enabled, see [`.clang-tidy`](../../.clang-tidy).

> **💡 TIP**: If you're looking for the git pre-commit hook configuration, see
> [`githooks`](../githooks).

## Advanced Usage

Some common use cases are described below, or use `--help` to see all options.

### Run with checks added or removed

To run adding a check _not_ specified in `.clang-tidy`:

```shell
dart ./tools/clang_tidy/bin/main.dart --checks="<check-name-to-run>"
```

It's possible also to use wildcards to add multiple checks:

```shell
dart ./tools/clang_tidy/bin/main.dart --checks="readability-*"
```

To remove a specific check:

```shell
dart ./tools/clang_tidy/bin/main.dart --checks="-<check-name-to-remove>"
```

To remove multiple checks:

```shell
dart ./tools/clang_tidy/bin/main.dart --checks="-readability-*"
```

To remove _all_ checks (usually to add a specific check):

```shell
dart ./tools/clang_tidy/bin/main.dart --checks="-*,<only-check-to-run>"
```

### Specify a specific build

There are some rules that are only applicable to certain builds, or to check
a difference in behavior between two builds.

Use `--target-variant` to specify a build:

```shell
dart ./tools/clang_tidy/bin/main.dart --target-variant <engine-variant>
```

For example, to check the `android_debug_unopt` build:

```shell
dart ./tools/clang_tidy/bin/main.dart --target-variant android_debug_unopt
```

In rarer cases, for example comparing two different checkouts of the engine,
use `--src-dir=<path/to/engine/src>`.

### Lint entire repository

When adding a new lint rule, or when checking lint rules that impact files that
have not changed.

Use `--lint-all` to lint all files in the repo:

```shell
dart ./tools/clang_tidy/bin/main.dart --lint-all
```

Or, provide a regular expression to lint files that match:

```shell
dart ./tools/clang_tidy/bin/main.dart --lint-regex=".*test.*\.cc"
```

> **⚠️ WARNING**: This may take a long time to run if a pattern is not provided
> or if the pattern matches a large number of files, i.e. on the order of
> thousands of files could take 30 minutes or more to run and lock your
> machine.

[^1]: Modified files are determined by a `git diff` command compared to `HEAD`.
[^2]: Latest build is the last updated directory in `src/out/`.
