# Golden Tests Harvester

A command-line tool that uploads golden images to Skia gold from a directory.

## Usage

This program assumes you've _already run_ a suite of golden tests that produce
a directory of images with a JSON digest named `digests.json`, which is
[documented in `lib/golden_tests_harvester.dart`][lib].

Provide the directory as the only positional argument to the program:

[lib]: lib/golden_tests_harvester.dart

```sh
dart ./tools/golden_tests_harvester/bin/golden_tests_harvester.dart <path/to/digests>
```

> [!INFO]
> Skia Gold must be setup and configured to accept the images being uploaded.
>
> In practice, that means it's the process is running on CI.

_Optionally_, you can run in `--dry-run` mode to see what would be uploaded
without actually uploading anything:

```sh
dart ./tools/golden_tests_harvester/bin/golden_tests_harvester.dart --dry-run <path/to/digests>
```

This flag is automatically set when running locally (i.e. outside of CI).
