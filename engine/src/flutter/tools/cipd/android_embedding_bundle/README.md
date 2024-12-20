# Updating the Embedding Dependencies

The instructions in this README explain how to create a CIPD package that
contains the build-time dependencies of the Android embedding of the Engine,
and the dependencies of the in-tree testing framework. The Android embedder is
shipped to Flutter end-users, but these build-time dependencies are not.
Therefore, the license script can skip over the destination of the CIPD package
in an Engine checkout at `src/third_party/android_embedding_dependencies`.
Even so, the CIPD package should contain a LICENSE file, and the instructions
below explain how to fetch the license information for the dependencies.

## Requirements

1. If you have a flutter/engine checkout, then you should already have
[Depot tools](http://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up) on your path.
1. You should have a copy of `gradle` in a flutter/engine checkout under
   `src/third_party/gradle/bin/gradle`.

## Steps

1. Update `src/flutter/tools/androidx/files.json`. (This file includes the Maven
   dependencies used to build Flutter apps).
1. `cd` into this directory: `src/flutter/tools/cipd/android_embedding_bundle`.
1. Run `gradle downloadLicenses`
1. Run `gradle updateDependencies`
1. Examine the file `./build/reports/license/license-dependency.xml`. If it
   contains licenses other than "The Apache License, Version 2.0" or something
   very similar, STOP. Ask Hixie for adivce on how to proceed.
1. Copy or move the `lib/` directory to `src/third_party/android_embedding_dependencies/`,
   overwriting its contents, and ensure the Android build still works.
1. Run `cipd create --pkg-def cipd.yaml -tag last_updated:"$version_tag"` where
   `$version_tag` is the output of `date +%Y-%m-%dT%T%z`.
1. Update the `DEPS` file entry for `android_embedding_dependencies` with the
   new tag: `last_updated:"$version_tag"`.
1. Update the GN list `embedding_dependencies_jars` in
   `src/flutter/shell/platform/android/BUILD.gn`.

## Updating Gradle Lockfiles in the Framework After Adding Dependencies
If you land a pr that changes the versions of the embedding dependencies,
or adds a new dependency and makes use of it, you will also need to
perform a manual roll of that change to the framework that re-generates
the Gradle lockfiles using the script at
`<framework_repo>/dev/tools/bin/generate_gradle_lockfiles.dart`
(run with the `--no-gradle-generation` and `--no-exclusion` flags).