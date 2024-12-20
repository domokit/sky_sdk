# Code signing

This covers the process of how to add / update code signing metadata of flutter
engine binaries.

## Overview

Flutter engine binaries are built with GN and ninja, referencing pre-defined
configurations such as ci/builders
[JSON files](https://github.com/flutter/engine/blob/main/ci/builders/mac_host_engine.json).
During flutter releases, engineers need to code sign mac engine binaries to
assure users that they come from a known source, have not been tampered with,
and should not be quarantined by Gatekeepers.

Each of the Flutter engine binaries are either code signed with entitlements, or
code signed without entitlements. (An entitlement, along with information from
the developer account, grant particular permissions to binaries, such as
capability to access the user's home automation network.) For example, impellerc
is code signed with flutter entitlements, whereas .dylib files are usually code
signed without entitlements.

## Add / Update code signing metadata

### Glossary

1.  BUILD.gn files: files that include build rules of GN targets. An example is
    the
    [BUILD.gn file of flutter engine](https://github.com/flutter/engine/blob/main/BUILD.gn).
2.  leaf node of an engine binary: the minimal gn target that could produce such
    an engine binary. That is, this target does not have any dependencies on
    other gn targets that could build this engine binary.
3.  dependencies: Every gn target could have dependencies on other gn targets.
    The dependency of a gn target is defined in the `deps` field of the target's
    build rule.

### ways to generate engine binary

Generally, there are two ways to generate an engine binary:

1.  Through build rules defined in BUILD.gn files.

2.  Through global generator scripts. (these scripts are normally .py files)

To distinguish between the two, an engine binary is built through global
generator if it is listed in the `archives` -> `destination` field of the
builder JSON
([mac_ios_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_ios_engine.json)
or
[mac_host_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_host_engine.json)).
For example, `darwin-x64/FlutterEmbedder.framework.zip`. Whereas binaries built
with BUILD.gn files are listed among the `builds` field of the JSON file. For
example, `darwin-x64/artifacts.zip`. We will provide examples for both
scenarios.

### To add / update code signing metadata in BUILD.gn files:

1.  Find the leaf node where the target engine binary is built. To do so,
    Recursively trace the `deps` field of the engine artifact. The paths in
    `deps` field of the GN target correspond to the paths of other GN targets
    that are dependencies of the current GN target.

2.  Add / Update the `metadata` field of the leaf node. For a new engine binary:

    2.1 if it should be code signed with entitlements, add [the name of the
    engine binary] to the `entitlement_file_path` field in `metadata` .

    2.2 if the binary shouldn't be code signed with entitlements, add [the name
    of the engine binary] to the `without_entitlement_file_path` field in
    `metadata` .

3.  If a `entitlement_file_path` or a `without_entitlement_file_path` field does
    not exist:

    **note**: this step is only needed if the target includes solely binaries
    that have never been code signed before. This step also requires some
    background on flutter engine and gn build rules.

    Add a `metadata` field in the gn target of the leaf node, and put the name
    of the binary in this field. e.g.

    ```
    metadata = {
        entitlement_file_path = [ "libtessellator.dylib" ]
    }
    ```

    In the same file that produces the engine artifact(zip file), add a build
    rule to collect the data keys. e.g.

    ```
    generated_file("artifacts_entitlement_config") {
        outputs = [ "$target_gen_dir/entitlements.txt" ]

        data_keys = [ "entitlement_file_path" ]

        deps = [ "//flutter/lib/snapshot:generate_snapshot_bin" ]
        if (flutter_runtime_mode == "debug") {
            deps += [
            "//flutter/impeller/compiler:impellerc",
            "//flutter/impeller/tessellator:tessellator_shared",
            "//flutter/shell/testing:testing",
            "//flutter/tools/path_ops:path_ops",
            ]
        }
    }
    ```

    Finally, embed the file with collected data keys in the zip artifact. e.g.

    ```
    if (host_os == "mac") {
        deps += [ ":artifacts_entitlement_config" ]
        files += [
            {
            source = "$target_gen_dir/entitlements.txt"
            destination = "entitlements.txt"
            },
        ]
    }
    ```

#### Example

Suppose impellerc is a binary that exist in a zip bundle called artifacts.zip.
Then impellerc is the name of the binary, and artifacts.zip is the flutter
engine artifact.

1.  Following step 1, the `deps` field of the GN target of artifacts.zip
    includes the path of impeller dependency:
    `//flutter/impeller/compiler:impellerc`. Following this path, we locate the
    GN file at `flutter/impeller/compiler/BUILD.gn`, and find the leaf node that
    builds impellerc: `impeller_component("impellerc")`.

2.  Following step 2, since `impellerc` should be code signed with entitlements,
    we go to the `metadata` field of the impellerc target, and add the name
    `impellerc` to the `entitlement_file_path` array inside the `metadata`
    field.

You can reference the
[BUILD.gn file of impellerc](https://github.com/flutter/engine/blob/main/impeller/compiler/BUILD.gn).

### To add / update code signing metadata in global generator files:

1.  Find the generator script path listed under `generators` -> `tasks` ->
    `script` of the ci/builder JSON files
    ([mac_ios_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_ios_engine.json)
    or
    [mac_host_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_host_engine.json)).

    The generator script related to iOS is located at
    `sky/tools/create_ios_framework.py`, and generator script related to
    macOS is located at `sky/tools/create_macos_framework.py`.

2.  Add / Update the variables ending with `with_entitlements` /
    `without_entitlements` suffix from the generator script you found in step
    one.

    As an example, you can find variables `ios_file_without_entitlements` and
    `ios_file_with_entitlements` in sky/tools/create_ios_framework.py; and
    find variables `filepath_without_entitlements` and
    `filepath_with_entitlements` in sky/tools/create_macos_framework.py

    2.1 if the binary should be code signed with entitlements, add [the name of
    the binary] to the variable name with the `with_entitlements` suffix.
    (`ios_file_with_entitlements` or `filepath_with_entitlements` depending on
    which script)

    2.2 if the binary shouldn't be code signed with entitlements, add [the name
    of the binary] to the variable name with the `without_entitlements` suffix.

#### Example

Suppose `Flutter.xcframework/ios-arm64/Flutter.framework/Flutter` is a binary
that exist in a zip bundle called `ios/artifacts.zip`.

1.  Following step 1, in
    [mac_ios_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_ios_engine.json),
    it builds the artifact with the
    `flutter/sky/tools/create_ios_framework.py` script.

2.  Following step 2, since
    `Flutter.xcframework/ios-arm64/Flutter.framework/Flutter` shouldn't be code
    signed with entitlements, we add the binary name
    `Flutter.xcframework/ios-arm64/Flutter.framework/Flutter` to the
    `ios_file_without_entitlements` variable.

You can reference the generator script
[create_ios_framework.py](https://github.com/flutter/engine/blob/main/sky/tools/create_ios_framework.py).

## Code signing artifacts other than flutter engine binaries

The code signing functionality is implemented as [a recipe module in flutter recipes](https://cs.opensource.google/flutter/recipes/+/master:recipe_modules/signing/api.py). Therefore it can also be used to
code sign arbitrary flutter artifacts built through recipe, for example, flutter iOS usb dependencies.

To code sign, after the artifacts are built, pass the file paths into
the code signing recipe module and invoke the function. An example is [how engine V2 invokes the code signing recipe module](https://cs.opensource.google/flutter/recipes/+/master:recipes/engine_v2/engine_v2.py;l=197-212).
