_If you've already built the engine and have the configuration set up but merely need a refresher on
actually compiling the code, see [Compiling the engine](Compiling-the-engine.md)._

_If you are checking these instructions to refresh your memory and your fork of the engine is stale,
make sure to merge up to HEAD before doing a `gclient sync`._

# Getting dependencies

Make sure you have the following dependencies available:

 * A Linux, macOS, or Windows host
     * Linux supports cross-compiling artifacts for Android and Fuchsia, but not iOS.
     * macOS supports cross-compiling artifacts for Android and iOS.
     * Windows doesn't support cross-compiling artifacts for any of Android, Fuchsia, or iOS.
 * `git` (used for source version control).
 * An ssh client (used to authenticate with GitHub).
 * `python3` (used by many of our tools, including `gclient`).
 * **Chromium's
   [depot_tools](https://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up)** (Which includes gclient)
    * Add the `depot_tools` directory to the *front* of your `PATH`.
 * On macOS and Linux: `curl` and `unzip` (used by `gclient sync`).
 * On Linux: The `pkg-config` package.
 * On Windows:
   - Visual Studio 2017 or later (required for non-Googlers only).
   - [Windows 10 SDK](https://developer.microsoft.com/en-us/windows/downloads/windows-sdk/) (required for non-Googlers only). Be sure to install the "Debugging Tools for Windows" feature.
 * On macOS:
   - Install the latest Xcode.
   - On Apple Silicon arm64 Macs, install the Rosetta translation environment by running `softwareupdate --install-rosetta`.

You do not need to install [Dart](https://www.dartlang.org/downloads/linux.html).
A Dart toolchain is automatically downloaded as part of the "Getting the source"
step. Similarly for the Android SDK, it is downloaded by the `gclient sync` step below.

## Getting the source

Run the following steps to set up your environment:

> [!IMPORTANT]
> Non-Googler Windows users should set the following environment variables to point
>   `depot_tools` to their Visual Studio installation directory:
>   * `DEPOT_TOOLS_WIN_TOOLCHAIN=0`
>   * `GYP_MSVS_OVERRIDE_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community`
>     * Use the path of your installation.

Clone the Flutter source code. As of late 2024, the engine source is part of the main [flutter/flutter repo](https://github.com/flutter/flutter). The convention is to fork this repo and point `origin` to your fork and `upstream` to `git@github.com:flutter/flutter.git`. See [Setting up the Framework development environment](https://github.com/flutter/flutter/blob/master/docs/contributing/Setting-up-the-Framework-development-environment.md#set-up-your-environment) for more.

> [!IMPORTANT]
> On Windows, the following must be run as an Administrator due to [a known issue](https://github.com/flutter/flutter/issues/94580).

[Setup a `.gclient` file](../../../../../engine/README.md) in the repository
root (the `flutter/flutter` repository root), and run `gclient sync`.

The "Engine Tool" called `et` is useful when working with the engine. It is located in the [`flutter/engine/src/flutter/bin`](https://github.com/flutter/flutter/tree/0c3359df8c8342c8907316488b1404a216f215b6/engine/src/flutter/bin) directory. Add this to your `$PATH` in your `.rc` file: e.g. on UNIX, using `export PATH=/path/to/flutter/engine/src/flutter/bin:$PATH`.

### Additional Steps for Web Engine

Amend the generated `.gclient` file in the root of the source directory to add the following:
```
solutions = [
  {
    # Same as above...
    "custom_vars": {
      "download_emsdk": True,
    },
  },
]
```

Now, run:

```sh
gclient sync
```

## Next steps:

 * [Compiling the engine](Compiling-the-engine.md) explains how to actually get builds, now that you have the code.
 * [The flutter tool](https://github.com/flutter/flutter/blob/master/docs/tool/README.md) has a section explaining how to use custom engine builds.
 * [Signing commits](https://github.com/flutter/flutter/blob/master/docs/contributing/Signing-commits.md), to configure your environment to securely sign your commits.

## Editor autocomplete support

### Xcode [Objective-C++]

On Mac, you can simply use Xcode (e.g., `open out/host_debug_unopt/products.xcodeproj`).

### VSCode with C/C++ Intellisense [C/C++]

VSCode can provide some IDE features using the [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools). It will provide basic support on install without needing any additional configuration. There will probably be some issues, like header not found errors and incorrect jump to definitions.

Intellisense can also use our `compile_commands.json` for more robust functionality. Either symlink `src/out/compile_commands.json` to the project root at `src` or provide an absolute path to it in the `c_cpp_properties.json` config file. See ["compile commands" in the c_cpp_properties.json reference](https://code.visualstudio.com/docs/cpp/c-cpp-properties-schema-reference). This will likely resolve the basic issues mentioned above.

The easiest way to do this is create a [multi-root workspace](https://code.visualstudio.com/docs/editor/workspaces/workspaces#_multiroot-workspaces) that includes the Flutter SDK. For example, something like this:

```json
# flutter.code-workspace
{
	"folders": [
		{
			"path": "path/to/the/flutter/sdk"
		}
	],
	"settings": {}
}
```

Then, edit the `"settings"` key:

```json
"settings": {
    "html.format.enable": false,
    "githubPullRequests.ignoredPullRequestBranches": [
        "master"
    ],
    "clangd.path": "engine/src/flutter/buildtools/mac-arm64/clang/bin/clangd",
    "clangd.arguments": [
        "--compile-commands-dir=engine/src/out/host_debug_unopt_arm64"
    ],
    "clang-format.executable": "engine/src/flutter/buildtools/mac-arm64/clang/bin/clang-format"
}
```

... which is built with:

```shell
# M1 Mac (host_debug_unopt_arm64)
et build -c host_debug_unopt_arm64
```

Some files (such as the Android embedder) will require an Android `clangd` configuration.

For adding IDE support to the Java code in the engine with VSCode, see ["Using VSCode as an IDE for the Android Embedding"](#using-vscode-as-an-ide-for-the-android-embedding-java).

### Zed Editor

[Zed](https://zed.dev/) can be used to edit C++ code in the Engine. To enable analysis and auto-completion, symlink `src/out/compile_commands.json` to the project root at `src`.

### cquery/ccls (multiple editors) [C/C++/Objective-C++]

Alternatively, [cquery](https://github.com/cquery-project/cquery) and a derivative [ccls](https://github.com/MaskRay/ccls) are highly scalable C/C++/Objective-C language server that supports IDE features like go-to-definition, call hierarchy, autocomplete, find reference etc that works reasonably well with our engine repo.

They(https://github.com/cquery-project/cquery/wiki/Editor-configuration) [supports](https://github.com/MaskRay/ccls/wiki/Editor-Configuration) editors like VSCode, emacs, vim etc.

To set up:
1. Install cquery
    1. `brew install cquery` or `brew install ccls` on osx; or
    1. [Build from source](https://github.com/cquery-project/cquery/wiki/Getting-started)
1. Generate compile_commands.json which our GN tool already does such as via `src/flutter/tools/gn --ios --unoptimized`
1. Install an editor extension such as [VSCode-cquery](https://marketplace.visualstudio.com/items?itemName=cquery-project.cquery) or [vscode-ccls](https://marketplace.visualstudio.com/items?itemName=ccls-project.ccls)
    1. VSCode-query and vscode-ccls requires the compile_commands.json to be at the project root. Copy or symlink `src/out/compile_commands.json` to `src/` or `src/flutter` depending on which folder you want to open.
    1. Follow [Setting up the extension](https://github.com/cquery-project/cquery/wiki/Visual-Studio-Code#setting-up-the-extension) to configure VSCode-query.

![](https://media.giphy.com/media/xjIrToRDVvMPvjkBcl/giphy.gif)

### Using VSCode as an IDE for the Android Embedding [Java]

1. Install the extensions vscjava.vscode-java-pack (Extension Pack for Java) and vscjava.vscode-java-dependency (Project Manager for Java).

1. Right click on the `shell/platform/android` folder in the engine source and click on `Add Folder to Java Source Path`. This creates an anonymous workspace and turns those files from ["syntax mode"](https://code.visualstudio.com/docs/java/java-project#_syntax-mode) to "compile mode". At this point, you should see a lot of errors since none of the external imports are found.

1. Find the "Java Dependencies" pane in your Explorer view. Use the "Explorer: Focus on Java Dependencies View" command if hidden.

1. Refresh the view and find the "flutter_*" project. There should be a "_/shell/platform/android" source folder there.

1. In the "Referenced Libraries" sibling node, click the + button, navigate to `engine/src/third_party/android_embedding_dependencies` and add the entire folder. This is the equivalent of adding
    ```
    "java.project.referencedLibraries": [
      "{path to engine}/src/third_party/android_embedding_dependencies/lib/**/*.jar"
    ]
    ```
    to your VSCode's settings.json for your user or for your workspace.

1. If you previously had a `shell/platform/android/.classpath`, delete it.

### Using Android Studio as an IDE for the Android Embedding [Java]

Alternatively, Android Studio can be used as an IDE for the Android Embedding Java code. See docs
at https://github.com/flutter/flutter/blob/master/engine/src/flutter/shell/platform/android/README.md#editing-java-code for
instructions.

## VSCode Additional Useful Configuration

1. Create [snippets](https://code.visualstudio.com/docs/editor/userdefinedsnippets) for header files with [this configuration](https://github.com/chromium/chromium/blob/master/tools/vscode/settings.json5). This will let you use `hdr` keyboard macro to create the boiler plate header code. Also consider some of [these settings](https://github.com/chromium/chromium/blob/master/tools/vscode/settings.json5) and [more tips](https://chromium.googlesource.com/chromium/src/+show/lkgr/docs/vscode.md).

2. To format GN files on save, [consider using this extension](https://marketplace.visualstudio.com/items?itemName=persidskiy.vscode-gnformat).
