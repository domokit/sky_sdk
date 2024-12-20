# RBE for Flutter Engine Developers

## Overview

This is documentation on setting up RBE for building the Flutter engine. It is
expected to work for Googlers only on corp Linux, macOS, and Windows hosts,
including cloudtop instances.

## Getting started

The first step is to add an entry to the `.gclient` file. The entry to add is
`"use_rbe": True` in the `custom_vars` section. It should look like this:

```
solutions = [
  {
    "managed": False,
    "name": "src/flutter",
    "url": "git@github.com:zanderso/engine.git",
    "custom_deps": {},
    "custom_vars": {
      "use_rbe": True,
    },
    "deps_file": "DEPS",
    "safesync_url": "",
  },
]
```

After making this edit, you must be authenticated as a Googler by CIPD so that
the RBE configurations can be downloaded from the `flutter_internal`
[CIPD bucket](https://chrome-infra-packages.appspot.com/p/flutter_internal):

```
cipd auth-login
```

After authentication successfully, run `gclient sync -D`.

## Running an RBE build

In the engine repo, all RBE builds must be initiated through the `et` tool whose
entrypoint is the script `//flutter/bin/et`. This is so that the local RBE
proxy is correctly initialized and shut down around invocations of `ninja`.

### gcloud

Before running an RBE build, you must be authenticated with the Google cloud
project that owns the RBE worker pool. A gcloud SDK is checked out with the
engine repo under e.g. `//flutter/buildtools/mac-arm64/gcloud`.
(Replace `mac-arm64` with the directory name that is correct for your host
platform.)

The `gcloud` tool in this SDK must be on your path. The tool lives under
`//flutter/buildtools/mac-arm64/gcloud/bin`, which is the path to add to your
`PATH` environment variable. Alternatively, you can get the gcloud SDK on your
path by installing it on your system by following the instructions at
[https://cloud.google.com/sdk/docs/install](https://cloud.google.com/sdk/docs/install).

On macOS, before running the `gcloud` command ensure that `python3` is on your
path, and does not come from e.g. homebrew. The command `which python3` should
return `/usr/bin/python3`.

```
gcloud init --project flutter-rbe-prod
```

If you get an error from `bootstrap` about not being able to find `Application
Default Credentials` you may need to execute the following to create the default
credentials:

```sh
gcloud auth application-default login
```

### Listing builds

The builds available to the `et` tool are those specified by the build
configuration json files under `//flutter/ci/builders`. A list of builds
suitable for local development can also be printed by running `et help build`.
The list of all builds can be printed by running `et help build --verbose`.
Builds in the verbose list prefixed with `ci/` or `ci\` are exactly the builds
run on CI with the same GN flags and Ninja targets.

### Running builds

To run a build, pass the name of the configuration to the `et build` command:

```
et build -c host_debug
```

If RBE is working correctly, you should see logs like the following:

```
[2024-04-22T09:58:48.643][windows/host_debug: GN]: OK
[2024-04-22T09:58:59.361][windows/host_debug: RBE startup]: OK
```

To disable RBE in a build, pass the flag `--no-rbe` to the `build` command.

```
et build -c host_debug --no-rbe
```

Since LTO is slow and rarely useful in local builds, `et` disables it by default
in all builds, even when it is specified by a build configuration. To enable
it, pass the `--lto` flag to the `build` command.

```
et build -c host_debug --lto
```

### Customizing builds

Beyond enabling/disabling RBE and LTO, the `build` command does not currently
support customizing builds at the command line.

If you need custom GN flags or Ninja targets, then this can be achieved by
making local edits to the build configuration json files in your checkout. In
particular, the file `//flutter/ci/builders/local_engine.json` is intended to
contain builds that are commonly used for local engine development.

If a configuration does not exist there, and will be needed by multiple people
over a long period of time, consider checking in the new configuration so that
it will be built on CI, which will keep the RBE cache warm for fast local
builds.

On the other hand, if a build is intended to produce artifacts or run tests on
CI, then it should _not_ go in `local_engine.json`. Instead, it should go in
the json file appropriate for its purpose. There are many examples available,
so following suit is likely your best bet, but if you are unsure, ask in the
`hackers-engine` Discord.

## Maintaining Compatibility with RBE

RBE is sensitive to what paths in compile commands look like. In particular, all
paths in compile commands must be relative paths, and those relative paths must
all resolve to files within the engine source tree. In practice, this means
that the GN function `rebase_path()` should only lack a second parameter when
its result won’t be used in a compile or link command. There are unfortunately
many examples in the tree where `rebase_path()` lacks a second parameter.
However to be on the safe side a good rule of thumb is: do not add new usages
of the single parameter `rebase_path()`.

## Troubleshooting

### Error obtaining credentials

If you get the following error while running `et build`:

```shell
E0815 09:30:03.169505 1413247 main.go:147] Error obtaining credentials: application default credentials were invalid: could not get valid Application Default Credentials token: oauth2: cannot fetch token: 400 Bad Request
```

Check your `${HOME}/.config/gcloud/application_default_credentials.json` to see if you are signed into another project / account. Run the following to reset the default credentials:

```shell
gcloud auth application-default login
```

### Slow builds

RBE builds can be slow for a few different reasons. The most common reason is
likely to be that the remote caches are cold. When the caches are warm, a
compile step consists only of downloading the compiled TU from the cache. When
the caches are cold, the remote workers must run the compilation commands,
which takes more time. If the worker pool is overloaded, compile commands may
run locally instead, which will also be slower.

RBE builds can also be slow if your network connection is bandwidth constrained.
Anecdotally, even with a warm cache, I have noticed slow builds from home due
to RBE saturating my low-tier Comcast Business connection.

For developers on a macOS host device, ensure that you're using the same version
of Xcode as is in use on the bots. The value of "Build version" returned by
`xcodebuild -version` should match the `sdk_version` value set in
`ci/builders/local_engine.json` for the build you're running.

For Googlers on a corp macOS device, both RBE and non-RBE builds can be slow due
to various background and monitoring processes running. See
[here](https://buganizer.corp.google.com/issues/324404733#comment16) for how to
disable some of them. You should also disable Spotlight scanning of the engine
source directory as described
[here](go/building-chrome-mac#add-the-source-directory-to-the-spotlight-privacy-list).

When RBE builds are slow, non-RBE builds may be faster, especially incremental
builds. You can disable remote builds without invalidating your existing build
by setting the environment variable `RBE_exec_strategy=local`.

### Proxy status and debug logs
> [!WARNING]
> Since `et` will start and stop the local RBE proxy while performing a build,
> the following command will only work when a build is running.

The status of the local RBE proxy can be queried with the following command

```
buildtools/mac-arm64/reclient/reproxystatus
```

It will give output describing the number of actions completed and in progress,
and the number of remote executions, local executions, and remote cache hits.

For example:

```sh
$ reproxystatus
Reproxy(unix:///tmp/reproxy.sock) is OK
Actions completed: 4405 (750 cache hits, 3075 racing locals, 580 racing remotes)
Actions in progress: 11
QPS: 12
```

The logs for RBE live under the system `/tmp` folder in the files `/tmp/reproxy.
{INFO,WARNING,ERROR}` and `/tmp/bootstrap.{INFO,WARNING,ERROR}`.

In CI runs, the RBE logs are also available by following the links as in the
screenshot below under `teardown remote execution > collect rbe logs`:

![LUCI logs links](luci.png "LUCI logs links")

### Dependency analysis failures

These logs can be useful to highlight failures in RBE’s dependency analysis. The
dependency analysis can fail for a variety of reasons, but a common one during
development is likely to be that the source file is really malformed somehow.
This can be debugged by doing a local build with RBE turned off.


## References

* Code for RBE (i.e. reproxy, rewrapper, bootstrap, etc.) lives in
  [this GitHub repository](https://github.com/bazelbuild/reclient). The tools are not
  well-documented, so the source code is the source of truth for the command
  line flags that they accept, for example.
* Internal-facing RBE migration guide is [here](go/reclient-migration-guide).
  (Mostly focused on Chrome and Android, so not all parts are relevant to
  Flutter.)
* The version of RBE for local development is set in the DEPS file
  [here](https://github.com/flutter/engine/blob/8578edf9c9393471ca9eab18e9154f0e6066dcb6/DEPS#L53).
  It needs to be manually rolled occasionally.
* The version of RBE used by CI is set in a LUCI recipe
  [here](https://flutter.googlesource.com/recipes/+/be12675150183af68223f5fbc6e0f888a1139e79/recipe_modules/rbe/api.py#16).
  It also needs to be manually rolled occasionally.
* Googler-only RBE configuration files live in the CIPD bucket
  [here](https://chrome-infra-packages.appspot.com/p/flutter_internal/rbe/reclient_cfgs).
  They need to be updated when we roll clang to a new version as described
  [here](https://github.com/flutter/engine/pull/52062#issuecomment-2050902282).
* Flutter’s RBE worker pool is defined
  [here](https://source.corp.google.com/piper///depot/google3/configs/cloud/gong/services/flutter_rbe/modules/infra/prod/main.tf).
* Using RBE for Engine clang-tidy is blocked on [b/326591374](http://b/326591374).
