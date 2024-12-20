# Flutter Engine Build Definition Language

The ***Flutter Engine Build Definition Language*** describes a build on CI
by defining a combination of *sub-builds*, *archives*, *generators* and *dependencies*. It
makes it simple to shard sub-builds by mapping build inputs to workflows, and listing
the sub-build-generated artifacts explicitly. The Build Definition Language, Engine
Recipes V2 and the generation of artifacts using GN+Ninja set the groundwork
for efficient builds with dependency reusability.

## Glossary

* **[recipes](https://github.com/luci/recipes-py)** - domain specific
language for specifying sequences of subprocess calls in a cross-platform and
testable way.
* **Generator** - scripts in Dart, python or bash that combine the output of
sub-builds to generate artifacts.
* **Builder** - a combination of configuration, recipes and a given commit to
build and test artifacts.
* **Build** - a builder running with specific properties, repository and
commit.
* **[GN](https://gn.googlesource.com/gn/)** - a meta-build system that
generates build files for [Ninja](https://ninja-build.org/).
* **[Ninja](https://ninja-build.org)** - Ninja is a small build system with a
focus on speed.
* **CAS** - a service that stores arbitrary binary blobs addressed by (hash of)
their content. It is specialized for low latency, high volume query/read/write
operations.

## USAGE EXAMPLES

Engine build definition files using the Build Definition Language can be found in the
[flutter/engine/ci/builders](https://github.com/flutter/engine/tree/main/ci/builders) directory.

The [engine orchestrator recipe](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipes/engine_v2/)
reads each file in that directory, shards their builds, collects artifacts and
uploads them to the Google Cloud Storage bucket.

The [.ci.yaml file](https://github.com/flutter/engine/blob/main/.ci.yaml) at the
root of the `flutter/engine` repository puts all the components together.
Builds are specified in that file using a property pointing to the build definition
file to be used by engine\_v2 recipes. Full documentation of the `.ci.yaml` file format
can be found [in the Cocoon repository here](https://github.com/flutter/cocoon/blob/main/CI_YAML.md).

The following is a sample build configuration referencing
[android\_aot\_engine.json](https://github.com/flutter/engine/blob/main/ci/builders/mac_android_aot_engine.json)
in the `config_name` under `properties`:

```yaml
  - name: Mac mac_android_aot_engine
    recipe: engine_v2/engine_v2
    timeout: 60
    properties:
      config_name: mac_android_aot_engine
      $flutter/osx_sdk : >-
        { "sdk_version": "15a240d" }

```

## Build Definition Language Assumptions

To keep the build definition language simple the following assumptions were
made during its design:

* A build can be expressed as a set of independent sub-builds.
* A sub-build can be defined as a sequence of a `gn` configuration step,
  a `ninja` build step, followed by self-contained test scripts, and self-contained
  generator scripts. All the elements are optional allowing to use gn+ninja without
  generators or generators without gn+ninja.
* All the sub-builds required by a global generator are defined within the same
configuration file.

## Build configuration file

The build configuration is a json file containing a list of builds, tests,
generators and archives. The following is an example of an empty configuration
file:

```json
{
   "builds": [],
   "tests": [],
   "generators": {
       "tasks": []
   },
   "archives": [
   ]
}
```

Note: tests, generators and archives can be omited if empty.

Build configuration files have to be checked into the
[engine_checkout/ci/builder](https://github.com/flutter/engine/tree/main/ci/builders)
directory where engine v2 recipes will be reading them from.

Configurations with a single build are supported. Single build configurations
are located have to be checked into the
[engine_checkout/ci/builder/standalone](https://github.com/flutter/engine/tree/main/ci/builders/standalone)

A configuration file defines a top-level builder that will show up as a column
in the
[Flutter Dashboard](https://flutter-dashboard.appspot.com/#/build?repo=engine&branch=master).


### Magic variables

Magic variables are special environment variables that can be used as parameters
for generators and test commands in the local and global contexts.

Magic environment variables have the following limitations:
only `${FLUTTER_LOGS_DIR}` is currently supported and it needs to be used
alone within the parameter string(e.g. `["${FLUTTER_LOGS_DIR}"]` is OK
but `["path=${FLUTTER_LOGS_DIR}"]` is not).

The current list of supported magic variables is:

* `${FLUTTER_LOGS_DIR}` - translated to the path of the temporary
  folder where logs are being placed.
* `${LUCI_WORKDIR}` - translated to the LUCI chroot working directory.
* `${LUCI_CLEANUP}` - translated to the LUCI chroot temp directory.
* `${REVISION}` - translated to the engine commit in postsubmit. In presubmit
  it is translated to an empty string.

### Build

A build is a dictionary with a gn command, a ninja command, zero or more
generator commands, zero or more local tests, zero or more local
generators and zero or more output artifacts.

The following is the high level structure of the build component:

```json
{
           "archives": [],
           "drone_dimensions": [],
           "gclient_variables": {},
           "gn": [],
           "name": "host_debug",
           "generators": [],
           "ninja": {},
           "tests": []
           "postsubmit_overrides": {}
}
```

Each build element will be translated to an independent sub-build and its
entire out directory will be uploaded to CAS.

`gn`, `ninja`, `generators`, `tests` and `postsubmit_overrides` properties are optional. Gn and
ninja properties can be used without generators or tests. Generators with
no gn and ninja properties is also supported.

#### Archives

An archive component is used to tell the recipes which artifacts are
generated by the build and where to upload them.

By default the build output is archived to CAS in order to be used
as a dependency for global tests. If no CAS archive
is required `cas_archive": false,` needs to be added to the
configuration.

```json
{
  "name": "host_debug",
  "base_path": "out/host_debug/zip_archives/",
  "type": "gcs",
  "include_paths": [
     "out/host_debug/zip_archives/linux-x64/artifacts.zip"
  ],
  "realm": "production"
}
```

Description of the fields:

* **name:** - by default the entire build output is uploaded to CAS.
 `name` is used to associate the CAS hash to a value that can be referenced
 later as a dependency of global tests. Name is also used to select the folder
 from within src/out to upload to CAS. e.g if the build generates
 src/out/host_debug name must be `host_debug`.
* **base\_path:** - the portion of the path to remove from the full path before
 uploading to its final destination. In the example the above the
 base\_path **“out/host\_debug/zip\_archives”** will be removed from the
 include path **"out/host\_debug/zip\_archives/linux-x64/artifacts.zip"**
 before uploading to GCS, e.g.
 &lt;bucket&gt;/flutter/&lt;commit>/linux-x64/artifacts.zip.
* **Type:** - the type of storage to use. Currently only **“gcs”** and
**“cas”** are supported. "gcs" uploads artifacts to GCS
and "cas" to archive to CAS service. Cas value is used during development where
we need to inspect the generated artifacts without worrying about location or
cleanups. Gcs is expected for any artifacts being consumed by the flutter tool.
* **Include\_paths:** - a list of strings with the paths to be uploaded to a
given destination.
* **cas_archive** - a boolean value indicating whether the build output will
be archived to CAS or not. The default value is true.
* **realm** - a string value of either `production` or `experimental`
where production means the artifact will be uploaded to the location expected
by the flutter tool and experimental will add an `experimental` prefix to the
path to avoid interfering with production artifacts.

#### Drone\_dimensions

A list of strings with key value pairs separated by an equal sign. These
dimensions are used to select the bot where the sub-build will be running.

To find the list of valid keys and values you need to select a [bot from the
swarming UI](https://chromium-swarm.appspot.com/botlist?c=id&c=task&c=os&c=status&d=asc&f=pool%3Aluci.flex.try&k=pool&s=id).
On the `dimensions` section the left column contains the keys and
the right column contains the allowed values. If multiple values are allowed
for a key they are separated using `|` (pipe symbol).

```json
"drone_dimensions": [
  "device_type=none",
  "os=Linux"
]
```

In the previous example, the build containing this drone\_dimensions component
will run on a bot with a Linux OS that does not have any devices attached to it.

Drone dimensions accept values separates by `|` to specify more than one value
for the dimension. E.g. assuming the pool of bots have Ubuntu and Debian bots
a dimension of `"os": "Debian|Ubuntu"` will resolve to use bots running either
Debian or Ubuntu.

#### Gclient\_variables

A dictionary with variables passed to gclient during a gclient sync operation.
They are usually used to add or remove gclient dependencies.

```json
"gclient_variables": {
   "download_android_deps": false
}
```

The example above is used to avoid downloading the
[android sdk dependencies](https://cs.opensource.google/flutter/engine/+/main:DEPS;l=80)
in builders that do not need it.

#### GN

A list of strings representing flags passed to the
[tools/gn](https://github.com/flutter/engine/blob/main/tools/gn) script. The strings can be in the form of “--flag=value” or
“--flag” followed by “value”.

```json
"gn": [
      "--runtime-mode",
      "debug",
      "--prebuilt-dart-sdk",
      "--build-embedder-examples"
],
```

The previous example will prepare the configurations to build a host debug
version using a prebuilt dart sdk and also build the embedder examples.

#### Ninja

A dictionary with two keys: “config” which references the configs created by gn
and “target” which is a list of strings with the Ninja targets to build.

```json
"ninja": {
    "config": "host_debug",
    "targets": [
        "flutter/build/archives:artifacts",
        "flutter/build/archives:embedder",
    ]
},
```

In the example above the ninja command will use the configuration for
host\_debug and will build artifacts and embedder targets as described
by the
[flutter/build/archives/BUILD.gn](https://github.com/flutter/engine/blob/main/build/archives/BUILD.gn)
file.

#### Tests

This section of the build configuration is also known as local tests. It
contains a list of dictionaries with configurations for scripts and
parameters used to run tests inside the current build unit. These tests
should not reference or use anything outside of the commit checkout or
the outputs generated by running gn and ninja sections of the build
configuration.

```json
"tests": [
   {
       "language": "python3",
       "test_timeout_secs": 600,
       "name": "Host Tests for host_debug_impeller_vulkan",
       "parameters": [
           "--variant",
           "host_debug_impeller_vulkan",
           "--type",
           "impeller",
           "--engine-capture-core-dump"
       ],
       "script": "flutter/testing/run_tests.py",
       "contexts": ["android_virtual_device"]
   }
]
```

Description of the fields:

* **language** - the executable used to run the script, e.g. python3, bash.
In general any executable found in the path can be used as language. The
default is empty which means no interpreter will be used to run the script
and it is assumed the script is already an executable with the right
permissions to run in the target platform.
* **test_timeout_secs** - the timeout in seconds for the step running the test. This value overrides the
default 1 hour timeout. When debugging, or if a third-party program is known to misbehave, it is recommended to add timeouts to allow LUCI services to collect logs.
* **name** - the name of the step running the script.
* **parameters** - flags or parameters passed to the script. Parameters
accept magic environment variables(placeholders replaced before executing
the test).
* **Script** - the path to the script to execute relative to the checkout
directory.
* **contexts** - a list of available contexts to add to the text execution step.
The list of supported contexts can be found [here](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipe_modules/flutter_deps/api.py#687). As of 06/20/23 two contexts are supported:
"android_virtual_device" and "metric_center_token".
* **test_if** - a regex of what branches this test should run on. Defaults
to everywhere.

The test scripts will run in a deferred context (failing the step only after
logs have been uploaded). Tester and builder recipes provide an environment
variable called FLUTTER\_LOGS\_DIR pointing a temporary directory where the
test runner can place any logs|artifacts needed to debug issues. At the end
of the test execution the content of FLUTTER\_LOGS\_DIR will be uploaded to
Google Cloud Storage before signaling the pass | fail test state.

Contexts are free form python contexts that communicate with the test script
through environment variables. E.g. metric_center_token saves an access token
to an [environment variable "token_path"](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipe_modules/token_util/api.py#14) for the test to access it.

Note that to keep the recipes generic they don’t know anything about what
the test script is doing and it is the responsibility of the test script to
copy the relevant files to the FLUTTER\_LOGS\_DIR directory.

#### postsubmit_overrides

Used to override top level build properties for postsubmit environments. An example is when we need to run different gn commands for presubmit and postsubmit
environments. Currently only `gn` override is supported.

```json
{
   "name": "host_debug",
   "gn": [
      "--runtime-mode",
      "debug",
      "--prebuilt-dart-sdk",
      "--build-embedder-examples"
   ],
   "ninja": {},
   "postsubmit_overrides": {
     "gn": [
        "--runtime-mode",
        "release"
     ],
   }
}
```

The example above shows how to override the gn command for postsubmit builds of host_debug.


#### Generators

Generators are scripts used to generate artifacts combining the output of two
or more sub-builds. The most common use case is to generate universal binaries for
Mac/iOS artifacts.

Generators can be written in any language but they are required to follow some
guidelines to make them compatible with the engine build system.

The guidelines are as follows:

* Flags receiving paths to resources from multiple sub-builds need to use paths
relative to the checkout (`src/`) directory. If there are global generators in a build
configuration, the engine\_v2 recipes will download the full sub-build archives
to the src/out/&lt;sub-build name> directory.
* Flags receiving paths to output directories must use paths relative to the
src/out folder. This is to be able to reference the artifacts in the global
archives section.
* The script is in charge of generating the final artifact, e.g. if the script
generates multiple files that will be zipped later, then it is the responsibility
of the script to generate the final zip.
* If the generator is producing a Mac/iOS artifact, then it is the responsibility
of the script to embed the signing metadata.

Generators contain a single property “tasks” which is a list of tasks to be
performed.

```json
"generators": {
    "tasks": []
 }
```

The example above represents a generator configuration with an empty list
of tasks.

##### Task

A `task` is a dictionary describing the scripts to be executed.

The property's description is as follows:

* **Name** - the name of the step running the script.
* **Parameters** - flags passed to the script. Both input and output paths must
be relative to the checkout directory.
* **Script**, the script path relative to the checkout repository.
* **Language**, the script language executable to run the script. If empty it is assumed to be bash.

```json
{
    "name": "Debug-FlutterMacOS.framework",
    "parameters": [
        "--dst",
        "out/debug",
        "--arm64-out-dir",
        "out/ios_debug",
        "--simulator-x64-out-dir",
        "out/ios_debug_sim",
        "--simulator-arm64-out-dir",
        "out/ios_debug_sim_arm64"
    ],
    "script": "flutter/sky/tools/create_ios_framework.py",
    "language": "python3"
}
```

### Global Tests

Tests in this section run on a separate bot as independent sub-builds.
As opposed to tests running within builds, global tests have access to the
the outputs of all the builds running in the same orchestrator build. A
use case for global tests is to run flutter/framework tests using the
artifacts generated by an specific engine build.

Global tests currently support two different scenarios:

* flutter/flutter tests with [tester](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipes/engine_v2/tester.py)
  recipe. This workflow checks out flutter/flutter to run any of the existing
  sharded tests using the engine artifacts archived to GCS.
* complicated engine tests that require the outputs from multiple subbuilds
  with [tester_engine](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipes/engine_v2/tester_engine.py).
  This workflow checks out [flutter/engine] and operates over the dependencies passed to it using cas.

Note: the supported scenarios can be later extended to support running devicelab tests although a
[smart scheduler](https://github.com/flutter/flutter/issues/128294) is a prerequisite for
it to be scalable(build/test separation model).

Framework test example:

```json
{
   "tests": [
      {
        "name": "web-tests-1",
        "shard": "web_tests",
        "subshard": "1",
        "test_dependencies": [
          {
            "dependency": "chrome_and_driver",
            "version": "version:111.0a"
          }
        ]
      }
    ]
}
```

The property's description is as follows:

* **name** the name that will be assigned to the sub-build.
* **shard** the flutter/flutter shard test to run. The supported shard names can be found
on the flutter framework [test.dart](https://github.com/flutter/flutter/blob/master/dev/bots/test.dart#L244).
* **subshard** one of the accepted subshard values for shard. Sub-shards are defined as part
of the shard implementation, please look at the corresponding shard implementation to find the
accepted values.
* **test_dependencies** a list of [dependencies](https://flutter.googlesource.com/recipes/+/refs/heads/main/recipe_modules/flutter_deps/api.py#75)
  required for the test to run.

Engine test example:

```json
{
  "tests": [
    {
       "name": "test: clang_tidy android_debug_arm64",
       "recipe": "engine_v2/tester_engine",
       "drone_dimensions": [
         "device_type=none",
         "os=Linux"
       ],
       "dependencies": [
         "host_debug",
         "android_debug_arm64"
       ],
       "tasks": [
         {
            "name": "test: clang_tidy android_debug_arm64",
            "parameters": [
              "--variant",
              "android_debug_arm64",
              "--lint-all",
              "--shard-id=0",
              "--shard-variants=host_debug"
            ],
            "max_attempts": 1,
            "script": "flutter/ci/clang_tidy.sh",
            "test_timeout_secs": 600,
         }
       ]
    }
  ]
}
```

The property's description is as follows:

* **name** the name to assign to the sub-build.
* **recipe** the recipe name to use if different than tester.
* **drone_dimensions** a list of strings with key values to select the
  bot where the test will run.
* **dependencies** a list of build outputs required
  by the test. These build outputs are referenced by the name of build
  generating the output. This type of dependency is shared using CAS and
  the contents are mounted in checkout/src/out. E.g. a build configuration
  building the `host_engine` configuration will upload the content of
  checkout/src/out/host_engine to CAS and a global test with a `host_engine`
  dependency will mount the content of host engine in the same location of
  the bot running the test.
* **tasks** a list of dictionaries representing scripts and parameters to run them.

Example task configuration:

```json
{
    "name": "test: clang_tidy android_debug_arm64",
    "parameters": [
       "--variant",
       "android_debug_arm64",
       "--lint-all",
       "--shard-id=0",
       "--shard-variants=host_debug"
    ],
    "max_attempts": 1,
    "script": "flutter/ci/clang_tidy.sh"
}
```

The property's description is as follows:

* **name** the name assigned to the step running the script.
* **parameters** a list of parameters passed to the script execution.
* **max_attempts** an integer with the maximum number of runs in case of failure.
* **script** the path relative to checkout/src/ to run.
* **test_timeout_secs** - the timeout in seconds for the step running the test. This value overrides the
default 1 hour timeout. When debugging, or if a third-party program is known to misbehave, it is recommended to add timeouts to allow LUCI services to collect logs.

### Global Generators

Global generators follow the same format as local generators but defined at
the build top level. The main difference is that global generators can create
new artifacts combining outputs of multiple sub-builds.

### Global Archives

The archives component provides instructions to upload the artifacts generated
by the global generators. Is a list of dictionaries with three keys: `source` and
`destination`, and `realm`. `source` is a path relative to the checkout repository,
`destination` is a relative path to &lt;bucket>/flutter/&lt;commit>, and `realm` is
a string with either `production` or `experimental` value.

The realm value is used to build the destination path of the artifacts.
`production` will upload the artifacts to the location expected by the flutter
tool and `experimental` will add experimental as a prefix to the path to avoid
interfering with the production artifacts.

```json
"archives": [
    {
        "source": "out/debug/artifacts.zip",
        "destination": "ios/artifacts.zip",
        "realm": "production"
    },
]
```

The example above will cause the file &lt;checkout>/out/debug/artifacts.zip to
be uploaded &lt;bucket>/flutter/&lt;commit>/ios/artifacts.zip.

## Triaging global generators

Global generators can run locally if all their sub-build dependencies are
downloaded. This section explains how to triage a local generator.

The instructions on this section can be used to triage problems with artifacts
created by glocal generators(E.g.`Debug|Release|Profile-ios-Flutter.xcframework`)
using the build outputs of CI subbuilds. During the migration to engine v2 we had
a regression in the size of the flutter libraries, using this process we were able
to inspect the files as they were generated by the CI, make changes to the generators
and run the generators locally to validate the fixes.

### Prerequisites (one time installation)

#### Install CAS utility

CAS client is required to download the sub-build artifacts. To install
it in your machine run the following steps:

* `mkdir $HOME/tools`
* Download and unzip CAS binaries from
  [https://chrome-infra-packages.appspot.com/p/infra/tools/luci/cas]
* Add $HOME/tools to path and your ~/.bashrc

#### Gclient engine checkout

Create a gclient checkout following instructions from
[Setting up the engine environment](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment).

### Download sub-builds to the gclient checkout out folder

CAS sub-build artifacts can be downloaded using information from a LUCI build.
Using [https://ci.chromium.org/p/flutter/builders/prod/Mac%20mac_ios_engine/2818]
as an example the execution details from steps 13 - 17 show the commands to
download the archives. `-dir` parameter needs to be updated to point to the
relative or full path to the out folder in your gclient checkout.

These are the commands to execute for our example build:

```bash
pushd <gclient checkout>/src/out
cas download -cas-instance projects/chromium-swarm/instances/default_instance -digest 39f15436deaed30f861bdd507ba6297f2f26a2ff13d45acfd8819dbcda346faa/88 -dir ./
cas download -cas-instance projects/chromium-swarm/instances/default_instance -digest bdec3208e70ba5e50ee7bbedaaff4588d3f58167ad3d8b1c46d29c6ac3a18c00/94 -dir ./
cas download -cas-instance projects/chromium-swarm/instances/default_instance -digest d19edb65072aa9d872872b55d3c270db40c6a626c8a851ffcb457a28974f3621/84 -dir ./
cas download -cas-instance projects/chromium-swarm/instances/default_instance -digest ac6f08662d18502cfcd844771bae736f4354cb3fe209552fcf2181771e139e0b/86 -dir ./
cas download -cas-instance projects/chromium-swarm/instances/default_instance -digest 1d4d1a3b93847451fe69c1939d7582c0d728b198a40abd06f43d845117ef3214/86 -dir ./
```

The previous commands will create the ios_debug, ios_debug_sim, ios_debug_sim_arm64,
ios_profile, and ios_release folders with the artifacts generated by its
corresponding sub-builds.

Once the checkout and dependencies are available locally we can cd|pushd to the
root of the client checkout and run the global generator. The command can be
copied verbatim from the executions details of the build.

The following example will run the generator to create the ios artifacts:

```bash
python3 flutter/sky/tools/create_ios_framework.py   \
  --dst out/release                                 \
  --arm64-out-dir out/ios_release                   \
  --simulator-x64-out-dir out/ios_debug_sim         \
  --simulator-arm64-out-dir out/ios_debug_sim_arm64 \
  --dsym                                            \
  --strip
```
