These instructions can be used to prepare a tabulated summary of the differences in the sizes of two AOT snapshots. The instructions assume that the Flutter Engine has been [setup](../contributing/Setting-up-the-Engine-development-environment.md) on the host (at `FLUTTER_ENGINE` in these instructions).

Build the AOT snapshot (`flutter build aot`) for the application but pass in the `--verbose` flag. We need to find the `gen_snapshot` invocation and re-run it with an extra option (`--print-instructions-sizes-to`). If you are instrumenting with a local engine, the `flutter build` [takes a `--local-engine` and `--local-engine-host` flag](../Debugging-the-engine.md#running-a-flutter-app-with-a-local-engine) as well.

Here is an example of the updated invocation. Specify the path to a JSON file that acts as a summary (`SUMMARY_LOCATION` in these instructions) as follows.

```bash
$FLUTTER_ENGINE/out/host_debug_unopt/gen_snapshot                    \
  --print-instructions-sizes-to=$SUMMARY_LOCATION                    \
  --causal_async_stacks                                              \
  --packages=.packages                                               \
  --deterministic                                                    \
  --snapshot_kind=app-aot-blobs                                      \
  --vm_snapshot_data=build/aot/vm_snapshot_data                      \
  --isolate_snapshot_data=build/aot/isolate_snapshot_data            \
  --vm_snapshot_instructions=build/aot/vm_snapshot_instr             \
  --isolate_snapshot_instructions=build/aot/isolate_snapshot_instr   \
  build/aot/app.dill
```

Save the file at `SUMMARY_LOCATION` as `before.json`

Now, either change the Dart code with the changes you wish to see the effects, or, rebuild the engine to create an updated `gen_snapshot` binary.

After you have made necessary changes, re-run `flutter build aot` again. This step is important because AOT compilation has a [kernel compilation step](../Custom-Flutter-Engine-Embedding-in-AOT-Mode.md#generating-the-kernel-snapshot) before the `gen_snapshot` invocation.

Re-run the gen_snapshot invocation and save the resulting file to `after.json`.

Run the `compare_size.dart` tool and pass in the `before.json` and `after.json` files to generate the summary.

An example invocation looks as follow.

```
$FLUTTER_ENGINE/out/host_debug_unopt/dart-sdk/bin/dart \
  before.json                                          \
  after.json
```

This should dump something like the following to the console.

```
+------------+----------------------------------------------------------+--------------+
| Library    | Method                                                   | Diff (Bytes) |
+------------+----------------------------------------------------------+--------------+
| dart:async | new ZoneSpecification.from                               |        +2136 |
| dart:async | runZoned                                                 |        +1488 |
| dart:async | new _CustomZone                                          |         +927 |
| dart:async | runZoned.<anonymous closure>                             |         +881 |
| dart:async | _rootFork                                                |         +504 |
| dart:async | _rootCreatePeriodicTimer                                 |         +500 |
| dart:async | _rootCreateTimer                                         |         +498 |
| dart:async | _rootRegisterUnaryCallback                               |         +485 |
| dart:async | _rootRegisterBinaryCallback                              |         +485 |
| dart:async | _rootRegisterCallback                                    |         +485 |
| dart:async | _rootPrint                                               |         +453 |
| dart:async | _CustomZone.fork                                         |         +396 |
| dart:async | _rootErrorCallback                                       |         +389 |
| dart:async | _CustomZone.bindUnaryCallbackGuarded                     |         +368 |
| dart:async | _rootHandleUncaughtError                                 |         +342 |
| dart:async | _CustomZone.runBinary                                    |         +296 |
| dart:async | _CustomZone.runUnary                                     |         +293 |
| dart:async | _CustomZone.[]                                           |         +291 |
| dart:async | _CustomZone.registerCallback                             |         +290 |
| dart:async | _CustomZone.run                                          |         +290 |
| dart:async | _CustomZone.registerUnaryCallback                        |         +290 |
| dart:async | _CustomZone.registerBinaryCallback                       |         +290 |
| dart:async | _CustomZone.runBinaryGuarded                             |         +289 |
| dart:async | _CustomZone.runUnaryGuarded                              |         +286 |
| dart:async | _RootZone.fork                                           |         +283 |
| dart:async | _CustomZone.bindCallback                                 |         +259 |
| dart:async | _CustomZone.bindUnaryCallback                            |         +259 |
| dart:async | _CustomZone.bindUnaryCallback.<anonymous closure>        |         +248 |
| dart:async | _RootZone.bindUnaryCallback.<anonymous closure>          |         +248 |
| dart:async | _CustomZone.bindUnaryCallbackGuarded.<anonymous closure> |         +248 |
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
|            | [Stub] Type Test Type: class 'PopupMenuEntry'            |         -128 |
|            | [Stub] Type Test Type: class '_SyncIterator@0150898'     |         -128 |
|            | [Stub] Type Test Type: class 'PopupMenuItem'             |         -128 |
|            | [Stub] Type Test Type: class 'FormFieldState'            |         -128 |
|            | [Stub] Type Test Type: class 'PopupMenuButton'           |         -128 |
|            | [Stub] Type Test Type: class '_SyncIterator@0150898'     |         -131 |
|            | [Stub] Type Test Type: class '_SplayTreeMapNode@3220832' |         -139 |
|            | [Stub] Type Test Type: class '_SplayTreeMapNode@3220832' |         -165 |
| dart:io    | new Directory                                            |         -211 |
| dart:io    | new Link                                                 |         -211 |
+------------+----------------------------------------------------------+--------------+
Total change +24036 bytes.

```
