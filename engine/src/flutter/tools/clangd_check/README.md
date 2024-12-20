# `clangd_check`

`clangd_check` is a tool to run clangd on a codebase and check for diagnostics.

The practical use of this tool is intentionally limited; it's designed to
provide a quick way to verify that `clangd` is able to parse and analyze a
C++ codebase.

## Usage

```sh
dart ./tools/clangd_check/bin/main.dart
```

On success, and with no diagnostics, `clangd_check` will exit with status 0.

By default, `clangd_check` will try to infer the path of `clangd`, as well as
the path to `--compile-commands-dir` based on what artifacts are present in
`$ENGINE/src/out`.

You can also specify the path to `clangd` and `--compile-commands-dir` manually:

```sh
dart ./tools/clangd_check/bin/main.dart \
  --clangd ../buildtools/mac-arm64/clang/bin/clangd \
  --compile-commands-dir ../out/host_Debug_unopt_arm64
```
