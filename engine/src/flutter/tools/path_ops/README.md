# path_ops

A small library that exposes C bindings for Skia's SkPathOps, with a minimal
interface for SkPath.

This library only supports four commands from SkPath: `moveTo`, `lineTo`,
`cubicTo`, and `close`.

This library is a subset of the functionality provided by Skia's `PathKit`
library. It is primarily intended for use with the `vector_graphics` optimizing
compiler. That library uses this one to optimize certain masking and clipping
operations at compile time.

## Testing

The `path_ops` library uses dynamic libraries (i.e. `libpath_ops.dylib` or the
equivalent on your platform) to link against the C++ code in
[`path_ops.cpp`](path_ops.cpp). When run through
[`run_tests.py`](../../testing/run_tests.py), paths are set automatically to
make finding the dynamic library work.

However, if run directly from the command line or IDE, an environment variable
`DYLD_LIBRARY_PATH` (or equivalent on your platform) must be set to the
directory containing the dynamic library. For example, on macOS:

```sh
export DYLD_LIBRARY_PATH=BUILD_DIR
```

... where `BUILD_DIR` is the output directory for the engine build, such as
`../out/host_debug_unopt_arm64`.
