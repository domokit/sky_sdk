# header_guard_check

A tool to check that C++ header guards are used consistently in the engine.

```shell
# Assuming you are in the `flutter` root of the engine repo.
dart ./tools/header_guard_check/bin/main.dart
```

The tool checks _all_ header files for the following pattern:

```h
// path/to/file.h

#ifndef PATH_TO_FILE_H_
#define PATH_TO_FILE_H_
...
#endif  // PATH_TO_FILE_H_
```

If the header file does not follow this pattern, the tool will print an error
message and exit with a non-zero exit code. For more information about why we
use this pattern, see [the Google C++ style guide](https://google.github.io/styleguide/cppguide.html#The__define_Guard).

## Automatic fixes

The tool can automatically fix header files that do not follow the pattern:

```shell
dart ./tools/header_guard_check/bin/main.dart --fix
```

## Advanced usage

### Restricting the files to check

By default, the tool checks all header files in the engine. You can restrict the
files to check by passing a relative (to the `engine/src/flutter` root) paths to
include:

```shell
dart ./tools/header_guard_check/bin/main.dart --include impeller
```
