# `flutter/third_party`

This directory contains third-party code that is a combination of:

- Code that is vendored into the Flutter repository, from an external source.
  For example, we might have `third_party/glfw`, which contains the GLFW
  library, vendored from an external repository.

  > 💡 **TIP**: See [`DEPS`](../DEPS) for where these sources are declared.

- Code that originates from another repository, but is copied (sometimes with
  alterations) into the Flutter repository. For an example, see
  [`third_party/spring_animation`](spring_animation/README.md).

- Code that is licensed separately from the rest of the Flutter repository.
  For example, see [`third_party/txt`](txt/).

When adding a new _externally_ sourced third-party library, update `.gitignore`:

```diff
# Ignores all third_party/ directories except for the ones we want to track.

+ !{folder_name}/
```
