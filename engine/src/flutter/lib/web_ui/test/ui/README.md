# UI Tests

These tests are intended to be renderer-agnostic. These tests should not use
APIs which only exist in either the HTML or CanvasKit renderers.

In practice, this means these tests should only use `dart:ui` APIs or
`dart:_engine` APIs which are not renderer-specific.

## Notes

These tests should call `setUpUnitTests()` at the top level to initialize the
renderer they are expected to run.
