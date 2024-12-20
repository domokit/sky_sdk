dart:zircon
===========

These are the Dart bindings for the Zircon [kernel
interface](https://fuchsia.googlesource.com/zircon/+/HEAD/docs/)

This package exposes a `System` object with methods for may Zircon system
calls, a `Handle` type representing a Zircon handle and a `HandleWaiter` type
representing a wait on a handle. `Handle`s are returned by various methods on
`System` and `HandleWaiter`s are returned by `handle.asyncWait(...)`.
