Android Toolkit
===============

Type-safe managed wrappers around Android objects vended by the NDK. Does not
require linking to libandroid.so. The symbols are resolved via dynamic runtime
lookup so that the toolkit can be built with an older NDK but still run on
modern Android versions and use the latest features.
