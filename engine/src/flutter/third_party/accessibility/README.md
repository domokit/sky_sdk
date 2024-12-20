Flutter Accessibility Library
==============

This accessibility library is a fork of the [chromium](https://www.chromium.org) accessibility code at commit
[4579d5538f06c5ef615a15bc67ebb9ac0523a973](https://chromium.googlesource.com/chromium/src/+/4579d5538f06c5ef615a15bc67ebb9ac0523a973).

For the main ax code, the following parts were not imported:
`fuzz_corpus`, `extensions` and `DEPS` files.

`ax/`: https://source.chromium.org/chromium/chromium/src/+/master:ui/accessibility/
`ax_build/`: https://source.chromium.org/chromium/chromium/src/+/master:build/
`base/`: https://source.chromium.org/chromium/chromium/src/+/master:base/
`gfx/`: https://source.chromium.org/chromium/chromium/src/+/master:ui/gfx/

Update to this Library
==============
Bug fixes to the forked files in the four directories should proceed as usual.
New features or changes that change the behaviors of these classes are discouraged.

If you do need to make such change, please log the change at the end of this file.
