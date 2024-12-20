# Web-specific coding conventions and terminology

Here you will find various naming and structural conventions used in the Web
engine code. This is not a code style guide. For code style refer to
[Flutter's style guide][1]. This document does not apply outside the `web_ui`
directory.

## CanvasKit Renderer

All code specific to the CanvasKit renderer lives in `lib/src/engine/canvaskit`.

CanvasKit bindings should use the exact names defined in CanvasKit's JavaScript
API, even if it violates Flutter's style guide, such as function names that
start with a capital letter (e.g. "MakeSkVertices"). This makes it easier to find
the relevant code in Skia's source code. CanvasKit bindings should all go in
the `canvaskit_api.dart` file.

Files and directories should use all-lower-case "canvaskit", without
capitalization or punctuation (such as "canvasKit", "canvas-kit", "canvas_kit").
This is consistent with Skia's conventions.

Variable, function, method, and class names should use camel case, i.e.
"canvasKit", "CanvasKit".

In documentation (doc comments, flutter.dev website, markdown files,
blog posts, etc) refer to Flutter's usage of CanvasKit as "CanvasKit renderer"
(to avoid confusion with CanvasKit as the standalone library, which can be used
without Flutter).

Classes that wrap CanvasKit classes should replace the `Sk` class prefix with
`Ck` (which stands for "CanvasKit"), e.g. `CkPaint` wraps `SkPaint`, `CkImage`
wraps `SkImage`.

## HTML Renderer

All code specific to the HTML renderer lives in `lib/src/engine/html`.

In documentation (doc comments, flutter.dev website, markdown files,
blog posts, etc) refer to Flutter's HTML implementation as "HTML renderer". We
include SVG, CSS, and Canvas 2D under the "HTML" umbrella.

The implementation of the layer system uses the term "surface" to refer to
layers. We rely on persisting the DOM information across frames to gain
efficiency. Each concrete implementation of the `Surface` class should start
with the prefix `Persisted`, e.g. `PersistedOpacity`, `PersistedPicture`.

## Semantics

The semantics (accessibility) code is shared between CanvasKit and HTML. All
semantics code lives in `lib/src/engine/semantics`.

## Text editing

Text editing code is shared between CanvasKit and HTML, and it lives in
`lib/src/engine/text_editing`.

## Common utilities

Small common utilities do not need dedicated directories. It is OK to put all
such utilities in `lib/src/engine` (see, for example, `alarm_clock.dart`).

[1]: https://github.com/flutter/flutter/wiki/Style-guide-for-Flutter-repo
