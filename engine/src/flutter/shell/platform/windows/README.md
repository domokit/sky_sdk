# Windows Platform Embedder

This code is the glue between the Flutter engine and the Windows platform.
It is responsible for:

1. Launching the Flutter engine.
2. Providing a view for the Flutter engine to render into.
3. Dispatching events to the Flutter engine.

For more information on embedders, see the
[Flutter architectural overview](https://docs.flutter.dev/resources/architectural-overview).

> [!CAUTION]
> This is a best effort attempt to document the Windows embedder. It is not
> guaranteed to be up to date or complete. If you find a discrepancy, please
> [send a pull request](https://github.com/flutter/engine/compare)!

See also:

1. [Flutter tool's Windows logic](https://github.com/flutter/flutter/tree/master/packages/flutter_tools/lib/src/windows) - Builds and runs Flutter Windows apps on
the command line.
1. [Windows app template](https://github.com/flutter/flutter/tree/master/packages/flutter_tools/templates/app/windows.tmpl) - The entrypoint for Flutter Windows app. This
launches the Windows embedder.
1. [`platform-windows` GitHub issues label](https://github.com/flutter/flutter/issues?q=is%3Aopen+label%3Aplatform-windows+sort%3Aupdated-desc)
1. [`#hackers-desktop` Discord channel](https://discord.com/channels/608014603317936148/608020180177780791)

## Developing

See:

1. [Setting up the Engine development environment](https://github.com/flutter/flutter/wiki/Setting-up-the-Engine-development-environment)
2. [Compiling for Windows](https://github.com/flutter/flutter/wiki/Compiling-the-engine#compiling-for-windows)
3. [Debugging Windows builds with Visual Studio](https://github.com/flutter/flutter/wiki/Debugging-the-engine#debugging-windows-builds-with-visual-studio)

### Notable files

Some notable files include:

1. [`flutter_windows_engine.h`](https://github.com/flutter/engine/blob/main/shell/platform/windows/flutter_windows_engine.h) - Connects the Windows embedder to the Flutter engine.
1. [`flutter_windows_view.h`](https://github.com/flutter/engine/blob/main/shell/platform/windows/flutter_windows_view.h) - The logic for a Flutter view.
1. [`flutter_window.h`](https://github.com/flutter/engine/blob/main/shell/platform/windows/flutter_window.h) - Integrates a Flutter view with Windows (using a win32 child window).
1. [`//shell/platform/embedder/embedder.h`](https://github.com/flutter/engine/blob/main/shell/platform/embedder/embedder.h) - The API boundary between the Windows embedder and the Flutter engine.
