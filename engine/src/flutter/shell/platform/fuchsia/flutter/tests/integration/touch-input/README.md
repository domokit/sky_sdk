# touch-input

`touch-input-test` exercises touch through a child view (in this case, the `touch-input-view` Dart component) and asserting
the precise location of the touch event. We validate a touch event as valid through two ways:
- By attaching the child view, injecting touch, and validating that the view reports the touch event back with the correct coordinates.
- By embedding a child view into a parent view, injecting touch into both views, and validating that each view reports its touch event back with the correct coordinates.

```shell
Injecting the tap event
[touch-input-test.cm] INFO: [portable_ui_test.cc(193)] Injecting tap at (-500, -500)

View receives the event
[flutter_jit_runner] INFO: touch-input-view.cm(flutter): touch-input-view received tap: PointerData(embedderId: 0, timeStamp: 0:01:03.623259,
change: PointerChange.add, kind: PointerDeviceKind.touch, signalKind: PointerSignalKind.none, device: -4294967295, pointerIdentifier: 0,
physicalX: 319.99998331069946, physicalY: 199.99999284744263, physicalDeltaX: 0.0, physicalDeltaY: 0.0, buttons: 0, synthesized: false,
pressure: 0.0, pressureMin: 0.0, pressureMax: 0.0, distance: 0.0, distanceMax: 0.0, size: 0.0, radiusMajor: 0.0, radiusMinor: 0.0,
radiusMin: 0.0, radiusMax: 0.0, orientation: 0.0, tilt: 0.0, platformData: 0, scrollDeltaX: 0.0, scrollDeltaY: 0.0, panX: 0.0, panY: 0.0,
panDeltaX: 0.0, panDeltaY: 0.0, scale: 0.0, rotation: 0.0)

Successfully received response from view
[touch-input-test.cm] INFO: [touch-input-test.cc(162)] Received ReportTouchInput event
[touch-input-test.cm] INFO: [touch-input-test.cc(255)] Expecting event for component touch-input-view at (320, 200)
[touch-input-test.cm] INFO: [touch-input-test.cc(257)] Received event for component touch-input-view at (320, 200), accounting for pixel scale of 1
```

Some interesting details (thanks to abrusher@):

There exists two coordinate spaces within our testing realm. The first is `touch-input-view`'s "logical" coordinate space. This
is determined based on `touch-input-view`'s size and is the space in which it sees incoming events. The second is the "injector"
coordinate space, which spans [-1000, 1000] on both axes.

The size/position of a view doesn't always match the bounds of a display exactly. As a result, Scenic has a separate coordinate space
to specify the location at which to inject a touch event. This is always fixed to the display bounds. Scenic knows how to map this
coordinate space onto the client view's space.

For example, if we inject at (-500, -500) `touch-input-view` will see a touch event at the middle of the upper-left quadrant of the screen.

## Running the Test

Reference the Flutter integration test [documentation](https://github.com/flutter/engine/blob/main/shell/platform/fuchsia/flutter/tests/integration/README.md) at //flutter/shell/platform/fuchsia/flutter/tests/integration/README.md

## Playing around with `touch-input-view`

Build Fuchsia with `terminal.qemu-x64`
```shell
fx set terminal.qemu-x64 && fx build
```

Build flutter/engine
```shell
$ENGINE_DIR/flutter/tools/gn --fuchsia --no-lto && ninja -C $ENGINE_DIR/out/fuchsia_debug_x64 flutter/shell/platform/fuchsia/flutter/tests/
integration/touch_input:tests
```

Start a Fuchsia package server
```shell
cd "$FUCHSIA_DIR"
fx serve
```

Publish `touch-input-view`
```shell
$FUCHSIA_DIR/.jiri_root/bin/fx pm publish -a -repo $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files -f $ENGINE_DIR/out/
fuchsia_debug_x64/gen/flutter/shell/platform/fuchsia/flutter/tests/integration/touch-input/touch-input-view/touch-input-view/touch-input-view.far
```

Launch Fuchsia emulator in a graphical environment
```shell
ffx emu start
```

**Before proceeding, make sure you have successfully completed the "Set a Password" screen**

Add `touch-input-view`
```shell
ffx session add fuchsia-pkg://fuchsia.com/touch-input-view#meta/touch-input-view.cm
```
