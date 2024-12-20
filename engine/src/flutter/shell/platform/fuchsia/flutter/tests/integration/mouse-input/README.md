# mouse-input

`mouse-input-test` exercises mouse input through a child view (in this case, the `mouse-input-view` Dart component) and
asserting the location as well as what button was used (mouse down, mouse up, wheel, etc) during the event. We do this by
attaching the child view, injecting mouse input, and validating that the view reports the event back with the expected
payload.

```shell
Injecting the mouse input
[mouse-input-test.cm] INFO: [portable_ui_test.cc(227)] Injecting mouse input

View receives the event
[flutter_jit_runner] INFO: mouse-input-view.cm(flutter): mouse-input-view received input: PointerData(embedderId: 0, timeStamp: 23:18:05.031003, change: PointerChange.add, kind: PointerDeviceKind.mouse, signalKind: PointerSignalKind.none, device: 4294967295, pointerIdentifier: 0, physicalX: 641.4656372070312, physicalY: 402.9313049316406, physicalDeltaX: 0.0, physicalDeltaY: 0.0, buttons: 0, synthesized: true, pressure: 0.0, pressureMin: 0.0, pressureMax: 0.0, distance: 0.0, distanceMax: 0.0, size: 0.0, radiusMajor: 0.0, radiusMinor: 0.0, radiusMin: 0.0, radiusMax: 0.0, orientation: 0.0, tilt: 0.0, platformData: 0, scrollDeltaX: 0.0, scrollDeltaY: 0.0, panX: 0.0, panY: 0.0, panDeltaX: 0.0, panDeltaY: 0.0, scale: 0.0, rotation: 0.0)

Successfully received response from view
[mouse-input-test.cm] INFO: [mouse-input-test.cc(120)] Received MouseInput event
[mouse-input-test.cm] INFO: [mouse-input-test.cc(207)] Client received mouse change at (641.466, 402.931) with buttons 0.
[mouse-input-test.cm] INFO: [mouse-input-test.cc(211)] Expected mouse change is at approximately (641, 402) with buttons 0.
```

## Running the Test

Reference the Flutter integration test [documentation](https://github.com/flutter/engine/blob/main/shell/platform/fuchsia/flutter/tests/integration/README.md) at `//flutter/shell/platform/fuchsia/flutter/tests/integration/README.md`

## Playing around with `mouse-input-view`

Build Fuchsia with `terminal.qemu-x64`
```shell
fx set terminal.qemu-x64 && fx build
```

Build flutter/engine
```shell
$ENGINE_DIR/flutter/tools/gn --fuchsia --no-lto && ninja -C $ENGINE_DIR/out/fuchsia_debug_x64 flutter/shell/platform/fuchsia/flutter/tests/integration/mouse-input:tests
```

Start a Fuchsia package server
```shell
cd "$FUCHSIA_DIR"
fx serve
```

Publish `mouse-input-view`
```shell
$FUCHSIA_DIR/.jiri_root/bin/fx pm publish -a -repo $FUCHSIA_DIR/$(cat $FUCHSIA_DIR/.fx-build-dir)/amber-files -f $ENGINE_DIR/out/fuchsia_debug_x64/gen/flutter/shell/platform/fuchsia/flutter/tests/integration/mouse-input/mouse-input-view/mouse-input-view/mouse-input-view.far
```

Launch Fuchsia emulator in a graphical environment
```shell
ffx emu start
```

**Before proceeding, make sure you have successfully completed the "Set a Password" screen**

Add `mouse-input-view`
```shell
ffx session add fuchsia-pkg://fuchsia.com/mouse-input-view#meta/mouse-input-view.cm
```
