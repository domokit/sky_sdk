// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { createWasmInstantiator } from "./instantiate_wasm.js";
import { resolveUrlWithSegments } from "./utils.js";

export const loadCanvasKit = (deps, config, browserEnvironment, canvasKitBaseUrl) => {
  window.flutterCanvasKitLoaded = (async () => {
    if (window.flutterCanvasKit) {
      // The user has set this global variable ahead of time, so we just return that.
      return window.flutterCanvasKit;
    }
    const supportsChromiumCanvasKit = browserEnvironment.hasChromiumBreakIterators && browserEnvironment.hasImageCodecs;
    if (!supportsChromiumCanvasKit && config.canvasKitVariant == "chromium") {
      throw "Chromium CanvasKit variant specifically requested, but unsupported in this browser";
    }
    const useChromiumCanvasKit = supportsChromiumCanvasKit && (config.canvasKitVariant !== "full");
    let baseUrl = canvasKitBaseUrl;
    if (useChromiumCanvasKit) {
      baseUrl = resolveUrlWithSegments(baseUrl, "chromium");
    }
    let canvasKitUrl = resolveUrlWithSegments(baseUrl, "canvaskit.js");
    if (deps.flutterTT.policy) {
      canvasKitUrl = deps.flutterTT.policy.createScriptURL(canvasKitUrl);
    }
    const wasmInstantiator = createWasmInstantiator(resolveUrlWithSegments(baseUrl, "canvaskit.wasm"));
    const canvasKitModule = await import(canvasKitUrl);
    window.flutterCanvasKit = await canvasKitModule.default({
      instantiateWasm: wasmInstantiator,
    });
    return window.flutterCanvasKit;
  })();
  return window.flutterCanvasKitLoaded;
}
