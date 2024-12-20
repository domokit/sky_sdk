// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { createWasmInstantiator } from "./instantiate_wasm.js";
import { resolveUrlWithSegments } from "./utils.js";

export const loadSkwasm = async (deps, config, browserEnvironment, baseUrl) => {
  const fileStem = (browserEnvironment.crossOriginIsolated && !config.forceSingleThreadedSkwasm) ? "skwasm" : "skwasm_st";
  const rawSkwasmUrl = resolveUrlWithSegments(baseUrl, `${fileStem}.js`)
  let skwasmUrl = rawSkwasmUrl;
  if (deps.flutterTT.policy) {
    skwasmUrl = deps.flutterTT.policy.createScriptURL(skwasmUrl);
  }
  const wasmInstantiator = createWasmInstantiator(resolveUrlWithSegments(baseUrl, `${fileStem}.wasm`));
  const skwasm = await import(skwasmUrl);
  return await skwasm.default({
    instantiateWasm: wasmInstantiator,
    // When hosted via a CDN or some other url that is not the same
    // origin as the main script of the page, we will fail to create
    // a web worker with the bootstrapping script. This workaround will
    // make sure that the worker JS can be loaded regardless of where
    // it is hosted.
    mainScriptUrlOrBlob: new Blob(
      [`import '${skwasmUrl}'`],
      { 'type': 'application/javascript' },
    ),
  });
}
