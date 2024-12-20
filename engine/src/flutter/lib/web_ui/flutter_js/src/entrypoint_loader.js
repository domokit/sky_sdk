// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { resolveUrlWithSegments } from "./utils.js";

/**
 * Handles injecting the main Flutter web entrypoint (main.dart.js), and notifying
 * the user when Flutter is ready, through `didCreateEngineInitializer`.
 *
 * @see https://docs.flutter.dev/development/platform-integration/web/initialization
 */
export class FlutterEntrypointLoader {
  /**
   * Creates a FlutterEntrypointLoader.
   */
  constructor() {
    // Watchdog to prevent injecting the main entrypoint multiple times.
    this._scriptLoaded = false;
  }
  /**
   * Injects a TrustedTypesPolicy (or undefined if the feature is not supported).
   * @param {TrustedTypesPolicy | undefined} policy
   */
  setTrustedTypesPolicy(policy) {
    this._ttPolicy = policy;
  }
  /**
   * @deprecated
   * Loads flutter main entrypoint, specified by `entrypointUrl`, and calls a
   * user-specified `onEntrypointLoaded` callback with an EngineInitializer
   * object when it's done.
   *
   * @param {*} options
   * @returns {Promise | undefined} that will eventually resolve with an
   * EngineInitializer, or will be rejected with the error caused by the loader.
   * Returns undefined when an `onEntrypointLoaded` callback is supplied in `options`.
   */
  async loadEntrypoint(options) {
    const { entrypointUrl = resolveUrlWithSegments("main.dart.js"), onEntrypointLoaded, nonce } =
      options || {};
    return this._loadJSEntrypoint(entrypointUrl, onEntrypointLoaded, nonce);
  }

  /**
   * Loads the entry point for a flutter application.
   * @param {import("./types").ApplicationBuild} build
   *   Information about the specific build that is to be loaded
   * @param {*} deps
   *   External dependencies that may be needed to load the app.
   * @param {import("./types").FlutterConfiguration} config
   *   The application configuration. If no callback is specified, this will be
   *   passed along to engine when initializing it.
   * @param {string} nonce
   *   A nonce to apply to the main application script tag, if necessary.
   * @param {import("./types").OnEntrypointLoadedCallback?} onEntrypointLoaded
   *   An optional callback to invoke when the entrypoint is loaded. If no
   *   callback is supplied, the engine initializer and app runner will be
   *   automatically invoked on load, passing along the supplied flutter
   *   configuration.
   */
  async load(build, deps, config, nonce, onEntrypointLoaded) {
    onEntrypointLoaded ??= (engineInitializer) => {
      engineInitializer.initializeEngine(config).then((appRunner) => appRunner.runApp())
    };
    const { entryPointBaseUrl } = config;
    if (build.compileTarget === "dart2wasm") {
      return this._loadWasmEntrypoint(build, deps, entryPointBaseUrl, onEntrypointLoaded);
    } else {
      const mainPath = build.mainJsPath ?? "main.dart.js";
      const entrypointUrl = resolveUrlWithSegments(entryPointBaseUrl, mainPath);
      return this._loadJSEntrypoint(entrypointUrl, onEntrypointLoaded, nonce);
    }
  }

  /**
   * Resolves the promise created by loadEntrypoint, and calls the `onEntrypointLoaded`
   * function supplied by the user (if needed).
   *
   * Called by Flutter through `_flutter.loader.didCreateEngineInitializer` method,
   * which is bound to the correct instance of the FlutterEntrypointLoader by
   * the FlutterLoader object.
   *
   * @param {Function} engineInitializer @see https://github.com/flutter/engine/blob/main/lib/web_ui/lib/src/engine/js_interop/js_loader.dart#L42
   */
  didCreateEngineInitializer(engineInitializer) {
    if (typeof this._didCreateEngineInitializerResolve === "function") {
      this._didCreateEngineInitializerResolve(engineInitializer);
      // Remove the resolver after the first time, so Flutter Web can hot restart.
      this._didCreateEngineInitializerResolve = null;
      // Make the engine revert to "auto" initialization on hot restart.
      delete _flutter.loader.didCreateEngineInitializer;
    }
    if (typeof this._onEntrypointLoaded === "function") {
      this._onEntrypointLoaded(engineInitializer);
    }
  }
  /**
   * Injects a script tag into the DOM, and configures this loader to be able to
   * handle the "entrypoint loaded" notifications received from Flutter web.
   *
   * @param {string} entrypointUrl the URL of the script that will initialize
   *                 Flutter.
   * @param {Function} onEntrypointLoaded a callback that will be called when
   *                   Flutter web notifies this object that the entrypoint is
   *                   loaded.
   * @returns {Promise | undefined} a Promise that resolves when the entrypoint
   *                                is loaded, or undefined if `onEntrypointLoaded`
   *                                is a function.
   */
  _loadJSEntrypoint(entrypointUrl, onEntrypointLoaded, nonce) {
    const useCallback = typeof onEntrypointLoaded === "function";
    if (!this._scriptLoaded) {
      this._scriptLoaded = true;
      const scriptTag = this._createScriptTag(entrypointUrl, nonce);
      if (useCallback) {
        // Just inject the script tag, and return nothing; Flutter will call
        // `didCreateEngineInitializer` when it's done.
        console.debug("Injecting <script> tag. Using callback.");
        this._onEntrypointLoaded = onEntrypointLoaded;
        document.head.append(scriptTag);
      } else {
        // Inject the script tag and return a promise that will get resolved
        // with the EngineInitializer object from Flutter when it calls
        // `didCreateEngineInitializer` later.
        return new Promise((resolve, reject) => {
          console.debug(
            "Injecting <script> tag. Using Promises. Use the callback approach instead!"
          );
          this._didCreateEngineInitializerResolve = resolve;
          scriptTag.addEventListener("error", reject);
          document.head.append(scriptTag);
        });
      }
    }
  }

  /**
   *
   * @param {import("./types").WasmApplicationBuild} build
   * @param {*} deps
   * @param {string} entryPointBaseUrl
   * @param {import("./types").OnEntrypointLoadedCallback} onEntrypointLoaded
   */
  async _loadWasmEntrypoint(build, deps, entrypointBaseUrl, onEntrypointLoaded) {
    if (!this._scriptLoaded) {
      this._scriptLoaded = true;

      this._onEntrypointLoaded = onEntrypointLoaded;
      const { mainWasmPath, jsSupportRuntimePath } = build;
      const moduleUri = resolveUrlWithSegments(entrypointBaseUrl, mainWasmPath);
      let jsSupportRuntimeUri = resolveUrlWithSegments(entrypointBaseUrl, jsSupportRuntimePath);
      if (this._ttPolicy != null) {
        jsSupportRuntimeUri = this._ttPolicy.createScriptURL(jsSupportRuntimeUri);
      }
      const jsSupportRuntime = await import(jsSupportRuntimeUri);

      const compiledDartAppPromise = jsSupportRuntime.compileStreaming(fetch(moduleUri));

      let importsPromise;
      if (build.renderer === "skwasm") {
        importsPromise = (async () => {
          const skwasmInstance = await deps.skwasm;
          window._flutter_skwasmInstance = skwasmInstance;
          return {
            skwasm: skwasmInstance.wasmExports,
            skwasmWrapper: skwasmInstance,
            ffi: {
              memory: skwasmInstance.wasmMemory,
            },
          };
        })();
      } else {
        importsPromise = Promise.resolve({});
      }
      const compiledDartApp = await compiledDartAppPromise;
      const dartApp = await compiledDartApp.instantiate(await importsPromise);
      await dartApp.invokeMain();
    }
  }

  /**
   * Creates a script tag for the given URL.
   * @param {string} url
   * @returns {HTMLScriptElement}
   */
  _createScriptTag(url, nonce) {
    const scriptTag = document.createElement("script");
    scriptTag.type = "application/javascript";
    if (nonce) {
      scriptTag.nonce = nonce;
    }
    // Apply TrustedTypes validation, if available.
    let trustedUrl = url;
    if (this._ttPolicy != null) {
      trustedUrl = this._ttPolicy.createScriptURL(url);
    }
    scriptTag.src = trustedUrl;
    return scriptTag;
  }
}
