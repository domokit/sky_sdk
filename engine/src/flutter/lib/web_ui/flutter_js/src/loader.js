// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { browserEnvironment } from './browser_environment.js';
import { FlutterEntrypointLoader } from './entrypoint_loader.js';
import { FlutterServiceWorkerLoader } from './service_worker_loader.js';
import { FlutterTrustedTypesPolicy } from './trusted_types.js';
import { loadCanvasKit } from './canvaskit_loader.js';
import { loadSkwasm } from './skwasm_loader.js';
import { getCanvaskitBaseUrl } from './utils.js';

/**
 * The public interface of _flutter.loader. Exposes two methods:
 * * loadEntrypoint (which coordinates the default Flutter web loading procedure)
 * * didCreateEngineInitializer (which is called by Flutter to notify that its
 *                              Engine is ready to be initialized)
 */
export class FlutterLoader {
  /**
   * @deprecated Use `load` instead.
   * Initializes the Flutter web app.
   * @param {*} options
   * @returns {Promise?} a (Deprecated) Promise that will eventually resolve
   *                     with an EngineInitializer, or will be rejected with
   *                     any error caused by the loader. Or Null, if the user
   *                     supplies an `onEntrypointLoaded` Function as an option.
   */
  async loadEntrypoint(options) {
    const { serviceWorker, ...entrypoint } = options || {};
    // A Trusted Types policy that is going to be used by the loader.
    const flutterTT = new FlutterTrustedTypesPolicy();
    // The FlutterServiceWorkerLoader instance could be injected as a dependency
    // (and dynamically imported from a module if not present).
    const serviceWorkerLoader = new FlutterServiceWorkerLoader();
    serviceWorkerLoader.setTrustedTypesPolicy(flutterTT.policy);
    await serviceWorkerLoader.loadServiceWorker(serviceWorker).catch(e => {
      // Regardless of what happens with the injection of the SW, the show must go on
      console.warn("Exception while loading service worker:", e);
    });
    // The FlutterEntrypointLoader instance could be injected as a dependency
    // (and dynamically imported from a module if not present).
    const entrypointLoader = new FlutterEntrypointLoader();
    entrypointLoader.setTrustedTypesPolicy(flutterTT.policy);
    // Install the `didCreateEngineInitializer` listener where Flutter web expects it to be.
    this.didCreateEngineInitializer =
      entrypointLoader.didCreateEngineInitializer.bind(entrypointLoader);
    return entrypointLoader.loadEntrypoint(entrypoint);
  }

  /**
   * Loads and initializes a flutter application.
   * @param {Object} options
   * @param {import("/.types".ServiceWorkerSettings?)} options.serviceWorkerSettings
   *   Settings for the service worker to be loaded. Can pass `undefined` or
   *   `null` to not launch a service worker at all.
   * @param {import("/.types".OnEntryPointLoadedCallback)} options.onEntrypointLoaded
   *   An optional callback to invoke 
   * @param {string} options.nonce
   *   A nonce to be applied to the main JS script when loading it, which may
   *   be required by the sites Content-Security-Policy.
   *   For more details, see {@link https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/Content-Security-Policy/script-src here}.
   * @param {import("./types".FlutterConfiguration)} arg.config
   */
  async load({
    serviceWorkerSettings,
    onEntrypointLoaded,
    nonce,
    config,
  } = {}) {
    config ??= {};

    /** @type {import("./types").BuildConfig} */
    const buildConfig = _flutter.buildConfig;
    if (!buildConfig) {
      throw "FlutterLoader.load requires _flutter.buildConfig to be set";
    }

    const rendererIsCompatible = (renderer) => {
      switch (renderer) {
        case "skwasm":
          return browserEnvironment.hasChromiumBreakIterators
            && browserEnvironment.hasImageCodecs
            && browserEnvironment.supportsWasmGC;
        default:
          return true;
      }
    }

    /**
     * @param {import("./types").ApplicationBuild} build
     * @param {import("./types").WebRenderer} renderer
     **/
    const buildContainsRenderer = (build, renderer) => {
      return build.renderer == renderer;
    }

    const buildIsCompatible = (build) => {
      if (build.compileTarget === "dart2wasm" && !browserEnvironment.supportsWasmGC) {
        return false;
      }
      if (config.renderer && !buildContainsRenderer(build, config.renderer)) {
        return false;
      }
      return rendererIsCompatible(build.renderer);
    };
    const build = buildConfig.builds.find(buildIsCompatible);
    if (!build) {
      throw "FlutterLoader could not find a build compatible with configuration and environment.";
    }

    const deps = {};
    deps.flutterTT = new FlutterTrustedTypesPolicy();
    if (serviceWorkerSettings) {
      deps.serviceWorkerLoader = new FlutterServiceWorkerLoader();
      deps.serviceWorkerLoader.setTrustedTypesPolicy(deps.flutterTT.policy);
      await deps.serviceWorkerLoader.loadServiceWorker(serviceWorkerSettings).catch(e => {
        // Regardless of what happens with the injection of the SW, the show must go on
        console.warn("Exception while loading service worker:", e);
      });
    }

    const canvasKitBaseUrl = getCanvaskitBaseUrl(config, buildConfig);
    if (build.renderer === "canvaskit") {
      deps.canvasKit = loadCanvasKit(deps, config, browserEnvironment, canvasKitBaseUrl);
    } else if (build.renderer === "skwasm") {
      deps.skwasm = loadSkwasm(deps, config, browserEnvironment, canvasKitBaseUrl);
    }

    // The FlutterEntrypointLoader instance could be injected as a dependency
    // (and dynamically imported from a module if not present).
    const entrypointLoader = new FlutterEntrypointLoader();
    entrypointLoader.setTrustedTypesPolicy(deps.flutterTT.policy);
    // Install the `didCreateEngineInitializer` listener where Flutter web expects it to be.
    this.didCreateEngineInitializer =
      entrypointLoader.didCreateEngineInitializer.bind(entrypointLoader);
    return entrypointLoader.load(build, deps, config, nonce, onEntrypointLoaded);
  }
}
