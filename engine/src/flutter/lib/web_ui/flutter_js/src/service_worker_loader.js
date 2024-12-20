// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { resolveUrlWithSegments } from "./utils.js";

/**
 * Wraps `promise` in a timeout of the given `duration` in ms.
 *
 * Resolves/rejects with whatever the original `promises` does, or rejects
 * if `promise` takes longer to complete than `duration`. In that case,
 * `debugName` is used to compose a legible error message.
 *
 * If `duration` is < 0, the original `promise` is returned unchanged.
 * @param {Promise} promise
 * @param {number} duration
 * @param {string} debugName
 * @returns {Promise} a wrapped promise.
 */
async function timeout(promise, duration, debugName) {
  if (duration < 0) {
    return promise;
  }
  let timeoutId;
  const _clock = new Promise((_, reject) => {
    timeoutId = setTimeout(() => {
      reject(
        new Error(
          `${debugName} took more than ${duration}ms to resolve. Moving on.`,
          {
            cause: timeout,
          }
        )
      );
    }, duration);
  });
  return Promise.race([promise, _clock]).finally(() => {
    clearTimeout(timeoutId);
  });
}

/**
 * Handles loading/reloading Flutter's service worker, if configured.
 *
 * @see: https://developers.google.com/web/fundamentals/primers/service-workers
 */
export class FlutterServiceWorkerLoader {
  /**
   * Injects a TrustedTypesPolicy (or undefined if the feature is not supported).
   * @param {TrustedTypesPolicy | undefined} policy
   */
  setTrustedTypesPolicy(policy) {
    this._ttPolicy = policy;
  }
  /**
   * Returns a Promise that resolves when the latest Flutter service worker,
   * configured by `settings` has been loaded and activated.
   *
   * Otherwise, the promise is rejected with an error message.
   * @param {import("./types").ServiceWorkerSettings} settings Service worker settings
   * @returns {Promise} that resolves when the latest serviceWorker is ready.
   */
  loadServiceWorker(settings) {
    if (!settings) {
      // In the future, settings = null -> uninstall service worker?
      console.debug("Null serviceWorker configuration. Skipping.");
      return Promise.resolve();
    }
    if (!("serviceWorker" in navigator)) {
      let errorMessage = "Service Worker API unavailable.";
      if (!window.isSecureContext) {
        errorMessage += "\nThe current context is NOT secure."
        errorMessage += "\nRead more: https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts";
      }
      return Promise.reject(
        new Error(errorMessage)
      );
    }
    const {
      serviceWorkerVersion,
      serviceWorkerUrl = resolveUrlWithSegments(`flutter_service_worker.js?v=${serviceWorkerVersion}`),
      timeoutMillis = 4000,
    } = settings;
    // Apply the TrustedTypes policy, if present.
    let url = serviceWorkerUrl;
    if (this._ttPolicy != null) {
      url = this._ttPolicy.createScriptURL(url);
    }
    const serviceWorkerActivation = navigator.serviceWorker
      .register(url)
      .then((serviceWorkerRegistration) => this._getNewServiceWorker(serviceWorkerRegistration, serviceWorkerVersion))
      .then(this._waitForServiceWorkerActivation);
    // Timeout race promise
    return timeout(
      serviceWorkerActivation,
      timeoutMillis,
      "prepareServiceWorker"
    );
  }
  /**
   * Returns the latest service worker for the given `serviceWorkerRegistration`.
   *
   * This might return the current service worker, if there's no new service worker
   * awaiting to be installed/updated.
   *
   * @param {ServiceWorkerRegistration} serviceWorkerRegistration
   * @param {string} serviceWorkerVersion
   * @returns {Promise<ServiceWorker>}
   */
  async _getNewServiceWorker(serviceWorkerRegistration, serviceWorkerVersion) {
    if (!serviceWorkerRegistration.active && (serviceWorkerRegistration.installing || serviceWorkerRegistration.waiting)) {
      // No active web worker and we have installed or are installing
      // one for the first time. Simply wait for it to activate.
      console.debug("Installing/Activating first service worker.");
      return serviceWorkerRegistration.installing || serviceWorkerRegistration.waiting;
    } else if (!serviceWorkerRegistration.active.scriptURL.endsWith(serviceWorkerVersion)) {
      // When the app updates the serviceWorkerVersion changes, so we
      // need to ask the service worker to update.
      const newRegistration = await serviceWorkerRegistration.update();
      console.debug("Updating service worker.");
      return newRegistration.installing || newRegistration.waiting || newRegistration.active;
    } else {
      console.debug("Loading from existing service worker.");
      return serviceWorkerRegistration.active;
    }
  }
  /**
   * Returns a Promise that resolves when the `serviceWorker` changes its
   * state to "activated".
   *
   * @param {ServiceWorker} serviceWorker
   * @returns {Promise<void>}
   */
  async _waitForServiceWorkerActivation(serviceWorker) {
    if (!serviceWorker || serviceWorker.state === "activated") {
      if (!serviceWorker) {
        throw new Error("Cannot activate a null service worker!");
      } else {
        console.debug("Service worker already active.");
        return;
      }
    }
    return new Promise((resolve, _) => {
      serviceWorker.addEventListener("statechange", () => {
        if (serviceWorker.state === "activated") {
          console.debug("Activated new service worker.");
          resolve();
        }
      });
    });
  }
}
