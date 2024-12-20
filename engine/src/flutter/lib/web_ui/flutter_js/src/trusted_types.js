// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/**
 * Handles the creation of a TrustedTypes `policy` that validates URLs based
 * on an (optional) incoming array of RegExes.
 */
export class FlutterTrustedTypesPolicy {
  /**
   * Constructs the policy.
   * @param {[RegExp]} validPatterns the patterns to test URLs
   * @param {String} policyName the policy name (optional)
   */
  constructor(validPatterns, policyName = "flutter-js") {
    const patterns = validPatterns || [
      /\.js$/,
      /\.mjs$/,
    ];
    if (window.trustedTypes) {
      this.policy = trustedTypes.createPolicy(policyName, {
        createScriptURL: function (url) {
          // Return blob urls without manipulating them
          if (url.startsWith('blob:')) {
            return url;
          }
          // Validate other urls
          const parsed = new URL(url, window.location);
          const file = parsed.pathname.split("/").pop();
          const matches = patterns.some((pattern) => pattern.test(file));
          if (matches) {
            return parsed.toString();
          }
          console.error(
            "URL rejected by TrustedTypes policy",
            policyName, ":", url, "(download prevented)");
        }
      });
    }
  }
}
