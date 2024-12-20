// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file adds JavaScript APIs that are accessible to the C++ layer.
// See: https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html#implement-a-c-api-in-javascript

mergeInto(LibraryManager.library, {
  $skwasm_threading_setup__postset: 'skwasm_threading_setup();',
  $skwasm_threading_setup: function() {
    let messageListener;
    skwasm_registerMessageListener = function(threadId, listener) {
      messageListener = listener;
    };
    skwasm_getCurrentTimestamp = function() {
      return performance.now();
    };
    skwasm_postMessage = function(message, transfers, threadId) {
      queueMicrotask(() => {
        messageListener(message);
      })
    };
  },
  $skwasm_threading_setup__deps: ['$skwasm_registerMessageListener', '$skwasm_getCurrentTimestamp', '$skwasm_postMessage'],
  $skwasm_registerMessageListener: function() {},
  $skwasm_registerMessageListener__deps: ['$skwasm_threading_setup'],
  $skwasm_getCurrentTimestamp: function () {},
  $skwasm_getCurrentTimestamp__deps: ['$skwasm_threading_setup'],
  $skwasm_postMessage: function () {},
  $skwasm_postMessage__deps: ['$skwasm_threading_setup'],
});
