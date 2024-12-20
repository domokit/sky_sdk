// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file adds JavaScript APIs that are accessible to the C++ layer.
// See: https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html#implement-a-c-api-in-javascript

mergeInto(LibraryManager.library, {
  $skwasm_threading_setup__postset: 'skwasm_threading_setup();',
  $skwasm_threading_setup: function() {
    // This value represents the difference between the time origin of the main
    // thread and whichever web worker this code is running on. This is so that
    // when we report frame timings, that they are in the same time domain
    // regardless of whether they are captured on the main thread or the web
    // worker.
    let timeOriginDelta = 0;
    skwasm_registerMessageListener = function(threadId, listener) {
      const eventListener = function({data}) {
        const skwasmMessage = data.skwasmMessage;
        if (!skwasmMessage) {
          return;
        }
        if (skwasmMessage == 'syncTimeOrigin') {
          timeOriginDelta = performance.timeOrigin - data.timeOrigin;
          return;
        }
        listener(data);
      };
      if (!threadId) {
        addEventListener("message", eventListener);
      } else {
        PThread.pthreads[threadId].addEventListener("message", eventListener);
        PThread.pthreads[threadId].postMessage({
          skwasmMessage: 'syncTimeOrigin',
          timeOrigin: performance.timeOrigin,
        });    
      }
    };
    skwasm_getCurrentTimestamp = function() {
      return performance.now() + timeOriginDelta;
    };
    skwasm_postMessage = function(message, transfers, threadId) {
      if (threadId) {
        PThread.pthreads[threadId].postMessage(message, transfers);
      } else {
        postMessage(message, transfers);
      }
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
