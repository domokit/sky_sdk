// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@JS()
library js_promise;

import 'dart:js_interop';
import 'dart:js_interop_unsafe';

/// This is the same as package:js_interop's FutureToPromise (.toJS), but with
/// a more descriptive error message.
extension CustomFutureOfJSAnyToJSPromise<T extends JSAny?> on Future<T> {
  /// A [JSPromise] that either resolves with the result of the completed
  /// [Future] or rejects with an object that contains its error.
  JSPromise<T> get toPromise {
    // TODO(ditman): Move to js_interop's .toJS, https://github.com/dart-lang/sdk/issues/56898
    return JSPromise<T>(
      (JSFunction resolve, JSFunction reject) {
        then(
          (JSAny? value) {
            resolve.callAsFunction(resolve, value);
          },
          onError: (Object error, StackTrace stackTrace) {
            final errorConstructor = globalContext['Error']! as JSFunction;
            var userError = '$error\n';
            // Only append the stack trace string if it looks like a DDC one...
            final stackTraceString = stackTrace.toString();
            if (!stackTraceString.startsWith('\n')) {
              userError += '\nDart stack trace:\n$stackTraceString';
            }
            final wrapper = errorConstructor.callAsConstructor<JSObject>(userError.toJS);
            reject.callAsFunction(reject, wrapper);
          },
        );
      }.toJS,
    );
  }
}
