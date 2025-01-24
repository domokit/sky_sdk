// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:web/web.dart' as web;

/// Defines a new property on an Object.
@JS('Object.defineProperty')
external void objectDefineProperty(JSAny o, String symbol, JSAny desc);

void createGetter(JSAny mock, String key, JSAny? Function() get) {
  objectDefineProperty(mock, key, <String, JSFunction>{'get': (() => get()).toJS}.jsify()!);
}

@JS()
@staticInterop
@anonymous
class DomXMLHttpRequestMock {
  external factory DomXMLHttpRequestMock({
    JSFunction? open,
    JSString responseType,
    JSNumber timeout,
    JSBoolean withCredentials,
    JSFunction? send,
    JSFunction? setRequestHeader,
    JSFunction addEventListener,
  });
}

typedef _DartDomEventListener = JSVoid Function(web.Event event);

class TestHttpRequest {
  TestHttpRequest() {
    _mock = DomXMLHttpRequestMock(
      open: open.toJS,
      send: send.toJS,
      setRequestHeader: setRequestHeader.toJS,
      addEventListener: addEventListener.toJS,
    );
    final JSAny mock = _mock as JSAny;
    createGetter(mock, 'headers', () => headers.jsify());
    createGetter(mock, 'responseHeaders', () => responseHeaders.jsify());
    createGetter(mock, 'status', () => status.toJS);
    createGetter(mock, 'response', () => response.jsify());
  }

  late DomXMLHttpRequestMock _mock;
  MockEvent? mockEvent;
  Map<String, String> headers = <String, String>{};
  int status = -1;
  Object? response;

  Map<String, String> get responseHeaders => headers;
  JSVoid open(String method, String url, bool async) {}
  JSVoid send() {}
  JSVoid setRequestHeader(String name, String value) {
    headers[name] = value;
  }

  JSVoid addEventListener(String type, web.EventListener listener) {
    if (type == mockEvent?.type) {
      final _DartDomEventListener dartListener =
          (listener as JSExportedDartFunction).toDart as _DartDomEventListener;
      dartListener(mockEvent!.event);
    }
  }

  web.XMLHttpRequest getMock() => _mock as web.XMLHttpRequest;
}

class MockEvent {
  MockEvent(this.type, this.event);

  final String type;
  final web.Event event;
}

@JS()
@staticInterop
@anonymous
class ImgElementMock {
  external factory ImgElementMock({JSFunction? decode});
}

class TestImgElement {
  TestImgElement() {
    _mock = ImgElementMock(decode: decode.toJS);
    final JSAny mock = _mock as JSAny;
    objectDefineProperty(
      mock,
      'src',
      <String, JSFunction>{
        'get': (() => src).toJS,
        'set':
            ((JSString newValue) {
              src = newValue.toDart;
            }).toJS,
      }.jsify()!,
    );
    objectDefineProperty(
      mock,
      'naturalWidth',
      <String, JSFunction>{
        'get': (() => naturalWidth).toJS,
        'set':
            ((JSNumber newValue) {
              naturalWidth = newValue.toDartInt;
            }).toJS,
      }.jsify()!,
    );
    objectDefineProperty(
      mock,
      'naturalHeight',
      <String, JSFunction>{
        'get': (() => naturalHeight).toJS,
        'set':
            ((JSNumber newValue) {
              naturalHeight = newValue.toDartInt;
            }).toJS,
      }.jsify()!,
    );
  }

  late ImgElementMock _mock;

  String src = '';
  int naturalWidth = -1;
  int naturalHeight = -1;

  // Either `decode` or `decodeSuccess/Failure` may be called first.
  // The following fields allow properly handling either case.
  bool _callbacksAssigned = false;
  late final JSFunction _resolveFunc;
  late final JSFunction _rejectFunc;

  bool _resultAssigned = false;
  late final bool _resultSuccessful;

  JSPromise<JSAny?> decode() {
    if (_resultAssigned) {
      return switch (_resultSuccessful) {
        true => Future<JSAny?>.value().toJS,
        false => Future<JSAny?>.error(Error()).toJS,
      };
    }
    _callbacksAssigned = true;
    return JSPromise<JSAny?>(
      (JSFunction resolveFunc, JSFunction rejectFunc) {
        _resolveFunc = resolveFunc;
        _rejectFunc = rejectFunc;
      }.toJS,
    );
  }

  void decodeSuccess() {
    if (_callbacksAssigned) {
      _resolveFunc.callAsFunction();
    } else {
      _resultAssigned = true;
      _resultSuccessful = true;
    }
  }

  void decodeFailure() {
    if (_callbacksAssigned) {
      _rejectFunc.callAsFunction();
    } else {
      _resultAssigned = true;
      _resultSuccessful = false;
    }
  }

  web.HTMLImageElement getMock() => _mock as web.HTMLImageElement;
}
